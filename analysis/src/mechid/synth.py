"""Synthetic session generator.

Simulates the same phase sequence the MechID Java routine runs (quasistatic ramps,
guarded dynamic steps, coast-down) against a known ground-truth model, and writes a
session folder (meta.json + data.csv) identical in shape to what the robot produces.
Used to validate the analysis pipeline end to end: recovered constants must match the
truth.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MOTOR_SPECS = {
    "KRAKEN_X60": {"nominalVoltage": 12.0, "stallTorqueNm": 7.09, "stallCurrentAmps": 366.0,
                   "freeCurrentAmps": 2.0, "freeSpeedRpm": 6000.0},
    "KRAKEN_X60_FOC": {"nominalVoltage": 12.0, "stallTorqueNm": 9.37, "stallCurrentAmps": 483.0,
                       "freeCurrentAmps": 2.0, "freeSpeedRpm": 5800.0},
    "KRAKEN_X44": {"nominalVoltage": 12.0, "stallTorqueNm": 4.11, "stallCurrentAmps": 279.0,
                   "freeCurrentAmps": 2.0, "freeSpeedRpm": 7758.0},
    "KRAKEN_X44_FOC": {"nominalVoltage": 12.0, "stallTorqueNm": 5.01, "stallCurrentAmps": 329.0,
                       "freeCurrentAmps": 2.0, "freeSpeedRpm": 7368.0},
    "NEO_VORTEX": {"nominalVoltage": 12.0, "stallTorqueNm": 3.60, "stallCurrentAmps": 211.0,
                   "freeCurrentAmps": 3.6, "freeSpeedRpm": 6784.0},
}


@dataclass
class Truth:
    """Ground-truth plant. kV must equal ke_mech + viscous for physical consistency."""

    kS: float
    kV: float
    kA: float
    kG: float = 0.0
    ratio: float = 1.0  # rotor rotations per mechanism unit
    motor: str = "KRAKEN_X60"

    @property
    def ke_mech(self) -> float:
        spec = MOTOR_SPECS[self.motor]
        resistance = spec["nominalVoltage"] / spec["stallCurrentAmps"]
        free_rps = spec["freeSpeedRpm"] / 60.0
        ke = (spec["nominalVoltage"] - spec["freeCurrentAmps"] * resistance) / free_rps
        return ke * self.ratio

    @property
    def viscous(self) -> float:
        return max(self.kV - self.ke_mech, 0.0)


@dataclass
class RoutineSpec:
    mech_type: str
    pid_domain: str
    gravity_model: str
    ramp_rate: float
    ramp_max_v: float
    steps: list[float]
    step_hold: float
    min_pos: float = float("nan")
    max_pos: float = float("nan")
    margin_frac: float = 0.08
    coastdown: bool = False
    settle_time: float = 1.0
    settle_tol: float = 0.05
    horizontal: float = 0.0
    position_units: str = "rot"
    extra: dict = field(default_factory=dict)
    n_motors: int = 1
    # For SWERVE_DRIVE the guard position is meters while motor units are wheel rot.
    guard_scale: float = 1.0
    # REV controllers have no acceleration signal; the recorder writes NaN.
    acc_available: bool = True


PRESETS: dict[str, tuple[Truth, RoutineSpec]] = {
    "TURRET": (
        Truth(kS=0.18, kV=6.20, kA=0.14, ratio=45.0),
        RoutineSpec("TURRET", "POSITION", "NONE", 0.25, 4.0, [1.5, 2.5, 3.5], 1.5,
                    min_pos=-0.51, max_pos=0.51),
    ),
    "ELEVATOR": (
        Truth(kS=0.12, kV=1.35, kA=0.06, kG=0.38, ratio=10.0),
        RoutineSpec("ELEVATOR", "POSITION", "CONSTANT", 0.35, 5.0, [2.0, 3.0, 4.0], 1.0,
                    min_pos=0.0, max_pos=5.0),
    ),
    "ARM": (
        Truth(kS=0.15, kV=8.5, kA=0.35, kG=0.45, ratio=60.0),
        RoutineSpec("ARM", "POSITION", "COSINE", 0.25, 4.0, [1.5, 2.5, 3.5], 1.0,
                    min_pos=-0.08, max_pos=0.30),
    ),
    "FLYWHEEL": (
        Truth(kS=0.22, kV=0.128, kA=0.007, ratio=1.0),
        RoutineSpec("FLYWHEEL", "VELOCITY", "NONE", 0.5, 10.0, [2.0, 4.0, 6.0, 8.0, 10.0], 3.0,
                    coastdown=True, settle_tol=0.5, n_motors=2),
    ),
    "SWERVE_DRIVE": (
        Truth(kS=0.16, kV=0.75, kA=0.055, ratio=6.12),
        RoutineSpec("SWERVE_DRIVE", "VELOCITY", "NONE", 1.0, 8.0, [3.0, 5.0, 7.0], 1.5,
                    min_pos=0.0, max_pos=4.0, position_units="wheel_rot",
                    extra={"wheelRadiusMeters": 0.0508, "travelMeters": 4.0, "moduleCount": 4.0},
                    n_motors=4, guard_scale=2.0 * np.pi * 0.0508),
    ),
    # REV SparkFlex/Vortex roller: no acceleration signal, no coast-down test.
    "VORTEX_ROLLER": (
        Truth(kS=0.20, kV=0.112, kA=0.005, ratio=1.0, motor="NEO_VORTEX"),
        RoutineSpec("FLYWHEEL", "VELOCITY", "NONE", 0.5, 10.0, [2.0, 4.0, 6.0, 8.0, 10.0], 3.0,
                    coastdown=False, settle_tol=0.5, acc_available=False),
    ),
    # Kraken X60 running field-oriented control (FOC-equivalent motor constants).
    "FLYWHEEL_FOC": (
        Truth(kS=0.20, kV=0.132, kA=0.006, ratio=1.0, motor="KRAKEN_X60_FOC"),
        RoutineSpec("FLYWHEEL", "VELOCITY", "NONE", 0.5, 10.0, [2.0, 4.0, 6.0, 8.0, 10.0], 3.0,
                    coastdown=True, settle_tol=0.5),
    ),
}


def generate(mech_type: str, out_dir: str | Path, seed: int = 42, rate_hz: float = 200.0) -> dict:
    if mech_type not in PRESETS:
        raise ValueError(f"No synthetic preset for {mech_type}; options: {list(PRESETS)}")
    truth, spec = PRESETS[mech_type]
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate_hz

    # Per-motor plants: identical except a small spread for multi-module mechanisms so
    # the module-spread reporting has something to see. Followers of a coupled
    # mechanism (flywheel) share the leader's motion exactly.
    independent = mech_type.startswith("SWERVE")
    n_units = spec.n_motors if independent else 1
    units = []
    for i in range(n_units):
        f = 1.0 + (0.03 * (i - (n_units - 1) / 2) if n_units > 1 else 0.0)
        units.append(Truth(truth.kS * f, truth.kV * f, truth.kA * f, truth.kG,
                           truth.ratio, truth.motor))

    phases = _build_phases(spec)
    rows = []
    t = 0.0
    state = [(0.0 if np.isnan(spec.min_pos) else spec.min_pos, 0.0)] * n_units  # (pos, vel)

    for test_id, mode, param in phases:
        elapsed = 0.0
        while True:
            volts_cmd, done_all = _phase_command(spec, mode, param, elapsed, state, truth)
            new_state = []
            sample = {"t": t, "test": test_id, "cmd_v": volts_cmd}
            battery = 12.3 + rng.normal(0, 0.03)
            sample["battery_v"] = battery
            for i, plant in enumerate(units):
                pos, vel = state[i]
                v_app = _applied_volts(mode, volts_cmd, vel, plant)
                acc = _accel(plant, spec, v_app, vel, pos, mode)
                stator = (v_app - plant.ke_mech * vel) / _resistance(plant) if mode != "coast" else 0.0
                supply = abs(stator) * min(abs(v_app) / battery, 1.0)
                prefix = _motor_name(spec, i)
                sample[f"{prefix}_pos"] = pos + rng.normal(0, 1e-4)
                sample[f"{prefix}_vel"] = vel + rng.normal(0, 0.004 * max(1.0, abs(vel)))
                sample[f"{prefix}_acc"] = (acc + rng.normal(0, 0.02 * max(1.0, abs(acc)))
                                           if spec.acc_available else float("nan"))
                sample[f"{prefix}_volts"] = v_app + rng.normal(0, 0.01)
                sample[f"{prefix}_stator_a"] = stator + rng.normal(0, 0.3)
                sample[f"{prefix}_supply_a"] = supply + rng.normal(0, 0.2)
                vel2 = vel + acc * dt
                if mode in ("neutral",) and abs(vel2) < 0.01:
                    vel2 = 0.0
                new_state.append((pos + vel2 * dt, vel2))

            # Coupled followers mirror the leader's motion.
            if not independent and spec.n_motors > 1:
                pos, vel = state[0]
                for j in range(1, spec.n_motors):
                    prefix = _motor_name(spec, j)
                    sample[f"{prefix}_pos"] = sample[f"{_motor_name(spec, 0)}_pos"]
                    sample[f"{prefix}_vel"] = sample[f"{_motor_name(spec, 0)}_vel"]
                    sample[f"{prefix}_acc"] = sample[f"{_motor_name(spec, 0)}_acc"]
                    sample[f"{prefix}_volts"] = sample[f"{_motor_name(spec, 0)}_volts"]
                    sample[f"{prefix}_stator_a"] = sample[f"{_motor_name(spec, 0)}_stator_a"]
                    sample[f"{prefix}_supply_a"] = sample[f"{_motor_name(spec, 0)}_supply_a"]

            rows.append(sample)
            state = new_state
            t += dt
            elapsed += dt
            if done_all or _guard_hit(spec, mode, param, state):
                break
            if elapsed > 40.0:
                break

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    _write_csv(out / "data.csv", rows, spec)
    _write_meta(out / "meta.json", truth, spec)
    truth_dict = {
        "kS": truth.kS, "kV": truth.kV, "kA": truth.kA, "kG": truth.kG,
        "ratio": truth.ratio, "motor": truth.motor,
    }
    (out / "truth.json").write_text(json.dumps(truth_dict, indent=2))
    return truth_dict


# ---------------------------------------------------------------------------

def _motor_name(spec: RoutineSpec, i: int) -> str:
    if spec.mech_type == "SWERVE_DRIVE":
        return f"m{i}_drive"
    if spec.mech_type == "SWERVE_STEER":
        return f"m{i}_steer"
    return "leader" if i == 0 else f"follower{i}"


def _resistance(plant: Truth) -> float:
    spec = MOTOR_SPECS[plant.motor]
    return spec["nominalVoltage"] / spec["stallCurrentAmps"]


def _build_phases(spec: RoutineSpec) -> list[tuple[str, str, dict]]:
    limited = not np.isnan(spec.min_pos)
    phases = [("quasistatic_forward", "ramp", {"dir": 1}), ("settle", "neutral", {})]
    if limited:
        phases += [("quasistatic_reverse", "ramp", {"dir": -1}), ("settle", "neutral", {})]
    for i, volts in enumerate(spec.steps):
        base = f"dynamic_s{i}_{volts:.1f}V"
        if limited:
            phases += [(base + "_a", "step", {"volts": volts, "auto": True}),
                       ("settle", "neutral", {}),
                       (base + "_b", "step", {"volts": volts, "auto": True}),
                       ("settle", "neutral", {})]
        else:
            phases += [(base, "step", {"volts": volts, "auto": False}),
                       ("settle", "neutral", {})]
    if spec.coastdown:
        phases += [("coastdown_spinup", "step", {"volts": 8.0, "auto": False, "hold": 2.0}),
                   ("coastdown", "coast", {}), ("settle", "neutral", {})]
    return phases


def _phase_command(spec, mode, param, elapsed, state, truth) -> tuple[float, bool]:
    pos, vel = state[0]
    if mode == "ramp":
        volts = param["dir"] * min(elapsed * spec.ramp_rate, spec.ramp_max_v)
        return volts, abs(volts) >= spec.ramp_max_v
    if mode == "step":
        if "resolved_dir" not in param:
            if param.get("auto") and not np.isnan(spec.min_pos):
                center = (spec.min_pos + spec.max_pos) / 2.0
                param["resolved_dir"] = 1 if pos * spec.guard_scale < center else -1
            else:
                param["resolved_dir"] = 1
        hold = param.get("hold", spec.step_hold)
        return param["resolved_dir"] * param["volts"], elapsed >= hold
    if mode == "neutral":
        still = abs(vel) < spec.settle_tol
        return 0.0, (still and elapsed >= spec.settle_time) or elapsed >= 8.0
    if mode == "coast":
        return 0.0, abs(vel) < spec.settle_tol or elapsed >= 30.0
    return 0.0, True


def _guard_hit(spec, mode, param, state) -> bool:
    if np.isnan(spec.min_pos) or mode not in ("ramp", "step"):
        return False
    pos = state[0][0] * spec.guard_scale
    margin = (spec.max_pos - spec.min_pos) * spec.margin_frac
    d = param.get("resolved_dir", param.get("dir", 1))
    if d > 0:
        return pos >= spec.max_pos - margin
    return pos <= spec.min_pos + margin


def _applied_volts(mode: str, volts_cmd: float, vel: float, plant: Truth) -> float:
    if mode == "coast":
        return 0.0
    if mode == "neutral":
        # Brake: windings shorted; model as applying a voltage equal to back-EMF
        # opposition (approximation — analysis ignores settle samples anyway).
        return 0.0
    return volts_cmd


def _accel(plant: Truth, spec: RoutineSpec, v_app: float, vel: float, pos: float, mode: str) -> float:
    g = 0.0
    if spec.gravity_model == "CONSTANT":
        g = 1.0
    elif spec.gravity_model == "COSINE":
        g = float(np.cos(2 * np.pi * (pos - spec.horizontal)))

    if mode == "coast":
        # No electrical torque: friction only.
        if vel == 0.0:
            return 0.0
        return (-plant.kS * np.sign(vel) - plant.kG * g - plant.viscous * vel) / plant.kA
    if mode == "neutral":
        # Dynamic braking: EMF-driven current through shorted windings plus friction.
        if vel == 0.0 and abs(plant.kG * g) <= plant.kS:
            return 0.0
        return (-plant.kS * np.sign(vel) - plant.kG * g - plant.kV * 1.5 * vel) / plant.kA

    # Stiction: refuse to start moving until the net static force exceeds kS.
    if vel == 0.0 and abs(v_app - plant.kG * g) <= plant.kS:
        return 0.0
    s = np.sign(vel) if vel != 0.0 else np.sign(v_app - plant.kG * g)
    return (v_app - plant.kS * s - plant.kG * g - plant.kV * vel) / plant.kA


def _write_csv(path: Path, rows: list[dict], spec: RoutineSpec) -> None:
    motor_cols = []
    for i in range(spec.n_motors):
        name = _motor_name(spec, i)
        motor_cols += [f"{name}_{c}" for c in ("pos", "vel", "acc", "volts", "stator_a", "supply_a")]
    cols = ["t", "test", "cmd_v", "battery_v"] + motor_cols
    with path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")


def _write_meta(path: Path, truth: Truth, spec: RoutineSpec) -> None:
    motors = [_motor_name(spec, i) for i in range(spec.n_motors)]
    meta = {
        "formatVersion": 1,
        "library": "MechID synth",
        "name": f"synth_{spec.mech_type.lower()}",
        "mechanismType": spec.mech_type,
        "pidDomain": spec.pid_domain,
        "gravityModel": spec.gravity_model,
        "positionUnits": spec.position_units,
        "startedAt": "synthetic",
        "extra": spec.extra,
        "motorModel": {"name": truth.motor, **MOTOR_SPECS[truth.motor]},
        "routine": {"sampleRateHz": 200.0, "useFoc": False},
        "envelope": {
            "minPosition": None if np.isnan(spec.min_pos) else spec.min_pos,
            "maxPosition": None if np.isnan(spec.max_pos) else spec.max_pos,
            "maxVelocityAbort": None,
            "statorCurrentAbort": 100.0,
            "armHorizontalPosition": spec.horizontal,
        },
        "gearing": {"rotorToMechanismRatio": truth.ratio, "source": "user",
                    "remoteSensorFeedback": False, "ratioTrusted": True},
        "motors": motors,
        "devices": {m: {"deviceId": i, "canBus": "synth"} for i, m in enumerate(motors)},
    }
    path.write_text(json.dumps(meta, indent=2))
