"""Loading and segmenting a MechID recording session (meta.json + data.csv)."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

# Segment kinds shared with the MATLAB engine.
KIND_IGNORE = 0
KIND_QUASISTATIC = 1
KIND_DYNAMIC = 2
KIND_COASTDOWN = 3


def classify_test(test_id: str) -> int:
    if test_id.startswith("quasistatic"):
        return KIND_QUASISTATIC
    if test_id.startswith("dynamic") or test_id == "coastdown_spinup":
        return KIND_DYNAMIC
    if test_id == "coastdown":
        return KIND_COASTDOWN
    return KIND_IGNORE  # settle, idle, done


@dataclass
class MotorData:
    """One motor's recorded time series, all numpy arrays of equal length."""

    name: str
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    volts: np.ndarray
    stator: np.ndarray
    supply: np.ndarray
    seg_kind: np.ndarray  # int, KIND_* per sample
    seg_id: np.ndarray  # int, increments at every test-id change
    tests: list[str] = field(default_factory=list)  # test id per sample

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))


@dataclass
class Session:
    path: Path
    meta: dict
    motors: list[MotorData]
    battery: np.ndarray

    @property
    def name(self) -> str:
        return self.meta.get("name", self.path.name)

    @property
    def mechanism_type(self) -> str:
        return self.meta["mechanismType"]

    @property
    def is_swerve(self) -> bool:
        return self.mechanism_type in ("SWERVE_DRIVE", "SWERVE_STEER")

    def fit_motors(self) -> list[MotorData]:
        """Motors to run identification on: every module for swerve (they see
        independent loads), the leader only otherwise (followers are hard-coupled
        and see the same voltage, so they add no information)."""
        if self.is_swerve:
            return self.motors
        return self.motors[:1]

    @property
    def n_motors(self) -> int:
        """Motor count on the mechanism, for total current budgeting."""
        return len(self.motors)


def load_session(session_dir: str | Path) -> Session:
    path = Path(session_dir)
    meta_file = path / "meta.json"
    data_file = path / "data.csv"
    if not meta_file.exists() or not data_file.exists():
        raise FileNotFoundError(
            f"{path} is not a MechID session (needs meta.json and data.csv)"
        )

    meta = json.loads(meta_file.read_text())
    df = pd.read_csv(data_file)
    if len(df) < 50:
        raise ValueError(f"{data_file} has only {len(df)} samples; not enough to analyze")

    t = df["t"].to_numpy(float)
    t = t - t[0]
    tests = df["test"].astype(str).tolist()
    kind = np.array([classify_test(x) for x in tests], dtype=int)
    seg_id = (df["test"] != df["test"].shift()).cumsum().to_numpy(int)

    motors = []
    for name in meta["motors"]:
        vel = df[f"{name}_vel"].to_numpy(float)
        acc = df[f"{name}_acc"].to_numpy(float)
        # REV controllers record no acceleration signal (all NaN); differentiate the
        # velocity instead, with light smoothing to tame differentiation noise.
        if np.isnan(acc).mean() > 0.9:
            acc = _derive_acceleration(t, vel)
        motors.append(
            MotorData(
                name=name,
                t=t,
                pos=df[f"{name}_pos"].to_numpy(float),
                vel=vel,
                acc=acc,
                volts=df[f"{name}_volts"].to_numpy(float),
                stator=df[f"{name}_stator_a"].to_numpy(float),
                supply=df[f"{name}_supply_a"].to_numpy(float),
                seg_kind=kind,
                seg_id=seg_id,
                tests=tests,
            )
        )

    return Session(path=path, meta=meta, motors=motors, battery=df["battery_v"].to_numpy(float))


def _derive_acceleration(t: np.ndarray, vel: np.ndarray, window: int = 9) -> np.ndarray:
    acc = np.gradient(vel, t)
    kernel = np.ones(window) / window
    return np.convolve(acc, kernel, mode="same")


def motor_physics(meta: dict) -> dict:
    """Electrical constants for the current-limit model, from meta.json."""
    m = meta["motorModel"]
    resistance = m["nominalVoltage"] / m["stallCurrentAmps"]
    free_rps = m["freeSpeedRpm"] / 60.0
    ke = (m["nominalVoltage"] - m["freeCurrentAmps"] * resistance) / free_rps
    return {
        "name": m["name"],
        "nominal_v": m["nominalVoltage"],
        "resistance": resistance,
        "kt": m["stallTorqueNm"] / m["stallCurrentAmps"],
        "ke_v_per_rotor_rps": ke,
        "stall_current": m["stallCurrentAmps"],
    }
