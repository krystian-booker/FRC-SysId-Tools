"""Identification math: feedforward regression, dynamics fit, LQR gains, and the
current-limit sweep simulation.

Model (mechanism units u, e.g. rotations):
    kA * dv/dt = V_applied - kS*sign(v) - kG*g(pos) - kV*v
where g(pos) = 0 (no gravity), 1 (elevator), or cos(2*pi*(pos - horizontal)) (arm).
kV lumps back-EMF and viscous friction; kS is Coulomb friction. All constants are in
volts per mechanism unit, matching Phoenix 6 Slot0 conventions.
"""

from __future__ import annotations

import numpy as np

from .session import (
    KIND_COASTDOWN,
    KIND_DYNAMIC,
    KIND_QUASISTATIC,
    MotorData,
)


def gravity_series(gravity_model: str, pos: np.ndarray, horizontal: float) -> np.ndarray:
    if gravity_model == "CONSTANT":
        return np.ones_like(pos)
    if gravity_model == "COSINE":
        return np.cos(2.0 * np.pi * (pos - horizontal))
    return np.zeros_like(pos)


# ---------------------------------------------------------------------------
# Feedforward (quasistatic)
# ---------------------------------------------------------------------------

def fit_feedforward(motor: MotorData, gravity_model: str, horizontal: float,
                    ka_comp: float = 0.0) -> dict:
    """kS/kV/kG from steady-state (quasistatic ramp) samples via least squares.

    ka_comp: once kA is known (second pass), the voltage spent on the small residual
    acceleration during the ramp is subtracted out. Without this, velocity lag during
    the ramp inflates kS by rampRate*kA/kV — noticeable on low-inertia mechanisms.
    """
    qs = motor.seg_kind == KIND_QUASISTATIC
    warnings: list[str] = []
    if qs.sum() < 50:
        raise ValueError(f"{motor.name}: only {int(qs.sum())} quasistatic samples")

    vel, acc, pos = motor.vel[qs], motor.acc[qs], motor.pos[qs]
    volts = motor.volts[qs] - ka_comp * acc
    vmax = float(np.max(np.abs(vel))) if len(vel) else 0.0
    if vmax <= 0:
        raise ValueError(f"{motor.name}: mechanism never moved during quasistatic test")

    # Velocity floor: proportional to top speed, but never below 8x the measurement
    # noise — samples whose sign is noise-ambiguous corrupt the kS (sign(v)) regressor.
    dv = np.diff(vel)
    sigma = 1.4826 * float(np.median(np.abs(dv - np.median(dv)))) / np.sqrt(2.0)
    vel_floor = max(0.02 * vmax, 8.0 * sigma)
    moving = np.abs(vel) > vel_floor
    # Keep the steadiest 80% of moving samples (acceleration near zero).
    acc_thresh = np.quantile(np.abs(acc[moving]), 0.80) if moving.sum() > 20 else np.inf
    mask = moving & (np.abs(acc) <= acc_thresh)
    if mask.sum() < 30:
        warnings.append(f"{motor.name}: few steady quasistatic samples ({int(mask.sum())})")
        mask = moving

    v, u, p = vel[mask], volts[mask], pos[mask]
    cols = [np.sign(v), v]
    names = ["kS", "kV"]
    if gravity_model in ("CONSTANT", "COSINE"):
        cols.insert(1, gravity_series(gravity_model, p, horizontal))
        names.insert(1, "kG")

    X = np.column_stack(cols)
    coef, _, _, _ = np.linalg.lstsq(X, u, rcond=None)
    fit = dict(zip(names, (float(c) for c in coef)))
    fit.setdefault("kG", 0.0)

    if fit["kS"] < 0:
        warnings.append(f"{motor.name}: regression gave kS={fit['kS']:.3f} < 0; clamped to 0")
        fit["kS"] = 0.0
    if fit["kV"] <= 0:
        raise ValueError(f"{motor.name}: kV={fit['kV']:.4f} <= 0; data unusable")

    residual = u - X @ coef
    ss_tot = float(np.sum((u - u.mean()) ** 2))
    r2 = 1.0 - float(np.sum(residual**2)) / ss_tot if ss_tot > 0 else 0.0

    return {
        **fit,
        "r_squared": r2,
        "n_samples": int(mask.sum()),
        "max_velocity_seen": vmax,
        "warnings": warnings,
        # Scatter for the report: velocity vs voltage with the fit line.
        "scatter_vel": v,
        "scatter_volts": u,
        "scatter_fit": X @ coef,
    }


# ---------------------------------------------------------------------------
# Dynamics (kA)
# ---------------------------------------------------------------------------

def fit_dynamics(motor: MotorData, ff: dict, gravity_model: str, horizontal: float) -> dict:
    """kA from the dynamic step segments.

    Uses the TalonFX's onboard filtered acceleration signal and regresses
        acc = (1/kA) * u_eff + (-kV_dyn/kA) * v
    where u_eff = volts - kS*sign(v) - kG*g(pos). kV_dyn is a cross-check against the
    quasistatic kV; a large mismatch means the linear model is a poor fit.
    """
    dyn = motor.seg_kind == KIND_DYNAMIC
    warnings: list[str] = []
    if dyn.sum() < 50:
        raise ValueError(f"{motor.name}: only {int(dyn.sum())} dynamic samples")

    vel, acc, volts, pos = motor.vel[dyn], motor.acc[dyn], motor.volts[dyn], motor.pos[dyn]
    u_eff = (
        volts
        - ff["kS"] * np.sign(vel)
        - ff["kG"] * gravity_series(gravity_model, pos, horizontal)
    )
    X = np.column_stack([u_eff, vel])
    coef, _, _, _ = np.linalg.lstsq(X, acc, rcond=None)
    c1, c2 = float(coef[0]), float(coef[1])
    if c1 <= 0:
        raise ValueError(f"{motor.name}: dynamics regression gave non-physical kA (1/c1={c1:.4f})")

    ka = 1.0 / c1
    kv_dyn = -c2 / c1
    if ff["kV"] > 0 and abs(kv_dyn - ff["kV"]) > 0.30 * ff["kV"]:
        warnings.append(
            f"{motor.name}: dynamic kV ({kv_dyn:.3f}) differs from quasistatic kV "
            f"({ff['kV']:.3f}) by >30% — model may be nonlinear (backlash, load variation)"
        )

    pred = X @ coef
    ss_tot = float(np.sum((acc - acc.mean()) ** 2))
    r2 = 1.0 - float(np.sum((acc - pred) ** 2)) / ss_tot if ss_tot > 0 else 0.0

    return {"kA": ka, "kV_dynamic": kv_dyn, "r_squared": r2,
            "n_samples": int(dyn.sum()), "warnings": warnings}


def friction_split(motor: MotorData, ff: dict, ka: float) -> dict | None:
    """Optional: coast-down decay separates viscous friction from back-EMF.
    During coast there is no electrical torque, so kA*acc = -(kS_f*sign(v) + b*v)."""
    coast = motor.seg_kind == KIND_COASTDOWN
    if coast.sum() < 30:
        return None
    vel, acc = motor.vel[coast], motor.acc[coast]
    moving = np.abs(vel) > 0.02 * max(np.max(np.abs(vel)), 1e-9)
    if moving.sum() < 20:
        return None
    X = np.column_stack([np.sign(vel[moving]), vel[moving]])
    coef, _, _, _ = np.linalg.lstsq(X, -ka * acc[moving], rcond=None)
    coulomb, viscous = float(coef[0]), float(coef[1])
    return {
        "coulomb_volts": coulomb,
        "viscous_volts_per_ups": viscous,
        "back_emf_volts_per_ups": ff["kV"] - viscous,
    }


# ---------------------------------------------------------------------------
# Gains (LQR, WPILib-SysId-style cost weighting)
# ---------------------------------------------------------------------------

def lqr_gains(pid_domain: str, kv: float, ka: float, *, max_position_error: float,
              max_velocity_error: float, max_effort: float) -> dict:
    from scipy.linalg import solve_continuous_are

    if pid_domain == "POSITION":
        A = np.array([[0.0, 1.0], [0.0, -kv / ka]])
        B = np.array([[0.0], [1.0 / ka]])
        Q = np.diag([1.0 / max_position_error**2, 1.0 / max_velocity_error**2])
        R = np.array([[1.0 / max_effort**2]])
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.solve(R, B.T @ P)
        return {"kP": float(K[0, 0]), "kI": 0.0, "kD": float(K[0, 1])}

    A = np.array([[-kv / ka]])
    B = np.array([[1.0 / ka]])
    Q = np.array([[1.0 / max_velocity_error**2]])
    R = np.array([[1.0 / max_effort**2]])
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return {"kP": float(K[0, 0]), "kI": 0.0, "kD": 0.0}


def motion_magic(ff: dict, ka: float, gravity_model: str, vbat: float) -> dict:
    """Conservative Motion Magic starting point: 85% of steady-state top speed and 70%
    of the acceleration still available at that cruise velocity."""
    kg_max = abs(ff["kG"]) if gravity_model in ("CONSTANT", "COSINE") else 0.0
    headroom = vbat - ff["kS"] - kg_max
    if headroom <= 0 or ff["kV"] <= 0:
        return {"cruiseVelocity": 0.0, "acceleration": 0.0}
    vmax = headroom / ff["kV"]
    cruise = 0.85 * vmax
    accel_avail = (headroom - ff["kV"] * cruise) / ka
    return {"cruiseVelocity": cruise, "acceleration": max(0.70 * accel_avail, 0.0)}


# ---------------------------------------------------------------------------
# Current-limit sweep
# ---------------------------------------------------------------------------

def current_sweep(ff: dict, ka: float, phys: dict, *, gravity_model: str,
                  horizontal: float, ratio: float, n_motors: int, vbat: float,
                  motion: dict, perf_target: float = 0.95,
                  limits: np.ndarray | None = None) -> dict:
    """Simulates a representative motion at each stator-current limit.

    The limiter model: stator current I = (V_applied - EMF)/R per motor, and the
    controller may apply any voltage within the battery that keeps |I| <= limit. The
    simulation always applies maximum effort within both constraints — full throttle,
    then (for point-to-point moves) full braking timed to stop at the target.

    motion: {"kind": "p2p"|"spinup"|"sprint", "distance": float, "target_vel": float}
    """
    ks, kv, kg = ff["kS"], ff["kV"], ff["kG"]
    r_motor = phys["resistance"]
    ke_mech = phys["ke_v_per_rotor_rps"] * ratio  # EMF volts per mechanism unit/s

    def g_of(p: float) -> float:
        if gravity_model == "CONSTANT":
            return 1.0
        if gravity_model == "COSINE":
            return float(np.cos(2.0 * np.pi * (p - horizontal)))
        return 0.0

    def accel(v_app: float, v: float, p: float) -> float:
        return (v_app - ks * np.sign(v) - kg * g_of(p) - kv * v) / ka

    def simulate(i_lim: float, record: bool):
        dt = 1e-3
        v = p = t = 0.0
        kind = motion["kind"]
        dist = motion.get("distance", 0.0)
        target_v = motion.get("target_vel", 0.0)
        trace = {"t": [], "vel": [], "pos": [], "stator": [], "supply": []} if record else None
        peak_supply = 0.0

        for _ in range(int(30.0 / dt)):
            emf = ke_mech * v
            v_hi = min(vbat, emf + i_lim * r_motor)
            v_lo = max(-vbat, emf - i_lim * r_motor)

            braking = False
            if kind == "p2p" and v > 0:
                a_brake = accel(v_lo, v, p)
                if a_brake < 0:
                    stop_dist = v * v / (2.0 * -a_brake)
                    braking = (dist - p) <= stop_dist
            v_app = v_lo if braking else v_hi

            a = accel(v_app, v, p)
            if not braking and a <= 0 and v <= 1e-9:
                return np.inf, trace, peak_supply  # cannot overcome friction/gravity

            i_stator = (v_app - emf) / r_motor
            i_supply = abs(i_stator) * min(abs(v_app) / vbat, 1.0) * n_motors
            peak_supply = max(peak_supply, i_supply)
            if record:
                trace["t"].append(t)
                trace["vel"].append(v)
                trace["pos"].append(p)
                trace["stator"].append(abs(i_stator))
                trace["supply"].append(i_supply)

            v = max(v + a * dt, 0.0)
            p += v * dt
            t += dt

            if kind in ("p2p", "sprint") and p >= dist:
                return t, trace, peak_supply
            if kind == "spinup" and v >= target_v:
                return t, trace, peak_supply
        return np.inf, trace, peak_supply

    if limits is None:
        top = min(phys["stall_current"], 150.0)
        limits = np.arange(10.0, top + 1e-9, 5.0)

    # Performance is measured against the highest limit anyone would realistically
    # configure (the top of the sweep), not the motor's stall current — the question
    # being answered is "how low can the limit go before it costs performance".
    baseline_t, _, _ = simulate(float(limits[-1]), record=False)
    times, perfs, peaks, traces = [], [], [], []
    for lim in limits:
        t_move, trace, peak = simulate(float(lim), record=True)
        times.append(t_move)
        perfs.append(0.0 if not np.isfinite(t_move) else 100.0 * baseline_t / t_move)
        peaks.append(peak)
        traces.append(_downsample_trace(trace))

    perfs_arr = np.array(perfs)

    def first_at(threshold: float):
        idx = np.nonzero(perfs_arr >= threshold)[0]
        return float(limits[idx[0]]) if len(idx) else None

    return {
        "motion": motion,
        "limitsAmps": [float(x) for x in limits],
        "moveTimeSec": [None if not np.isfinite(x) else float(x) for x in times],
        "performancePercent": [float(x) for x in perfs],
        "peakSupplyAmpsTotal": [float(x) for x in peaks],
        "baselineTimeSec": None if not np.isfinite(baseline_t) else float(baseline_t),
        "recommended": {
            "amps90": first_at(90.0),
            "amps95": first_at(95.0),
            "amps98": first_at(98.0),
            "target": first_at(100.0 * perf_target),
            "targetPercent": 100.0 * perf_target,
        },
        "traces": traces,
    }


def _downsample_trace(trace: dict | None, max_points: int = 200) -> dict:
    if trace is None or not trace["t"]:
        return {"t": [], "vel": [], "pos": [], "stator": [], "supply": []}
    n = len(trace["t"])
    idx = np.linspace(0, n - 1, min(n, max_points)).astype(int)
    return {k: [float(np.asarray(v)[i]) for i in idx] for k, v in trace.items()}


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_segments(motor: MotorData, ff: dict, ka: float, gravity_model: str,
                      horizontal: float, max_segments: int = 6) -> list[dict]:
    """Replay the identified model against measured voltage over each dynamic segment
    and report the velocity-fit percentage (NRMSE)."""
    out = []
    dt = motor.dt
    for seg in np.unique(motor.seg_id[motor.seg_kind == KIND_DYNAMIC]):
        sel = motor.seg_id == seg
        if sel.sum() < 20:
            continue
        t, vel, volts, pos = motor.t[sel], motor.vel[sel], motor.volts[sel], motor.pos[sel]
        sim = np.empty_like(vel)
        sim[0] = vel[0]
        for k in range(len(vel) - 1):
            a = (
                volts[k]
                - ff["kS"] * np.sign(sim[k])
                - ff["kG"] * (gravity_series(gravity_model, pos[k : k + 1], horizontal)[0])
                - ff["kV"] * sim[k]
            ) / ka
            sim[k + 1] = sim[k] + a * dt
        denom = float(np.linalg.norm(vel - vel.mean()))
        fit = 100.0 * (1.0 - float(np.linalg.norm(vel - sim)) / denom) if denom > 0 else 0.0
        n = len(t)
        idx = np.linspace(0, n - 1, min(n, 300)).astype(int)
        out.append({
            "id": motor.tests[int(np.nonzero(sel)[0][0])],
            "fitPercent": fit,
            "t": [float(t[i] - t[0]) for i in idx],
            "measured": [float(vel[i]) for i in idx],
            "simulated": [float(sim[i]) for i in idx],
        })
        if len(out) >= max_segments:
            break
    return out
