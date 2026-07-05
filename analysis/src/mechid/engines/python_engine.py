"""Pure-Python analysis engine (numpy/scipy). Produces the same results schema as the
MATLAB engine; useful as a cross-check and for machines without MATLAB."""

from __future__ import annotations

import numpy as np

from .. import physics
from ..options import AnalysisOptions
from ..prep import lqr_defaults, median_battery, resolve_gearing, sweep_motion
from ..session import Session, motor_physics


def analyze(session: Session, opts: AnalysisOptions) -> dict:
    meta = session.meta
    gravity = meta["gravityModel"]
    horizontal = meta.get("envelope", {}).get("armHorizontalPosition", 0.0) or 0.0
    warnings: list[str] = []

    motors_out = []
    for motor in session.fit_motors():
        # Two-pass fit: kA from the first pass compensates the ramp-lag bias in the
        # quasistatic regression on the second.
        ff = physics.fit_feedforward(motor, gravity, horizontal)
        dyn = physics.fit_dynamics(motor, ff, gravity, horizontal)
        ff = physics.fit_feedforward(motor, gravity, horizontal, ka_comp=dyn["kA"])
        dyn = physics.fit_dynamics(motor, ff, gravity, horizontal)
        warnings.extend(ff.pop("warnings"))
        warnings.extend(dyn.pop("warnings"))
        split = physics.friction_split(motor, ff, dyn["kA"])
        validation = physics.validate_segments(motor, ff, dyn["kA"], gravity, horizontal)

        n = len(ff["scatter_vel"])
        idx = np.linspace(0, n - 1, min(n, 1500)).astype(int)
        motors_out.append({
            "name": motor.name,
            "kS": ff["kS"], "kV": ff["kV"], "kA": dyn["kA"], "kG": ff["kG"],
            "kV_dynamic": dyn["kV_dynamic"],
            "r2_quasistatic": ff["r_squared"],
            "r2_dynamic": dyn["r_squared"],
            "n_quasistatic": ff["n_samples"],
            "n_dynamic": dyn["n_samples"],
            "max_velocity_seen": ff["max_velocity_seen"],
            "frictionSplit": split,
            "scatter": {
                "vel": [float(ff["scatter_vel"][i]) for i in idx],
                "volts": [float(ff["scatter_volts"][i]) for i in idx],
                "fit": [float(ff["scatter_fit"][i]) for i in idx],
            },
            "validation": validation,
        })

    combined = _combine(motors_out, warnings)
    max_vel_seen = max(m["max_velocity_seen"] for m in motors_out)
    vbat = median_battery(session)
    gearing = resolve_gearing(session, combined["kV"], warnings)
    lqr_params = lqr_defaults(session, opts, max_vel_seen)

    combined["pid"] = physics.lqr_gains(
        meta["pidDomain"], combined["kV"], combined["kA"], **lqr_params)
    combined["motionMagic"] = physics.motion_magic(
        combined, combined["kA"], gravity, vbat)
    combined["lqrParams"] = lqr_params

    motion = sweep_motion(session, max_vel_seen, warnings)
    sweep = physics.current_sweep(
        combined, combined["kA"], motor_physics(meta),
        gravity_model=gravity, horizontal=horizontal,
        ratio=gearing["ratio"], n_motors=session.n_motors, vbat=vbat,
        motion=motion, perf_target=opts.perf_target)

    return {
        "engine": "python",
        "sessionName": session.name,
        "mechanismType": meta["mechanismType"],
        "pidDomain": meta["pidDomain"],
        "gravityModel": gravity,
        "positionUnits": meta.get("positionUnits", "rot"),
        "vbat": vbat,
        "nMotors": session.n_motors,
        "motors": motors_out,
        "combined": combined,
        "gearing": gearing,
        "phys": motor_physics(meta),
        "currentSweep": sweep,
        "warnings": warnings,
    }


def _combine(motors: list[dict], warnings: list[str]) -> dict:
    keys = ("kS", "kV", "kA", "kG")
    combined = {k: float(np.mean([m[k] for m in motors])) for k in keys}
    if len(motors) > 1:
        spreads = []
        for k in ("kV", "kA"):
            vals = np.array([m[k] for m in motors])
            if abs(combined[k]) > 1e-9:
                spreads.append(100.0 * float(np.ptp(vals)) / abs(combined[k]))
        spread = max(spreads) if spreads else 0.0
        combined["moduleSpreadPercent"] = spread
        if spread > 15.0:
            warnings.append(
                f"Per-module constants spread by {spread:.0f}% — check for a dragging "
                "or mechanically unhealthy module before trusting shared gains.")
    else:
        combined["moduleSpreadPercent"] = None
    return combined
