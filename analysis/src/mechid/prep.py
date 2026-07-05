"""Shared pre/post-analysis logic used by both engines: derived defaults, gearing
trust, and the representative motion for the current-limit sweep."""

from __future__ import annotations

import numpy as np

from .options import AnalysisOptions
from .session import Session, motor_physics


def median_battery(session: Session) -> float:
    v = float(np.median(session.battery))
    return v if 8.0 < v < 14.0 else 12.0


def resolve_gearing(session: Session, kv_combined: float, warnings: list[str]) -> dict:
    """Rotor rotations per mechanism unit. Falls back to estimating from identified kV
    against the motor's EMF constant when the device-reported ratio can't be trusted
    (RemoteCANcoder feedback)."""
    gearing = session.meta.get("gearing", {})
    ratio = gearing.get("rotorToMechanismRatio")
    trusted = bool(gearing.get("ratioTrusted", False))
    phys = motor_physics(session.meta)

    if trusted and ratio and ratio > 0:
        return {"ratio": float(ratio), "trusted": True, "estimated": False}

    est = kv_combined / phys["ke_v_per_rotor_rps"]
    warnings.append(
        f"Gear ratio not trusted from device config (RemoteCANcoder feedback?). "
        f"Estimated rotor-to-mechanism ratio {est:.1f} from identified kV — set "
        f".withRotorToMechanismRatio() in the robot config for accurate current modeling."
    )
    return {"ratio": float(est), "trusted": False, "estimated": True}


def sweep_motion(session: Session, max_vel_seen: float, warnings: list[str]) -> dict:
    """The representative motion whose completion time defines 'performance' in the
    current-limit sweep."""
    meta = session.meta
    mech = session.mechanism_type
    env = meta.get("envelope", {})

    if mech == "SWERVE_DRIVE":
        extra = meta.get("extra", {})
        radius = extra.get("wheelRadiusMeters")
        travel = extra.get("travelMeters") or 3.0
        if radius and np.isfinite(radius) and radius > 0:
            dist_m = min(3.0, travel)
            return {
                "kind": "sprint",
                "distance": dist_m / (2.0 * np.pi * radius),  # wheel rotations
                "label": f"{dist_m:.1f} m sprint from standstill",
            }
        warnings.append("No wheel radius provided; using spin-up motion for the drive sweep")
        return {"kind": "spinup", "target_vel": 0.9 * max_vel_seen,
                "label": f"spin-up to {0.9 * max_vel_seen:.1f} wheel rot/s"}

    if mech == "SWERVE_STEER":
        return {"kind": "p2p", "distance": 0.25, "label": "90° steer move"}

    if meta["pidDomain"] == "POSITION":
        lo, hi = env.get("minPosition"), env.get("maxPosition")
        if lo is not None and hi is not None and np.isfinite(lo) and np.isfinite(hi):
            dist = 0.9 * (hi - lo)
        else:
            dist = 0.25
        return {"kind": "p2p", "distance": float(dist),
                "label": f"{dist:.2f} {meta.get('positionUnits', 'rot')} point-to-point move"}

    target = 0.9 * max_vel_seen
    return {"kind": "spinup", "target_vel": float(target),
            "label": f"spin-up to {target:.1f} {meta.get('positionUnits', 'rot')}/s"}


def lqr_defaults(session: Session, opts: AnalysisOptions, max_vel_seen: float) -> dict:
    env = session.meta.get("envelope", {})
    qp = opts.max_position_error
    if qp is None:
        lo, hi = env.get("minPosition"), env.get("maxPosition")
        if lo is not None and hi is not None and np.isfinite(lo) and np.isfinite(hi) and hi > lo:
            qp = 0.01 * (hi - lo)
        else:
            qp = 0.02
    qv = opts.max_velocity_error if opts.max_velocity_error is not None else 0.15 * max_vel_seen
    qv = max(qv, 1e-6)
    return {
        "max_position_error": float(qp),
        "max_velocity_error": float(qv),
        "max_effort": float(opts.max_effort),
    }
