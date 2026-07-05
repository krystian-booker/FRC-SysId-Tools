"""MATLAB analysis engine. Packs the session into a .mat file, runs
matlab/mechid_analyze.m headless via `matlab -batch`, and reads back results.json.

MATLAB does the identification with the System Identification Toolbox (tfest for the
dynamics) and the Control System Toolbox (lqr for gains); the results schema matches
the Python engine so reports and comparisons are engine-agnostic.
"""

from __future__ import annotations

import glob
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import scipy.io

from ..options import AnalysisOptions
from ..prep import median_battery, sweep_motion
from ..session import Session, motor_physics

MATLAB_DIR = Path(__file__).resolve().parent.parent / "matlab"


def find_matlab(opts: AnalysisOptions) -> str | None:
    import os

    candidates = [opts.matlab_path, os.environ.get("MECHID_MATLAB"), shutil.which("matlab")]
    for c in candidates:
        if c and Path(c).exists():
            return c
    apps = sorted(glob.glob("/Applications/MATLAB_R*.app/bin/matlab"), reverse=True)
    return apps[0] if apps else None


def analyze(session: Session, opts: AnalysisOptions, workdir: Path) -> dict:
    matlab = find_matlab(opts)
    if matlab is None:
        raise RuntimeError(
            "MATLAB not found. Install MATLAB, pass --matlab-path, or use --engine python."
        )

    workdir.mkdir(parents=True, exist_ok=True)
    mat_path = workdir / "session.mat"
    out_json = workdir / "results_matlab.json"
    if out_json.exists():
        out_json.unlink()

    _write_mat(session, opts, mat_path)

    def q(p) -> str:  # single-quote a path for MATLAB
        return "'" + str(p).replace("'", "''") + "'"

    cmd = [
        matlab, "-batch",
        f"addpath({q(MATLAB_DIR)}); mechid_analyze({q(mat_path)}, {q(out_json)})",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    log = (workdir / "matlab_log.txt")
    log.write_text(proc.stdout + "\n--- stderr ---\n" + proc.stderr)
    if proc.returncode != 0 or not out_json.exists():
        raise RuntimeError(
            f"MATLAB analysis failed (exit {proc.returncode}). See {log}.\n"
            + proc.stderr[-2000:]
        )

    results = json.loads(out_json.read_text())
    return _normalize(results)


def _write_mat(session: Session, opts: AnalysisOptions, mat_path: Path) -> None:
    meta = session.meta
    env = meta.get("envelope", {})
    gearing = meta.get("gearing", {})
    warnings: list[str] = []

    # Position-error default is data-independent; velocity default (0.15 * max seen)
    # is resolved inside MATLAB after the fit.
    qp = opts.max_position_error
    if qp is None:
        lo, hi = env.get("minPosition"), env.get("maxPosition")
        if lo is not None and hi is not None and np.isfinite(lo) and np.isfinite(hi) and hi > lo:
            qp = 0.01 * (hi - lo)
        else:
            qp = 0.02

    # Motion template: target_vel (spin-up) is filled in by MATLAB post-fit.
    motion = sweep_motion(session, max_vel_seen=1.0, warnings=[])
    if motion["kind"] == "spinup":
        motion = {"kind": "spinup", "target_vel_factor": 0.9, "label": motion["label"]}

    analysis = {
        "sessionName": session.name,
        "mechanismType": meta["mechanismType"],
        "pidDomain": meta["pidDomain"],
        "gravityModel": meta["gravityModel"],
        "horizontal": env.get("armHorizontalPosition", 0.0) or 0.0,
        "positionUnits": meta.get("positionUnits", "rot"),
        "vbat": median_battery(session),
        "nMotors": session.n_motors,
        "phys": motor_physics(meta),
        "gearing": {
            "ratio": gearing.get("rotorToMechanismRatio"),
            "trusted": bool(gearing.get("ratioTrusted", False)),
        },
        "envelope": {"minPosition": env.get("minPosition"), "maxPosition": env.get("maxPosition")},
        "motionTemplate": motion,
        "lqr": {
            "max_position_error": qp,
            "max_velocity_error": opts.max_velocity_error,
            "max_effort": opts.max_effort,
        },
        "perf_target": opts.perf_target,
        "prewarnings": warnings,
    }

    motors = [
        {
            "name": m.name,
            "t": m.t, "pos": m.pos, "vel": m.vel, "acc": m.acc,
            "volts": m.volts, "stator": m.stator, "supply": m.supply,
            "seg_kind": m.seg_kind.astype(float), "seg_id": m.seg_id.astype(float),
        }
        for m in session.fit_motors()
    ]
    scipy.io.savemat(
        mat_path,
        {"analysis_json": json.dumps(analysis), "motors": motors},
        oned_as="column",
    )


def _normalize(results: dict) -> dict:
    """MATLAB's jsonencode collapses 1-element arrays to scalars/objects; re-listify."""

    def as_list(x):
        if x is None:
            return []
        return x if isinstance(x, list) else [x]

    results["motors"] = as_list(results.get("motors"))
    for m in results["motors"]:
        m["validation"] = as_list(m.get("validation"))
    results["warnings"] = [w for w in as_list(results.get("warnings")) if w]
    sweep = results.get("currentSweep", {})
    for key in ("limitsAmps", "moveTimeSec", "performancePercent", "peakSupplyAmpsTotal"):
        sweep[key] = as_list(sweep.get(key))
    sweep["traces"] = as_list(sweep.get("traces"))
    return results
