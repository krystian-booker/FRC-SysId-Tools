"""End-to-end pipeline tests against synthetic data with known ground truth.

The Python engine runs everywhere. The MATLAB test runs only when MATLAB is
installed (skipped otherwise) — run it before releases.
"""

import json
import shutil
from pathlib import Path

import pytest

from mechid.engines import python_engine
from mechid.engines.matlab_engine import find_matlab
from mechid.options import AnalysisOptions
from mechid.session import load_session
from mechid.synth import generate

MECHS = ["TURRET", "ELEVATOR", "ARM", "FLYWHEEL", "SWERVE_DRIVE", "VORTEX_ROLLER"]


def _check(results: dict, truth: dict, tol: float):
    c = results["combined"]
    for key in ("kS", "kV", "kA"):
        got, want = c[key], truth[key]
        assert got == pytest.approx(want, rel=tol, abs=0.02), (
            f"{key}: recovered {got:.4f}, truth {want:.4f}"
        )
    if truth.get("kG"):
        assert c["kG"] == pytest.approx(truth["kG"], rel=tol, abs=0.03)
    pid = c["pid"]
    assert pid["kP"] > 0
    if results["pidDomain"] == "POSITION":
        assert pid["kD"] >= 0
    sweep = results["currentSweep"]
    assert sweep["recommended"]["target"] is not None
    perfs = sweep["performancePercent"]
    assert perfs[-1] >= perfs[0]  # more current never hurts


@pytest.fixture(scope="module")
def sessions(tmp_path_factory):
    root = tmp_path_factory.mktemp("synth")
    out = {}
    for mech in MECHS:
        d = root / mech.lower()
        truth = generate(mech, d)
        out[mech] = (d, truth)
    return out


@pytest.mark.parametrize("mech", MECHS)
def test_python_engine_recovers_truth(sessions, mech):
    d, truth = sessions[mech]
    session = load_session(d)
    results = python_engine.analyze(session, AnalysisOptions())
    _check(results, truth, tol=0.10)


def test_swerve_fits_all_modules(sessions):
    d, _ = sessions["SWERVE_DRIVE"]
    session = load_session(d)
    results = python_engine.analyze(session, AnalysisOptions())
    assert len(results["motors"]) == 4
    assert results["combined"]["moduleSpreadPercent"] is not None


def test_report_renders(sessions, tmp_path):
    from mechid.report import java_snippet, render_report

    d, _ = sessions["TURRET"]
    session = load_session(d)
    results = python_engine.analyze(session, AnalysisOptions())
    out = render_report(results, session.meta, tmp_path / "report.html")
    html = out.read_text()
    assert "plotly" in html.lower()
    assert "withKS" in html
    snippet = java_snippet(results, session.meta)
    assert "Slot0Configs" in snippet


@pytest.mark.skipif(find_matlab(AnalysisOptions()) is None, reason="MATLAB not installed")
@pytest.mark.parametrize("mech", ["TURRET", "FLYWHEEL"])
def test_matlab_engine_recovers_truth_and_agrees_with_python(sessions, mech, tmp_path):
    from mechid.engines import matlab_engine

    d, truth = sessions[mech]
    session = load_session(d)
    results = matlab_engine.analyze(session, AnalysisOptions(), tmp_path / "matlab")
    _check(results, truth, tol=0.10)

    # Cross-engine agreement on the same data. Feedforward comes from the same OLS in
    # both engines and must agree tightly; kA comes from different estimators (tfest
    # vs acceleration regression), so it gets a looser band.
    py = python_engine.analyze(session, AnalysisOptions())
    cm, cp = results["combined"], py["combined"]
    for key, tol in (("kS", 0.05), ("kV", 0.02), ("kA", 0.30)):
        assert cm[key] == pytest.approx(cp[key], rel=tol, abs=0.01), (
            f"{key}: matlab {cm[key]:.4f} vs python {cp[key]:.4f}"
        )
    assert cm["pid"]["kP"] == pytest.approx(cp["pid"]["kP"], rel=0.05)
