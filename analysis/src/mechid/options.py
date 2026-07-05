"""Analysis options shared by both engines."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AnalysisOptions:
    engine: str = "auto"  # auto | matlab | python | both
    # LQR weights (WPILib-SysId-style). None = derived from the data:
    # position error tolerance defaults to 1% of travel, velocity error tolerance to
    # 15% of the max velocity seen in the test.
    max_position_error: float | None = None
    max_velocity_error: float | None = None
    max_effort: float = 7.0
    # Current-limit recommendation: smallest limit achieving this fraction of
    # unlimited performance.
    perf_target: float = 0.95
    matlab_path: str | None = None
