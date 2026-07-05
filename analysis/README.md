# MechID analysis tool

Turns a MechID session recorded on the robot into feedforward, PID, Motion Magic,
and stator-current-limit recommendations, plus an interactive HTML report.

See the [repository README](../README.md) for the full workflow. Quick reference:

```bash
cd analysis
uv sync

# Copy sessions off the roboRIO (tries 10.28.52.2, mDNS, then USB)
uv run mechid pull --team 2852

# Analyze (MATLAB engine by default when installed; falls back to Python)
uv run mechid analyze mechid_data/turret_20260704_193012

# Cross-check both engines against each other
uv run mechid analyze mechid_data/turret_20260704_193012 --engine both

# Try the pipeline without a robot
uv run mechid synth TURRET --out demo && uv run mechid analyze demo
```

Useful flags for `analyze`:

| Flag | Meaning | Default |
|---|---|---|
| `--engine` | `auto`, `matlab`, `python`, `both` | `auto` |
| `--max-position-error` | LQR position tolerance (mechanism units) — smaller = stiffer kP | 1% of travel |
| `--max-velocity-error` | LQR velocity tolerance (units/s) | 15% of max velocity seen |
| `--max-effort` | LQR voltage budget | 7 V |
| `--perf-target` | current-limit recommendation threshold | 0.95 |
| `--matlab-path` | explicit MATLAB binary | auto-detected |

MATLAB engine requirements: MATLAB with the **System Identification Toolbox** (`tfest`)
and **Control System Toolbox** (`lqr`). Run headless — no license for the desktop UI is
consumed beyond a normal MATLAB session.

Tests (`uv run pytest`) generate synthetic sessions from known ground-truth constants
and assert the pipeline recovers them; the MATLAB tests are skipped automatically when
MATLAB isn't installed.
