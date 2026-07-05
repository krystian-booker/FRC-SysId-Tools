"""MechID command-line interface.

    mechid pull --team 2852              # copy sessions off the roboRIO
    mechid analyze mechid_data/turret_20260704_193012
    mechid synth TURRET --out demo       # synthetic session for trying the pipeline
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from .options import AnalysisOptions
from .session import load_session


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="mechid", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    pa = sub.add_parser("analyze", help="Analyze one or more session folders")
    pa.add_argument("sessions", nargs="+", help="Session folder(s) containing meta.json + data.csv")
    pa.add_argument("--engine", choices=["auto", "matlab", "python", "both"], default="auto")
    pa.add_argument("--out", default=None, help="Output folder (default: <session>/analysis)")
    pa.add_argument("--max-position-error", type=float, default=None,
                    help="LQR position error tolerance, mechanism units (default: 1%% of travel)")
    pa.add_argument("--max-velocity-error", type=float, default=None,
                    help="LQR velocity error tolerance, units/s (default: 15%% of max velocity)")
    pa.add_argument("--max-effort", type=float, default=7.0,
                    help="LQR control effort budget in volts (default 7)")
    pa.add_argument("--perf-target", type=float, default=0.95,
                    help="Current limit recommendation performance target (default 0.95)")
    pa.add_argument("--matlab-path", default=None)

    pp = sub.add_parser("pull", help="Copy recorded sessions from the roboRIO")
    group = pp.add_mutually_exclusive_group(required=True)
    group.add_argument("--team", type=int, help="Team number (derives 10.TE.AM.2 etc.)")
    group.add_argument("--host", help="Explicit roboRIO hostname/IP")
    pp.add_argument("--dest", default="mechid_data")
    pp.add_argument("--source", default="/home/lvuser/mechid")

    ps = sub.add_parser("synth", help="Generate a synthetic session (pipeline demo/testing)")
    ps.add_argument("mech_type",
                    choices=["TURRET", "ELEVATOR", "ARM", "FLYWHEEL", "SWERVE_DRIVE", "VORTEX_ROLLER"])
    ps.add_argument("--out", default=None)
    ps.add_argument("--seed", type=int, default=42)

    args = parser.parse_args(argv)
    if args.cmd == "analyze":
        return cmd_analyze(args)
    if args.cmd == "pull":
        return cmd_pull(args)
    if args.cmd == "synth":
        return cmd_synth(args)
    return 2


# ---------------------------------------------------------------------------

def cmd_analyze(args) -> int:
    from .report import console_summary, render_report

    opts = AnalysisOptions(
        engine=args.engine,
        max_position_error=args.max_position_error,
        max_velocity_error=args.max_velocity_error,
        max_effort=args.max_effort,
        perf_target=args.perf_target,
        matlab_path=args.matlab_path,
    )

    exit_code = 0
    for session_dir in args.sessions:
        try:
            session = load_session(session_dir)
        except Exception as e:
            print(f"error: {session_dir}: {e}", file=sys.stderr)
            exit_code = 1
            continue

        out_dir = Path(args.out) if args.out else Path(session_dir) / "analysis"
        out_dir.mkdir(parents=True, exist_ok=True)

        engines = _resolve_engines(opts)
        all_results = {}
        for engine in engines:
            try:
                results = _run_engine(engine, session, opts, out_dir)
            except Exception as e:
                print(f"error: {session.name} [{engine}]: {e}", file=sys.stderr)
                exit_code = 1
                continue
            all_results[engine] = results
            (out_dir / f"results_{engine}.json").write_text(json.dumps(results, indent=2))
            suffix = f"_{engine}" if len(engines) > 1 else ""
            report = render_report(results, session.meta, out_dir / f"report{suffix}.html")
            print(console_summary(results))
            print(f"  Report: {report}\n")

        if len(all_results) == 2:
            _print_engine_comparison(all_results)
    return exit_code


def _resolve_engines(opts: AnalysisOptions) -> list[str]:
    from .engines.matlab_engine import find_matlab

    if opts.engine == "both":
        return ["matlab", "python"]
    if opts.engine in ("matlab", "python"):
        return [opts.engine]
    if find_matlab(opts):
        return ["matlab"]
    print("note: MATLAB not found; using the Python engine (pass --matlab-path to override)")
    return ["python"]


def _run_engine(engine: str, session, opts: AnalysisOptions, out_dir: Path) -> dict:
    if engine == "matlab":
        from .engines import matlab_engine

        print(f"Running MATLAB analysis for {session.name} (first run takes ~30s to start MATLAB)...")
        return matlab_engine.analyze(session, opts, out_dir / "matlab_work")
    from .engines import python_engine

    return python_engine.analyze(session, opts)


def _print_engine_comparison(all_results: dict) -> None:
    m, p = all_results["matlab"]["combined"], all_results["python"]["combined"]
    print("=== Engine cross-check (matlab vs python) ===")
    for key in ("kS", "kV", "kA", "kG"):
        vm, vp = m[key], p[key]
        delta = abs(vm - vp) / max(abs(vm), 1e-9) * 100
        print(f"  {key}: {vm:.4g} vs {vp:.4g}  ({delta:.1f}% apart)")
    print(f"  kP: {m['pid']['kP']:.4g} vs {p['pid']['kP']:.4g}   "
          f"kD: {m['pid']['kD']:.4g} vs {p['pid']['kD']:.4g}")


# ---------------------------------------------------------------------------

def cmd_pull(args) -> int:
    if args.host:
        hosts = [args.host]
    else:
        t = args.team
        hosts = [f"10.{t // 100}.{t % 100}.2", f"roborio-{t}-frc.local", "172.22.11.2"]

    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    ssh_opts = ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
                "-o", "ConnectTimeout=5", "-o", "LogLevel=ERROR"]

    for host in hosts:
        print(f"Trying {host}...")
        for extra in ([], ["-O"]):  # newer scp defaults to SFTP; -O falls back to legacy scp
            cmd = ["scp", "-r", *extra, *ssh_opts, f"lvuser@{host}:{args.source}/*", str(dest)]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            if proc.returncode == 0:
                sessions = sorted(d.name for d in dest.iterdir()
                                  if d.is_dir() and (d / "meta.json").exists())
                print(f"Pulled {len(sessions)} session(s) into {dest}/:")
                for s in sessions:
                    print(f"  {s}")
                if sessions:
                    print(f"\nNext: mechid analyze {dest}/{sessions[-1]}")
                return 0
        print(f"  no luck ({proc.stderr.strip().splitlines()[-1] if proc.stderr else 'connection failed'})")
    print("Could not reach the roboRIO. Are you on the robot network?", file=sys.stderr)
    return 1


def cmd_synth(args) -> int:
    from .synth import generate

    out = Path(args.out) if args.out else Path(f"synth_{args.mech_type.lower()}")
    truth = generate(args.mech_type, out, seed=args.seed)
    print(f"Wrote synthetic {args.mech_type} session to {out}/")
    print(f"Ground truth: {json.dumps(truth)}")
    print(f"Try: mechid analyze {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
