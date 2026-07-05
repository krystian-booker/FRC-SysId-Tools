"""Self-contained HTML report: fit quality, gains with a paste-ready Phoenix 6 snippet,
and the interactive current-limit explorer.

Colors follow the MechID viz palette (validated categorical order; status colors
reserved for status).
"""

from __future__ import annotations

import datetime
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Validated categorical palette (light surface), fixed order.
SERIES = ["#2a78d6", "#1baf7a", "#eda100", "#008300", "#4a3aa7", "#e34948", "#e87ba4", "#eb6834"]
STATUS_GOOD = "#0ca30c"
STATUS_WARNING = "#fab219"
STATUS_CRITICAL = "#d03b3b"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"
PAGE = "#f9f9f7"

LAYOUT_DEFAULTS = dict(
    paper_bgcolor=SURFACE,
    plot_bgcolor=SURFACE,
    font=dict(family='system-ui, -apple-system, "Segoe UI", sans-serif', color=INK_2, size=13),
    margin=dict(l=60, r=20, t=48, b=48),
    hovermode="closest",
)


def _axes(fig):
    fig.update_xaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor="#c3c2b7",
                     title_font_color=MUTED, tickfont_color=MUTED)
    fig.update_yaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor="#c3c2b7",
                     title_font_color=MUTED, tickfont_color=MUTED)
    return fig


def fmt(x, digits=4):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "—"
    if x == 0:
        return "0"
    return f"{x:.{digits}g}"


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def _fig_quasistatic(results: dict) -> go.Figure:
    fig = go.Figure()
    motors = results["motors"]
    for i, m in enumerate(motors):
        sc = m.get("scatter") or {}
        if not sc.get("vel"):
            continue
        color = SERIES[i % len(SERIES)]
        fig.add_trace(go.Scattergl(
            x=sc["vel"], y=sc["volts"], mode="markers",
            marker=dict(size=4, color=color, opacity=0.35),
            name=f"{m['name']} measured",
            hovertemplate="v=%{x:.3f}, V=%{y:.2f}<extra>" + m["name"] + "</extra>"))
        order = np.argsort(sc["vel"])
        fig.add_trace(go.Scatter(
            x=[sc["vel"][j] for j in order], y=[sc["fit"][j] for j in order],
            mode="lines", line=dict(color=color, width=2),
            name=f"{m['name']} fit"))
    u = results.get("positionUnits", "rot")
    fig.update_layout(
        **LAYOUT_DEFAULTS,
        title=dict(text="Quasistatic: voltage vs velocity (slope = kV, intercept = kS±kG)",
                   font=dict(color=INK, size=15)),
        xaxis_title=f"velocity ({u}/s)", yaxis_title="applied volts (V)",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        height=420)
    return _axes(fig)


def _fig_validation(results: dict) -> go.Figure | None:
    segs = []
    for i, m in enumerate(results["motors"]):
        for seg in m.get("validation", []):
            segs.append((m["name"], seg))
    segs = segs[:6]
    if not segs:
        return None
    cols = min(3, len(segs))
    rows = int(np.ceil(len(segs) / cols))
    titles = [f"{name}: {seg['id']} — fit {seg['fitPercent']:.0f}%" for name, seg in segs]
    fig = make_subplots(rows=rows, cols=cols, subplot_titles=titles,
                        shared_yaxes=False, vertical_spacing=0.16)
    for k, (name, seg) in enumerate(segs):
        r, c = k // cols + 1, k % cols + 1
        fig.add_trace(go.Scatter(x=seg["t"], y=seg["measured"], mode="lines",
                                 line=dict(color=SERIES[0], width=2),
                                 name="measured", showlegend=(k == 0)), row=r, col=c)
        fig.add_trace(go.Scatter(x=seg["t"], y=seg["simulated"], mode="lines",
                                 line=dict(color=SERIES[5], width=2, dash="dot"),
                                 name="model", showlegend=(k == 0)), row=r, col=c)
    u = results.get("positionUnits", "rot")
    fig.update_layout(
        **LAYOUT_DEFAULTS,
        title=dict(text="Model validation: measured vs simulated velocity on step tests",
                   font=dict(color=INK, size=15)),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        height=280 * rows + 80)
    fig.update_annotations(font=dict(size=12, color=INK_2))
    fig.update_xaxes(title_text="t (s)", row=rows)
    fig.update_yaxes(title_text=f"vel ({u}/s)", col=1)
    return _axes(fig)


def _fig_sweep_static(results: dict) -> go.Figure | None:
    sweep = results.get("currentSweep") or {}
    limits = sweep.get("limitsAmps") or []
    if not limits:
        return None
    perf = sweep["performancePercent"]
    peak = sweep["peakSupplyAmpsTotal"]
    rec = sweep.get("recommended") or {}

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08,
                        subplot_titles=("Performance vs stator current limit",
                                        "Peak total supply current draw"))
    fig.add_trace(go.Scatter(x=limits, y=perf, mode="lines+markers",
                             line=dict(color=SERIES[0], width=2), marker=dict(size=6),
                             name="performance",
                             hovertemplate="%{x:.0f} A → %{y:.1f}%<extra></extra>"),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=limits, y=peak, mode="lines+markers",
                             line=dict(color=SERIES[7], width=2), marker=dict(size=6),
                             name="peak supply (all motors)",
                             hovertemplate="%{x:.0f} A limit → %{y:.0f} A supply<extra></extra>"),
                  row=2, col=1)

    target = rec.get("target")
    if target is not None:
        for row in (1, 2):
            fig.add_vline(x=target, line=dict(color=STATUS_GOOD, width=2, dash="dash"),
                          row=row, col=1)
        fig.add_annotation(x=target, y=max(perf), text=f"✓ recommended {target:.0f} A "
                           f"(≥{rec.get('targetPercent', 95):.0f}% perf)",
                           showarrow=False, yshift=14, font=dict(color=STATUS_GOOD, size=12),
                           row=1, col=1)
    for pct_key, pct in (("amps90", 90), ("amps98", 98)):
        a = rec.get(pct_key)
        if a is not None and a != target:
            fig.add_vline(x=a, line=dict(color=GRID, width=1, dash="dot"), row=1, col=1)
            fig.add_annotation(x=a, y=min(perf), text=f"{pct}% @ {a:.0f} A", showarrow=False,
                               yshift=-4, font=dict(color=MUTED, size=11), row=1, col=1)

    fig.update_layout(
        **LAYOUT_DEFAULTS,
        legend=dict(orientation="h", yanchor="bottom", y=1.04, xanchor="right", x=1),
        height=560)
    fig.update_annotations(font=dict(size=13, color=INK_2))
    fig.update_xaxes(title_text="stator current limit (A)", row=2)
    fig.update_yaxes(title_text="% of unlimited", row=1)
    fig.update_yaxes(title_text="amps", row=2)
    return _axes(fig)


def _fig_sweep_explorer(results: dict) -> go.Figure | None:
    """Slider over the stator limit: what the representative motion looks like."""
    sweep = results.get("currentSweep") or {}
    limits = sweep.get("limitsAmps") or []
    traces = sweep.get("traces") or []
    times = sweep.get("moveTimeSec") or []
    if not limits or not traces:
        return None
    u = results.get("positionUnits", "rot")
    motion_label = (sweep.get("motion") or {}).get("label", "representative motion")

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08,
                        subplot_titles=(f"Velocity — {motion_label}", "Current"))
    n = len(limits)
    default_idx = n - 1
    rec = (sweep.get("recommended") or {}).get("target")
    if rec is not None and rec in limits:
        default_idx = limits.index(rec)

    for i, (lim, tr) in enumerate(zip(limits, traces)):
        vis = i == default_idx
        fig.add_trace(go.Scatter(x=tr["t"], y=tr["vel"], mode="lines",
                                 line=dict(color=SERIES[0], width=2), name="velocity",
                                 visible=vis, showlegend=False,
                                 hovertemplate="t=%{x:.2f}s v=%{y:.2f}<extra></extra>"),
                      row=1, col=1)
        fig.add_trace(go.Scatter(x=tr["t"], y=tr["stator"], mode="lines",
                                 line=dict(color=SERIES[7], width=2), name="stator (per motor)",
                                 visible=vis, showlegend=vis,
                                 hovertemplate="t=%{x:.2f}s I=%{y:.0f}A<extra>stator</extra>"),
                      row=2, col=1)
        fig.add_trace(go.Scatter(x=tr["t"], y=tr["supply"], mode="lines",
                                 line=dict(color=SERIES[4], width=2, dash="dot"),
                                 name="supply (total)", visible=vis, showlegend=vis,
                                 hovertemplate="t=%{x:.2f}s I=%{y:.0f}A<extra>supply</extra>"),
                      row=2, col=1)

    steps = []
    for i, lim in enumerate(limits):
        visible = [False] * (3 * n)
        for k in range(3):
            visible[3 * i + k] = True
        t_txt = "did not finish" if times[i] is None else f"{times[i]:.2f} s"
        steps.append(dict(method="update", label=f"{lim:.0f}",
                          args=[{"visible": visible},
                                {"title.text": f"Motion at {lim:.0f} A stator limit — "
                                               f"move time {t_txt}"}]))

    t0 = "did not finish" if times[default_idx] is None else f"{times[default_idx]:.2f} s"
    fig.update_layout(
        **LAYOUT_DEFAULTS,
        title=dict(text=f"Motion at {limits[default_idx]:.0f} A stator limit — move time {t0}",
                   font=dict(color=INK, size=15)),
        sliders=[dict(active=default_idx, currentvalue=dict(prefix="Stator limit: ", suffix=" A",
                                                            font=dict(color=INK)),
                      pad=dict(t=36), steps=steps, tickcolor=MUTED,
                      font=dict(color=MUTED, size=11))],
        legend=dict(orientation="h", yanchor="bottom", y=1.04, xanchor="right", x=1),
        height=620)
    fig.update_annotations(font=dict(size=13, color=INK_2))
    fig.update_xaxes(title_text="t (s)", row=2)
    fig.update_yaxes(title_text=f"vel ({u}/s)", row=1)
    fig.update_yaxes(title_text="amps", row=2)
    return _axes(fig)


# ---------------------------------------------------------------------------
# Gains & snippet
# ---------------------------------------------------------------------------

def java_snippet(results: dict, meta: dict) -> str:
    c = results["combined"]
    pid = c["pid"]
    mm = c["motionMagic"]
    sweep = results.get("currentSweep") or {}
    rec = (sweep.get("recommended") or {}).get("target")
    gravity_line = {
        "CONSTANT": "        .withGravityType(GravityTypeValue.Elevator_Static)\n",
        "COSINE": "        .withGravityType(GravityTypeValue.Arm_Cosine)\n",
    }.get(results["gravityModel"], "")
    kg_line = f".withKG({fmt(c['kG'])})" if results["gravityModel"] != "NONE" else ""
    date = datetime.date.today().isoformat()
    m0 = results["motors"][0]

    lines = (
        f"// MechID {results['sessionName']} — {results['engine']} engine, {date}\n"
        f"// Quasistatic R²={m0['r2_quasistatic']:.3f}, dynamic R²={m0['r2_dynamic']:.3f}\n"
        f"config.Slot0 = new Slot0Configs()\n"
        f"        .withKS({fmt(c['kS'])}).withKV({fmt(c['kV'])}).withKA({fmt(c['kA'])}){kg_line}\n"
        f"        .withKP({fmt(pid['kP'])}).withKI({fmt(pid['kI'])}).withKD({fmt(pid['kD'])})\n"
        f"{gravity_line}        ;\n"
    )
    if results["pidDomain"] == "POSITION" and mm["cruiseVelocity"] > 0:
        lines += (
            f"config.MotionMagic.MotionMagicCruiseVelocity = {fmt(mm['cruiseVelocity'])};\n"
            f"config.MotionMagic.MotionMagicAcceleration = {fmt(mm['acceleration'])};\n"
        )
    if rec is not None:
        lines += (
            f"config.CurrentLimits.StatorCurrentLimitEnable = true;\n"
            f"config.CurrentLimits.StatorCurrentLimit = {rec:.0f}; "
            f"// ≥{(sweep['recommended'].get('targetPercent', 95)):.0f}% of unlimited performance\n"
        )
    return lines


def _existing_gains(meta: dict) -> dict | None:
    devices = meta.get("devices") or {}
    first = meta.get("motors", [None])[0]
    dev = devices.get(first) if first else None
    if dev and isinstance(dev, dict) and "slot0" in dev:
        return dev["slot0"]
    return None


# ---------------------------------------------------------------------------
# Page assembly
# ---------------------------------------------------------------------------

def render_report(results: dict, meta: dict, out_html: Path) -> Path:
    import plotly.offline

    c = results["combined"]
    pid = c["pid"]
    mm = c["motionMagic"]
    figs = [f for f in (_fig_quasistatic(results), _fig_validation(results),
                        _fig_sweep_explorer(results), _fig_sweep_static(results)) if f]
    figs_html = "\n".join(
        f.to_html(full_html=False, include_plotlyjs=False,
                  config={"displaylogo": False}) for f in figs)

    warnings_html = ""
    if results.get("warnings"):
        items = "".join(f"<li>{w}</li>" for w in results["warnings"])
        warnings_html = (
            f'<div class="card warn"><h2>⚠ Warnings</h2><ul>{items}</ul></div>'
        )

    gain_rows = [
        ("kS", c["kS"], "V"), ("kV", c["kV"], "V per unit/s"), ("kA", c["kA"], "V per unit/s²"),
        ("kG", c["kG"], "V"), ("kP", pid["kP"], ""), ("kI", pid["kI"], ""), ("kD", pid["kD"], ""),
        ("MM cruise", mm["cruiseVelocity"], "unit/s"), ("MM accel", mm["acceleration"], "unit/s²"),
    ]
    existing = _existing_gains(meta)
    header = "<tr><th>Constant</th><th>New</th>"
    if existing:
        header += "<th>Currently on robot</th>"
    header += "<th>Units</th></tr>"
    rows_html = ""
    old_map = {"kS": "kS", "kV": "kV", "kA": "kA", "kG": "kG", "kP": "kP", "kI": "kI", "kD": "kD"}
    for name, val, unit in gain_rows:
        old = ""
        if existing:
            old_val = existing.get(old_map.get(name, ""), None)
            old = f"<td>{fmt(old_val)}</td>" if old_val is not None else "<td>—</td>"
        rows_html += f"<tr><td><code>{name}</code></td><td><b>{fmt(val)}</b></td>{old}<td class='muted'>{unit}</td></tr>"

    fit_badges = ""
    for m in results["motors"]:
        fits = [s["fitPercent"] for s in m.get("validation", [])]
        vtxt = f", validation {np.mean(fits):.0f}%" if fits else ""
        fit_badges += (f"<span class='badge'>{m['name']}: R²(qs) {m['r2_quasistatic']:.3f}, "
                       f"R²(dyn) {m['r2_dynamic']:.3f}{vtxt}</span> ")

    spread = c.get("moduleSpreadPercent")
    spread_html = (f"<p>Per-module constant spread: <b>{spread:.1f}%</b></p>"
                   if isinstance(spread, (int, float)) and np.isfinite(spread or np.nan) else "")

    snippet = java_snippet(results, meta)
    gearing = results.get("gearing", {})
    date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

    # The identified gains drive Voltage control requests. Teams using
    # TorqueCurrentFOC requests (output in amps) need the electrical conversion:
    # divide by winding resistance, and drop back-EMF from kV (the FOC drive
    # compensates it), leaving only viscous friction.
    torque_html = ""
    phys = results.get("phys")
    ratio = gearing.get("ratio")
    if phys and ratio:
        r_w = phys["resistance"]
        ke_mech = phys["ke_v_per_rotor_rps"] * ratio
        torque_html = (
            "<p class='muted'>Using <code>TorqueCurrentFOC</code> requests instead of "
            "Voltage requests? Approximate amp-domain starting points: "
            f"kS≈{fmt(c['kS'] / r_w, 3)} A, kV≈{fmt(max(c['kV'] - ke_mech, 0) / r_w, 3)} A/(unit/s), "
            f"kA≈{fmt(c['kA'] / r_w, 3)} A/(unit/s²), kP≈{fmt(pid['kP'] / r_w, 3)}, "
            f"kD≈{fmt(pid['kD'] / r_w, 3)} — verify on the robot.</p>"
        )

    html = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MechID — {results['sessionName']}</title>
<script>{plotly.offline.get_plotlyjs()}</script>
<style>
  body {{ background:{PAGE}; color:{INK}; margin:0;
         font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }}
  .wrap {{ max-width: 1080px; margin: 0 auto; padding: 24px 20px 64px; }}
  h1 {{ font-size: 24px; margin: 8px 0 2px; }}
  h2 {{ font-size: 16px; margin: 0 0 12px; color:{INK}; }}
  .sub {{ color:{INK_2}; margin-bottom: 20px; }}
  .card {{ background:{SURFACE}; border:1px solid rgba(11,11,11,0.10); border-radius:10px;
           padding:18px 20px; margin-bottom:20px; overflow-x:auto; }}
  .card.warn {{ border-left: 4px solid {STATUS_WARNING}; }}
  .card.warn ul {{ margin: 4px 0 0 18px; color:{INK_2}; }}
  table {{ border-collapse: collapse; font-variant-numeric: tabular-nums; }}
  th, td {{ text-align:left; padding: 6px 18px 6px 0; border-bottom: 1px solid {GRID};
            font-size: 14px; }}
  th {{ color:{MUTED}; font-weight:600; }}
  .muted {{ color:{MUTED}; }}
  .badge {{ display:inline-block; background:#eef3fb; color:#1c5cab; border-radius: 999px;
            padding: 3px 10px; font-size: 12px; margin: 2px 4px 2px 0; }}
  pre {{ background:#111; color:#eee; border-radius:8px; padding:14px 16px; overflow-x:auto;
        font-size:13px; line-height:1.5; }}
  button.copy {{ float:right; border:1px solid {GRID}; background:{SURFACE}; color:{INK_2};
                 border-radius:6px; padding:4px 10px; cursor:pointer; font-size:12px; }}
  button.copy:hover {{ background:#f0efec; }}
</style></head><body><div class="wrap">
<h1>MechID — {results['sessionName']}</h1>
<div class="sub">{results['mechanismType']} · {results['pidDomain']} loop ·
{results['nMotors']} motor(s) · engine: <b>{results['engine']}</b> · {date}<br>
{fit_badges}</div>
{warnings_html}
<div class="card">
  <h2>Identified constants (Phoenix 6 units: volts per mechanism {results['positionUnits']})</h2>
  <table>{header}{rows_html}</table>
  {spread_html}
  {torque_html}
  <p class="muted">Gear ratio (rotor : mechanism): {fmt(gearing.get('ratio'))}
  {'(estimated from kV — set withRotorToMechanismRatio for accuracy)' if gearing.get('estimated') else ''}
  · LQR weights: pos err {fmt(c['lqrParams']['max_position_error'])},
  vel err {fmt(c['lqrParams']['max_velocity_error'])},
  effort {fmt(c['lqrParams']['max_effort'])} V</p>
</div>
<div class="card">
  <h2>Paste into your TalonFX config
  <button class="copy" onclick="navigator.clipboard.writeText(document.getElementById('snippet').innerText)">Copy</button></h2>
  <pre id="snippet">{snippet}</pre>
</div>
{figs_html}
<p class="muted">Generated by MechID analysis · session {meta.get('startedAt', '')}</p>
</div>
</body></html>"""

    out_html.write_text(html)
    return out_html


def console_summary(results: dict) -> str:
    c = results["combined"]
    pid = c["pid"]
    mm = c["motionMagic"]
    sweep = results.get("currentSweep") or {}
    rec = (sweep.get("recommended") or {}).get("target")
    lines = [
        f"=== MechID {results['sessionName']} ({results['mechanismType']}, {results['engine']} engine) ===",
        f"  kS = {fmt(c['kS'])} V",
        f"  kV = {fmt(c['kV'])} V/(unit/s)",
        f"  kA = {fmt(c['kA'])} V/(unit/s^2)",
    ]
    if results["gravityModel"] != "NONE":
        lines.append(f"  kG = {fmt(c['kG'])} V ({results['gravityModel']})")
    lines += [
        f"  kP = {fmt(pid['kP'])}   kI = {fmt(pid['kI'])}   kD = {fmt(pid['kD'])}",
        f"  MotionMagic cruise = {fmt(mm['cruiseVelocity'])} unit/s, accel = {fmt(mm['acceleration'])} unit/s^2",
    ]
    if rec is not None:
        lines.append(f"  Stator limit ≈ {rec:.0f} A for ≥{sweep['recommended'].get('targetPercent', 95):.0f}% performance")
    for m in results["motors"]:
        lines.append(f"  [{m['name']}] R2 qs={m['r2_quasistatic']:.3f} dyn={m['r2_dynamic']:.3f}")
    for w in results.get("warnings", []):
        lines.append(f"  WARNING: {w}")
    return "\n".join(lines)
