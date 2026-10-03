"""Values and PNG versions of Figs. 3, 5 and 6 of the paper.

Reads the five runs in results/rep1..rep5 (and results/e3_50hz_n50_extra/run1..run5
for E2 at 50 Hz and N = 50), writes doc/figures/paper_figures.json with the plotted
values and doc/figures/fig3_frame_time.png, fig5_swarm_latency.png and
fig6_dead_reckoning.png. The paper draws the same values with pgfplots.

Percentile rule of the paper: sorted[min(n - 1, int(p / 100 * n))].

Usage:  python experiments/paper_figures.py [results_dir] [output_dir]
"""
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "results")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "doc", "figures")
REPS = [f"rep{i}" for i in range(1, 6)]
TF = 125.0
BLUE, ORANGE, GRAY = "#3D85C6", "#E69138", "#D9D9D9"

plt.rcParams.update({"font.size": 9, "font.family": "serif", "axes.grid": True,
                     "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False})


def load(name):
    return [json.load(open(os.path.join(R, r, name))) for r in REPS]


def pct(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100 * len(s)))]


def mci(v):
    """Mean and half-width of the 95% confidence interval (1.96 s / sqrt(n))."""
    return st.mean(v), 1.96 * st.stdev(v) / math.sqrt(len(v))


# ---------------------------------------------------------------- Fig. 3 (E1)
NS1 = [1, 2, 5, 10, 25, 50, 75, 100]
e1 = load("e1_scalability.json")
fig3 = {}
for i, n in enumerate(NS1):
    assert all(rep[i]["n_agents"] == n for rep in e1)
    pooled = sum((rep[i]["raw_frame_compute_ms"] for rep in e1), [])
    fig3[n] = {"frames": len(pooled), "mean": st.mean(pooled), "p99": pct(pooled, 99),
               "max": max(pooled), "overruns": sum(rep[i]["overruns"] for rep in e1)}

# ---------------------------------------------------------------- Fig. 5 (E2)
e3 = load("e3_swarm.json")
e3n = load("e3_swarm_50hz.json")
extra_dir = os.path.join(R, "e3_50hz_n50_extra")
extra = [json.load(open(os.path.join(extra_dir, f"run{k}", "e3_swarm_50hz.json")))[0]
         for k in range(1, 6)] if os.path.isdir(extra_dir) else []
fig5 = {}
for rate, src in (("8.33 Hz", e3), ("50 Hz", e3n)):
    for i, n in enumerate([10, 25, 50]):
        runs = [rep[i] for rep in src]
        assert all(r["n_agents"] == n for r in runs)
        if rate == "50 Hz" and n == 50:
            runs += extra
        per_run = [r["raw_swarm_lat_ms"] for r in runs]
        pooled = sum(per_run, [])
        fig5[f"{rate}, N={n}"] = {
            "rate": rate, "n_agents": n, "runs": len(per_run), "commands": len(pooled),
            "p5": pct(pooled, 5), "p50": pct(pooled, 50), "p95": pct(pooled, 95),
            "max": max(pooled), "within_Tf_pct": 100 * sum(x <= TF for x in pooled) / len(pooled),
            "diverged_runs": sum(max(r) > 1000 for r in per_run),
            "max_per_run": [round(max(r), 1) for r in per_run]}

# ---------------------------------------------------------------- Fig. 6 (E3)
cells = defaultdict(list)
for rep in load("e4_fidelity.json"):
    for row in rep:
        for dom, m, err_dr, err_hold in row["rows_dom_m_err_hold"]:
            cells[(dom, min(m, 3))].append((err_dr, err_hold))
fig6 = {}
for dom in ("aerial", "surface"):
    for m in (0, 1, 2, 3):
        c = cells[(dom, m)]
        if len(c) < 2:
            continue
        fig6[f"{dom}, m={m if m < 3 else '>=3'}"] = {
            "domain": dom, "m": m, "agent_frames": len(c),
            "dead_reckoning": mci([a for a, _ in c]), "last_state": mci([b for _, b in c])}

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "paper_figures.json"), "w") as f:
    json.dump({"fig3_frame_time_ms": fig3, "fig5_swarm_latency_ms": fig5,
               "fig6_mean_position_error_m": fig6}, f, indent=1)

# ---------------------------------------------------------------- plots
x = list(range(len(NS1)))
fig, ax = plt.subplots(figsize=(6.0, 2.8))
mx = [fig3[n]["max"] for n in NS1]
ax.fill_between(x, mx, TF, color=GRAY, alpha=0.6, lw=0, label="headroom to $T_f$")
ax.plot(x, [fig3[n]["p99"] for n in NS1], "-o", color=BLUE, ms=4, label="p99")
ax.plot(x, mx, ":^", color="black", ms=4, label="max")
ax.axhline(TF, ls="--", color="black", lw=0.8)
ax.text(len(NS1) - 1, TF - 3, "$T_f$ = 125 ms", ha="right", va="top", fontsize=8)
ax.set_xticks(x, [str(n) for n in NS1])
ax.set_ylim(0, 140)
ax.set_xlabel("Fleet size N (agents)")
ax.set_ylabel("Frame time (ms)")
ax.legend(loc="center left", bbox_to_anchor=(0.01, 0.55), fontsize=8, frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig3_frame_time.png"), dpi=200)
plt.close(fig)

fig, ax = plt.subplots(figsize=(6.0, 2.8))
for k, (rate, col, mk, dx) in enumerate((("8.33 Hz", BLUE, "o", -0.12), ("50 Hz", ORANGE, "s", 0.12))):
    for j, n in enumerate([10, 25, 50]):
        s = fig5[f"{rate}, N={n}"]
        top = min(s["p95"], 160)
        ax.plot([j + dx, j + dx], [s["p5"], top], color=col, lw=3, solid_capstyle="butt")
        if s["p95"] > 160:
            ax.annotate("", xy=(j + dx, 162), xytext=(j + dx, 150),
                        arrowprops={"arrowstyle": "-|>", "color": col, "lw": 1.5})
            ax.text(j + dx + 0.06, 100, f"p95\n{s['p95'] / 1000:.1f} s", fontsize=7, va="center")
            ax.text(j + dx - 0.06, 150, f"{s['diverged_runs']} of {s['runs']} runs diverged",
                    fontsize=7, ha="right", va="center")
        ax.plot(j + dx, s["p50"], mk, color=col, mec="black", mew=0.5, ms=6,
                label=rate if j == 0 else None)
ax.axhline(TF, ls="--", color="black", lw=0.8)
ax.text(0.5, TF + 2, "$T_f$ = 125 ms", fontsize=8, ha="center", va="bottom")
ax.set_xticks(range(3), ["10", "25", "50"])
ax.set_xlim(-0.5, 2.5)
ax.set_ylim(0, 165)
ax.set_xlabel("Fleet size N (agents)")
ax.set_ylabel("Reaction latency (ms)")
ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2, fontsize=8, frameon=False,
          title="marker: median; bar: 5th to 95th percentile", title_fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig5_swarm_latency.png"), dpi=200)
plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(6.0, 2.6))
for ax, dom, ymax in ((axes[0], "aerial", 4.0), (axes[1], "surface", 0.7)):
    for key, col, mk, ls, dx, lab in (("dead_reckoning", BLUE, "o", "-", -0.05, "dead reckoning, Eq. (2)"),
                                      ("last_state", ORANGE, "s", "--", 0.05, "last received state")):
        pts = [fig6[f"{dom}, m={m}"][key] for m in (0, 1, 2)]
        ax.errorbar([m + dx for m in (0, 1, 2)], [p[0] for p in pts], yerr=[p[1] for p in pts],
                    fmt=mk + ls, color=col, ms=4, capsize=2, ecolor="black", elinewidth=0.6, label=lab)
    ax.set_title(f"{dom.capitalize()} agents", fontsize=9)
    ax.set_xticks([0, 1, 2])
    ax.set_ylim(0, ymax)
    ax.set_xlabel("Consecutive stale frames m")
axes[0].set_ylabel("Mean position error (m)")
axes[0].legend(loc="upper left", fontsize=7, frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig6_dead_reckoning.png"), dpi=200)
plt.close(fig)

print(f"wrote {OUT}/paper_figures.json and three PNG files")
