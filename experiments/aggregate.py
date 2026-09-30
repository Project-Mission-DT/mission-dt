"""
Aggregate the repetitions of the experiment battery (E1..E6).

    python experiments/aggregate.py results/rep*

Each argument is a repetition directory holding the JSON files written by
run_experiments.py (E1, E2), run_e3.py, run_e4.py (with and without
--no-regulator), run_e5.py and run_e6.py. Missing files are skipped. The script writes
summary.json and summary.md into the parent directory of the first argument.

Per configuration and per scalar metric: mean, sample standard deviation
(statistics.stdev, None with one repetition), min, max and the number of
repetitions over the per-repetition values.
Pooled statistics: computed over the raw_* sample arrays (and E4
rows_dom_m_err_hold) concatenated over all repetitions. Percentiles use the
rank floor(pn/100)+1 rule of the producing scripts: sorted[min(n-1, int(p/100*n))].
"""
import json
import math
import os
import statistics as st
import sys
import time

FILES = [
    # (file stem, configuration key)
    ("e1_scalability", "n_agents"),
    ("e2_regulator", "regulator"),
    ("e3_swarm", "n_agents"),
    ("e4_fidelity", "loss"),
    ("e4_fidelity_noreg", "loss"),
    ("e5_resources", "n_agents"),
    ("e6_comparison", ("stack", "n_agents")),
]
# Fields that identify or parametrise a run; they are not metrics.
CONFIG_FIELDS = {"n_agents", "regulator", "loss", "duration_s", "sep_m"}
FRAME_MS = 125.0
HIST_BIN_MS, HIST_MAX_MS = 10.0, 300.0
STREAK_BINS = ["0", "1", "2", ">=3"]
DOMAINS = ["aerial", "surface"]
E6_STACKS = ["mission_dt", "ros2", "gazebo"]      # order of the E6 tables
E6_PROCS = {"mission_dt": ["core", "agents", "broker"], "ros2": ["core", "agents"],
            "gazebo": ["gz_sim"]}


# ---------------------------------------------------------------- helpers
def pctl(v, p):
    """Percentile (sample of rank floor(pn/100)+1) on a sorted list (same rule as the runners)."""
    if not v:
        return None
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))]


def is_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def flatten(rec, prefix=""):
    """Numeric scalar fields of one result record, nested dicts as a.b."""
    out = {}
    for k, v in rec.items():
        if k.startswith("raw_") or k.startswith("rows_"):
            continue
        if not prefix and k in CONFIG_FIELDS:
            continue
        name = prefix + k
        if isinstance(v, dict):
            out.update(flatten(v, name + "."))
        elif is_num(v):
            out[name] = v
    return out


def rep_stats(values):
    """Statistics over the per-repetition values of one metric."""
    v = [x for x in values if is_num(x)]
    if not v:
        return None
    return {"mean": st.mean(v),
            "std": st.stdev(v) if len(v) > 1 else None,
            "min": min(v), "max": max(v), "n": len(v)}


def pooled_stats(samples, pcts=(50, 99)):
    """Pooled mean, population std, percentiles, max and n over raw samples."""
    s = sorted(x for x in samples if is_num(x))
    if not s:
        return None
    out = {"mean": st.fmean(s), "std": st.pstdev(s)}
    for p in pcts:
        out[f"p{p}"] = pctl(s, p)
    out["max"] = s[-1]
    out["n"] = len(s)
    return out


def rmse(v):
    return math.sqrt(sum(e * e for e in v) / len(v)) if v else None


def streak_bin(m):
    return str(m) if m < 3 else ">=3"


def config_label(key, val):
    if key == "regulator":
        return "ON" if val else "OFF"
    return val


# ---------------------------------------------------------------- loading
def load_reps(dirs):
    """{stem: {rep_label: [records]}}; missing or unreadable files skipped."""
    data, found = {}, {}
    for d in dirs:
        label = os.path.basename(os.path.normpath(d))
        found[label] = []
        for stem, _ in FILES:
            fn = os.path.join(d, stem + ".json")
            if not os.path.isfile(fn):
                continue
            try:
                with open(fn) as f:
                    recs = json.load(f)
            except (OSError, ValueError) as e:
                print(f"warning: skipping {fn}: {e}", file=sys.stderr)
                continue
            if isinstance(recs, dict):
                recs = [recs]
            data.setdefault(stem, {})[label] = recs
            found[label].append(stem)
    return data, found


def group_by_config(per_rep, key):
    """{config value: {rep_label: record}} keeping configuration order."""
    groups = {}
    for rep, recs in per_rep.items():
        for r in recs:
            v = tuple(r.get(k) for k in key) if isinstance(key, tuple) else r.get(key)
            groups.setdefault(v, {})[rep] = r
    if isinstance(key, tuple):       # E6: (stack, N), stacks in E6_STACKS order
        order = sorted(groups, key=lambda v: (
            E6_STACKS.index(v[0]) if v[0] in E6_STACKS else len(E6_STACKS), str(v[0]), v[1:]))
        return [(v, groups[v]) for v in order]
    try:
        order = sorted(groups, key=lambda v: (v is None, v))
    except TypeError:
        order = list(groups)
    if key == "regulator":           # ON first, as in the runner
        order = sorted(groups, key=lambda v: not v)
    return [(v, groups[v]) for v in order]


def per_rep_metrics(recs_by_rep):
    flat = {rep: flatten(r) for rep, r in recs_by_rep.items()}
    names = []
    for f in flat.values():
        for k in f:
            if k not in names:
                names.append(k)
    metrics = {}
    for k in names:
        s = rep_stats([f.get(k) for f in flat.values()])
        if s is not None:
            metrics[k] = s
    return metrics


def pool(recs_by_rep, key):
    out = []
    for r in recs_by_rep.values():
        v = r.get(key)
        if isinstance(v, list):
            out.extend(v)
    return out


# ---------------------------------------------------------------- per experiment
def summarise_timing(recs_by_rep):
    """Pooled statistics for E1/E2 records."""
    p = {}
    for name, key, pcts in [
            ("frame_ms", "raw_frame_compute_ms", (50, 99)),
            ("telemetry_lat_ms", "raw_telemetry_lat_ms", (50, 99)),
            ("actuation_lat_ms", "raw_actuation_lat_ms", (50, 99)),
            ("release_jitter_ms", "raw_release_jitter_ms", (50, 99))]:
        s = pooled_stats(pool(recs_by_rep, key), pcts)
        if s is not None:
            p[name] = s
    fr = [r.get("frames") for r in recs_by_rep.values() if is_num(r.get("frames"))]
    ov = [r.get("overruns") for r in recs_by_rep.values() if is_num(r.get("overruns"))]
    p["frames_total"] = sum(fr) if fr else None
    p["overruns_total"] = sum(ov) if ov else None
    return p


def summarise_e3(recs_by_rep):
    p = {}
    lat = sorted(x for x in pool(recs_by_rep, "raw_swarm_lat_ms") if is_num(x))
    s = pooled_stats(lat, (50, 95, 99))
    if s is not None:
        n = len(lat)
        s["frac_le_1frame"] = sum(1 for x in lat if x <= FRAME_MS) / n
        s["frac_le_2frames"] = sum(1 for x in lat if x <= 2 * FRAME_MS) / n
        nb = int(round(HIST_MAX_MS / HIST_BIN_MS))
        counts = [0] * nb
        over = under = 0
        for x in lat:
            if x < 0:
                under += 1
            elif x >= HIST_MAX_MS:
                over += 1
            else:
                counts[min(nb - 1, int(x // HIST_BIN_MS))] += 1
        p["swarm_lat_hist"] = {
            "bin_ms": HIST_BIN_MS,
            "edges_ms": [i * HIST_BIN_MS for i in range(nb + 1)],
            "counts": counts, "below_0": under, "at_or_above_max": over}
        p["swarm_lat_ms"] = s
    for name, key in [("frame_ms", "raw_frame_compute_ms"),
                      ("release_jitter_ms", "raw_release_jitter_ms")]:
        s = pooled_stats(pool(recs_by_rep, key), (50, 99))
        if s is not None:
            p[name] = s
    fr = [r.get("frames") for r in recs_by_rep.values() if is_num(r.get("frames"))]
    ov = [r.get("overruns") for r in recs_by_rep.values() if is_num(r.get("overruns"))]
    p["frames_total"] = sum(fr) if fr else None
    p["overruns_total"] = sum(ov) if ov else None
    return p


def summarise_e4(recs_by_rep):
    rows = [r for r in pool(recs_by_rep, "rows_dom_m_err_hold")
            if isinstance(r, (list, tuple)) and len(r) >= 4]
    p = {}
    if rows:
        err = sorted(r[2] for r in rows)
        stale = [r[2] for r in rows if r[1] >= 1]
        p["pos_rmse_m"] = rmse(err)
        p["pos_p99_m"] = pctl(err, 99)
        p["pos_max_m"] = err[-1]
        p["pos_max_stale_m"] = max(stale) if stale else 0.0
        p["samples"] = len(err)
        p["stale_samples"] = len(stale)
        by = {}
        for dom in sorted(set(r[0] for r in rows) | set(DOMAINS)):
            by[dom] = {}
            for b in STREAK_BINS:
                sel = [r for r in rows if r[0] == dom and streak_bin(r[1]) == b]
                by[dom][b] = {"count": len(sel),
                              "dr_rmse_m": rmse([r[2] for r in sel]),
                              "hold_rmse_m": rmse([r[3] for r in sel])}
        p["by_domain_streak"] = by
    else:
        # no raw rows: fall back on the per-repetition maximum
        ms = [r.get("pos_max_stale_m") for r in recs_by_rep.values()
              if is_num(r.get("pos_max_stale_m"))]
        if ms:
            p["pos_max_stale_m"] = max(ms)
    return p


def ratio_off_on(configs_raw):
    """E2: per-repetition ratio OFF/ON of uplink_Bps and dup_updates."""
    on = dict(configs_raw).get(True, {})
    off = dict(configs_raw).get(False, {})
    out = {}
    for metric in ("uplink_Bps", "dup_updates"):
        per = {}
        for rep in sorted(set(on) & set(off)):
            a, b = on[rep].get(metric), off[rep].get(metric)
            if is_num(a) and is_num(b) and a != 0:
                per[rep] = b / a
        v = list(per.values())
        out[metric] = {"per_rep": per,
                       "mean": st.mean(v) if v else None,
                       "std": st.stdev(v) if len(v) > 1 else None,
                       "n": len(v)}
    return out


def summarise(data):
    summary = {}
    for stem, key in FILES:
        if stem not in data:
            continue
        grouped = group_by_config(data[stem], key)
        configs = []
        for val, recs in grouped:
            c = dict(zip(key, val)) if isinstance(key, tuple) else {key: val}
            c.update({"reps": sorted(recs), "n_reps": len(recs),
                      "metrics": per_rep_metrics(recs)})
            if stem in ("e1_scalability", "e2_regulator"):
                c["pooled"] = summarise_timing(recs)
            elif stem == "e3_swarm":
                c["pooled"] = summarise_e3(recs)
            elif stem.startswith("e4_"):
                c["pooled"] = summarise_e4(recs)
            configs.append(c)
        entry = {"config_key": ",".join(key) if isinstance(key, tuple) else key,
                 "configs": configs}
        if stem == "e2_regulator":
            entry["ratio_off_on"] = ratio_off_on(grouped)
        summary[stem] = entry
    return summary


# ---------------------------------------------------------------- markdown
def f(x, nd=2):
    if x is None:
        return "–"
    if isinstance(x, int) and not isinstance(x, bool):
        return str(x)
    return f"{x:.{nd}f}"


def ms(metrics, name, nd=2, scale=1.0):
    """'mean ± std' of one per-repetition metric."""
    m = metrics.get(name)
    if m is None:
        return "–"
    s = f(m["mean"] * scale, nd)
    if m["std"] is not None:
        s += " ± " + f(m["std"] * scale, nd)
    return s


def g(d, *path):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def table(head, rows):
    out = ["| " + " | ".join(head) + " |",
           "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return out


def render_md(summary, meta):
    L = [f"# Experiment summary ({meta['n_reps']} repetitions)", "",
         f"Repetitions: {', '.join(meta['reps'])}. Cells show mean ± sample "
         "standard deviation over repetitions; columns marked *pooled* use all "
         "samples of all repetitions (percentile = sample of rank floor(pn/100)+1). Times in ms.", ""]

    e = summary.get("e1_scalability")
    if e:
        L += ["## E1 Scalability", ""]
        rows = []
        for c in e["configs"]:
            m, p = c["metrics"], c["pooled"]
            rows.append([c["n_agents"], c["n_reps"],
                         f"{f(p.get('overruns_total'))}/{f(p.get('frames_total'))}",
                         ms(m, "frame_ms.mean", 3), ms(m, "frame_ms.p99"),
                         f(g(p, "frame_ms", "p99")), f(g(p, "frame_ms", "max")),
                         f(g(p, "release_jitter_ms", "p99")),
                         ms(m, "telemetry_lat_ms.p99"), f(g(p, "telemetry_lat_ms", "p99")),
                         ms(m, "actuation_lat_ms.p99"), f(g(p, "actuation_lat_ms", "p99")),
                         ms(m, "stale_pct"), ms(m, "dup_updates", 0),
                         ms(m, "uplink_Bps", 1, 1 / 1024), ms(m, "core_cpu_pct", 1)])
        L += table(["N", "reps", "overruns/frames", "frame mean", "frame p99",
                    "frame p99 pooled", "frame max pooled", "jitter p99 pooled",
                    "tele p99", "tele p99 pooled", "act p99", "act p99 pooled",
                    "stale %", "dup", "uplink KiB/s", "core CPU %"], rows)
        L.append("")

    e = summary.get("e2_regulator")
    if e:
        L += ["## E2 Bandwidth regulator (N = 10)", ""]
        rows = []
        for c in e["configs"]:
            m, p = c["metrics"], c["pooled"]
            rows.append([config_label("regulator", c["regulator"]), c["n_reps"],
                         ms(m, "uplink_Bps", 1, 1 / 1024), ms(m, "uplink_msgs_s", 1),
                         ms(m, "dup_updates", 0), ms(m, "stale_pct"),
                         ms(m, "telemetry_lat_ms.p99"), f(g(p, "telemetry_lat_ms", "p99")),
                         ms(m, "frame_ms.p99"), f(g(p, "frame_ms", "p99"))])
        L += table(["regulator", "reps", "uplink KiB/s", "msgs/s", "dup", "stale %",
                    "tele p99", "tele p99 pooled", "frame p99", "frame p99 pooled"], rows)
        r = e["ratio_off_on"]
        L += ["", "| ratio OFF/ON | mean | std | reps |", "|---|---|---|---|"]
        for k in ("uplink_Bps", "dup_updates"):
            L.append(f"| {k} | {f(r[k]['mean'])} | {f(r[k]['std'])} | {r[k]['n']} |")
        L.append("")

    e = summary.get("e3_swarm")
    if e:
        L += ["## E3 Swarm-reaction latency", ""]
        rows = []
        for c in e["configs"]:
            m, p = c["metrics"], c["pooled"]
            s = p.get("swarm_lat_ms") or {}
            rows.append([c["n_agents"], c["n_reps"], ms(m, "avoid_events", 0),
                         ms(m, "swarm_lat_ms.p50", 1), ms(m, "swarm_lat_ms.p99", 1),
                         f(s.get("mean"), 1), f(s.get("p50"), 1), f(s.get("p95"), 1),
                         f(s.get("p99"), 1), f(s.get("max"), 1), f(s.get("n")),
                         f(100 * s["frac_le_1frame"], 1) if "frac_le_1frame" in s else "–",
                         f(100 * s["frac_le_2frames"], 1) if "frac_le_2frames" in s else "–",
                         f(g(p, "frame_ms", "p99")),
                         f"{f(p.get('overruns_total'))}/{f(p.get('frames_total'))}"])
        L += table(["N", "reps", "events", "p50", "p99", "mean pooled", "p50 pooled",
                    "p95 pooled", "p99 pooled", "max pooled", "n pooled",
                    "% ≤125 ms", "% ≤250 ms", "frame p99 pooled", "overruns/frames"], rows)
        L.append("")

    for stem, title in (("e4_fidelity", "regulator ON"),
                        ("e4_fidelity_noreg", "regulator OFF")):
        e = summary.get(stem)
        if not e:
            continue
        L += [f"## E4 Twin fidelity under packet loss ({title})", ""]
        rows, srows = [], []
        for c in e["configs"]:
            m, p = c["metrics"], c["pooled"]
            rows.append([f"{100 * c['loss']:.0f} %", c["n_reps"],
                         ms(m, "lost_msgs", 0), ms(m, "stale_pct"),
                         ms(m, "pos_rmse_m", 3), f(p.get("pos_rmse_m"), 3),
                         ms(m, "pos_p99_m", 3), f(p.get("pos_p99_m"), 3),
                         f(p.get("pos_max_stale_m"), 3),
                         ms(m, "hdg_rmse_deg"), ms(m, "hdg_p99_deg")])
            for dom, bins in (p.get("by_domain_streak") or {}).items():
                for b in STREAK_BINS:
                    x = bins[b]
                    if x["count"]:
                        srows.append([f"{100 * c['loss']:.0f} %", dom, b, x["count"],
                                      f(x["dr_rmse_m"], 3), f(x["hold_rmse_m"], 3)])
        L += table(["loss", "reps", "lost msgs", "stale %", "pos RMSE m",
                    "pos RMSE pooled", "pos p99 m", "pos p99 pooled",
                    "max stale err pooled", "hdg RMSE deg", "hdg p99 deg"], rows)
        if srows:
            L += ["", "Pooled error by domain and stale streak m "
                  "(dead reckoning vs. hold of the last received state):", ""]
            L += table(["loss", "domain", "m", "count", "DR RMSE m", "hold RMSE m"], srows)
        L.append("")

    e = summary.get("e5_resources")
    if e:
        L += ["## E5 Resource footprint of the Mission-DT process", ""]
        rows = [[c["n_agents"], c["n_reps"], ms(c["metrics"], "cpu_pct", 1),
                 ms(c["metrics"], "peak_rss_mib", 1), ms(c["metrics"], "frames", 0),
                 ms(c["metrics"], "overruns", 0)] for c in e["configs"]]
        L += table(["N", "reps", "CPU % of one core", "peak RSS MiB", "frames",
                    "overruns"], rows)
        L.append("")

    e = summary.get("e6_comparison")
    if e:
        L += ["## E6 Resource comparison: Mission-DT (MQTT), ROS 2, Gazebo", "",
              "Per process, over the measured window: CPU = (utime + stime) / wall "
              "in % of one core; peak RSS = VmHWM; RSS end = VmRSS at the end of the "
              "window. All processes on core 0. *total* sums the processes of the stack.", ""]
        rows = []
        for c in e["configs"]:
            m = c["metrics"]
            for proc in E6_PROCS.get(c["stack"], []) + ["total"]:
                pre = "total." if proc == "total" else f"processes.{proc}."
                rows.append([c["stack"], c["n_agents"], c["n_reps"], proc,
                             ms(m, pre + "cpu_pct", 1), ms(m, pre + "peak_rss_mib", 1),
                             ms(m, pre + "rss_end_mib", 1)])
        L += table(["stack", "N", "reps", "process", "CPU % of one core",
                    "peak RSS MiB", "RSS end MiB"], rows)
        rows = []
        for c in e["configs"]:
            if c["stack"] == "gazebo":
                continue
            m = c["metrics"]
            rows.append([c["stack"], c["n_agents"], c["n_reps"],
                         f"{f(g(m, 'mission.overruns', 'max'), 0)}/"
                         f"{f(g(m, 'mission.frames', 'mean'), 0)}",
                         ms(m, "mission.frame_ms.mean", 2), ms(m, "mission.frame_ms.max", 2),
                         ms(m, "mission.telemetry_agents_received", 0),
                         ms(m, "mission.telemetry_min_per_agent", 0),
                         ms(m, "mission.telemetry_rate_hz_per_agent", 2),
                         ms(m, "mission.telemetry_lat_ms.p50", 2),
                         ms(m, "mission.telemetry_lat_ms.p99", 2),
                         ms(m, "agents.actuation_min_per_agent", 0),
                         ms(m, "agents.actuation_lat_ms.p99", 2)])
        if rows:
            L += ["", "Mission layer in the window (ms):", ""]
            L += table(["stack", "N", "reps", "max overruns/mean frames", "frame mean", "frame max",
                        "agents received", "min msgs per agent", "telemetry Hz per agent",
                        "tele p50", "tele p99", "min act. per agent", "act p99"], rows)
        rows = []
        for c in e["configs"]:
            if c["stack"] != "gazebo":
                continue
            m = c["metrics"]
            rows.append([c["n_agents"], c["n_reps"], ms(m, "gazebo.rtf", 3),
                         ms(m, "gazebo.rtf_stats_min", 3), ms(m, "gazebo.steps_per_s", 0),
                         ms(m, "gazebo.odom_models_received", 0),
                         ms(m, "gazebo.odom_rate_hz_per_model", 2),
                         ms(m, "monitor.cpu_pct", 1)])
        if rows:
            L += ["", "Gazebo real-time factor over the window (sim time / wall time; "
                  "1 ms physics step, target 1.0) and odometry received by the monitor:", ""]
            L += table(["N", "reps", "RTF", "min RTF (stats msg)", "steps/s",
                        "models received", "odom Hz per model (wall)", "monitor CPU %"], rows)
        L.append("")
    return "\n".join(L)


# ---------------------------------------------------------------- main
def main(argv):
    dirs = [d for d in argv if os.path.isdir(d)]
    if not dirs:
        print("usage: python experiments/aggregate.py results/rep1 [results/rep2 ...]",
              file=sys.stderr)
        return 2
    outdir = os.path.dirname(os.path.abspath(os.path.normpath(dirs[0])))
    data, found = load_reps(dirs)
    reps = [os.path.basename(os.path.normpath(d)) for d in dirs if found.get(
        os.path.basename(os.path.normpath(d)))]
    meta = {"reps": reps, "n_reps": len(reps), "rep_dirs": [os.path.relpath(os.path.abspath(d), outdir) for d in dirs],
            "empty_rep_dirs": [k for k, v in found.items() if not v],
            "files_found": found,
            "generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "percentile_rule": "rank floor(pn/100)+1: sorted[min(n-1, int(p/100*n))]",
            "rep_std": "sample standard deviation over repetitions (None with 1 rep)",
            "pooled_std": "population standard deviation over pooled samples"}
    summary = {"meta": meta}
    summary.update(summarise(data))
    with open(os.path.join(outdir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    with open(os.path.join(outdir, "summary.md"), "w") as fh:
        fh.write(render_md(summary, meta) + "\n")
    print(f"wrote {outdir}/summary.json and {outdir}/summary.md "
          f"({len(reps)} repetitions; experiments: "
          f"{', '.join(k for k in summary if k != 'meta') or 'none'})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
