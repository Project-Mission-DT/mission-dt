"""
E1 of the paper (frame compute time) and an additional publication-rate
measurement that the paper does not report. Raw data is stored as JSON in
results/ (or MDT_RESULTS).

e1    E1, frame compute time: N in {1,2,5,10,25,50,75,100} virtual agents,
      30 s each. Metrics: frame compute time, frame overruns, telemetry
      latency, actuation latency, stale/duplicate updates.
      Output: e1_scalability.json
rate  Publication rate: 10 agents publishing at 8.33 Hz (one of every six
      50 Hz samples) and at 50 Hz (every sample). Metrics: bytes/s on the
      wire, latency, duplicate updates per frame. Output:
      e2_publication_rate.json (not reported in the paper; "e2" is accepted
      as the former name of this option)
all   both (default)

    python experiments/run_experiments.py [e1 | rate | all]
"""
import gc
import json
import statistics as st
import sys
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
from mission_dt.core import MissionDT
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON, RESULT_RATE_FIELD

RES = __import__("os").environ.get("MDT_RESULTS") or str(__import__("pathlib").Path(__file__).resolve().parent.parent / "results")
__import__("os").makedirs(RES, exist_ok=True)


def pctl(v, p):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def run_trial(n_agents, duration=30.0, decimate=True, mix_aerial=True):
    """decimate=True: agents publish at 8.33 Hz; False: every 50 Hz sample."""
    gc.collect()
    seeded = [0]
    dt = MissionDT(on_frame=lambda M: seeded.__setitem__(0, seeded[0] + sum(1 for B in M.B.values() if B.seeded)))
    agents, goals = [], {}
    for i in range(n_agents):
        dom = "aerial" if (mix_aerial and i % 2) else "surface"
        a = VirtualAgent(f"ag{i:03d}", domain=dom,
                         decimate=decimate, duration_s=duration + 2)
        agents.append(a)
        goals[a.aid] = (BASE_LAT + 0.002 * (i % 7 - 3),
                        BASE_LON + 0.002 * (i // 7 - 3),
                        15.0 if dom == "aerial" else 0.0)
    time.sleep(1.0)          # registration settle
    for a in agents:
        a.start()
    dt.run(duration, goals=goals)
    for a in agents:
        a.join(timeout=5)
    lat_t = dt.msg_latencies
    lat_a = [x for a in agents for x in a.act_latencies]
    total_bytes = sum(a.bytes_out for a in agents)
    total_msgs = sum(a.msgs_out for a in agents)
    # rates over the publication window, not over the core run time
    window = max(a.t_last_pub for a in agents) - min(a.t_first_pub for a in agents)
    return {
        "n_agents": n_agents, "duration_s": duration, RESULT_RATE_FIELD: decimate,
        "frames": dt.frames, "overruns": dt.frame_overruns,
        "stale_updates": dt.stale_updates, "dup_updates": dt.dup_updates,
        "frame_ms": {"mean": st.mean(dt.frame_compute) * 1e3,
                     "std": st.pstdev(dt.frame_compute) * 1e3,
                     "p99": pctl(dt.frame_compute, 99) * 1e3,
                     "max": max(dt.frame_compute) * 1e3},
        "telemetry_lat_ms": {"mean": st.mean(lat_t) * 1e3,
                             "std": st.pstdev(lat_t) * 1e3,
                             "p50": pctl(lat_t, 50) * 1e3,
                             "p99": pctl(lat_t, 99) * 1e3,
                             "max": max(lat_t) * 1e3, "n": len(lat_t)},
        "actuation_lat_ms": {"mean": st.mean(lat_a) * 1e3,
                             "std": st.pstdev(lat_a) * 1e3,
                             "p99": pctl(lat_a, 99) * 1e3, "n": len(lat_a)},
        "pub_window_s": window,
        "uplink_Bps": total_bytes / window,
        "uplink_msgs_s": total_msgs / window,
        "seeded_agent_frames": seeded[0],
        "stale_pct": 100.0 * dt.stale_updates / max(1, seeded[0]),
        "release_jitter_ms": {"mean": st.mean(dt.release_jitter) * 1e3,
                              "std": st.pstdev(dt.release_jitter) * 1e3,
                              "p99": pctl(dt.release_jitter, 99) * 1e3,
                              "max": max(dt.release_jitter) * 1e3},
        "core_cpu_s": {"loop": dt.cpu_loop_s, "net": dt.cpu_net_s},
        "core_cpu_pct": 100.0 * (dt.cpu_loop_s + dt.cpu_net_s) / duration,
        "raw_frame_compute_ms": [x * 1e3 for x in dt.frame_compute],
        "raw_release_jitter_ms": [x * 1e3 for x in dt.release_jitter],
        "raw_telemetry_lat_ms": [x * 1e3 for x in lat_t],
        "raw_actuation_lat_ms": [x * 1e3 for x in lat_a],
    }


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"

    if which in ("e1", "all"):
        out = []
        for n in [1, 2, 5, 10, 25, 50, 75, 100]:
            print(f"[E1] N={n} ...", flush=True)
            r = run_trial(n)
            out.append(r); json.dump(out, open(f"{RES}/_partial.json","w"))
            print(f"     frames={r['frames']} overruns={r['overruns']} "
                  f"frame_p99={r['frame_ms']['p99']:.2f}ms "
                  f"jitter_p99={r['release_jitter_ms']['p99']:.2f}ms "
                  f"core_cpu={r['core_cpu_pct']:.1f}% "
                  f"tele_p99={r['telemetry_lat_ms']['p99']:.2f}ms "
                  f"stale={r['stale_updates']} dup={r['dup_updates']}", flush=True)
            time.sleep(2)
        json.dump(out, open(f"{RES}/e1_scalability.json", "w"))

    if which in ("rate", "e2", "all"):
        out = []
        for dec in (True, False):
            print(f"[rate] publication {'8.33' if dec else '50'} Hz ...", flush=True)
            r = run_trial(10, decimate=dec)
            out.append(r); json.dump(out, open(f"{RES}/_partial.json","w"))
            print(f"     uplink={r['uplink_Bps']/1024:.1f} KiB/s "
                  f"({r['uplink_msgs_s']:.0f} msg/s) "
                  f"tele_p99={r['telemetry_lat_ms']['p99']:.2f}ms "
                  f"dup={r['dup_updates']}", flush=True)
            time.sleep(2)
        json.dump(out, open(f"{RES}/e2_publication_rate.json", "w"))
    print("done")
