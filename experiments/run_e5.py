"""
E4 of the paper -- Resource footprint of the Mission-DT process.

The mission core runs in its own process and the N virtual agents run in a
second process, so the CPU time and memory measured belong to the Mission-DT
alone. Both processes use the MQTT broker on 127.0.0.1:1883.

Measured in the Mission-DT process during the 30 s run:
    CPU utilisation = (user + system CPU time) / wall time, in % of one core
    peak resident memory (VmHWM on Linux, ru_maxrss elsewhere), in MiB
Baseline: the same process with the core connected and no agent.

    python experiments/run_e5.py [N ...]          # default: 0 10 50 100
"""
import json
import os
import resource
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
RES = os.environ.get("MDT_RESULTS") or str(ROOT / "results")
DURATION = float(os.environ.get("MDT_E5_DURATION", "30"))


def goal(i, dom):
    from mission_dt.agents import BASE_LAT, BASE_LON
    return (BASE_LAT + 0.002 * (i % 7 - 3), BASE_LON + 0.002 * (i // 7 - 3),
            15.0 if dom == "aerial" else 0.0)


def peak_rss_mib():
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmHWM:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / (1024.0 * 1024.0) if sys.platform == "darwin" else r / 1024.0


def child_core(n):
    from mission_dt.core import MissionDT
    dt = MissionDT()
    goals = {f"rs{i:03d}": goal(i, "aerial" if i % 2 else "surface") for i in range(n)}
    print("READY", flush=True)
    sys.stdin.readline()                     # wait for the agents
    c0, w0 = os.times(), time.monotonic()
    dt.run(DURATION, goals=goals)
    c1, w1 = os.times(), time.monotonic()
    cpu = (c1.user - c0.user) + (c1.system - c0.system)
    print(json.dumps({"n_agents": n, "wall_s": w1 - w0, "cpu_s": cpu,
                      "cpu_pct": 100.0 * cpu / (w1 - w0),
                      "peak_rss_mib": peak_rss_mib(),
                      "frames": dt.frames, "overruns": dt.frame_overruns}), flush=True)


def child_agents(n):
    from mission_dt.agents import VirtualAgent
    agents = [VirtualAgent(f"rs{i:03d}", domain="aerial" if i % 2 else "surface",
                           duration_s=DURATION + 2) for i in range(n)]
    print("READY", flush=True)
    sys.stdin.readline()
    for a in agents:
        a.start()
    for a in agents:
        a.join(timeout=DURATION + 10)
    print("DONE", flush=True)


def trial(n):
    py = sys.executable
    me = str(Path(__file__).resolve())
    core = subprocess.Popen([py, me, "--core", str(n)], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True)
    assert core.stdout.readline().strip() == "READY"
    ag = None
    if n > 0:
        ag = subprocess.Popen([py, me, "--agents", str(n)], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, text=True)
        assert ag.stdout.readline().strip() == "READY"
    time.sleep(1.0)                          # registration settle
    if ag:
        ag.stdin.write("go\n"); ag.stdin.flush()
    core.stdin.write("go\n"); core.stdin.flush()
    r = json.loads(core.stdout.readline())
    core.wait()
    if ag:
        ag.wait()
    return r


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--core":
        child_core(int(sys.argv[2])); sys.exit(0)
    if len(sys.argv) > 2 and sys.argv[1] == "--agents":
        child_agents(int(sys.argv[2])); sys.exit(0)
    sizes = [int(x) for x in sys.argv[1:]] or [0, 10, 50, 100]
    os.makedirs(RES, exist_ok=True)
    out = []
    for n in sizes:
        print(f"[E4] N={n} ...", flush=True)
        r = trial(n)
        out.append(r)
        json.dump(out, open(f"{RES}/e5_resources.json", "w"))
        print(f"     cpu={r['cpu_pct']:.1f}% of one core  peak RSS={r['peak_rss_mib']:.1f} MiB "
              f"overruns={r['overruns']}", flush=True)
        time.sleep(2)
    print("done")
