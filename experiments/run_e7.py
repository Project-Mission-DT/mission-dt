"""
E6 of the paper -- Hybrid fleet: ArduRover SITL boats and virtual agents under
one mission.

Scenario (the E2 scenario with surface agents only): N agents start on a
circle of radius 40 m and receive antipodal goals, so every agent crosses
the center and the separation rule (d_sep = 12 m) fires between agents of
both kinds. N_SITL agents are physical agents: an ArduRover SITL instance
(motorboat model, configs/sitl_boat.parm) with the MAVLink-to-MQTT adapter
(mission_dt/mavlink_adapter.py). The adapters and the virtual agents connect
to the single ground-station broker.

CPU placement (2 vCPU): core 0 = ground station (broker, mission core and the
virtual agents, as in E1 to E5); core 1 = vehicle side (SITL instances and
adapters).

Measured per run:
  telemetry latency (t_pub at the agent or adapter -> core receive), by kind
  actuation latency (core publish -> agent or adapter receive), by kind
  swarm-reaction latency (t_pub of the triggering telemetry -> corrective
    command at the neighbor), by the kinds of the receiver and the trigger
  commands passed to the autopilot and commands discarded (older than T_f)
  age of the MAVLink position sample at publication, frame time and overruns

Usage:  python experiments/run_e7.py [n_sitl] [n_virtual]      (default 2 8)
Env:    MDT_ARDUROVER  path of the ArduRover SITL binary (default: ardurover)
        MDT_RESULTS    results directory (default results/); writes e7_hybrid.json
"""
import json
import math
import os
import shutil
import signal
import statistics as st
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import paho.mqtt.client as mqtt                                   # noqa: E402
from mission_dt.core import MissionDT                             # noqa: E402
from mission_dt.agents import VirtualAgent, BASE_LAT, BASE_LON    # noqa: E402

RES = os.environ.get("MDT_RESULTS") or str(ROOT / "results")
SITL = os.path.abspath(os.environ.get("MDT_ARDUROVER") or shutil.which("ardurover") or "ardurover")
DURATION, RADIUS, SEP = 40.0, 40.0, 12.0


def pctl(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))] if s else None


def summ(v):
    return {"n": len(v), "mean": st.mean(v) if v else None, "p50": pctl(v, 50),
            "p95": pctl(v, 95), "p99": pctl(v, 99), "max": max(v) if v else None}


def cores():
    try:
        c = sorted(os.sched_getaffinity(0))
    except AttributeError:
        return None, None
    return (c[0], c[1]) if len(c) > 1 else (c[0], c[0])


def pinned(cmd, core):
    return (["taskset", "-c", str(core)] + cmd) if core is not None and shutil.which("taskset") else cmd


class CoreE7(MissionDT):
    """Mission core that also keeps the telemetry latency per agent."""
    def __init__(self, **kw):
        self.lat_by = defaultdict(list)
        self.rx_by = defaultdict(int)
        super().__init__(**kw)

    def _on_msg(self, cli, ud, msg):
        if msg.topic.endswith("/telemetry"):
            now = time.time()
            try:
                aid = msg.topic.split("/")[2]
                self.lat_by[aid].append(now - json.loads(msg.payload)["t_pub"])
                self.rx_by[aid] += 1
            except (ValueError, KeyError):
                pass
        super()._on_msg(cli, ud, msg)


class AgentE7(VirtualAgent):
    """Virtual agent that also keeps the id of the agent that triggered each
    corrective command."""
    def __init__(self, *a, **kw):
        self.swarm_by = []
        super().__init__(*a, **kw)

    def _on_act(self, cli, ud, msg):
        now = time.time()
        a = json.loads(msg.payload)
        if a.get("avoid") and a.get("trig_t"):
            self.swarm_by.append((a.get("trig_id"), now - a["trig_t"]))
        super()._on_act(cli, ud, msg)


def broker_conf(path, port):
    Path(path).write_text("\n".join([f"listener {port} 127.0.0.1", "allow_anonymous true",
                                      "set_tcp_nodelay true"]) + "\n")

def wait_port(port, timeout=10.0):
    import socket
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            socket.create_connection(("127.0.0.1", port), 0.5).close()
            return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError(f"port {port} not open")


def run_e7(n_sitl=2, n_virtual=8):
    c_gnd, c_veh = cores()
    if c_gnd is not None:
        os.sched_setaffinity(0, {c_gnd})
    tmp = Path(tempfile.mkdtemp(prefix="e7_"))
    procs = []
    n = n_sitl + n_virtual
    slots = [round(i * n / n_sitl) for i in range(n_sitl)]      # SITL slots spread on the circle
    pos, goals, kind = {}, {}, {}
    for i in range(n):
        th = 2 * math.pi * i / n
        dlat = RADIUS * math.cos(th) / 111_320.0
        dlon = RADIUS * math.sin(th) / (111_320.0 * math.cos(math.radians(BASE_LAT)))
        aid = f"boat{slots.index(i) + 1}" if i in slots else f"v{i:02d}"
        kind[aid] = "physical" if i in slots else "virtual"
        pos[aid] = (BASE_LAT + dlat, BASE_LON + dlon, th + math.pi)
        goals[aid] = (BASE_LAT - dlat, BASE_LON - dlon, 0.0)
    sitl_ids = [a for a in pos if kind[a] == "physical"]
    try:
        # ground-station broker
        broker_conf(tmp / "ground.conf", 1883)
        procs.append(subprocess.Popen(pinned(["mosquitto", "-c", str(tmp / "ground.conf")], c_gnd),
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        wait_port(1883)
        adapters = []
        for i, aid in enumerate(sitl_ids):
            lat, lon, yaw = pos[aid]
            d = tmp / aid
            d.mkdir()
            procs.append(subprocess.Popen(
                pinned([SITL, "-w", "-M", "motorboat", "-I", str(i),
                        "--home", f"{lat},{lon},0,{math.degrees(yaw) % 360:.1f}",
                        "--defaults", str(ROOT / "configs" / "sitl_boat.parm")], c_veh),
                cwd=d, stdout=open(d / "sitl.log", "w"), stderr=subprocess.STDOUT))
            time.sleep(1.0)
            adapters.append(subprocess.Popen(
                pinned([sys.executable, "-m", "mission_dt.mavlink_adapter", "--id", aid,
                        "--mavlink", f"tcp:127.0.0.1:{5760 + 10 * i}",
                        "--mqtt-port", "1883", "--metrics", str(d / "adapter.json")], c_veh),
                cwd=ROOT, stdout=open(d / "adapter.log", "w"), stderr=subprocess.STDOUT))
        # wait for the retained registrations of the physical agents at the ground broker
        seen = set()
        mon = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="e7-monitor")
        mon.on_message = lambda c, u, m: seen.add(m.topic.split("/")[2]) if m.payload else None
        mon.connect("127.0.0.1", 1883)
        mon.subscribe("missiondt/agents/+/register", qos=1)
        mon.loop_start()
        t_end = time.time() + 300
        while not set(sitl_ids) <= seen:
            if time.time() > t_end or any(a.poll() is not None for a in adapters):
                raise RuntimeError("physical agents not ready; see the adapter logs in " + str(tmp))
            time.sleep(0.5)
        mon.loop_stop()
        mon.disconnect()
        t_ready = time.time()

        core = CoreE7(swarm=True, sep_m=SEP)
        frames_phi = []
        core.on_frame = lambda M: frames_phi.append(
            {k: (M.phi.dist.get(k), M.phi.nb.get(k)) for k in sitl_ids if k in M.B})
        virt = []
        for aid in pos:
            if kind[aid] == "virtual":
                a = AgentE7(aid, domain="surface", duration_s=DURATION + 2)
                a.lat, a.lon, a.yaw = pos[aid]
                virt.append(a)
        time.sleep(1.0)
        for a in virt:
            a.start()
        core.run(DURATION, goals=goals)
        for a in virt:
            a.join(timeout=5)
        for a in adapters:
            a.send_signal(signal.SIGTERM)
        for a in adapters:
            a.wait(timeout=20)
        ad = {aid: json.load(open(tmp / aid / "adapter.json")) for aid in sitl_ids}
    finally:
        for p in procs[::-1]:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()

    kinds = ("physical", "virtual")
    tele = {k: [x * 1e3 for a, v in core.lat_by.items() if kind.get(a) == k for x in v] for k in kinds}
    act = {"physical": [x for a in ad.values() for x in a["raw_act_lat_ms"]],
           "virtual": [x * 1e3 for a in virt for x in a.act_latencies]}
    sw = defaultdict(list)
    for a in virt:
        for j, x in a.swarm_by:
            sw[f"virtual<-{kind.get(j)}"].append(x * 1e3)
    for a in ad.values():
        for j, x in a["swarm"]:
            sw[f"physical<-{kind.get(j)}"].append(x)
    min_sep = {k: min((f[k][0] for f in frames_phi if k in f and f[k][0] is not None),
                      default=None) for k in sitl_ids}
    conflicts = {k: sum(1 for f in frames_phi if k in f and f[k][0] is not None and f[k][0] < SEP)
                 for k in sitl_ids}
    track = {}
    for aid, a in ad.items():
        tr = [r for r in a["track"] if r[0] >= t_ready]
        dist = sum(math.hypot((b[1] - a_[1]) * 111_320.0,
                              (b[2] - a_[2]) * 111_320.0 * math.cos(math.radians(a_[1])))
                   for a_, b in zip(tr, tr[1:]))
        track[aid] = {"path_m": dist, "speed_mean": st.mean(r[3] for r in tr) if tr else None,
                      "speed_max": max((r[3] for r in tr), default=None)}
    return {
        "n_sitl": n_sitl, "n_virtual": n_virtual, "duration_s": DURATION, "sep_m": SEP,
        "sitl": {"binary": SITL, "model": "motorboat", "params": "configs/sitl_boat.parm"},
        "cpu": {"ground": c_gnd, "vehicle": c_veh},
        "frames": core.frames, "overruns": core.frame_overruns, "avoid_events": core.avoid_events,
        "frame_ms": summ([x * 1e3 for x in core.frame_compute]),
        "telemetry_rx": dict(core.rx_by),
        "telemetry_lat_ms": {k: summ(v) for k, v in tele.items()},
        "actuation_lat_ms": {k: summ(v) for k, v in act.items()},
        "swarm_lat_ms": {k: summ(v) for k, v in sorted(sw.items())},
        "adapter": {aid: {"msgs_out": a["msgs_out"], "cmd_sent": a["cmd_sent"],
                          "cmd_discarded": a["cmd_discarded"],
                          "sample_age_ms": summ(a["raw_sample_age_ms"]),
                          "act_to_mav_ms": summ(a["raw_act_to_mav_ms"])} for aid, a in ad.items()},
        "sitl_min_sep_m": min_sep, "sitl_conflict_frames": conflicts, "sitl_track": track,
        "raw_telemetry_lat_ms": tele, "raw_actuation_lat_ms": act,
        "raw_swarm_lat_ms": dict(sw),
        "raw_frame_compute_ms": [x * 1e3 for x in core.frame_compute],
    }


if __name__ == "__main__":
    args = [int(x) for x in sys.argv[1:] if not x.startswith("--")]
    n_sitl = args[0] if args else 2
    n_virtual = args[1] if len(args) > 1 else 8
    os.makedirs(RES, exist_ok=True)
    fn = f"{RES}/e7_hybrid.json"
    print(f"[E6] {n_sitl} ArduRover SITL + {n_virtual} virtual agents ...", flush=True)
    r = run_e7(n_sitl, n_virtual)
    json.dump([r], open(fn, "w"))
    t, s = r["telemetry_lat_ms"], r["swarm_lat_ms"]
    print(f"     tele p50 phys={t['physical']['p50']:.1f} virt={t['virtual']['p50']:.1f} ms | "
          + " ".join(f"{k}: p50={v['p50']:.0f} n={v['n']}" for k, v in s.items())
          + f" | overruns={r['overruns']} discarded="
          + str(sum(a['cmd_discarded'] for a in r['adapter'].values())), flush=True)
