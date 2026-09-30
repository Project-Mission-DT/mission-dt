"""
E6 -- Resource comparison: Mission-DT (MQTT) vs. ROS 2 vs. Gazebo.

Three implementations of the same fleet, N agents on the goal grid of E5:

  mission_dt  the mission core process of E5 (mission_dt/core.py), the agents
              process of E5 (VirtualAgent threads) and a Mosquitto broker
              started for the trial with mosquitto.conf.
  ros2        a mission node process (rclpy) that subscribes /mdt/<id>/telemetry,
              calls the mission transition mission_transition of mission_dt/model.py as
              core.py does (history, parameters, stale/dup accounting, trigger
              fields) in a 125 ms timer and publishes /mdt/<id>/actuation
              (std_msgs/String, same JSON payloads); an agents process (rclpy)
              with N VirtualAgent threads whose MQTT client is replaced by a
              ROS 2 publisher (same kinematics, 50 Hz step, 8.33 Hz telemetry).
              QoS best effort, keep last 1, volatile (MQTT QoS 0). Default RMW.
  gazebo      gz sim server (headless, no GUI) with a generated SDF world of N box
              vehicles, VelocityControl (constant twist) and OdometryPublisher at
              8.33 Hz, 1 ms physics step, real-time factor 1. No mission layer.

Every process runs on CPU 0 (run the script under `taskset -c 0`; children
inherit the affinity). After a warm-up of MDT_E6_WARMUP s (default 5), the
script measures a window of MDT_E6_DURATION s (default 30) and reports, per
process of each stack:
    CPU utilisation = (utime + stime delta of /proc/<pid>/stat) / wall, % of one core
    peak resident memory = VmHWM of /proc/<pid>/status at the end of the window
    resident memory at the end of the window = VmRSS
The mission processes report the frames, overruns and received telemetry
inside the same window (the script marks the window start and end on their
stdin). For Gazebo a monitor process subscribes to /world/<w>/stats and to
every odometry topic (gz-transport; its CPU and memory are reported apart as
"monitor", outside the stack total) and reports the real-time factor over
the window.

    taskset -c 0 python3.12 experiments/run_e6.py [--stacks mission_dt,ros2,gazebo] [N ...]
                                                            # default N: 10 50 100
Environment: MDT_RESULTS (output dir), MDT_ROS_SETUP (default
/opt/ros/jazzy/setup.bash), MDT_E6_RMW (RMW_IMPLEMENTATION for the ROS 2
stack; unset = the default RMW of the distribution).
"""
import json
import math
import os
import platform
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from collections import defaultdict
from functools import partial
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
RES = os.environ.get("MDT_RESULTS") or str(ROOT / "results")
DURATION = float(os.environ.get("MDT_E6_DURATION", "30"))
WARMUP = float(os.environ.get("MDT_E6_WARMUP", "5"))
TAIL = 3.0                                   # margin after the window
RUN_S = WARMUP + DURATION + TAIL             # run time of the mission processes
FRAME_MS = 125.0
STACKS = ("mission_dt", "ros2", "gazebo")
ROS_SETUP = os.environ.get("MDT_ROS_SETUP", "/opt/ros/jazzy/setup.bash")
ROS_DOMAIN_ID = "42"
GZ_WORLD = "e6"
GZ_ODOM_HZ = 50.0 / 6.0                      # 8.33 Hz, the regulated rate
TCK = os.sysconf("SC_CLK_TCK")
# the script subscribes to every odometry topic to verify the 8.33 Hz output;
# MDT_E6_GZ_ODOM_SUB=0 leaves the odometry without a subscriber
ODOM_SUB = os.environ.get("MDT_E6_GZ_ODOM_SUB", "1") != "0"

from run_e5 import goal                      # noqa: E402  same goals as E5


def aid(i):
    return f"rs{i:03d}"


def dom_of(i):
    return "aerial" if i % 2 else "surface"


def pctl(v, p):
    """Percentile of rank floor(pn/100)+1 on a sorted list (rule of the other runners)."""
    return v[min(len(v) - 1, int(p / 100.0 * len(v)))] if v else None


def stats_ms(v):
    s = sorted(1000.0 * x for x in v)
    if not s:
        return {}
    return {"mean": sum(s) / len(s), "p50": pctl(s, 50), "p99": pctl(s, 99),
            "max": s[-1], "n": len(s)}


# ---------------------------------------------------------------- /proc
def proc_cpu_s(pid):
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / TCK


def proc_status(pid):
    out = {}
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            k, _, v = line.partition(":")
            if k in ("VmHWM", "VmRSS"):
                out[k] = int(v.split()[0]) / 1024.0      # kB -> MiB
            elif k == "Threads":
                out[k] = int(v)
    return out


def descendants(pid):
    """pid and all its descendant processes (from /proc/<p>/stat ppid)."""
    children = defaultdict(list)
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            children[ppid].append(int(d))
        except (OSError, ValueError, IndexError):
            pass
    out, todo = [], [pid]
    while todo:
        p = todo.pop()
        out.append(p)
        todo.extend(children.get(p, []))
    return sorted(out)


def affinity_ok():
    return os.sched_getaffinity(0) == {0}


# ---------------------------------------------------------------- child side
class Marks:
    """Reads 'mark' lines on stdin after 'go' and records snap() at each one."""
    def __init__(self, snap):
        self.snap, self.snaps = snap, []
        self._ev = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in sys.stdin:
            if line.strip() == "mark":
                self.snaps.append(self.snap())
                if len(self.snaps) >= 2:
                    self._ev.set()

    def wait(self, timeout=30.0):
        self._ev.wait(timeout)
        return self.snaps[:2]


def mission_report(n, snaps, rx, lat, frames, jitter, frames_total, overruns_total):
    """Window statistics of a mission process (MQTT core or ROS 2 node)."""
    if len(snaps) < 2:
        return {"error": "window marks missing"}
    (t0, rx0, l0, f0, j0), (t1, rx1, l1, f1, j1) = snaps
    fr = frames[f0:f1]
    counts = [rx1.get(aid(i), 0) - rx0.get(aid(i), 0) for i in range(n)]
    win = t1 - t0
    return {
        "frames": len(fr), "overruns": sum(1 for x in fr if x > FRAME_MS / 1000.0),
        "frames_run": frames_total, "overruns_run": overruns_total,
        "frame_ms": stats_ms(fr), "release_jitter_ms": stats_ms(jitter[j0:j1]),
        "telemetry_msgs": sum(counts),
        "telemetry_agents_received": sum(1 for c in counts if c > 0),
        "telemetry_min_per_agent": min(counts) if counts else 0,
        "telemetry_max_per_agent": max(counts) if counts else 0,
        "telemetry_rate_hz_per_agent": sum(counts) / n / win if n else 0.0,
        "telemetry_lat_ms": stats_ms(lat[l0:l1]),
    }


def agents_report(agents, snaps):
    if len(snaps) < 2:
        return {"error": "window marks missing"}
    (t0, a0, m0), (t1, a1, m1) = snaps
    win = t1 - t0
    act = [a1[i] - a0[i] for i in range(len(agents))]
    sent = [m1[i] - m0[i] for i in range(len(agents))]
    lat = []
    for i, a in enumerate(agents):
        lat.extend(a.act_latencies[a0[i]:a1[i]])
    return {"actuation_msgs": sum(act),
            "actuation_agents_received": sum(1 for c in act if c > 0),
            "actuation_min_per_agent": min(act), "actuation_max_per_agent": max(act),
            "telemetry_sent": sum(sent),
            "telemetry_rate_hz_per_agent": sum(sent) / len(agents) / win,
            "telemetry_bytes_mean": (sum(a.bytes_out for a in agents)
                                     / max(1, sum(a.msgs_out for a in agents))),
            "actuation_lat_ms": stats_ms(lat)}


def agents_snap(agents):
    return lambda: (time.monotonic(), [len(a.act_latencies) for a in agents],
                    [a.msgs_out for a in agents])


def child_mqtt_core(n):
    """Mission-DT core of E5 (mission_dt/core.py) with a per-agent receive counter."""
    from mission_dt.core import MissionDT

    class CountingMissionDT(MissionDT):
        def __init__(self, **kw):
            self.rx = {}
            super().__init__(**kw)

        def _on_msg(self, cli, ud, msg):
            if msg.topic.endswith("/telemetry"):
                k = msg.topic.split("/")[2]
                self.rx[k] = self.rx.get(k, 0) + 1
            super()._on_msg(cli, ud, msg)

    dt = CountingMissionDT()
    goals = {aid(i): goal(i, dom_of(i)) for i in range(n)}
    print("READY", flush=True)
    sys.stdin.readline()                     # go
    marks = Marks(lambda: (time.monotonic(), dict(dt.rx), len(dt.msg_latencies),
                           len(dt.frame_compute), len(dt.release_jitter)))
    dt.run(RUN_S, goals=goals)
    r = mission_report(n, marks.wait(), dt.rx, dt.msg_latencies, dt.frame_compute,
                       dt.release_jitter, dt.frames, dt.frame_overruns)
    r["transport"] = "paho-mqtt " + __import__("paho.mqtt").mqtt.__version__
    print(json.dumps(r), flush=True)


def child_mqtt_agents(n):
    """Agents process of E5: N VirtualAgent threads."""
    from mission_dt.agents import VirtualAgent
    agents = [VirtualAgent(aid(i), domain=dom_of(i), duration_s=RUN_S + 2)
              for i in range(n)]
    print("READY", flush=True)
    sys.stdin.readline()
    marks = Marks(agents_snap(agents))
    for a in agents:
        a.start()
    snaps = marks.wait(RUN_S + 10)
    for a in agents:
        a.join(timeout=RUN_S + 10)
    print(json.dumps(agents_report(agents, snaps)), flush=True)


# ---- ROS 2 stack --------------------------------------------------------
def ros_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)


class RosMissionDT:
    """ROS 2 counterpart of MissionDT (mission_dt/core.py).

    Same data path: telemetry callback -> pending z^t -> mission_transition once per
    125 ms frame -> actuation with t_pub (and the trigger fields). The frame
    runs in an rclpy timer of a SingleThreadedExecutor instead of the sleep
    loop of core.py. The agent domains are known from the start (the MQTT
    core learns them from the retained register messages before the run).
    """
    def __init__(self, node, ids, dom, frame_ms=FRAME_MS, swarm=False, sep_m=12.0):
        from std_msgs.msg import String
        from mission_dt import model as md
        self.md, self.String = md, String
        self.P = md.Params(T_f=frame_ms / 1000.0, d_s=sep_m, separation=swarm)
        self.frame_s = self.P.T_f
        self.HM = md.MissionHistory()
        self.dom = dict(dom)
        self._lock = threading.Lock()
        self._pending = defaultdict(list)
        self.msg_latencies, self.frame_compute, self.release_jitter = [], [], []
        self.frame_overruns = self.frames = 0
        self.stale_updates = self.dup_updates = self.avoid_events = 0
        self.bytes_in = 0
        self.rx = {}
        self.goals = {}
        self.node = node
        q = ros_qos()
        self.pub = {k: node.create_publisher(String, f"/mdt/{k}/actuation", q) for k in ids}
        self.subs = [node.create_subscription(String, f"/mdt/{k}/telemetry",
                                              partial(self._on_msg, k), q) for k in ids]
        self.timer = None
        self._t_timer = None

    def _on_msg(self, k, msg):
        now = time.time()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.bytes_in += len(msg.data)
        self.rx[k] = self.rx.get(k, 0) + 1
        self.msg_latencies.append(now - payload["t_pub"])
        with self._lock:
            self._pending[k].append(self.md.Msg.from_payload(payload))

    def start(self, goals):
        self.goals = goals
        self._t_timer = time.monotonic()
        self.timer = self.node.create_timer(self.frame_s, self._frame)

    def _frame(self):
        md = self.md
        t0 = time.monotonic()
        # the rcl timer releases at t_create + k*T_f (drift-free schedule)
        self.release_jitter.append((t0 - self._t_timer) % self.frame_s)
        with self._lock:
            pending, self._pending = self._pending, defaultdict(list)
            dom = dict(self.dom)
        I = {k: tuple(v) for k, v in pending.items() if k in dom}
        M, A, trig = md.mission_transition(self.HM, I, self.goals, dom, self.P)
        for k, B in M.B.items():
            if B.seeded and B.stale:
                self.stale_updates += 1
            n = len(I.get(k, ()))
            if n > 1:
                self.dup_updates += n - 1
        for k, a in A.items():
            a = dict(a, t_pub=time.time())
            j = trig.get(k)
            if j is not None:
                a.update(avoid=True, trig_t=M.B[j].t, trig_id=j)
                self.avoid_events += 1
            m = self.String()
            m.data = json.dumps(a)
            self.pub[k].publish(m)
        work = time.monotonic() - t0
        self.frame_compute.append(work)
        self.frames += 1
        if work > self.frame_s:
            self.frame_overruns += 1


def child_ros_core(n):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = rclpy.create_node("mission_dt")
    ids = [aid(i) for i in range(n)]
    dt = RosMissionDT(node, ids, {aid(i): dom_of(i) for i in range(n)})
    goals = {aid(i): goal(i, dom_of(i)) for i in range(n)}
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    print("READY", flush=True)
    sys.stdin.readline()                     # go
    marks = Marks(lambda: (time.monotonic(), dict(dt.rx), len(dt.msg_latencies),
                           len(dt.frame_compute), len(dt.release_jitter)))
    dt.start(goals)
    t_end = time.monotonic() + RUN_S
    while time.monotonic() < t_end:
        ex.spin_once(timeout_sec=0.05)
    r = mission_report(n, marks.wait(), dt.rx, dt.msg_latencies, dt.frame_compute,
                       dt.release_jitter, dt.frames, dt.frame_overruns)
    r["transport"] = rclpy.get_rmw_implementation_identifier()
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    print(json.dumps(r), flush=True)


class _RosLink:
    """Stands in for the paho client of a VirtualAgent: telemetry goes to a
    ROS 2 publisher; registration and deregistration have no ROS 2 counterpart
    (the mission node knows the fleet) and are dropped."""
    def __init__(self, node, agent_id, on_act, String, qos):
        self.String = String
        self.pub = node.create_publisher(String, f"/mdt/{agent_id}/telemetry", qos)
        self.sub = node.create_subscription(
            String, f"/mdt/{agent_id}/actuation",
            lambda m: on_act(None, None, SimpleNamespace(payload=m.data)), qos)

    def publish(self, topic, payload=None, qos=0, retain=False):
        if topic.endswith("/telemetry"):
            m = self.String()
            m.data = payload
            self.pub.publish(m)

    def loop_stop(self):
        pass

    def disconnect(self):
        pass


def child_ros_agents(n):
    """N VirtualAgent threads (same _init_state, _step, _telemetry, run and
    _on_act as mission_dt/agents.py) with a ROS 2 transport."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from mission_dt.agents import VirtualAgent

    class RosAgent(VirtualAgent):
        def __init__(self, agent_id, domain, node, duration_s):
            threading.Thread.__init__(self, daemon=True)
            self._init_state(agent_id, domain, True, duration_s, True, 0.0)
            self.cli = _RosLink(node, agent_id, self._on_act, String, ros_qos())

    rclpy.init()
    node = rclpy.create_node("mdt_agents")
    agents = [RosAgent(aid(i), dom_of(i), node, RUN_S + 2) for i in range(n)]
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    # wait for discovery: every telemetry and actuation topic matched
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90:
        if all(a.cli.pub.get_subscription_count() >= 1 and
               a.cli.sub.get_publisher_count() >= 1 for a in agents):
            break
        time.sleep(0.2)
    matched = sum(1 for a in agents if a.cli.pub.get_subscription_count() >= 1 and
                  a.cli.sub.get_publisher_count() >= 1)
    disc = time.monotonic() - t0
    print("READY", flush=True)
    sys.stdin.readline()
    marks = Marks(agents_snap(agents))
    for a in agents:
        a.start()
    snaps = marks.wait(RUN_S + 10)
    for a in agents:
        a.join(timeout=RUN_S + 10)
    r = agents_report(agents, snaps)
    r.update(discovery_s=disc, matched_agents=matched)
    ex.shutdown()
    spin.join(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()
    print(json.dumps(r), flush=True)


# ---------------------------------------------------------------- harness side
def port_open(port):
    try:
        socket.create_connection(("127.0.0.1", port), 0.5).close()
        return True
    except OSError:
        return False


_ROS_ENV = None


def ros_env():
    """Environment of `source $MDT_ROS_SETUP`, for the ROS 2 processes only."""
    global _ROS_ENV
    if _ROS_ENV is None:
        out = subprocess.run(["bash", "-c", f"source {ROS_SETUP} >/dev/null 2>&1; env -0"],
                             capture_output=True, check=True).stdout
        env = dict(x.split("=", 1) for x in out.decode().split("\0") if "=" in x)
        env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        if os.environ.get("MDT_E6_RMW"):
            env["RMW_IMPLEMENTATION"] = os.environ["MDT_E6_RMW"]
        _ROS_ENV = env
    return _ROS_ENV


def spawn(role, n, env=None):
    p = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), f"--{role}", str(n)],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                         env=env, cwd=str(ROOT))
    line = p.stdout.readline().strip()
    if line != "READY":
        p.kill()
        raise RuntimeError(f"{role}: expected READY, got {line!r}")
    return p


def send(procs, line):
    for p in procs:
        p.stdin.write(line + "\n")
        p.stdin.flush()


def measure(groups, mark=()):
    """CPU and memory of each process group over the window.

    groups: {name: callable returning the pids of that group}. mark: child
    processes that receive 'mark' at the window start and end; extra marks run
    callables at the same instants.
    """
    pids = {k: f() for k, f in groups.items()}
    c0 = {k: sum(proc_cpu_s(p) for p in v) for k, v in pids.items()}
    w0 = time.monotonic()
    hc0 = os.times()
    for m in mark:
        m()
    time.sleep(DURATION)
    pids1 = {k: f() for k, f in groups.items()}
    c1 = {k: sum(proc_cpu_s(p) for p in v) for k, v in pids.items()}
    w1 = time.monotonic()
    hc1 = os.times()
    sts = {k: [proc_status(p) for p in v] for k, v in pids.items()}
    for m in mark:
        m()
    wall = w1 - w0
    out = {}
    for k in pids:
        cpu = c1[k] - c0[k]
        out[k] = {"cpu_s": cpu, "cpu_pct": 100.0 * cpu / wall,
                  "peak_rss_mib": sum(s.get("VmHWM", 0.0) for s in sts[k]),
                  "rss_end_mib": sum(s.get("VmRSS", 0.0) for s in sts[k]),
                  "threads": sum(s.get("Threads", 0) for s in sts[k]),
                  "n_processes": len(pids[k])}
        if pids1[k] != pids[k]:
            out[k]["pids_changed"] = 1
    harness = (hc1.user - hc0.user) + (hc1.system - hc0.system)
    return out, wall, 100.0 * harness / wall


def totals(procs):
    return {"cpu_pct": sum(p["cpu_pct"] for p in procs.values()),
            "peak_rss_mib": sum(p["peak_rss_mib"] for p in procs.values()),
            "rss_end_mib": sum(p["rss_end_mib"] for p in procs.values())}


def finish(p, timeout=RUN_S + 30):
    line = p.stdout.readline()
    p.wait(timeout=timeout)
    return json.loads(line) if line.strip() else {"error": "no report"}


def trial_mission_dt(n):
    if port_open(1883):
        raise RuntimeError("a broker already listens on 127.0.0.1:1883; stop it, "
                           "E6 starts its own broker for each trial")
    broker = subprocess.Popen(["mosquitto", "-c", str(ROOT / "mosquitto.conf")],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.monotonic()
    while not port_open(1883):
        if time.monotonic() - t0 > 10:
            broker.kill()
            raise RuntimeError("mosquitto did not start")
        time.sleep(0.1)
    try:
        core = spawn("mqtt-core", n)
        ag = spawn("mqtt-agents", n)
        time.sleep(1.0)                      # registration settle (as E5)
        send([ag, core], "go")
        time.sleep(WARMUP)
        procs, wall, hcpu = measure(
            {"core": lambda: [core.pid], "agents": lambda: [ag.pid],
             "broker": lambda: [broker.pid]},
            mark=[lambda: send([core, ag], "mark")])
        r_core, r_ag = finish(core), finish(ag)
    finally:
        broker.terminate()
        broker.wait(timeout=10)
    return {"stack": "mission_dt", "n_agents": n, "window_s": wall,
            "processes": procs, "total": totals(procs), "harness_cpu_pct": hcpu,
            "mission": r_core, "agents": r_ag,
            "transport": r_core.get("transport"), "broker": "mosquitto (mosquitto.conf)"}


def trial_ros2(n):
    env = ros_env()
    core = spawn("ros-core", n, env)
    ag = spawn("ros-agents", n, env)
    time.sleep(1.0)
    send([ag, core], "go")
    time.sleep(WARMUP)
    procs, wall, hcpu = measure(
        {"core": lambda: [core.pid], "agents": lambda: [ag.pid]},
        mark=[lambda: send([core, ag], "mark")])
    r_core, r_ag = finish(core), finish(ag)
    return {"stack": "ros2", "n_agents": n, "window_s": wall,
            "processes": procs, "total": totals(procs), "harness_cpu_pct": hcpu,
            "mission": r_core, "agents": r_ag, "transport": r_core.get("transport"),
            "ros_distro": env.get("ROS_DISTRO"), "qos": "best_effort, keep_last 1, volatile"}


# ---- Gazebo stack -------------------------------------------------------
def gz_world(n):
    """SDF world: N box vehicles on the E5 goal grid, constant twist,
    odometry at 8.33 Hz; systems: Physics only at world level."""
    from mission_dt.agents import BASE_LAT, BASE_LON
    m = 111_320.0
    models = []
    for i in range(n):
        d = dom_of(i)
        lat, lon, alt = goal(i, d)
        x = (lon - BASE_LON) * m * math.cos(math.radians(BASE_LAT))
        y = (lat - BASE_LAT) * m
        v = 12.0 if d == "aerial" else 2.0   # vmax of the kinematic agents
        models.append(f"""
  <model name="{aid(i)}"><pose>{x:.3f} {y:.3f} {alt:.1f} 0 0 0</pose>
    <link name="base_link">
      <inertial><mass>10</mass><inertia><ixx>0.28</ixx><iyy>0.9</iyy><izz>1.0</izz></inertia></inertial>
      <collision name="hull"><geometry><box><size>1.2 0.6 0.3</size></box></geometry></collision>
    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <initial_linear>{v} 0 0</initial_linear><initial_angular>0 0 0.05</initial_angular>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_publish_frequency>{GZ_ODOM_HZ:.6f}</odom_publish_frequency><dimensions>3</dimensions>
    </plugin>
  </model>""")
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
<world name="{GZ_WORLD}">
  <physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>
  </physics>
  <gravity>0 0 0</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>{''.join(models)}
</world>
</sdf>
"""


def gz_version():
    try:
        return subprocess.run(["gz", "sim", "--versions"], capture_output=True,
                              text=True, timeout=30).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def child_gz_monitor(n):
    """gz-transport subscriber of the Gazebo stack: /world/<w>/stats (real-time
    factor) and every /model/<id>/odometry (output of each vehicle). Runs in its
    own process so that it ends with the trial."""
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    node = Node()
    last, stats, odom = {}, [], defaultdict(int)

    def on_stats(msg):
        last["s"] = (time.monotonic(), msg.sim_time.sec + msg.sim_time.nsec * 1e-9,
                     msg.real_time.sec + msg.real_time.nsec * 1e-9, msg.iterations,
                     msg.real_time_factor)
        stats.append(last["s"])

    def on_odom(k, msg):
        odom[k] += 1

    def sub_odom(k):
        node.subscribe(Odometry, f"/model/{k}/odometry", partial(on_odom, k))

    node.subscribe(WorldStatistics, f"/world/{GZ_WORLD}/stats", on_stats)
    t0 = time.monotonic()
    while "s" not in last:
        if time.monotonic() - t0 > 120:
            print("NO_STATS", flush=True)
            return
        time.sleep(0.2)
    startup = time.monotonic() - t0
    # subscribe to the odometry once the server advertises it; a subscription
    # that has delivered nothing after 5 s is renewed (some subscriptions made
    # right after the advertisement never matched); at most 60 s
    time.sleep(2.0)
    ids = [aid(i) for i in range(n)] if ODOM_SUB else []
    for k in ids:
        sub_odom(k)
    t1, renewed = time.monotonic(), 0
    t_renew = t1 + 5.0
    while len(odom) < len(ids) and time.monotonic() - t1 < 60:
        time.sleep(0.2)
        if time.monotonic() >= t_renew:
            for k in ids:
                if odom.get(k, 0) == 0:
                    node.unsubscribe(f"/model/{k}/odometry")
                    sub_odom(k)
                    renewed += 1
            t_renew = time.monotonic() + 5.0
    connect = time.monotonic() - t1
    print("READY", flush=True)
    marks = Marks(lambda: (time.monotonic(), last.get("s"), dict(odom), len(stats)))
    snaps = marks.wait(DURATION + 60)
    if len(snaps) < 2 or snaps[0][1] is None:
        print(json.dumps({"error": "window marks missing"}), flush=True)
        return
    (w0, l0, o0, s0), (w1, l1, o1, s1) = snaps
    _, sim0, real0, it0, _ = l0
    _, sim1, real1, it1, _ = l1
    counts = [o1.get(aid(i), 0) - o0.get(aid(i), 0) for i in range(n)]
    rtf_field = [x[4] for x in stats[s0:s1]]
    print(json.dumps({
        "rtf": (sim1 - sim0) / (real1 - real0),
        "rtf_stats_mean": sum(rtf_field) / len(rtf_field) if rtf_field else None,
        "rtf_stats_min": min(rtf_field) if rtf_field else None,
        "sim_s": sim1 - sim0, "real_s": real1 - real0,
        "steps_per_s": (it1 - it0) / (real1 - real0),
        "step_ms": 1.0, "startup_s": startup, "odom_connect_s": connect,
        "odom_subscriptions_renewed": renewed,
        "odom_msgs": sum(counts),
        "odom_models_received": sum(1 for c in counts if c > 0),
        "odom_min_per_model": min(counts) if counts else 0,
        "odom_max_per_model": max(counts) if counts else 0,
        "odom_rate_hz_per_model": sum(counts) / n / (w1 - w0) if n else 0.0}), flush=True)


def trial_gazebo(n):
    os.environ.setdefault("GZ_PARTITION", f"mdt_e6_{os.getpid()}")
    tmp = tempfile.mkdtemp(prefix="mdt_e6_")
    sdf = os.path.join(tmp, "fleet.sdf")
    with open(sdf, "w") as fh:
        fh.write(gz_world(n))
    log = open(os.path.join(tmp, "gz.log"), "w")
    gz = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1", sdf],
                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        mon = spawn("gz-monitor", n)         # READY once stats and odometry flow
        time.sleep(WARMUP)
        procs, wall, hcpu = measure(
            {"gz_sim": lambda: descendants(gz.pid), "monitor": lambda: [mon.pid]},
            mark=[lambda: send([mon], "mark")])
        g = finish(mon, timeout=60)
    finally:
        os.killpg(gz.pid, signal.SIGINT)
        try:
            gz.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(gz.pid, signal.SIGKILL)
            gz.wait()
        log.close()
    monitor = procs.pop("monitor")
    return {"stack": "gazebo", "n_agents": n, "window_s": wall,
            "processes": procs, "total": totals(procs), "harness_cpu_pct": hcpu,
            "monitor": monitor, "gazebo": g, "gz_sim_version": gz_version()}


TRIALS = {"mission_dt": trial_mission_dt, "ros2": trial_ros2, "gazebo": trial_gazebo}
CHILDREN = {"--mqtt-core": child_mqtt_core, "--mqtt-agents": child_mqtt_agents,
            "--ros-core": child_ros_core, "--ros-agents": child_ros_agents,
            "--gz-monitor": child_gz_monitor}


def env_info():
    out = {"python": platform.python_version()}
    try:
        out["mosquitto"] = subprocess.run(["mosquitto", "-h"], capture_output=True,
                                          text=True).stdout.splitlines()[0]
    except (OSError, IndexError):
        pass
    return out


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] in CHILDREN:
        CHILDREN[sys.argv[1]](int(sys.argv[2]))
        sys.exit(0)
    args = sys.argv[1:]
    stacks = list(STACKS)
    if args and args[0] == "--stacks":
        stacks = args[1].split(",")
        args = args[2:]
    sizes = [int(x) for x in args] or [10, 50, 100]
    if not affinity_ok():
        print("warning: run E6 under `taskset -c 0` (current affinity "
              f"{sorted(os.sched_getaffinity(0))})", flush=True)
    os.makedirs(RES, exist_ok=True)
    info = env_info()
    out = []
    for n in sizes:
        for s in stacks:
            print(f"[E6] {s} N={n} ...", flush=True)
            r = TRIALS[s](n)
            r.update(info, cpu_affinity=sorted(os.sched_getaffinity(0)))
            out.append(r)
            json.dump(out, open(f"{RES}/e6_comparison.json", "w"), indent=1)
            line = "  ".join(f"{k}={v['cpu_pct']:.1f}%/{v['peak_rss_mib']:.1f}MiB"
                             for k, v in r["processes"].items())
            extra = ""
            if "mission" in r:
                m = r["mission"]
                extra = (f" frames={m.get('frames')} overruns={m.get('overruns')} "
                         f"rx_agents={m.get('telemetry_agents_received')}/{n} "
                         f"min_rx={m.get('telemetry_min_per_agent')}")
            if "gazebo" in r:
                extra = (f" RTF={r['gazebo']['rtf']:.3f} "
                         f"odom_models={r['gazebo']['odom_models_received']}/{n}")
            print(f"     {line}  total={r['total']['cpu_pct']:.1f}%/"
                  f"{r['total']['peak_rss_mib']:.1f}MiB{extra}", flush=True)
            time.sleep(3)
    print("done")
