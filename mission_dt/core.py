"""
Mission-DT core, rewritten on top of the formal model (mission_dt/model.py).

The core only does I/O: it collects z^t from MQTT, calls
(M^t, u^t) = mission_transition(H_M^t, z^t, omega^t) once per frame, and publishes u^t.
The MQTT contract (topics, QoS, payload fields) is unchanged, so the
virtual agents, the viewer and the experiment scripts work as before.
"""
import json
import os
import socket
import threading
import time
from collections import defaultdict
from types import SimpleNamespace

import paho.mqtt.client as mqtt

from . import model as md

FRAME_MS = 125.0


def _thread_cpu(native_id):
    """CPU time (s) of one thread, from /proc (Linux); 0.0 elsewhere."""
    if native_id is None:
        return 0.0
    try:
        with open(f"/proc/self/task/{native_id}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError):
        return 0.0


class _AgentView:
    """Read-only view kept for scripts that read dt.agents[k].state.*"""
    def __init__(self, aid, dom, kind, core):
        self.agent_id, self.domain, self.kind, self._core = aid, dom, kind, core

    @property
    def state(self):
        B = self._core.HM.last.get(self.agent_id, md.State())
        return SimpleNamespace(lat=B.p[0], lon=B.p[1], alt=B.p[2], yaw=B.yaw, t=B.t)

    @property
    def last_seq(self):
        return self._core.HM.last.get(self.agent_id, md.State()).seq


class MissionDT:
    def __init__(self, host="127.0.0.1", frame_ms=FRAME_MS, viz_hook=None,
                 swarm=False, sep_m=12.0, on_frame=None):
        self.P = md.Params(T_f=frame_ms / 1000.0, d_s=sep_m, separation=swarm)
        self.frame_s = self.P.T_f
        self.swarm, self.sep_m = swarm, sep_m
        self.HM = md.MissionHistory()
        self.dom, self.kind = {}, {}
        self.agents = {}
        self.viz_hook, self.on_frame = viz_hook, on_frame
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._pending = defaultdict(list)
        # metrics (same names as the original core)
        self.msg_latencies, self.frame_compute = [], []
        self.frame_overruns = self.frames = 0
        self.stale_updates = self.dup_updates = self.avoid_events = 0
        self.bytes_in = 0
        # frame release jitter: actual start minus scheduled start (s)
        self.release_jitter = []
        # CPU time of the core: frame loop thread + MQTT network thread (s)
        self.cpu_loop_s = self.cpu_net_s = 0.0
        self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                               client_id="mission-dt", protocol=mqtt.MQTTv5)
        self.cli.on_message = self._on_msg
        self.cli.connect(host, 1883)
        # disable Nagle on the client socket: paho does not set TCP_NODELAY
        self.cli.socket().setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.cli.subscribe("missiondt/agents/+/telemetry", qos=0)
        self.cli.subscribe("missiondt/agents/+/register", qos=1)
        self.cli.loop_start()

    def _on_msg(self, cli, ud, msg):
        now = time.time()
        try:
            payload = json.loads(msg.payload)
        except json.JSONDecodeError:
            return
        self.bytes_in += len(msg.payload)
        _, _, aid, topic = msg.topic.split("/")[:4]
        if topic == "register":
            with self._lock:
                self.dom[aid], self.kind[aid] = payload["domain"], payload["kind"]
                self.agents[aid] = _AgentView(aid, payload["domain"], payload["kind"], self)
            return
        self.msg_latencies.append(now - payload["t_pub"])
        with self._lock:
            self._pending[aid].append(md.Msg.from_payload(payload))

    def run(self, duration_s, goals=None):
        goals = goals if goals is not None else {}
        t_next = time.monotonic()
        t_end = t_next + duration_s
        cpu0 = time.thread_time()
        net_tid = getattr(getattr(self.cli, "_thread", None), "native_id", None)
        net0 = _thread_cpu(net_tid)
        while not self._stop.is_set() and time.monotonic() < t_end:
            t0 = time.monotonic()
            self.release_jitter.append(t0 - t_next)
            with self._lock:
                pending, self._pending = self._pending, defaultdict(list)
                dom = dict(self.dom)
            I = {k: tuple(v) for k, v in pending.items() if k in dom}
            M, A, trig = md.mission_transition(self.HM, I, goals, dom, self.P)
            # metrics: stale counts only seeded agents (fix D10)
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
                self.cli.publish(f"missiondt/agents/{k}/actuation", json.dumps(a), qos=0)
            # frame time: from z^t collection to the publication of the
            # last actuation; instrumentation hooks run after the measurement
            work = time.monotonic() - t0
            if self.on_frame:
                self.on_frame(M)
            if self.viz_hook:
                self.viz_hook(M)
            self.frame_compute.append(work)
            self.frames += 1
            if work > self.frame_s:
                self.frame_overruns += 1
            t_next += self.frame_s
            sleep = t_next - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.monotonic()
        self.cpu_loop_s = time.thread_time() - cpu0
        self.cpu_net_s = _thread_cpu(net_tid) - net0
        self.cli.loop_stop()
        self.cli.disconnect()

    def stop(self):
        self._stop.set()
