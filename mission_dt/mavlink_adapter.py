"""
MAVLink-to-MQTT adapter of a physical agent (Section III-B of the paper).

The adapter runs on the companion computer of an ArduPilot vehicle (or next
to an ArduPilot SITL instance, as in E6) and connects the autopilot to the
ground-station MQTT broker over the wireless link of the vehicle.
Towards Mission-DT it implements the same contract as a virtual agent:

  register   missiondt/agents/<id>/register   QoS 1, retained,
             {"domain": ..., "kind": "physical"}
  telemetry  missiondt/agents/<id>/telemetry  QoS 0, z_k^t with seq and t_pub
  actuation  missiondt/agents/<id>/actuation  QoS 0, u_k^t = {"tau", "alpha"}

Telemetry: the adapter requests GLOBAL_POSITION_INT and ATTITUDE at 50 Hz and
publishes one of every six position samples (8.33 Hz), the publication rate of
the virtual agents (--publish-all: every 50 Hz sample).

Actuation: the adapter discards a command older than T_f (t_pub of the core)
and passes the others to the autopilot in GUIDED mode as SET_ATTITUDE_TARGET
with the attitude ignored: body yaw rate = YAW_GAIN * alpha (rad/s) and
thrust = tau, which ArduRover maps to a target speed of tau * WP_SPEED.
YAW_GAIN and WP_SPEED = 2 m/s match the kinematic model of the virtual
surface agents (mission_dt/agents.py). Geofence and failsafe functions of the
autopilot stay active and override the mission twin.

Usage (one process per vehicle):
    python -m mission_dt.mavlink_adapter --id boat1 --mavlink tcp:127.0.0.1:5760 \
        --mqtt-host <ground-station broker> [--mqtt-port 1883] [--domain surface] \
        [--publish-all] [--metrics out.json]
SIGTERM or SIGINT stops the vehicle (tau = 0), disarms it, removes the
retained registration, and writes the metrics file.
"""
import argparse
import json
import math
import signal
import socket
import threading
import time

import paho.mqtt.client as mqtt
from pymavlink import mavutil

SENSOR_HZ = 50.0
DECIM = 6                  # 50 Hz / 6 = 8.33 Hz, as the virtual agents
T_F = 0.125                # commands older than one frame are discarded
YAW_GAIN = {"surface": 0.6, "aerial": 1.5}   # rad/s per unit alpha (agents.py)
MAV = mavutil.mavlink
ATT_MASK = 0b10000000      # ignore attitude quaternion, use body yaw rate and thrust
ATT_MASK |= 0b00000011     # ignore body roll and pitch rates


class MavlinkAdapter:
    def __init__(self, agent_id, mav_url, mqtt_host="127.0.0.1", mqtt_port=1883,
                 domain="surface", decimate=True):
        self.aid, self.domain = agent_id, domain
        self.publish_every = DECIM if decimate else 1
        for attempt in range(60):          # the autopilot may still be starting
            try:
                self.mav = mavutil.mavlink_connection(mav_url, source_system=255,
                                                      source_component=191)
                break
            except ConnectionRefusedError:
                time.sleep(1.0)
        else:
            raise RuntimeError(f"{agent_id}: no MAVLink connection at {mav_url}")
        self._tx = threading.Lock()
        self._stop = threading.Event()
        self.att = None                        # last ATTITUDE (roll, pitch, yaw)
        self.vb = 0.0
        self.seq = 0
        self.n_pos = 0
        # metrics
        self.t_ready = None
        self.msgs_out = self.bytes_out = 0
        self.sample_age = []       # arrival of GLOBAL_POSITION_INT -> MQTT publish (s)
        self.act_latencies = []    # core publish -> adapter receive (s)
        self.swarm = []            # (trig_id, latency s) of corrective commands
        self.cmd_sent = 0          # SET_ATTITUDE_TARGET sent to the autopilot
        self.cmd_discarded = 0     # commands older than T_f
        self.act_to_mav = []       # adapter receive -> MAVLink send (s)
        self.track = []            # (t, lat, lon, speed m/s, yaw rad) at 8.33 Hz

        self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                               client_id=f"adapter-{agent_id}", protocol=mqtt.MQTTv5)
        self.cli.on_message = self._on_act
        self.cli.connect(mqtt_host, mqtt_port)
        self.cli.socket().setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.cli.subscribe(f"missiondt/agents/{agent_id}/actuation", qos=0)
        self.cli.loop_start()

    # ------------------------------------------------------------ MAVLink setup
    def _interval(self, msg_id, hz):
        with self._tx:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def prepare(self, timeout_s=180.0):
        """Wait for the autopilot and a valid EKF position, set the telemetry
        rates, switch to GUIDED, and arm."""
        t_end = time.time() + timeout_s
        self.mav.wait_heartbeat(timeout=timeout_s)
        while time.time() < t_end:
            m = self.mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1)
            # attitude, horizontal velocity, absolute horizontal position; no GPS glitch
            if m and (m.flags & 0x01) and (m.flags & 0x02) and (m.flags & 0x10) \
                    and not (m.flags & 0x8000) and not (m.flags & 0x400):
                break
        else:
            raise RuntimeError(f"{self.aid}: EKF not ready")
        self._interval(MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, SENSOR_HZ)
        self._interval(MAV.MAVLINK_MSG_ID_ATTITUDE, SENSOR_HZ)
        self._interval(MAV.MAVLINK_MSG_ID_SYS_STATUS, 2)
        while time.time() < t_end:
            with self._tx:
                self.mav.set_mode("GUIDED")
                self.mav.arducopter_arm()
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.get_srcSystem() == self.mav.target_system and \
                    (hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED) and \
                    mavutil.mode_string_v10(hb) == "GUIDED":
                break
        else:
            raise RuntimeError(f"{self.aid}: not armed in GUIDED")
        self._send_cmd(0.0, 0.0)
        self.t_ready = time.time()
        self.cli.publish(f"missiondt/agents/{self.aid}/register",
                         json.dumps({"domain": self.domain, "kind": "physical"}),
                         qos=1, retain=True)

    # ------------------------------------------------------------ actuation
    def _send_cmd(self, tau, alpha):
        rate = YAW_GAIN[self.domain] * alpha
        with self._tx:
            self.mav.mav.set_attitude_target_send(
                0, self.mav.target_system, self.mav.target_component, ATT_MASK,
                [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, rate, max(-1.0, min(1.0, tau)))

    def _on_act(self, cli, ud, msg):
        now = time.time()
        a = json.loads(msg.payload)
        age = now - a["t_pub"]
        self.act_latencies.append(age)
        if age > T_F:
            self.cmd_discarded += 1
            return
        if a.get("avoid") and a.get("trig_t"):
            self.swarm.append((a.get("trig_id"), now - a["trig_t"]))
        self._send_cmd(a["tau"], a["alpha"])
        self.cmd_sent += 1
        self.act_to_mav.append(time.time() - now)

    # ------------------------------------------------------------ telemetry
    def _publish(self, gpi, t_rx):
        yaw = self.att[2] if self.att else math.radians(gpi.hdg / 100.0)
        vn, ve = gpi.vx / 100.0, gpi.vy / 100.0
        u = vn * math.cos(yaw) + ve * math.sin(yaw)          # body forward
        v = -vn * math.sin(yaw) + ve * math.cos(yaw)         # body right
        lat, lon = gpi.lat / 1e7, gpi.lon / 1e7
        self.seq += 1
        p = json.dumps({
            "t_pub": time.time(), "seq": self.seq,
            "gps": [lat, lon, gpi.relative_alt / 1000.0],
            "att": [self.att[0], self.att[1], yaw] if self.att else [0.0, 0.0, yaw],
            "vel": [u, v, gpi.vz / 100.0],
            "vb": self.vb, "t_boot_ms": gpi.time_boot_ms,
        })
        self.cli.publish(f"missiondt/agents/{self.aid}/telemetry", p, qos=0)
        self.sample_age.append(time.time() - t_rx)
        self.msgs_out += 1
        self.bytes_out += len(p)
        self.track.append((time.time(), lat, lon, math.hypot(vn, ve), yaw))

    def run(self):
        while not self._stop.is_set():
            m = self.mav.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            t = m.get_type()
            if t == "ATTITUDE":
                self.att = (m.roll, m.pitch, m.yaw)
            elif t == "SYS_STATUS":
                self.vb = m.voltage_battery / 1000.0
            elif t == "GLOBAL_POSITION_INT":
                t_rx = time.time()
                self.n_pos += 1
                if self.n_pos % self.publish_every == 0:
                    self._publish(m, t_rx)

    def stop(self):
        self._stop.set()

    def shutdown(self):
        self.cli.unsubscribe(f"missiondt/agents/{self.aid}/actuation")
        self._send_cmd(0.0, 0.0)
        with self._tx:
            self.mav.set_mode("HOLD")
            self.mav.arducopter_disarm()
        self.cli.publish(f"missiondt/agents/{self.aid}/register", payload=b"",
                         qos=1, retain=True)
        time.sleep(0.2)
        self.cli.loop_stop()
        self.cli.disconnect()

    def metrics(self):
        return {"id": self.aid, "domain": self.domain, "kind": "physical",
                "t_ready": self.t_ready, "msgs_out": self.msgs_out,
                "bytes_out": self.bytes_out, "cmd_sent": self.cmd_sent,
                "cmd_discarded": self.cmd_discarded,
                "raw_sample_age_ms": [x * 1e3 for x in self.sample_age],
                "raw_act_lat_ms": [x * 1e3 for x in self.act_latencies],
                "raw_act_to_mav_ms": [x * 1e3 for x in self.act_to_mav],
                "swarm": [(j, x * 1e3) for j, x in self.swarm],
                "track": self.track}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", required=True)
    ap.add_argument("--mavlink", required=True)
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--domain", default="surface", choices=("surface", "aerial"))
    ap.add_argument("--publish-all", action="store_true",
                    help="publish every 50 Hz position sample (default: one of every six, 8.33 Hz)")
    ap.add_argument("--metrics")
    args = ap.parse_args()
    ad = MavlinkAdapter(args.id, args.mavlink, args.mqtt_host, args.mqtt_port,
                        args.domain, not args.publish_all)
    signal.signal(signal.SIGTERM, lambda *_: ad.stop())
    signal.signal(signal.SIGINT, lambda *_: ad.stop())
    ad.prepare()
    print(f"[adapter {args.id}] ready", flush=True)
    ad.run()
    ad.shutdown()
    if args.metrics:
        with open(args.metrics, "w") as f:
            json.dump(ad.metrics(), f)
    print(f"[adapter {args.id}] stopped: {ad.msgs_out} telemetry, {ad.cmd_sent} commands, "
          f"{ad.cmd_discarded} discarded", flush=True)


if __name__ == "__main__":
    main()
