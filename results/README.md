# Results

| Directory | Content |
|---|---|
| `results/rep1/` to `results/rep5/` | E1 to E4 of the paper and the publication-rate measurement: five runs of the battery with the revised core, `mosquitto.conf` (`set_tcp_nodelay true`) and `TCP_NODELAY` on the core and agent sockets; E5 (`e6_comparison.json`) and E6 (`e7_hybrid.json`) ran after the battery. `summary.json` and `summary.md` aggregate the five runs (`experiments/aggregate.py`). The console log of the five runs of the battery is `logs/reps.log`. |
| `results/e3_50hz_n50_extra/` | Five more runs of E2 at 50 Hz publication and N = 50 (`run1/` to `run5/`, file `e3_swarm_50hz.json`); console log `logs/e3_50hz_n50_extra.log`. |
| `results/other_instance/` | Two runs of the same code on a second instance of the same VM type, before the instance restarted; the second run stops after E2. Console log in `other_instance/reps.log`. |
| `results/single_run/` | One earlier run of E1 to E3 and of the publication-rate measurement with the revised core and `TCP_NODELAY` on all sockets, before the instrumentation hooks moved out of the timed frame interval. Logs in `single_run/logs/`. |
| `results/rerun_client_nagle/` | One run with the revised core and the broker setting, before `TCP_NODELAY` on the client sockets (actuation p99 near 43 ms). |
| `results/linux_original_code/` | Five runs of E1 to E3 and of the publication-rate measurement with the original code (the code of the submitted paper, from `main` before this branch) on the Linux host of `results/rep*/`, with the default Mosquitto configuration (Nagle's algorithm enabled) and every process pinned to core 0. These runs reproduce the values of the submitted paper: median telemetry latency at N = 100 of 38.2 ms (submitted: 38.5 ms), E2 median of 118 to 119 ms (118 ms), E3 position RMSE of 0.89 to 0.91 m (0.88 to 0.92 m). Console log in `logs/reps.log`. |
| `results/windows_original_code/` | One run of E1 to E3 and of the publication-rate measurement on Windows (Intel i7-9700) with the original core and the default Mosquitto configuration (Nagle's algorithm enabled), with its figures. `platform-b/` holds E1 and the publication-rate measurement of one run of the original core on a second machine. |

The JSON files of the runs behind the submitted paper and the macOS runs are not in this repository; `results/linux_original_code/` reproduces the Linux values of the submitted paper.

Files of one repetition directory (the file names do not follow the experiment numbers of the paper):

| File | Paper experiment | Content |
|---|---|---|
| `e1_scalability.json` | E1 | Frame compute time, N = 1 to 100, publication at 8.33 Hz |
| `e3_swarm.json` | E2 | Swarm-reaction latency, N = 10, 25, 50, publication at 8.33 Hz |
| `e3_swarm_50hz.json` | E2 | Swarm-reaction latency, N = 10, 25, 50, publication at 50 Hz |
| `e4_fidelity.json` | E3 | Twin fidelity at 0, 5 and 10% loss, publication at 8.33 Hz |
| `e4_fidelity_50hz.json` | E3 | Twin fidelity at 0, 5 and 10% loss, publication at 50 Hz |
| `e5_resources.json` | E4 | CPU and peak memory of the Mission-DT process, N = 0, 10, 50, 100 |
| `e6_comparison.json` | E5 | CPU and memory per process of Mission-DT (MQTT), ROS 2 and Gazebo, N = 10, 50, 100 |
| `e7_hybrid.json` | E6 | Hybrid fleet: 2 ArduRover SITL boats and 8 virtual agents |
| `e2_publication_rate.json` | not in the paper | Additional publication-rate measurement: N = 10 agents publishing at 8.33 Hz and at 50 Hz (uplink traffic, messages per second, redundant samples, latencies) |

The `raw_*` arrays hold every sample (ms). The boolean field `decimate` of `e1_scalability.json`, `e2_publication_rate.json`, `e3_swarm*.json` and `e4_fidelity*.json` records the publication rate: `true` for publication at 8.33 Hz (one of every six 50 Hz samples), `false` for publication at 50 Hz (every sample). In E3, each row of `rows_dom_m_err_hold` holds the domain, the stale streak m, the position error of the twin (m) and the position error of a hold of the last received state (m) for one agent frame. The main `README.md` defines frame time, release jitter and CPU accounting.

E2 at 50 Hz: `results/rep<r>/e3_swarm_50hz.json` (N = 10, 25, 50, five runs, console log `logs/e3_50hz.log`) and `results/e3_50hz_n50_extra/run1..run5/e3_swarm_50hz.json` (five more runs at N = 50). At N = 50 the 2,500 messages per second saturate the shared core, and the MQTT queues grew in 4 of these 10 runs (latencies of 1.1 s to 31.7 s). Command: `taskset -c 0 python experiments/run_e3.py --publish-all [N ...]`.

E5 (`e6_comparison.json` in `results/rep1/` to `results/rep5/`) ran after E1 to E4 on a Linux VM of the same type (kernel 6.18, Intel Xeon 2.10 GHz, 2 vCPU, every process on core 0), with Python 3.12.3 for every stack, ROS 2 Jazzy (rclpy 7.1.12, `rmw_fastrtps_cpp` 8.4.4, Fast DDS 2.14.6) and Gazebo Harmonic (gz-sim 8.15.0, gz-physics 7.8.0 with DART, gz-transport 13.6.0); each record holds one stack at one N, with `processes.<name>.cpu_pct`, `peak_rss_mib` (VmHWM), `rss_end_mib` (VmRSS at the end of the window) and `threads`, the mission-layer counters of the window (`mission`, `agents`) and, for Gazebo, the real-time factor (`gazebo.rtf`). The console log is `logs/e6_reps.log`. The main `README.md` describes the three stacks. In every Gazebo run the monitor received the odometry of all N vehicles (`rep1` at N = 100 after renewing 89 subscriptions that had not matched, `gazebo.odom_subscriptions_renewed`).

Platform of `results/rep*/`, `results/other_instance/`, `results/single_run/` and `results/rerun_client_nagle/`: Linux VM (kernel 6.18), Intel Xeon 2.10 GHz, 2 vCPU, broker and experiments pinned to core 0 (`taskset -c 0`), Python 3.11.15 (E1 to E4 and E6), paho-mqtt 2.1.0, Mosquitto 2.0.18.

E6 (hybrid fleet): `results/rep<r>/e7_hybrid.json`, five runs of `experiments/run_e7.py` (2 ArduRover 4.7.1 SITL boats, motorboat model, and 8 virtual surface agents; console log `logs/e7_reps.log`). One MQTT broker runs at the ground station, and the MAVLink-to-MQTT adapters of the SITL boats and the virtual agents connect to it. Each record holds the frame times, the telemetry and actuation latencies by agent kind (`raw_telemetry_lat_ms`, `raw_actuation_lat_ms`, keys `physical` and `virtual`), the swarm-reaction latencies by receiver and trigger kind (`raw_swarm_lat_ms`, e.g. `physical<-virtual`), the adapter counters (`adapter.<id>.cmd_sent`, `cmd_discarded`, `sample_age_ms`), and the minimum separation and path of each SITL boat. Platform: the Linux VM of `results/rep*/`, ground-station broker, mission core and virtual agents on core 0, SITL instances and adapters on core 1, Python 3.11.15, pymavlink 2.4.50.
