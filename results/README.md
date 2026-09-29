# Results

| Directory | Content |
|---|---|
| `results/rep1/` to `results/rep5/` | E1 to E5 of the revised paper: five runs of the battery with the revised core, `mosquitto.conf` (`set_tcp_nodelay true`) and `TCP_NODELAY` on the core and agent sockets. `summary.json` and `summary.md` aggregate the five runs (`experiments/aggregate.py`). The console log of the five runs is `logs/reps.log`. |
| `results/other_instance/` | Two runs of the same code on a second instance of the same VM type, before the instance restarted; the second run stops after E3. Console log in `other_instance/reps.log`. |
| `results/single_run/` | One earlier run of E1 to E4 with the revised core and `TCP_NODELAY` on all sockets, before the instrumentation hooks moved out of the timed frame interval. Logs in `single_run/logs/`. |
| `results/rerun_client_nagle/` | One run with the revised core and the broker setting, before `TCP_NODELAY` on the client sockets (actuation p99 near 43 ms). |
| `results/linux_original_code/` | Five runs of E1 to E4 of the original code (the code of the submitted paper, from `main` before this branch) on the Linux host of `results/rep*/`, with the default Mosquitto configuration (Nagle's algorithm enabled) and every process pinned to core 0. These runs reproduce the values of the submitted paper: median telemetry latency at N = 100 of 38.2 ms (submitted: 38.5 ms), E3 median of 118 to 119 ms (118 ms), E4 position RMSE of 0.89 to 0.91 m (0.88 to 0.92 m). Console log in `logs/reps.log`. |
| `results/windows_original_code/` | One run of E1 to E4 on Windows (Intel i7-9700) with the original core and the default Mosquitto configuration (Nagle's algorithm enabled), with its figures. `platform-b/` holds E1 and E2 of one run of the original core on a second machine. |

The JSON files of the runs behind the submitted paper and the macOS runs are not in this repository; `results/linux_original_code/` reproduces the Linux values of the submitted paper.

Files of one repetition directory:

| File | Experiment |
|---|---|
| `e1_scalability.json` | E1 scalability, N = 1 to 100 |
| `e2_regulator.json` | E2 regulator on and off, N = 10 |
| `e3_swarm.json` | E3 swarm propagation latency, N = 10, 25, 50 |
| `e4_fidelity.json` | E4 twin fidelity at 0, 5 and 10% loss, regulator on |
| `e4_fidelity_noreg.json` | E4 with the regulator off |
| `e5_resources.json` | E5 CPU and peak memory of the Mission-DT process, N = 0, 10, 50, 100 |
| `e6_comparison.json` | E6 CPU and memory per process of Mission-DT (MQTT), ROS 2 and Gazebo, N = 10, 50, 100 |

The `raw_*` arrays hold every sample (ms). In E4, each row of `rows_dom_m_err_hold` holds the domain, the stale streak m, the position error of the twin (m) and the position error of a hold of the last received state (m) for one agent frame. The main `README.md` defines frame time, release jitter and CPU accounting.

E6 (`e6_comparison.json` in `results/rep1/` to `results/rep5/`) ran after E1 to E5 on a Linux VM of the same type (kernel 6.18, Intel Xeon 2.10 GHz, 2 vCPU, every process on core 0), with Python 3.12.3 for every stack, ROS 2 Jazzy (rclpy 7.1.12, `rmw_fastrtps_cpp` 8.4.4, Fast DDS 2.14.6) and Gazebo Harmonic (gz-sim 8.15.0, gz-physics 7.8.0 with DART, gz-transport 13.6.0); each record holds one stack at one N, with `processes.<name>.cpu_pct`, `peak_rss_mib` (VmHWM), `rss_end_mib` (VmRSS at the end of the window) and `threads`, the mission-layer counters of the window (`mission`, `agents`) and, for Gazebo, the real-time factor (`gazebo.rtf`). The console log is `logs/e6_reps.log`. The main `README.md` describes the three stacks. In every Gazebo run the monitor received the odometry of all N vehicles (`rep1` at N = 100 after renewing 89 subscriptions that had not matched, `gazebo.odom_subscriptions_renewed`).

Platform of `results/rep*/`, `results/other_instance/`, `results/single_run/` and `results/rerun_client_nagle/`: Linux VM (kernel 6.18), Intel Xeon 2.10 GHz, 2 vCPU, broker and experiments pinned to core 0 (`taskset -c 0`), Python 3.11.15 (E1 to E5), paho-mqtt 2.1.0, Mosquitto 2.0.18.
