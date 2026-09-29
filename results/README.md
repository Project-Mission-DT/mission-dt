# Results

| Directory | Content |
|---|---|
| `results/` | E1 to E4 of the revised paper: revised core, `mosquitto.conf` (`set_tcp_nodelay true`), `TCP_NODELAY` on the core and agent sockets, one run per configuration. Logs in `logs/`. |
| `results/rerun_client_nagle/` | First rerun with the revised core and the broker setting, before `TCP_NODELAY` on the client sockets (actuation p99 near 43 ms). |
| `results/submitted/` | Data and figures of the submitted version: original core, default Mosquitto configuration. `platform-b/` holds the E1 and E2 runs of the second platform. |

Platform of `results/` and `results/rerun_client_nagle/`: Linux VM (kernel 6.18), Intel Xeon 2.10 GHz, 2 vCPU with the broker and the experiments pinned to core 0 (`taskset -c 0`), Python 3.11.15, paho-mqtt 2.1.0, Mosquitto 2.0.18.
