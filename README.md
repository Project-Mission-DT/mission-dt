# Mission-DT

**Mission-level Digital Twin for hybrid fleets of physical and virtual
unmanned vehicles** (aerial + surface), built on MQTT with a lightweight
Python core, a decoupled 3D mission view, and a live agent panel.
<br>by Filipo Novo Mór

Companion code for the paper *"A Mission-Level Digital Twin for Hybrid
Fleets of Physical and Virtual Unmanned Vehicles"*. All numbers in the
paper come from the five runs in `results/rep1/` to `results/rep5/` (raw
measurements included) and their aggregate in `results/summary.json`.

## Key ideas
- The **mission** is the twinned entity with state `M^t = <B^t, φ^t, g^t>` (agent states,
  Mission Context, agent goals). Once per 125 ms frame, the mission transition `Δ^e`
  composes `δ^e` (state update), `Φ` (context), `σ` (goals) and `λ` (actuation), in this
  order (`mission_dt/model.py`).
- **Hybrid agents**: each agent is a physical vehicle (ArduPilot bridged
  to MQTT) or a **virtual agent — an independent digital twin** with its
  own model, state and MQTT connection. The mission core cannot tell
  them apart; agents can move to other processes/machines unchanged.
- **Swarm coordination**: the Mission Context φ (horizontal distance to the
  nearest same-domain neighbour) triggers corrective actuation. E3 measures
  the time a corrective command takes to reach the neighbours. The runtime
  gives no timing guarantee.
- **Bandwidth regulators** publish one of every six 50 Hz cycles
  (8.33 Hz, about 6x less uplink).

## Requirements
- Python 3.10+ · Mosquitto MQTT broker (localhost); for the paper results,
  start it with `mosquitto -c mosquitto.conf` (`set_tcp_nodelay true`)
- `pip install -r requirements.txt`
- 3D view (GPU machine): `pip install ursina imageio imageio-ffmpeg`
- Panel (optional pretty mode): `pip install rich`

## Quick start
The launcher checks your virtual environment, starts the broker if it
isn't already running, clears any leftover retained MQTT state, and
opens the mission plus the 3D view in one go:
```bash
./run_mission.sh                              # default mission
./run_mission.sh configs/mission_photo.json   # any other mission file
```
Press **ESC** in the 3D view to stop everything cleanly.

Or, step by step, in separate terminals (useful if you want the live
terminal panel running alongside):
```bash
mosquitto -v                                   # terminal 1 (broker)
python experiments/demo_mission.py             # terminal 2 (mission)
python viz/mission_viz.py                      # terminal 3 (3D view)
python experiments/panel.py                    # terminal 4 (dashboard)
```

## Mission configuration
`configs/mission_default.json` controls fleet size and routes:
```json
{ "profile": "orbitas",
  "drones": { "aerial": 5, "surface": 5 },
  "radius_m": 45.0, "arrive_m": 3.0, "aerial_alt_m": 15.0,
  "checkpoints": {}, "assignments": {} }
```
**Profiles** (auto-generated routes): `orbitas` (antipodal patrol),
`paralelo` (parallel lanes), `explorador` (aerial drones scout a moving
point ahead of their paired vessel), `aleatorio` (random waypoints
converging on a common destination, regenerated every lap).
**Explicit routes**: define labelled `checkpoints` (P01…, `[lat, lon,
alt_m]`; alt>0 air corridor, 0 surface, <0 submerged) and per-drone
`assignments` — checkpoints may be shared. See
`configs/mission_checkpoints_example.json` for a small hand-written
example, or `configs/mission_photo.json` for a denser 6-agent, 10-
checkpoint mission (this is the one used to compose the paper's Figure
7). Checkpoints and planned routes are published retained on MQTT, so
the 3D view labels them with no config file of its own.

## Staging a screenshot deterministically
`experiments/staged_photo.py` is a different kind of script: instead of
generating a route and waiting for an interesting moment to occur
live, it places every agent directly at a chosen pose and holds it
there (goal = current position), so the safety-sphere states you want
(white/orange/red) are present and stable from the very first frame —
no timing, no luck, no waiting. One agent still patrols normally in
the background, to also show the ROTA route-line feature against an
otherwise-frozen scene:
```bash
python experiments/staged_photo.py     # terminal 1 — scene freezes instantly
python viz/mission_viz.py              # terminal 2 — screenshot whenever
```
Edit the coordinate constants near the top of the file to change the
composition (cluster spacing, which pairs are in conflict, etc.).

## 3D view controls
| Button / key | Action |
|---|---|
| FOTO / `P` | screenshot → `captures/` |
| REC / `R` | record MP4 (H.264, YouTube/Vimeo-ready) → `captures/` |
| HQ / `H` | toggle 30 fps high / 15 fps low quality |
| ID / `I` | drone name labels |
| ROTA / `K` | planned-route lines (green; grey dots = past trail) |
| LEG / `L` | translucent legend |
| PANEL / `O` | floating agent panel inside the 3D window (reuses `panel.py`) |
| `G` / `ESC` | sky grid / quit |

Status cues: agent turns **grey** after 1.5 s without telemetry;
pulsing **red** marker = battery below 17.6 V; safety spheres (12 m
true-scale) — white on approach (<24 m), orange in conflict (<12 m),
red on near-collision (<3 m).

## Live monitoring
Besides the in-view floating panel, `experiments/panel.py` runs as a
standalone terminal dashboard — a plain, independent MQTT client, so it
works on any machine that can reach the broker, with or without the 3D
view running:
```bash
python experiments/panel.py
```
Shows one row per agent (position, altitude, speed, battery, telemetry
rate, status) twice a second. Renders as a colour-coded live table if
`rich` is installed, otherwise falls back to plain printed rows.

## Experiments (reproduce the paper)
The whole battery, one command:
```bash
bash run_all.sh              # one run, into results/
REPS=5 bash run_all.sh       # five runs, into results/rep1/ to results/rep5/ (paper setting)
```
Checks your virtual environment (by locating its python interpreter
directly, rather than relying on `source activate` — more robust
across Git Bash on Windows), starts the broker if needed, clears
retained state, then runs E1 through E5 and regenerates the figures in
sequence. With `REPS` above 1, repetition r writes its JSON files and
figures into `results/rep<r>/`, and `experiments/aggregate.py` then writes
`results/summary.json` and `results/summary.md`. One repetition takes
about 15 min.

| Experiment | Script | Output file |
|---|---|---|
| E1 scalability, N = 1 to 100 agents, regulator on | `run_experiments.py` | `e1_scalability.json` |
| E2 regulator on and off, N = 10 | `run_experiments.py` | `e2_regulator.json` |
| E3 swarm propagation latency, N = 10, 25, 50 | `run_e3.py` | `e3_swarm.json` |
| E4 twin fidelity at 0, 5 and 10% loss, N = 10, regulator on | `run_e4.py` | `e4_fidelity.json` |
| E4 with the regulator off | `run_e4.py --no-regulator` | `e4_fidelity_noreg.json` |
| E5 CPU and peak memory of the Mission-DT process, N = 0, 10, 50, 100 | `run_e5.py` | `e5_resources.json` |

Or step by step:
```bash
python experiments/run_experiments.py all      # E1 scalability + E2 regulators (~6 min)
python experiments/run_e3.py                   # E3 swarm propagation latency
python experiments/run_e4.py                   # E4 twin fidelity at 0/5/10% loss
python experiments/run_e4.py --no-regulator    # E4 with the regulator off
python experiments/run_e5.py                   # E5 CPU and memory of the core process
python experiments/make_figures.py             # figures into results/
python experiments/aggregate.py results/rep1 results/rep2 ...   # summary over repetitions
```
Every script reads and writes the directory in `MDT_RESULTS` (default
`results/`), e.g. `MDT_RESULTS=results/rep1 python experiments/run_e3.py`.
`run_all.sh` starts Mosquitto with `mosquitto.conf` when no broker is running
and pins the broker and the experiments to core 0 with `taskset` when available.
A broker that is already running keeps its own configuration.

### Measurements
| Quantity | Definition |
|---|---|
| Frame time (`frame_ms`) | Time from the start of the frame (collection of the inputs I^t) to the return of the `publish` call of the last actuation. The `on_frame` and 3D-view hooks run after the measurement. A frame overrun is a frame time above 125 ms. |
| Release jitter (`release_jitter_ms`) | Actual start of a frame minus its scheduled start. The schedule advances 125 ms per frame and restarts from the current time after an overrun. |
| Core CPU (`core_cpu_pct`, E1 to E3) | User + system CPU time of the frame loop thread (`time.thread_time`) plus the paho-mqtt network thread (`/proc/self/task/<tid>/stat`, Linux only, resolution one clock tick) over the run duration, in % of one core. The virtual agents run as threads of the same process and are not counted. `core_cpu_s` (E1, E2) gives the two threads apart. |
| Process CPU (`cpu_pct`, E5) | User + system CPU time of the whole Mission-DT process over wall time, in % of one core. The agents run in a second process; N = 0 is the core connected with no agent. |
| Peak memory (`peak_rss_mib`, E5) | Peak resident set of the Mission-DT process (`VmHWM` on Linux, `ru_maxrss` elsewhere). |
| Repetition statistics (`summary.*`) | Mean and sample standard deviation over the per-run values of each metric; *pooled* columns use all raw samples of all repetitions, with nearest-rank percentiles. |

Results of the revised paper (`results/rep1/` to `results/rep5/`, five runs,
Linux VM, Xeon 2.10 GHz, broker and experiments on core 0): <TBD overruns>
frame overruns up to 100 agents (longest frame <TBD> ms); regulators cut uplink
<TBD>x and redundant samples <TBD>x; corrective commands reach neighbours with a
median of <TBD> ms and at most <TBD> ms; position RMSE <TBD> m with the regulator
and <TBD> m without it up to 10% injected loss; the Mission-DT process uses
<TBD>% of one core and <TBD> MiB at 100 agents.
`results/README.md` describes each results directory, including a Windows run
of the original core with the default Mosquitto configuration (Nagle's
algorithm enabled) in `results/windows_original_code/`.

`experiments/clear_retained.py` clears leftover retained MQTT state
(ghost agent registrations, stale checkpoints/routes) — run it before
experiments if you've been poking around the broker manually;
`run_mission.sh` already calls it automatically.

## Maintainers: checking your checkout
```bash
bash check_uptodate.sh
```
Greps your local files for markers of every major feature (swarm
separation, retained registration, the experiment scripts, the 3D
view's checkpoints/routes/panel support, …) and reports what's missing
— a quick sanity check after pulling or before reporting an issue.

## Repository layout
```
mission_dt/       model.py (Δ^e and its functions), core.py (MQTT I/O),
                  agents.py (virtual agents), core_orig.py (original core)
experiments/      demo_mission, staged_photo, run_experiments (E1/E2),
                  run_e3, run_e4, run_e5, aggregate, panel, make_figures,
                  clear_retained
viz/              3D mission view (Ursina)
configs/          mission configuration files
results/          rep1/ to rep5/ (raw measurements, JSON), summary.json/.md,
                  logs; see results/README.md
tests/            test_equivalence.py (original core vs. model.py)
mosquitto.conf    broker configuration of the paper runs
run_mission.sh    one-command launcher (broker + mission + 3D view)
run_all.sh        one-command experiment battery (E1-E5 + figures)
check_uptodate.sh maintainer script: verifies a checkout is current
requirements.txt  Python dependencies
```

## Connecting real vehicles
Run a MAVLink→MQTT bridge on the companion computer (e.g. Raspberry
Pi 4 + ArduPilot): publish telemetry on
`missiondt/agents/<id>/telemetry`, consume
`missiondt/agents/<id>/actuation`. The local broker bridges to the
ground station over Wi-Fi (see the paper, Section III).

## License
MIT (see `LICENSE`).