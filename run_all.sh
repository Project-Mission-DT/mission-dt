#!/bin/bash
# Mission-DT -- run the full experiment battery (E1..E5 + figures).
# Usage, from anywhere:   bash run_all.sh            (one run, into results/)
#                         REPS=5 bash run_all.sh     (five runs, into results/rep1..rep5/)
# Cross-platform: finds the venv's python executable directly by path
# (Unix .venv/bin/python or Windows .venv/Scripts/python.exe), avoiding
# PATH/activation quirks seen with Git Bash on Windows.

set -e
cd "$(dirname "$0")"                      # repo root = script location

# 1) locate the venv's python interpreter directly (no PATH/activate needed)
if [ -f ".venv/bin/python" ]; then
    PYTHON="./.venv/bin/python"
elif [ -f ".venv/Scripts/python.exe" ]; then
    PYTHON="./.venv/Scripts/python.exe"
elif [ -n "$VIRTUAL_ENV" ]; then
    PYTHON="python"
    echo "Using already-active virtual environment: $VIRTUAL_ENV"
else
    echo "ERROR: .venv not found and no virtual environment is active."
    echo "  Create it first:  python3 -m venv .venv"
    exit 1
fi
echo "Using python: $PYTHON"
"$PYTHON" --version

# 1b) pin the broker and the experiments to one core when taskset exists
#     (Linux); the paper runs use core 0.
PIN=""
if command -v taskset >/dev/null 2>&1; then PIN="taskset -c 0"; fi

# 2) macOS default file-descriptor limit (256) is too low for 100 agents;
#    harmless no-op on Windows/Linux.
ulimit -n 4096 2>/dev/null || true

# 3) MQTT broker: check, and try to start it if needed
broker_up() {
    "$PYTHON" -c "import socket; socket.create_connection(('127.0.0.1', 1883), 2).close()" 2>/dev/null
}

if ! broker_up; then
    echo "Broker not running -- attempting to start Mosquitto..."
    if command -v mosquitto >/dev/null 2>&1; then
        nohup $PIN mosquitto -c mosquitto.conf >/tmp/mosquitto_run_all.log 2>&1 &
        sleep 2
    elif command -v brew >/dev/null 2>&1; then
        brew services start mosquitto >/dev/null 2>&1 || true
        sleep 2
    elif command -v net.exe >/dev/null 2>&1; then
        net.exe start mosquitto >/dev/null 2>&1 || true
        sleep 2
    fi
    if ! broker_up; then
        echo "ERROR: could not start an MQTT broker on 127.0.0.1:1883."
        echo "  Windows: start the 'Mosquitto Broker' service, or run"
        echo "           mosquitto.exe -v in another terminal."
        echo "  macOS:   brew install mosquitto"
        echo "  Linux:   sudo apt install mosquitto"
        exit 1
    fi
fi
echo "Broker OK."
echo "NOTE: a broker that was already running keeps its own configuration;"
echo "      the paper results use mosquitto.conf (set_tcp_nodelay true)."

echo "== Clearing retained ghosts from the broker =="
"$PYTHON" experiments/clear_retained.py

# REPS=5 bash run_all.sh repeats the battery; repetition r writes to
# results/rep<r>/ and experiments/aggregate.py reports mean and standard
# deviation over the repetitions. REPS=1 (default) writes to results/.
REPS="${REPS:-1}"
for r in $(seq 1 "$REPS"); do
    if [ "$REPS" -gt 1 ]; then export MDT_RESULTS="results/rep$r"; echo "== Repetition $r/$REPS =="; fi

    echo "== E1 + E2 (scalability + regulators, ~6 min) =="
    $PIN "$PYTHON" experiments/run_experiments.py

    echo "== E3 (swarm propagation, ~2.5 min) =="
    $PIN "$PYTHON" experiments/run_e3.py

    echo "== E4 (twin fidelity vs. packet loss, regulator on and off, ~4 min) =="
    $PIN "$PYTHON" experiments/run_e4.py
    $PIN "$PYTHON" experiments/run_e4.py --no-regulator

    echo "== E5 (CPU and memory of the Mission-DT process, ~2.5 min) =="
    $PIN "$PYTHON" experiments/run_e5.py

    echo "== Figures =="
    "$PYTHON" experiments/make_figures.py
done
unset MDT_RESULTS

if [ "$REPS" -gt 1 ]; then
    echo "== Aggregation over $REPS repetitions =="
    REP_DIRS=""
    for r in $(seq 1 "$REPS"); do REP_DIRS="$REP_DIRS results/rep$r"; done
    "$PYTHON" experiments/aggregate.py $REP_DIRS
    echo "All experiments done. Raw data and figures are in results/rep1..rep$REPS/;"
    echo "results/summary.json and results/summary.md hold the aggregate."
else
    echo "All experiments done. Raw data and figures are in results/."
fi
