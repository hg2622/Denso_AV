#!/bin/bash
# CaRL / Roach RL planners (autonomousvision/CaRL) drive around an open world (server must be running: bash carla_server.sh).
#   bash carl_drive.sh [--map Town04] [--model CaRL_v1_1_PY_01|CaRL_PY_00|Roach_00] [--dist 600] [--speed 1]
# Privileged map-based agents (no cameras): run lock-step at 20 Hz in real time. ESC / Ctrl-C stops.
export CUBLAS_WORKSPACE_CONFIG=:4096:8   # CaRL enables torch deterministic algorithms; cuBLAS needs this
exec /home/all/miniconda3/envs/carl/bin/python -u /home/all/carla_ws/carl_freeroam.py "$@"
