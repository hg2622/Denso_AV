#!/bin/bash
# BridgeDrive drives itself around an open world on a random GPS route (server must be running: bash carla_server.sh [-windowed]).
#   bash bd_drive.sh [--map Town04] [--dist 600] [--no-dense] [--sim-rate 3|0] [--sync]   (default real time; --sim-rate 3 = fixed slow clock; --sync = lock-step) [--sim-rate 3|0] [--sync]   (default real time; --sim-rate 3 = fixed slow clock; --sync = lock-step)

# Opens a viewer window: chase cam + top-down, green = the ~50 m GPS points the agent gets, red/orange = current/next target, blue = its predicted path.
export PATH=/home/all/miniconda3/envs/lead/bin:$PATH
exec /home/all/miniconda3/envs/lead/bin/python -u /home/all/carla_ws/bd_freeroam.py "$@"
