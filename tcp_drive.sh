#!/bin/bash
# TCP baseline drives around an open world by itself (server must be running: bash carla_server.sh).
#   bash tcp_drive.sh [--map Town10HD] [--max-speed 4] [--planner only_traj|merge_ctrl_traj]
# Default max-speed is Bench2Drive's own 1.5 m/s (~5 km/h crawl); 4 m/s (~15 km/h) drives sensibly.
# Watching: start the server with `bash carla_server.sh -windowed` — the spectator follows the car.
# Ctrl+C to stop.
exec /home/all/miniconda3/envs/b2d/bin/python -u /home/all/carla_ws/tcp_freeroam.py "$@"
