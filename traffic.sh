#!/bin/bash
# Spawn NPC cars + pedestrians AROUND YOUR CAR and keep them around you. Ctrl+C removes them.
# Open the driving window first (drive.sh), then:
#   bash traffic.sh [N_CARS=60] [N_WALKERS=40] [RADIUS_M=250] [--lights]
N=${1:-60}; W=${2:-40}; R=${3:-250}; shift 3 2>/dev/null
exec /home/all/miniconda3/envs/b2d/bin/python /home/all/carla_ws/traffic_near.py -n "$N" -w "$W" -r "$R" --follow "$@"
