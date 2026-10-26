#!/bin/bash
# Hazards around your car: cars spawn in a ring around you; inside 100 m every same-direction car speeds up, the ones behind
# or beside you overtake and cut in front of you, cars ahead brake-check, pedestrians run across. Normal traffic outside.
#   bash traffic_chaos.sh [N_CARS=60] [N_WALKERS=50] [SPAWN_RADIUS=200] [--chaos-radius 100] [--max-actors 3] [--max-walkers 3]
N=${1:-60}; W=${2:-50}; R=${3:-200}; shift 3 2>/dev/null
exec /home/all/miniconda3/envs/b2d/bin/python -u /home/all/carla_ws/traffic_near.py -n "$N" -w "$W" -r "$R" --follow --chaos "$@"
