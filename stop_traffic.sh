#!/bin/bash
# Stop only the traffic/chaos spawner (keeps CARLA + your driving window). NPCs are removed by the spawner on exit.
pkill -INT -f "traffic_nea[r].py" && echo "stopping traffic ..." || echo "no traffic spawner running"
