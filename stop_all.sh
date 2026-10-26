#!/bin/bash
# Kill everything: CARLA server, Bench2Drive evaluator, driving window, traffic spawner.
pkill -f "traffic_nea[r].py"; pkill -f "manual_contro[l].py"; pkill -f "generate_traffi[c].py"; pkill -INT -f "bd_freeroa[m].py"; pkill -INT -f "carl_freeroa[m].py"; pkill -f "side_view[s].py"; sleep 2; pkill -9 -f "bd_freeroa[m].py"; pkill -9 -f "carl_freeroa[m].py"
pkill -9 -f "leaderboard_evaluato[r].py"; sleep 1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; sleep 1
pgrep -fa "CarlaUE4-Linux-Shipp[i]ng|leaderboard_evaluato[r]|manual_contro[l]|traffic_nea[r]|bd_freeroa[m]|carl_freeroa[m]" >/dev/null && echo "something survived" || echo "all stopped"
