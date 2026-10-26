#!/bin/bash
# Play BridgeDrive's debug video of a Bench2Drive route (cameras + LiDAR BEV + planned path + model state).
#   bash show_bd_video.sh 25378          # plays LEAD/outputs/local_evaluation/25378/25378_debug.mp4
R=${1:?route id}; F=/home/all/carla_ws/LEAD/outputs/local_evaluation/$R/${R}_debug.mp4
[ -f "$F" ] || { echo "no video for route $R (run: bash run_bridgedrive_eval.sh $R first)"; ls /home/all/carla_ws/LEAD/outputs/local_evaluation/ 2>/dev/null; exit 1; }
echo "$F"; (vlc --play-and-exit "$F" >/dev/null 2>&1 || xdg-open "$F") &
