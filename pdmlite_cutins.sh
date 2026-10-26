#!/bin/bash
# PDM-Lite (privileged expert) on the same scenes as WA-JEPA: plain 2286, fast cut-in, lane invader + pedestrian, hand-driven scene1.
WS=/home/all/carla_ws
bash $WS/pdmlite_runs.sh "2286 2709"
SET="HighwayCutIn.cut_in_distance=4,HighwayCutIn.speed_perc=120" EXTEND=250 TAG=_FASTCUT bash $WS/pdmlite_runs.sh "2286"
SET="DynamicObjectCrossing.invader=1,DynamicObjectCrossing.invader_dist=50,DynamicObjectCrossing.invader_trigger=50,DynamicObjectCrossing.invader_speed=9,DynamicObjectCrossing.invader_offset=2.0,DynamicObjectCrossing.invader_start=0.3,DynamicObjectCrossing.invader_in=1.0,DynamicObjectCrossing.invader_hold=2.0,DynamicObjectCrossing.invader_out=1.2" TAG=_LaneInvader_PED bash $WS/pdmlite_runs.sh "24211"
SET="DynamicObjectCrossing.npc_track=/home/all/carla_ws/npc_tracks/scene1_run.json" TAG=_Track_scene1 bash $WS/pdmlite_runs.sh "24211"
echo "[pdmlite cutins done] $(date +%H:%M)"
