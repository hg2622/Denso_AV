#!/bin/bash
# WA-JEPA (HUGSIM controller) on the aggressive cut-in scenes.   bash wajepa_cutins.sh
WS=/home/all/carla_ws; export WAJEPA_CTRL=hugsim
SET="HighwayCutIn.cut_in_distance=4,HighwayCutIn.speed_perc=120" EXTEND=250 TAG=_FASTCUT_hugsim bash $WS/wajepa_runs.sh "2286"
SET="DynamicObjectCrossing.invader=1,DynamicObjectCrossing.invader_dist=50,DynamicObjectCrossing.invader_trigger=50,DynamicObjectCrossing.invader_speed=9,DynamicObjectCrossing.invader_offset=2.0,DynamicObjectCrossing.invader_start=0.3,DynamicObjectCrossing.invader_in=1.0,DynamicObjectCrossing.invader_hold=2.0,DynamicObjectCrossing.invader_out=1.2" TAG=_LaneInvader_PED_hugsim bash $WS/wajepa_runs.sh "24211"
SET="DynamicObjectCrossing.npc_track=/home/all/carla_ws/npc_tracks/scene1_run.json" TAG=_Track_scene1_hugsim bash $WS/wajepa_runs.sh "24211"
echo "[cutins done] $(date +%H:%M)"
