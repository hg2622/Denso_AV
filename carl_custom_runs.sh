#!/bin/bash
# CaRL on the 6 custom scenarios (same overrides as the BridgeDrive videos), recorded into selected_runs/custom/carl_*.mp4
WS=/home/all/carla_ws; OUT=$WS/selected_runs/custom; S=$WS/logs; REC=/home/all/miniconda3/envs/lead/bin/python
run() {  # name route SET EXTEND
  bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; sleep 3
  echo "[run] $1 (route $2, SET=$3, EXTEND=$4)"
  (SET="$3" EXTEND="$4" bash $WS/run_carl_eval.sh $2 CaRL_v1_1_PY_01 2000 > $S/carl_custom_$1.log 2>&1 &)
  $REC $WS/record_hero.py --port 2000 --out $OUT/carl_$1.mp4 --label "CaRL $1" --wait 600 > $S/carl_custom_$1.rec.log 2>&1
  for i in $(seq 1 60); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done
  echo "[done] $1: $($WS/../miniconda3/envs/carl/bin/python - <<PY
import json; d=json.load(open('$WS/CaRL/CARLA/results/eval_b2d/CaRL_v1_1_PY_01/$2.json'))['_checkpoint']['records'][0]
print(d['status'], 'DS=%.0f' % d['scores']['score_composed'], {k:len(v) for k,v in d['infractions'].items() if v and k!='min_speed_infractions'} or 'no infractions')
PY
)"
}
run 2286_HighwayCutIn_SYNC_then_CONST 2286 "HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=1" 250
run 2286_HighwayCutIn_CONST_MEET 2286 "HighwayCutIn.const_meet=1,HighwayCutIn.cut_in_distance=1" 250
run 2286_HighwayCutIn_NO_CUT_IN_fast 2286 "HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=1,HighwayCutIn.exit_speed=30" 250
run 24759_ParkingCutIn_NO_CUT_IN 24759 "ParkingCutIn.no_cut_in=1" 0
run 24211_DynamicObjectCrossing_NO_CROSS 24211 "DynamicObjectCrossing.no_cross=1" 0
run 26396_StaticCutIn_LATE_CUT 26396 "StaticCutIn.reaction_time=1.2,StaticCutIn.min_trigger_dist=6,StaticCutIn.change_time=1.5" 0
bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; echo "[all done]"
