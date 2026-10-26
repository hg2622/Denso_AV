#!/bin/bash
# Overnight batch: SimLingo (original: sentence every tick), Drive-pi0, DriveMoE on cut-in / crossing / merge routes.
# Each run -> selected_runs/night/<model>/<route>_<type>_DS<score>.mp4 (annotated: path, text, HUD, speed strip), .png speed profile,
# trace.jsonl, meta.jsonl, raw.mp4, and a line in selected_runs/night/RESULTS.txt.   bash night_runs.sh [models] [routes]
WS=/home/all/carla_ws; OUT=$WS/selected_runs/night; mkdir -p $OUT; RES=$OUT/RESULTS.txt; REC=/home/all/miniconda3/envs/lead/bin/python
MODELS=${1:-"simlingo drivepi0 drivemoe"}; ROUTES=${2:-"2286 26396 24211 24759 2273 27506 23658"}
declare -A TYPE=([2286]=HighwayCutIn [26396]=StaticCutIn [24211]=DynamicObjectCrossing [24759]=ParkingCutIn [2273]=MergerIntoSlowTraffic [27506]=EnterActorFlow [23658]=HighwayExit)
score_line() { /home/all/miniconda3/envs/lead/bin/python - "$1" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
print(f"DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
PY
}
for M in $MODELS; do for R in $ROUTES; do
  D=$OUT/$M/$R; mkdir -p $D; rm -f $D/trace.jsonl $D/meta.jsonl
  case $M in
    simlingo) PORT=8848; CK=$WS/SimLingo/outputs/eval_b2d/$R/checkpoint_endpoint.json; YL="--y-left"
              (AGENT_TRACE=$D/trace.jsonl SL_LANG_EVERY=1 bash $WS/run_simlingo_eval.sh $R > $D/eval.log 2>&1 &) ;;
    drivepi0|drivemoe) PORT=8858; CK=$WS/DriveMoE/outputs/eval_b2d/$M/$R/checkpoint_endpoint.json; YL=""
              (AGENT_TRACE=$D/trace.jsonl systemd-run --user --scope -q -p MemoryMax=22G -p MemorySwapMax=0 bash $WS/run_pi0_eval.sh $R $M $PORT > $D/eval.log 2>&1 &) ;;
  esac
  echo "[$(date +%H:%M)] $M $R (${TYPE[$R]}) started"
  $REC $WS/record_hero.py --port $PORT --out $D/raw.mp4 --meta $D/meta.jsonl --label "$M $R ${TYPE[$R]}" --wait 900 > $D/rec.log 2>&1
  for i in $(seq 1 400); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done
  pkill -9 -f "rpc-port=$PORT " 2>/dev/null; pkill -9 -f "leaderboard_evaluato[r]" 2>/dev/null; sleep 2
  if [ -f "$CK" ]; then S=$(score_line $CK); else S="NO RESULT (see eval.log)"; fi
  DS=$(echo "$S" | grep -oE "DS=[0-9]+" | cut -d= -f2); NAME=${R}_${TYPE[$R]}_DS${DS:-NA}
  [ -f "$CK" ] && cp $CK $D/checkpoint_endpoint.json
  if [ -s $D/trace.jsonl ] && [ -s $D/meta.jsonl ]; then
    $REC $WS/annotate_run.py --video $D/raw.mp4 --meta $D/meta.jsonl --trace $D/trace.jsonl --out $OUT/$M/$NAME.mp4 --label "$M $R ${TYPE[$R]}" --score "$S" $YL > $D/annotate.log 2>&1 && A=$(tail -1 $D/annotate.log) || A="annotate FAILED"
  else A="no trace/meta"; fi
  echo "$M $R ${TYPE[$R]}: $S | $A" | tee -a $RES
done; done
bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; echo "[all done] $(date +%H:%M)" | tee -a $RES
