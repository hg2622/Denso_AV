#!/bin/bash
# Tour of all 44 Bench2Drive scenario types, one route each (tour44_routes.txt), with one model: run, record, annotate, score.
#   bash tour44.sh [MODEL=carl] [TYPES="HighwayCutIn DynamicObjectCrossing ..."]     models: carl bridgedrive simlingo drivepi0 drivemoe
# Output: selected_runs/tour44/<model>/NN_<type>_<route>_DS<score>.mp4 (+ .png speed profile) and RESULTS.txt; already finished types are
# skipped, so re-running the same command resumes the tour.  Stop with: bash tour44_stop.sh
WS=/home/all/carla_ws; M=${1:-carl}; ONLY=${2:-}; OUT=$WS/selected_runs/tour44/$M; WORK=$OUT/_work; mkdir -p $WORK; RES=$OUT/RESULTS.txt
REC=/home/all/miniconda3/envs/lead/bin/python
if pgrep -f "scene_driv[e].py" >/dev/null || pgrep -f "leaderboard_evaluato[r]" >/dev/null; then echo "something else is using CARLA (scene_drive / an evaluator) - stop it first"; exit 1; fi
score_line() { $REC - "$1" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
print(f"DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
PY
}
echo "[tour44 $M] start $(date '+%F %H:%M')" | tee -a $RES
grep -v '^#' $WS/tour44_routes.txt | while read -r N T R TOWN; do
  [ -n "$ONLY" ] && ! grep -qw "$T" <<<"$ONLY" && continue
  if ls $OUT/${N}_${T}_${R}_DS*.mp4 >/dev/null 2>&1; then echo "[skip] $N $T $R already done"; continue; fi
  D=$WORK/${N}_$T; rm -rf $D; mkdir -p $D; bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; sleep 3
  case $M in
    carl)        PORT=2000; CK=$WS/CaRL/CARLA/results/eval_b2d/CaRL_v1_1_PY_01/$R.json; TR=none; YL=""; rm -f $CK
                 (bash $WS/run_carl_eval.sh $R CaRL_v1_1_PY_01 2000 > $D/eval.log 2>&1 &) ;;
    bridgedrive) PORT=8848; CK=$WS/LEAD/outputs/local_evaluation/$R/checkpoint_endpoint.json; TR=none; YL=""; rm -f $CK
                 (bash $WS/run_bridgedrive_eval.sh $R > $D/eval.log 2>&1 &) ;;
    simlingo)    PORT=8848; CK=$WS/SimLingo/outputs/eval_b2d/$R/checkpoint_endpoint.json; TR=$D/trace.jsonl; YL="--y-left"; rm -f $CK
                 (AGENT_TRACE=$D/trace.jsonl bash $WS/run_simlingo_eval.sh $R > $D/eval.log 2>&1 &) ;;
    drivepi0|drivemoe) PORT=8858; CK=$WS/DriveMoE/outputs/eval_b2d/$M/$R/checkpoint_endpoint.json; TR=$D/trace.jsonl; YL=""; rm -f $CK
                 (AGENT_TRACE=$D/trace.jsonl systemd-run --user --scope -q -p MemoryMax=22G -p MemorySwapMax=0 bash $WS/run_pi0_eval.sh $R $M $PORT > $D/eval.log 2>&1 &) ;;
    *) echo "unknown model $M"; exit 1 ;;
  esac
  echo "[$(date +%H:%M)] $N/44 $T route $R ($TOWN) started"
  $REC $WS/record_hero.py --port $PORT --out $D/raw.mp4 --meta $D/meta.jsonl --label "$M $T $R" --wait 900 > $D/rec.log 2>&1 &
  RPID=$!; sleep 60
  for i in $(seq 1 600); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done     # episode over (or never started)
  for i in $(seq 1 10); do kill -0 $RPID 2>/dev/null || break; sleep 2; done; kill -INT $RPID 2>/dev/null; wait $RPID 2>/dev/null   # recorder flushes the video
  bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; sleep 2
  if [ -f "$CK" ]; then SC=$(score_line $CK 2>&1 | tail -1); cp $CK $D/checkpoint_endpoint.json; else SC="NO RESULT (see $D/eval.log)"; fi
  DS=$(echo "$SC" | grep -oE "DS=[0-9]+" | cut -d= -f2); NAME=${N}_${T}_${R}_DS${DS:-NA}
  if [ -s $D/meta.jsonl ]; then $REC $WS/annotate_run.py --video $D/raw.mp4 --meta $D/meta.jsonl --trace $TR --out $OUT/$NAME.mp4 --label "$M $T $R" --score "$SC" $YL > $D/annotate.log 2>&1 && A=$(tail -1 $D/annotate.log) || A="annotate FAILED"
  else A="no video"; fi
  echo "$N $T $R: $SC | $A" | tee -a $RES
done
bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; echo "[tour44 $M] done $(date '+%F %H:%M')" | tee -a $RES
