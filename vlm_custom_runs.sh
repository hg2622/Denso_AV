#!/bin/bash
# The 6 custom scenarios (same SCENARIO_OVERRIDES as the BridgeDrive / CaRL videos) on the VLM baselines, recorded + annotated.
#   bash vlm_custom_runs.sh [models="simlingo drivepi0 drivemoe"]   -> selected_runs/custom/<model>_<name>.mp4 (+ .png speed profile) and custom/RESULTS_vlm.txt
WS=/home/all/carla_ws; OUT=$WS/selected_runs/custom; WORK=$OUT/_vlm_work; mkdir -p $OUT $WORK; RES=$OUT/RESULTS_vlm.txt; REC=/home/all/miniconda3/envs/lead/bin/python
MODELS=${1:-"simlingo drivepi0 drivemoe"}
score_line() { $REC - "$1" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
print(f"DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
PY
}
run() {  # name route SET EXTEND model
  N=$1; R=$2; S=$3; E=$4; M=$5; D=$WORK/${M}_$N; mkdir -p $D; rm -f $D/trace.jsonl $D/meta.jsonl
  case $M in
    simlingo) PORT=8848; CK=$WS/SimLingo/outputs/eval_b2d/$R/checkpoint_endpoint.json; YL="--y-left"
              (AGENT_TRACE=$D/trace.jsonl SL_LANG_EVERY=1 SET="$S" EXTEND="$E" bash $WS/run_simlingo_eval.sh $R > $D/eval.log 2>&1 &) ;;
    drivepi0|drivemoe) PORT=8858; CK=$WS/DriveMoE/outputs/eval_b2d/$M/$R/checkpoint_endpoint.json; YL=""; Q=Low
              (AGENT_TRACE=$D/trace.jsonl SET="$S" EXTEND="$E" CARLA_QUALITY=$Q systemd-run --user --scope -q -p MemoryMax=22G -p MemorySwapMax=0 bash $WS/run_pi0_eval.sh $R $M $PORT > $D/eval.log 2>&1 &) ;;
  esac
  echo "[$(date +%H:%M)] $M $N started (SET=$S EXTEND=$E)"
  $REC $WS/record_hero.py --port $PORT --out $D/raw.mp4 --meta $D/meta.jsonl --label "$M $N" --wait 900 > $D/rec.log 2>&1
  for i in $(seq 1 400); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done   # up to 20 min for the evaluator to finish after the recorder stops
  pkill -9 -f "rpc-port=$PORT " 2>/dev/null; pkill -9 -f "leaderboard_evaluato[r]" 2>/dev/null; sleep 2
  if [ -f "$CK" ]; then SC=$(score_line $CK); cp $CK $D/checkpoint_endpoint.json; else SC="NO RESULT (see eval.log)"; fi
  if [ -s $D/trace.jsonl ] && [ -s $D/meta.jsonl ]; then
    $REC $WS/annotate_run.py --video $D/raw.mp4 --meta $D/meta.jsonl --trace $D/trace.jsonl --out $OUT/${M}_$N.mp4 --label "$M $N" --score "$SC" $YL > $D/annotate.log 2>&1 && A=$(tail -1 $D/annotate.log) || A="annotate FAILED"
    $REC $WS/collision_text.py --trace $D/trace.jsonl --meta $D/meta.jsonl --at-end > $OUT/${M}_$N.collision_text.txt 2>&1
  else A="no trace/meta"; fi
  echo "$M $N: $SC | $A" | tee -a $RES
}
for M in $MODELS; do
  if [ "$M" = simlingo ]; then   # SimLingo (1 Hz): only the two pairs the user asked for: cut-in vs no cut-in, pedestrian crosses (night run) vs not
    grep -q "^simlingo 2286_HighwayCutIn_SYNC_CUT_IN: DS" $RES 2>/dev/null || run 2286_HighwayCutIn_SYNC_CUT_IN 2286 "" 250 $M   # original scenario (sync + cut-in) on the extended route; skipped if already scored
    run 24211_DynamicObjectCrossing_NO_CROSS 24211 "DynamicObjectCrossing.no_cross=1" 0 $M
    continue
  fi
  run 2286_HighwayCutIn_SYNC_CUT_IN 2286 "" 250 $M
  run 2286_HighwayCutIn_SYNC_then_CONST 2286 "HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=1" 250 $M
  run 2286_HighwayCutIn_CONST_MEET 2286 "HighwayCutIn.const_meet=1,HighwayCutIn.cut_in_distance=1" 250 $M
  run 2286_HighwayCutIn_NO_CUT_IN_fast 2286 "HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=1,HighwayCutIn.exit_speed=30" 250 $M
  run 24759_ParkingCutIn_NO_CUT_IN 24759 "ParkingCutIn.no_cut_in=1" 0 $M
  run 24211_DynamicObjectCrossing_NO_CROSS 24211 "DynamicObjectCrossing.no_cross=1" 0 $M
  run 26396_StaticCutIn_LATE_CUT 26396 "StaticCutIn.reaction_time=1.2,StaticCutIn.min_trigger_dist=6,StaticCutIn.change_time=1.5" 0 $M
done
bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; echo "[all done] $(date +%H:%M)" | tee -a $RES
