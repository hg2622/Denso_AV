#!/bin/bash
# The staged "oncoming lane invader" scene on the pedestrian route 24211 (pedestrian stays, an oncoming car drifts into our lane
# and eases back) for CaRL, SimLingo (original 1 Hz) and BridgeDrive, recorded + annotated.   bash invader_runs.sh [models] [SET-extra]
#   -> selected_runs/custom/<model>_24211_LaneInvader.mp4 (+ .png), RESULTS_invader.txt
WS=/home/all/carla_ws; OUT=$WS/selected_runs/custom; WORK=$OUT/_invader_work; mkdir -p $WORK; RES=$OUT/RESULTS_invader.txt; REC=/home/all/miniconda3/envs/lead/bin/python
MODELS=${1:-"carl simlingo bridgedrive"}; TAG=${TAG:-LaneInvader}
if pgrep -f "scene_driv[e].py" >/dev/null || pgrep -f "scene_captur[e].py" >/dev/null; then
  echo "a scene recorder is still running - close its window first (it holds CARLA on port 2000)"; exit 1
fi
# PED=1: the pedestrian ALSO steps out (original crossing) while the car invades -> TAG defaults to LaneInvader_PED, faster + deeper invader
if [ "${PED:-0}" = 1 ]; then TAG=${TAG_OVERRIDE:-LaneInvader_PED}; BASE="DynamicObjectCrossing.invader=1,DynamicObjectCrossing.invader_dist=50,DynamicObjectCrossing.invader_trigger=50,DynamicObjectCrossing.invader_speed=9,DynamicObjectCrossing.invader_offset=2.0,DynamicObjectCrossing.invader_start=0.3,DynamicObjectCrossing.invader_in=1.0,DynamicObjectCrossing.invader_hold=2.0,DynamicObjectCrossing.invader_out=1.2"
else BASE="DynamicObjectCrossing.no_cross=1,DynamicObjectCrossing.invader=1"; fi
if [ -n "${TRACK:-}" ]; then [ "${TRACK#/}" = "$TRACK" ] && TRACK=$WS/$TRACK; BASE="DynamicObjectCrossing.npc_track=$TRACK${NOCROSS:+,DynamicObjectCrossing.no_cross=1}"; TAG=${TAG_OVERRIDE:-Track_$(basename ${TRACK%.json})}; fi   # TRACK=npc_tracks/x.json: replay a hand-driven NPC
SETX="$BASE${2:+,$2}"
score_line() { $REC - "$1" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
print(f"DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
PY
}
for M in $MODELS; do
  D=$WORK/${M}_$TAG; rm -rf $D; mkdir -p $D; bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; sleep 3
  case $M in
    carl)        PORT=2000; CK=$WS/CaRL/CARLA/results/eval_b2d/CaRL_v1_1_PY_01/24211.json; TR=none; YL=""; rm -f $CK
                 (SET="$SETX" bash $WS/run_carl_eval.sh 24211 CaRL_v1_1_PY_01 2000 > $D/eval.log 2>&1 &) ;;
    simlingo)    PORT=8848; CK=$WS/SimLingo/outputs/eval_b2d/24211/checkpoint_endpoint.json; TR=$D/trace.jsonl; YL="--y-left"; rm -f $CK
                 (AGENT_TRACE=$D/trace.jsonl SL_LANG_EVERY=1 SET="$SETX" bash $WS/run_simlingo_eval.sh 24211 > $D/eval.log 2>&1 &) ;;
    bridgedrive) PORT=8848; CK=$WS/LEAD/outputs/local_evaluation/24211/checkpoint_endpoint.json; TR=none; YL=""; rm -f $CK
                 (SET="$SETX" bash $WS/run_bridgedrive_eval.sh 24211 > $D/eval.log 2>&1 &) ;;
  esac
  echo "[$(date +%H:%M)] $M $TAG started (SET=$SETX)"
  $REC $WS/record_hero.py --port $PORT --out $D/raw.mp4 --meta $D/meta.jsonl --label "$M $TAG" --wait 900 > $D/rec.log 2>&1
  for i in $(seq 1 400); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done
  bash $WS/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; sleep 2
  if [ -f "$CK" ]; then SC=$(score_line $CK 2>&1 | tail -1); cp $CK $D/checkpoint_endpoint.json; else SC="NO RESULT (see eval.log)"; fi
  grep -h "LaneInvader" $D/eval.log $WS/logs/*24211*.log 2>/dev/null | head -3 > $D/invader.log
  if [ -s $D/meta.jsonl ]; then $REC $WS/annotate_run.py --video $D/raw.mp4 --meta $D/meta.jsonl --trace $TR --out $OUT/${M}_24211_$TAG.mp4 --label "$M 24211 $TAG" --score "$SC" $YL > $D/annotate.log 2>&1 && A=$(tail -1 $D/annotate.log) || A="annotate FAILED"
    [ "$TR" != none ] && $REC $WS/collision_text.py --trace $TR --meta $D/meta.jsonl --at-end > $OUT/${M}_24211_$TAG.collision_text.txt 2>&1
  else A="no video"; fi
  echo "$M 24211 $TAG: $SC | $A" | tee -a $RES
done
echo "[invader done] $(date +%H:%M)" | tee -a $RES
