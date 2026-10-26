#!/bin/bash
# PDM-Lite (privileged rule-based expert) on Bench2Drive routes: run + record + annotate.   bash pdmlite_runs.sh "2286 24211"  (SET/EXTEND/TAG env as usual)
WS=/home/all/carla_ws; SL=/home/all/carla_ws/SimLingo; OUT=$WS/selected_runs/pdmlite; REC=/home/all/miniconda3/envs/lead/bin/python; mkdir -p $OUT; RES=$OUT/RESULTS.txt
TYPE() { grep -v '^#' $WS/tour44_routes.txt | awk -v r="$1" '$3==r {print $2}'; }
for R in ${1:-"24211 2709"}; do
  D=$SL/outputs/runs_pdmlite/$R; mkdir -p $D; rm -f $D/trace.jsonl $D/meta.jsonl $D/raw.mp4; T=$(TYPE $R); [ -z "$T" ] && T=$(grep -oE "route id=\"$R\".{0,400}type=\"[A-Za-z_]+\"" $WS/Bench2Drive/leaderboard/data/bench2drive220.xml | grep -oE 'type="[A-Za-z_]+"' | head -1 | cut -d'"' -f2)
  echo "[$(date +%H:%M)] PDM-Lite $R ($T) started"
  (AGENT_TRACE=$D/trace.jsonl bash $WS/run_pdmlite_eval.sh $R > $D/eval.log 2>&1 &)
  sleep 20; $REC $WS/record_hero.py --port 8848 --out $D/raw.mp4 --meta $D/meta.jsonl --label "PDM-Lite $R $T" --wait 900 > $D/rec.log 2>&1
  for i in $(seq 1 600); do pgrep -f "leaderboard_evaluato[r]" >/dev/null || break; sleep 3; done
  SC=$(grep -o "PDM-Lite RouteScenario.*" $D/eval.log | tail -1 | sed 's/PDM-Lite RouteScenario_[0-9_rep]*: //'); [ -z "$SC" ] && SC="NO RESULT"
  if [ -s $D/meta.jsonl ]; then $REC $WS/annotate_run.py --video $D/raw.mp4 --meta $D/meta.jsonl --trace $D/trace.jsonl --out $OUT/pdmlite_${R}_${T}${TAG:-}.mp4 --label "PDM-Lite $R $T (privileged expert${TAG:-})" --score "$SC" > $D/annotate.log 2>&1 && A=$(tail -1 $D/annotate.log) || A="annotate FAILED"; else A="no video"; fi
  echo "PDM-Lite $R $T${TAG:-}: $SC | $A" | tee -a $RES
done
echo "[pdmlite done] $(date +%H:%M)" | tee -a $RES
