#!/bin/bash
# Run one Bench2Drive episode with BridgeDrive, record a chase-cam video, and play it back at REAL speed afterwards.
#   bash bd_watch.sh 25857
# (the benchmark itself runs in slow-motion lock-step so every frame is processed; the recording plays at 1x)
R=${1:?route id}; WS=/home/all/carla_ws; OUT=$WS/selected_runs/watch; mkdir -p $OUT; V=$OUT/bridgedrive_$R.mp4
rm -f $WS/logs/bridgedrive_$R.log
bash $WS/run_bridgedrive_eval.sh $R > $OUT/bridgedrive_$R.log 2>&1 &
EV=$!
for i in $(seq 1 90); do P=$(grep -o "carla-rpc-port=[0-9]*" $WS/logs/bridgedrive_$R.log 2>/dev/null | head -1 | cut -d= -f2); [ -n "$P" ] && break; sleep 1; done
echo ">>> evaluator CARLA port: ${P:-8848}; recording to $V"
/home/all/miniconda3/envs/lead/bin/python $WS/record_hero.py --port ${P:-8848} --out $V --label "BridgeDrive $R" --wait 900
wait $EV
bash $WS/show_bd_score.sh $R
echo ">>> chase video: $V   model view: $WS/LEAD/outputs/local_evaluation/$R/${R}_debug.mp4"
(vlc --play-and-exit "$V" >/dev/null 2>&1 || xdg-open "$V") &
