#!/bin/bash
# Re-run every overnight run whose RESULTS line ended without a video ("no trace/meta" / "NO RESULT"), per model.
WS=/home/all/carla_ws; R=$WS/selected_runs/night/RESULTS.txt
for M in simlingo drivepi0 drivemoe; do
  ROUTES=$(grep -E "^$M [0-9]+ " $R | grep -E "no trace/meta|NO RESULT|annotated [0-9] frames|annotated [1-4][0-9] frames" | awk "{print \$2}" | sort -u | tr "\n" " ")
  [ -n "$ROUTES" ] && { echo "[rerun] $M: $ROUTES"; bash $WS/night_runs.sh "$M" "$ROUTES"; }   # Low-quality CARLA segfaults on Town05 with the pi0/MoE sensor set
done
echo "[rerun done] $(date +%H:%M)" >> $R
