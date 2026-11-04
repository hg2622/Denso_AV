#!/bin/bash
# RIFT: fine-tuned NPCs (controlled background vehicles, "CBVs") around a chosen ego, on Bench2Drive routes.
#   bash rift_eval.sh [CBV_POLICY] [EGO_POLICY] [EXTRA ARGS...]
# CBV_POLICY (how the NPCs behave; all share the same Pluto backbone, different fine-tuning):
#   rift_pluto      group-relative RL fine-tuning        (the paper's method)
#   pluto           imitation-learning baseline, no RL   (realistic, least adversarial)
#   grpo_pluto      GRPO                                 ppo_pluto / reinforce_pluto / rs_pluto / rtr_pluto / sft_pluto
#   standard        CARLA Traffic Manager NPCs           (the usual non-reactive baseline)
#   frea / fppo_rs / ppo   earlier adversarial baselines
# EGO_POLICY: pdm_lite (privileged rule-based expert, default), expert, behavior, plant, ppo.
#             vad / uniad / sparsedrive need the b2d toolchain (mmcv + compiled ops), not installed here.
# Output: RIFT/log/eval/<ego>-<cbv>-rule-seed0/<Town>/videos/route_N.mp4 and simulation_results.json
#         (DS/RC per route plus cbv_* metrics: how many NPCs were taken over, collisions they caused, speed profile).
# Notes: --render records the video; the run resumes where it stopped, so delete the log folder to start fresh.
set -u
CBV=${1:-rift_pluto}; EGO=${2:-pdm_lite}; shift 2 2>/dev/null || shift $#
WS=/home/all/carla_ws; R=$WS/RIFT; PY=/home/all/miniconda3/envs/rift/bin/python
export CARLA_ROOT=$WS/carla PYTHONPATH=$WS/carla/PythonAPI/carla:$R CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
mkdir -p $WS/logs; LOG=$WS/logs/rift_${EGO}_${CBV}.log
pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null; sleep 2
cd $R && $PY scripts/run.py --ego_cfg ${EGO}.yaml --cbv_cfg ${CBV}.yaml --mode eval -rep 1 --render "$@" 2>&1 | tee $LOG | grep -vE "FutureWarning|applymap|DeprecationWarning|pkg_resources|WARNING: (Traffic|Sign)|^\s*$"
echo; echo "log: $LOG"; echo "results: $R/log/eval/${EGO}-${CBV}-rule-seed0/simulation_results.json"
$PY $WS/rift_report.py ${EGO}-${CBV} 2>/dev/null
