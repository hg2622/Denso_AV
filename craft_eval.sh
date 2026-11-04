#!/bin/bash
# CRAFT (github.com/CurryChen77/CraftPolicy, NeurIPS 2026): closed-loop post-training of the EGO policy against
# reactive NPCs. Same codebase family as RIFT, so the NPC side is identical; the fine-tuning target is the ego instead.
#   bash craft_eval.sh [CBV_POLICY] [EGO_POLICY] [EXTRA ARGS...]
# CBV_POLICY (the NPCs): scenario (CRAFT's interaction-critical traffic, default), autopilot (Traffic Manager),
#   pluto, rift_pluto, grpo_pluto, ppo_pluto, reinforce_pluto, forsim_pluto, frea, fppo_rs, rs_pluto, rtr_pluto, sft_pluto
# EGO_POLICY: pdm_lite (default), expert, behavior, plant_v2, ppo.
#   The end-to-end egos CRAFT fine-tunes (sparsedrive_scorer, hip_ad, mind_drive, uniad, vad) need the b2d toolchain
#   (mmcv + compiled CUDA ops), which is not installed here; the official fine-tuned checkpoints are also unreleased
#   ("Post-Acceptance Release" on their TODO list), so this evaluates the base ego against CRAFT's NPCs.
# Output: CRAFT/log/eval/<ego>-<cbv>-seed0/... with videos and simulation_results.json; summarise with rift_report.py
set -u
CBV=${1:-scenario}; EGO=${2:-pdm_lite}; shift 2 2>/dev/null || shift $#
WS=/home/all/carla_ws; C=$WS/CRAFT; PY=/home/all/miniconda3/envs/craft/bin/python
export CARLA_ROOT=$WS/carla PYTHONPATH=$WS/carla/PythonAPI/carla:$C CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
mkdir -p $WS/logs; LOG=$WS/logs/craft_${EGO}_${CBV}.log
pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null; sleep 2
cd $C && $PY scripts/run.py --ego_cfg ${EGO}.yaml --cbv_cfg ${CBV}.yaml --mode eval --seed 0 \
    --record_video --save_eval_result "$@" 2>&1 | tee $LOG | grep -vE "FutureWarning|applymap|DeprecationWarning|pkg_resources|WARNING: (Traffic|Sign)|^\s*$"
echo; echo "log: $LOG"
$PY $WS/rift_report.py ${EGO}-${CBV} 2>/dev/null
