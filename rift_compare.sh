#!/bin/bash
# Apples-to-apples: the SAME ego drives the SAME routes with the SAME seed, while the NPCs around it are driven by a
# different trained checkpoint each run. The only variable is the NPC policy, so the numbers are directly comparable.
#   bash rift_compare.sh [EGO] [CBV ...]
# EGO defaults to pdm_lite - the policy every released NPC checkpoint was fine-tuned against, so that is the fair
# pairing (the checkpoint folders are literally named "pdm_lite-rule-seed0"). Using another ego is valid but then the
# NPC policies are off-distribution and the comparison says less about them.
# CBV list defaults to: standard (plain Traffic Manager floor), pluto (imitation only), rift_pluto, grpo_pluto,
#   ppo_pluto, reinforce_pluto (the RL fine-tuners).
# Everything else is pinned: same route file, seed 0, one repetition, resume disabled, benchmark selection gates.
# Results: rift_report.py prints one row per policy. Videos under RIFT/log/eval/<ego>-<cbv>-rule-seed0/<Town>/videos/
set -u
EGO=${1:-pdm_lite}; shift 1 2>/dev/null || true
CBVS=("$@"); [ ${#CBVS[@]} -eq 0 ] && CBVS=(standard pluto rift_pluto grpo_pluto ppo_pluto reinforce_pluto)
WS=/home/all/carla_ws; R=$WS/RIFT; PY=/home/all/miniconda3/envs/rift/bin/python
export CARLA_ROOT=$WS/carla PYTHONPATH=$WS/carla/PythonAPI/carla:$R CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
# benchmark gates and the standard blocked-timeout: a model ego drives at a steady pace, so the defaults are correct
# here. (The 'human' profile exists for manual driving only; mixing it in would make runs non-comparable.)
export RIFT_CBV_PROFILE=eval
unset RIFT_BLOCKED_TIME RIFT_AUTODRIVE RIFT_CBV_REUSE
mkdir -p $WS/logs
echo ">> ego: $EGO   |   NPC policies: ${CBVS[*]}   |   seed 0, same routes, no resume"
for CBV in "${CBVS[@]}"; do
    echo; echo "=================== $EGO  vs  $CBV ==================="
    rm -rf $R/log/eval/${EGO}-${CBV}-rule-seed0           # fresh run so every policy sees the full route set
    pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null; sleep 3
    ( cd $R && timeout 3600 $PY scripts/run.py --ego_cfg ${EGO}.yaml --cbv_cfg ${CBV}.yaml \
        --mode eval -rep 1 --seed 0 --render --no_resume ) > $WS/logs/cmp_${EGO}_${CBV}.log 2>&1
    echo "   exit $?  (log: $WS/logs/cmp_${EGO}_${CBV}.log)"
done
pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null
echo; echo "=================== comparison ==================="
$PY $WS/rift_report.py $(printf "%s-" "$EGO" | sed 's/-$//')
