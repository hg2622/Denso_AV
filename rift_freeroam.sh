#!/bin/bash
# FREE ROAM with RIFT: a continuous world (no Bench2Drive episode, no scoring). You drive wherever you like; the learned
# NPC policy keeps taking over the cars near you and drives them towards your path.
#   bash rift_freeroam.sh [CBV_POLICY] [EXTRA ARGS...]
# CBV_POLICY: rift_pluto (default, most adversarial) | grpo_pluto | ppo_pluto | reinforce_pluto | rs_pluto | rtr_pluto |
#             sft_pluto | pluto (imitation only) | standard (plain Traffic Manager, nothing learned)
# Extra args go to RIFT/scripts/freeroam.py: --town Town05 --npcs 40 --ego autopilot --profile eval --record out.mp4
#             --show-routes (draw the NPCs' planned routes) --test SECONDS (headless self-test, ego on autopilot)
# Keys: W/S throttle-brake, A/D steer, SHIFT gentle, Q reverse, SPACE handbrake, TAB camera, ESC quit.
# Red marker above a car = that NPC is driven by the policy right now (up to 4 at once with the 'human' profile).
# The route the NPCs aim at is generated from wherever you are and regenerated when you turn off it.
# Starts the CARLA server on :2000 if none is running.
set -u
CBV=${1:-rift_pluto}; shift 1 2>/dev/null || true
WS=/home/all/carla_ws; R=$WS/RIFT; PY=/home/all/miniconda3/envs/rift/bin/python
export CARLA_ROOT=$WS/carla PYTHONPATH=$WS/carla/PythonAPI/carla:$R CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
export DISPLAY=${DISPLAY:-:1}
mkdir -p $WS/logs; LOG=$WS/logs/rift_freeroam_${CBV}.log
if ! pgrep -f "CarlaUE4-Linux-Shipp[i]ng.*rpc-port=2000" >/dev/null; then
    echo ">> starting CARLA server"; (bash $WS/carla_server.sh > $WS/logs/carla_server.log 2>&1 &); sleep 12
fi
echo ">> free roam: you drive, NPC policy = $CBV ; window on DISPLAY $DISPLAY ; log $LOG"
cd $R && $PY scripts/freeroam.py --cbv "$CBV" "$@" 2>&1 \
  | grep --line-buffered -vE "FutureWarning|applymap|DeprecationWarning|pkg_resources|WARNING: (Traffic|Sign)|UserWarning|warnings.warn|Gym|gymnasium|migration|^\s*$" | tee $LOG
