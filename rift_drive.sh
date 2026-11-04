#!/bin/bash
# YOU drive, the fine-tuned policy drives the NPCs around you and tries to create trouble.
#   bash rift_drive.sh [CBV_POLICY] [EXTRA ARGS...]
# CBV_POLICY = which trained model controls the NPCs near you (all share the Pluto backbone):
#   rift_pluto   group-relative RL fine-tuning  -> most adversarial of the released set (default)
#   grpo_pluto / ppo_pluto / reinforce_pluto / rs_pluto / rtr_pluto / sft_pluto   other fine-tuners, for comparison
#   pluto        imitation learning only        -> realistic traffic, much less aggressive
#   standard     plain CARLA Traffic Manager    -> non-reactive baseline
# Window keys: W/S throttle-brake, A/D steer, SHIFT gentle, Q reverse, SPACE handbrake, TAB camera, ESC end route.
# The route is scored like any model run; results land in RIFT/log/eval/manual-<cbv>-rule-seed0/ and
# `python rift_report.py manual` prints your driving score next to what the NPCs did.
# Each launch starts from route 0 (--no_resume); otherwise it resumes a previous run and can skip straight to the
# last routes. Pass --no_resume's opposite by editing this line if you want resume behaviour back.
# Needs a display: run it from the desktop session (or set DISPLAY=:1 when driving over NoMachine/VNC).
set -u
CBV=${1:-rift_pluto}; shift 1 2>/dev/null || true
WS=/home/all/carla_ws; R=$WS/RIFT; PY=/home/all/miniconda3/envs/rift/bin/python
export CARLA_ROOT=$WS/carla PYTHONPATH=$WS/carla/PythonAPI/carla:$R CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
export DISPLAY=${DISPLAY:-:1}
# A human needs time to look around and pull away, and may stop legitimately. The benchmark ends a route after 3 s
# below 0.1 m/s ("Agent got blocked"), which scored 0 and stopped the NPC policy before it engaged anyone.
export RIFT_BLOCKED_TIME=${RIFT_BLOCKED_TIME:-120}
# The benchmark's NPC-selection gates assume a model ego driving at a steady pace: a vehicle only becomes an
# adversary if its path length to an interaction point is within 20 m of yours. Drive slower or pause and nothing
# qualifies. The 'human' profile widens that to 60 m, searches further, and allows 4 adversaries at once.
export RIFT_CBV_PROFILE=${RIFT_CBV_PROFILE:-human}
mkdir -p $WS/logs; LOG=$WS/logs/rift_drive_${CBV}.log
pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null; sleep 2
echo ">> you drive; NPC policy = $CBV ; window opens on DISPLAY $DISPLAY"
echo ">> standstill allowed: ${RIFT_BLOCKED_TIME}s ; NPC selection profile: ${RIFT_CBV_PROFILE} (RIFT_CBV_PROFILE=eval for benchmark gates)"
echo ">> DRIVE THE ROUTE: the policy only takes over NPCs near you as you move along it"
cd $R && $PY scripts/run.py --ego_cfg manual.yaml --cbv_cfg ${CBV}.yaml --mode eval -rep 1 --render --no_resume "$@" 2>&1 \
  | tee $LOG | grep -vE "FutureWarning|applymap|DeprecationWarning|pkg_resources|WARNING: (Traffic|Sign)|^\s*$"
echo; echo "log: $LOG"
$PY $WS/rift_report.py manual-${CBV} 2>/dev/null
