#!/bin/bash
# Drive-pi0 / DriveMoE drives around an open world, lock-step (server must be running: QUALITY=Low bash carla_server.sh  <- Low: 5x faster steps).
#   bash pi0_drive.sh [--model drivemoe] [--map Town04] [--speed 0] [--green-lights] [--no-viewer]
WS=/home/all/carla_ws; DM=$WS/DriveMoE; MODEL=drivepi0; ARGS=()
while [ $# -gt 0 ]; do case $1 in --model) MODEL=$2; shift 2 ;; *) ARGS+=("$1"); shift ;; esac; done
case $MODEL in
  drivepi0) AGENT=$DM/src/agent/team_code/drivepi0_carla_agent.py; CFG=$DM/config/eval/DrivePi0/closed_loop_local.yaml; LABEL="Drive-pi0" ;;
  drivemoe) AGENT=$DM/src/agent/team_code/drivemoe_carla_agent.py; CFG=$DM/config/eval/DriveMoE/closed_loop_local.yaml; LABEL="DriveMoE" ;;
  *) echo "--model drivepi0|drivemoe"; exit 1 ;;
esac
export CARLA_ROOT=$WS/carla DRIVEMOE_REPO_DIR=$DM REPO_DIR=$DM IS_BENCH2DRIVE=True PLANNER_TYPE=traj TF_CPP_MIN_LOG_LEVEL=2
export SAVE_PATH=$DM/outputs/freeroam/$(date +%m%d_%H%M%S)   # SAVE_PATH set = AgentWrapper allows the 50 m BEV cam the agent requires; agent writes per-frame meta there
cd $DM && exec /home/all/miniconda3/envs/drivemoe/bin/python -u $WS/agent_freeroam.py --agent $AGENT --config "$CFG+freeroam" \
   --leaderboard $WS/Bench2Drive/leaderboard --scenario-runner $WS/Bench2Drive/scenario_runner --pythonpath $DM:$DM/src/agent --label "$LABEL" "${ARGS[@]}"
