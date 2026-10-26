#!/bin/bash
# Run a Bench2Drive closed-loop evaluation with the TCP baseline agent.
#   bash run_tcp_eval.sh [ROUTES_XML] [ROUTES_SUBSET] [SAVE_PATH]
# e.g. bash run_tcp_eval.sh leaderboard/data/drivetransformer_bench2drive_dev10.xml 0 ./eval_dev10
# The evaluator launches CarlaUE4.sh itself (wrapped to pin the NVIDIA Vulkan ICD).
set -e
ROUTES=${1:-leaderboard/data/drivetransformer_bench2drive_dev10.xml}
SUBSET=${2:-0}
SAVE=${3:-./eval_dev10}
cd /home/all/carla_ws/Bench2Drive
PY=/home/all/miniconda3/envs/b2d/bin/python
export CARLA_ROOT=/home/all/carla_ws/carla
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:leaderboard:leaderboard/team_code:scenario_runner:Bench2DriveZoo
export SCENARIO_RUNNER_ROOT=scenario_runner LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS REPETITIONS=1 RESUME=True DEBUG_CHALLENGE=0
export IS_BENCH2DRIVE=True SAVE_PATH=$SAVE PLANNER_TYPE=only_traj
export TEAM_AGENT=leaderboard/team_code/tcp_b2d_agent.py TEAM_CONFIG=/home/all/carla_ws/data/ckpts/tcp_b2d.ckpt
mkdir -p "$SAVE"
CUDA_VISIBLE_DEVICES=0 $PY leaderboard/leaderboard/leaderboard_evaluator.py \
  --routes "$ROUTES" --routes-subset "$SUBSET" --repetitions 1 --track SENSORS \
  --checkpoint "$SAVE/result.json" --agent "$TEAM_AGENT" --agent-config "$TEAM_CONFIG" \
  --debug 0 --record "" --resume True --port 30000 --traffic-manager-port 50000 --gpu-rank 0
