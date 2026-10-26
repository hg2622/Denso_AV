#!/bin/bash
# Drive one Bench2Drive route yourself and get scored.
#   bash run_human_eval.sh [ROUTE_ID|random|random220] [ROUTES_XML] [SAVE_DIR]
# e.g. bash run_human_eval.sh 25378 | random | random220 | PedestrianCrossing | HighwayCutIn  (any scenario type name)
# A pygame window opens (1280x720 front camera). Follow the green arrows to the red dot.
# W/Up throttle, S/Down brake, A/D steer, Q toggle reverse, Space handbrake.
# The evaluator launches CARLA itself (~30 s startup). Score: bash show_score.sh <SAVE_DIR>
set -e
SUBSET=${1:-25378}
ROUTES=${2:-leaderboard/data/drivetransformer_bench2drive_dev10.xml}
SAVE=${3:-./eval_human}
cd /home/all/carla_ws/Bench2Drive
# "random"    -> random route from the dev10 set;  "random220" -> random route from the full 220 benchmark
if [ "$SUBSET" = "random220" ]; then ROUTES=leaderboard/data/bench2drive220.xml; SUBSET=random; fi
# scenario name (e.g. PedestrianCrossing, HighwayCutIn) -> random route of that type from the 220 set
if [[ "$SUBSET" =~ ^[A-Za-z]+$ && "$SUBSET" != "random" ]]; then
  ROUTES=leaderboard/data/bench2drive220.xml
  PICK=$(/home/all/miniconda3/envs/b2d/bin/python /home/all/carla_ws/list_routes.py "$ROUTES" "$SUBSET" | shuf -n1)
  [ -z "$PICK" ] && { echo "no scenario named '$SUBSET'. Types:"; grep -oE 'type="[A-Za-z]+"' "$ROUTES" | sort -u | sed 's/type=//; s/"//g' | tr '\n' ' '; echo; exit 1; }
  echo ">>> $SUBSET pick: $PICK"; SUBSET=$(echo "$PICK" | awk '{print $1}')
fi
if [ "$SUBSET" = "random" ]; then
  SUBSET=$(grep -oE 'route id="[0-9]+" town="[^"]+"' "$ROUTES" | shuf -n1)
  echo ">>> random pick: $SUBSET"; SUBSET=$(echo "$SUBSET" | grep -oE 'id="[0-9]+"' | grep -oE '[0-9]+')
fi
PY=/home/all/miniconda3/envs/b2d/bin/python
export CARLA_ROOT=/home/all/carla_ws/carla
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:leaderboard:leaderboard/team_code:scenario_runner:Bench2DriveZoo
export SCENARIO_RUNNER_ROOT=scenario_runner LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS REPETITIONS=1 RESUME=True DEBUG_CHALLENGE=0
export IS_BENCH2DRIVE=True SAVE_PATH=$SAVE   # SAVE_PATH set => Bench2Drive scenario/scoring mode
pkill -9 -f "leaderboard_evaluato[r].py" 2>/dev/null || true; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng.*rpc-port=3000[0]" 2>/dev/null || true; sleep 1  # clear stale runs
mkdir -p "$SAVE"; rm -f "$SAVE/result.json"   # fresh score each run
CUDA_VISIBLE_DEVICES=0 $PY leaderboard/leaderboard/leaderboard_evaluator.py \
  --routes "$ROUTES" --routes-subset "$SUBSET" --repetitions 1 --track SENSORS \
  --checkpoint "$SAVE/result.json" --agent leaderboard/team_code/human_b2d_agent.py \
  --agent-config leaderboard/leaderboard/autoagents/human_agent_config.txt \
  --debug 0 --record "" --resume True --port 30000 --traffic-manager-port 50000 --gpu-rank 0
bash /home/all/carla_ws/show_score.sh "$SAVE"
