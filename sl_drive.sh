#!/bin/bash
# SimLingo (VLA, InternVL2-1B, camera only) drives around an open world, lock-step (server must be running: bash carla_server.sh).
#   bash sl_drive.sh [--map Town04] [--dist 600] [--speed 0]     (--speed 1 = real-time pacing if the model is fast enough)
WS=/home/all/carla_ws; SL=$WS/SimLingo
export CARLA_ROOT=$WS/carla WORK_DIR=$SL SAVE_PATH=$SL/outputs/freeroam/ ROUTES=freeroam   # agent_simlingo.setup needs SAVE_PATH + ROUTES (debug viz dir name)
cd $SL && exec /home/all/miniconda3/envs/simlingo/bin/python -u $WS/agent_freeroam.py --agent $SL/team_code/agent_simlingo.py \
   --config "$SL/ckpt/simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt+freeroam" --leaderboard $SL/Bench2Drive/leaderboard \
   --scenario-runner $SL/Bench2Drive/scenario_runner --pythonpath $SL/team_code:$SL --label SimLingo "$@"
