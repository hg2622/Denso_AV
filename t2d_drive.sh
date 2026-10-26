#!/bin/bash
# Think2Drive (DreamerV3 world-model RL, CaRL reproduction) free run: drives random long routes (no scenarios) with its own
# pipeline (it starts its own CARLA server + leaderboard; stop any other CARLA first: bash stop_all.sh).
#   bash t2d_drive.sh [Town04] [N_ROUTES=3]
# Watch it: it prints "Starting CARLA on port: N"; then in another terminal  bash ~/carla_ws/views.sh chase bev --port N
# Outputs (BEV frames, videos, metrics): CaRL/Think2Drive/carla_leaderboard_checkpoints/freeroam_*/
TOWN=${1:-Town04}; N=${2:-3}; shift $(( $# > 2 ? 2 : $# ))   # remaining args go to dreamer
export PATH=/home/all/miniconda3/bin:$PATH
T2D=/home/all/carla_ws/CaRL/Think2Drive
CKPT=${T2D_CKPT:-$(find "$T2D/logdir" -path "*posttrain*" -name "checkpoint_*.ckpt" | sort -t_ -k2 -n | tail -1)}   # highest-step post-training checkpoint; override with T2D_CKPT=...
[ -z "$CKPT" ] && { echo "no checkpoint found under $T2D/logdir (unzip Think2Drive_checkpoints.zip there)"; exit 1; }
/home/all/miniconda3/envs/carl/bin/python /home/all/carla_ws/t2d_make_routes.py --map $TOWN --n $N --dist 700 --out $T2D/freeroam_routes || exit 1
TS=$(date +%Y%m%d_%H%M%S)
cd $T2D/dreamerv3
echo ">>> checkpoint: $CKPT"
T2D_RENDER=1 /home/all/miniconda3/envs/think2drive/bin/python dreamer/main.py \
    --configs carla --run.script eval_only_carla_parallel \
    --env.carla.carla_installation_path /home/all/carla_ws/carla \
    --env.carla.pretraining False \
    --run.num_envs 1 --run.actor_batch 1 --run.actor_threads 1 \
    --env.carla.eval_times 1 --env.carla.eval_routes_directory ../freeroam_routes \
    --batch_length 3 --batch_length_eval 3 --seed 0 \
    --logdir ../logdir/freeroam_eval_$TS \
    --run.from_checkpoint "$CKPT" \
    --env.carla.results_directory freeroam_$TS "$@"
