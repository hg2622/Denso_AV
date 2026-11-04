#!/bin/bash
# YOU drive, CTG++ (scene-level diffusion traffic model, NVlabs/CTG) drives the NPCs around you.
#   bash ctg_drive.sh [GUIDANCE] [EXTRA ARGS...]
# GUIDANCE = what the diffusion sampler is steered towards (guidance losses, applied at every denoising step):
#   trouble   the nearest NPC ahead is pulled onto your future path, the rest avoid each other (default)
#   cutin     the nearest NPC ahead cuts in front of you and brakes
#   tailgate  the nearest NPC stays 3-8 m from you
#   safe      everyone avoids collisions and stays on the road (realistic traffic)
#   none      unguided samples (pure imitation of nuScenes driving)
#   FILE.json custom list of {name, weight, params, agents}  (names: tbsim GUIDANCE_FUNC_MAP + ctg_model extras)
# Extra args go to ctg_carla/ctg_carla.py: --town Town05 --npcs 24 --control 8 --samples 5 --stride 1 --record out.mp4 ...
# Window keys: W/S throttle-brake, A/D steer, SHIFT gentle, Q reverse, SPACE handbrake, TAB camera, ESC quit.
# Red dots = the NPC being guided towards you, blue dots = the other CTG++ controlled NPCs (their next 2 s).
# Latency: 100 denoising steps x 5 samples ~1.7 s per plan on this GPU; --stride 4 --samples 2 gives ~0.2 s
# (replans every 0.5 s of simulated time, so the window stutters at stride 1; the simulation itself is lock-step).
set -u
G=${1:-trouble}; shift 1 2>/dev/null || true
WS=/home/all/carla_ws; PY=/home/all/miniconda3/envs/ctg/bin/python
export DISPLAY=${DISPLAY:-:1} CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
mkdir -p $WS/logs; LOG=$WS/logs/ctg_drive_${G##*/}.log
if ! pgrep -f "CarlaUE4-Linux-Shipp[i]ng.*rpc-port=2000" >/dev/null; then
    echo ">> starting CARLA server"; (bash $WS/carla_server.sh > $WS/logs/carla_server.log 2>&1 &); sleep 12
fi
echo ">> you drive; NPC generator = CTG++ with guidance '$G' ; window on DISPLAY $DISPLAY ; log $LOG"
cd $WS && $PY ctg_carla/ctg_carla.py --ego manual --guidance "$G" "$@" 2>&1 | grep --line-buffered -vE "Warning|warn|pkg_resources|Zen of|^(If |Although|Namespaces|Unless|In the|There should|Now is|Beautiful|Explicit|Simple|Complex|Flat|Sparse|Readability|Special|Errors|Instantiating|\[|DIFFUSER|  [0-9]|disable_control|registered_name|torchvision|add_coeffs|div_coeffs)|^\s*$" | tee $LOG
