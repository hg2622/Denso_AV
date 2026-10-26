#!/bin/bash
# Run BridgeDrive (ICLR'26 diffusion-bridge planner, LEAD/TFv6 backbone) on one Bench2Drive route, closed loop.
#   bash run_bridgedrive_eval.sh ROUTE_ID          # e.g. 25378 (ids: python list_routes.py)
#   bash run_bridgedrive_eval.sh random            # random one of the 220
#   bash run_bridgedrive_eval.sh all               # the whole 220-route benchmark (many hours)
# Score: printed at the end; also bash show_bd_score.sh <route>
# Debug video of what the model saw/planned: LEAD/outputs/local_evaluation/<route>/<route>_debug.mp4
set -u
# custom scenario knobs: SET="HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=5" bash run_..._eval.sh ROUTE  (custom scenario -> score not comparable)
export SCENARIO_OVERRIDES="${SET:-}"
# npc_track replay: freeze the episode's background-traffic spawner from frame 0 so deleted cars never appear and the scene is exactly the recording
case "${SCENARIO_OVERRIDES:-}" in *npc_track=*) export NPC_RECORD_BG_FREEZE=/tmp/npc_replay_bgfreeze_$$; : > $NPC_RECORD_BG_FREEZE; trap 'rm -f $NPC_RECORD_BG_FREEZE' EXIT; export SCENE_END="${SCENE_END:-3}"; echo "background spawner frozen (recorded scene); route ends ${SCENE_END}s after it";; esac
# EXTEND=250 bash run_bridgedrive_eval.sh ROUTE  -> route continues 250 m past its official end (custom; score not comparable)
ROUTE_ARG=${1:-random}
WS=/home/all/carla_ws; LEAD=$WS/LEAD; PY=/home/all/miniconda3/envs/lead/bin/python
B2D220=$WS/Bench2Drive/leaderboard/data/bench2drive220.xml
PORT=8848; TMPORT=8100
cd $LEAD; export PATH=/home/all/miniconda3/envs/lead/bin:$PATH
mkdir -p data/benchmark_routes/bench2drive220routes
if [[ "$ROUTE_ARG" == "all" ]]; then
    ROUTES=$B2D220; RID=bench2drive220
else
    if [[ "$ROUTE_ARG" == "random" ]]; then ROUTE_ARG=$(grep -o '<route id="[0-9]*"' $B2D220 | sed 's/[^0-9]//g' | shuf -n1); fi
    RID=${ROUTE_ARG//,/_}; ROUTES=data/benchmark_routes/bench2drive220routes/$RID.xml
    $PY - "$B2D220" "$ROUTE_ARG" "$ROUTES" <<'PYEOF'
import sys, xml.etree.ElementTree as ET
src, rids, dst = sys.argv[1:]; rids = rids.split(',')
root = ET.parse(src).getroot(); byid = {r.get('id'): r for r in root.findall('route')}
new = ET.Element('routes')
for rid in rids:
    assert rid in byid, f'route {rid} not in bench2drive220.xml'; new.append(byid[rid]); print('route', rid, 'town', byid[rid].get('town'))
ext = float(__import__('os').environ.get('EXTEND', '0') or 0)
if ext > 0:   # custom: extend each route by EXTEND metres along the lane past its last waypoint (so post-scenario behaviour is visible)
    sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla
    for r in new.findall('route'):
        town = r.get('town'); xodr = next(f for f in [f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/{town}.xodr', f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/{town}/OpenDrive/{town}.xodr'] if __import__('os').path.exists(f))
        m = carla.Map(town, open(xodr).read()); wps = r.find('waypoints'); last = wps.findall('position')[-1]
        wp = m.get_waypoint(carla.Location(x=float(last.get('x')), y=float(last.get('y')), z=float(last.get('z'))), project_to_road=True, lane_type=carla.LaneType.Driving)
        cur, left = wp, ext
        while left > 0:
            nxt = cur.next(min(left, 20.0))
            if not nxt: break
            cur = nxt[0]; left -= 20.0
        ET.SubElement(wps, 'position', x=f'{cur.transform.location.x:.1f}', y=f'{cur.transform.location.y:.1f}', z=f'{cur.transform.location.z:.1f}')
        print(f'route {r.get("id")} extended by ~{ext:.0f} m to ({cur.transform.location.x:.0f},{cur.transform.location.y:.0f})')
ET.ElementTree(new).write(dst)
PYEOF
fi
# NOTE: LEAD's evaluator starts its own offscreen CARLA server (first free port from $PORT) and kills it on exit.
export LEAD_PROJECT_ROOT=$LEAD
export LD_LIBRARY_PATH=""
source $LEAD/scripts/main.sh
export CHECKPOINT_DIR=outputs/local_training/bridgedrive_k60
export BENCHMARK_ROUTE_ID=$RID
export EVALUATION_OUTPUT_DIR=outputs/local_evaluation/$RID
export PYTHONPATH=3rd_party/Bench2Drive/leaderboard:3rd_party/Bench2Drive/scenario_runner:$PYTHONPATH
export SCENARIO_RUNNER_ROOT=3rd_party/Bench2Drive/scenario_runner
export LEADERBOARD_ROOT=3rd_party/Bench2Drive/leaderboard
export IS_BENCH2DRIVE=1
export PLANNER_TYPE=only_traj
export SAVE_PATH=$EVALUATION_OUTPUT_DIR/
export PYTHONUNBUFFERED=1
export LEAD_CLOSED_LOOP_CONFIG="steer_modality=route throttle_modality=target_speed brake_modality=target_speed step_num=20 diffusion_speed=False"
export LEAD_TRAINING_CONFIG="diffusion_speed=False"
rm -rf $EVALUATION_OUTPUT_DIR; mkdir -p $EVALUATION_OUTPUT_DIR
echo ">>> BridgeDrive on route $RID  (output: $LEAD/$EVALUATION_OUTPUT_DIR)"
CUDA_VISIBLE_DEVICES=0 $PY 3rd_party/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator_v2.py \
    --routes=$ROUTES --track=SENSORS --checkpoint=$EVALUATION_OUTPUT_DIR/checkpoint_endpoint.json \
    --agent=lead/inference/sensor_agent_bridgedrive.py --agent-config=$CHECKPOINT_DIR \
    --debug=0 --record=None --resume=False --port=$PORT --traffic-manager-port=$TMPORT --timeout=60 \
    --debug-checkpoint=$EVALUATION_OUTPUT_DIR/debug_checkpoint/debug_checkpoint_endpoint.txt \
    --traffic-manager-seed=0 --repetitions=1 2>&1 | tee $WS/logs/bridgedrive_$RID.log
echo; bash $WS/show_bd_score.sh $RID
