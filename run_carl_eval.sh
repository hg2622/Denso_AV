#!/bin/bash
# CaRL / Roach on one Bench2Drive route with the CARLA leaderboard 2.0 evaluator (scored like Bench2Drive: DS, infractions).
#   bash run_carl_eval.sh ROUTE_ID [MODEL=CaRL_v1_1_PY_01] [PORT=2000]
# Starts its own CARLA server on PORT (killed at the end). Output: CaRL/CARLA/results/eval_b2d/<MODEL>/<ROUTE>.json (+ bev debug video)
set -u
# custom scenario knobs: SET="HighwayCutIn.no_cut_in=1,HighwayCutIn.cut_in_distance=5" bash run_..._eval.sh ROUTE  (custom scenario -> score not comparable)
export SCENARIO_OVERRIDES="${SET:-}"
# npc_track replay: freeze the episode's background-traffic spawner from frame 0 so deleted cars never appear and the scene is exactly the recording
case "${SCENARIO_OVERRIDES:-}" in *npc_track=*) export NPC_RECORD_BG_FREEZE=/tmp/npc_replay_bgfreeze_$$; : > $NPC_RECORD_BG_FREEZE; trap 'rm -f $NPC_RECORD_BG_FREEZE' EXIT; export SCENE_END="${SCENE_END:-3}"; echo "background spawner frozen (recorded scene); route ends ${SCENE_END}s after it";; esac
RID=${1:?route id}; MODEL=${2:-CaRL_v1_1_PY_01}; PORT=${3:-2000}
WS=/home/all/carla_ws; CARL=$WS/CaRL/CARLA; PY=/home/all/miniconda3/envs/carl/bin/python
OUT=$CARL/results/eval_b2d/$MODEL; mkdir -p $OUT $WS/logs
RFILE=$OUT/route_$RID.xml
$PY - "$WS/Bench2Drive/leaderboard/data/bench2drive220.xml" "$RID" "$RFILE" <<'PYEOF'
import sys, xml.etree.ElementTree as ET
src, rid, dst = sys.argv[1:]; root = ET.parse(src).getroot(); keep = [r for r in root.findall('route') if r.get('id') == rid]
assert keep, f'route {rid} not found'; new = ET.Element('routes'); new.append(keep[0]); print('route', rid, 'town', keep[0].get('town'))
ext = float(__import__('os').environ.get('EXTEND', '0') or 0)
if ext > 0:   # custom: extend the route EXTEND metres along the lane past its last waypoint
    sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, os
    for r in new.findall('route'):
        town = r.get('town'); xodr = next(f for f in [f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/{town}.xodr', f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/{town}/OpenDrive/{town}.xodr'] if os.path.exists(f))
        m = carla.Map(town, open(xodr).read()); wps = r.find('waypoints'); last = wps.findall('position')[-1]
        wp = m.get_waypoint(carla.Location(x=float(last.get('x')), y=float(last.get('y')), z=float(last.get('z'))), project_to_road=True, lane_type=carla.LaneType.Driving)
        cur, left = wp, ext
        while left > 0:
            nxt = cur.next(min(left, 20.0))
            if not nxt: break
            cur = nxt[0]; left -= 20.0
        ET.SubElement(wps, 'position', x=f'{cur.transform.location.x:.1f}', y=f'{cur.transform.location.y:.1f}', z=f'{cur.transform.location.z:.1f}'); print(f'route {r.get("id")} extended by ~{ext:.0f} m')
ET.ElementTree(new).write(dst)
PYEOF
# always a fresh server on PORT (a leftover one may be frozen in synchronous mode)
pkill -9 -f "rpc-port=$PORT " 2>/dev/null; for i in $(seq 1 30); do pgrep -f "rpc-port=$PORT " >/dev/null || break; sleep 1; done; sleep 2
(cd $WS/carla && ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Epic -carla-rpc-port=$PORT -carla-streaming-port=$((PORT+1)) > $WS/logs/carla_$PORT.log 2>&1 &)
for i in $(seq 1 60); do $PY -c "import carla; c=carla.Client('localhost',$PORT); c.set_timeout(2.0); c.get_world()" 2>/dev/null && break; sleep 2; done
STARTED=1
export CARLA_ROOT=$WS/carla WORK_DIR=$CARL
export SCENARIO_RUNNER_ROOT=$CARL/original_leaderboard/scenario_runner LEADERBOARD_ROOT=$CARL/original_leaderboard/leaderboard
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$SCENARIO_RUNNER_ROOT:$LEADERBOARD_ROOT:$CARL/team_code
export CUBLAS_WORKSPACE_CONFIG=:4096:8 DEBUG_ENV_AGENT=${DEBUG_ENV_AGENT:-0} SAVE_PATH=$OUT/debug_$RID ROUTES=route_$RID
case "$MODEL" in Roach*) export SAMPLE_TYPE=roach;; *) export SAMPLE_TYPE=mean;; esac
rm -f $OUT/$RID.json
cd $CARL && $PY $LEADERBOARD_ROOT/leaderboard/leaderboard_evaluator.py --routes=$RFILE --track=MAP --checkpoint=$OUT/$RID.json \
    --agent=$CARL/team_code/eval_agent.py --agent-config=$CARL/results/$MODEL --debug=0 --resume=0 --port=$PORT \
    --traffic-manager-port=$((PORT+6000)) --timeout=600 --repetitions=1 2>&1 | tee $WS/logs/carl_eval_${MODEL}_$RID.log | grep -v "^$" | tail -3
[ -n "${STARTED:-}" ] && pkill -f "rpc-port=$PORT " ; true
