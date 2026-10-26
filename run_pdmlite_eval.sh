#!/bin/bash
# PDM-Lite (privileged rule-based expert from the TransFuser++/SimLingo lineage; SimLingo/team_code/autopilot.py) on Bench2Drive routes.
#   bash run_pdmlite_eval.sh ROUTE_ID[,ROUTE_ID...] [PORT=8848]       SET="..." / EXTEND=N as usual; AGENT_TRACE=file for annotate_run.py
# Runs on the MAP track (it reads actors, map and lights straight from the simulator). Output: SimLingo/outputs/eval_b2d_pdmlite/<routes>/
set -u
ROUTE_ARG=${1:?route id}; PORT=${2:-8848}
WS=/home/all/carla_ws; SL=$WS/SimLingo; PY=/home/all/miniconda3/envs/simlingo/bin/python
export SCENARIO_OVERRIDES="${SET:-}"
case "${SCENARIO_OVERRIDES:-}" in *npc_track=*) export NPC_RECORD_BG_FREEZE=/tmp/npc_replay_bgfreeze_$$; : > $NPC_RECORD_BG_FREEZE; trap 'rm -f $NPC_RECORD_BG_FREEZE' EXIT; export SCENE_END="${SCENE_END:-3}";; esac
RID=${ROUTE_ARG//,/_}; OUT=$SL/outputs/eval_b2d_pdmlite/$RID; mkdir -p $OUT $WS/logs; RFILE=$OUT/route_$RID.xml
$PY - "$WS/Bench2Drive/leaderboard/data/bench2drive220.xml" "$ROUTE_ARG" "$RFILE" <<'PYEOF'
import sys, os, xml.etree.ElementTree as ET
src, rids, dst = sys.argv[1:]; rids = rids.split(',')
root = ET.parse(src).getroot(); byid = {r.get('id'): r for r in root.findall('route')}; new = ET.Element('routes')
for rid in rids:
    assert rid in byid, f'route {rid} not in bench2drive220.xml'; new.append(byid[rid]); print('route', rid, 'town', byid[rid].get('town'))
ext = float(os.environ.get('EXTEND', '0') or 0)
if ext > 0:
    sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla
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
pkill -9 -f "rpc-port=$PORT " 2>/dev/null; for i in $(seq 1 30); do pgrep -f "rpc-port=$PORT " >/dev/null || break; sleep 1; done
export CARLA_ROOT=$WS/carla WORK_DIR=$SL
export SCENARIO_RUNNER_ROOT=$SL/Bench2Drive/scenario_runner LEADERBOARD_ROOT=$SL/Bench2Drive/leaderboard
export PYTHONPATH=$SL:$SL/team_code:$CARLA_ROOT/PythonAPI/carla:$SCENARIO_RUNNER_ROOT:$LEADERBOARD_ROOT
export CUDA_VISIBLE_DEVICES=0 DEBUG_CHALLENGE=0 DATAGEN=0
unset SAVE_PATH   # the autopilot would otherwise try to log a dataset (needs TOWN/REPETITION env)
rm -f $OUT/checkpoint_endpoint.json
cd $SL && $PY -u Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py --routes=$RFILE --track=MAP --checkpoint=$OUT/checkpoint_endpoint.json \
    --agent=$SL/team_code/autopilot.py --agent-config="" --debug=0 --record=None --port=$PORT --traffic-manager-port=$((PORT+6000)) \
    --timeout=600 --repetitions=1 --gpu-rank=0 2>&1 | tee $WS/logs/pdmlite_$RID.log | grep -v "^$" | tail -3
pkill -9 -f "rpc-port=$PORT " 2>/dev/null; true
echo; $PY - $OUT/checkpoint_endpoint.json <<'PY'
import json, sys
try:
    r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
    print(f"PDM-Lite {r['route_id']}: DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
except Exception as e: print('no result:', e)
PY
