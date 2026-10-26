#!/bin/bash
# WA-JEPA (V-JEPA 2.1 world-action model, trained on nuPlan/NAVSIM; zero-shot in CARLA) on Bench2Drive routes.
# Uses the Bench2Drive evaluator in ~/carla_ws/Bench2Drive, which starts AND stops its own CARLA on PORT.
#   bash run_wajepa_eval.sh ROUTE_ID[,ROUTE_ID...] [PORT=8868]     SET="..." / EXTEND=N as usual; AGENT_TRACE=file for annotate_run.py --y-left
# Output: WA-JEPA/outputs/eval_b2d/<routes>/checkpoint_endpoint.json
set -u
ROUTE_ARG=${1:?route id}; PORT=${2:-8868}
WS=/home/all/carla_ws; WJ=/home/all/WA-JEPA; PY=/home/all/miniconda3/envs/wajepa/bin/python
AGENT=$WJ/close_loop/carla_agent.py; CFG=$WJ/configs/wa_jepa_hugsim.yaml
export SCENARIO_OVERRIDES="${SET:-}"
case "${SCENARIO_OVERRIDES:-}" in *npc_track=*) export NPC_RECORD_BG_FREEZE=/tmp/npc_replay_bgfreeze_$$; : > $NPC_RECORD_BG_FREEZE; trap 'rm -f $NPC_RECORD_BG_FREEZE' EXIT; export SCENE_END="${SCENE_END:-3}";; esac
RID=${ROUTE_ARG//,/_}; OUT=$WJ/outputs/eval_b2d/$RID; mkdir -p $OUT $WS/logs; RFILE=$OUT/route_$RID.xml
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
export CARLA_ROOT=$WS/carla IS_BENCH2DRIVE=True WAJEPA_CKPT=${WAJEPA_CKPT:-$WJ/ckpt} WAJEPA_CFG=$CFG
export SCENARIO_RUNNER_ROOT=$WS/Bench2Drive/scenario_runner LEADERBOARD_ROOT=$WS/Bench2Drive/leaderboard
export PYTHONPATH=$WJ:$CARLA_ROOT/PythonAPI/carla:$SCENARIO_RUNNER_ROOT:$LEADERBOARD_ROOT:$LEADERBOARD_ROOT/team_code
TOWN=$(grep -oE 'town="[A-Za-z0-9_]+"' $RFILE | head -1 | cut -d'"' -f2); QDEF=Low; [ "$TOWN" = "Town05" ] && QDEF=Epic
export SAVE_PATH=$OUT/viz CUDA_VISIBLE_DEVICES=0 CARLA_QUALITY=${CARLA_QUALITY:-$QDEF}
echo "CARLA quality: $CARLA_QUALITY ($TOWN)"
rm -rf $OUT/checkpoint_endpoint.json $OUT/viz
cd $WS/Bench2Drive && $PY -u $LEADERBOARD_ROOT/leaderboard/leaderboard_evaluator.py --routes=$RFILE --track=SENSORS --checkpoint=$OUT/checkpoint_endpoint.json \
    --agent=$AGENT --agent-config=$CFG --debug=0 --record=None --port=$PORT --traffic-manager-port=$((PORT+6000)) \
    --timeout=600 --repetitions=1 --gpu-rank=0 2>&1 | tee $WS/logs/wajepa_$RID.log | grep -v "^$" | tail -3
pkill -9 -f "rpc-port=$PORT " 2>/dev/null; true
echo; $PY - $OUT/checkpoint_endpoint.json <<'PY'
import json, sys
try:
    r = json.load(open(sys.argv[1]))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
    print(f"WA-JEPA {r['route_id']}: DS={r['scores']['score_composed']:.0f} RC={r['scores']['score_route']:.0f}% {r['status']} {inf or 'clean'} game={r['meta']['duration_game']:.0f}s wall={r['meta']['duration_system']:.0f}s")
except Exception as e: print('no result:', e)
PY
