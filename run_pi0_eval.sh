#!/bin/bash
# Drive-pi0 / DriveMoE (CVPR'26 VLA on PaliGemma-3B, Thinklab-SJTU/DriveMoE) on Bench2Drive routes.
# Uses the Bench2Drive evaluator in ~/carla_ws/Bench2Drive, which starts AND stops its own CARLA on PORT.
#   bash run_pi0_eval.sh ROUTE_ID[,ROUTE_ID...] [MODEL=drivepi0|drivemoe] [PORT=8858]     SET="..." / EXTEND=N as usual
# Output: DriveMoE/outputs/eval_b2d/<MODEL>/<routes>/checkpoint_endpoint.json  (+ viz/ meta per frame)
set -u
ROUTE_ARG=${1:?route id}; MODEL=${2:-drivepi0}; PORT=${3:-8858}
WS=/home/all/carla_ws; DM=$WS/DriveMoE; PY=/home/all/miniconda3/envs/drivemoe/bin/python
case $MODEL in
  drivepi0) AGENT=$DM/src/agent/team_code/drivepi0_carla_agent.py; CFG=$DM/config/eval/DrivePi0/closed_loop_local.yaml ;;
  drivemoe) AGENT=$DM/src/agent/team_code/drivemoe_carla_agent.py; CFG=$DM/config/eval/DriveMoE/closed_loop_local.yaml ;;
  *) echo "MODEL must be drivepi0 or drivemoe"; exit 1 ;;
esac
export SCENARIO_OVERRIDES="${SET:-}"
# npc_track replay: freeze the episode's background-traffic spawner from frame 0 so deleted cars never appear and the scene is exactly the recording
case "${SCENARIO_OVERRIDES:-}" in *npc_track=*) export NPC_RECORD_BG_FREEZE=/tmp/npc_replay_bgfreeze_$$; : > $NPC_RECORD_BG_FREEZE; trap 'rm -f $NPC_RECORD_BG_FREEZE' EXIT; export SCENE_END="${SCENE_END:-3}"; echo "background spawner frozen (recorded scene); route ends ${SCENE_END}s after it";; esac
RID=${ROUTE_ARG//,/_}; OUT=$DM/outputs/eval_b2d/$MODEL/$RID; mkdir -p $OUT $WS/logs; RFILE=$OUT/route_$RID.xml
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
export CARLA_ROOT=$WS/carla DRIVEMOE_REPO_DIR=$DM REPO_DIR=$DM IS_BENCH2DRIVE=True PLANNER_TYPE=traj
export SCENARIO_RUNNER_ROOT=$WS/Bench2Drive/scenario_runner LEADERBOARD_ROOT=$WS/Bench2Drive/leaderboard
export PYTHONPATH=$DM:$DM/src/agent:$CARLA_ROOT/PythonAPI/carla:$SCENARIO_RUNNER_ROOT:$LEADERBOARD_ROOT   # DM/src/agent provides team_code.planner
TOWN=$(grep -oE 'town="[A-Za-z0-9_]+"' $RFILE | head -1 | cut -d'"' -f2); QDEF=Low; [ "$TOWN" = "Town05" ] && QDEF=Epic   # Low-quality CARLA segfaults at route start on Town05 (4/4 runs, even with 800x450 cameras); Epic is fine there
export SAVE_PATH=$OUT/viz CUDA_VISIBLE_DEVICES=0 TF_CPP_MIN_LOG_LEVEL=2 CARLA_QUALITY=${CARLA_QUALITY:-$QDEF}   # Low elsewhere: 3.5 B model + CARLA must fit in 11.5 GB
echo "CARLA quality: $CARLA_QUALITY ($TOWN)"
rm -rf $OUT/checkpoint_endpoint.json $OUT/viz
cd $WS/Bench2Drive && $PY -u $LEADERBOARD_ROOT/leaderboard/leaderboard_evaluator.py --routes=$RFILE --track=SENSORS --checkpoint=$OUT/checkpoint_endpoint.json \
    --agent=$AGENT --agent-config=$CFG --debug=0 --record=None --port=$PORT --traffic-manager-port=$((PORT+6000)) \
    --timeout=600 --repetitions=1 --gpu-rank=0 2>&1 | tee $WS/logs/pi0_${MODEL}_$RID.log | grep -v "^$" | tail -3
pkill -9 -f "rpc-port=$PORT " 2>/dev/null; true
echo; bash $WS/show_pi0_score.sh $RID $MODEL
