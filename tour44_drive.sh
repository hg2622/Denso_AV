#!/bin/bash
# YOU drive Bench2Drive scenarios in one window (scored like any agent). Pick what to drive by scenario type name:
#   bash tour44_drive.sh HighwayCutIn                  # one route of that type
#   bash tour44_drive.sh "HighwayCutIn PedestrianCrossing T_Junction"   # these types back to back
#   bash tour44_drive.sh cutin                         # case-insensitive substring works (HighwayCutIn, ParkingCutIn, StaticCutIn)
#   bash tour44_drive.sh 5                             # the 5th type in the list (05 ConstructionObstacleTwoWays);  "5+" = from the 5th to the 44th
#   bash tour44_drive.sh 2286                          # a specific route id (bench2drive220.xml)
#   bash tour44_drive.sh all                           # the whole 44-type tour, resumes where it stopped ("all fresh" = start over)
#   bash tour44_drive.sh list                          # show the 44 types and their route
# Keys: W/S throttle/brake, A/D steer, Shift = gentle, Q reverse, Space handbrake, N = skip route, ESC = quit.
# HUD: "NN/44 ScenarioType (route, Town)", next turn + distance; bright green marker = 20 m ahead on the route, dim dots = route, red dot = goal.  Scores print at the end.
WS=/home/all/carla_ws; B2D=$WS/Bench2Drive; PY=/home/all/miniconda3/envs/b2d/bin/python; LIST=$WS/tour44_routes.txt
SEL=${1:-list}
if [ "$SEL" = list ]; then grep -v '^#' $LIST | awk '{printf "  %s %-42s route %-6s %s\n",$1,$2,$3,$4}'; echo; echo "usage: bash tour44_drive.sh <TypeName|substring|route_id|all>"; exit 0; fi
if [ "$SEL" = all ]; then XML=$B2D/leaderboard/data/tour44.xml; SAVE=$B2D/eval_tour44; [ "${2:-}" = fresh ] && rm -f $SAVE/result.json
else
  RIDS=""
  for w in $SEL; do
    if [[ "$w" =~ ^[0-9]{1,2}\+$ ]]; then RIDS="$RIDS $(grep -v '^#' $LIST | awk -v n="${w%+}" '$1+0>=n+0 {print $3}' | tr '\n' ' ')"; continue; fi   # "5+" = from the 5th type to the 44th
    if [[ "$w" =~ ^[0-9]{1,2}$ ]]; then HIT=$(grep -v '^#' $LIST | awk -v n="$w" '$1+0==n+0 {print $3}'); [ -z "$HIT" ] && { echo "no scenario number $w (1-44)"; exit 1; }; RIDS="$RIDS $HIT"; continue; fi   # "5" = the 5th type
    if [[ "$w" =~ ^[0-9]+$ ]]; then RIDS="$RIDS $w"; continue; fi
    HIT=$(grep -v '^#' $LIST | awk -v q="$(tr 'A-Z' 'a-z' <<<"$w")" 'tolower($2)==q {print $3}')
    [ -z "$HIT" ] && HIT=$(grep -v '^#' $LIST | awk -v q="$(tr 'A-Z' 'a-z' <<<"$w")" 'index(tolower($2),q) {print $3}')
    [ -z "$HIT" ] && { echo "no scenario type matches '$w'. Types:"; grep -v '^#' $LIST | awk '{print "  "$2}'; exit 1; }
    RIDS="$RIDS $HIT"
  done
  echo "driving:"; for r in $RIDS; do grep -w "$r" $LIST | awk '{printf "  %s %s (route %s, %s)\n",$1,$2,$3,$4}' || echo "  route $r"; done
  XML=$B2D/leaderboard/data/tour_pick.xml; SAVE=$B2D/eval_pick; rm -f $SAVE/result.json
  $PY - $B2D/leaderboard/data/bench2drive220.xml $XML $RIDS <<'PYEOF'
import sys, xml.etree.ElementTree as ET
src, dst, *rids = sys.argv[1:]; byid = {r.get('id'): r for r in ET.parse(src).getroot().findall('route')}; new = ET.Element('routes')
for rid in rids:
    assert rid in byid, f'route {rid} not in bench2drive220.xml'; new.append(byid[rid])
ET.ElementTree(new).write(dst)
PYEOF
fi
mkdir -p $SAVE; cd $B2D
[ "${DRY:-0}" = 1 ] && { echo "(dry run) routes xml: $XML"; grep -o '<route [^>]*>' $XML; exit 0; }
export CARLA_ROOT=$WS/carla
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:leaderboard:leaderboard/team_code:scenario_runner:Bench2DriveZoo
export SCENARIO_RUNNER_ROOT=scenario_runner LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS REPETITIONS=1 RESUME=True DEBUG_CHALLENGE=0
export IS_BENCH2DRIVE=True SAVE_PATH=$SAVE TOUR_ROUTES=$LIST TOUR_XML=$B2D/leaderboard/data/tour44.xml
export DISPLAY=${DISPLAY:-:1} MIRRORS=${MIRRORS:-1}
NROUTES=$(grep -c "<route " $XML)
done_count() { $PY -c "
import json,sys
try: r=json.load(open('$SAVE/result.json'))['_checkpoint']; print(sum(1 for x in r['records'] if x.get('status','') not in ('Started','') and 'crashed' not in x.get('status','')))
except Exception: print(0)" 2>/dev/null; }
for attempt in 1 2 3 4 5 6; do   # a crashed / hung CARLA (big-town load) ends the evaluator; relaunch and resume until all routes are done or you press ESC
  pkill -9 -f "leaderboard_evaluato[r].py" 2>/dev/null; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null; sleep 2
  [ $attempt -gt 1 ] && echo "[tour] CARLA or the evaluator died - relaunching and resuming (attempt $attempt)"
  CUDA_VISIBLE_DEVICES=0 $PY leaderboard/leaderboard/leaderboard_evaluator.py \
    --routes $XML --repetitions 1 --track SENSORS \
    --checkpoint "$SAVE/result.json" --agent leaderboard/team_code/human_b2d_agent.py \
    --agent-config leaderboard/leaderboard/autoagents/human_agent_config.txt \
    --debug 0 --record "" --resume True --port 30000 --traffic-manager-port 50000 --gpu-rank 0 --timeout 240 2>&1 | tee $WS/logs/tour44_drive.log \
    | sed -u -E 's/^.*> Loading the world.*$/[tour] loading the town (a new town takes 1-2 min, the window reappears after)/' \
    | grep --line-buffered -E "Preparing|\[tour\]|Results of|could not be loaded|Error|Traceback"
  pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng" 2>/dev/null
  grep -q "\[tour\] ESC pressed" $WS/logs/tour44_drive.log && break
  [ "$(done_count)" -ge "$NROUTES" ] && break
done
echo; bash $WS/show_score.sh $SAVE 2>/dev/null | tail -40
