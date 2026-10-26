#!/bin/bash
# bash show_pi0_score.sh ROUTE_ID[_ROUTE_ID] [MODEL=drivepi0]
RID=${1:?route id}; MODEL=${2:-drivepi0}
/home/all/miniconda3/envs/drivemoe/bin/python - "/home/all/carla_ws/DriveMoE/outputs/eval_b2d/$MODEL/$RID/checkpoint_endpoint.json" <<'PY'
import json, sys
for r in json.load(open(sys.argv[1]))['_checkpoint']['records']:
    inf = {k: len(v) for k, v in r['infractions'].items() if v}
    print(f"{r['route_id']}  {r['status']}  DS={r['scores']['score_composed']:6.2f}  RC={r['scores']['score_route']:6.2f}%  penalty={r['scores']['score_penalty']:.3f}  infractions={inf}")
PY
