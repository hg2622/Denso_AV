#!/bin/bash
# Bench2Drive score of a BridgeDrive run.   bash show_bd_score.sh ROUTE_ID|bench2drive220
R=${1:?route id}; F=/home/all/carla_ws/LEAD/outputs/local_evaluation/$R/checkpoint_endpoint.json
python3 - "$F" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))["_checkpoint"]
for rec in d["records"]:
    s = rec["scores"]; inf = {k: len(v) for k, v in rec["infractions"].items() if v}
    print(f'{rec["route_id"]:>22}  {rec["status"]:<10} DS={s["score_composed"]:6.2f}  RC={s["score_route"]:6.2f}%  penalty={s["score_penalty"]:.3f}  infractions={inf}')
g = d.get("global_record") or {}
if g.get("scores"): print("GLOBAL:", g["scores"])
PY
