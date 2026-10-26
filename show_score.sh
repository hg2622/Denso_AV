#!/bin/bash
# Print Bench2Drive scores from a save dir's result.json.   bash show_score.sh ./eval_human
SAVE=${1:-./eval_human}; cd /home/all/carla_ws/Bench2Drive
/home/all/miniconda3/envs/b2d/bin/python - "$SAVE/result.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))["_checkpoint"]
for rec in r["records"]:
    s = rec["scores"]
    print(f"\n{rec['route_id']}  [{rec['status']}]")
    print(f"  Driving Score      : {s['score_composed']:6.2f}   (= route completion x infraction penalty)")
    print(f"  Route Completion   : {s['score_route']:6.2f} %")
    print(f"  Infraction Penalty : {s['score_penalty']:6.2f}")
    inf = {k: len(v) for k, v in rec["infractions"].items() if isinstance(v, list) and v}
    print("  Infractions        :", inf if inf else "none")
    print(f"  Game time {rec['meta']['duration_game']:.1f}s / wall {rec['meta']['duration_system']:.0f}s")
g = r.get("global_record") or {}
if g.get("scores"): print("\nGlobal:", {k: round(v, 2) for k, v in g["scores"].items()})
PY
