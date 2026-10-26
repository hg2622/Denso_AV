"""Turn a recorded run (record_hero --meta meta.jsonl) into a reference track for npc_record.py:
  python ref_profile.py --meta <meta.jsonl> --route 24211 --label carl --out npc_tracks/ref_carl_24211.json [--ego-start 50]
t = 0 when the agent's car was first within --ego-start m of the scenario trigger point (the same start line as the recorder's ghost ego)."""
import argparse, json, math, xml.etree.ElementTree as ET
ap = argparse.ArgumentParser(); ap.add_argument('--meta', required=True); ap.add_argument('--route', required=True); ap.add_argument('--label', required=True); ap.add_argument('--out', required=True)
ap.add_argument('--ego-start', type=float, default=50.0); ap.add_argument('--scenario', default='DynamicObjectCrossing'); a = ap.parse_args()
root = ET.parse('/home/all/carla_ws/Bench2Drive/leaderboard/data/bench2drive220.xml').getroot(); route = next(r for r in root.findall('route') if r.get('id') == a.route)
tp = next(s for s in route.find('scenarios') if s.get('type') == a.scenario).find('trigger_point'); tx, ty = float(tp.get('x')), float(tp.get('y'))
fr = [json.loads(l) for l in open(a.meta) if '"kind": "frame"' in l]
poses = [(f['t'], f['hero'][0][3], f['hero'][1][3], f['hero'][2][3], math.degrees(math.atan2(f['hero'][1][0], f['hero'][0][0])), f['speed']) for f in fr]
t0 = next((t for t, x, y, z, yaw, v in poses if math.hypot(x - tx, y - ty) <= a.ego_start), None)
assert t0 is not None, 'the car never came within ego_start of the trigger'
track = [[round(t - t0, 3), round(x, 3), round(y, 3), round(z, 3), round(yaw, 2), round(v / 3.6, 2)] for t, x, y, z, yaw, v in poses if t - t0 >= -3.0]
json.dump({'route': a.route, 'scenario': a.scenario, 'label': a.label, 'ego_start': a.ego_start, 'trigger': [tx, ty], 'track': track}, open(a.out, 'w'))
print(f'{a.label}: {len(track)} poses, {track[0][0]:.1f}..{track[-1][0]:.1f} s, max {max(p[5] for p in track)*3.6:.0f} km/h -> {a.out}')
