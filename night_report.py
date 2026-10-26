"""Build selected_runs/night/REPORT.md from the overnight runs: per-model table (route, type, DS, infractions, speed stats),
success / collision picks, links to videos + speed profiles, and the SimLingo sentence around each collision.
  python night_report.py [--night selected_runs/night] [--custom selected_runs/custom]"""
import argparse, json, os, glob, subprocess, numpy as np
ap = argparse.ArgumentParser(); ap.add_argument('--night', default='/home/all/carla_ws/selected_runs/night'); ap.add_argument('--custom', default='/home/all/carla_ws/selected_runs/custom'); a = ap.parse_args()
PY = '/home/all/miniconda3/envs/lead/bin/python'; WS = '/home/all/carla_ws'
def stats(trace_path, meta_path):
    if not os.path.exists(meta_path): return {}
    fr = [json.loads(l) for l in open(meta_path) if '"kind": "frame"' in l]
    if not fr: return {}
    sp = np.array([f['speed'] for f in fr]); br = np.array([f.get('brake', 0.0) for f in fr]); dt = np.median(np.diff([f['t'] for f in fr])) if len(fr) > 2 else 0.05
    if 'brake' not in fr[0] and os.path.exists(trace_path):   # early runs: controls only in the agent trace
        br = np.array([json.loads(l)['brake'] for l in open(trace_path) if l.strip()])
    hard = int(np.sum((br[1:] >= 0.9) & (br[:-1] < 0.9)))   # brake-slam events
    dec = np.diff(sp / 3.6) / dt if len(sp) > 2 else np.array([0.0])
    return {'max_kmh': float(sp.max()), 'mean_kmh': float(sp.mean()), 'stopped_pct': float(100 * np.mean(sp < 1.0)), 'brake_slams': hard, 'max_decel': float(-np.percentile(dec, 1))}
def rec(ck):
    r = json.load(open(ck))['_checkpoint']['records'][0]; inf = {k: len(v) for k, v in r['infractions'].items() if v and k != 'min_speed_infractions'}
    return r['scores']['score_composed'], r['scores']['score_route'], r['status'], inf, r['meta'].get('duration_game', 0), r['meta'].get('duration_system', 0)
out = ['# Overnight baseline runs — ' + os.popen('date +%Y-%m-%d').read().strip(), '',
       'Models in their original form: SimLingo with its sentence every tick (1 Hz), Drive-pi0 / DriveMoE (NF4-quantised backbone, the only way they fit beside CARLA here), Think2Drive (world-model RL, own pipeline).',
       'Each run: annotated video (orange = predicted route, red = speed waypoints, yellow = the model\'s sentence, strip = speed / desired / throttle / brake, red bars = collisions), `.png` speed profile, `trace.jsonl` (per-tick model state), `collision_text.txt`.', '']
def section(title, runs):
    out.append(f'## {title}'); out.append('')
    out.append('| model | route | scenario | DS | RC | infractions | max / mean km/h | stopped % | brake slams | video |'); out.append('|---|---|---|---|---|---|---|---|---|---|')
    picks = {'success': [], 'collision': []}
    for m, route, typ, ck, d, video in runs:
        if not os.path.exists(ck): out.append(f'| {m} | {route} | {typ} | – | – | NO RESULT | | | | |'); continue
        ds, rc, st, inf, g, w = rec(ck); s = stats(f'{d}/trace.jsonl', f'{d}/meta.jsonl')
        vid = f'[{os.path.basename(video)}]({os.path.relpath(video, a.night)})' if video and os.path.exists(video) else '(no video)'
        out.append(f"| {m} | {route} | {typ} | {ds:.0f} | {rc:.0f}% | {inf or 'clean'} | {s.get('max_kmh', float('nan')):.0f} / {s.get('mean_kmh', float('nan')):.0f} | {s.get('stopped_pct', float('nan')):.0f} | {s.get('brake_slams', 0)} | {vid} |")
        key = 'collision' if any('collision' in k for k in inf) else ('success' if ds >= 99 else None)
        if key: picks[key].append((m, route, typ, ds, video))
    out.append('')
    return picks
# standard routes
TYPE = {'2286': 'HighwayCutIn', '26396': 'StaticCutIn', '24211': 'DynamicObjectCrossing', '24759': 'ParkingCutIn', '2273': 'MergerIntoSlowTraffic', '27506': 'EnterActorFlow', '23658': 'HighwayExit'}
runs = []
for m in ['simlingo', 'drivepi0', 'drivemoe', 'think2drive']:
    for route in TYPE:
        if m == 'think2drive':
            d = f'{WS}/CaRL/Think2Drive/custom_runs/std_{route}'; ck = f'{d}/checkpoint_endpoint.json'
        else:
            d = f'{a.night}/{m}/{route}'; ck = f'{d}/checkpoint_endpoint.json'
        vids = glob.glob(f'{a.night}/{m}/{route}_*.mp4'); runs.append((m, route, TYPE[route], ck, d, vids[0] if vids else ''))
picks = section('Standard Bench2Drive routes (cut-in / crossing / merge)', runs)
# custom scenes
NAMES = ['2286_HighwayCutIn_SYNC_then_CONST', '2286_HighwayCutIn_CONST_MEET', '2286_HighwayCutIn_NO_CUT_IN_fast', '24759_ParkingCutIn_NO_CUT_IN', '24211_DynamicObjectCrossing_NO_CROSS', '26396_StaticCutIn_LATE_CUT']
cruns = []
for m in ['simlingo', 'drivepi0', 'drivemoe', 'think2drive']:
    for n in NAMES:
        d = f'{a.custom}/_vlm_work/{m}_{n}' if m != 'think2drive' else f'{WS}/CaRL/Think2Drive/custom_runs/{n}'
        cruns.append((m, n.split('_')[0], n.split('_', 1)[1], f'{d}/checkpoint_endpoint.json', d, f'{a.custom}/{m}_{n}.mp4'))
cpicks = section('Custom scenes (same overrides as the BridgeDrive / CaRL videos)', cruns)
out.append('## Picks'); out.append('')
for key in ['success', 'collision']:
    out.append(f'**{key}**'); out.append('')
    for m, route, typ, ds, video in picks[key] + cpicks[key]: out.append(f'- {m} {route} {typ} DS {ds:.0f} — {os.path.relpath(video, a.night) if video else "no video"}')
    out.append('')
out.append('## What the model said around each collision (SimLingo only has text; others: speed / brake)'); out.append('')
for m, route, typ, ck, d, video in runs + cruns:
    tr, me = f'{d}/trace.jsonl', f'{d}/meta.jsonl'
    if os.path.exists(tr) and os.path.exists(me) and os.path.exists(ck) and any('collision' in k for k in rec(ck)[3]):
        txt = subprocess.run([PY, f'{WS}/collision_text.py', '--trace', tr, '--meta', me, '--before', '3', '--after', '0.3', '--stride', '0.5'], capture_output=True, text=True).stdout
        out.append(f'### {m} {route} {typ}'); out.append('```'); out.append(txt.strip()[:6000]); out.append('```'); out.append('')
open(f'{a.night}/REPORT.md', 'w').write('\n'.join(out)); print('wrote', f'{a.night}/REPORT.md', len(out), 'lines')
