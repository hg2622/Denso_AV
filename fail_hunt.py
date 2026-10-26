"""Phase 1: run BridgeDrive on candidate Bench2Drive routes (give-way / cut-in / merging abilities) until N failures are found.
Phase 2: run CaRL on exactly those failed routes. Keep the videos of the failed BridgeDrive runs + the CaRL runs.
  python3 fail_hunt.py --n 10 --out ~/carla_ws/selected_runs/fail_hunt
"""
import argparse, json, os, re, subprocess, time
WS = '/home/all/carla_ws'; REC = '/home/all/miniconda3/envs/lead/bin/python'
ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=10); ap.add_argument('--out', default=f'{WS}/selected_runs/fail_hunt')
ap.add_argument('--candidates', default='/tmp/claude-1000/-home-all-Glass-killer/673a933b-c856-429a-9d85-faf1e5bc0c5e/scratchpad/fail_candidates.txt')
ap.add_argument('--carl-model', default='CaRL_v1_1_PY_01'); a = ap.parse_args()
os.makedirs(f'{a.out}/bridgedrive', exist_ok=True); os.makedirs(f'{a.out}/carl', exist_ok=True); os.makedirs(f'{a.out}/logs', exist_ok=True)
routes_md = open(f'{WS}/BENCH2DRIVE_ROUTES.md').read()
def route_info(rid):
    m = re.search(rf'^\| {rid}\*? \| (\w+) \| (\w+) \| ([\w/-]+) \| ([^|]*) \| (\d+) m \|', routes_md, re.M)
    return dict(town=m.group(1), scenario=m.group(2), ability=m.group(3), weather=m.group(4).strip(), length=int(m.group(5))) if m else {}
# collision-prone scenario types first (merging into flows, cut-ins, overtaking with oncoming traffic); round-robin for diversity
PRIORITY = ['HighwayCutIn', 'StaticCutIn', 'ParkingCutIn', 'MergerIntoSlowTraffic', 'MergerIntoSlowTrafficV2', 'EnterActorFlow', 'SequentialLaneChange',
            'HighwayExit', 'InterurbanActorFlow', 'InterurbanAdvancedActorFlow', 'CrossingBicycleFlow', 'NonSignalizedJunctionLeftTurnEnterFlow',
            'SignalizedJunctionLeftTurnEnterFlow', 'ParkingExit', 'AccidentTwoWays', 'ConstructionObstacleTwoWays', 'HazardAtSideLaneTwoWays',
            'ParkedObstacleTwoWays', 'VehicleOpensDoorTwoWays', 'InvadingTurn']
cands = open(a.candidates).read().strip().split(',')
extra = [m.group(1) for m in re.finditer(r'^\| (\d+)\*? \| \w+ \| (AccidentTwoWays|ConstructionObstacleTwoWays|HazardAtSideLaneTwoWays|ParkedObstacleTwoWays|VehicleOpensDoorTwoWays) \|', routes_md, re.M)]
cands = [r for r in cands + extra if route_info(r).get('scenario') in PRIORITY]
by = {}
for r in cands: by.setdefault(route_info(r).get('scenario', '?'), []).append(r)
order = []
while any(by.values()):
    for k in PRIORITY:
        if by.get(k): order.append(by[k].pop(0))
# group consecutive candidates by town into chunks (one CARLA boot + one map load per chunk)
def chunks(seq, n=4):
    pending = {}; out = []
    for r in seq:
        t = route_info(r).get('town'); pending.setdefault(t, []).append(r)
        if len(pending[t]) == n: out.append(pending.pop(t))
    out += [v for v in pending.values() if v]
    return out
state_path = f'{a.out}/state.json'; state = json.load(open(state_path)) if os.path.exists(state_path) else {'bridgedrive': {}, 'carl': {}}
def save(): json.dump(state, open(state_path, 'w'), indent=1)
def kill_all(): subprocess.run(f'bash {WS}/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; pkill -9 -f "record_her[o]"; sleep 3', shell=True)
def parse(path):
    d = json.load(open(path))['_checkpoint']['records'][0]
    return dict(status=d['status'], DS=d['scores']['score_composed'], RC=d['scores']['score_route'], infractions={k: len(v) for k, v in d['infractions'].items() if v},
                details={k: v for k, v in d['infractions'].items() if v and k != 'min_speed_infractions'})
def run(agent, rid):
    kill_all(); t0 = time.time(); multi = ',' in rid; tag = rid.replace(',', '_')
    if agent == 'bridgedrive':
        cmd = f'bash {WS}/run_bridgedrive_eval.sh {rid}'; res = f'{WS}/LEAD/outputs/local_evaluation/{tag}/checkpoint_endpoint.json'
        lp = f'{WS}/logs/bridgedrive_{tag}.log'
        if os.path.exists(lp): os.remove(lp)
    else:
        cmd = f'bash {WS}/run_carl_eval.sh {rid} {a.carl_model} 2000'; res = f'{WS}/CaRL/CARLA/results/eval_b2d/{a.carl_model}/{rid}.json'
    if os.path.exists(res): os.remove(res)
    video = f'{a.out}/{agent}/{agent}_{tag}.mp4'
    ev = subprocess.Popen(cmd, shell=True, stdout=open(f'{a.out}/logs/{agent}_{tag}.log', 'w'), stderr=subprocess.STDOUT)
    port = 2000
    if agent == 'bridgedrive':
        for _ in range(120):
            m = re.search(r'carla-rpc-port=(\d+)', open(lp).read()) if os.path.exists(lp) else None
            if m: port = int(m.group(1)); break
            time.sleep(1)
    rec = subprocess.Popen([REC, f'{WS}/record_hero.py', '--port', str(port), '--out', video, '--label', f'{agent} {tag}', '--wait', '1200'] + (['--multi'] if multi else []), stdout=open(f'{a.out}/logs/{agent}_{tag}.rec.log', 'w'), stderr=subprocess.STDOUT)
    while ev.poll() is None and time.time() - t0 < 2400 * (1 + rid.count(',')): time.sleep(5)
    if ev.poll() is None: ev.kill()
    for _ in range(12):
        if rec.poll() is not None: break
        time.sleep(5)
    if rec.poll() is None: rec.terminate()
    if multi:   # one record + one video segment per route, in XML order
        out = {}
        try: recs = json.load(open(res))['_checkpoint']['records']
        except Exception: recs = []
        rids = rid.split(',')
        for i, r_id in enumerate(rids):
            rec_i = next((x for x in recs if x['route_id'].endswith(f'_{r_id}_rep0') or x['route_id'] == f'RouteScenario_{r_id}'), None)
            seg = video.replace('.mp4', f'_{i:02d}.mp4'); final = f'{a.out}/{agent}/{agent}_{r_id}.mp4'
            if os.path.exists(seg): os.replace(seg, final)
            if rec_i is None: r = dict(status='no-result', DS=None, RC=None, infractions={}, details={})
            else: r = dict(status=rec_i['status'], DS=rec_i['scores']['score_composed'], RC=rec_i['scores']['score_route'], infractions={k: len(v) for k, v in rec_i['infractions'].items() if v}, details={k: v for k, v in rec_i['infractions'].items() if v and k != 'min_speed_infractions'})
            r.update(route=r_id, video=final if os.path.exists(final) else None, wall_s=round((time.time() - t0) / len(rids)), **route_info(r_id)); out[r_id] = r
        return out
    try: r = parse(res)
    except Exception as e: r = dict(status='no-result', DS=None, RC=None, infractions={}, details={}, error=str(e))
    r.update(route=rid, video=video if os.path.exists(video) else None, wall_s=round(time.time() - t0), **route_info(rid))
    return r
def is_fail(r):   # user rule: only COLLISIONS count as failures (not yield-to-emergency, lane, speed, etc.)
    return any(k.startswith('collisions') for k in (r.get('details') or {}))
for rid, v in state['bridgedrive'].items():   # re-apply the rule to runs already done
    v['failed'] = is_fail(v)
    if not v['failed']:
        for f in (f"{a.out}/bridgedrive/bridgedrive_{rid}.mp4", f"{a.out}/bridgedrive/bridgedrive_{rid}_model_view.mp4"):
            if os.path.exists(f): os.remove(f)
save()
fails = [r for r, v in state['bridgedrive'].items() if v.get('failed')]
for chunk in chunks([r for r in order if r not in state['bridgedrive']]):
    if len(fails) >= a.n: break
    print(f'[bd] {route_info(chunk[0]).get("town")} routes {chunk} ({[route_info(r).get("scenario") for r in chunk]}) ...', flush=True)
    results = run('bridgedrive', ','.join(chunk))
    for rid, r in results.items():
        r['failed'] = is_fail(r); state['bridgedrive'][rid] = r; save()
        print(f"[bd done] {rid} {r.get('scenario')}: {r['status']} DS={r['DS']} {r['details'] or r['infractions']} -> {'FAIL' if r['failed'] else 'pass'} (~{r['wall_s']} s)", flush=True)
        if r['failed']: fails.append(rid)
        elif r['video'] and os.path.exists(r['video']): os.remove(r['video'])   # only keep collisions
print(f'[phase1 done] {len(fails)} failures: {fails}', flush=True)
for rid in fails:
    if rid in state['carl']: continue
    info = route_info(rid); print(f'[carl] route {rid} {info.get("scenario")} ...', flush=True)
    r = run('carl', rid); state['carl'][rid] = r; save()
    print(f"[carl done] {rid}: {r['status']} DS={r['DS']} {r['details'] or r['infractions']} ({r['wall_s']} s)", flush=True)
kill_all()
lines = ['# BridgeDrive COLLISION hunt (give-way / cut-in / merging scenes) + CaRL on the same scenes', '',
         '| route | town | scenario | ability | weather | BridgeDrive | CaRL |', '|---|---|---|---|---|---|---|']
for rid in fails:
    b = state['bridgedrive'][rid]; c = state['carl'].get(rid, {})
    f = lambda r: f"{r.get('status')} DS={r.get('DS')} {r.get('details') or r.get('infractions')}" if r else '-'
    lines.append(f"| {rid} | {b.get('town')} | {b.get('scenario')} | {b.get('ability')} | {b.get('weather')} | {f(b)} | {f(c)} |")
lines += ['', '## BridgeDrive runs without a collision (videos deleted)', ', '.join(f"{r} ({v.get('scenario')}, DS {v.get('DS')})" for r, v in state['bridgedrive'].items() if not v.get('failed'))]
open(f'{a.out}/summary.md', 'w').write('\n'.join(lines) + '\n'); print('[all done]', flush=True)
