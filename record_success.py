"""Record BridgeDrive runs on given routes (batched per town) and keep only the ones with NO collision.
  python3 record_success.py --out ~/carla_ws/selected_runs/avoid --batches "24759,26396,27506;2273,2286;24211"
"""
import argparse, json, os, re, subprocess, time
WS = '/home/all/carla_ws'; REC = '/home/all/miniconda3/envs/lead/bin/python'
ap = argparse.ArgumentParser(); ap.add_argument('--out', default=f'{WS}/selected_runs/avoid'); ap.add_argument('--batches', required=True); a = ap.parse_args()
os.makedirs(a.out, exist_ok=True); routes_md = open(f'{WS}/BENCH2DRIVE_ROUTES.md').read()
def info(rid):
    m = re.search(rf'^\| {rid}\*? \| (\w+) \| (\w+) \| ([\w/-]+) \| ([^|]*) \|', routes_md, re.M); return dict(town=m.group(1), scenario=m.group(2), ability=m.group(3), weather=m.group(4).strip()) if m else {}
def kill_all(): subprocess.run(f'bash {WS}/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; pkill -9 -f "record_her[o]"; sleep 3', shell=True)
summary = []
for batch in a.batches.split(';'):
    rids = batch.split(','); tag = '_'.join(rids); kill_all(); t0 = time.time()
    res = f'{WS}/LEAD/outputs/local_evaluation/{tag}/checkpoint_endpoint.json'; lp = f'{WS}/logs/bridgedrive_{tag}.log'
    for f in (res, lp):
        if os.path.exists(f): os.remove(f)
    print(f'[run] {info(rids[0]).get("town")} {rids} {[info(r).get("scenario") for r in rids]}', flush=True)
    ev = subprocess.Popen(f'bash {WS}/run_bridgedrive_eval.sh {batch}', shell=True, stdout=open(f'{a.out}/{tag}.log', 'w'), stderr=subprocess.STDOUT)
    port = 8848
    for _ in range(120):
        m = re.search(r'carla-rpc-port=(\d+)', open(lp).read()) if os.path.exists(lp) else None
        if m: port = int(m.group(1)); break
        time.sleep(1)
    video = f'{a.out}/tmp_{tag}.mp4'
    rec = subprocess.Popen([REC, f'{WS}/record_hero.py', '--port', str(port), '--out', video, '--label', f'BridgeDrive', '--wait', '1200', '--multi'], stdout=open(f'{a.out}/{tag}.rec.log', 'w'), stderr=subprocess.STDOUT)
    while ev.poll() is None and time.time() - t0 < 2400 * len(rids): time.sleep(5)
    for _ in range(12):
        if rec.poll() is not None: break
        time.sleep(5)
    if rec.poll() is None: rec.terminate()
    try: recs = json.load(open(res))['_checkpoint']['records']
    except Exception: recs = []
    for i, rid in enumerate(rids):
        r = next((x for x in recs if x['route_id'].endswith(f'_{rid}_rep0')), None); seg = video.replace('.mp4', f'_{i:02d}.mp4')
        inf = {k: v for k, v in (r['infractions'].items() if r else []) if v and k != 'min_speed_infractions'}
        ok = r is not None and r['status'] == 'Completed' and not any(k.startswith('collisions') for k in inf)
        name = f"{a.out}/bridgedrive_{rid}_{info(rid).get('scenario')}_DS{r['scores']['score_composed']:.0f}.mp4" if r else None
        if ok and os.path.exists(seg): os.replace(seg, name)
        elif os.path.exists(seg): os.remove(seg)
        summary.append(f"| {rid} | {info(rid).get('town')} | {info(rid).get('scenario')} | {info(rid).get('weather')} | {r['status'] if r else 'no-result'} | {r['scores']['score_composed'] if r else '-'} | {inf or '-'} | {'KEPT' if ok else 'discarded'} |")
        print(f"[done] {rid} {info(rid).get('scenario')}: {summary[-1]}", flush=True)
kill_all()
open(f'{a.out}/summary.md', 'w').write('# BridgeDrive: successful avoidance runs (no collision)\n\n| route | town | scenario | weather | status | DS | infractions (non-speed) | video |\n|---|---|---|---|---|---|---|---|\n' + '\n'.join(summary) + '\n')
print('[all done]', flush=True)
