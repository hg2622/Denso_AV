"""Run BridgeDrive and CaRL on the same Bench2Drive routes, record a chase-cam video of every run, collect scores.
Usage: python3 batch_eval.py [--routes 24211,24294,...] [--agents bridgedrive,carl] [--videos DIR]
Results: DIR/results.json (one record per run) and DIR/<agent>_<route>.mp4
"""
import argparse, json, os, subprocess, time, sys, glob
WS = '/home/all/carla_ws'
ap = argparse.ArgumentParser()
ap.add_argument('--routes', default='24211,24294,24816,26944,24759,25358,25863,24330')
ap.add_argument('--agents', default='carl,bridgedrive'); ap.add_argument('--videos', default=f'{WS}/eval_videos'); ap.add_argument('--carl-model', default='CaRL_v1_1_PY_01')
a = ap.parse_args(); os.makedirs(a.videos, exist_ok=True)
REC = '/home/all/miniconda3/envs/lead/bin/python'
results_path = os.path.join(a.videos, 'results.json'); results = json.load(open(results_path)) if os.path.exists(results_path) else []
done = {(r['agent'], r['route']) for r in results}

def kill_servers():
    subprocess.run(f'bash {WS}/stop_all.sh >/dev/null 2>&1; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; pkill -9 -f "leaderboard_evaluato[r]"; sleep 3', shell=True)

def parse_result(path):
    d = json.load(open(path))['_checkpoint']['records'][0]
    return dict(status=d['status'], DS=d['scores']['score_composed'], RC=d['scores']['score_route'], penalty=d['scores']['score_penalty'],
                infractions={k: len(v) for k, v in d['infractions'].items() if v},
                collisions=sum(len(v) for k, v in d['infractions'].items() if 'collision' in k), game_time=d['meta'].get('duration_game'))

for agent in a.agents.split(','):
    for rid in a.routes.split(','):
        if (agent, rid) in done: print(f'[skip] {agent} {rid} already done', flush=True); continue
        kill_servers(); t0 = time.time()
        if agent == 'bridgedrive':
            port = 8848; cmd = f'bash {WS}/run_bridgedrive_eval.sh {rid}'; res = f'{WS}/LEAD/outputs/local_evaluation/{rid}/checkpoint_endpoint.json'
        else:
            port = 2000; cmd = f'bash {WS}/run_carl_eval.sh {rid} {a.carl_model} {port}'; res = f'{WS}/CaRL/CARLA/results/eval_b2d/{a.carl_model}/{rid}.json'
        if os.path.exists(res): os.remove(res)
        if agent == 'bridgedrive' and os.path.exists(f'{WS}/logs/bridgedrive_{rid}.log'): os.remove(f'{WS}/logs/bridgedrive_{rid}.log')   # stale log would give a stale port
        video = os.path.join(a.videos, f'{agent}_{rid}.mp4')
        print(f'[run] {agent} route {rid} (port {port}) ...', flush=True)
        ev = subprocess.Popen(cmd, shell=True, stdout=open(os.path.join(a.videos, f'{agent}_{rid}.log'), 'w'), stderr=subprocess.STDOUT)
        if agent == 'bridgedrive':   # LEAD's evaluator picks the first free port from 8848: read the one it actually used
            import re; lp = f'{WS}/logs/bridgedrive_{rid}.log'
            for _ in range(90):
                m = re.search(r'carla-rpc-port=(\d+)', open(lp).read()) if os.path.exists(lp) else None
                if m: port = int(m.group(1)); break
                time.sleep(1)
            print(f'  evaluator CARLA port: {port}', flush=True)
        rec = subprocess.Popen([REC, f'{WS}/record_hero.py', '--port', str(port), '--out', video, '--label', f'{agent} {rid}', '--wait', '900'],
                               stdout=open(os.path.join(a.videos, f'{agent}_{rid}.rec.log'), 'w'), stderr=subprocess.STDOUT)
        while ev.poll() is None and time.time() - t0 < 1500: time.sleep(5)
        if ev.poll() is None: ev.kill(); print('[timeout] evaluator killed', flush=True)
        for _ in range(12):
            if rec.poll() is not None: break
            time.sleep(5)
        if rec.poll() is None: rec.terminate(); time.sleep(3)
        rec_info = dict(status='no-result'); 
        try: rec_info = parse_result(res)
        except Exception as e: rec_info['error'] = str(e)
        rec_info.update(agent=agent, route=rid, video=video if os.path.exists(video) else None, wall_s=round(time.time() - t0))
        results.append(rec_info); json.dump(results, open(results_path, 'w'), indent=1)
        print(f'[done] {agent} {rid}: {rec_info.get("status")} DS={rec_info.get("DS")} RC={rec_info.get("RC")} infr={rec_info.get("infractions")} video={"yes" if rec_info["video"] else "NO"} ({rec_info["wall_s"]} s)', flush=True)
kill_servers(); print('[batch finished]', flush=True)
