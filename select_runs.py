"""Pick 3 good and 3 bad runs from batch_eval results, keep only their videos (+ BridgeDrive's own debug video when available).
  python3 select_runs.py [--videos ~/carla_ws/eval_videos] [--out ~/carla_ws/selected_runs]
"""
import argparse, json, os, shutil, glob
ap = argparse.ArgumentParser(); ap.add_argument('--videos', default='/home/all/carla_ws/eval_videos'); ap.add_argument('--out', default='/home/all/carla_ws/selected_runs')
a = ap.parse_args(); R = [r for r in json.load(open(os.path.join(a.videos, 'results.json'))) if r.get('video')]
def badness(r):   # higher = worse: collisions, failures, blocked, low score
    s = 0; s += 100 * r.get('collisions', 0); s += 50 * (r.get('status') != 'Completed'); s += 30 * ('route_dev' in str(r.get('infractions')) or 'blocked' in str(r.get('infractions')))
    return s + (100 - (r.get('DS') or 0))
def goodness(r): return (r.get('status') == 'Completed') * 1000 - 1000 * r.get('collisions', 0) + (r.get('DS') or 0)
good, bad, used = [], [], set()
for r in sorted(R, key=goodness, reverse=True):      # best first, prefer one per agent first
    if len(good) == 3: break
    if (r['agent'], r['route']) in used: continue
    good.append(r); used.add((r['agent'], r['route']))
for r in sorted(R, key=badness, reverse=True):
    if len(bad) == 3: break
    if (r['agent'], r['route']) in used: continue
    bad.append(r); used.add((r['agent'], r['route']))
if os.path.exists(a.out): shutil.rmtree(a.out)
lines = ['# Selected runs\n', '| set | agent | route | status | DS | RC | infractions | video |', '|---|---|---|---|---|---|---|---|']
for tag, lst in (('good', good), ('bad', bad)):
    d = os.path.join(a.out, tag); os.makedirs(d, exist_ok=True)
    for r in lst:
        name = f"{r['agent']}_{r['route']}_DS{r['DS']:.0f}_{r['status']}.mp4"; shutil.copy(r['video'], os.path.join(d, name))
        if r['agent'] == 'bridgedrive':
            dbg = f"/home/all/carla_ws/LEAD/outputs/local_evaluation/{r['route']}/{r['route']}_debug.mp4"
            if os.path.exists(dbg): shutil.copy(dbg, os.path.join(d, name.replace('.mp4', '_model_view.mp4')))
        lines.append(f"| {tag} | {r['agent']} | {r['route']} | {r['status']} | {r['DS']:.1f} | {r['RC']:.0f} | {r.get('infractions')} | {tag}/{name} |")
lines += ['', '## All runs', '| agent | route | status | DS | RC | penalty | infractions |', '|---|---|---|---|---|---|---|']
for r in sorted(R, key=lambda r: (r['agent'], r['route'])): lines.append(f"| {r['agent']} | {r['route']} | {r.get('status')} | {r.get('DS')} | {r.get('RC')} | {r.get('penalty')} | {r.get('infractions')} |")
open(os.path.join(a.out, 'summary.md'), 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines[:12]))
# delete the non-selected videos
keep = {r['video'] for r in good + bad}
for f in glob.glob(os.path.join(a.videos, '*.mp4')):
    if f not in keep: os.remove(f)
print(f'\nselected videos in {a.out}; deleted {len(R) - len(keep)} non-selected videos')
