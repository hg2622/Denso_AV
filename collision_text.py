"""What was the model 'thinking' around each collision?  python collision_text.py --trace trace.jsonl --meta meta.jsonl [--before 3 --after 1]
Prints, per collision, the agent's per-tick text (SimLingo sentence) with speed / desired / brake from `before` s before impact to `after` s after.
Models without text (Drive-pi0 / DriveMoE) get the numbers only. Also handy at the END of a run: --at-end lists the last sentences."""
import argparse, json
ap = argparse.ArgumentParser(); ap.add_argument('--trace', required=True); ap.add_argument('--meta', required=True); ap.add_argument('--before', type=float, default=3.0); ap.add_argument('--after', type=float, default=1.0); ap.add_argument('--at-end', action='store_true'); ap.add_argument('--stride', type=float, default=0.5, help='also print a line at least every N s')
a = ap.parse_args()
trace = [json.loads(l) for l in open(a.trace) if l.strip()]
cols = [json.loads(l) for l in open(a.meta) if '"collision"' in l]
frames = [json.loads(l) for l in open(a.meta) if '"frame"' in l and '"kind": "frame"' in l]
def estimate_lag(frames, trace):
    # the leaderboard hands the agent a stale timestamp (~1 s on this setup): align the trace speed to the recorder speed
    import numpy as _np
    if not frames or not trace or not all('speed' in f for f in frames): return 0.0
    ft = _np.array([f['t'] for f in frames]); fs = _np.array([f['speed'] for f in frames]); tt = _np.array([r['t'] for r in trace]); ts = _np.array([r['speed'] * 3.6 for r in trace])
    if len(tt) < 10 or _np.std(fs) < 0.5: return 0.0
    best = min((float(_np.mean(_np.abs(_np.interp(ft, tt + lag, ts) - fs))), lag) for lag in _np.arange(-3.0, 3.001, 0.05))
    return float(best[1])
t0 = frames[0]['t'] if frames else (trace[0]['t'] if trace else 0.0)
LAG = estimate_lag(frames, trace)
for r in trace: r['t'] += LAG
print(f'(trace->recorder lag applied: {LAG:+.2f} s; times are seconds since the video start)')
def window(tc, before, after):
    last = None; last_t = None
    for r in trace:
        if tc - before <= r['t'] <= tc + after:
            txt = r.get('text') or ''
            mark = ' <-- IMPACT' if abs(r['t'] - tc) < 0.051 else ''
            core = txt.split(' in ')[0] if ' in ' in txt and 'meters' in txt else txt   # 'in 12.3 meters' varies every tick: collapse
            if core != last or mark or (r['t'] - (last_t if last_t is not None else -1e9)) >= a.stride:
                print(f"  t={r['t']-t0:6.2f}s  {r['speed']*3.6:5.1f} km/h  desired {r['desired']*3.6:5.1f}  thr {r['throttle']:.2f} brk {r['brake']:.2f}  | {txt}{mark}"); last = core; last_t = r['t']
if not cols: print('no collision in this run')
for c in cols:
    print(f"\nCOLLISION at t={c['t']-t0:.2f}s with {c['other']}  (model text / state from {a.before:.0f}s before to {a.after:.0f}s after; a line is printed when the sentence changes)")
    window(c['t'], a.before, a.after)
if a.at_end and trace:
    print('\nlast 3 s of the run:'); window(trace[-1]['t'], 3.0, 0.0)
