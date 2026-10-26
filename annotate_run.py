"""Overlay an agent's per-tick trace on a record_hero video: predicted path on chase + BEV, inference text, HUD,
collision markers and a scrolling speed / throttle / brake profile; also writes <out>.png (speed profile).
  python annotate_run.py --video raw.mp4 --meta meta.jsonl --trace trace.jsonl --out annotated.mp4 --label "SimLingo 2286" [--y-left]
--y-left: the trace's ego frame has y to the LEFT (SimLingo/transfuser); default y right (CARLA / Drive-pi0)."""
import argparse, json, bisect, textwrap, os, numpy as np, cv2
def estimate_lag(frames, trace):
    # the leaderboard hands the agent a stale timestamp (~1 s on this setup): align the trace speed to the recorder speed
    import numpy as _np
    if not frames or not trace or not all('speed' in f for f in frames): return 0.0
    ft = _np.array([f['t'] for f in frames]); fs = _np.array([f['speed'] for f in frames]); tt = _np.array([r['t'] for r in trace]); ts = _np.array([r['speed'] * 3.6 for r in trace])
    if len(tt) < 10 or _np.std(fs) < 0.5: return 0.0
    best = min((float(_np.mean(_np.abs(_np.interp(ft, tt + lag, ts) - fs))), lag) for lag in _np.arange(-3.0, 3.001, 0.05))
    return float(best[1])
ap = argparse.ArgumentParser(); ap.add_argument('--video', required=True, help='raw video; if missing/unreadable the recorder\'s .avi sibling is used'); ap.add_argument('--meta', required=True); ap.add_argument('--trace', required=True, help="agent trace JSONL, or 'none' to use the recorder's own speed/controls")
ap.add_argument('--out', required=True); ap.add_argument('--label', default=''); ap.add_argument('--y-left', action='store_true'); ap.add_argument('--score', default='')
a = ap.parse_args()
cams, frames, cols = None, [], []
for line in open(a.meta):
    r = json.loads(line)
    if r['kind'] == 'cams': cams = r
    elif r['kind'] == 'frame': frames.append(r)
    elif r['kind'] == 'collision': cols.append(r)
if a.trace == 'none':   # agents without a trace (e.g. Think2Drive: controls only): build it from the recorder's meta
    trace = [{'step': f['i'], 't': f['t'], 'speed': f['speed'] / 3.6, 'desired': float('nan'), 'throttle': f.get('throttle', 0.0), 'brake': f.get('brake', 0.0), 'steer': f.get('steer', 0.0), 'route': [], 'speed_wps': [], 'text': ''} for f in frames]
else:
    trace = [json.loads(l) for l in open(a.trace) if l.strip()]
LAG = 0.0 if a.trace == 'none' else estimate_lag(frames, trace)
for r in trace: r['t'] += LAG
tt = [r['t'] for r in trace]
print(f'trace->recorder lag applied: {LAG:+.2f} s')
_g = []
for c in cols:
    if not _g or c['t'] - _g[-1]['t'] > 1.0: _g.append(c)
cols = _g
T0 = frames[0]['t'] if frames else 0.0; T1 = frames[-1]['t'] if frames else 1.0
def project(inv, w, h, fov, pts_world):
    P = np.c_[pts_world, np.ones(len(pts_world))] @ np.asarray(inv).T; f = (w / 2.0) / np.tan(np.radians(fov) / 2); x = np.maximum(P[:, 0], 1e-3)
    return np.c_[f * P[:, 1] / x + w / 2.0, -f * P[:, 2] / x + h / 2.0], P[:, 0] > 0.3
def ego_to_world(M, pts):
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    if a.y_left: pts = pts * np.array([1.0, -1.0])
    P = np.c_[pts, np.zeros(len(pts)), np.ones(len(pts))] @ np.asarray(M).T
    return P[:, :3] + np.array([0, 0, 0.3])
def draw_path(img, inv, w, h, fov, pts_world, color, r=3, line=True, xoff=0, labels=None):
    """labels: {index: text} drawn next to those path points (e.g. {1: '1s', 3: '2s', 7: '4s'} for a 2 Hz plan)"""
    if not len(pts_world): return
    uv, ok = project(inv, w, h, fov, pts_world); prev = None
    for k, ((u, v), good) in enumerate(zip(uv, ok)):
        if not good or not (-50 < u < w + 50 and -50 < v < h + 50): prev = None; continue
        pt = (int(u) + xoff, int(v))
        if line and prev is not None: cv2.line(img, prev, pt, color, 2)
        cv2.circle(img, pt, r, color, -1); prev = pt
        if labels and k in labels: cv2.putText(img, labels[k], (pt[0] + 5, pt[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3); cv2.putText(img, labels[k], (pt[0] + 5, pt[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
cap = cv2.VideoCapture(a.video)
if not cap.isOpened() or int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 0:
    _avi = os.path.splitext(a.video)[0] + '.avi'
    if os.path.exists(_avi): cap = cv2.VideoCapture(_avi); print('raw mp4 unreadable -> using', _avi); fps = cap.get(cv2.CAP_PROP_FPS) or 20; W = int(cap.get(3)); H = int(cap.get(4)); CW, BW = cams['chase']['w'], cams['bev']['w']
STRIP = 130; vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (W, H + STRIP))
# speed profile data (from the trace: the agent's own view of speed / desired)
ts = np.array(tt); sp = np.array([r['speed'] for r in trace]) * 3.6; de = np.array([r['desired'] for r in trace]) * 3.6
th = np.array([r['throttle'] for r in trace]); br = np.array([r['brake'] for r in trace]); col_ts = [c['t'] for c in cols]
def strip_img(t_now):
    img = np.full((STRIP, W, 3), 25, np.uint8); L, R, top, bot = 50, W - 10, 12, STRIP - 18
    def X(t): return int(L + (R - L) * (t - T0) / max(T1 - T0, 1e-3))
    vmax = max(5.0, float(np.nanmax(np.r_[sp, de])) * 1.1) if len(sp) else 40
    def Y(v, lo, hi): return int(bot - (bot - top) * (v - lo) / (hi - lo))
    cv2.line(img, (L, bot), (R, bot), (90, 90, 90), 1); cv2.line(img, (L, top), (L, bot), (90, 90, 90), 1)
    for name, arr, lo, hi, color in [('thr', th, 0, 1, (80, 200, 80)), ('brk', br, 0, 1, (60, 60, 230)), ('desired', de, 0, vmax, (0, 165, 255)), ('speed', sp, 0, vmax, (255, 255, 255))]:
        pts = [(X(t), Y(v, lo, hi)) for t, v in zip(ts, arr) if np.isfinite(v)]
        if len(pts) > 1: cv2.polylines(img, [np.array(pts, np.int32)], False, color, 1 if name in ('thr', 'brk') else 2)
    for ct in col_ts: cv2.line(img, (X(ct), top), (X(ct), bot), (0, 0, 255), 2); cv2.putText(img, 'collision', (X(ct) + 3, top + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    cv2.line(img, (X(t_now), top), (X(t_now), bot), (0, 255, 255), 1)
    cv2.putText(img, f'{vmax:.0f} km/h', (2, top + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1); cv2.putText(img, '0', (30, bot + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    cv2.putText(img, 'speed (white)  desired (orange)  throttle (green)  brake (red)   time ->', (L + 5, STRIP - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
    return img
i = 0; col_frames = {c['i'] for c in cols}; flash = 0
while True:
    ok, img = cap.read()
    if not ok or i >= len(frames): break
    fr = frames[i]; k = bisect.bisect_right(tt, fr['t'] + 1e-6) - 1; tr = trace[k] if k >= 0 else None
    if tr is not None and tr['route']:
        LB = {1: '1s', 3: '2s', 5: '3s', 7: '4s'} if len(tr['route']) == 8 else None
        pw = ego_to_world(fr['hero'], tr['route']); draw_path(img[:, :CW], fr['chase'], CW, H, cams['chase']['fov'], pw, (0, 140, 255), labels=LB); draw_path(img[:, CW:], fr['bev'], BW, H, cams['bev']['fov'], pw, (0, 140, 255), labels=LB)
        if tr['speed_wps']:
            pw2 = ego_to_world(fr['hero'], tr['speed_wps']); draw_path(img[:, :CW], fr['chase'], CW, H, cams['chase']['fov'], pw2, (0, 0, 255), r=4, line=False); draw_path(img[:, CW:], fr['bev'], BW, H, cams['bev']['fov'], pw2, (0, 0, 255), r=3, line=False)
    y = 52
    if tr is not None:
        hud = f"model: speed {tr['speed']*3.6:4.1f} km/h  desired {(tr['desired']*3.6 if tr['desired']==tr['desired'] else float('nan')):4.1f}  thr {tr['throttle']:.2f} brk {tr['brake']:.2f} steer {tr['steer']:+.2f}   (age {fr['t']-tr['t']:.2f}s)"
        cv2.putText(img, hud, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3); cv2.putText(img, hud, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1); y += 24
        for ln in textwrap.wrap(tr.get('text') or '', 70)[:3]:
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3); cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1); y += 22
    if i in col_frames: flash = 15
    if flash > 0: cv2.rectangle(img, (2, 2), (W - 3, H - 3), (0, 0, 255), 6); cv2.putText(img, 'COLLISION', (CW // 2 - 90, H - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3); flash -= 1
    if a.score: cv2.putText(img, a.score, (CW + 8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3); cv2.putText(img, a.score, (CW + 8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(img, 'orange: predicted route   red: speed waypoints', (CW + 8, H - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3); cv2.putText(img, 'orange: predicted route   red: speed waypoints', (CW + 8, H - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    vw.write(np.vstack([img, strip_img(fr['t'])])); i += 1
vw.release(); cap.release()
# the VLM's sentence every time it changes -> <out>.text_changes.txt (next to the video), for agents that have one
if any((r.get('text') or '').strip() for r in trace):
    import sys as _sys; _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from text_changes import write_changes
    _n = write_changes(trace, os.path.splitext(a.out)[0] + '.text_changes.txt', a.trace); print(f'{_n} sentence changes -> {os.path.splitext(a.out)[0]}.text_changes.txt')
# speed profile png
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
ax[0].plot(ts - T0, sp, 'k-', lw=1.5, label='speed'); ax[0].plot(ts - T0, de, color='tab:orange', lw=1, label='desired (model)'); ax[0].set_ylabel('km/h'); ax[0].legend(loc='upper left'); ax[0].grid(alpha=.3)
ax[1].plot(ts - T0, th, 'g-', lw=1, label='throttle'); ax[1].plot(ts - T0, br, 'r-', lw=1, label='brake'); ax[1].plot(ts - T0, [r['steer'] for r in trace], 'b-', lw=0.8, alpha=.6, label='steer'); ax[1].set_ylabel('control'); ax[1].set_xlabel('sim time [s]'); ax[1].legend(loc='upper left'); ax[1].grid(alpha=.3)
for ct in col_ts:
    for x in ax: x.axvline(ct - T0, color='red', lw=2, alpha=.6)
fig.suptitle(f'{a.label}  {a.score}'); fig.tight_layout(); fig.savefig(a.out.rsplit('.', 1)[0] + '.png', dpi=110)
print(f'annotated {i} frames -> {a.out}; collisions at t = {[round(c - T0, 1) for c in col_ts]}')
