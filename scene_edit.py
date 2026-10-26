"""Pass 2/3: edit a captured scene in 2D (no simulator at all).
  python scene_edit.py --scene scene1 [--horizon 8]
Mouse:  left click = select   drag = move   right click or DELETE = remove   double click = this is the car I will drive
Keys:   + / -  speed (m/s, 0 = stationary)      [ / ]  heading 15 deg (any actor)      r = reload      s or ESC = save and quit
Shown:  lanes (grey), route (dark green), ego start (white), cars (their colour), pedestrians (magenta), and for the selected actor
        a dashed path with markers at 2 / 5 / horizon seconds; the reference dots (CaRL / BridgeDrive) at the same instants.
Cars preview along their lane, pedestrians straight along their heading - exactly what the drive/replay passes do.
Writes the edits back into scenes/<name>.json (actors keep x/y/yaw/speed, removed ones are dropped, 'driven' marks your car)."""
import argparse, glob, json, math, os, sys
import numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--scene', required=True); ap.add_argument('--horizon', type=float, default=8.0)
ap.add_argument('--refs', default='auto'); ap.add_argument('--snapshot', default=''); a = ap.parse_args()
WS = '/home/all/carla_ws'; path = f'{WS}/scenes/{a.scene}.json'; S = json.load(open(path))
lanes = np.array([[l[0], l[1], l[2], l[3]] for l in S['lanes']]) if S['lanes'] else np.zeros((0, 4))
REFS = []
for i, f in enumerate(sorted(glob.glob(f"{WS}/npc_tracks/ref_*_{S['route_id']}.json")) if a.refs == 'auto' else [x for x in a.refs.split(',') if x]):
    r = json.load(open(f)); REFS.append((r['label'], r['track'], [(0, 220, 0), (60, 140, 255), (255, 0, 255), (255, 200, 0)][i % 4]))
def ref_pose(track, t):
    if t <= track[0][0]: return track[0]
    for i in range(len(track) - 1):
        if track[i][0] <= t <= track[i + 1][0]:
            p, q = track[i], track[i + 1]; f = (t - p[0]) / max(q[0] - p[0], 1e-3); return [t, p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2])] + p[3:]
    return track[-1]
W, H = 1400, 820; pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption(f'scene editor - {a.scene} (2D, no simulator)')
font = pygame.font.SysFont('monospace', 14, bold=True); big = pygame.font.SysFont('monospace', 20, bold=True)
cx, cy = S['centre']; yaw0 = math.radians(S['ego']['yaw'])   # rotate so the ego drives to the right
ca, sa = math.cos(-yaw0), math.sin(-yaw0)
def to_px(x, y, scale, px, py):
    dx, dy = x - cx, y - cy; rx = dx * ca - dy * sa; ry = dx * sa + dy * ca
    return int(W / 2 + rx * scale + px), int(H / 2 + ry * scale + py)
def to_world(u, v, scale, px, py):
    rx = (u - W / 2 - px) / scale; ry = (v - H / 2 - py) / scale
    dx = rx * ca + ry * sa; dy = -rx * sa + ry * ca; return cx + dx, cy + dy
def lane_ahead(x, y, yaw, dist):   # follow the captured lane points: nearest point, then step along the heading, re-snapping
    if len(lanes) == 0 or dist <= 0: return [(x, y)]
    pts = [(x, y)]; px, py, pyaw = x, y, yaw; step = 2.0; left = dist
    while left > 0:
        d = min(step, left); px += d * math.cos(math.radians(pyaw)); py += d * math.sin(math.radians(pyaw)); left -= d
        k = int(np.argmin((lanes[:, 0] - px) ** 2 + (lanes[:, 1] - py) ** 2))
        if (lanes[k, 0] - px) ** 2 + (lanes[k, 1] - py) ** 2 < 9.0:
            px, py = float(lanes[k, 0]), float(lanes[k, 1])
            lyaw = float(lanes[k, 2])
            if abs(((lyaw - pyaw + 180) % 360) - 180) < 100: pyaw = lyaw
        pts.append((px, py))
    return pts
def fit_scale():   # zoom so the actors + route fill the window
    xs = [S['ego']['x'], S['trigger'][0]] + [x['x'] for x in S['actors']] + [p[0] for p in S['route']]
    ys = [S['ego']['y'], S['trigger'][1]] + [x['y'] for x in S['actors']] + [p[1] for p in S['route']]
    pts = [( (x - cx) * ca - (y - cy) * sa, (x - cx) * sa + (y - cy) * ca ) for x, y in zip(xs, ys)]
    ex = max(abs(q[0]) for q in pts) + 15; ey = max(abs(q[1]) for q in pts) + 15
    return max(3.0, min((W / 2) / ex, (H / 2) / ey, 22.0))
scale = fit_scale(); pan = [0, 0]; sel = None; drag = None; last_click = (None, 0.0); driven = next((x for x in S['actors'] if x.get('driven')), None)
import time
def save():
    S['actors'] = [x for x in S['actors'] if not x.get('_dead')]
    for x in S['actors']: x['driven'] = (x is driven)
    json.dump(S, open(path, 'w'), indent=1); print(f"saved {len(S['actors'])} actors (driven: {driven['label'] if driven else 'none'}) -> {path}", flush=True)
def actor_at(pos):
    best = None
    for x in S['actors']:
        if x.get('_dead'): continue
        u, v = to_px(x['x'], x['y'], scale, *pan); d = math.hypot(u - pos[0], v - pos[1])
        if d < 18 and (best is None or d < best[0]): best = (d, x)
    return best[1] if best else None
run = True; snap_n = 0
while run:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: save(); run = False
        elif ev.type == pygame.KEYDOWN:
            if ev.key in (pygame.K_ESCAPE, pygame.K_s): save(); run = False
            elif ev.key == pygame.K_r: S = json.load(open(path)); sel = None; driven = next((x for x in S['actors'] if x.get('driven')), None)
            elif sel is not None:
                if ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS, pygame.K_UP): sel['speed'] = round(min(25.0, sel['speed'] + 1.0), 1)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS, pygame.K_DOWN): sel['speed'] = round(max(0.0, sel['speed'] - 1.0), 1)
                elif ev.key in (pygame.K_LEFTBRACKET, pygame.K_LEFT): sel['yaw'] = (sel['yaw'] - 15.0) % 360
                elif ev.key in (pygame.K_RIGHTBRACKET, pygame.K_RIGHT): sel['yaw'] = (sel['yaw'] + 15.0) % 360
                elif ev.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                    sel['_dead'] = True
                    if sel is driven: driven = None
                    sel = None
        elif ev.type == pygame.MOUSEBUTTONDOWN:
            x_ = actor_at(ev.pos)
            if ev.button == 3 and x_ is not None:
                x_['_dead'] = True
                if x_ is driven: driven = None
                if x_ is sel: sel = None
            elif ev.button == 1 and x_ is not None:
                if last_click[0] is x_ and time.time() - last_click[1] < 0.45:
                    if x_['kind'] == 'walker': print('pedestrians cannot be driven', flush=True)
                    else: driven = x_; print(f"you will drive {x_['label']}", flush=True)
                    drag = None
                else: drag = (x_, ev.pos, False); last_click = (x_, time.time())
                sel = x_
            elif ev.button == 1: sel = None
            elif ev.button == 4: scale = min(30.0, scale * 1.15)
            elif ev.button == 5: scale = max(2.0, scale / 1.15)
        elif ev.type == pygame.MOUSEMOTION and drag is not None and ev.buttons[0]:
            if math.hypot(ev.pos[0] - drag[1][0], ev.pos[1] - drag[1][1]) > 4:
                x_, _, _ = drag; wx, wy = to_world(ev.pos[0], ev.pos[1], scale, *pan); x_['x'], x_['y'] = wx, wy; drag = (x_, ev.pos, True)
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1: drag = None
    screen.fill((28, 28, 30))
    for l in S['lanes']:   # lane points drawn as road-width patches -> reads as a road
        u, v = to_px(l[0], l[1], scale, *pan)
        if -60 < u < W + 60 and -60 < v < H + 60:
            pygame.draw.circle(screen, (58, 58, 62), (u, v), max(2, int(l[3] * scale * 0.5)))
    rp = [to_px(p[0], p[1], scale, *pan) for p in S['route']]
    if len(rp) > 1: pygame.draw.lines(screen, (0, 90, 40), False, rp, 3)
    tu, tv = to_px(S['trigger'][0], S['trigger'][1], scale, *pan); pygame.draw.circle(screen, (255, 160, 0), (tu, tv), 7, 2); screen.blit(font.render('trigger', True, (255, 160, 0)), (tu + 9, tv - 7))
    eu, ev_ = to_px(S['ego']['x'], S['ego']['y'], scale, *pan); pygame.draw.circle(screen, (240, 240, 240), (eu, ev_), 10, 3); screen.blit(font.render('EGO start (the model under test)', True, (240, 240, 240)), (eu + 12, ev_ - 8))
    for label, track, col in REFS:
        p0 = ref_pose(track, 0.0); u, v = to_px(p0[1], p0[2], scale, *pan); pygame.draw.circle(screen, col, (u, v), 6)
        screen.blit(font.render(f'{label}', True, col), (u + 9, v + 6 + 14 * REFS.index((label, track, col))))
    for x in S['actors']:
        if x.get('_dead'): continue
        u, v = to_px(x['x'], x['y'], scale, *pan); col = tuple(int(c) for c in x['color'].split(','))
        if x['kind'] == 'walker': pygame.draw.circle(screen, col, (u, v), 7)
        else:
            L, Wd = 4.6 * scale / 2, 2.0 * scale / 2; yr = math.radians(x['yaw']) - yaw0
            pts = [(u + L * math.cos(yr) - Wd * math.sin(yr), v + L * math.sin(yr) + Wd * math.cos(yr)),
                   (u + L * math.cos(yr) + Wd * math.sin(yr), v + L * math.sin(yr) - Wd * math.cos(yr)),
                   (u - L * math.cos(yr) + Wd * math.sin(yr), v - L * math.sin(yr) - Wd * math.cos(yr)),
                   (u - L * math.cos(yr) - Wd * math.sin(yr), v - L * math.sin(yr) + Wd * math.cos(yr))]
            pygame.draw.polygon(screen, col, pts); pygame.draw.polygon(screen, (0, 0, 0), pts, 1)
        if x is driven: pygame.draw.circle(screen, (255, 255, 0), (u, v), 20, 3)
        if x is sel: pygame.draw.circle(screen, (255, 255, 255), (u, v), 15, 1)
        tag = f"{x['label']}  {x['speed']:.0f} m/s" + ('  [YOU]' if x is driven else '')
        lab = font.render(tag, True, col); bg = pygame.Surface((lab.get_width() + 6, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0))
        dy = -26 if (S['actors'].index(x) % 2 == 0) else 14
        screen.blit(bg, (u + 10, v + dy - 1)); screen.blit(lab, (u + 13, v + dy))
    if sel is not None and not sel.get('_dead'):   # future path of the selection + the agents at the same instants
        T = a.horizon
        path_pts = lane_ahead(sel['x'], sel['y'], sel['yaw'], sel['speed'] * T) if sel['kind'] == 'car' else [(sel['x'] + sel['speed'] * t * math.cos(math.radians(sel['yaw'])), sel['y'] + sel['speed'] * t * math.sin(math.radians(sel['yaw']))) for t in np.linspace(0, T, 40)]
        px = [to_px(p[0], p[1], scale, *pan) for p in path_pts]
        for i in range(0, len(px) - 1, 2): pygame.draw.line(screen, (255, 255, 0), px[i], px[i + 1], 2)
        for t in (2.0, 5.0, T):
            frac = t / T if T > 0 else 0; k = min(len(px) - 1, int(frac * (len(px) - 1)))
            pygame.draw.circle(screen, (255, 255, 0), px[k], 9, 2); screen.blit(font.render(f'{t:.0f}s', True, (255, 255, 0)), (px[k][0] + 10, px[k][1] - 6))
            for label, track, col in REFS:
                r = ref_pose(track, t); u, v = to_px(r[1], r[2], scale, *pan); pygame.draw.circle(screen, col, (u, v), 7, 2)
                screen.blit(font.render(f'{label} {t:.0f}s', True, col), (u + 9, v + 7))
        d = sel['speed'] * T
        screen.blit(font.render(f"{sel['label']}: {sel['speed']:.0f} m/s -> {d:.0f} m in {T:.0f} s   (+/- speed, [ ] heading)", True, (255, 255, 0)), (12, H - 26))
    n_live = len([x for x in S['actors'] if not x.get('_dead')])
    screen.blit(big.render(f'{a.scene}: {n_live} actors   click select | drag move | +/- speed | [ ] heading | right-click/DEL remove | double-click = drive | S save+quit', True, (255, 255, 0)), (12, 10))
    msg = f"YOU DRIVE: {driven['label']}" if driven else 'DOUBLE-CLICK a car to choose the one you will drive'
    screen.blit(font.render(msg + '     wheel = zoom     S = save and continue', True, (255, 255, 0) if driven else (255, 120, 120)), (12, 34))
    pygame.display.flip()
    if a.snapshot:
        snap_n += 1
        if snap_n == 10:
            sel = next((x for x in S['actors'] if x['kind'] == 'walker'), S['actors'][0]); sel['speed'] = 2.0
        if snap_n == 20: pygame.image.save(screen, a.snapshot); print('snapshot saved', flush=True); save(); run = False
    pygame.time.wait(16)
pygame.quit()
