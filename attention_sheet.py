"""Figures for the AttentionTwin characterization (driver - vehicle - environment): what the twin measures around one
Level-2 vehicle, generated in CARLA where the environment state is exact.

    ~/miniconda3/envs/b2d/bin/python attention_sheet.py --out DIR [--town Town10HD] [--seconds 90]

Runs the ego on autopilot through traffic and pedestrians, logs the vehicle state (CAN-like) and the scene-demand
quantities every tick, then writes:
  driver_view.png    windshield view from the driver's eye point with the gaze zones a cabin camera classifies
  scene_state.png    bird's-eye ground truth at the most demanding moment: lead gap, TTC, nearest pedestrian, agents in 50 m
  timeseries.png     vehicle state + scene demand over the run, with the illustrative demand index and where it peaks
  demand_moments.png driver view at the 4 most demanding moments with their demand values
"""
import argparse, math, os, random, sys
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
import carla, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Polygon, Circle

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town10HD')
ap.add_argument('--seconds', type=float, default=90); ap.add_argument('--vehicles', type=int, default=40); ap.add_argument('--walkers', type=int, default=50); ap.add_argument('--seed', type=int, default=5)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); random.seed(a.seed); np.random.seed(a.seed)
DT = 0.05
client = carla.Client('localhost', 2000); client.set_timeout(180.0)
w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
for v in w.get_actors().filter('*vehicle*'): v.destroy()
for v in w.get_actors().filter('*walker*'): v.destroy()
st = w.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = DT; w.apply_settings(st); w.set_weather(carla.WeatherParameters.ClearNoon); w.tick()
m = w.get_map(); lib = w.get_blueprint_library(); tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)

def to_rgb(im): return np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].copy()

# ------------------------------------------------------------------ actors
spawns = m.get_spawn_points(); random.shuffle(spawns)
eb = lib.find('vehicle.tesla.model3'); eb.set_attribute('role_name', 'hero')
ego = None
for sp in spawns:
    ego = w.try_spawn_actor(eb, sp)
    if ego: break
cars = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4]
vehicles = []
for sp in spawns[1:]:
    if len(vehicles) >= a.vehicles: break
    if sp.location.distance(ego.get_location()) < 8: continue
    v = w.try_spawn_actor(random.choice(cars), sp)
    if v: v.set_autopilot(True, 8000); vehicles.append(v)
ego.set_autopilot(True, 8000); tm.ignore_walkers_percentage(ego, 0); tm.distance_to_leading_vehicle(ego, 4.0)
walkers, ctrls = [], []
wbps = lib.filter('walker.pedestrian.*'); cbp = lib.find('controller.ai.walker')
for _ in range(a.walkers):
    loc = w.get_random_location_from_navigation()
    if loc is None: continue
    wk = w.try_spawn_actor(random.choice(wbps), carla.Transform(loc))
    if wk: walkers.append(wk)
w.tick()
for wk in walkers: ctrls.append(w.spawn_actor(cbp, carla.Transform(), attach_to=wk))
w.tick()
w.set_pedestrians_cross_factor(0.3)
for c in ctrls: c.start(); c.go_to_location(w.get_random_location_from_navigation()); c.set_max_speed(1.0 + random.random())
print(f'{a.town}: ego + {len(vehicles)} vehicles + {len(walkers)} walkers', flush=True)

# cameras: driver eye point (left seat) through the windshield, and a bird's-eye camera
img = {}
def cam(name, loc, rot, fov, wpx, hpx):
    bp = lib.find('sensor.camera.rgb'); bp.set_attribute('image_size_x', str(wpx)); bp.set_attribute('image_size_y', str(hpx)); bp.set_attribute('fov', str(fov))
    s = w.spawn_actor(bp, carla.Transform(loc, rot), attach_to=ego); s.listen(lambda d, n=name: img.__setitem__(n, to_rgb(d))); return s
sensors = [cam('driver', carla.Location(x=0.2, y=-0.38, z=1.25), carla.Rotation(pitch=-4), 100, 1280, 720),
           cam('bev', carla.Location(z=42), carla.Rotation(pitch=-90), 75, 900, 900)]
for _ in range(40): w.tick()

# ------------------------------------------------------------------ per-tick state: CAN-like vehicle state + scene demand (ground truth)
def lead_vehicle():
    """nearest vehicle ahead in the ego's lane within 60 m -> (actor, gap m) or (None, None)"""
    et = ego.get_transform(); ewp = m.get_waypoint(et.location); fwd = et.get_forward_vector(); best = (None, None)
    for v in vehicles:
        if not v.is_alive: continue
        d = v.get_location() - et.location; along = d.x * fwd.x + d.y * fwd.y
        if along < 1.0 or along > 60: continue
        vwp = m.get_waypoint(v.get_location())
        if vwp.road_id != ewp.road_id or vwp.lane_id != ewp.lane_id: continue
        gap = along - ego.bounding_box.extent.x - v.bounding_box.extent.x
        if best[0] is None or gap < best[1]: best = (v, gap)
    return best

log = []; frames = {}; prev_v = None
N = int(a.seconds / DT)
for k in range(N):
    w.tick()
    et = ego.get_transform(); vel = ego.get_velocity(); ctl = ego.get_control(); acc = ego.get_acceleration()
    speed = math.hypot(vel.x, vel.y); fwd = et.get_forward_vector()
    ax = acc.x * fwd.x + acc.y * fwd.y; ay = -acc.x * fwd.y + acc.y * fwd.x
    lead, gap = lead_vehicle()
    if lead is not None:
        lv = lead.get_velocity(); closing = speed - math.hypot(lv.x, lv.y); ttc = gap / closing if closing > 0.3 else float('inf'); thw = gap / speed if speed > 0.5 else float('inf')
    else: ttc = thw = float('inf'); gap = float('nan')
    el = et.location; peds = [(wk, wk.get_location().distance(el)) for wk in walkers if wk.is_alive]; peds = [(p, d) for p, d in peds if d < 60]
    ped_d = min([d for _, d in peds], default=float('nan'))
    # pedestrian "ahead": within +-30 deg of heading
    ped_ahead = float('nan')
    for p, d in peds:
        dv = p.get_location() - el; ang = math.degrees(math.atan2(-dv.x * fwd.y + dv.y * fwd.x, dv.x * fwd.x + dv.y * fwd.y))
        if abs(ang) < 30: ped_ahead = min(ped_ahead, d) if not math.isnan(ped_ahead) else d
    n50 = sum(1 for v in vehicles if v.is_alive and v.get_location().distance(el) < 50) + sum(1 for _, d in peds if d < 50)
    wp = m.get_waypoint(el); junction = 1.0 if wp.is_junction else 0.0
    tl = ego.get_traffic_light(); tls = str(tl.get_state()) if tl is not None else 'none'
    # illustrative scene-demand index in [0,1]: how much attention this moment asks for
    demand = (0.30 * max(0.0, 1 - min(ttc, 8) / 8) + 0.25 * max(0.0, 1 - (ped_ahead if not math.isnan(ped_ahead) else 60) / 30)
              + 0.15 * min(n50, 15) / 15 + 0.15 * junction + 0.15 * min(speed, 20) / 20)
    log.append(dict(t=k * DT, speed=speed, ax=ax, ay=ay, steer=ctl.steer, throttle=ctl.throttle, brake=ctl.brake, gap=gap, thw=thw, ttc=ttc,
                    ped_d=ped_d, ped_ahead=ped_ahead, n50=n50, junction=junction, light=tls, demand=demand, x=el.x, y=el.y, yaw=et.rotation.yaw,
                    lead_id=lead.id if lead else -1))
    if k % 4 == 0 and 'driver' in img and 'bev' in img: frames[k] = (img['driver'].copy(), img['bev'].copy())
    if k % 400 == 0: print(f'  t={k*DT:5.1f}s speed {speed*3.6:5.1f} km/h gap {gap if not math.isnan(gap) else -1:5.1f} m ttc {min(ttc, 99):5.1f} s ped {ped_d if not math.isnan(ped_d) else -1:5.1f} m n50 {n50:2d} demand {demand:.2f}', flush=True)

T = np.array([r['t'] for r in log]); D = np.array([r['demand'] for r in log])
def arr(key, inf=np.nan): return np.array([(np.nan if (isinstance(r[key], float) and (math.isinf(r[key]) or math.isnan(r[key]))) else r[key]) for r in log], dtype=float)

# ------------------------------------------------------------------ gaze zones over the driver's view (what a cabin camera classifies)
def draw_gaze_zones(ax_, W=1280, H=720):
    zones = [('Road ahead', [(0.22, 0.10), (0.78, 0.10), (0.90, 0.58), (0.10, 0.58)], '#2ca02c'),
             ('Left mirror', [(0.00, 0.30), (0.12, 0.30), (0.12, 0.52), (0.00, 0.52)], '#1f77b4'),
             ('Right mirror', [(0.88, 0.30), (1.00, 0.30), (1.00, 0.52), (0.88, 0.52)], '#1f77b4'),
             ('Rear-view mirror', [(0.42, 0.00), (0.58, 0.00), (0.58, 0.09), (0.42, 0.09)], '#1f77b4'),
             ('Instrument cluster', [(0.14, 0.62), (0.44, 0.62), (0.44, 0.78), (0.14, 0.78)], '#ff7f0e'),
             ('Centre screen', [(0.52, 0.62), (0.80, 0.62), (0.80, 0.82), (0.52, 0.82)], '#d62728'),
             ('Lap / phone', [(0.25, 0.84), (0.75, 0.84), (0.75, 1.00), (0.25, 1.00)], '#d62728')]
    for name, pts, col in zones:
        poly = Polygon([(x * W, y * H) for x, y in pts], closed=True, fc=col, ec=col, alpha=0.22, lw=2); ax_.add_patch(poly)
        cx = np.mean([x for x, _ in pts]) * W; cy = np.mean([y for _, y in pts]) * H
        ax_.text(cx, cy, name, ha='center', va='center', fontsize=10, color='white', weight='bold', bbox=dict(boxstyle='round,pad=0.25', fc=col, ec='none', alpha=0.85))

# ------------------------------------------------------------------ pick the most demanding moments (at least 8 s apart)
order = np.argsort(-D); picks = []
for i in order:
    k = int(i) - int(i) % 4
    if k in frames and all(abs(k - p) > 8 / DT for p in picks): picks.append(k)
    if len(picks) == 4: break
kp = picks[0]; r = log[kp]
print('peak demand %.2f at t=%.1f s (ttc %.1f, ped_ahead %.1f, n50 %d, junction %d, speed %.1f km/h)' % (r['demand'], r['t'], min(r['ttc'], 99), r['ped_ahead'] if not math.isnan(r['ped_ahead']) else -1, r['n50'], r['junction'], r['speed'] * 3.6), flush=True)

# driver view with gaze zones (peak moment)
dimg, bimg = frames[kp]
fig, ax_ = plt.subplots(figsize=(12.8, 7.2)); ax_.imshow(dimg); draw_gaze_zones(ax_); ax_.axis('off')
ax_.set_title(f"Driver's view through the windshield (eye point, FOV 100) with the gaze zones a cabin camera classifies   t = {r['t']:.1f} s", fontsize=12)
fig.tight_layout(); fig.savefig(f'{a.out}/driver_view.png', dpi=120); plt.close(fig)

# scene state: annotated BEV at the peak moment
fig, ax_ = plt.subplots(figsize=(9, 9)); ax_.imshow(bimg); ax_.axis('off')
Wb, Hb = bimg.shape[1], bimg.shape[0]; mpp = (2 * 42 * math.tan(math.radians(75 / 2))) / Wb   # metres per pixel at ground level
def to_px(x, y):   # world -> image px for a top-down camera above the ego (image up = ego forward, CARLA yaw)
    dx, dy = x - r['x'], y - r['y']; yaw = math.radians(r['yaw']); f = dx * math.cos(yaw) + dy * math.sin(yaw); l = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return Wb / 2 + l / mpp, Hb / 2 - f / mpp
ax_.add_patch(Circle((Wb / 2, Hb / 2), 20 / mpp, fill=False, ec='white', ls='--', lw=1.2)); ax_.text(Wb / 2 + 20 / mpp * 0.72, Hb / 2 - 20 / mpp * 0.72, '20 m', color='white', fontsize=10)
ax_.plot(Wb / 2, Hb / 2, 's', color='red', ms=9, mec='white'); ax_.text(Wb / 2 + 12, Hb / 2 + 4, 'ego', color='red', fontsize=11, weight='bold')
lead_act = next((v for v in vehicles if v.id == r['lead_id']), None)
if lead_act is not None:
    lx, ly = to_px(lead_act.get_location().x, lead_act.get_location().y); ax_.plot([Wb / 2, lx], [Hb / 2, ly], color='yellow', lw=2); ax_.plot(lx, ly, 'o', color='yellow', ms=8, mec='k')
    ax_.text(lx + 10, ly, f"lead: gap {r['gap']:.1f} m\nheadway {min(r['thw'], 99):.1f} s, TTC {min(r['ttc'], 99):.1f} s", color='yellow', fontsize=10, weight='bold', bbox=dict(fc='black', alpha=0.5, ec='none'))
pbest = None
for wk in walkers:
    if wk.is_alive:
        d = wk.get_location().distance(carla.Location(x=r['x'], y=r['y'], z=0))
        if d < 60 and (pbest is None or d < pbest[1]): pbest = (wk, d)
if pbest is not None:
    px_, py_ = to_px(pbest[0].get_location().x, pbest[0].get_location().y); ax_.plot([Wb / 2, px_], [Hb / 2, py_], color='orange', lw=2); ax_.plot(px_, py_, 'o', color='orange', ms=8, mec='k')
    ax_.text(px_ + 10, py_, f'nearest pedestrian {pbest[1]:.1f} m', color='orange', fontsize=10, weight='bold', bbox=dict(fc='black', alpha=0.5, ec='none'))
ax_.text(12, 28, f"t = {r['t']:.1f} s   speed {r['speed']*3.6:.0f} km/h   agents within 50 m: {r['n50']}   junction: {'yes' if r['junction'] else 'no'}   light: {r['light']}   demand index {r['demand']:.2f}",
         color='white', fontsize=10.5, bbox=dict(fc='black', alpha=0.6, ec='none'))
ax_.set_xlim(0, Wb); ax_.set_ylim(Hb, 0)
ax_.set_title('Environment state the twin needs, exact in CARLA (bird\'s-eye, ego forward = up)', fontsize=12)
fig.tight_layout(); fig.savefig(f'{a.out}/scene_state.png', dpi=120); plt.close(fig)

# time series
fig, axs = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
axs[0].plot(T, arr('speed') * 3.6, color='k'); axs[0].set_ylabel('speed\n[km/h]')
axs[1].plot(T, arr('ax'), color='tab:blue', lw=0.8, label='longitudinal'); axs[1].plot(T, arr('ay'), color='tab:purple', lw=0.8, label='lateral'); axs[1].axhline(-2.94, color='r', ls='--', lw=0.8); axs[1].text(T[-1], -2.94, ' hard braking 0.3 g', color='r', fontsize=8, va='center'); axs[1].set_ylim(-9, 6); axs[1].set_ylabel('accel.\n[m/s$^2$]'); axs[1].legend(fontsize=8, loc='upper right', ncol=2)
axs[2].plot(T, arr('ttc'), color='tab:red', label='TTC to lead'); axs[2].plot(T, arr('thw'), color='tab:orange', label='time headway'); axs[2].set_ylim(0, 12); axs[2].axhline(1.5, color='r', ls=':', lw=0.8); axs[2].set_ylabel('[s]'); axs[2].legend(fontsize=8, loc='upper right', ncol=2)
axs[3].plot(T, arr('ped_ahead'), color='tab:orange', label='nearest pedestrian ahead [m]'); axs[3].plot(T, arr('n50'), color='tab:green', label='agents within 50 m'); axs[3].set_ylabel('[m] / [count]'); axs[3].legend(fontsize=8, loc='upper right', ncol=2)
axs[4].fill_between(T, 0, D, color='tab:red', alpha=0.35); axs[4].plot(T, D, color='tab:red'); axs[4].set_ylim(0, 1); axs[4].set_ylabel('scene demand\nindex [0-1]'); axs[4].set_xlabel('time [s]')
jn = arr('junction'); axs[4].fill_between(T, 0, 1, where=jn > 0.5, color='gray', alpha=0.15, label='in junction'); axs[4].legend(fontsize=8, loc='upper right')
for p in picks: axs[4].axvline(p * DT, color='k', lw=0.8, ls='--')
axs[0].set_title('Vehicle state (CAN-like) and scene demand (ground truth) over one run; dashed lines = the moments shown below', fontsize=11)
fig.tight_layout(); fig.savefig(f'{a.out}/timeseries.png', dpi=130); plt.close(fig)

# four demanding moments
fig, axs = plt.subplots(1, 4, figsize=(22, 3.8))
for ax_, p in zip(axs, picks):
    rr = log[p]; ax_.imshow(frames[p][0]); ax_.axis('off')
    ax_.set_title(f"t={rr['t']:.0f} s  demand {rr['demand']:.2f}  {rr['speed']*3.6:.0f} km/h  TTC {min(rr['ttc'], 99):.1f} s  ped {rr['ped_ahead'] if not math.isnan(rr['ped_ahead']) else 99:.0f} m  n50={rr['n50']}", fontsize=9.5)
fig.tight_layout(); fig.savefig(f'{a.out}/demand_moments.png', dpi=130); plt.close(fig)

# summary numbers for the document
sp = arr('speed') * 3.6; axv = arr('ax'); ttcv = arr('ttc'); pd = arr('ped_d'); n50v = arr('n50')
print('SUMMARY speed mean %.1f max %.1f km/h | ax min %.2f max %.2f m/s2 | hard-braking ticks %d | TTC<1.5s ticks %d | ped<10m ticks %d | n50 mean %.1f max %d | demand mean %.2f max %.2f | junction frac %.2f'
      % (np.nanmean(sp), np.nanmax(sp), np.nanmin(axv), np.nanmax(axv), int(np.sum(axv < -2.94)), int(np.nansum(ttcv < 1.5)), int(np.nansum(pd < 10)), np.nanmean(n50v), int(np.nanmax(n50v)), D.mean(), D.max(), jn.mean()), flush=True)
np.save(f'{a.out}/log.npy', log, allow_pickle=True)
for s in sensors:
    try: s.stop(); s.destroy()
    except Exception: pass
for c in ctrls:
    try: c.stop(); c.destroy()
    except Exception: pass
st = w.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; w.apply_settings(st)
print('done ->', a.out)
