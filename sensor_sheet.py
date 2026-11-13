"""Visual catalogue of the CARLA twin for documentation: every sensor modality from one ego at one instant, a bird's-eye
view with the lane graph and all agents, overviews of several towns, and the same street under several weathers.

    ~/miniconda3/envs/b2d/bin/python sensor_sheet.py --out DIR [--town Town10HD] [--towns Town03,Town04,Town05,Town10HD]

Writes individual PNGs plus montages (sensors.png, bev.png, towns.png, weather.png). Needs a CARLA server on :2000.
"""
import argparse, math, os, queue, random, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
import carla, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town10HD')
ap.add_argument('--towns', default='Town03,Town04,Town05,Town10HD'); ap.add_argument('--vehicles', type=int, default=45); ap.add_argument('--walkers', type=int, default=30)
ap.add_argument('--seed', type=int, default=3); ap.add_argument('--no-towns', action='store_true'); ap.add_argument('--settle', type=int, default=160, help='ticks of traffic before the capture')
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); random.seed(a.seed); np.random.seed(a.seed)
DT = 0.05
client = carla.Client('localhost', 2000); client.set_timeout(180.0)

def load(town):
    w = client.get_world()
    if town not in w.get_map().name: w = client.load_world(town)
    for v in w.get_actors().filter('*vehicle*'): v.destroy()
    for v in w.get_actors().filter('*walker*'): v.destroy()
    s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = DT; w.apply_settings(s)
    w.set_weather(carla.WeatherParameters.ClearNoon); w.tick(); return w

def to_rgb(im, conv=None):
    if conv is not None: im.convert(conv)
    return np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].copy()

def aerial(w, cx, cy, half, out, px=1600):
    bp = w.get_blueprint_library().find('sensor.camera.rgb'); bp.set_attribute('image_size_x', str(px)); bp.set_attribute('image_size_y', str(px)); bp.set_attribute('fov', '60')
    z = half / math.tan(math.radians(30)); got = []
    cam = w.spawn_actor(bp, carla.Transform(carla.Location(x=cx, y=cy, z=min(z, 1900.0)), carla.Rotation(pitch=-90, yaw=-90))); cam.listen(got.append)
    for _ in range(40):
        w.tick()
        if len(got) >= 4: break
    img = to_rgb(got[-1]); cam.stop(); cam.destroy(); cv2.imwrite(out, img[:, :, ::-1]); return img

# ------------------------------------------------------------------ town overviews
towns = []
for t in ([] if a.no_towns else a.towns.split(',')):
    w = load(t); m = w.get_map(); pts = np.array([[p.transform.location.x, p.transform.location.y] for p in m.generate_waypoints(5.0)])
    lo, hi = pts.min(0), pts.max(0); c = (lo + hi) / 2; half = float(max(hi - lo) / 2 + 40)
    img = aerial(w, c[0], c[1], half, f'{a.out}/town_{t}.png'); towns.append((t, img, 2 * half))
    print(f'{t}: extent {2*half:.0f} m, {len(pts)} lane samples', flush=True)
if towns:
  fig, axs = plt.subplots(1, len(towns), figsize=(4.2 * len(towns), 4.6))
  for ax, (t, img, ext) in zip(np.atleast_1d(axs), towns):
    ax.imshow(img); ax.set_title(f'{t}  ({ext:.0f} m x {ext:.0f} m)', fontsize=11); ax.axis('off')
  fig.tight_layout(); fig.savefig(f'{a.out}/towns.png', dpi=160); plt.close(fig)

# ------------------------------------------------------------------ populated scene for the sensor sheet
w = load(a.town); m = w.get_map(); lib = w.get_blueprint_library(); tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)
spawns = m.get_spawn_points(); random.shuffle(spawns)
ego_bp = lib.find('vehicle.lincoln.mkz_2020'); ego_bp.set_attribute('role_name', 'hero')
ego = None
for sp in spawns:
    ego = w.try_spawn_actor(ego_bp, sp)
    if ego: break
cars = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4]
vehicles = []
for sp in spawns[1:]:
    if len(vehicles) >= a.vehicles: break
    if sp.location.distance(ego.get_location()) < 8: continue
    v = w.try_spawn_actor(random.choice(cars), sp)
    if v: v.set_autopilot(True, 8000); vehicles.append(v)
ego.set_autopilot(True, 8000)
walkers = []; ctrls = []
wbps = lib.filter('walker.pedestrian.*'); cbp = lib.find('controller.ai.walker')
for _ in range(a.walkers):
    loc = w.get_random_location_from_navigation()
    if loc is None: continue
    wk = w.try_spawn_actor(random.choice(wbps), carla.Transform(loc))
    if wk is None: continue
    walkers.append(wk)
w.tick()
for wk in walkers:
    c = w.spawn_actor(cbp, carla.Transform(), attach_to=wk); ctrls.append(c)
w.tick()
for c in ctrls:
    c.start(); c.go_to_location(w.get_random_location_from_navigation()); c.set_max_speed(1.2 + random.random())
print(f'{a.town}: ego + {len(vehicles)} vehicles + {len(walkers)} walkers', flush=True)
for _ in range(a.settle): w.tick()

# sensor rig (positions in the ego frame, metres)
RIG = {   # name: (blueprint, location, rotation, attributes)
    'rgb':      ('sensor.camera.rgb',                   (1.5, 0.0, 2.0), (0, 0, 0),   {'image_size_x': '1280', 'image_size_y': '720', 'fov': '90'}),
    'depth':    ('sensor.camera.depth',                 (1.5, 0.0, 2.0), (0, 0, 0),   {'image_size_x': '1280', 'image_size_y': '720', 'fov': '90'}),
    'semseg':   ('sensor.camera.semantic_segmentation', (1.5, 0.0, 2.0), (0, 0, 0),   {'image_size_x': '1280', 'image_size_y': '720', 'fov': '90'}),
    'instance': ('sensor.camera.instance_segmentation', (1.5, 0.0, 2.0), (0, 0, 0),   {'image_size_x': '1280', 'image_size_y': '720', 'fov': '90'}),
    'lidar':    ('sensor.lidar.ray_cast',               (0.0, 0.0, 2.5), (0, 0, 0),   {'channels': '64', 'range': '80', 'points_per_second': '1300000', 'rotation_frequency': '20', 'upper_fov': '10', 'lower_fov': '-25'}),
    'radar':    ('sensor.other.radar',                  (2.3, 0.0, 0.8), (0, 0, 0),   {'horizontal_fov': '60', 'vertical_fov': '20', 'range': '100', 'points_per_second': '3000'}),
    'bev':      ('sensor.camera.rgb',                   (0.0, 0.0, 55.0), (-90, 0, 0), {'image_size_x': '1000', 'image_size_y': '1000', 'fov': '70'}),
    'gnss':     ('sensor.other.gnss',                   (0.0, 0.0, 2.0), (0, 0, 0),   {}),
    'imu':      ('sensor.other.imu',                    (0.0, 0.0, 1.0), (0, 0, 0),   {}),
}
sensors = {}; data = {}
for name, (bpid, loc, rot, attrs) in RIG.items():
    bp = lib.find(bpid)
    for k, v in attrs.items(): bp.set_attribute(k, v)
    if bp.has_attribute('sensor_tick'): bp.set_attribute('sensor_tick', '0.0')
    s = w.spawn_actor(bp, carla.Transform(carla.Location(*loc), carla.Rotation(pitch=rot[0], yaw=rot[1], roll=rot[2])), attach_to=ego)
    s.listen(lambda d, n=name: data.__setitem__(n, d)); sensors[name] = s
for _ in range(8): w.tick()
snap = w.get_snapshot(); frame = snap.frame
for name in RIG: assert name in data, f'no data from {name}'
print('capture frame', frame, 'sim time %.2f s' % snap.timestamp.elapsed_seconds, flush=True)

# ------------------------------------------------------------------ render each modality
out = {}
out['rgb'] = to_rgb(data['rgb']); out['depth'] = to_rgb(data['depth'], carla.ColorConverter.LogarithmicDepth)
out['semseg'] = to_rgb(data['semseg'], carla.ColorConverter.CityScapesPalette); out['instance'] = to_rgb(data['instance'])
out['bev'] = to_rgb(data['bev'])
for k in ('rgb', 'depth', 'semseg', 'instance', 'bev'): cv2.imwrite(f'{a.out}/sensor_{k}.png', out[k][:, :, ::-1])
# lidar: (x, y, z, intensity) in the sensor frame, CARLA y right -> plot with -y so the image is a proper top view (up = forward)
pc = np.frombuffer(data['lidar'].raw_data, dtype=np.float32).reshape(-1, 4)
fig, ax = plt.subplots(figsize=(7.2, 7.2)); fig.patch.set_facecolor('black'); ax.set_facecolor('black')
sc = ax.scatter(-pc[:, 1], pc[:, 0], c=pc[:, 2], s=0.25, cmap='turbo', vmin=-2.5, vmax=4)
ax.set_xlim(-60, 60); ax.set_ylim(-60, 60); ax.set_aspect('equal'); ax.axis('off')
ax.text(0.02, 0.97, f'LiDAR  64 ch  {len(pc):,} pts / sweep  (colour = height)', color='w', transform=ax.transAxes, va='top', fontsize=11)
fig.savefig(f'{a.out}/sensor_lidar.png', dpi=150, bbox_inches='tight', facecolor='black'); plt.close(fig); out['lidar'] = cv2.imread(f'{a.out}/sensor_lidar.png')[:, :, ::-1]
# radar: detections (velocity, azimuth, altitude, depth)
rd = np.frombuffer(data['radar'].raw_data, dtype=np.float32).reshape(-1, 4)
fig, ax = plt.subplots(figsize=(7.2, 5.0)); fig.patch.set_facecolor('black'); ax.set_facecolor('black')
x = rd[:, 3] * np.cos(rd[:, 1]); y = rd[:, 3] * np.sin(rd[:, 1])
sc = ax.scatter(y, x, c=rd[:, 0], cmap='coolwarm', vmin=-12, vmax=12, s=14)
ax.set_xlim(-45, 45); ax.set_ylim(0, 100); ax.set_aspect('equal'); ax.tick_params(colors='w'); [sp.set_color('w') for sp in ax.spines.values()]
ax.set_xlabel('lateral [m]', color='w'); ax.set_ylabel('range ahead [m]', color='w'); cb = fig.colorbar(sc, ax=ax); cb.set_label('radial velocity [m/s]', color='w'); cb.ax.yaxis.set_tick_params(color='w'); plt.setp(cb.ax.get_yticklabels(), color='w')
ax.set_title(f'Radar  {len(rd)} detections  (60 deg x 20 deg, 100 m)', color='w')
fig.savefig(f'{a.out}/sensor_radar.png', dpi=150, bbox_inches='tight', facecolor='black'); plt.close(fig); out['radar'] = cv2.imread(f'{a.out}/sensor_radar.png')[:, :, ::-1]
# gnss / imu / vehicle state readout panel
g = data['gnss']; imu = data['imu']; v = ego.get_velocity(); tf = ego.get_transform(); ctl = ego.get_control()
lines = [f'frame {frame}   t = {snap.timestamp.elapsed_seconds:7.2f} s   dt = {DT} s (20 Hz)', '',
         f'GNSS   lat {g.latitude:+.6f} deg   lon {g.longitude:+.6f} deg   alt {g.altitude:6.2f} m',
         f'IMU    acc  ({imu.accelerometer.x:+6.2f}, {imu.accelerometer.y:+6.2f}, {imu.accelerometer.z:+6.2f}) m/s^2',
         f'       gyro ({imu.gyroscope.x:+6.3f}, {imu.gyroscope.y:+6.3f}, {imu.gyroscope.z:+6.3f}) rad/s   compass {math.degrees(imu.compass):6.1f} deg', '',
         f'pose   x {tf.location.x:8.2f}  y {tf.location.y:8.2f}  z {tf.location.z:5.2f} m   yaw {tf.rotation.yaw:7.2f} deg',
         f'speed  {3.6 * math.hypot(v.x, v.y):5.1f} km/h   control: throttle {ctl.throttle:.2f}  steer {ctl.steer:+.2f}  brake {ctl.brake:.2f}', '',
         f'actors in world: {len(w.get_actors().filter("*vehicle*"))} vehicles, {len(w.get_actors().filter("*walker.pedestrian*"))} pedestrians, {len(w.get_actors().filter("*traffic_light*"))} traffic lights']
fig, ax = plt.subplots(figsize=(7.2, 3.4)); fig.patch.set_facecolor('#111'); ax.axis('off')
ax.text(0.02, 0.95, '\n'.join(lines), family='monospace', fontsize=9.5, color='#eee', va='top', transform=ax.transAxes)
ax.set_title('GNSS + IMU + vehicle state (one tick)', color='w', fontsize=11)
fig.savefig(f'{a.out}/sensor_state.png', dpi=150, bbox_inches='tight', facecolor='#111'); plt.close(fig); out['state'] = cv2.imread(f'{a.out}/sensor_state.png')[:, :, ::-1]

panels = [('rgb', 'RGB camera 1280x720, FOV 90'), ('depth', 'Depth camera (log scale)'), ('semseg', 'Semantic segmentation (CityScapes palette)'),
          ('instance', 'Instance segmentation'), ('lidar', 'LiDAR point cloud (top view)'), ('radar', 'Radar detections'), ('bev', "Bird's-eye camera, 55 m above the ego"), ('state', 'GNSS / IMU / state')]
fig, axs = plt.subplots(2, 4, figsize=(22, 10))
for ax, (k, title) in zip(axs.ravel(), panels): ax.imshow(out[k]); ax.set_title(title, fontsize=13); ax.axis('off')
fig.suptitle(f'One instant of the twin, every modality from the same ego ({a.town}, frame {frame})', fontsize=15)
fig.tight_layout(); fig.savefig(f'{a.out}/sensors.png', dpi=130); plt.close(fig)

# ------------------------------------------------------------------ BEV: lane graph + agents + ego (the "state" the twin keeps)
el = ego.get_location(); R = 70.0
wps = [p for p in m.generate_waypoints(2.0) if p.transform.location.distance(el) < R * 1.5]
fig, axs = plt.subplots(1, 2, figsize=(14, 7))
axs[0].imshow(out['bev']); axs[0].set_title('Rendered bird\'s-eye camera (what a drone would see)', fontsize=12); axs[0].axis('off')
ax = axs[1]
for p in wps:
    loc = p.transform.location; yaw = math.radians(p.transform.rotation.yaw); hw = p.lane_width / 2
    nx, ny = -math.sin(yaw) * hw, math.cos(yaw) * hw
    ax.plot([loc.x - nx, loc.x + nx], [loc.y - ny, loc.y + ny], color='#bbb', lw=0.5, alpha=0.5)
    ax.plot(loc.x, loc.y, '.', color='#444', ms=1.5)
def box(ax, actor, color, lw=1.2):
    t = actor.get_transform(); e = actor.bounding_box.extent; yaw = math.radians(t.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw); pts = np.array([[e.x, e.y], [e.x, -e.y], [-e.x, -e.y], [-e.x, e.y], [e.x, e.y]])
    xy = pts @ np.array([[c, s], [-s, c]]) + [t.location.x, t.location.y]; ax.plot(xy[:, 0], xy[:, 1], color=color, lw=lw)
    v = actor.get_velocity(); ax.arrow(t.location.x, t.location.y, v.x * 1.0, v.y * 1.0, color=color, head_width=0.8, lw=0.8)
for vh in vehicles:
    if vh.get_location().distance(el) < R: box(ax, vh, 'tab:blue')
for wk in walkers:
    if wk.get_location().distance(el) < R: l = wk.get_location(); ax.plot(l.x, l.y, 'o', color='tab:orange', ms=4)
for tl in w.get_actors().filter('*traffic_light*'):
    if tl.get_location().distance(el) < R:
        l = tl.get_location(); st = str(tl.get_state()); ax.plot(l.x, l.y, 's', color={'Red': 'r', 'Yellow': 'y', 'Green': 'g'}.get(st, 'k'), ms=6)
box(ax, ego, 'red', lw=2.0)
ax.set_xlim(el.x - R, el.x + R); ax.set_ylim(el.y + R, el.y - R); ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m] (CARLA, y down)')
ax.set_title('Twin state at the same tick (ground truth)', fontsize=12)
ax.text(0.5, -0.14, 'lane graph (OpenDRIVE)   vehicles (blue)   pedestrians (orange)   signals (red/yellow/green squares)   ego (red)   arrows = velocity x 1 s', transform=ax.transAxes, ha='center', fontsize=9)
fig.tight_layout(); fig.savefig(f'{a.out}/bev.png', dpi=140, bbox_inches='tight'); plt.close(fig)

# ------------------------------------------------------------------ weather: same view, four conditions
presets = [('ClearNoon', carla.WeatherParameters.ClearNoon), ('CloudySunset', carla.WeatherParameters.CloudySunset),
           ('HardRainNoon', carla.WeatherParameters.HardRainNoon), ('Night', carla.WeatherParameters(sun_altitude_angle=-30.0, cloudiness=10.0))]
wimgs = []
for name, wp in presets:
    w.set_weather(wp)
    for _ in range(12): w.tick()
    wimgs.append((name, to_rgb(data['rgb']))); cv2.imwrite(f'{a.out}/weather_{name}.png', wimgs[-1][1][:, :, ::-1])
fig, axs = plt.subplots(1, 4, figsize=(22, 3.6))
for ax, (n, img) in zip(axs, wimgs): ax.imshow(img); ax.set_title(n, fontsize=12); ax.axis('off')
fig.tight_layout(); fig.savefig(f'{a.out}/weather.png', dpi=130); plt.close(fig)

for s in sensors.values():
    try: s.stop(); s.destroy()
    except Exception: pass
for c in ctrls:
    try: c.stop(); c.destroy()
    except Exception: pass
s = w.get_settings(); s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
print('done ->', a.out)
