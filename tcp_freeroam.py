"""Let the TCP baseline drive in an open CARLA world (no Bench2Drive route/scoring).
Spawns a hero, plans a route to a random destination with the leaderboard's route tools,
attaches TCP's sensors, runs the agent each tick, re-plans on arrival. Spectator follows the car.

Server must be running (bash carla_server.sh). Run with the b2d env.
  python tcp_freeroam.py [--map Town10HD] [--max-speed 4] [--dist 800] [--fps 20]
Ctrl+C to stop.  Spawn NPCs separately with traffic.sh after this starts (it is the hero).
Note: the Bench2Drive TCP agent clamps itself to ~5 km/h (1.5 m/s); --max-speed raises that.
Runs ~0.3x real time (5 cameras at 1600x900 + net); if it stalls against something, Ctrl+C and restart.
"""
import argparse, os, sys, time, random, math
import numpy as np
os.environ.pop('SAVE_PATH', None)  # no frame dumping
os.environ['IS_BENCH2DRIVE'] = 'True'   # agent's non-B2D setup branch is broken upstream; B2D branch adds a bev cam we simply also attach
sys.path[:0] = ['/home/all/carla_ws/Bench2Drive/leaderboard', '/home/all/carla_ws/Bench2Drive/leaderboard/team_code',
                '/home/all/carla_ws/Bench2Drive/scenario_runner', '/home/all/carla_ws/Bench2Drive/Bench2DriveZoo',
                '/home/all/carla_ws/carla/PythonAPI/carla']
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from leaderboard.utils.route_manipulation import interpolate_trajectory

ap = argparse.ArgumentParser()
ap.add_argument('--map', default=None); ap.add_argument('--dist', type=float, default=800, help='min straight-line distance to destination (m)')
ap.add_argument('--fps', type=int, default=20); ap.add_argument('--ckpt', default='/home/all/carla_ws/data/ckpts/tcp_b2d.ckpt')
ap.add_argument('--planner', default='only_traj', choices=['only_traj', 'only_ctrl', 'merge_ctrl_traj'])
ap.add_argument('--max-speed', type=float, default=1.5, help='m/s; Bench2Drive clamps throttle above 1.5 m/s (~5 km/h). Try 3-4. Uncapped (99) crashes.')
ap.add_argument('--filter', default='vehicle.lincoln.mkz_2017'); ap.add_argument('--port', type=int, default=2000)
args = ap.parse_args()
os.environ['PLANNER_TYPE'] = args.planner

client = carla.Client('localhost', args.port); client.set_timeout(120.0)
if args.map and client.get_world().get_map().name.split('/')[-1] != args.map:
    print('loading', args.map); client.load_world(args.map)
world = client.get_world()
settings = world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = 1.0 / args.fps; world.apply_settings(settings)
tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True)
CarlaDataProvider.set_client(client); CarlaDataProvider.set_world(world)
cmap = world.get_map(); spawn_pts = cmap.get_spawn_points()

bp = world.get_blueprint_library().find(args.filter); bp.set_attribute('role_name', 'hero')
hero = None
for sp in random.sample(spawn_pts, len(spawn_pts)):
    hero = world.try_spawn_actor(bp, sp)
    if hero: break
CarlaDataProvider.register_actor(hero)
world.tick()

from tcp_b2d_agent import TCPAgent
import torch
torch.cuda.empty_cache = lambda: None  # agent calls it every step; re-allocating everything per step is very slow next to CARLA's VRAM use
os.environ['IS_BENCH2DRIVE'] = 'True'
agent = TCPAgent('localhost', args.port); agent.setup(args.ckpt + '+freeroam')
if args.max_speed > 1.5:
    # tcp_b2d_agent.run_step clamps throttle to 0.05 once speed > 1.5 m/s. Re-apply the same rule with our
    # own limit, using the net's own PID throttle (pid_metadata['throttle_traj']) below the limit.
    _orig_run_step = agent.run_step
    def _relaxed(inp, ts):
        c = _orig_run_step(inp, ts)
        if c.brake < 0.5 and inp['SPEED'][1]['speed'] < args.max_speed:
            c.throttle = float(np.clip(getattr(agent, 'pid_metadata', {}).get('throttle_traj', c.throttle), 0.0, 0.75))
        return c
    agent.run_step = _relaxed

# ---- sensors (from the agent's own spec) -------------------------------------------------------
latest = {}
def make_cb(sid, kind):
    def cb(data):
        if kind == 'sensor.camera.rgb':
            arr = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(data.height, data.width, 4)
            latest[sid] = (data.frame, arr)
        elif kind == 'sensor.other.gnss':
            latest[sid] = (data.frame, np.array([data.latitude, data.longitude, data.altitude], dtype=np.float64))
        elif kind == 'sensor.other.imu':
            latest[sid] = (data.frame, np.array([data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
                                                 data.gyroscope.x, data.gyroscope.y, data.gyroscope.z, data.compass], dtype=np.float64))
    return cb

sensors = []
lib = world.get_blueprint_library()
for s in agent.sensors():
    if s['type'] == 'sensor.speedometer':
        continue
    b = lib.find(s['type'])
    if s['type'] == 'sensor.camera.rgb':
        b.set_attribute('image_size_x', str(s['width'])); b.set_attribute('image_size_y', str(s['height'])); b.set_attribute('fov', str(s['fov']))
    elif s['type'] == 'sensor.other.gnss':
        for k in ('noise_alt_stddev', 'noise_lat_stddev', 'noise_lon_stddev'): b.set_attribute(k, '0.0')
    elif s['type'] == 'sensor.other.imu':
        for k in ('noise_accel_stddev_x', 'noise_accel_stddev_y', 'noise_accel_stddev_z', 'noise_gyro_stddev_x', 'noise_gyro_stddev_y', 'noise_gyro_stddev_z'): b.set_attribute(k, '0.0')
    tf = carla.Transform(carla.Location(x=s.get('x', 0), y=s.get('y', 0), z=s.get('z', 0)),
                         carla.Rotation(pitch=s.get('pitch', 0), roll=s.get('roll', 0), yaw=s.get('yaw', 0)))
    a = world.spawn_actor(b, tf, attach_to=hero); a.listen(make_cb(s['id'], s['type'])); sensors.append(a)
def forward_speed():  # same formula as leaderboard's SpeedometerReader (which we avoid: its thread busy-spins and starves CUDA)
    v = hero.get_velocity(); tf = hero.get_transform()
    pitch, yaw = math.radians(tf.rotation.pitch), math.radians(tf.rotation.yaw)
    return float(np.dot([v.x, v.y, v.z], [math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)]))

# ---- route ---------------------------------------------------------------------------------------
def new_route():
    start = hero.get_transform().location
    far = [p for p in spawn_pts if p.location.distance(start) > args.dist] or spawn_pts
    dest = random.choice(far).location
    gps_route, route = interpolate_trajectory([start, dest])
    agent._global_plan = None; agent.initialized = False
    agent.set_global_plan(gps_route, route)
    for i in range(0, len(route), 5):
        world.debug.draw_point(route[i][0].location + carla.Location(z=0.3), size=0.08, color=carla.Color(0, 255, 0), life_time=600)
    world.debug.draw_point(dest + carla.Location(z=1.0), size=0.5, color=carla.Color(255, 0, 0), life_time=600)
    print(f'new route: {len(route)} m -> ({dest.x:.0f},{dest.y:.0f})')
    return dest
dest = new_route()

spectator = world.get_spectator()
t0 = time.time(); step = 0
print('TCP driving. Ctrl+C to stop. Spawn NPCs with: bash traffic.sh 60 40')
try:
    while True:
        world.tick(); step += 1
        frame = world.get_snapshot().frame
        # wait until every sensor has delivered THIS frame (data streams in after tick() returns)
        need = ('CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'GPS', 'IMU', 'bev')
        t_wait = time.time()
        while not all(k in latest and latest[k][0] >= frame for k in need):
            if time.time() - t_wait > 5: break
            time.sleep(0.001)
        inp = dict(latest); inp['SPEED'] = (frame, {'speed': forward_speed()})
        control = agent.run_step(inp, step / args.fps)
        hero.apply_control(control)
        tf = hero.get_transform()
        spectator.set_transform(carla.Transform(tf.location + carla.Location(z=6) - tf.get_forward_vector() * 8, carla.Rotation(pitch=-20, yaw=tf.rotation.yaw)))
        if step % (args.fps * 2) == 0:
            v = hero.get_velocity(); print(f't={step/args.fps:6.1f}s  {3.6*math.hypot(v.x, v.y):5.1f} km/h  thr={control.throttle:.2f} brk={control.brake:.2f} str={control.steer:+.2f}  to goal {tf.location.distance(dest):6.1f} m  ({(time.time()-t0)/(step/args.fps):.2f}x wall/sim)')
        if tf.location.distance(dest) < 8:
            print('arrived'); dest = new_route()
except KeyboardInterrupt:
    pass
finally:
    print('cleaning up')
    for a in sensors: a.stop(); a.destroy()
    hero.destroy()
    settings.synchronous_mode = False; settings.fixed_delta_seconds = None; world.apply_settings(settings); tm.set_synchronous_mode(False)
