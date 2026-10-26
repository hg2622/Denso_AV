"""BridgeDrive (LEAD/TFv6 + diffusion-bridge planner) drives in an open CARLA world: random GPS route to a random
destination, re-planned on arrival, no Bench2Drive scenarios/scoring.

Default = REAL TIME: the world (traffic, pedestrians, physics) runs at full speed; the net reads the latest sensor data, thinks
(~4-5 steps/s on this laptop) in a background thread; LEAD's own PID controller then tracks the net's latest predicted path at
the nominal 20 Hz (so steering/speed control behaves as in the benchmark even though the planner is slower). --sync = lock-step mode
(world waits for the net every 50 ms step; slow motion but every frame processed), which is how the benchmark evaluates.
--sim-rate N = fixed slow clock: one 50 ms sim step every 1/N wall seconds on a timer (never waits for the net); N=3 matches
this GPU so the net sees 50 ms-spaced frames as trained, but everything moves at N/20 real speed (slow motion).

Opens a viewer window (chase cam + top-down) with overlays drawn CLIENT-SIDE (nothing is drawn into the world, so the agent cameras stay clean):
  GREEN spheres + label  : the sparse route points the agent actually receives (one per ~50 m / every command change)
  RED sphere             : the current target point the net is conditioned on (next one ORANGE)
  BLUE dots              : the 10 route checkpoints predicted by the diffusion bridge this frame (what it steers along)
  thin green line        : the dense lane-centre route the leaderboard would use for scoring (NOT given to the agent)

Server must be running (bash carla_server.sh [-windowed]). Run with the `lead` env from the LEAD dir (bd_drive.sh does that).
  python bd_freeroam.py [--map Town04] [--dist 600] [--filter vehicle.lincoln.mkz_2017] [--no-dense]
Ctrl+C to stop.  Traffic: bash traffic.sh / traffic_chaos.sh after it starts (the car is the hero).
"""
import argparse, os, sys, time, random, math
import numpy as np
LEAD = '/home/all/carla_ws/LEAD'; os.chdir(LEAD)
os.environ.setdefault('NUMBA_NUM_THREADS', '6')   # LiDAR ground removal is numba-parallel; leave cores for the CARLA server + 20 Hz loop
os.environ.pop('SAVE_PATH', None)                        # no frame/video dumping
os.environ.pop('IS_BENCH2DRIVE', None)   # B2D mode writes metric_info.json every step into SAVE_PATH; not wanted here
_sync = ('--sync' in sys.argv) or ('--sim-rate' in sys.argv and float(sys.argv[sys.argv.index('--sim-rate') + 1]) > 0)   # fixed-clock modes: ~1 net step per 50 ms sim step
os.environ.setdefault('LEAD_CLOSED_LOOP_CONFIG', 'steer_modality=route throttle_modality=target_speed brake_modality=target_speed step_num=20 diffusion_speed=False use_kalman_filter=False '
                      + ('sensor_agent_stuck_threshold=300' if _sync else 'sensor_agent_stuck_threshold=60'))   # creep after ~15 s stuck (threshold is in net steps: 20/s sync, ~4/s real time)
os.environ.setdefault('LEAD_TRAINING_CONFIG', 'diffusion_speed=False')
os.environ.setdefault('CUDA_VISIBLE_DEVICES', '0')
sys.path[:0] = [f'{LEAD}/3rd_party/Bench2Drive/leaderboard', f'{LEAD}/3rd_party/Bench2Drive/scenario_runner', '/home/all/carla_ws/carla/PythonAPI/carla']
import carla
if os.environ.get('BD_TICKTRACE'):   # debug: who calls world.tick()?
    _orig_tick = carla.World.tick; _tick_calls = [0]
    def _tick_traced(self, *a, **k):
        _tick_calls[0] += 1
        if _tick_calls[0] % 200 == 0:
            import traceback; print(f'[ticktrace] world.tick() call #{_tick_calls[0]} from:', flush=True); traceback.print_stack(limit=6)
        return _orig_tick(self, *a, **k)
    carla.World.tick = _tick_traced
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from leaderboard.utils.route_manipulation import interpolate_trajectory
from leaderboard.autoagents.agent_wrapper import AgentWrapper

ap = argparse.ArgumentParser()
ap.add_argument('--map', default=None); ap.add_argument('--dist', type=float, default=600, help='min straight-line distance to destination (m)')
ap.add_argument('--ckpt', default='outputs/local_training/bridgedrive_k60'); ap.add_argument('--filter', default='vehicle.lincoln.mkz_2017')
ap.add_argument('--port', type=int, default=2000); ap.add_argument('--no-dense', action='store_true', help='do not draw the dense lane-centre route')
ap.add_argument('--fps', type=int, default=20)
ap.add_argument('--sim-rate', type=float, default=0.0, help='world ticks per WALL second, each tick = 50 ms of sim time, on a fixed timer (never waits for the net). 3 = slow-motion matching this GPU; 0 = real time')
ap.add_argument('--sync', action='store_true', help='lock-step mode: the world waits for the net each step (slow motion). Default: real-time world, the net applies controls as fast as it can')
ap.add_argument('--route', default=None, help='A/B check: start on this Bench2Drive route id (its town, start pose and waypoints) instead of a random spot')
args = ap.parse_args()
FIXED = (not args.sync) and args.sim_rate > 0      # fixed-clock slow motion (default)
SYNC_WORLD = args.sync or FIXED                    # world in synchronous (fixed 50 ms step) mode

ROUTE_XML = None
if args.route:
    import xml.etree.ElementTree as ET
    r = [x for x in ET.parse('/home/all/carla_ws/Bench2Drive/leaderboard/data/bench2drive220.xml').getroot().findall('route') if x.get('id') == args.route]
    assert r, f'route {args.route} not found'; args.map = r[0].get('town')
    ROUTE_XML = [carla.Location(x=float(p.get('x')), y=float(p.get('y')), z=float(p.get('z'))) for p in r[0].find('waypoints').findall('position')]
    print(f'A/B mode: Bench2Drive route {args.route} on {args.map}, {len(ROUTE_XML)} key points', flush=True)
client = carla.Client('localhost', args.port); client.set_timeout(10.0)
for _try in range(30):   # server still starting / loading a map -> wait instead of dying with a 120 s timeout
    try:
        world = client.get_world(); st = world.get_settings()
        if st.synchronous_mode:   # left behind by a client that died: the server waits for ticks forever -> unstick it
            print('server was stuck in synchronous mode (previous client died) -> switching to asynchronous', flush=True)
            st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
        world.get_map().name; break
    except RuntimeError:
        print(f'waiting for the CARLA server on port {args.port} ... ({_try + 1}/30)', flush=True)
else:
    sys.exit('CARLA server not reachable - start it with: bash ~/carla_ws/carla_server.sh')
client.set_timeout(120.0)
if args.map and client.get_world().get_map().name.split('/')[-1] != args.map:
    print('loading', args.map); client.load_world(args.map)
world = client.get_world()
settings = world.get_settings(); settings.synchronous_mode = SYNC_WORLD; settings.fixed_delta_seconds = (1.0 / args.fps) if SYNC_WORLD else None; world.apply_settings(settings)
tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(SYNC_WORLD)
CarlaDataProvider.set_client(client); CarlaDataProvider.set_world(world)
cmap = world.get_map(); spawn_pts = cmap.get_spawn_points()

bp = world.get_blueprint_library().find(args.filter); bp.set_attribute('role_name', 'hero')
hero = None
if ROUTE_XML:
    wp0 = cmap.get_waypoint(ROUTE_XML[0]); tf0 = wp0.transform; tf0.location.z += 0.5
    hero = world.try_spawn_actor(bp, tf0)
for sp in random.sample(spawn_pts, len(spawn_pts)):
    if hero: break
    hero = world.try_spawn_actor(bp, sp)
CarlaDataProvider.register_actor(hero); CarlaDataProvider._carla_actor_pool[hero.id] = hero; world.tick()   # pool is what get_hero_actor() searches

# ---- route -----------------------------------------------------------------------------------------
_first = [True]
def plan_route():
    if _first[0] and not ROUTE_XML:
        # spawn points can face against the lane / into dead ends: align the car with the travel direction of the lane it is on
        wp = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving)
        tf = wp.transform; tf.location.z += 0.3; hero.set_transform(tf)
        if not SYNC_WORLD: time.sleep(0.3)
        else: world.tick()
    start = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving).next(3.0)[0].transform.location   # lane point just ahead -> route leaves forward
    if ROUTE_XML and _first[0]:
        _first[0] = False; gps_route, route = interpolate_trajectory(ROUTE_XML); return ROUTE_XML[-1], gps_route, route
    far = [p for p in spawn_pts if p.location.distance(start) > args.dist] or spawn_pts
    dest = random.choice(far).location
    gps_route, route = interpolate_trajectory([start, dest])     # dense lane-centre route (1 m) + GPS version, like the leaderboard
    return dest, gps_route, route

from lead.inference.sensor_agent_bridgedrive import SensorAgent
dest, gps_route, route = plan_route()
agent = SensorAgent('localhost', args.port)
agent.set_global_plan(gps_route, route)        # downsamples to ~50 m points: this is ALL the navigation the agent gets
agent.setup(args.ckpt)
# LEAD estimates the map's GPS reference from the first route point by solving a nonlinear equation (find_gps_ref); from an
# arbitrary free-roam start that can converge to a wrong root (seen: ~110 m position offset). Use the exact reference from the
# map's OpenDRIVE header instead - the same one the GNSS sensor and the leaderboard's route GPS use.
from leaderboard.utils.route_manipulation import _get_latlon_ref
_est = (agent.noisy_lat_ref, agent.noisy_lon_ref); agent.noisy_lat_ref, agent.noisy_lon_ref = _get_latlon_ref(world)
for planner in agent.gps_waypoint_planners_dict.values():
    planner.set_route(agent._global_plan, True, lat_ref=agent.noisy_lat_ref, lon_ref=agent.noisy_lon_ref)
print(f'GPS reference: LEAD estimate {_est} -> map header {(agent.noisy_lat_ref, agent.noisy_lon_ref)}', flush=True)
if not SYNC_WORLD:
    # The GPS Kalman filter (position used for route popping + target points) is a 20 Hz filter (dt = 50 ms per net step).
    # At ~3 net steps/s it lags the car by metres, so in real-time mode use the raw GNSS fix (noise ~0.5 m) instead.
    agent.training_config.use_kalman_filter_for_gps = False; print('real-time mode: GPS Kalman filter off (20 Hz filter)', flush=True)
lib = world.get_blueprint_library()
wrapper = AgentWrapper(agent)
if args.sync:
    wrapper.setup_sensors(hero)   # leaderboard harness: every sensor must deliver the same frame before the net runs
else:
    # Real-time harness: sensors stream freely, the net always takes the LATEST reading of each (frames may differ by a few
    # ms) and applies its control when it is done, like a real car whose planner is slower than its sensors.
    latest = {}; lidar_buf = {}
    from collections import deque
    import threading
    def _mk(sid, kind):
        def cb(d):
            if kind == 'sensor.camera.rgb':
                latest[sid] = (d.frame, np.frombuffer(d.raw_data, dtype=np.uint8).reshape((d.height, d.width, 4)).copy())
            elif kind == 'sensor.lidar.ray_cast':   # each callback is half a rotation: keep the last 2 so every net step sees a full sweep
                chunk = np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                if FIXED: latest[sid] = (d.frame, chunk); return          # one chunk per 50 ms tick, exactly as in the benchmark
                lidar_buf.setdefault(sid, deque(maxlen=2)).append(chunk)
                full = np.concatenate(list(lidar_buf[sid]), axis=0); budget = len(lidar_buf[sid][-1])
                if len(full) > budget: full = full[np.random.choice(len(full), budget, replace=False)]   # one chunk's point budget, full 360 deg
                latest[sid] = (d.frame, full)
            elif kind == 'sensor.other.radar':   # leaderboard order: [depth, azimuth, altitude, velocity] -> flipped
                latest[sid] = (d.frame, np.flip(np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4), 1).copy())
            elif kind == 'sensor.other.gnss':
                latest[sid] = (d.frame, np.array([d.latitude, d.longitude, d.altitude], dtype=np.float64))
            elif kind == 'sensor.other.imu':
                latest[sid] = (d.frame, np.array([d.accelerometer.x, d.accelerometer.y, d.accelerometer.z, d.gyroscope.x, d.gyroscope.y, d.gyroscope.z, d.compass], dtype=np.float64))
        return cb
    agent_sensors = []; speed_id = None
    for spec in agent.sensors():
        if spec['type'] == 'sensor.speedometer': speed_id = spec['id']; continue   # pseudo-sensor: computed from the hero velocity below
        type_, id_, tf, attrs = wrapper._preprocess_sensor_spec(spec)      # same attributes/noise as the leaderboard
        b = lib.find(type_)
        for k, v in attrs.items(): b.set_attribute(str(k), str(v))
        a = world.spawn_actor(b, tf, attach_to=hero); a.listen(_mk(id_, type_)); agent_sensors.append(a)
    need = [sp['id'] for sp in agent.sensors() if sp['type'] != 'sensor.speedometer']
    def forward_speed():
        v = hero.get_velocity(); tf = hero.get_transform(); p, y = math.radians(tf.rotation.pitch), math.radians(tf.rotation.yaw)
        return np.float64(v.x * math.cos(p) * math.cos(y) + v.y * math.cos(p) * math.sin(y) + v.z * math.sin(p))   # numpy scalar: the agent calls .item() on it
    # The net runs in its own thread at whatever rate the GPU allows (~4-6 Hz here). Each time it finishes, we keep its
    # predicted 10-checkpoint path (converted to WORLD coordinates), its target speed and its post-processed control.
    # The MAIN loop then runs LEAD's own PID controller (execute_route_and_target_speed) at the nominal 20 Hz on that path,
    # re-expressed in the current ego frame -> the controller sees the 50 ms steps it was tuned for.
    ctrl_lock = threading.Lock(); plan = {}; net_err = []; net_stats = {'steps': 0, 't0': time.time()}
    _exec = agent.closed_loop_inference.execute_route_and_target_speed
    def _exec_locked(*a, **k):
        with ctrl_lock: return _exec(*a, **k)
    agent.closed_loop_inference.execute_route_and_target_speed = _exec_locked
    import torch
    def net_loop():
        try:
            t_wait = time.time()
            while not all(k in latest for k in need):
                if time.time() - t_wait > 10: raise RuntimeError('sensors never delivered: ' + str([k for k in need if k not in latest]))
                time.sleep(0.002)
            last_frame = -1
            while not stop_flag[0]:
                snap = world.get_snapshot()
                if FIXED:   # one net step per NEW sim frame (never re-process the same frame; wait for its sensor data)
                    if snap.frame == last_frame: time.sleep(0.003); continue
                    t_w = time.time()
                    while not all(latest[k][0] >= snap.frame for k in need) and time.time() - t_w < 0.3: time.sleep(0.002)
                    last_frame = snap.frame
                inp = dict(latest); inp[speed_id] = (snap.frame, {'speed': forward_speed()})
                tf = hero.get_transform()
                _t = time.time(); control = agent.run_step(inp, snap.timestamp.elapsed_seconds); net_stats['steps'] += 1; net_stats['last_ms'] = (time.time() - _t) * 1000
                if 'route' in last_pred and agent.compass is not None and last_pred.get('tgt') is not None:
                    pos = np.array([tf.location.x, tf.location.y]); yaw_off = agent.compass - math.radians(tf.rotation.yaw)
                    # NOTE: the released diffusion sampler forces y=0 on the LAST row every step (x0_mean[...,-1,-1]=0, meant for an
                    # optional speed-holder row that this config does not have) -> the 10th checkpoint is bogus; use the first 9.
                    path_w = np.array([pos] + [common_utils.conversion_2d(np.asarray(pt, dtype=np.float64), pos, agent.compass) for pt in last_pred['route'][:-1]])   # ego pose + 9 checkpoints
                    plan.update(path_w=path_w, tgt=float(last_pred['tgt']), yaw_off=yaw_off, net_control=control, t=time.time())
                    if os.environ.get('BD_DEBUG') and net_stats['steps'] % 3 == 0:
                        cp = last_pred.get('clp'); r0 = last_pred['route'][0]; r9 = last_pred['route'][-1]
                        tfn = hero.get_transform(); posn = np.array([tfn.location.x, tfn.location.y]); yawn = math.radians(tfn.rotation.yaw) + yaw_off
                        pe = np.array([common_utils.inverse_conversion_2d(pw, posn, yawn) for pw in path_w]); re = resample_ahead(pe)
                        print(f"[cmp] net raw: route_steer={getattr(cp, 'route_steer', float('nan')):+.2f} final steer={control.steer:+.2f} thr={control.throttle:.2f} brk={control.brake:.2f} "
                              f"p0={np.round(r0, 1)} p9={np.round(r9, 1)} | tracker now: p0={np.round(re[0], 1)} p9={np.round(re[-1], 1)} force_move={agent.force_move_post_processor.force_move} stuck={agent.force_move_post_processor.stuck_detector} speed_in={float(forward_speed()):.1f}", flush=True)
                else:
                    plan.update(net_control=control, t=time.time())
        except Exception as e:
            import traceback; net_err.append(traceback.format_exc())
    # profiling of one net step: preprocessing (tick) vs network forward vs rest (BD_DEBUG=1 only: adds CUDA syncs)
    prof = {'tick': [], 'fwd': []}
    PROFILE = bool(os.environ.get('BD_DEBUG'))
    _tick0 = agent.tick
    def _tick_timed(*a, **k):
        _t = time.time(); r = _tick0(*a, **k); prof['tick'].append(time.time() - _t); return r
    if PROFILE: agent.tick = _tick_timed
    _fwd0 = agent.closed_loop_inference.forward
    def _fwd_timed(*a, **k):
        _t = time.time(); r = _fwd0(*a, **k); torch.cuda.synchronize(); prof['fwd'].append(time.time() - _t); return r
    if PROFILE: agent.closed_loop_inference.forward = _fwd_timed
    try:   # sub-timers: backbone (perception) vs planning decoder (diffusion bridge)
        _net0 = agent.closed_loop_inference.nets[0]
        for name in (('backbone', 'planning_decoder') if PROFILE else ()):
            mod = getattr(_net0, name); _f = mod.forward; prof[name] = []
            def _mk_t(f, key):
                def g(*a, **k):
                    _t = time.time(); r = f(*a, **k); torch.cuda.synchronize(); prof[key].append(time.time() - _t); return r
                return g
            mod.forward = _mk_t(_f, name)
    except Exception as e: print('sub-timers off:', e)
    stop_flag = [False]; net_thread = threading.Thread(target=net_loop, daemon=True); net_thread.start()

    def resample_ahead(pe, n=10, first=2.5, spacing=1.0):
        """The net's path was predicted up to ~0.4 s ago; the car has moved along it since. Re-sample it the way the net emits it:
        n points, `spacing` apart, starting `first` m ahead of the car's projection onto the (forward-extended) path."""
        # NO extrapolation beyond the net's last point: at speed the extension of a curving path end turned into a hard
        # turn the net never asked for. Past the end we fall back to straight-ahead until the next prediction (<= 0.4 s).
        seg = np.diff(pe, axis=0); L = np.linalg.norm(seg, axis=1); cum = np.r_[0.0, np.cumsum(L)]
        # projection of the origin (= ego now) onto the polyline
        best_s, best_d = 0.0, 1e9
        for i in range(len(seg)):
            if L[i] < 1e-6: continue
            t = float(np.clip(-pe[i] @ seg[i] / (L[i] ** 2), 0.0, 1.0)); q = pe[i] + t * seg[i]; dist = float(np.linalg.norm(q))
            if dist < best_d: best_d, best_s = dist, cum[i] + t * L[i]
        ss = best_s + first + spacing * np.arange(n)
        ss = ss[ss <= cum[-1] + 1e-6]                                     # only points that exist on the predicted path
        if len(ss) < 2: return np.c_[first + spacing * np.arange(n), np.zeros(n)].astype(np.float32)   # path used up: straight ahead
        out = np.c_[np.interp(ss, cum, pe[:, 0]), np.interp(ss, cum, pe[:, 1])]
        if len(out) < n: out = np.vstack([out, np.repeat(out[-1:], n - len(out), axis=0)])   # LEAD's controller indexes up to n-1 (+2): keep n rows
        # Re-centre on the car: LEAD's route controller is fed a path that always starts at the ego (re-predicted every 50 ms
        # in the benchmark), so it is a feed-forward curvature follower, not a lateral-error regulator. Keep the path's shape
        # and heading but remove the lateral offset; the net corrects the drift at its next prediction, as in the benchmark.
        q = np.array([np.interp(best_s, cum, pe[:, 0]), np.interp(best_s, cum, pe[:, 1])])
        return (out - q).astype(np.float32)

    def tracker_step():
        """20 Hz: LEAD's PID on the net's last path. Returns a carla.VehicleControl."""
        if net_err: raise RuntimeError('net thread died:\n' + net_err[0])
        if 'path_w' not in plan: return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
        tf = hero.get_transform(); pos = np.array([tf.location.x, tf.location.y]); yaw = math.radians(tf.rotation.yaw) + plan['yaw_off']
        pe = np.array([common_utils.inverse_conversion_2d(pw, pos, yaw) for pw in plan['path_w']])
        path_e = resample_ahead(pe)
        spd = float(forward_speed())
        steer, throttle, brake = _exec_locked(torch.tensor(path_e)[None], torch.tensor([[plan['tgt']]], dtype=torch.float32), torch.tensor([[spd]], dtype=torch.float32))
        nc = plan.get('net_control')
        if nc is not None:   # keep the net's post-processed decisions: stop-sign/collision/stop braking, and force-move creep when stuck
            if nc.brake > 0.5: throttle, brake = 0.0, 1.0
            if agent.force_move_post_processor.force_move > 0: throttle, brake = nc.throttle, nc.brake
        plan['trk'] = (float(steer), path_e[0].tolist(), path_e[-1].tolist())
        return carla.VehicleControl(throttle=float(throttle), brake=float(brake), steer=float(steer))

def apply_new_route(keep_dest=False):
    global dest, gps_route, route
    if keep_dest:
        st = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving).next(3.0)[0].transform.location
        gps_route, route = interpolate_trajectory([st, dest]); print('rerouting to the same goal from here', flush=True)
    else:
        dest, gps_route, route = plan_route()
    agent.set_global_plan(gps_route, route)
    for planner in agent.gps_waypoint_planners_dict.values():
        planner.set_route(agent._global_plan, True, lat_ref=agent.noisy_lat_ref, lon_ref=agent.noisy_lon_ref); planner.is_last = False
    print(f'new route: {len(route)} m to ({dest.x:.0f},{dest.y:.0f}); agent receives {len(agent._global_plan)} GPS points', flush=True)
print(f'route: {len(route)} m to ({dest.x:.0f},{dest.y:.0f}); agent receives {len(agent._global_plan)} GPS points', flush=True)

# capture the net's predicted route each frame for drawing
last_pred = {}
_fwd = agent.closed_loop_inference.forward
_ncall = [0]
def _capture(data):
    p = _fwd(data); _ncall[0] += 1; last_pred['clp'] = p
    try:
        last_pred['route'] = p.pred_route.detach().cpu().numpy().reshape(-1, 2)[:10].copy()
        last_pred['tgt'] = float(p.pred_target_speed_scalar.detach().cpu().reshape(-1)[0]) if p.pred_target_speed_scalar is not None else None
    except Exception as e: last_pred['err'] = repr(e)
    if os.environ.get('BD_DEBUG') and _ncall[0] % 20 == 0:
        ts = float(p.pred_target_speed_scalar.detach().cpu().reshape(-1)[0]) if p.pred_target_speed_scalar is not None else None
        nbb = len(p.pred_bounding_box_vehicle_system) if p.pred_bounding_box_vehicle_system is not None else None
        tp = data['target_point'].detach().cpu().numpy().reshape(-1); cmd = int(data['command'].detach().cpu().reshape(-1).argmax())
        print(f"[dbg] tgt_speed={ts} route0={last_pred.get('route', [[None]])[0]} route9={last_pred.get('route', [[None]]*10)[-1]} steer={p.steer:+.2f} thr={p.throttle:.2f} brk={p.brake:.2f} "
              f"route_steer={p.route_steer:+.2f} ts_thr={p.target_speed_throttle:.2f} ts_brk={p.target_speed_brake:.2f} bboxes={nbb} target_point={tp} cmd={cmd} "
              f"stuck={agent.force_move_post_processor.stuck_detector} force={agent.force_move_post_processor.force_move} speed={float(data['speed'].detach().cpu().reshape(-1)[0]):.2f} "
              f"light={hero.is_at_traffic_light()}/{hero.get_traffic_light_state()} junction={cmap.get_waypoint(hero.get_location()).is_junction} speedlimit={hero.get_speed_limit():.0f}", flush=True)
        htf = hero.get_transform(); pr = agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance].route
        ns = getattr(agent, 'filtered_state', None) if agent.config_closed_loop.use_kalman_filter else None
        print(f"[frame] hero=({htf.location.x:.1f},{htf.location.y:.1f}) yaw={math.radians(htf.rotation.yaw):+.2f} compass={agent.compass:+.2f} "
              f"gps_pos={np.round(agent.noisy_state_dbg, 1) if hasattr(agent, 'noisy_state_dbg') else '?'} planner_tgt=({pr[1][0][0]:.1f},{pr[1][0][1]:.1f}) "
              f"my_ego_frame={np.round(common_utils.inverse_conversion_2d(np.array(pr[1][0][:2]), np.array([htf.location.x, htf.location.y]), math.radians(htf.rotation.yaw)), 1) if len(pr) > 1 else '?'}", flush=True)
        if _ncall[0] == 100:
            import cv2; rgb = data['rgb'][0].detach().cpu().numpy(); print('[dbg] rgb shape', rgb.shape, 'range', rgb.min(), rgb.max(), flush=True)
            img = np.transpose(rgb, (1, 2, 0)); img = (img * 255 if img.max() <= 1.0 else img).astype(np.uint8); cv2.imwrite('/home/all/carla_ws/logs/bd_free_rgb.png', img[:, :, ::-1])
            lid = data['rasterized_lidar'].detach().cpu().numpy(); print('[dbg] lidar raster', lid.shape, lid.min(), lid.max(), flush=True)
            cv2.imwrite('/home/all/carla_ws/logs/bd_free_lidar.png', (np.clip(lid.reshape(-1, lid.shape[-2], lid.shape[-1])[0], 0, 1) * 255).astype(np.uint8))
            tl = hero.get_traffic_light(); wp = cmap.get_waypoint(hero.get_location())
            print('[dbg] at_light', hero.is_at_traffic_light(), 'state', hero.get_traffic_light_state(), 'junction', wp.is_junction, 'road', wp.road_id, 'lane', wp.lane_id, 'hero yaw', hero.get_transform().rotation.yaw, 'compass', agent.compass, flush=True)
    return p
agent.closed_loop_inference.forward = _capture
from lead.common import common_utils

# ---- viewer window (client side: nothing is drawn into the CARLA world, so the net's cameras stay clean) ----
import pygame
VIEW_W, VIEW_H, BEV_W = 800, 450, 450
view_specs = {'chase': (carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), VIEW_W, VIEW_H, 90.0),
              'bev':   (carla.Location(z=48.0),          carla.Rotation(pitch=-90), BEV_W, VIEW_H, 90.0)}
view_imgs, view_cams = {}, {}
lib = world.get_blueprint_library()
for name, (loc, rot, w, h, fov) in view_specs.items():
    b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(w)); b.set_attribute('image_size_y', str(h)); b.set_attribute('fov', str(fov))
    cam = world.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
    def _cb(img, n=name):
        a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
        view_imgs[n] = (pygame.surfarray.make_surface(a.swapaxes(0, 1)), img.transform)   # transform = camera pose at capture time
    cam.listen(_cb); view_cams[name] = cam
pygame.init(); screen = pygame.display.set_mode((VIEW_W + BEV_W, VIEW_H)); pygame.display.set_caption('BridgeDrive free roam  |  green: GPS points given to the agent  red/orange: current/next target  blue: predicted path')
font = pygame.font.SysFont('monospace', 14, bold=True)

def project(cam_tf, w, h, fov, pts):
    """World points (N,3) -> pixel coords for a pinhole camera at cam_tf. Returns (N,2) and a validity mask."""
    M = np.array(cam_tf.get_inverse_matrix()); P = np.c_[pts, np.ones(len(pts))] @ M.T       # camera frame: x fwd, y right, z up
    f = w / (2.0 * math.tan(math.radians(fov) / 2.0))
    x = np.maximum(P[:, 0], 1e-3); u = f * P[:, 1] / x + w / 2.0; v = -f * P[:, 2] / x + h / 2.0
    return np.c_[u, v], P[:, 0] > 0.3

def overlay(surf, cam_tf, w, h, fov):
    def draw_pts(pts3, color, r, labels=None, line=False):
        if len(pts3) == 0: return
        uv, ok = project(cam_tf, w, h, fov, np.asarray(pts3, dtype=np.float64)); prev = None
        for i, ((u, v), good) in enumerate(zip(uv, ok)):
            if not good or not (-50 < u < w + 50 and -50 < v < h + 50): prev = None; continue
            if line and prev is not None: pygame.draw.line(surf, color, prev, (u, v), 2)
            pygame.draw.circle(surf, color, (int(u), int(v)), r); prev = (u, v)
            if labels is not None: surf.blit(font.render(labels[i], True, color), (int(u) + 6, int(v) - 8))
    if not args.no_dense: draw_pts([[t.location.x, t.location.y, t.location.z + 0.2] for t, _ in route[::4]], (0, 110, 0), 1, line=True)
    draw_pts([[t.location.x, t.location.y, t.location.z + 0.3] for t, _ in agent._global_plan_world_coord], (0, 255, 0), 5,
             labels=[str(c).split('.')[-1] for _, c in agent._global_plan_world_coord])
    draw_pts([[dest.x, dest.y, dest.z + 0.5]], (255, 0, 255), 7, labels=['GOAL'])
    planner = agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance]
    for k, col in ((1, (255, 140, 0)), (0, (255, 0, 0))):
        if len(planner.route) > k: p = planner.route[k][0]; draw_pts([[float(p[0]), float(p[1]), hero.get_transform().location.z + 0.5]], col, 6)   # GPS altitude is not reliable: use ego z
    if 'route' in last_pred and agent.compass is not None:
        hl = hero.get_transform().location
        pts = [common_utils.conversion_2d(np.asarray(pt, dtype=np.float64), np.array([hl.x, hl.y]), agent.compass) for pt in last_pred['route']]
        draw_pts([[float(q[0]), float(q[1]), hl.z + 0.3] for q in pts], (60, 140, 255), 3, line=True)

def draw(step):
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
    x = 0
    for name, (loc, rot, w, h, fov) in view_specs.items():
        if name in view_imgs:
            surf, cam_tf = view_imgs[name]; surf = surf.copy(); overlay(surf, cam_tf, w, h, fov); screen.blit(surf, (x, 0))
        x += w
    v = hero.get_velocity(); planner = agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance]
    hud = [f'{3.6*math.hypot(v.x, v.y):5.1f} km/h   GPS points left {len(planner.route)}   goal {hero.get_transform().location.distance(dest):.0f} m',
           'green = sparse GPS route points the agent receives (~50 m apart / at command changes)   red = current target   blue = net prediction']
    for i, t in enumerate(hud): screen.blit(font.render(t, True, (255, 255, 0)), (8, 8 + 18 * i))
    pygame.display.flip()
    if os.environ.get('BD_SNAP'):
        n = int(os.environ['BD_SNAP'])
        if n > 0 and step % n == 0: pygame.image.save(screen, f'/home/all/carla_ws/logs/bd_view_{step:06d}.png'); print(f'[dbg] saved view step {step}', flush=True)

# collision log (hero): tells you WHY it got stuck, visible in the terminal
_col_bp = world.get_blueprint_library().find('sensor.other.collision'); _col = world.spawn_actor(_col_bp, carla.Transform(), attach_to=hero)
def _on_col(ev):
    o = ev.other_actor; print(f'[collision] with {o.type_id} (id {o.id}) impulse {math.hypot(ev.normal_impulse.x, ev.normal_impulse.y):.0f}', flush=True)
_col.listen(_on_col)
spectator = world.get_spectator(); t0 = time.time(); step = 0; T_SIM0 = world.get_snapshot().timestamp.elapsed_seconds; t_loop = time.time()
print('BridgeDrive driving. Ctrl+C to stop.', flush=True)
try:
    while True:
        step += 1
        if args.sync:
            world.tick(); GameTime.on_carla_tick(world.get_snapshot().timestamp)    # the leaderboard clock the sensor interface/speedometer use
            control = agent()
        elif FIXED:
            # fixed clock: one 50 ms sim step per 1/sim_rate wall seconds, regardless of whether the net is done
            t_next = t_loop + 1.0 / args.sim_rate; d = t_next - time.time()
            if d > 0: time.sleep(d)
            t_loop = max(t_next, time.time() - 1.0 / args.sim_rate)
            world.tick(); control = tracker_step()   # LEAD's controller once per sim step (= 20 Hz in sim time, as trained)
        else:
            t_next = t_loop + 0.05; control = tracker_step()   # 20 Hz wall-clock controller on the net's latest plan
            d = t_next - time.time()
            if d > 0: time.sleep(d)
            t_loop = max(t_next, time.time() - 0.05)
        hero.apply_control(control)
        tf = hero.get_transform()
        spectator.set_transform(carla.Transform(tf.location + carla.Location(z=6) - tf.get_forward_vector() * 9, carla.Rotation(pitch=-18, yaw=tf.rotation.yaw)))
        if SYNC_WORLD or step % 2 == 0: draw(step)   # viewer at 10 Hz in real-time mode: leaves the GIL to the net thread
        if step % (args.fps * 2 if not FIXED else max(1, int(args.sim_rate * 2))) == 0:
            v = hero.get_velocity(); planner = agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance]
            sim_t = step / args.fps if SYNC_WORLD else world.get_snapshot().timestamp.elapsed_seconds - T_SIM0
            rate = f'{(time.time()-t0)/(step/args.fps):.1f}x wall/sim' if args.sync else (f'sim {args.sim_rate:.0f} ticks/s ({args.sim_rate/args.fps:.2f}x real time), ' if FIXED else '') + f'net {net_stats["steps"]/max(1e-6, time.time()-net_stats["t0"]):.1f} Hz, control {step/(time.time()-t0):.1f} Hz, net step {net_stats.get("last_ms", 0):.0f} ms' + (f' (tick {1000*np.mean(prof["tick"][-10:]) if prof["tick"] else 0:.0f} + fwd {1000*np.mean(prof["fwd"][-10:]) if prof["fwd"] else 0:.0f}: backbone {1000*np.mean(prof.get("backbone", [0])[-10:]):.0f} decoder {1000*np.mean(prof.get("planning_decoder", [0])[-10:]):.0f})' if PROFILE else '') + (f' | net_steer={plan["net_control"].steer:+.2f} trk={plan["trk"][0]:+.2f} p0={np.round(plan["trk"][1],1)} p9={np.round(plan["trk"][2],1)}' if plan.get('trk') and plan.get('net_control') else '')
            print(f't={sim_t:6.1f}s {3.6*math.hypot(v.x, v.y):5.1f} km/h thr={control.throttle:.2f} brk={control.brake:.2f} str={control.steer:+.2f} | GPS pts left {len(planner.route):3d} | goal {tf.location.distance(dest):6.1f} m | {rate}', flush=True)
        if tf.location.distance(dest) < 8 or len(agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance].route) <= 1:
            print('arrived', flush=True); apply_new_route()
        elif step % 20 == 0 and step > 100:   # left the route (wrong turn / overshoot)? -> reroute like a nav system would
            fwd = tf.get_forward_vector(); pr = agent.gps_waypoint_planners_dict[agent.config_closed_loop.route_planner_min_distance].route
            off_route = min(tf.location.distance(r[0].location) for r in route[::5]) > 12.0
            tgt_behind = len(pr) > 1 and ((pr[1][0][0] - tf.location.x) * fwd.x + (pr[1][0][1] - tf.location.y) * fwd.y) < -40.0
            if off_route or tgt_behind:
                print(f'[reroute] off_route={off_route} (min dist {min(tf.location.distance(r[0].location) for r in route[::5]):.1f} m) target_behind={tgt_behind} pts_left={len(pr)}', flush=True)
                apply_new_route(keep_dest=True)
except KeyboardInterrupt:
    pass
except Exception:
    import traceback; traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush()
finally:
    print('cleaning up', flush=True)
    try: stop_flag[0] = True; net_thread.join(5)
    except Exception: pass
    try: wrapper.cleanup() if args.sync else [(a.stop(), a.destroy()) for a in agent_sensors]
    except Exception: pass
    for c in list(view_cams.values()) + [_col]:
        try: c.stop(); c.destroy()
        except Exception: pass
    pygame.quit()
    try: hero.destroy()
    except Exception: pass
    if SYNC_WORLD: settings.synchronous_mode = False; settings.fixed_delta_seconds = None; world.apply_settings(settings); tm.set_synchronous_mode(False)
    sys.stdout.flush(); os._exit(0)   # skip interpreter teardown: CARLA sensor threads abort otherwise
