"""CaRL / Roach (RL planners from autonomousvision/CaRL) drive in an open CARLA world: random route to a random destination,
re-planned on arrival. These agents use NO cameras/LiDAR: they read a privileged bird's-eye map (lane geometry, other
actors, lights, route) straight from the simulator (leaderboard track MAP), so they are fast enough to run in lock-step
at the full 20 Hz in real time.

  bash carl_drive.sh [--map Town04] [--model CaRL_v1_1_PY_01 | CaRL_PY_00 | Roach_00..04] [--dist 600] [--no-dense]
Viewer window: chase cam + top-down; green = route the agent gets, red = current target, magenta = goal.
Server must be running (bash carla_server.sh). Ctrl-C / ESC stops. Traffic: traffic.sh / traffic_chaos.sh after it starts.
"""
import argparse, os, sys, time, random, math
import numpy as np
WS = '/home/all/carla_ws'; CARL = f'{WS}/CaRL/CARLA'
os.environ.setdefault('SAMPLE_TYPE', 'mean'); os.environ.setdefault('DEBUG_ENV_AGENT', '0'); os.environ.pop('SAVE_PATH', None)
os.environ['SCENARIO_RUNNER_ROOT'] = f'{CARL}/original_leaderboard/scenario_runner'; os.environ['LEADERBOARD_ROOT'] = f'{CARL}/original_leaderboard/leaderboard'
sys.path[:0] = [f'{CARL}/team_code', f'{CARL}/original_leaderboard/leaderboard', f'{CARL}/original_leaderboard/scenario_runner', f'{WS}/carla/PythonAPI/carla']
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from leaderboard.utils.route_manipulation import interpolate_trajectory

ap = argparse.ArgumentParser()
ap.add_argument('--map', default=None); ap.add_argument('--dist', type=float, default=600)
ap.add_argument('--model', default='CaRL_v1_1_PY_01', help='folder under CaRL/CARLA/results (CaRL_* or Roach_*)')
ap.add_argument('--filter', default='vehicle.lincoln.mkz_2017'); ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--fps', type=int, default=20); ap.add_argument('--no-dense', action='store_true'); ap.add_argument('--no-viewer', action='store_true')
ap.add_argument('--speed', type=float, default=1.0, help='sim speed factor: 1 = real time, 0 = as fast as possible')
args = ap.parse_args()
if args.model.lower().startswith('roach'): os.environ['SAMPLE_TYPE'] = 'roach'

client = carla.Client('localhost', args.port); client.set_timeout(10.0)
for _try in range(30):
    try:
        world = client.get_world(); st = world.get_settings()
        if st.synchronous_mode: st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
        world.get_map().name; break
    except RuntimeError: print(f'waiting for the CARLA server ... ({_try + 1}/30)', flush=True)
else: sys.exit('CARLA server not reachable - start it with: bash ~/carla_ws/carla_server.sh')
client.set_timeout(120.0)
if args.map and client.get_world().get_map().name.split('/')[-1] != args.map:
    print('loading', args.map, flush=True); client.load_world(args.map)
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
CarlaDataProvider.register_actor(hero); CarlaDataProvider._carla_actor_pool[hero.id] = hero; world.tick(); CarlaDataProvider.on_carla_tick()
wp0 = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving)
tf0 = wp0.transform; tf0.location.z += 0.3; hero.set_transform(tf0); world.tick(); CarlaDataProvider.on_carla_tick()   # align with the lane

def plan_route():
    start = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving).next(3.0)[0].transform.location
    far = [p for p in spawn_pts if p.location.distance(start) > args.dist]
    if not far: far = sorted(spawn_pts, key=lambda p: -p.location.distance(start))[:10]   # small map: take the 10 farthest
    dest = random.choice(far).location
    gps_route, route = interpolate_trajectory([start, dest])
    return dest, gps_route, route

from eval_agent import EvalAgent
dest, gps_route, route = plan_route()
agent = EvalAgent('localhost', args.port); agent.set_global_plan(gps_route, route); agent.setup(f'{CARL}/results/{args.model}')
print(f'model {args.model}: route {len(route)} m to ({dest.x:.0f},{dest.y:.0f})', flush=True)

def apply_new_route():
    global dest, gps_route, route
    dest, gps_route, route = plan_route(); agent.set_global_plan(gps_route, route)
    agent.initialized = False   # agent_init() re-attaches the BEV manager and route planner to the new dense plan on the next step
    print(f'new route: {len(route)} m to ({dest.x:.0f},{dest.y:.0f})', flush=True)

# ---- viewer (client side, chase cam + top-down with overlays) ----
if not args.no_viewer:
    import pygame
    VIEW_W, VIEW_H, BEV_W = 800, 450, 450
    view_specs = {'chase': (carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), VIEW_W, VIEW_H, 90.0),
                  'bev': (carla.Location(z=48.0), carla.Rotation(pitch=-90), BEV_W, VIEW_H, 90.0)}
    view_imgs, view_cams = {}, {}; lib = world.get_blueprint_library()
    for name, (loc, rot, w, h, fov) in view_specs.items():
        b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(w)); b.set_attribute('image_size_y', str(h)); b.set_attribute('fov', str(fov))
        cam = world.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
        def _cb(img, n=name):
            a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
            view_imgs[n] = (pygame.surfarray.make_surface(a.swapaxes(0, 1)), img.transform)
        cam.listen(_cb); view_cams[name] = cam
    pygame.init(); screen = pygame.display.set_mode((VIEW_W + BEV_W, VIEW_H)); pygame.display.set_caption(f'{args.model} free roam  |  green: route  red: current target  magenta: goal')
    font = pygame.font.SysFont('monospace', 14, bold=True)
    def project(cam_tf, w, h, fov, pts):
        M = np.array(cam_tf.get_inverse_matrix()); P = np.c_[pts, np.ones(len(pts))] @ M.T
        f = w / (2.0 * math.tan(math.radians(fov) / 2.0)); x = np.maximum(P[:, 0], 1e-3)
        return np.c_[f * P[:, 1] / x + w / 2.0, -f * P[:, 2] / x + h / 2.0], P[:, 0] > 0.3
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
        draw_pts([[t.location.x, t.location.y, t.location.z + 0.3] for t, _ in agent._global_plan_world_coord], (0, 255, 0), 5, labels=[str(c).split('.')[-1] for _, c in agent._global_plan_world_coord])
        draw_pts([[dest.x, dest.y, dest.z + 0.5]], (255, 0, 255), 7, labels=['GOAL'])
        try:
            rp = agent.route_planner
            if len(rp.route) > 0: p = rp.route[0]; draw_pts([[float(p[0].location.x), float(p[0].location.y), hero.get_transform().location.z + 0.5]], (255, 0, 0), 6)
        except Exception: pass
    def draw(step):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
        x = 0
        for name, (loc, rot, w, h, fov) in view_specs.items():
            if name in view_imgs:
                surf, cam_tf = view_imgs[name]; surf = surf.copy(); overlay(surf, cam_tf, w, h, fov); screen.blit(surf, (x, 0))
            x += w
        v = hero.get_velocity()
        screen.blit(font.render(f'{3.6*math.hypot(v.x, v.y):5.1f} km/h   goal {hero.get_transform().location.distance(dest):.0f} m   {args.model}', True, (255, 255, 0)), (8, 8))
        pygame.display.flip()
        if os.environ.get('BD_SNAP') and step % int(os.environ['BD_SNAP']) == 0: pygame.image.save(screen, f'{WS}/logs/carl_view_{step:06d}.png')

_col = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=hero)
_col.listen(lambda ev: print(f'[collision] with {ev.other_actor.type_id}', flush=True))
spectator = world.get_spectator(); t0 = time.time(); step = 0; t_loop = time.time(); agent_ms = []
print(f'{args.model} driving (lock-step {args.fps} Hz, speed x{args.speed}). Ctrl+C to stop.', flush=True)
try:
    while True:
        if args.speed > 0:   # pace the lock-step loop to real time (x speed)
            t_next = t_loop + 1.0 / (args.fps * args.speed); d = t_next - time.time()
            if d > 0: time.sleep(d)
            t_loop = max(t_next, time.time() - 1.0 / (args.fps * args.speed))
        world.tick(); step += 1; snap = world.get_snapshot()
        GameTime.on_carla_tick(snap.timestamp); CarlaDataProvider.on_carla_tick()
        _t = time.time(); control = agent.run_step({}, snap.timestamp.elapsed_seconds); agent_ms.append((time.time() - _t) * 1000)
        hero.apply_control(control)
        tf = hero.get_transform()
        spectator.set_transform(carla.Transform(tf.location + carla.Location(z=6) - tf.get_forward_vector() * 9, carla.Rotation(pitch=-18, yaw=tf.rotation.yaw)))
        if not args.no_viewer and step % 2 == 0: draw(step)
        if step % (args.fps * 2) == 0:
            v = hero.get_velocity()
            print(f't={step/args.fps:6.1f}s {3.6*math.hypot(v.x, v.y):5.1f} km/h thr={control.throttle:.2f} brk={control.brake:.2f} str={control.steer:+.2f} | goal {tf.location.distance(dest):6.1f} m | {step/(time.time()-t0):.1f} ticks/s wall, agent {np.mean(agent_ms[-40:]):.1f} ms/step', flush=True)
        if tf.location.distance(dest) < 8: print('arrived', flush=True); apply_new_route()
        elif step % 40 == 0 and step > 100 and min(tf.location.distance(r[0].location) for r in route[::5]) > 15.0:
            print('[reroute] left the route', flush=True); dest_keep = dest
            st_ = cmap.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving).next(3.0)[0].transform.location
            gps_route, route = interpolate_trajectory([st_, dest_keep]); agent.set_global_plan(gps_route, route); agent.initialized = False
except KeyboardInterrupt:
    pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    print('cleaning up', flush=True)
    try: _col.stop(); _col.destroy()
    except Exception: pass
    if not args.no_viewer:
        for c in view_cams.values():
            try: c.stop(); c.destroy()
            except Exception: pass
        pygame.quit()
    try: hero.destroy()
    except Exception: pass
    settings.synchronous_mode = False; settings.fixed_delta_seconds = None; world.apply_settings(settings); tm.set_synchronous_mode(False)
    sys.stdout.flush(); os._exit(0)
