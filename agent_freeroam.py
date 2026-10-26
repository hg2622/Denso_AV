"""Generic open-world driving for any CARLA-leaderboard agent (SENSORS or MAP track), lock-step (the world waits for the
agent each 50 ms step, like the benchmark). Random route to a random destination, re-planned on arrival; client-side
viewer (chase + top-down) with the route / current target / goal; collision log.

  python agent_freeroam.py --agent team_code/agent_simlingo.py --config /path/to/pytorch_model.pt \
      --leaderboard SimLingo/Bench2Drive/leaderboard --scenario-runner SimLingo/Bench2Drive/scenario_runner \
      [--pythonpath DIR[:DIR]] [--map Town04] [--dist 600] [--speed 1|0] [--no-viewer] [--green-lights]
--speed 1 paces the lock-step to real time if the agent is fast enough (CaRL); 0 = as fast as the agent allows.
"""
import argparse, importlib.util, os, sys, time, random, math
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument('--agent', required=True); ap.add_argument('--config', required=True)
ap.add_argument('--leaderboard', required=True); ap.add_argument('--scenario-runner', required=True); ap.add_argument('--pythonpath', default='')
ap.add_argument('--map', default=None); ap.add_argument('--dist', type=float, default=600); ap.add_argument('--filter', default='vehicle.lincoln.mkz_2017')
ap.add_argument('--port', type=int, default=2000); ap.add_argument('--fps', type=int, default=20); ap.add_argument('--speed', type=float, default=1.0)
ap.add_argument('--no-viewer', action='store_true'); ap.add_argument('--green-lights', action='store_true', help='freeze every traffic light green (slow models would otherwise wait minutes of wall time at each red)'); ap.add_argument('--no-dense', action='store_true'); ap.add_argument('--label', default='agent')
args = ap.parse_args()
WS = '/home/all/carla_ws'
sys.path[:0] = [p for p in args.pythonpath.split(':') if p] + [args.leaderboard, args.scenario_runner, f'{WS}/carla/PythonAPI/carla']
os.environ.setdefault('SCENARIO_RUNNER_ROOT', args.scenario_runner); os.environ.setdefault('LEADERBOARD_ROOT', args.leaderboard)
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from leaderboard.utils.route_manipulation import interpolate_trajectory
from leaderboard.autoagents.agent_wrapper import AgentWrapper

client = carla.Client('localhost', args.port); client.set_timeout(10.0)
for _try in range(30):
    try:
        world = client.get_world(); st = world.get_settings()
        if st.synchronous_mode: st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
        world.get_map().name; break
    except RuntimeError: print(f'waiting for the CARLA server on port {args.port} ... ({_try + 1}/30)', flush=True)
else: sys.exit('CARLA server not reachable - start it with: bash ~/carla_ws/carla_server.sh')
client.set_timeout(120.0)
if args.map and client.get_world().get_map().name.split('/')[-1] != args.map: print('loading', args.map, flush=True); client.load_world(args.map)
world = client.get_world()
settings = world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = 1.0 / args.fps; world.apply_settings(settings)
tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True)
if args.green_lights:
    world.tick(); tls = list(world.get_actors().filter('traffic.traffic_light'))   # a fresh sync client sees no actors before its first tick
    for tl in tls: tl.set_state(carla.TrafficLightState.Green); tl.freeze(True)
    print(f'{len(tls)} traffic lights frozen green', flush=True)
CarlaDataProvider.set_client(client); CarlaDataProvider.set_world(world); cmap = world.get_map(); spawn_pts = cmap.get_spawn_points()
bp = world.get_blueprint_library().find(args.filter); bp.set_attribute('role_name', 'hero'); hero = None
for sp in random.sample(spawn_pts, len(spawn_pts)):
    hero = world.try_spawn_actor(bp, sp)
    if hero: break
CarlaDataProvider.register_actor(hero); CarlaDataProvider._carla_actor_pool[hero.id] = hero; world.tick(); CarlaDataProvider.on_carla_tick()
wp0 = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving); tf0 = wp0.transform; tf0.location.z += 0.3; hero.set_transform(tf0); world.tick(); CarlaDataProvider.on_carla_tick()

def plan_route(keep=None):
    start = cmap.get_waypoint(hero.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving).next(3.0)[0].transform.location
    if keep is None:
        far = [p for p in spawn_pts if p.location.distance(start) > args.dist] or sorted(spawn_pts, key=lambda p: -p.location.distance(start))[:10]
        keep = random.choice(far).location
    gps_route, route = interpolate_trajectory([start, keep]); return keep, gps_route, route

# agent module from file
spec = importlib.util.spec_from_file_location('user_agent', args.agent); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
AgentCls = getattr(mod, mod.get_entry_point())
dest, gps_route, route = plan_route()
agent = AgentCls('localhost', args.port); agent.set_global_plan(gps_route, route); agent.setup(args.config)
wrapper = AgentWrapper(agent); wrapper.setup_sensors(hero)
print(f'{args.label}: route {len(route)} m to ({dest.x:.0f},{dest.y:.0f})', flush=True)
def new_route(keep=None):
    global dest, gps_route, route
    dest, gps_route, route = plan_route(keep); agent.set_global_plan(gps_route, route)
    for name in ('initialized', 'initialised'):
        if hasattr(agent, name): setattr(agent, name, False)   # most garage-style agents rebuild their route planners in _init()
    print(f'new route: {len(route)} m to ({dest.x:.0f},{dest.y:.0f})', flush=True)

if not args.no_viewer:
    import pygame
    VW, VH, BW = 800, 450, 450; specs = {'chase': (carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), VW, VH), 'bev': (carla.Location(z=48.0), carla.Rotation(pitch=-90), BW, VH)}
    imgs, cams = {}, {}; lib = world.get_blueprint_library()
    for name, (loc, rot, w, h) in specs.items():
        b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(w)); b.set_attribute('image_size_y', str(h)); b.set_attribute('fov', '90')
        cam = world.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
        cam.listen(lambda img, n=name: imgs.__setitem__(n, (pygame.surfarray.make_surface(np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)), img.transform))); cams[name] = cam
    pygame.init(); screen = pygame.display.set_mode((VW + BW, VH)); pygame.display.set_caption(f'{args.label} free roam  |  green: route  red: current target  magenta: goal'); font = pygame.font.SysFont('monospace', 14, bold=True)
    def project(cam_tf, w, h, pts):
        M = np.array(cam_tf.get_inverse_matrix()); P = np.c_[pts, np.ones(len(pts))] @ M.T; f = w / 2.0; x = np.maximum(P[:, 0], 1e-3)
        return np.c_[f * P[:, 1] / x + w / 2.0, -f * P[:, 2] / x + h / 2.0], P[:, 0] > 0.3
    def draw_pts(surf, cam_tf, w, h, pts3, color, r, line=False):
        if not len(pts3): return
        uv, ok = project(cam_tf, w, h, np.asarray(pts3, dtype=np.float64)); prev = None
        for (u, v), good in zip(uv, ok):
            if not good or not (-50 < u < w + 50 and -50 < v < h + 50): prev = None; continue
            if line and prev is not None: pygame.draw.line(surf, color, prev, (u, v), 2)
            pygame.draw.circle(surf, color, (int(u), int(v)), r); prev = (u, v)
    def draw():
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
        x = 0
        for name, (loc, rot, w, h) in specs.items():
            if name in imgs:
                surf, cam_tf = imgs[name]; surf = surf.copy()
                if not args.no_dense: draw_pts(surf, cam_tf, w, h, [[t.location.x, t.location.y, t.location.z + 0.2] for t, _ in route[::4]], (0, 110, 0), 1, line=True)
                draw_pts(surf, cam_tf, w, h, [[t.location.x, t.location.y, t.location.z + 0.3] for t, _ in agent._global_plan_world_coord], (0, 255, 0), 5)
                draw_pts(surf, cam_tf, w, h, [[dest.x, dest.y, dest.z + 0.5]], (255, 0, 255), 7); screen.blit(surf, (x, 0))
            x += w
        v = hero.get_velocity(); screen.blit(font.render(f'{3.6*math.hypot(v.x, v.y):5.1f} km/h   goal {hero.get_transform().location.distance(dest):.0f} m   {args.label}', True, (255, 255, 0)), (8, 8)); pygame.display.flip()
col = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=hero); col.listen(lambda ev: print(f'[collision] with {ev.other_actor.type_id}', flush=True))
spectator = world.get_spectator(); t0 = time.time(); step = 0; t_loop = time.time(); ms = []
print(f'{args.label} driving (lock-step {args.fps} Hz). Ctrl+C / ESC to stop.', flush=True)
try:
    while True:
        if args.speed > 0:
            t_next = t_loop + 1.0 / (args.fps * args.speed); d = t_next - time.time()
            if d > 0: time.sleep(d)
            t_loop = max(t_next, time.time() - 1.0 / (args.fps * args.speed))
        world.tick(); step += 1; snap = world.get_snapshot(); GameTime.on_carla_tick(snap.timestamp); CarlaDataProvider.on_carla_tick()
        _t = time.time(); control = agent(); ms.append((time.time() - _t) * 1000); hero.apply_control(control)
        tf = hero.get_transform(); spectator.set_transform(carla.Transform(tf.location + carla.Location(z=6) - tf.get_forward_vector() * 9, carla.Rotation(pitch=-18, yaw=tf.rotation.yaw)))
        if not args.no_viewer and step % 2 == 0: draw()
        if step % (args.fps * 2) == 0:
            v = hero.get_velocity(); print(f't={step/args.fps:6.1f}s {3.6*math.hypot(v.x, v.y):5.1f} km/h thr={control.throttle:.2f} brk={control.brake:.2f} str={control.steer:+.2f} | goal {tf.location.distance(dest):6.1f} m | light={str(hero.get_traffic_light_state()) if hero.is_at_traffic_light() else None} | {step/(time.time()-t0):.1f} ticks/s wall, agent {np.mean(ms[-40:]):.0f} ms/step', flush=True)
        if tf.location.distance(dest) < 8: print('arrived', flush=True); new_route()
        elif step % 40 == 0 and step > 100 and min(tf.location.distance(r[0].location) for r in route[::5]) > 15.0: print('[reroute] left the route', flush=True); new_route(dest)
except KeyboardInterrupt: pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    print('cleaning up', flush=True)
    try: col.stop(); col.destroy()
    except Exception: pass
    try: wrapper.cleanup()
    except Exception: pass
    if not args.no_viewer:
        for c in cams.values():
            try: c.stop(); c.destroy()
            except Exception: pass
        pygame.quit()
    try: hero.destroy()
    except Exception: pass
    settings.synchronous_mode = False; settings.fixed_delta_seconds = None; world.apply_settings(settings); tm.set_synchronous_mode(False)
    sys.stdout.flush(); os._exit(0)
