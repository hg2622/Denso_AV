#!/usr/bin/env python
"""Free roam with RIFT: a continuous CARLA world (no Bench2Drive episode, no scoring) where the learned CBV policy
(Pluto / RL fine-tuned Pluto) takes over the vehicles near the ego and drives them, for as long as you like.

    python scripts/freeroam.py [--town Town05] [--cbv rift_pluto] [--ego manual|autopilot] [--npcs 40] [--test SECONDS]

What is reused from RIFT unchanged: the CBV candidate selection (rule-based, "will this car meet the ego's route?"),
the CBV route planner (route to the interaction point + extra waypoints), the Pluto observation/feature builder,
the policy itself (candidate trajectories -> PID control) and the CarlaDataProvider bookkeeping.
What replaces the episode: the ego's route is generated on the fly from wherever the ego is (random turns at every
junction) and extended as it is consumed; if you leave the route it is regenerated from your new position. Traffic is
spawned around the ego and any Traffic-Manager car that falls far behind is moved to a lane ahead, so the pool of
candidates never runs dry. A CBV that reaches its interaction point, collides, gets stuck or falls behind is handed
back to the Traffic Manager (and may be picked again after a cool-down).
Keys (manual ego): W/S throttle-brake, A/D steer, SHIFT gentle, Q reverse, SPACE handbrake, TAB camera, ESC quit.
Red markers = NPCs currently driven by the policy. Needs a CARLA server on --port (bash carla_server.sh).
"""
import argparse
import math
import os
import sys
import time
from collections import deque

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
os.environ.setdefault('CUDA_VISIBLE_DEVICES', '0')

ap = argparse.ArgumentParser()
ap.add_argument('--town', default='Town05')
ap.add_argument('--cbv', default='rift_pluto', help='standard | pluto | rift_pluto | grpo_pluto | ppo_pluto | reinforce_pluto | rs_pluto | rtr_pluto | sft_pluto')
ap.add_argument('--ego', default='manual', choices=['manual', 'autopilot'])
ap.add_argument('--npcs', type=int, default=40, help='Traffic-Manager vehicles kept around the ego')
ap.add_argument('--port', type=int, default=2000); ap.add_argument('--tm_port', type=int, default=8000)
ap.add_argument('--fps', type=float, default=10.0, help='RIFT policies run at 10 Hz')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--profile', default='human', help='CBV selection gates: human (wide) | eval (the paper\'s)')
ap.add_argument('--test', type=float, default=0.0, help='headless: ego on autopilot for N seconds, then print stats')
ap.add_argument('--record', default='', help='write the window to this mp4')
ap.add_argument('--snapshot', default='', help='save window frames to this prefix every 5 s')
ap.add_argument('--show-routes', action='store_true', help='draw the CBV routes in the world')
a = ap.parse_args()
if a.test: os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('RIFT_CBV_PROFILE', a.profile)

import carla  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
import pygame  # noqa: E402
from rift.util.torch_util import set_torch_variable, set_seed  # noqa: E402
set_torch_variable(); set_seed(a.seed)
from rift.scenario.tools.carla_data_provider import CarlaDataProvider  # noqa: E402
from rift.scenario.tools.global_route_planner import GlobalRoutePlanner  # noqa: E402
from rift.scenario.tools.route_manipulation import get_latlon_ref  # noqa: E402
from rift.scenario.tools.timer import GameTime  # noqa: E402
from rift.cbv.planning.pluto.utils.nuplan_map_utils import CarlaMap  # noqa: E402
from rift.cbv.planning import CBV_POLICY_LIST  # noqa: E402
from rift.cbv.recognition import CBV_RECOGNITION_LIST  # noqa: E402
from rift.gym_carla.observation import OBSERVATION_LIST  # noqa: E402
from rift.gym_carla.action import ACTION_TYPE  # noqa: E402
from rift.ego.route_planner.route_planner import EgoRoutePlanner  # noqa: E402
from rift.gym_carla.utils.common import get_nearby_agents, store_agent_state  # noqa: E402
from rift.util.logger import Logger  # noqa: E402

rng = np.random.default_rng(a.seed)
DT = 1.0 / a.fps
SEARCH_RADIUS = 60
USE_POLICY = a.cbv != 'standard'

# ------------------------------------------------------------------ world
client = carla.Client('localhost', a.port); client.set_timeout(120.0)
world = client.get_world()
if a.town not in world.get_map().name:
    print(f'loading {a.town} ...', flush=True); world = client.load_world(a.town)
for act in world.get_actors().filter('vehicle.*'): act.destroy()
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = DT; st.no_rendering_mode = False; world.apply_settings(st)
tm = client.get_trafficmanager(a.tm_port); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)
tm.set_global_distance_to_leading_vehicle(5.0); tm.global_percentage_speed_difference(-10)
world.tick()
cmap = world.get_map()
CarlaDataProvider.set_client(client); CarlaDataProvider.set_world(world, a.town)
CarlaDataProvider.set_traffic_manager_port(a.tm_port)
CarlaDataProvider.set_frame_rate(a.fps); CarlaDataProvider.set_desired_speed(8)
CarlaDataProvider.set_global_route_planner(GlobalRoutePlanner(cmap, 1.0))
CarlaDataProvider.set_map_api(CarlaMap(carla_town_name=a.town))
CarlaDataProvider.set_gps_info(get_latlon_ref(world))
GameTime.restart()

# ------------------------------------------------------------------ RIFT components (same construction as carla_runner.py)
def load_yaml(p):
    with open(p) as f: return yaml.safe_load(f)
common = dict(ROOT_DIR=ROOT, mode='eval', num_scenario=1, seed=a.seed, pretrain_seed=0, cbv_recog='rule', pretrain_ego='pdm_lite',
              frame_rate=a.fps, render=False, output_dir='log', resume=True, port=a.port, tm_port=a.tm_port, repetitions=1)
logger = Logger(os.path.join(ROOT, 'log', 'freeroam', f'{a.cbv}-{a.town}'))
env_params = dict(mode='eval', num_scenario=1, search_radius=SEARCH_RADIUS, ego_policy_type='rule-based', cbv_policy_type='il',
                  ego_obs_type='ego_no_obs', cbv_obs_type='cbv_no_obs', collect_ego_obs_type='ego_no_obs', ROOT_DIR=ROOT,
                  need_video_render=False, signalized_junction=False, warm_up_steps=4, img_size=[1280, 720],
                  acc_range=[-3.0, 3.0], steer_range=[-0.3, 0.3], out_lane_thres=4, desired_speed=8, frame_rate=a.fps)
recog_config = load_yaml('rift/cbv/recognition/config/rule.yaml'); recog_config.update(common); recog_config['desired_speed'] = 8
recog = CBV_RECOGNITION_LIST['rule'](recog_config, logger=logger); recog.set_mode('eval'); recog.set_world()
if USE_POLICY:
    cbv_config = load_yaml(f'rift/cbv/planning/config/{a.cbv}.yaml'); cbv_config.update(common)
    cbv_config.update(ego_policy='manual', ego_obs_type='ego_no_obs', need_video_render=False, desired_speed=8)
    env_params['cbv_policy_type'] = cbv_config['policy_type']; env_params['cbv_obs_type'] = cbv_config['obs_type']
    cbv_policy = CBV_POLICY_LIST[a.cbv](cbv_config, logger=logger)
    cbv_policy.set_mode('eval'); cbv_policy.load_model(resume=True)
CBVAction = ACTION_TYPE['cbv_action'](env_params['acc_range'], env_params['steer_range'], policy_type=env_params['cbv_policy_type'])
CBVObs = OBSERVATION_LIST[env_params['cbv_obs_type']](env_params, CBVAction)
route_planner = CBVObs.CBV_route_planner
recog.set_route_planner(route_planner)
if USE_POLICY: cbv_policy.set_route_planner(route_planner)
ego_rp = EgoRoutePlanner(env_params)
print(f'>> free roam on {a.town}: NPC policy = {a.cbv}, selection profile = {os.environ["RIFT_CBV_PROFILE"]} '
      f'(search {recog.search_radius} m, up to {recog.max_agent_num} CBVs at once)', flush=True)

# ------------------------------------------------------------------ ego + route
spawns = cmap.get_spawn_points(); rng.shuffle(spawns)
ego = None
for sp in spawns:
    try: ego = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', sp, rolename='hero'); break
    except Exception: continue
assert ego is not None, 'ego spawn failed'

def extend_route(wp, n):
    """n more 1 m waypoints along the lane from wp, a random branch at each junction."""
    out = []
    for _ in range(n):
        nxt = wp.next(1.0)
        if not nxt: break
        wp = nxt[int(rng.integers(len(nxt)))] if len(nxt) > 1 else nxt[0]
        out.append(wp)
    return out

def new_route():
    wp = cmap.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    return extend_route(wp, 600)

def set_route(route_wps):
    """(Re)start both planners on a fresh route; the Traffic Manager ego follows the same path."""
    locs = [w.transform.location for w in route_wps]
    if ego.id not in CarlaDataProvider._scenario_actors:
        CarlaDataProvider.set_ego_vehicle_route(ego, locs, 0)      # first time: also creates the BV/CBV bookkeeping
    else:
        CarlaDataProvider._ego_vehicle_route[ego.id] = locs        # later: keep the bookkeeping (set_ego_vehicle_route would wipe it)
    ego_rp.reset(ego, route_wps); route_planner.reset(ego, route_wps)
    if a.ego == 'autopilot' or a.test:
        tm.set_path(ego, [w.transform.location for w in route_wps[::5]])

def keep_route_alive():
    """Extend the route as it is consumed; regenerate it when the ego has left it (a human turned elsewhere)."""
    local = ego_rp.local_route_waypoints or []
    el = ego.get_location()
    if local and min(w.transform.location.distance(el) for w in local) > 12.0:
        for cid in list(CBVs): release(cid, 'route changed')
        set_route(new_route()); return 'regenerated'
    q = ego_rp._waypoints_queue
    if len(q) < 300:
        more = extend_route(q[-1][0] if q else local[-1], 500)
        from rift.ego.route_planner.route_planner import RoadOption
        ego_rp._waypoints_queue = deque(list(q) + [(w, RoadOption.LANEFOLLOW) for w in more], maxlen=len(q) + len(more) + 10)
        if a.ego == 'autopilot' or a.test:
            tm.set_path(ego, [w.transform.location for w, _ in list(ego_rp._waypoint_buffer) + list(ego_rp._waypoints_queue)][::5])
        return 'extended'
    return ''

set_route(new_route())
if a.ego == 'autopilot' or a.test: ego.set_autopilot(True, a.tm_port)

# ------------------------------------------------------------------ traffic
def lane_slots(center_wp, dist_min, dist_max, step=4.0):
    out = []
    for direction in ('next', 'previous'):
        frontier = [center_wp]; d = 0.0
        while d < dist_max and frontier:
            d += step; nxt = []
            for w in frontier: nxt += (w.next(step) if direction == 'next' else w.previous(step))
            frontier = nxt[:6]
            if d < dist_min: continue
            for w in frontier:
                for ww in (w, w.get_left_lane(), w.get_right_lane()):
                    if ww is not None and ww.lane_type == carla.LaneType.Driving and not ww.is_junction: out.append(ww)
    return out

def all_vehicle_locations():
    return [v.get_location() for v in world.get_actors().filter('vehicle.*')]

def free_slots(center_wp, dist_min, dist_max, n, margin=10.0):
    slots = lane_slots(center_wp, dist_min, dist_max); rng.shuffle(slots); taken = all_vehicle_locations(); out = []
    for wp in slots:
        loc = wp.transform.location
        if all(loc.distance(t) > margin for t in taken):
            tf = carla.Transform(carla.Location(loc.x, loc.y, loc.z + 0.3), wp.transform.rotation); out.append(tf); taken.append(loc)
        if len(out) >= n: break
    return out

pts = free_slots(cmap.get_waypoint(ego.get_location()), 15.0, 160.0, a.npcs)
npcs = [v for v in CarlaDataProvider.request_new_batch_actors('vehicle.*', len(pts), pts, autopilot=True, rolename='background', safe_blueprint=True) if v is not None]
CarlaDataProvider.set_scenario_actors(ego, npcs)
for v in npcs: tm.vehicle_percentage_speed_difference(v, float(rng.uniform(-20, 10)))
print(f'>> ego + {len(npcs)} Traffic-Manager vehicles spawned around it', flush=True)

def recycle_far_npcs(max_dist=170.0):
    """A Traffic-Manager car that fell far behind is moved to a free lane 60-130 m ahead of the ego."""
    el = ego.get_location()
    bvs = CarlaDataProvider.get_scenario_actors_by_ego(ego.id)['BVs']
    far = [v for v in bvs.values() if v.is_alive and v.get_location().distance(el) > max_dist]
    if not far: return 0
    tfs = free_slots(cmap.get_waypoint(el), 60.0, 130.0, len(far)); n = 0
    for v, tf in zip(far, tfs):
        v.set_transform(tf); v.set_target_velocity(carla.Vector3D(0, 0, 0)); v.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        hs = CarlaDataProvider._actor_history_state_map.get(v)
        if hs is not None: hs.clear()
        n += 1
    return n

# ------------------------------------------------------------------ CBV management
CBVs = {}            # id -> actor currently driven by the policy
cbv_sensors = {}; cbv_collided = {}; cbv_since = {}; cbv_last_move = {}
released_at = {}     # id -> tick when handed back (cool-down before it can be picked again)
takeovers = 0; control_ticks = 0; cbv_log = []
COOLDOWN_TICKS = int(15 * a.fps)
HIST_SAMPLES = int(2.0 * a.fps) + 1   # Pluto history horizon 2 s + current

def take(cbv):
    global takeovers
    CBVs[cbv.id] = cbv; takeovers += 1
    cbv.set_autopilot(False, a.tm_port)
    CarlaDataProvider.add_CBV(ego, cbv)
    CarlaDataProvider.set_CBV_nearby_agents(ego, cbv, get_nearby_agents(cbv, SEARCH_RADIUS))
    bp = CarlaDataProvider._blueprint_library.find('sensor.other.collision')
    s = world.spawn_actor(bp, carla.Transform(), attach_to=cbv); cbv_collided[cbv.id] = None
    s.listen(lambda e, cid=cbv.id: cbv_collided.__setitem__(cid, e.other_actor.type_id))
    cbv_sensors[cbv.id] = s; cbv_since[cbv.id] = tick; cbv_last_move[cbv.id] = tick

def release(cid, reason):
    cbv = CBVs.pop(cid, None)
    if cbv is None: return
    s = cbv_sensors.pop(cid, None)
    if s is not None:
        try: s.stop(); s.destroy()
        except Exception: pass
    cbv_collided.pop(cid, None)
    if cbv.is_alive: cbv.set_autopilot(True, a.tm_port)
    CarlaDataProvider.pop_CBV_nearby_agents(ego, cbv)
    CarlaDataProvider.CBV_back_to_BV(ego, cbv)
    # the selector skips "reached goal" ids; keep it there during the cool-down, then forget it so it can be picked again
    CarlaDataProvider._scenario_actors[ego.id]['CBVs_reach_goal'][cid] = cbv; released_at[cid] = tick
    cbv_log.append((round(tick * DT, 1), cid, reason, round((tick - cbv_since.pop(cid, tick)) * DT, 1)))
    if len(cbv_log) % 5 == 1 or reason == 'collision':
        print(f'[{tick * DT:6.1f}s] NPC {cid} handed back ({reason}) after {cbv_log[-1][3]} s; {len(CBVs)} still under policy', flush=True)

def manage_cbvs(local, rest):
    # cool-down expiry
    for cid in [c for c, t in released_at.items() if tick - t > COOLDOWN_TICKS]:
        CarlaDataProvider._scenario_actors[ego.id]['CBVs_reach_goal'].pop(cid, None); released_at.pop(cid, None)
    el = ego.get_location(); goal = CarlaDataProvider.get_CBVs_reach_goal_by_ego(ego.id)
    for cid, cbv in list(CBVs.items()):
        if not cbv.is_alive: release(cid, 'destroyed'); continue
        v = cbv.get_velocity()
        if math.hypot(v.x, v.y) > 0.3: cbv_last_move[cid] = tick
        if cid in goal: release(cid, 'reached interaction point')
        elif cbv_collided.get(cid): release(cid, 'collision')
        elif cbv.get_location().distance(el) > 130: release(cid, 'too far')
        elif tick - cbv_last_move[cid] > 12 * a.fps: release(cid, 'stuck')
    if tick % 5 == 0 and len(CBVs) < recog.max_agent_num and USE_POLICY and tick > HIST_SAMPLES:
        for cbv in recog.get_CBVs(ego, set(CBVs.keys()), local, rest):
            # the feature builder uses the controlled car's own stored history unpadded (neighbours are padded), so a car
            # whose history is shorter than the model's 2 s window (freshly spawned or just recycled) cannot be taken yet
            if cbv.id not in CBVs and len(CarlaDataProvider.get_history_state(cbv)) >= HIST_SAMPLES: take(cbv)
    route_planner.update_CBV(rest)
    CarlaDataProvider.set_ego_nearby_agents(ego, get_nearby_agents(ego, SEARCH_RADIUS))
    for cbv in CBVs.values(): CarlaDataProvider.set_CBV_nearby_agents(ego, cbv, get_nearby_agents(cbv, SEARCH_RADIUS))

# ------------------------------------------------------------------ ego collision sensor + window
collisions = []
def _on_collision(e):
    t = world.get_snapshot().timestamp.elapsed_seconds
    if collisions and collisions[-1][2] == e.other_actor.id and t - collisions[-1][0] < 2.0: collisions[-1] = (t, collisions[-1][1], e.other_actor.id); return
    collisions.append((t, e.other_actor.type_id, e.other_actor.id))
cs = world.spawn_actor(CarlaDataProvider._blueprint_library.find('sensor.other.collision'), carla.Transform(), attach_to=ego); cs.listen(_on_collision)

Wn, Hn = 1280, 720; img = {}
pygame.init(); screen = pygame.display.set_mode((Wn, Hn)); pygame.display.set_caption(f'RIFT free roam - {a.cbv} drives the NPCs'); font = pygame.font.SysFont('monospace', 17, bold=True)
CAMS = [('chase', carla.Location(x=-7, z=3.2), carla.Rotation(pitch=-12)), ('top', carla.Location(z=45), carla.Rotation(pitch=-90)), ('bonnet', carla.Location(x=0.8, z=1.3), carla.Rotation())]
cams = []; cam_idx = 0
def attach_cam():
    global cams
    for c in cams:
        try: c.stop(); c.destroy()
        except Exception: pass
    cb = CarlaDataProvider._blueprint_library.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(Wn)); cb.set_attribute('image_size_y', str(Hn)); cb.set_attribute('fov', '95')
    _, loc, rot = CAMS[cam_idx]; cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im: img.__setitem__('cam', pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1))))
    cams = [cam]
attach_cam()
writer = None
if a.record:
    import cv2
    writer = cv2.VideoWriter(a.record, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (Wn, Hn))

# ------------------------------------------------------------------ loop
for _ in range(4): world.tick(); GameTime.on_carla_tick(world.get_snapshot().timestamp); CarlaDataProvider.on_carla_tick(); store_agent_state(ego)
steer = 0.0; ctl = carla.VehicleControl(); clock = pygame.time.Clock(); tick = 0; t_start = time.time(); ms_last = 100
cbv_actions = {}; infer_lat = []; min_gap = float('inf')
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_TAB: cam_idx = (cam_idx + 1) % len(CAMS); attach_cam()
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        if a.ego == 'manual' and not a.test:
            gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
            ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
            inc = 4e-4 * ms_last; smax = 0.7 if kmh < 50 else max(0.25, 0.7 - (kmh - 50) * (0.45 / 70))
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-smax, steer - inc)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(smax, steer + inc)
            else: steer = 0.0
            ctl.steer = round(steer, 2); ego.apply_control(ctl)
        # the policy's controls for this tick (computed from the previous observation, like the RIFT env does)
        for cid, act in cbv_actions.items():
            cbv = CBVs.get(cid)
            if cbv is None or act is None: continue
            r, th, stv, br = CBVAction.convert_action(act)
            cbv.apply_control(carla.VehicleControl(reverse=bool(r), throttle=float(th), steer=float(stv), brake=float(br)))
        ms_last = clock.tick_busy_loop(int(a.fps)) if not a.test else int(1000 / a.fps)
        world.tick(); tick += 1
        GameTime.on_carla_tick(world.get_snapshot().timestamp); CarlaDataProvider.on_carla_tick()
        local, rest, _ = ego_rp.run_step(); route_planner.run_step(local, rest)
        manage_cbvs(local, rest)
        store_agent_state(ego)
        if tick % 20 == 0:
            r = keep_route_alive()
            if r == 'regenerated': local, rest, _ = ego_rp.run_step(); route_planner.run_step(local, rest)
            recycle_far_npcs()
        cbv_actions = {}
        if USE_POLICY and CBVs:
            t0 = time.time(); obs = CBVObs.get_obs(ego)
            if obs: cbv_actions = cbv_policy.get_action([obs], [{'env_id': 0}])['CBVs_actions'][0]
            infer_lat.append(time.time() - t0)
        control_ticks += len(CBVs)
        el = ego.get_location()
        for cbv in CBVs.values():
            loc = cbv.get_location(); min_gap = min(min_gap, loc.distance(el))
            world.debug.draw_point(loc + carla.Location(z=2.6), size=0.18, color=carla.Color(255, 40, 40), life_time=DT * 1.5)
        if a.show_routes and CBVs: route_planner.vis_route()
        # window
        screen.fill((15, 15, 15))
        if 'cam' in img: screen.blit(img['cam'], (0, 0))
        lat = np.mean(infer_lat[-20:]) * 1000 if infer_lat else 0.0
        hud = [f'{kmh:5.1f} km/h  NPCs under {a.cbv}: {len(CBVs)}/{recog.max_agent_num}  takeovers {takeovers}  policy {lat:.0f} ms  ego collisions {len(collisions)}  route left {len(ego_rp._waypoints_queue)} m',
               'W/S A/D  Shift gentle  Q reverse  Space handbrake  TAB camera  ESC quit   (red marker = NPC driven by the policy)']
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 230, 90)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if writer is not None or (a.snapshot and tick % int(5 * a.fps) == 0):
            import cv2
            frame = pygame.surfarray.array3d(screen).swapaxes(0, 1)[:, :, ::-1]
            if writer is not None: writer.write(np.ascontiguousarray(frame))
            if a.snapshot and tick % int(5 * a.fps) == 0: cv2.imwrite(f'{a.snapshot}_{tick:05d}.png', frame)
        if a.test and tick % int(10 * a.fps) == 0:
            print(f'[{tick * DT:6.1f}s] {len(CBVs)} NPCs under policy (takeovers so far {takeovers}), ego {kmh:.0f} km/h, policy {lat:.0f} ms, ego collisions {len(collisions)}', flush=True)
        if a.test and tick * DT >= a.test: break
except KeyboardInterrupt: pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    reasons = {}
    for _, _, r, _ in cbv_log: reasons[r] = reasons.get(r, 0) + 1
    print(f'\n=== {tick * DT:.0f} s simulated in {time.time() - t_start:.0f} s wall; NPC takeovers {takeovers}, NPC-seconds under policy {control_ticks * DT:.0f}, '
          f'hand-backs {reasons}, closest approach {min_gap:.1f} m, policy latency {np.mean(infer_lat) * 1000 if infer_lat else 0:.0f} ms, '
          f'ego collisions {len(collisions)} {[c[1] for c in collisions][:6]}', flush=True)
    if writer is not None: writer.release(); print('video:', a.record)
    for c in cams:
        try: c.stop(); c.destroy()
        except Exception: pass
    try: cs.stop(); cs.destroy()
    except Exception: pass
    for cid in list(CBVs): release(cid, 'exit')
    try:
        tm.set_synchronous_mode(False); st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    pygame.quit(); os._exit(0)
