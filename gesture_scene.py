"""Staged scene in front of your car (needs the CARLA server on port 2000; any map):
  python gesture_scene.py [--test]
  1. a car ahead of you drives a few metres and STOPS (brake lights)
  2. it flashes the LEFT indicator, then the RIGHT indicator, then HONKS (CARLA has no horn: the headlights flash and a horn
     sound is synthesised and played through pygame)
  3. the driver door and the passenger door open; two people get out and walk to a spot in front of your car, facing you
  4. gestures, ~3 s each, posed on the walker skeleton (set_bones + blend_pose): WAVE, HAND UP to stop traffic,
     HEAD TURNED towards your car (tracks you from then on), PHONE to the ear, HANDS IN POCKETS
  5. finally they turn and RUN away from you at full sprint (4.5 m/s), the doors close
You can drive meanwhile (W/S A/D, Shift gentle, Q reverse, Space handbrake, ESC quit) or just watch; R restarts the scene.
--test: headless, captures one frame per stage into gesture_stages.png and exits."""
import argparse, math, os, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--port', type=int, default=2000); ap.add_argument('--spawn', type=int, default=-1, help='map spawn point index for your car (-1 = pick a straight road)')
ap.add_argument('--test', action='store_true'); ap.add_argument('--out', default='/home/all/carla_ws/selected_runs/gesture_stages.png'); a = ap.parse_args()

client = carla.Client('localhost', a.port); client.set_timeout(60.0); world = client.get_world(); cmap = world.get_map(); lib = world.get_blueprint_library()
st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
for act in list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('walker.*')) + list(world.get_actors().filter('sensor.*')):
    try: act.destroy()
    except Exception: pass
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = 0.05; world.apply_settings(st); world.tick()
world.set_weather(carla.WeatherParameters(cloudiness=20.0, sun_altitude_angle=50.0, sun_azimuth_angle=200.0))

# ---- a straight stretch: spawn point whose lane continues straight for 60 m
def straight_score(tf):
    wp = cmap.get_waypoint(tf.location); d = 0.0; prev = wp
    for _ in range(12):
        n = prev.next(5.0)
        if not n or n[0].is_junction: break
        if abs(((n[0].transform.rotation.yaw - wp.transform.rotation.yaw) + 180) % 360 - 180) > 6: break
        prev = n[0]; d += 5.0
    return d
sps = cmap.get_spawn_points()
ego_tf = sps[a.spawn] if a.spawn >= 0 else max(sps, key=straight_score)
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '10,10,10')
ego = world.try_spawn_actor(eb, ego_tf); assert ego is not None, 'ego spawn failed'
fwd = ego_tf.get_forward_vector(); right = ego_tf.get_right_vector(); E = ego_tf.location
def ahead(dx, dy=0.0, z=0.3): return carla.Location(x=E.x + fwd.x * dx + right.x * dy, y=E.y + fwd.y * dx + right.y * dy, z=E.z + z)
nb = lib.find('vehicle.audi.tt'); nb.set_attribute('color', '200,30,30'); nb.set_attribute('role_name', 'scene')
npc = world.try_spawn_actor(nb, carla.Transform(ahead(11.0), ego_tf.rotation)); assert npc is not None, 'npc spawn failed'
world.tick()

# ---- horn sound (synthesised): two-tone car horn, 0.9 s
pygame.mixer.pre_init(22050, -16, 1, 512); pygame.init()
def make_horn():
    sr = 22050; t = np.arange(int(sr * 0.9)) / sr
    sig = 0.5 * np.sign(np.sin(2 * np.pi * 420 * t)) * 0.3 + 0.5 * np.sin(2 * np.pi * 420 * t) + 0.35 * np.sin(2 * np.pi * 525 * t) + 0.15 * np.sin(2 * np.pi * 840 * t)
    env = np.minimum(1.0, t / 0.03) * np.minimum(1.0, (0.9 - t) / 0.08); snd = (sig * env / np.max(np.abs(sig)) * 32000 * 0.8).astype(np.int16)
    try: return pygame.sndarray.make_sound(np.ascontiguousarray(snd))
    except Exception: return None
HORN = make_horn()

# ---- walker posing (bone axes measured on this skeleton: upper-arm pitch = forward, yaw = sideways/up; forearm yaw = elbow; head yaw = turn)
GEST = {'wave': {'crl_arm__R': (0, 150, 0)}, 'stop': {'crl_arm__R': (80, 20, 0)}, 'phone': {'crl_arm__R': (30, 30, 0), 'crl_foreArm__R': (0, -135, 0)},
        'pockets': {'crl_arm__R': (15, -10, 0), 'crl_foreArm__R': (0, -55, 0), 'crl_arm__L': (15, 10, 0), 'crl_foreArm__L': (0, 55, 0)}, 'look': {}, 'none': {}}
class Person:
    def __init__(self, bp_id, loc, yaw):
        b = lib.find(bp_id); b.set_attribute('is_invincible', 'false'); self.w = None
        for k in range(6):   # the door swing / the car body can block the first spot: step outwards a little and retry
            f = 1.0 + 0.3 * k; cand = carla.Location(E.x + (loc.x - E.x), E.y + (loc.y - E.y), loc.z + 0.6)
            cand = carla.Location(loc.x + (loc.x - npc.get_location().x) * (f - 1.0), loc.y + (loc.y - npc.get_location().y) * (f - 1.0), loc.z + 0.6)
            self.w = world.try_spawn_actor(b, carla.Transform(cand, carla.Rotation(yaw=yaw)))
            if self.w is not None: break
        if self.w is None: print(f'[scene] could not spawn {bp_id}', flush=True)
        self.rest = None; self.gesture = 'none'; self.look = False; self.t_g = 0.0
    def snap_rest(self): self.rest = {b.name: b.relative for b in self.w.get_bones().bone_transforms}
    def walk_to(self, loc, speed):
        p = self.w.get_location(); d = carla.Vector3D(loc.x - p.x, loc.y - p.y, 0.0); n = math.hypot(d.x, d.y)
        if n < 0.5: self.w.apply_control(carla.WalkerControl(speed=0.0)); return True
        self.w.apply_control(carla.WalkerControl(direction=carla.Vector3D(d.x / n, d.y / n, 0.0), speed=speed)); return False
    def face(self, loc):   # stand still, body turned to loc
        p = self.w.get_transform(); yaw = math.degrees(math.atan2(loc.y - p.location.y, loc.x - p.location.x))
        self.w.set_transform(carla.Transform(p.location, carla.Rotation(yaw=yaw))); self.w.apply_control(carla.WalkerControl(speed=0.0))
    def apply_pose(self, tsim, target):
        if self.rest is None: self.snap_rest()
        mods = dict(GEST[self.gesture])
        if self.gesture == 'wave': mods['crl_foreArm__R'] = (0, 35.0 * math.sin(2 * math.pi * 2.0 * (tsim - self.t_g)), 0)   # elbow swings at 2 Hz
        if self.look and target is not None:   # head towards the target: bearing in the walker's frame (CARLA yaw: + = to the right)
            tf = self.w.get_transform(); d = target - tf.location; bearing = (math.degrees(math.atan2(d.y, d.x)) - tf.rotation.yaw + 180) % 360 - 180
            mods['crl_Head__C'] = (10.0, max(-70.0, min(70.0, bearing)), 0)
        if not mods: self.w.blend_pose(0.0); return
        bc = []
        for name, (dp, dy, dr) in mods.items():
            t = self.rest[name]; r = t.rotation; bc.append((name, carla.Transform(t.location, carla.Rotation(pitch=r.pitch + dp, yaw=r.yaw + dy, roll=r.roll + dr))))
        self.w.set_bones(carla.WalkerBoneControlIn(bc)); self.w.blend_pose(1.0)
    def run_from(self, loc, speed=4.5):
        p = self.w.get_location(); d = carla.Vector3D(p.x - loc.x, p.y - loc.y, 0.0); n = max(math.hypot(d.x, d.y), 0.1)
        self.w.blend_pose(0.0); self.w.apply_control(carla.WalkerControl(direction=carla.Vector3D(d.x / n, d.y / n, 0.0), speed=speed))
    def destroy(self):
        try: self.w.destroy()
        except Exception: pass

STAGES = [('car drives up and stops', 0.0), ('LEFT indicator', 3.0), ('RIGHT indicator', 5.0), ('HONK', 7.0), ('doors open', 8.5), ('two people get out and walk in front of you', 9.5)]
GESTURES = [('WAVE', 'wave', 3.5), ('HAND UP - stop!', 'stop', 3.5), ('HEAD turned towards your car', 'look', 3.5), ('PHONE to the ear', 'phone', 3.5), ('HANDS in pockets', 'pockets', 3.5)]
RUN_T = 9.0; SC = {'t_arr': None}   # gestures start when both people have arrived in front of you (t_arr), not at a fixed time
def stage_at(t):
    cur = STAGES[0]
    for s in STAGES:
        if t >= s[1]: cur = s
    if SC['t_arr'] is None: return cur[0]
    tg = t - SC['t_arr']
    for name, g, dur in GESTURES:
        if tg < dur: return name
        tg -= dur
    return 'RUN away at full sprint' if tg < RUN_T else 'done'
def gesture_at(t):
    tg = t - SC['t_arr']
    for name, g, dur in GESTURES:
        if tg < dur: return g
        tg -= dur
    return 'run' if tg < RUN_T else 'gone'
people = []; t_scene = None; done_once = set(); L = carla.VehicleLightState
def reset_scene():
    global people, t_scene, done_once
    for p in people: p.destroy()
    people = []; done_once = set()
    npc.set_transform(carla.Transform(ahead(11.0), ego_tf.rotation)); npc.set_target_velocity(carla.Vector3D(0, 0, 0)); npc.set_light_state(L.NONE)
    SC['t_arr'] = None
    for d in (carla.VehicleDoor.FL, carla.VehicleDoor.FR):
        try: npc.close_door(d)
        except Exception: pass
    t_scene = world.get_snapshot().timestamp.elapsed_seconds
reset_scene()

def step_scene(tsim):
    t = tsim - t_scene; s = stage_at(t); ego_loc = ego.get_location()
    # --- the car: drive to ~24 m ahead then stop with the brake lights on
    npc_loc = npc.get_location(); dist = (npc_loc - E).x * fwd.x + (npc_loc - E).y * fwd.y
    if t < 3.0 and dist < 18.0: npc.apply_control(carla.VehicleControl(throttle=0.4)); npc.set_light_state(L.NONE)
    else: npc.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
    lights = L.Brake if t >= 2.0 else L.NONE
    if 3.0 <= t < 5.0: lights |= L.LeftBlinker if int(t * 2.5) % 2 == 0 else L.NONE
    if 5.0 <= t < 7.0: lights |= L.RightBlinker if int(t * 2.5) % 2 == 0 else L.NONE
    if 7.0 <= t < 8.5:
        lights |= L.HighBeam if int(t * 5) % 2 == 0 else L.LowBeam
        for k in (0, 1):
            if ('horn', k) not in done_once and t >= 7.0 + 0.55 * k:
                done_once.add(('horn', k))
                if HORN is not None and not a.test: HORN.play()
                print('[scene] HONK', flush=True)
    npc.set_light_state(carla.VehicleLightState(int(lights)))
    if t >= 8.5 and 'doors' not in done_once:
        done_once.add('doors')
        for d in (carla.VehicleDoor.FL, carla.VehicleDoor.FR):
            try: npc.open_door(d)
            except Exception: pass
    if t >= 9.5 and 'people' not in done_once:
        done_once.add('people'); ntf = npc.get_transform(); nf = ntf.get_forward_vector(); nr = ntf.get_right_vector(); nl = ntf.location
        for bp, side in (('walker.pedestrian.0012', -1.0), ('walker.pedestrian.0004', 1.0)):
            loc = carla.Location(nl.x + nr.x * 1.6 * side + nf.x * 0.4, nl.y + nr.y * 1.6 * side + nf.y * 0.4, nl.z)
            people.append(Person(bp, loc, ego_tf.rotation.yaw + 180.0))
    if t >= 14.0 and 'doors_close' not in done_once:
        done_once.add('doors_close')
        for d in (carla.VehicleDoor.FL, carla.VehicleDoor.FR):
            try: npc.close_door(d)
            except Exception: pass
    # --- the people: walk to 6 m in front of your car (1.3 m left / right of its centre line); the gestures start once both are there
    alive = [p for p in people if p.w is not None and p.w.is_alive]
    if SC['t_arr'] is None:
        arrived = 0
        for i, p in enumerate(alive):
            if p.rest is None and t >= 10.0: p.snap_rest()
            target = ahead(6.0, 1.3 if i == 1 else -1.3, 0.0)
            if p.walk_to(target, 1.6): p.face(ego_loc); arrived += 1
            p.gesture = 'none'; p.look = False; p.w.blend_pose(0.0)
        if alive and (arrived == len(alive) or t > 9.5 + 25.0): SC['t_arr'] = t
        return stage_at(t)
    g = gesture_at(t)
    if g == 'gone':
        if 'gone' not in done_once:
            done_once.add('gone')
            for p in people: p.destroy()
            people.clear()
        return 'done'
    for p in alive:
        if g == 'run': p.run_from(ego_loc, 4.5); continue
        p.face(ego_loc)
        if g != p.gesture: p.gesture = g; p.t_g = tsim
        p.look = g in ('look', 'phone', 'pockets'); p.apply_pose(tsim, ego_loc)
    return stage_at(t)

# ---- window
W, H = 1250, 660; CW, BW = 800, 450; img = {}
screen = pygame.display.set_mode((W, H)); pygame.display.set_caption('gesture scene'); font = pygame.font.SysFont('monospace', 18, bold=True); big = pygame.font.SysFont('monospace', 26, bold=True)
cams = []
for name, (loc, rot, w_, h_) in {'chase': (carla.Location(x=0.8, z=1.5), carla.Rotation(pitch=-4), CW, H), 'bev': (carla.Location(x=8.0, z=30.0), carla.Rotation(pitch=-90), BW, H)}.items():   # windshield view: people 5 m ahead fill the frame
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(w_)); cb.set_attribute('image_size_y', str(h_)); cb.set_attribute('fov', '70' if name == 'chase' else '90')
    cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im, n=name: img.__setitem__(n, pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)))); cams.append(cam)
clock = pygame.time.Clock(); ms_last = 50; steer = 0.0; ctl = carla.VehicleControl(); shots = {}; last_stage = None
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYUP and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
            if ev.type == pygame.KEYUP and ev.key == pygame.K_r: reset_scene()
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
        ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
        inc = 5e-4 * ms_last
        if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-0.7, steer - inc)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(0.7, steer + inc)
        else: steer = 0.0
        ctl.steer = round(steer, 2); ego.apply_control(ctl if not a.test else carla.VehicleControl(brake=1.0))
        ms_last = clock.tick_busy_loop(20) if not a.test else 50; world.tick(); tsim = world.get_snapshot().timestamp.elapsed_seconds
        stage = step_scene(tsim)
        screen.fill((15, 15, 15)); x = 0
        for n, w_ in (('chase', CW), ('bev', BW)):
            if n in img: screen.blit(img[n], (x, 0))
            x += w_
        t = tsim - t_scene
        for i, line in enumerate([f'{kmh:5.1f} km/h   t={t:4.1f}s   W/S A/D  Shift gentle  Q reverse  Space handbrake  R restart  ESC quit']):
            lab = font.render(line, True, (255, 255, 0)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6)); screen.blit(lab, (10, 8))
        lab = big.render(stage, True, (255, 90, 90)); bg = pygame.Surface((lab.get_width() + 12, lab.get_height() + 4)); bg.set_alpha(170); bg.fill((0, 0, 0)); screen.blit(bg, (6, 34)); screen.blit(lab, (12, 36))
        pygame.display.flip()
        if a.test:
            if stage != last_stage:
                last_stage = stage; shots[stage] = ('pending', tsim); el = ego.get_location(); nl = npc.get_location()
                print(f"[dbg] t={t:5.1f} stage={stage!r} ego=({el.x:.1f},{el.y:.1f}) npc=({nl.x:.1f},{nl.y:.1f}) d_npc={el.distance(nl):.1f}  people={[(round(pp.w.get_location().x,1), round(pp.w.get_location().y,1), round(el.distance(pp.w.get_location()),1)) for pp in people if pp.w and pp.w.is_alive]}  target0={ahead(6.0,-1.3,0.0)}", flush=True)
            for sname, v in list(shots.items()):
                if v[0] == 'pending' and tsim - v[1] >= 1.6 and 'chase' in img:   # 1.6 s into the stage
                    arr = pygame.surfarray.array3d(screen).swapaxes(0, 1)[:, :CW][:, :, ::-1]; shots[sname] = ('done', arr.copy())
            if stage == 'done' and SC['t_arr'] is not None and t > SC['t_arr'] + sum(d for _, _, d in GESTURES) + RUN_T + 1.5:
                import cv2
                frames = [cv2.resize(v[1], (500, 412)) for k, v in shots.items() if v[0] == 'done' and k != 'done']
                for fr, k in zip(frames, [k for k, v in shots.items() if v[0] == 'done' and k != 'done']): cv2.putText(fr, k, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                while len(frames) % 4: frames.append(np.zeros_like(frames[0]))
                cv2.imwrite(a.out, np.vstack([np.hstack(frames[i:i + 4]) for i in range(0, len(frames), 4)])); print('stages saved ->', a.out, flush=True); break
except KeyboardInterrupt: pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    pygame.quit()
    for cam in cams:
        try: cam.stop()
        except Exception: pass
    time.sleep(0.3)
    try:
        st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    print('bye', flush=True); os._exit(0)
