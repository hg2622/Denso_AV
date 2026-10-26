"""Town05 parking lot, live: you drive the ego through a busy lot (needs the CARLA server on port 2000 at EPIC quality - Low segfaults on Town05).
  python parking_lot.py [--parked 36] [--lot-cars 3] [--walkers 30] [--street 30] [--seed N] [--test-exit]
What happens around you:
  * bays are full of parked cars; a few cars circulate slowly in the aisles and stop for you / walkers / each other
  * pedestrians walk ONLY along the aisles and the lot edge (never into parked cars): from a car door to the road edge and back; a car that
    hits one knocks them down and they stay down (they are mortal; they never path into a standing car, so brushing one does not happen)
  * now and then, near you: a parked car opens its door and a pedestrian GETS OUT and walks away (door closes again)
  * now and then, near you: a pedestrian walks to a parked car, the door opens, they GET IN, and the car backs out of the bay, drives the
    aisle to the lot exit, crosses onto the street and joins the Traffic Manager traffic (= an NPC leaves the lot for the open road)
  * street cars drive the surrounding streets on the Traffic Manager
Keys: W/S A/D (Bench2Drive steering, real-time 20 Hz), Shift gentle, Q reverse, Space handbrake, ESC quit (actors stay; the next start clears them).
Geometry (measured from a top-down render): lot x in [-40, 22], y in [-48, -10]; bay rows at x = 9.8/3.5, -7/-12.9, -23.2/-29 (cars face +-x), bay
centres y = -44.1 + 2.95 k (k = 0..9); aisles at x = 17, -1.6, -18.2, -35; the ground is flat from the lot across the sidewalk to the street at y = -4.4."""
import argparse, math, os, random, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--parked', type=int, default=36); ap.add_argument('--lot-cars', type=int, default=0); ap.add_argument('--walkers', type=int, default=30)
ap.add_argument('--street', type=int, default=30); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--test', action='store_true', help='headless: snapshot after a few seconds and exit'); ap.add_argument('--test-exit', action='store_true', help='headless: make one parked car leave the lot right away and report whether it reached the street')
ap.add_argument('--test-arrive', action='store_true', help='headless: a street car comes in and parks; report'); ap.add_argument('--snapshot', default=''); a = ap.parse_args()
rng = random.Random(a.seed or None)
LOT = dict(x0=-40.0, x1=22.0, y0=-48.0, y1=-10.0)
BAY_X = [9.8, 3.5, -7.0, -12.9, -23.2, -29.0]; BAY_Y = [-44.1 + 2.95 * k for k in range(10)]
AISLE_OF = {9.8: 17.0, 3.5: -1.6, -7.0: -1.6, -12.9: -18.2, -23.2: -18.2, -29.0: -35.0}   # which aisle serves which bay row
LOOPS = [[(-35, -14.5), (10, -14.5), (10, -16.5), (-35, -16.5)]]   # the top corridor only (the aisles are dead ends)
EDGE_Y = -11.5                       # the walkable lot edge next to the street (y = -10 is the kerb line)
TOP_Y = -14.5                        # the only cross corridor: along the top edge (the aisles dead-end at the bottom fence)
DRIVEWAY_Y = -30.0                   # the 12 m gap in the south fence (y -35.5 .. -24) onto the street lane at x = -44.1 (road 47 lane 2, traffic heads -y)
STREET_X = -44.1
def exit_path(aisle):                # from the top of an aisle: along the top corridor to the west aisle, down to the driveway, out onto the street
    pts = [] if abs(aisle + 35.0) < 0.5 else [(aisle, TOP_Y), (-35.0, TOP_Y)]
    return pts + [(-35.0, DRIVEWAY_Y + 4.0), (-36.5, DRIVEWAY_Y), (STREET_X + 1.5, DRIVEWAY_Y - 1.0), (STREET_X, DRIVEWAY_Y - 6.0)]
def arrive_path(bay):                # from the street (upstream of the driveway) into the lot and nose-first into a bay of the west row (x = -29, aisle -35)
    x, y = bay; return [(STREET_X, DRIVEWAY_Y + 0.5), (-39.0, DRIVEWAY_Y), (-35.0, DRIVEWAY_Y + (2.5 if y > DRIVEWAY_Y else -2.5)), (-35.0, y + (-3.0 if y > DRIVEWAY_Y else 3.0)), (-33.0, y), (-30.5, y)]
ENTRANCE = carla.Transform(carla.Location(x=-41.0, y=DRIVEWAY_Y, z=0.3), carla.Rotation(yaw=0.0))   # on the driveway, pointing into the lot

client = carla.Client('localhost', a.port); client.set_timeout(300.0); world = client.get_world()
if 'Town05' not in world.get_map().name: print('loading Town05 ...', flush=True); world = client.load_world('Town05'); time.sleep(3); world = client.get_world()
st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)   # a previous session may have left sync mode on: destroying would block
try: client.get_trafficmanager(a.port + 6000).set_synchronous_mode(False)
except Exception: pass
for act in list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('walker.*')) + list(world.get_actors().filter('controller.*')) + list(world.get_actors().filter('sensor.*')):
    try: act.destroy()
    except Exception: pass
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = 0.05; world.apply_settings(st)
world.set_weather(carla.WeatherParameters(cloudiness=20.0, sun_altitude_angle=55.0, sun_azimuth_angle=120.0))
tm = client.get_trafficmanager(a.port + 6000); tm.set_synchronous_mode(True); tm.set_global_distance_to_leading_vehicle(3.0); tm.global_percentage_speed_difference(10.0)
lib = world.get_blueprint_library(); cmap = world.get_map(); world.tick()
cars_bp = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and not any(k in b.id for k in ('carlacola', 'firetruck', 'ambulance', 'sprinter', 'truck', 'bus', 'cybertruck', 't2'))]
wbps = lib.filter('walker.pedestrian.*')
def car_bp(role):
    b = rng.choice(cars_bp)
    if b.has_attribute('color'): b.set_attribute('color', rng.choice(b.get_attribute('color').recommended_values))
    b.set_attribute('role_name', role); return b
def walker_bp():
    b = rng.choice(wbps)
    if b.has_attribute('is_invincible'): b.set_attribute('is_invincible', 'false')   # mortal: a car that hits them knocks them down (invincible walkers act like posts and flip cars)
    return b

# ---- ego
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '10,10,10')
ego = world.try_spawn_actor(eb, ENTRANCE if not (a.test_exit or a.test_arrive) else carla.Transform(carla.Location(x=17.0, y=-18.0, z=0.3), carla.Rotation(yaw=-90.0))); assert ego is not None, 'ego spawn failed at the entrance'
# ---- which bays are already taken by the map's own static cars (meshes, not actors): ray-cast each bay
def bay_has_static_car(x, y):
    for dx in (-1.5, 0.0, 1.5):
        for pt in world.cast_ray(carla.Location(x=x + dx, y=y, z=3.0), carla.Location(x=x + dx, y=y, z=-1.0)):
            if str(pt.label).split('.')[-1] in ('Car', 'Vehicles'): return True
    return False
STATIC_BAYS = {(x, y) for x in BAY_X for y in BAY_Y if bay_has_static_car(x, y)}
print(f'bays: {len(BAY_X) * len(BAY_Y)} total, {len(STATIC_BAYS)} hold the map\'s own static cars, {len(BAY_X) * len(BAY_Y) - len(STATIC_BAYS)} free for actors', flush=True)
# ---- parked cars (nose away from the aisle, as in a real bay, so leaving = reversing out)
bays = [(x, y) for x in BAY_X for y in BAY_Y if (x, y) not in STATIC_BAYS]; rng.shuffle(bays); parked = []
for (x, y) in bays:
    if len(parked) >= a.parked: break
    yaw = (180.0 if AISLE_OF[x] > x else 0.0) + rng.uniform(-3, 3)
    v = world.try_spawn_actor(car_bp('parked'), carla.Transform(carla.Location(x=x, y=y, z=0.3), carla.Rotation(yaw=yaw)))
    if v is not None: v.apply_control(carla.VehicleControl(hand_brake=True)); parked.append(dict(actor=v, x=x, y=y, aisle=AISLE_OF[x], state='parked', t=0.0, steer=0.0, path=[], walker=None))
# ---- lot cars on loops
lot_cars = []
for i in range(a.lot_cars):
    loop = LOOPS[i % len(LOOPS)]; v = None
    for _try in range(10):
        k = rng.randrange(len(loop)); p, q = loop[k], loop[(k + 1) % len(loop)]; f = rng.uniform(0.15, 0.85)
        x, y = p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]); yaw = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
        if math.hypot(x - ENTRANCE.location.x, y - ENTRANCE.location.y) < 14.0: continue
        v = world.try_spawn_actor(car_bp('lot'), carla.Transform(carla.Location(x=x, y=y, z=0.5), carla.Rotation(yaw=yaw)))
        if v is not None: break
    if v is not None: lot_cars.append(dict(actor=v, loop=loop, k=(k + 1) % len(loop), speed=rng.uniform(1.8, 2.6), steer=0.0))
# ---- walkers: live on the aisles and the lot edge only
def door_point(p=None):
    if p is None: x = rng.choice(BAY_X); y = rng.choice(BAY_Y)
    else: x, y = p['x'], p['y']
    side = 1.0 if AISLE_OF[x] > x else -1.0; return carla.Location(x=x + side * 3.4, y=y + rng.uniform(-0.6, 0.6), z=0.3)   # in the aisle next to the car's door
def aisle_x_for(loc): return min(AISLE_OF.values(), key=lambda ax: abs(ax - loc.x))
def route_to(from_loc, to_loc):
    """aisle-only path: along the current aisle to the edge, along the edge to the target aisle, down that aisle"""
    ax, bx = aisle_x_for(from_loc), aisle_x_for(to_loc); pts = []
    if abs(ax - bx) > 0.5: pts += [carla.Location(x=ax, y=EDGE_Y, z=0.3), carla.Location(x=bx, y=EDGE_Y, z=0.3)]
    pts.append(to_loc); return pts
def road_point(): return carla.Location(x=rng.choice(list(AISLE_OF.values())), y=EDGE_Y, z=0.3)
walkers = []
def add_walker(loc, target, speed=None):
    w = world.try_spawn_actor(walker_bp(), carla.Transform(carla.Location(x=loc.x, y=loc.y, z=0.8), carla.Rotation(yaw=rng.uniform(0, 360))))
    if w is None: return None
    wk = dict(actor=w, path=route_to(loc, target), speed=speed or rng.uniform(0.8, 1.5), wait=rng.randint(0, 60), leg='road' if target.y > EDGE_Y - 1 else 'door', dead=False, enter=None)
    walkers.append(wk); return wk
for i in range(a.walkers):
    for _try in range(10):
        d = door_point(); wk = add_walker(d, road_point())
        if wk is not None: break
# ---- street traffic
street = []; sps = [p for p in cmap.get_spawn_points() if math.hypot(p.location.x + 9, p.location.y + 29) < 160]; rng.shuffle(sps)
for p in sps[:a.street]:
    v = world.try_spawn_actor(car_bp('street'), p)
    if v is not None: v.set_autopilot(True, tm.get_port()); street.append(v)
for _ in range(3): world.tick()
print(f'lot ready: {len(parked)} parked, {len(lot_cars)} lot cars, {len(walkers)} walkers, {len(street)} street cars', flush=True)

# ---------------------------------------------------------------- behaviours
def all_obstacles(): return list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('walker.*'))
LAST_BLOCK = {}
def blocked(v, dist, reverse=False, width=2.0, wdist=None, wwidth=1.4):
    """something in the way within dist m ahead (or behind when reversing); walkers use wdist / wwidth (closer, so bystanders beside the path don't stall a manoeuvre)"""
    tf = v.get_transform(); f = tf.get_forward_vector(); r = tf.get_right_vector(); loc = tf.location
    for o in all_obstacles():
        if o.id == v.id: continue
        d = o.get_location() - loc; lon = d.x * f.x + d.y * f.y; lat = d.x * r.x + d.y * r.y
        if reverse: lon = -lon
        walker = 'walker' in o.type_id
        if 0 < lon < ((wdist or dist) if walker else dist) and abs(lat) < (wwidth if walker else width): LAST_BLOCK[v.id] = o.type_id; return True
    return False
def drive_to(v, c, tx, ty, speed, reverse=False):
    tf = v.get_transform(); f = tf.get_forward_vector(); r = tf.get_right_vector(); d = carla.Location(x=tx, y=ty, z=tf.location.z) - tf.location
    lon = d.x * f.x + d.y * f.y; lat = d.x * r.x + d.y * r.y
    if reverse: lon, lat = -lon, -lat
    L = max(math.hypot(lon, lat), 2.0); ang = math.atan2(lat, max(lon, 0.3)); delta = math.atan2(2.0 * 2.8 * math.sin(ang), L); st_ = max(-0.7, min(0.7, delta if not reverse else -delta))
    c['steer'] = max(c['steer'] - 0.06, min(c['steer'] + 0.06, st_)); vv = v.get_velocity(); spd = math.hypot(vv.x, vv.y)
    speed = speed * (0.45 if abs(ang) > 0.7 else (0.7 if abs(ang) > 0.35 else 1.0))   # slow down for sharp turns (a 5 m radius at 3 m/s overshoots into the fences)
    thr = max(0.0, min(0.65, 0.35 + 0.35 * (speed - spd))); brk = 0.6 if spd > speed + 0.8 else 0.0
    if spd < 0.3 and brk == 0: thr = max(thr, 0.5)   # enough torque to start moving with the wheels turned (or over a kerb)
    v.apply_control(carla.VehicleControl(throttle=thr if brk == 0 else 0.0, brake=brk, steer=c['steer'], reverse=reverse))
def follow_path(v, c, speed, reverse=False):
    """drive the car along c['path'] (list of (x, y)); returns True when the path is done. Stuck against something for 2.5 s (not an actor):
    back up for 1.5 s with the wheels turned the other way, then try again."""
    if not c['path']: return True
    tx, ty = c['path'][0]; loc = v.get_location(); vv = v.get_velocity(); spd = math.hypot(vv.x, vv.y)
    if math.hypot(tx - loc.x, ty - loc.y) < 2.2: c['path'].pop(0); c['stuck'] = 0; return not c['path']
    if c.get('unstick', 0) > 0: c['unstick'] -= 1; v.apply_control(carla.VehicleControl(throttle=0.45, reverse=not reverse, steer=-c['steer'])); return False
    if blocked(v, 5.0, reverse, width=1.6, wdist=3.0, wwidth=1.0): v.apply_control(carla.VehicleControl(brake=0.8, steer=c['steer'], reverse=reverse)); c['stuck'] = 0
    else:
        drive_to(v, c, tx, ty, speed, reverse); c['stuck'] = c.get('stuck', 0) + (1 if spd < 0.15 else -c.get('stuck', 0))
        if c['stuck'] > 50: c['stuck'] = 0; c['unstick'] = 30
    return False
def step_lot_cars():
    for c in lot_cars:
        v = c['actor']; tx, ty = c['loop'][c['k']]; loc = v.get_location()
        if math.hypot(tx - loc.x, ty - loc.y) < 2.5: c['k'] = (c['k'] + 1) % len(c['loop']); tx, ty = c['loop'][c['k']]
        if blocked(v, 7.0): v.apply_control(carla.VehicleControl(brake=0.8, steer=c['steer']))
        else: drive_to(v, c, tx, ty, c['speed'])
def step_walkers():
    for wk in list(walkers):
        w = wk['actor']
        if not w.is_alive or wk['dead']: continue
        loc = w.get_location()
        if loc.z < -1.0: wk['dead'] = True; continue
        # a slow / stationary car pointing at me from close by (parking, backing out): get out of its way sideways, whatever I was doing
        yield_dir = None
        for o in world.get_actors().filter('vehicle.*'):
            tf = o.get_transform(); fo = tf.get_forward_vector(); ro = tf.get_right_vector(); d = loc - tf.location; lon = d.x * fo.x + d.y * fo.y; lat = d.x * ro.x + d.y * ro.y
            ov = o.get_velocity(); rev = o.get_control().reverse
            if rev: lon = -lon
            if 0 < lon < 4.5 and abs(lat) < 1.6 and math.hypot(ov.x, ov.y) < 1.5 and o.attributes.get('role_name') != 'hero': yield_dir = carla.Vector3D(ro.x * (1 if lat >= 0 else -1), ro.y * (1 if lat >= 0 else -1), 0.0); break
        if yield_dir is not None: w.apply_control(carla.WalkerControl(direction=yield_dir, speed=1.2)); continue
        if wk['wait'] > 0: wk['wait'] -= 1; w.apply_control(carla.WalkerControl(speed=0.0)); continue
        if not wk['path']:
            if wk['enter'] is not None: continue
            wk['leg'] = 'door' if wk['leg'] == 'road' else 'road'; wk['path'] = route_to(loc, door_point() if wk['leg'] == 'door' else road_point()); wk['wait'] = rng.randint(40, 200); continue
        t = wk['path'][0]
        if math.hypot(t.x - loc.x, t.y - loc.y) < 0.9: wk['path'].pop(0); continue
        f = carla.Vector3D(t.x - loc.x, t.y - loc.y, 0.0); n = math.hypot(f.x, f.y); f = carla.Vector3D(f.x / n, f.y / n, 0.0)
        # a car right in front: a MOVING car -> stand still and let it pass; a STANDING car -> step around it (otherwise car and walker wait for each other forever)
        side = 0.0
        for o in world.get_actors().filter('vehicle.*'):
            d = o.get_location() - loc; lon = d.x * f.x + d.y * f.y; lat = -d.x * f.y + d.y * f.x
            if 0 < lon < 3.0 and abs(lat) < 2.0:
                ov = o.get_velocity()
                if math.hypot(ov.x, ov.y) > 1.2: side = None; break   # a car coming at walking pace or slower: step aside instead (else both wait forever)
                side = 1.0 if lat < 0 else -1.0
        if side is None: w.apply_control(carla.WalkerControl(speed=0.0)); continue
        if side: f = carla.Vector3D(-f.y * side, f.x * side, 0.0)   # sidestep: 90 degrees away from the car
        w.apply_control(carla.WalkerControl(direction=f, speed=wk['speed']))
EVENT = ['', -99.0]
def say(txt, tsim): EVENT[0] = txt; EVENT[1] = tsim; print(f'[lot] {txt}', flush=True)
def door(v, open_):
    try: (v.open_door if open_ else v.close_door)(carla.VehicleDoor.FL)
    except Exception: pass
def step_parked(tsim, ego_loc, force_exit=False, force_arrive=False):
    """events near you: (1) a parked car opens its door and a pedestrian gets out; (2) a pedestrian gets into a car which then leaves the lot."""
    busy = any(p['state'] not in ('parked', 'gone', 'stalled') for p in parked)
    if not busy and (force_arrive or (rng.random() < 0.0015 and not (a.test_exit or a.test_arrive))):
        taken = {(round(q['x'], 1), round(q['y'], 1)) for q in parked if q['state'] != 'gone'}
        free = [(x, y) for x in (-29.0,) for y in BAY_Y if (round(x, 1), round(y, 1)) not in taken and (x, y) not in STATIC_BAYS and y > DRIVEWAY_Y - 12]
        if free:
            bay = rng.choice(free); v = world.try_spawn_actor(car_bp('parked'), carla.Transform(carla.Location(x=STREET_X, y=DRIVEWAY_Y + 22.0, z=0.3), carla.Rotation(yaw=270.0)))
            if v is not None:
                parked.append(dict(actor=v, x=bay[0], y=bay[1], aisle=-35.0, state='arriving', t=tsim, steer=0.0, path=arrive_path(bay), walker=None)); say('a car is coming in from the street to park', tsim); busy = True
    for p in parked:
        v = p['actor']
        if p['state'] in ('gone', 'stalled'): continue
        if p['state'] == 'parked':
            if busy: continue
            near = 7.0 < ego_loc.distance(v.get_location()) < 24.0
            if force_exit or (near and rng.random() < 0.0025 and not (a.test_exit or a.test_arrive)):
                d = door_point(p); cand = [wk for wk in walkers if wk['actor'].is_alive and not wk['dead'] and wk['enter'] is None and wk['actor'].get_location().distance(d) < 40]
                wk = min(cand, key=lambda wk: wk['actor'].get_location().distance(d)) if cand else add_walker(carla.Location(x=d.x, y=d.y + 2.0, z=0.3), d)
                if wk is not None:
                    wk['path'] = route_to(wk['actor'].get_location(), d); wk['wait'] = 0; wk['enter'] = p; p['walker'] = wk; p['state'] = 'waiting_walker'; p['t'] = tsim
                    say('a pedestrian is walking to a parked car to drive it out', tsim); busy = True
            elif near and rng.random() < 0.0025 and not (a.test_exit or a.test_arrive):
                door(v, True); d = door_point(p); wk = add_walker(carla.Location(x=d.x - 0.8 * (1 if p['aisle'] > p['x'] else -1), y=d.y, z=0.3), road_point(), speed=rng.uniform(0.9, 1.4))
                if wk is not None: wk['wait'] = 25
                p['state'] = 'door_open'; p['t'] = tsim; say('a car door opens ahead - someone is getting out', tsim); busy = True
            continue
        if p['state'] == 'door_open':
            if tsim - p['t'] > 4.0: door(v, False); p['state'] = 'parked'
        elif p['state'] == 'waiting_walker':
            wk = p['walker']
            if wk is None or not wk['actor'].is_alive or wk['dead'] or tsim - p['t'] > 60: p['state'] = 'parked'; p['walker'] = None
            elif not wk['path'] and wk['actor'].get_location().distance(door_point(p)) < 2.5: door(v, True); p['state'] = 'boarding'; p['t'] = tsim
        elif p['state'] == 'boarding':
            if tsim - p['t'] > 2.0:
                wk = p['walker']
                try: wk['actor'].destroy()
                except Exception: pass
                wk['dead'] = True; door(v, False); p['state'] = 'back_out'; p['t'] = tsim; p['steer'] = 0.0; p['path'] = [(p['aisle'], p['y'])]; say('the car backs out of its bay', tsim)
        elif p['state'] == 'back_out':   # phase A: reverse straight until the car sits in the aisle
            loc = v.get_location()
            if abs(loc.x - p['aisle']) < 1.0 or tsim - p['t'] > 10: p['state'] = 'swing'; p['t'] = tsim
            elif blocked(v, 4.0, reverse=True, width=1.3, wdist=3.0, wwidth=1.1): v.apply_control(carla.VehicleControl(brake=0.8, reverse=True))
            else: v.apply_control(carla.VehicleControl(throttle=0.35, reverse=True, steer=0.0))
        elif p['state'] == 'swing':      # phase B: keep reversing with full lock so the nose swings towards the exit direction (+y along the aisle)
            yaw = v.get_transform().rotation.yaw; dyaw = (90.0 - yaw + 180.0) % 360.0 - 180.0
            if abs(dyaw) < 30.0 or tsim - p['t'] > 8:
                v.apply_control(carla.VehicleControl(brake=1.0)); loc = v.get_location()
                p['state'] = 'to_exit'; p['t'] = tsim; p['path'] = [(p['aisle'], loc.y + 7.0)] + exit_path(p['aisle']); p['steer'] = 0.0; say('the car drives to the exit', tsim)
            elif blocked(v, 3.5, reverse=True, width=1.3, wdist=2.5, wwidth=1.1): v.apply_control(carla.VehicleControl(brake=0.8, reverse=True))
            else: v.apply_control(carla.VehicleControl(throttle=0.4, reverse=True, steer=-0.7 if dyaw > 0 else 0.7))   # reversing: the yaw rate has the opposite sign of the steer
        elif p['state'] == 'to_exit':
            loc = v.get_location(); wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if follow_path(v, p, 3.0) or (wp.transform.location.distance(loc) < 1.2 and loc.x < STREET_X + 2.5):
                v.apply_control(carla.VehicleControl()); v.set_autopilot(True, tm.get_port()); p['state'] = 'gone'; say('the car left the lot and joined the street traffic', tsim)
            elif tsim - p['t'] > 150: v.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True)); p['state'] = 'stalled'; say('a car gave up leaving (stuck)', tsim)
        elif p['state'] == 'arriving':   # a street car comes in through the driveway and drives nose-first towards its bay
            if follow_path(v, p, 2.6) or tsim - p['t'] > 90: p['state'] = 'docking'; p['t'] = tsim
        elif p['state'] == 'docking':    # final metres: creep straight into the bay centre, then handbrake
            loc = v.get_location(); d = math.hypot(p['x'] - loc.x, p['y'] - loc.y); yaw = v.get_transform().rotation.yaw
            if d < 0.6 or tsim - p['t'] > 25 or (d < 1.2 and abs((yaw + 180) % 360 - 180) < 8):
                v.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True)); p['state'] = 'parked'; say('a car has parked', tsim)
            elif blocked(v, 2.5, width=1.2, wdist=2.2, wwidth=1.0): v.apply_control(carla.VehicleControl(brake=0.8))   # someone at the door: wait
            else: drive_to(v, p, p['x'] + 3.0, p['y'], 0.9)   # aim past the bay centre so the car straightens out (bay heading = +x)

# ---------------------------------------------------------------- window
W, H = 1250, 660; CW, BW = 800, 450; img = {}
pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption('Town05 parking lot'); font = pygame.font.SysFont('monospace', 17, bold=True)
cams = []
for name, (loc, rot, w_, h_) in {'chase': (carla.Location(x=-6.5, z=3.0), carla.Rotation(pitch=-12), CW, H), 'bev': (carla.Location(z=38.0), carla.Rotation(pitch=-90), BW, H)}.items():
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(w_)); cb.set_attribute('image_size_y', str(h_)); cb.set_attribute('fov', '90')
    cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im, n=name: img.__setitem__(n, pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)))); cams.append(cam)
clock = pygame.time.Clock(); ms_last = 50; steer = 0.0; ctl = carla.VehicleControl(); tick_n = 0; t_start = time.time(); exit_car = None
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYUP and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
        ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
        inc = 5e-4 * ms_last
        if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-0.7, steer - inc)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(0.7, steer + inc)
        else: steer = 0.0
        ctl.steer = round(steer, 2); ego.apply_control(ctl if not (a.test_exit or a.test_arrive) else carla.VehicleControl(brake=1.0))
        ms_last = clock.tick_busy_loop(20) if not (a.test or a.test_exit or a.test_arrive) else 50; world.tick(); tick_n += 1; tsim = world.get_snapshot().timestamp.elapsed_seconds
        step_lot_cars(); step_walkers(); step_parked(tsim, ego.get_location(), force_exit=(a.test_exit and tick_n == 40), force_arrive=(a.test_arrive and tick_n == 40))
        if a.test_exit or a.test_arrive:
            if tick_n == 40: exit_car = next((p for p in parked if p['state'] == ('arriving' if a.test_arrive else 'waiting_walker')), None); print('exit test: car', exit_car['actor'].type_id if exit_car else None, 'bay', (round(exit_car['x'], 1), round(exit_car['y'], 1)) if exit_car else None, flush=True)
            if exit_car and tick_n % 40 == 0:
                l = exit_car['actor'].get_location(); vv2 = exit_car['actor'].get_velocity(); wp = cmap.get_waypoint(l, project_to_road=True, lane_type=carla.LaneType.Driving)
                print(f"  t={tick_n * 0.05:5.1f}s state={exit_car['state']:14s} pos=({l.x:6.1f},{l.y:6.1f}) yaw={exit_car['actor'].get_transform().rotation.yaw:6.1f} v={3.6 * math.hypot(vv2.x, vv2.y):4.1f} km/h  nearest lane {wp.transform.location.distance(l):4.1f} m  blocked_by={LAST_BLOCK.pop(exit_car['actor'].id, '-')}", flush=True)
                if exit_car['state'] == 'gone' and wp.transform.location.distance(l) < 1.5 and math.hypot(vv2.x, vv2.y) > 2.0: print('EXIT TEST PASSED: the car is on a street lane under Traffic Manager control', flush=True); break
                if a.test_arrive and exit_car['state'] == 'parked': print(f"ARRIVE TEST {'PASSED' if math.hypot(l.x - exit_car['x'], l.y - exit_car['y']) < 1.2 and abs((exit_car['actor'].get_transform().rotation.yaw + 180) % 360 - 180) < 15 else 'FAILED'}: parked at ({l.x:.1f},{l.y:.1f}) target ({exit_car['x']:.1f},{exit_car['y']:.1f}) yaw {exit_car['actor'].get_transform().rotation.yaw:.0f}", flush=True); break
            if tick_n > 20 * 180: print('EXIT TEST FAILED: timed out', flush=True); break
        screen.fill((15, 15, 15)); x = 0
        for n, w_ in (('chase', CW), ('bev', BW)):
            if n in img: screen.blit(img[n], (x, 0))
            x += w_
        hud = [f'{kmh:5.1f} km/h {"REVERSE " if ctl.reverse else ""}  parked {sum(1 for p in parked if p["state"] != "gone")}  lot cars {len(lot_cars)}  walkers {sum(1 for w in walkers if w["actor"].is_alive and not w["dead"])}  street {len(street)}   W/S A/D  Shift gentle  Q reverse  Space handbrake  ESC quit']
        if tsim - EVENT[1] < 4.0: hud.append('!! ' + EVENT[0])
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 255, 0)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if a.test and a.snapshot and tick_n == 120: pygame.image.save(screen, a.snapshot); print('snapshot saved', flush=True); break
        if a.test and time.time() - t_start > 60: break
except KeyboardInterrupt: pass
finally:
    pygame.quit()
    try:
        for v in street:
            try: v.set_autopilot(False, tm.get_port())
            except Exception: pass
        tm.set_synchronous_mode(False); st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    print('bye', flush=True); os._exit(0)
