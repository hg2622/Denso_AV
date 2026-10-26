"""Spawn NPC cars and pedestrians AROUND the hero vehicle (not across the whole map).
Usage: python traffic_near.py [-n CARS] [-w WALKERS] [-r RADIUS_M] [--lights] [--follow]
  --follow : every few seconds, top up cars/walkers that drifted > radius away (keeps traffic around you)
Ctrl+C removes everything it spawned."""
import argparse, random, time, sys, math, os, signal, threading
import carla

ap = argparse.ArgumentParser()
ap.add_argument('-n', type=int, default=60, help='cars')
ap.add_argument('-w', type=int, default=40, help='walkers')
ap.add_argument('-r', type=float, default=250.0, help='radius around hero (m)')
ap.add_argument('--lights', action='store_true', help='car headlights on')
ap.add_argument('--follow', action='store_true', help='keep topping up traffic near the hero')
ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--chaos-radius', type=float, default=100.0, help='metres from the hero: inside, every same-direction car speeds up, cars behind overtake and cut in; normal outside')
ap.add_argument('--max-actors', type=int, default=99, help='max cars doing an overtake/cut-in at once (default: every car in range)')
ap.add_argument('--max-walkers', type=int, default=3, help='max pedestrians crossing in front of you at once')
ap.add_argument('--chaos', action='store_true', help='purposeful hazards near you: cars behind overtake and cut in, cars ahead brake-check, pedestrians run across (limited to --max-actors / --max-walkers at a time)')
args = ap.parse_args()

client = carla.Client('localhost', args.port); client.set_timeout(30.0)
world = client.get_world(); bpl = world.get_blueprint_library()
tm = client.get_trafficmanager(8000); tm.set_global_distance_to_leading_vehicle(2.5)
tm.set_respawn_dormant_vehicles(True); tm.set_boundaries_respawn_dormant_vehicles(25, args.r)
tm.set_hybrid_physics_mode(False)
if args.chaos:
    world.set_pedestrians_cross_factor(1.0)   # every walker may cross roads anywhere

def hero():
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == 'hero':
            return a
    return None

try: world.wait_for_tick(seconds=10.0)   # sync-mode worlds (bd_drive.sh, evaluations): a new client sees no actors before its first tick
except RuntimeError: pass
h = hero()
if h is None:
    sys.exit('No hero vehicle — open the driving window (drive.sh) first, then run this.')

car_bps = [b for b in bpl.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4
           and not any(k in b.id for k in ('firetruck', 'ambulance', 'police', 'carlacola', 'sprinter', 't2', 'cybertruck'))]
walker_bps = bpl.filter('walker.pedestrian.*'); ctrl_bp = bpl.find('controller.ai.walker')
spawn_pts = world.get_map().get_spawn_points()
cars, walkers, ctrls = [], [], []

cmap = world.get_map()
ROLE_LIMIT, WALKER_LIMIT = args.max_actors, args.max_walkers
import collections; DBG = collections.Counter()
SIM_T = 0.0     # simulation seconds (from the last snapshot): manoeuvre timers use this, not the wall clock
aggressive = {}  # vehicle id -> currently on the aggressive profile
roles = {}      # vehicle id -> dict(role, phase, t0, side)   (cars currently doing a manoeuvre)
cooldown = {}   # actor id -> time before it may be picked again
wroles = {}     # walker id -> end time of its road crossing
def _tm(call):
    try: call()
    except RuntimeError: pass   # TM may not have registered the vehicle yet this tick
def set_profile(v, bad):
    if bad:   # aggressive: runs lights/signs, tailgates, 2x speed limit. Still avoids other vehicles (TM collision checks stay on)
        _tm(lambda: tm.ignore_lights_percentage(v, 100)); _tm(lambda: tm.ignore_signs_percentage(v, 100))
        _tm(lambda: tm.ignore_vehicles_percentage(v, 0)); _tm(lambda: tm.ignore_walkers_percentage(v, 50))
        _tm(lambda: tm.distance_to_leading_vehicle(v, 1.0)); _tm(lambda: tm.vehicle_percentage_speed_difference(v, -100))
    else:     # normal law-abiding TM
        _tm(lambda: tm.ignore_lights_percentage(v, 0)); _tm(lambda: tm.ignore_signs_percentage(v, 0))
        _tm(lambda: tm.ignore_vehicles_percentage(v, 0)); _tm(lambda: tm.ignore_walkers_percentage(v, 0))
        _tm(lambda: tm.distance_to_leading_vehicle(v, 2.5)); _tm(lambda: tm.vehicle_percentage_speed_difference(v, random.uniform(-20, 10)))
    _tm(lambda: tm.random_left_lanechange_percentage(v, 0)); _tm(lambda: tm.random_right_lanechange_percentage(v, 0)); _tm(lambda: tm.auto_lane_change(v, False))

def _rel(hl, fwd, right, loc):
    d = loc - hl; return d.x * fwd.x + d.y * fwd.y, d.x * right.x + d.y * right.y     # (longitudinal, lateral) in hero frame
def _wp(loc): return cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
def _fs(wp): return wp.s if wp.lane_id < 0 else -wp.s          # along-lane coordinate that increases in the driving direction
def _speed(a): v = a.get_velocity(); return math.sqrt(v.x ** 2 + v.y ** 2)
def lane_free(allv, wp_lane, s_from, s_to, exclude):
    """No vehicle in this lane (same road & lane id) with along-lane offset in (s_from, s_to) relative to wp_lane."""
    s0 = _fs(wp_lane)
    for o in allv:
        if o.id in exclude: continue
        wo = _wp(o.get_location())
        if wo is None or wo.road_id != wp_lane.road_id or wo.lane_id != wp_lane.lane_id: continue
        if s_from < _fs(wo) - s0 < s_to: return False
    return True
def neighbour(wp, side):
    nb = wp.get_left_lane() if side < 0 else wp.get_right_lane()
    return nb if (nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * wp.lane_id > 0) else None
def ahead_blocked(allv, v, dist=10.0):
    """Any vehicle within dist m straight ahead of v (own frame, +-2.2 m)?"""
    tf = v.get_transform(); f = tf.get_forward_vector(); r = tf.get_right_vector()
    for o in allv:
        if o.id == v.id: continue
        lon, lat = _rel(tf.location, f, r, o.get_location())
        if 0 < lon < dist and abs(lat) < 2.2: return True
    return False
def steer_to(v, r, target, vspeed, tgt_speed):
    """Pure-pursuit steering (bicycle geometry, rate-limited so the car never jerks) + simple speed control toward a world point."""
    tf = v.get_transform(); f = tf.get_forward_vector(); rt = tf.get_right_vector()
    lon, lat = _rel(tf.location, f, rt, target)
    L = max(math.hypot(lon, lat), 4.0); alpha = math.atan2(lat, max(lon, 0.5))
    delta = math.atan2(2.0 * 2.9 * math.sin(alpha), L)            # front-wheel angle for a 2.9 m wheelbase
    steer = max(-0.5, min(0.5, delta / 1.0))                      # ~1 rad max wheel angle in CARLA cars
    prev = r.get('steer', 0.0); step = 0.03                        # at most 0.03/tick (~1 full-lock/s): smooth, no sawing
    steer = max(prev - step, min(prev + step, steer)); r['steer'] = steer
    err = tgt_speed - vspeed
    thr = max(0.0, min(1.0, 0.5 + 0.25 * err)); brk = 1.0 if err < -3.0 else 0.0
    v.apply_control(carla.VehicleControl(throttle=thr if brk == 0 else 0.0, brake=brk, steer=steer))
def end_role(v, now, cd):
    roles.pop(v.id, None); cooldown[v.id] = now + cd
    if v.is_alive:
        try: v.set_autopilot(True, tm.get_port())
        except RuntimeError: pass
        set_profile(v, False)

def drive_actors(h):
    """Every tick: steer the cars that currently have a manoeuvre (they are off the Traffic Manager while doing it)."""
    if not roles: return
    htf = h.get_transform(); hl = htf.location; fwd = htf.get_forward_vector(); right = htf.get_right_vector(); now = SIM_T
    hspeed = _speed(h); allv = list(world.get_actors().filter('vehicle.*')); wh = _wp(hl)
    for v in list(cars):
        r = roles.get(v.id)
        if r is None: continue
        if not v.is_alive or v.get_location().distance(hl) > args.chaos_radius * 1.5: end_role(v, now, 10); continue
        if r['role'] == 'brake':
            if now - r['t0'] < r['dur']: v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=r.get('steer', 0.0)))
            else: end_role(v, now, 25)
            continue
        # ---- overtake: pass in the neighbouring lane, then cut back into the hero's lane in front of it
        wv = _wp(v.get_location())
        if wv is None: end_role(v, now, 10); continue
        lon, lat = _rel(hl, fwd, right, v.get_location()); vspeed = _speed(v)
        in_hero_lane = wh is not None and wv.road_id == wh.road_id and wv.lane_id == wh.lane_id
        if r['phase'] == 'pass':
            lane = neighbour(wv, r['side']) if in_hero_lane else wv       # still in my lane -> head for the passing lane
            if lane is None: print(f'[chaos] car {v.id} gave up: no passing lane', flush=True); end_role(v, now, 15); continue
            if ahead_blocked(allv, v, 8.0 + vspeed * 0.6):                # something close ahead -> brake, no ramming
                v.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8, steer=r.get('steer', 0.0)))
            else:
                nxt = lane.next(max(10.0, vspeed * 1.2)); tgt = nxt[0].transform.location if nxt else lane.transform.location
                steer_to(v, r, tgt, vspeed, min(hspeed + 10.0, 40.0))
            if lon > 5 and not in_hero_lane and wh is not None and lane_free(allv, wh, 1.0, lon + 12.0, {h.id, v.id}):   # ahead of me -> cut in as soon as my lane is clear to just past it
                r['phase'] = 'cut'; r['t0'] = now; print(f'[chaos] car {v.id} cuts in front of you', flush=True)
            elif lon > 7: DBG['cut_wait_in_hero_lane' if in_hero_lane else 'cut_wait_lane_busy'] += 1
            elif now - r['t0'] > 25 or lon > 60 or lon < -80:
                print(f'[chaos] car {v.id} gave up overtaking (lon={lon:.0f})', flush=True); end_role(v, now, 15)
        else:   # cut
            lane = wv if in_hero_lane else neighbour(wv, -r['side'])
            if lane is None or ahead_blocked(allv, v, 6.0 + vspeed * 0.5):
                v.apply_control(carla.VehicleControl(throttle=0.0, brake=0.6, steer=r.get('steer', 0.0)))
            else:
                nxt = lane.next(max(10.0, vspeed * 1.2)); tgt = nxt[0].transform.location if nxt else lane.transform.location
                steer_to(v, r, tgt, vspeed, max(hspeed - 1.0, 4.0))          # settle in front of the hero slightly slower than it
            if (in_hero_lane and now - r['t0'] > 2.5) or now - r['t0'] > 8: end_role(v, now, 20)

def manage_chaos(h):
    """Every 0.25 s: start new manoeuvres while slots are free (at most ROLE_LIMIT cars / WALKER_LIMIT walkers at a time, only
    within --chaos-radius; every other NPC drives normally).
       overtake : car behind me in my lane pulls into a free neighbouring lane, floors it past me, then cuts back in front of me --
                  only started when that lane is open ahead of it and my lane ahead of me is open (so the cut-in hits nobody).
       brake    : car in front of me in my lane brakes hard for ~2 s.
       walkers  : pedestrians near me run across the road through my lane ahead of me."""
    htf = h.get_transform(); hl = htf.location; fwd = htf.get_forward_vector(); right = htf.get_right_vector(); now = SIM_T
    hspeed = _speed(h); allv = list(world.get_actors().filter('vehicle.*')); wh = _wp(hl)
    for v in cars:   # every same-direction car inside the radius drives aggressively (speeds up, tailgates); normal outside
        if not v.is_alive or v.id in roles: continue
        d = v.get_location().distance(hl); vf = v.get_transform().get_forward_vector(); same = (vf.x * fwd.x + vf.y * fwd.y) > 0.5
        want = d < args.chaos_radius and same
        if aggressive.get(v.id) != want: set_profile(v, want); aggressive[v.id] = want
    free = ROLE_LIMIT - len(roles)
    DBG['calls'] += 1; DBG['hero_junction'] += int(wh is None or wh.is_junction); DBG['hero_moving'] += int(hspeed > 1.0)
    if DBG['calls'] % 40 == 0 and os.environ.get('CHAOS_DEBUG'): print('[dbg]', dict(DBG), 'roles', len(roles), 'hspeed', round(hspeed, 1), flush=True)
    if free > 0 and wh is not None and not wh.is_junction:
        cand = [v for v in cars if v.is_alive and v.id not in roles and cooldown.get(v.id, 0) < now
                and v.get_location().distance(hl) < args.chaos_radius]
        random.shuffle(cand)
        for v in cand:
            if free <= 0: break
            lon, lat = _rel(hl, fwd, right, v.get_location())
            vf = v.get_transform().get_forward_vector()
            if (vf.x * fwd.x + vf.y * fwd.y) < 0.7 or abs(lat) > 6.0: DBG['not_near'] += 1; continue   # same direction, my lane or the next one
            in_my_lane = abs(lat) < 2.0
            if -60 < lon < (-5 if in_my_lane else 10) and hspeed > 0.5 and random.random() < 0.9:   # behind me (or beside me in the neighbour lane)
                wv = _wp(v.get_location())
                if wv is None or wv.is_junction: DBG['junction'] += 1; continue
                if in_my_lane:   # needs a free same-direction lane next to it to pull into
                    side = -1 if neighbour(wv, -1) is not None else (1 if neighbour(wv, 1) is not None else 0)
                    if side == 0: DBG['no_side_lane'] += 1; continue
                    plane = neighbour(wv, side)
                else:            # already in a neighbour lane: must be the lane right next to mine, same direction
                    side = -1 if lat < 0 else 1
                    if neighbour(wh, side) is None or neighbour(wh, side).lane_id != wv.lane_id or neighbour(wh, side).road_id != wv.road_id:
                        DBG['other_lane'] += 1; continue
                    plane = wv
                if not lane_free(allv, plane, 2.0, 25.0, {v.id}): DBG['pass_lane_busy'] += 1; continue   # its passing lane open for the next 25 m
                if not lane_free(allv, wh, 1.0, 20.0, {h.id}): DBG['my_lane_busy'] += 1; continue         # my lane open for 20 m ahead of me
                try: v.set_autopilot(False, tm.get_port())
                except RuntimeError: continue
                roles[v.id] = dict(role='overtake', phase='pass', t0=now, side=side); free -= 1
                print(f'[chaos] car {v.id} overtaking you on the {"left" if side < 0 else "right"}', flush=True)
            elif in_my_lane and 6 < lon < 30 and hspeed > 3.0 and random.random() < 0.08:   # ahead of me in my lane -> brake check
                try: v.set_autopilot(False, tm.get_port())
                except RuntimeError: continue
                roles[v.id] = dict(role='brake', phase='brake', t0=now, dur=random.uniform(1.5, 2.5), steer=v.get_control().steer); free -= 1
                print(f'[chaos] car {v.id} brakes hard in front of you', flush=True)
    # --- walkers: a few run across the road through my lane
    wmap = {w.id: (w, c) for w, c in zip(walkers, ctrls)}
    for wid in [k for k, t in wroles.items() if t < now]:          # finished crossers go back to wandering
        del wroles[wid]; wc = wmap.get(wid)
        if wc and wc[0].is_alive and wc[1].is_alive:
            dest = world.get_random_location_from_navigation(); wc[1].go_to_location(dest if dest else wc[0].get_location()); wc[1].set_max_speed(random.uniform(0.8, 2.0))
    wfree = WALKER_LIMIT - len(wroles)
    for w, c in zip(walkers, ctrls):
        if wfree <= 0: break
        if not (w.is_alive and c.is_alive) or w.id in wroles or cooldown.get(w.id, 0) > now: continue
        lon, lat = _rel(hl, fwd, right, w.get_location())
        if 8 < lon < 35 and 2.5 < abs(lat) < 10 and hspeed > 1.0 and random.random() < 0.15:
            tgt = hl + carla.Location(x=fwd.x * (lon + 2) - right.x * math.copysign(4.0, lat), y=fwd.y * (lon + 2) - right.y * math.copysign(4.0, lat))
            c.go_to_location(tgt); c.set_max_speed(random.uniform(2.0, 3.5))         # cross to the far side, right through my lane
            wroles[w.id] = now + 8.0; cooldown[w.id] = now + 30.0; wfree -= 1
            print(f'[chaos] pedestrian {w.id} runs across the road ahead', flush=True)
def spawn_cars(n, center):
    pts = sorted(spawn_pts, key=lambda t: t.location.distance(center))
    pts = [p for p in pts if 15 < p.location.distance(center) < args.r]
    random.shuffle(pts); ok = 0
    for p in pts:
        if ok >= n: break
        bp = random.choice(car_bps)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        v = world.try_spawn_actor(bp, p)
        if v is None: continue
        v.set_autopilot(True, tm.get_port())
        set_profile(v, False)
        if args.lights:
            v.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position))
        tm.vehicle_percentage_speed_difference(v, random.uniform(-20, 10))
        cars.append(v); ok += 1
    return ok

def spawn_behind(h, want=3):
    """Make sure `want` cars are 25-90 m behind the hero in its lane / the same-direction lanes next to it (spawned on the lane
    centre via map waypoints, not at the map's fixed spawn points)."""
    htf = h.get_transform(); hl = htf.location; fwd = htf.get_forward_vector(); right = htf.get_right_vector()
    allv = list(world.get_actors().filter('vehicle.*'))
    behind = 0
    for o in allv:
        if o.id == h.id: continue
        lon, lat = _rel(hl, fwd, right, o.get_location()); of = o.get_transform().get_forward_vector()
        if -90 < lon < -10 and abs(lat) < 6 and (of.x * fwd.x + of.y * fwd.y) > 0.5: behind += 1
    ok = 0
    wh = _wp(hl)
    if wh is None or wh.is_junction: return 0
    lanes = [wh] + [nb for nb in (neighbour(wh, -1), neighbour(wh, 1)) if nb is not None]
    for _ in range((want - behind) * 4):
        if behind + ok >= want: break
        lane = random.choice(lanes); prev = lane.previous(random.uniform(25.0, 90.0))
        if not prev: continue
        wp = prev[0]
        if wp.is_junction: continue
        if any(o.get_location().distance(wp.transform.location) < 10.0 for o in allv): continue
        bp = random.choice(car_bps)
        if bp.has_attribute('color'): bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        tf = wp.transform; tf.location.z += 0.3
        v = world.try_spawn_actor(bp, tf)
        if v is None: continue
        v.set_autopilot(True, tm.get_port()); set_profile(v, False); cars.append(v); allv.append(v); ok += 1
    return ok

def ring_points(h, rmin=12.0, rmax=90.0, step=7.0):
    """Lane-centre waypoints around the hero: ahead and behind along its road (own lane + same-direction neighbours), plus the
    opposite-direction lanes - i.e. a ring left/right/front/back, independent of the map's fixed spawn points."""
    wh = _wp(h.get_location())
    if wh is None: return []
    pts, seen = [], set()
    def add(wp):
        k = (wp.road_id, wp.lane_id, round(wp.s / step))
        if k in seen: return
        seen.add(k); pts.append(wp)
    for lane0 in [wh] + [nb for nb in (neighbour(wh, -1), neighbour(wh, 1)) if nb is not None] + [wh.get_left_lane(), wh.get_right_lane()]:
        if lane0 is None or lane0.lane_type != carla.LaneType.Driving: continue
        for direction in ('next', 'previous'):
            frontier, d = [lane0], 0.0
            while frontier and d < rmax:
                d += step; nxt = []
                for wp in frontier:
                    for w2 in (getattr(wp, direction)(step) or []):
                        if w2.lane_type == carla.LaneType.Driving: nxt.append(w2)
                        if d >= rmin and not w2.is_junction: add(w2)
                        for nb in (neighbour(w2, -1), neighbour(w2, 1)):
                            if nb is not None and d >= rmin and not nb.is_junction: add(nb)
                frontier = nxt[:4]
    random.shuffle(pts)
    return [p for p in pts if rmin <= p.transform.location.distance(h.get_location()) <= rmax]

def spawn_ring(h, n):
    """Spawn n cars on lane points around the hero (all sides), spaced from other vehicles."""
    allv = list(world.get_actors().filter('vehicle.*')); ok = 0
    for wp in ring_points(h):
        if ok >= n: break
        if any(o.get_location().distance(wp.transform.location) < 9.0 for o in allv): continue
        bp = random.choice(car_bps)
        if bp.has_attribute('color'): bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        tf = wp.transform; tf.location.z += 0.3
        v = world.try_spawn_actor(bp, tf)
        if v is None: continue
        v.set_autopilot(True, tm.get_port()); set_profile(v, False); cars.append(v); allv.append(v); ok += 1
    return ok

def spawn_walkers(n, center):
    ok, tries = 0, 0
    while ok < n and tries < n * 30:
        tries += 1
        loc = world.get_random_location_from_navigation()
        if loc is None or loc.distance(center) > args.r or loc.distance(center) < 10: continue
        bp = random.choice(walker_bps)
        if bp.has_attribute('is_invincible'): bp.set_attribute('is_invincible', 'true')   # mortal walkers get destroyed when hit -> their AI controller thread throws in C++ and aborts the client
        w = world.try_spawn_actor(bp, carla.Transform(loc))
        if w is None: continue
        c = world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=w)
        world.wait_for_tick()
        dest = world.get_random_location_from_navigation()
        c.start(); c.go_to_location(dest if dest else loc); c.set_max_speed(random.uniform(0.8, 2.0))
        walkers.append(w); ctrls.append(c); ok += 1
    return ok

def cleanup_far(center):
    global cars, walkers, ctrls
    keep, drop = [], []
    for v in cars:
        (drop if (not v.is_alive or v.get_location().distance(center) > args.r * 2.5) else keep).append(v)
    for v in drop:
        if v.is_alive: v.destroy()
    cars = keep
    kw, kc = [], []
    for w, c in zip(walkers, ctrls):
        if w.is_alive and w.get_location().distance(center) <= args.r * 2.5:
            kw.append(w); kc.append(c)
        else:
            if c.is_alive: c.stop(); c.destroy()
            if w.is_alive: w.destroy()
    walkers, ctrls = kw, kc

center = h.get_location()
print(f'hero {h.type_id} at ({center.x:.0f},{center.y:.0f}); spawning within {args.r:.0f} m ...')
print(f'  cars: {(spawn_ring(h, args.n) if args.chaos else spawn_cars(args.n, center))} / {args.n}   walkers: {spawn_walkers(args.w, center)} / {args.w}')
STOP = False
def _sigint(sig, frm):
    global STOP
    if STOP:   # second Ctrl-C: do not wait for cleanup, just leave (NPCs stay; stop_all.sh / server restart clears them)
        print('\nforced exit'); os._exit(1)
    STOP = True; print('\nstopping (Ctrl-C again to force) ...')
signal.signal(signal.SIGINT, _sigint); signal.signal(signal.SIGTERM, _sigint)   # Ctrl-C, kill, timeout all clean up
try:
    last = time.time(); last_chaos = 0.0
    while not STOP:
        # walker navigation is computed client-side and only advances while THIS client processes ticks
        try: w_snap = world.wait_for_tick(seconds=2.0)   # short timeout so Ctrl-C is noticed even if the server stalls
        except RuntimeError: continue
        SIM_T = w_snap.timestamp.elapsed_seconds
        if args.chaos:
            hh = hero()
            if hh is not None:
                drive_actors(hh)
                if time.time() - last_chaos > 0.25: last_chaos = time.time(); manage_chaos(hh)
        if time.time() - last < 5: continue
        last = time.time()
        h = hero()
        if h is None: continue
        center = h.get_location()
        if args.follow:
            cleanup_far(center)
            if args.chaos:
                nc = spawn_ring(h, args.n - len(cars)) if len(cars) < args.n else 0   # ring around the hero: ahead, behind, left, right
            else:
                nc = spawn_cars(args.n - len(cars), center) if len(cars) < args.n else 0
            nw = spawn_walkers(args.w - len(walkers), center) if len(walkers) < args.w else 0
            if nc or nw: print(f'  topped up: +{nc} cars +{nw} walkers  (now {len(cars)} cars, {len(walkers)} walkers near you)')
except KeyboardInterrupt:
    pass
finally:
    print('removing NPCs ...')
    def _cleanup():
        try:
            for c in ctrls:
                if c.is_alive: c.stop()
            client.apply_batch_sync([carla.command.DestroyActor(a) for a in ctrls + walkers + cars if a.is_alive], False)   # plain apply_batch is dropped in sync-mode worlds
        except Exception as e: print('cleanup error:', e)
    t = threading.Thread(target=_cleanup, daemon=True); t.start(); t.join(15)
    print('done' if not t.is_alive() else 'cleanup timed out (server gone?)'); os._exit(0)
