"""Convert CARLA Leaderboard 1.0 routes (+ scenario JSON) to the Leaderboard 2.0 XML format used by
Bench2Drive's vendored leaderboard, so TransFuser-era benchmarks (Town05 Long/Short, Longest6, LAV) run here.

  python convert_lb1_routes.py ROUTES_1.0.xml SCENARIOS_1.0.json OUT_2.0.xml [--min-spacing 50] [--max-per-route N]

Needs a running CARLA server (bash carla_server.sh): the 2.0 route planner is used to densify each route
(the server switches maps as needed). Scenario placement follows leaderboard 1.0's rules: a scenario config
matches the route if its trigger is within 2 m / 10 deg of a route point; junction scenarios are only valid
where the route actually turns that way; at each trigger the highest-numbered valid scenario wins.
APPROXIMATION: 1.0's "all_scenarios" JSONs list every type at every road point, which 1.0 then thins out by
spawn failures at run time. Here --min-spacing (metres along the route) thins them deterministically instead.
"""
import argparse, json, math, sys, time
import numpy as np
import xml.etree.ElementTree as ET
sys.path[:0] = ['/home/all/carla_ws/Bench2Drive/leaderboard', '/home/all/carla_ws/Bench2Drive/scenario_runner', '/home/all/carla_ws/carla/PythonAPI/carla']
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from leaderboard.utils.route_manipulation import interpolate_trajectory
from agents.navigation.local_planner import RoadOption

ap = argparse.ArgumentParser()
ap.add_argument('routes'); ap.add_argument('scenarios'); ap.add_argument('out')
ap.add_argument('--min-spacing', type=float, default=50.0, help='min metres between consecutive scenarios on a route')
ap.add_argument('--max-per-route', type=int, default=9999)
ap.add_argument('--sampling', choices=['prioritized', 'random'], default='prioritized',
                help="per trigger: 1.0's highest-number-wins, or seeded random among valid candidates (better for dense 'all_scenarios' JSONs)")
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--junction-lookahead', type=float, default=40.0, help='metres ahead within which a junction must start for junction scenarios')
ap.add_argument('--port', type=int, default=2000)
args = ap.parse_args()

# 1.0 scenario number (+ subtype from route geometry) -> 2.0 scenario type (+ extra params)
def map_type(num, subtype):
    if num == 'Scenario1': return 'ControlLoss', {}
    if num == 'Scenario3': return 'DynamicObjectCrossing', {}
    if num == 'Scenario4': return 'VehicleTurningRoute', {}
    if num == 'Scenario7': return 'OppositeVehicleRunningRedLight', {}
    if num == 'Scenario8': return 'SignalizedJunctionLeftTurn', {}
    if num == 'Scenario9': return 'SignalizedJunctionRightTurn', {}
    if num == 'Scenario10': return 'OppositeVehicleTakingPriority', {'direction': 'right'}
    return None, {}

def subtype_for(num, idx, route, cum, cmap, world, d=None):
    """Leaderboard 1.0 get_scenario_type (route must turn the right way next) PLUS a geometric check the 1.0
    scenario classes did at run time: a junction must begin within --junction-lookahead metres, signalized for
    Scenario7/8/9 and unsignalized for Scenario10."""
    if num in ('Scenario1', 'Scenario3'): return 'valid'
    # the 2.0 scenario classes locate themselves with the LAST route point closest to the trigger (helper uses <=);
    # if the route passes this spot twice that is the later pass, so judge the junction from there
    if d is not None: idx = int(np.where(d <= d.min() + 1e-6)[0][-1])
    turn, junction = None, None
    for j in range(idx, len(route)):
        if cum[j] - cum[idx] > args.junction_lookahead: break
        wp = cmap.get_waypoint(route[j][0].location)
        if wp.is_junction:
            junction = wp.get_junction(); break
        opt = route[j][1]
        if opt not in (RoadOption.LANEFOLLOW, RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT) and turn is None: turn = opt
    if junction is None: return None
    for j in range(idx, len(route)):  # the RoadOption describing this junction is set on the waypoints entering it
        opt = route[j][1]
        if opt not in (RoadOption.LANEFOLLOW, RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT): turn = opt; break
        if cum[j] - cum[idx] > args.junction_lookahead + 30: break
    # the 2.0 junction scenarios need the route to EXIT the junction (back to LANEFOLLOW) with road left after it
    j1 = next((j for j in range(idx, len(route)) if route[j][1] in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT)), None)
    if j1 is None: return None
    j2 = next((j for j in range(j1, len(route)) if route[j][1] not in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT)), None)
    if j2 is None or cum[-1] - cum[j2] < 15: return None
    signalized = len(world.get_traffic_lights_in_junction(junction.id)) > 0
    if num == 'Scenario4': return 'valid' if turn in (RoadOption.LEFT, RoadOption.RIGHT) else None
    if num == 'Scenario7': return 'valid' if signalized and turn in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT) else None
    if num == 'Scenario8': return 'valid' if signalized and turn == RoadOption.LEFT else None
    if num == 'Scenario9': return 'valid' if signalized and turn == RoadOption.RIGHT else None
    if num == 'Scenario10': return 'valid' if (not signalized) and turn in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT) else None
    return None

import random
rng = random.Random(args.seed)
client = carla.Client('localhost', args.port); client.set_timeout(300.0)
CarlaDataProvider.set_client(client)
annotations = json.load(open(args.scenarios))['available_scenarios'][0]
src = ET.parse(args.routes).getroot()
out = ET.Element('routes')
total = 0
for r in src.iter('route'):
    rid, town = r.get('id'), r.get('town')
    world = client.get_world()
    if world.get_map().name.split('/')[-1] != town:
        print(f'loading {town} ...', flush=True); client.load_world(town); time.sleep(2); world = client.get_world()
    CarlaDataProvider.set_world(world); cmap = world.get_map()
    kps = [carla.Location(x=float(w.get('x')), y=float(w.get('y')), z=float(w.get('z'))) for w in r.iter('waypoint')]
    _, route = interpolate_trajectory(kps)                      # [(carla.Transform, RoadOption)], 1 m spacing
    rx = np.array([[t.location.x, t.location.y, t.location.z, t.rotation.yaw] for t, _ in route])
    cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(rx[:, :2], axis=0), axis=1))])
    # match every 1.0 scenario config to the route (2 m, 10 deg); collect valid candidates per trigger
    cands = {}  # route index -> list of (num, transform)
    subtype_cache = {}
    for sc in annotations.get(town, []):
        num = sc['scenario_type']
        for ev in sc['available_event_configurations']:
            tf = ev['transform']; x, y, z, yaw = float(tf['x']), float(tf['y']), float(tf['z']), float(tf['yaw'])
            d = np.sqrt((rx[:, 0] - x) ** 2 + (rx[:, 1] - y) ** 2 + (rx[:, 2] - z) ** 2)
            dyaw = (yaw - rx[:, 3]) % 360
            ok = np.where((d < 2.0) & ((dyaw < 10) | (dyaw > 350)))[0]
            if len(ok) == 0: continue
            idx = int(ok[0])
            if (num, idx) not in subtype_cache: subtype_cache[(num, idx)] = subtype_for(num, idx, route, cum, cmap, world, d)
            if subtype_cache[(num, idx)] is None: continue
            cands.setdefault(idx, []).append((num, (x, y, z, yaw)))
    triggers = {}
    for idx, lst in cands.items():
        if args.sampling == 'prioritized':
            num, tf = max(lst, key=lambda c: int(c[0].replace('Scenario', '')))
        else:  # junction scenarios are rarer and more interesting: always prefer one if available, else random
            jn = [c for c in lst if c[0] not in ('Scenario1', 'Scenario3')]
            num, tf = rng.choice(jn) if jn else rng.choice(lst)
        triggers[idx] = (num, tf, 'valid')
    # thin by spacing along the route (1.0 relied on run-time spawn failures for this)
    chosen, last_s = [], -1e9
    for idx in sorted(triggers):
        if cum[idx] - last_s < args.min_spacing: continue
        chosen.append((idx, *triggers[idx])); last_s = cum[idx]
        if len(chosen) >= args.max_per_route: break
    # write 2.0 route
    o = ET.SubElement(out, 'route', id=rid, town=town)
    wps = ET.SubElement(o, 'waypoints')
    for k in kps: ET.SubElement(wps, 'position', x=f'{k.x:.2f}', y=f'{k.y:.2f}', z=f'{k.z:.2f}')
    scs = ET.SubElement(o, 'scenarios'); counts = {}
    for idx, num, (x, y, z, yaw), st in chosen:
        t2, params = map_type(num, st); counts[t2] = counts.get(t2, 0) + 1
        s = ET.SubElement(scs, 'scenario', name=f'{t2}_{counts[t2]}', type=t2)
        ET.SubElement(s, 'trigger_point', x=f'{x:.2f}', y=f'{y:.2f}', z=f'{z:.2f}', yaw=f'{yaw:.1f}')
        for k, v in params.items(): ET.SubElement(s, k, value=v)
    ws = ET.SubElement(o, 'weathers')
    src_w = list(r.iter('weather'))
    keys = ('cloudiness', 'precipitation', 'precipitation_deposits', 'wetness', 'wind_intensity', 'sun_azimuth_angle', 'sun_altitude_angle', 'fog_density')
    base = {k: src_w[0].get(k) for k in keys if src_w and src_w[0].get(k) is not None} if src_w else {'cloudiness': '10', 'precipitation': '0', 'precipitation_deposits': '0', 'wetness': '0', 'wind_intensity': '10', 'sun_azimuth_angle': '45', 'sun_altitude_angle': '60', 'fog_density': '2'}
    for pct in ('0', '100'): ET.SubElement(ws, 'weather', route_percentage=pct, **base)
    total += len(chosen)
    print(f'route {rid:>4} {town:7s} {cum[-1]/1000:5.2f} km  {len(triggers):4d} candidate triggers -> {len(chosen):3d} scenarios  {counts}', flush=True)
ET.indent(out); ET.ElementTree(out).write(args.out, encoding='UTF-8', xml_declaration=True)
print(f'wrote {args.out}: {len(list(out))} routes, {total} scenarios')
