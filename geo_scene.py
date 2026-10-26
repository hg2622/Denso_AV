"""Drive a real-world site reconstructed from map data (East Liberty Target, Pittsburgh by default):
  python geo_scene.py [--site geo_scenes/eastliberty] [--cars 30] [--walkers 60] [--test --snapshot out.png]
Scale-faithful parts (metres from OSM / UTM projection, same frame as the satellite image and the SAM3 detections):
  roads      : site.xodr (OSM -> OpenDRIVE via carla.Osm2Odr), generated live
  buildings  : every OSM building footprint, built from CARLA's own cooked building meshes (mall / apartment / church / house ...)
               chosen by OSM tags + size, uniformly scaled and tiled so the footprint and the height (building:levels) are kept;
               irregular footprints are filled cell by cell so the outline is followed
  trees      : SAM3 'tree' detections on the satellite image -> maple / beech / acer / pine meshes scaled to the detected crown
  parked cars: SAM3 'car' detections (off the driving lanes) -> CARLA vehicles at the detected pose
  shop fronts: OSM shop / amenity buildings get awnings, benches, vending machines, planters, ads, carts (supermarkets) at the street-side facade
  bus stops / lamps / bins : OSM points and the main roads
NPC cars drive under the Traffic Manager; pedestrians walk the OSM footways. HUD: chase cam + satellite minimap with your position.
You drive the ego (W/S A/D, Shift gentle, Q reverse, Space handbrake, P autopilot, ESC quit); you start in the Target parking lot.
Needs the CARLA server on port 2000 (Epic quality recommended)."""
import argparse, json, math, os, random, sys, time, xml.etree.ElementTree as ET
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
from pyproj import Proj
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity
ap = argparse.ArgumentParser(); ap.add_argument('--site', default='/home/all/carla_ws/geo_scenes/eastliberty'); ap.add_argument('--cars', type=int, default=30); ap.add_argument('--walkers', type=int, default=60)
ap.add_argument('--parked-stride', type=int, default=1); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--no-buildings', action='store_true'); ap.add_argument('--no-marks', action='store_true', help='skip the painted lane dashes and parking-bay lines'); ap.add_argument('--paint', action='store_true', help='draw lane/bay paint as debug lines (they render white-hot in most lighting, so off by default)'); ap.add_argument('--test', action='store_true'); ap.add_argument('--snapshot', default=''); a = ap.parse_args()
rng = random.Random(a.seed or None); SITE = a.site
sat_meta = json.load(open(f'{SITE}/sat.json')); ASSETS = json.load(open('/home/all/carla_ws/geo_scenes/geo_assets.json'))
LON0 = sat_meta['lon0']; proj = Proj(proj='utm', zone=int((LON0 + 180) // 6) + 1, datum='WGS84'); e0, n0 = proj(sat_meta['lon0'], sat_meta['lat0'])
def ll2c(lat, lon):  # -> CARLA frame (x east, y south), metres, same as site.xodr / detections.json
    e, n = proj(lon, lat); return e - e0, -(n - n0)

client = carla.Client('localhost', a.port); client.set_timeout(300.0)
print('generating the road network from site.xodr ...', flush=True)
XODR = f'{SITE}/site_lots.xodr' if os.path.exists(f'{SITE}/site_lots.xodr') else f'{SITE}/site.xodr'
world = client.generate_opendrive_world(open(XODR).read(), carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = 0.05; world.apply_settings(st)
world.set_weather(carla.WeatherParameters(cloudiness=20.0, sun_altitude_angle=50.0, sun_azimuth_angle=200.0, fog_density=1.0))
tm = client.get_trafficmanager(a.port + 6000); tm.set_synchronous_mode(True); tm.set_global_distance_to_leading_vehicle(4.0); tm.global_percentage_speed_difference(-10.0)
lib = world.get_blueprint_library(); cmap = world.get_map(); world.tick()

# ---------------- OSM parse (buildings, footways, roads, points) ----------------
sys.path.insert(0, '/home/all/carla_ws/geo_scenes'); from osmutil import parse_osm
ways, nodes, node_tags = parse_osm(f'{SITE}/osm.xml', ll2c)
buildings = [(t, Polygon(p)) for t, p in ways if 'building' in t and len(p) >= 4 and Polygon(p).is_valid and Polygon(p).area > 15]
parts = [(t, Polygon(p)) for t, p in ways if 'building:part' in t and 'building' not in t and len(p) >= 4 and Polygon(p).is_valid and Polygon(p).area > 15]
footways = [p for t, p in ways if t.get('highway') in ('footway', 'path', 'pedestrian', 'steps') and t.get('footway') != 'crossing']
main_roads = [p for t, p in ways if t.get('highway') in ('primary', 'secondary', 'tertiary', 'residential')]
grass = [Polygon(p) for t, p in ways if t.get('landuse') == 'grass' and len(p) >= 4 and Polygon(p).is_valid]
bld_union = [b for _, b in buildings]
def in_building(x, y, margin=0.0):
    pt = Point(x, y); return any(b.distance(pt) < margin if margin else b.contains(pt) for b in bld_union)
def on_road(x, y, margin=0.6):   # inside a driving lane of the generated map?
    wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.5), project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None: return False
    return wp.transform.location.distance(carla.Location(x=float(x), y=float(y), z=wp.transform.location.z)) < wp.lane_width / 2 + margin
def road_dist(x, y):
    wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.5), project_to_road=True, lane_type=carla.LaneType.Driving)
    return 1e9 if wp is None else wp.transform.location.distance(carla.Location(x=float(x), y=float(y), z=wp.transform.location.z)) - wp.lane_width / 2

# ---------------- props ----------------
props = []; mesh_bp = lib.find('static.prop.mesh')
def spawn_mesh(name, x, y, yaw, scale, z=0.0):
    """Place a cooked static mesh so that its bounding-box footprint centre is at (x, y) and its base sits at z.
    CARLA's bounding-box CENTRE OFFSET is reported in unscaled actor space and does not scale with the `scale` attribute,
    while the box SIZE does. Scaling the offset (as a naive placement does) sinks or floats every scaled piece - which put
    building modules through the ground and across the garage aisles - so the offset is applied unscaled."""
    A = ASSETS[name]; cx, cy, cz = A['center']; sz = A['size'][2]
    if not all(math.isfinite(v) for v in (x, y, yaw, scale, z)) or scale < 0.05: return None   # degenerate footprint -> never send nan/0 to the server
    r = math.radians(yaw)                                                        # bbox centre offset: unscaled, in the actor frame
    wx = x - (cx * math.cos(r) - cy * math.sin(r)); wy = y - (cx * math.sin(r) + cy * math.cos(r))
    wz = z - (cz - sz * scale / 2)                                               # size scales, offset does not
    mesh_bp.set_attribute('mesh_path', A['path']); mesh_bp.set_attribute('scale', f'{scale:.3f}')
    p = world.try_spawn_actor(mesh_bp, carla.Transform(carla.Location(x=float(wx), y=float(wy), z=float(wz)), carla.Rotation(yaw=float(yaw))))
    if p is not None: p.set_simulate_physics(False); props.append(p)
    return p
def spawn_prop(bpid, x, y, yaw, z=0.0):
    if not all(math.isfinite(v) for v in (x, y, yaw, z)): return None
    try: bp = lib.find('static.prop.' + bpid)
    except IndexError: return None
    p = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(yaw))))
    if p is not None: p.set_simulate_physics(False); props.append(p)
    return p
# ground: sunk 100 m cubes under the whole site
mesh_bp.set_attribute('mesh_path', '/Engine/BasicShapes/Cube.Cube'); mesh_bp.set_attribute('scale', '100')
for x in range(-450, 551, 100):
    for y in range(-450, 551, 100):
        p = world.try_spawn_actor(mesh_bp, carla.Transform(carla.Location(x=float(x), y=float(y), z=-50.01)))   # ground slab top at -1 cm; every road and lot lane is at exactly 0
        if p is not None: p.set_simulate_physics(False); props.append(p)
world.tick()

# ---------------- buildings from OSM footprints with CARLA building meshes ----------------
LEVEL_H = 3.4; HEIGHTS = json.load(open(f'{SITE}/heights.json')) if os.path.exists(f'{SITE}/heights.json') else {}
def target_height(t, poly=None):
    if t.get('height'):
        try: return float(t['height'].split()[0])
        except ValueError: pass
    if t.get('building:levels'):
        try: return float(t['building:levels']) * LEVEL_H
        except ValueError: pass
    key = t.get('name') if t.get('name') in HEIGHTS else (f'osm@{poly.centroid.x:.0f},{poly.centroid.y:.0f}' if poly is not None else None)
    if key in HEIGHTS: return HEIGHTS[key]
    return {'retail': 8.0, 'commercial': 9.0, 'apartments': 15.0, 'church': 14.0, 'house': 7.5, 'residential': 8.0, 'parking': 12.0, 'roof': 3.5, 'construction': 6.0, 'garage': 3.5}.get(t.get('building'), 8.0)
def pick_mesh(t, L, W, h):
    kind = t.get('building'); amen = t.get('amenity', ''); shop = t.get('shop', '')
    if L * W > 600 and kind not in ('church', 'house', 'residential', 'apartments', 'garage', 'roof', 'parking') and amen != 'place_of_worship': return 'BLOCK'   # modular storeys: solid, exact footprint, right height
    if kind == 'church' or amen == 'place_of_worship': return 'SM_Church01'
    if kind == 'roof': return 'SM_Garage01'
    if kind == 'parking': return 'BLOCK'   # parking structures as storey modules (the SM_Parking mesh has an off-centre bbox and overshoots)
    if kind == 'garage' or (L < 9 and W < 9 and h < 6): return 'SM_Garage01'
    if h >= 25: return 'SM_Skycraper01'
    if kind == 'apartments' or h >= 13: return rng.choice(['SM_Apartment01_1', 'SM_Bloc04', 'SM_Block08'])
    if kind in ('house', 'residential'): return rng.choice(['SM_House01', 'SM_TerracedHouse_1', 'SM_Mansion01'])
    if (shop == 'supermarket' or kind == 'retail') and L * W > 2500 and W > 28: return 'SM_GasStation01'   # big box only: low flat-roof strip (its bbox includes a canopy)
    if kind == 'commercial' or shop or amen: return rng.choice(['SM_Apartment02_1', 'SM_TerracedHouse_1', 'SM_House06'])
    return rng.choice(['SM_Apartment02_1', 'SM_Apartment01_1', 'SM_TerracedHouse_1', 'SM_Block08'])
def place_block(cx, cy, yaw, L, W, h, z=0.0):
    """Solid building from SM_Block03Floor storey modules: module scale from footprint and height, exact width, modules overlap along the
    length so there are no gaps, stacked to the target height."""
    sc = max(1.5, min(6.0, max(math.sqrt(L * W) / 45, h / 110))); mL0, mW0, mH0 = 7.69, 5.1, 2.57
    m = max(1, round(W / (sc * mW0))); s = W / (m * mW0); sw = W / m
    n = max(1, math.ceil(L / (s * mL0) - 0.05)); step = (L - s * mL0) / (n - 1) if n > 1 else 0.0
    floors = max(1, round(h / (mH0 * s))); r = math.radians(yaw); k = 0
    for i in range(n):
        for j in range(m):
            u = -L / 2 + s * mL0 / 2 + step * i; v = -W / 2 + sw * (j + 0.5); x, y = cx + u * math.cos(r) - v * math.sin(r), cy + u * math.sin(r) + v * math.cos(r)
            if blocks_lane(x, y, z): continue   # a road runs here: leave the passage open
            for f in range(floors):
                if spawn_mesh('SM_Block03Floor', x, y, yaw + 90, s, z=z + f * mH0 * s) is not None: k += 1
    return k
def place_ring(poly, h, z=0.0, sc=None):
    """Irregular footprint: storey modules laid along every edge of the exact polygon (windows outward, overlapping, no staircase),
    the interior filled cell by cell inside the ring. Returns number of pieces."""
    mL0, mW0, mH0 = 7.69, 5.1, 2.57; sc = sc or max(1.5, min(4.0, max(math.sqrt(poly.area) / 45, h / 110))); s = sc
    floors = max(1, round(h / (mH0 * s))); k = 0; cs = list(poly.exterior.coords); inward = 1.0 if poly.exterior.is_ccw else -1.0
    for (x1, y1), (x2, y2) in zip(cs, cs[1:]):
        e = math.hypot(x2 - x1, y2 - y1)
        if e < 1.0: continue
        ux, uy = (x2 - x1) / e, (y2 - y1) / e; nx, ny = -uy * inward, ux * inward   # left normal for ccw rings points inward
        if not poly.contains(Point((x1 + x2) / 2 + nx * 2.0, (y1 + y2) / 2 + ny * 2.0)): nx, ny = -nx, -ny
        if e < 0.6 * mL0 * s: continue   # edge shorter than a module: the neighbours and the interior cover it (a module here would overshoot)
        n = max(1, math.ceil(e / (mL0 * s * 0.92))); step = (e - mL0 * s) / (n - 1) if n > 1 else 0.0; yaw = math.degrees(math.atan2(uy, ux))
        for i in range(n):
            d = min(mL0 * s / 2 + step * i, e - mL0 * s / 2) if e > mL0 * s else e / 2
            x, y = x1 + ux * d + nx * mW0 * s / 2, y1 + uy * d + ny * mW0 * s / 2
            if blocks_lane(x, y, z): continue   # a road enters here: leave the passage open
            for f in range(floors):
                if spawn_mesh('SM_Block03Floor', x, y, yaw + 90, s, z=z + f * mH0 * s) is not None: k += 1
    inner = poly.buffer(-mW0 * s * 0.9)
    if not inner.is_empty:
        for part in (inner.geoms if inner.geom_type != 'Polygon' else [inner]):
            if part.area < 30: continue
            mrr = part.minimum_rotated_rectangle; c4 = list(mrr.exterior.coords)[:4]; e1 = math.hypot(c4[1][0] - c4[0][0], c4[1][1] - c4[0][1]); e2 = math.hypot(c4[2][0] - c4[1][0], c4[2][1] - c4[1][1])
            L, W, yaw = (e1, e2, math.degrees(math.atan2(c4[1][1] - c4[0][1], c4[1][0] - c4[0][0]))) if e1 >= e2 else (e2, e1, math.degrees(math.atan2(c4[2][1] - c4[1][1], c4[2][0] - c4[1][0])))
            cx, cy = mrr.centroid.x, mrr.centroid.y; r = math.radians(yaw); local = affinity.rotate(affinity.translate(part, -cx, -cy), -yaw, origin=(0, 0))
            cell = mL0 * s; nx_, ny_ = max(1, round(L / cell)), max(1, round(W / cell)); cl, cw = L / nx_, W / ny_
            for i in range(nx_):
                for j in range(ny_):
                    u, v = -L / 2 + cl * (i + 0.5), -W / 2 + cw * (j + 0.5)
                    cellp = Polygon([(u - cl / 2, v - cw / 2), (u + cl / 2, v - cw / 2), (u + cl / 2, v + cw / 2), (u - cl / 2, v + cw / 2)])
                    if local.intersection(cellp).area < 0.3 * cl * cw: continue
                    if z == 0 and any(cellp.buffer(1.0).contains(Point((px_ - cx) * math.cos(r) + (py_ - cy) * math.sin(r), -(px_ - cx) * math.sin(r) + (py_ - cy) * math.cos(r))) for px_, py_ in road_pts): continue
                    k += place_block(cx + u * math.cos(r) - v * math.sin(r), cy + u * math.sin(r) + v * math.cos(r), yaw, cl, cw, h, z)
    return k
def place_box(name, cx, cy, yaw, L, W, h, z=0.0):
    if name == 'BLOCK': return place_block(cx, cy, yaw, L, W, h, z)
    """Fill an L x W (along yaw) box of height h with uniformly scaled copies of a mesh: the scale follows the target height
    (clamped so one copy is never wider than the box), copies are tiled in a grid so the footprint is covered exactly."""
    mL, mW, mH = ASSETS[name]['size']
    if mL < mW: mL, mW = mW, mL; yaw_m = yaw + 90
    else: yaw_m = yaw
    s = max(0.3, min(h / mH, W / mW, L / mL)); s = max(s, min(W / mW, L / mL) * 0.5)
    n, m = max(1, round(L / (s * mL))), max(1, round(W / (s * mW))); sl, sw = L / n, W / m
    s = min(s, sl / mL, sw / mW)
    r = math.radians(yaw); k = 0; floors = max(1, round(h / (s * mH)))   # towers: stack copies until the target height is reached
    for i in range(n):
        for j in range(m):
            u, v = -L / 2 + sl * (i + 0.5), -W / 2 + sw * (j + 0.5)
            bx, by = cx + u * math.cos(r) - v * math.sin(r), cy + u * math.sin(r) + v * math.cos(r)
            if blocks_lane(bx, by, z): continue   # a road runs here: leave the passage open
            for f in range(floors):
                if spawn_mesh(name, bx, by, yaw_m, s, z=z + f * s * mH) is not None: k += 1
    return k
BUILD_CLEAR = 1.0   # metres of clearance a ground-level building piece keeps from any drivable lane centre-line edge
def blocks_lane(x, y, z=0.0, half=4.0):
    """True if a building piece whose base is at z would stand on a lane cars drive on (roads, aisles and lot lanes)."""
    if z > 3.0: return False   # above a garage deck: meant to span the aisles
    wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.3), project_to_road=True, lane_type=carla.LaneType.Driving | carla.LaneType.Parking)
    if wp is None: return False
    d = math.hypot(wp.transform.location.x - x, wp.transform.location.y - y)
    return d < wp.lane_width / 2 + BUILD_CLEAR + half * 0.0
road_pts = [(wp.transform.location.x, wp.transform.location.y) for wp in cmap.generate_waypoints(5.0)]
def roads_inside(poly): return sum(1 for x, y in road_pts if poly.contains(Point(x, y)))
ELEV_Z = 5.0; SLAB_T = 5.14; columns = []   # ground-level garage clearance and deck thickness under an elevated store
def elevated_structure(poly, yaw):
    """Slab (SM_Plane turned flat, tiled) on columns (SM_ColumnTunnel) over the footprint; roads underneath stay drivable."""
    mrr = poly.minimum_rotated_rectangle; cx, cy = mrr.centroid.x, mrr.centroid.y; r = math.radians(yaw)
    local = affinity.rotate(affinity.translate(poly, -cx, -cy), -yaw, origin=(0, 0)); minu, minv, maxu, maxv = local.bounds
    sc = 2.0; tu, tv = 5.1 * sc, 7.69 * sc; k = 0   # SM_Block03Floor: one storey module of a block building -> solid garage deck tiles
    u = minu
    while u < maxu:
        v = minv
        while v < maxv:
            cell = Polygon([(u, v), (u + tu, v), (u + tu, v + tv), (u, v + tv)])
            if local.intersection(cell).area > 0.3 * tu * tv:
                lu, lv = u + tu / 2, v + tv / 2
                if spawn_mesh('SM_Block03Floor', cx + lu * math.cos(r) - lv * math.sin(r), cy + lu * math.sin(r) + lv * math.cos(r), yaw, sc, z=ELEV_Z) is not None: k += 1
            v += tv
        u += tu
    cs = 0.7; u = minu + 6
    while u < maxu:
        v = minv + 6
        while v < maxv:
            x, y = cx + u * math.cos(r) - v * math.sin(r), cy + u * math.sin(r) + v * math.cos(r)
            if local.contains(Point(u, v)) and not on_road(x, y, 1.2): spawn_mesh('SM_ColumnTunnel', x, y, yaw, cs); columns.append((x, y)); k += 1
            v += 14
        u += 14
    return k
n_b = 0; elevated = []
if not a.no_buildings:
    polys = [p for _, p in buildings]
    def part_base(t):
        if t.get('min_height'):
            try: return float(t['min_height'].split()[0])
            except ValueError: pass
        if t.get('building:min_level'):
            try: return float(t['building:min_level']) * LEVEL_H
            except ValueError: pass
        return 0.0
    jobs = []
    for t, poly in buildings:
        inside = [(pt, pp) for pt, pp in parts if pp.intersection(poly).area > 0.5 * pp.area]
        if inside and sum(pp.area for _, pp in inside) > 0.3 * poly.area:   # podium + towers mapped as building:part: outline = podium up to the lowest tower base
            bases = [part_base(pt) for pt, _ in inside]; tops = [target_height(pt, pp) for pt, pp in inside]
            podium = min([b for b in bases if b > 0] or [min(tops)]); jobs.append((dict(t, building=t.get('building'), _podium='1'), poly, 0.0, max(LEVEL_H, podium)))
            for pt, pp in inside: jobs.append((dict(pt, building=pt.get('building:part') if pt.get('building:part') not in ('yes', None) else t.get('building', 'yes')), pp, part_base(pt), target_height(pt, pp)))
        else: jobs.append((t, poly, 0.0, target_height(t, poly)))
    for t, poly, zb, h in jobs:
        mrr = poly.minimum_rotated_rectangle; coords = list(mrr.exterior.coords)[:4]
        e1 = math.hypot(coords[1][0] - coords[0][0], coords[1][1] - coords[0][1]); e2 = math.hypot(coords[2][0] - coords[1][0], coords[2][1] - coords[1][1])
        if e1 >= e2: L, W, yaw = e1, e2, math.degrees(math.atan2(coords[1][1] - coords[0][1], coords[1][0] - coords[0][0]))
        else: L, W, yaw = e2, e1, math.degrees(math.atan2(coords[2][1] - coords[1][1], coords[2][0] - coords[1][0]))
        cx, cy = mrr.centroid.x, mrr.centroid.y
        if zb == 0 and '_podium' not in t and 'building:part' not in t and any(q is not poly and q.area > poly.area and poly.intersection(q).area > 0.6 * poly.area for q in polys): continue   # footprint nested in a bigger one (garage under the store)
        if t.get('location') == 'underground' or t.get('layer', '0').lstrip('-').isdigit() and int(t.get('layer', '0') or 0) < 0 or t.get('building:levels') == '0': continue   # underground (metro stations ...)
        if t.get('building') == 'roof' and poly.area < 80: spawn_prop('pergola', cx, cy, yaw); continue
        name = pick_mesh(t, L, W, h); z = zb
        if zb > 0: n_in = 0
        n_in = roads_inside(poly); lifted = (t.get('layer', '0').isdigit() and int(t.get('layer', '0')) >= 1) or t.get('building:min_level') or t.get('min_height')
        if n_in >= 6 and lifted and zb == 0:   # store built over a ground-level garage: roads run under it
            z = ELEV_Z + SLAB_T; n_b += elevated_structure(poly, yaw); elevated.append(poly); print(f"  elevated: {t.get('name', t.get('building'))} ({roads_inside(poly)} road samples under it)", flush=True)
        hh = max(LEVEL_H, h - zb)   # pieces above the base
        if (poly.area / mrr.area > 0.78 or L < 14) and (z > 0 or n_in == 0):   # rectangular enough: one tiled grid
            n_b += place_box(name, cx, cy, yaw, L, W, hh, z)
        elif name == 'BLOCK' and poly.geom_type == 'Polygon' and z == 0:   # irregular large building: modules along the exact outline + filled interior
            n_b += place_ring(poly, hh, z)
        else:                                       # irregular: fill the footprint cell by cell in the rectangle's frame
            cell = max(6.0, min(14.0, W / max(1, round(W / 12.0)))); r = math.radians(yaw)
            local = affinity.rotate(affinity.translate(poly, -cx, -cy), -yaw, origin=(0, 0))
            nx, ny = max(1, round(L / cell)), max(1, round(W / cell)); cl, cw = L / nx, W / ny
            for i in range(nx):
                for j in range(ny):
                    u, v = -L / 2 + cl * (i + 0.5), -W / 2 + cw * (j + 0.5)
                    cellp = Polygon([(u - cl / 2, v - cw / 2), (u + cl / 2, v - cw / 2), (u + cl / 2, v + cw / 2), (u - cl / 2, v + cw / 2)])
                    if local.intersection(cellp).area < 0.72 * cl * cw: continue   # a cell mostly outside the outline would overshoot it
                    if z == 0 and n_in and any(cellp.contains(Point((px_ - cx) * math.cos(r) + (py_ - cy) * math.sin(r), -(px_ - cx) * math.sin(r) + (py_ - cy) * math.cos(r))) for px_, py_ in road_pts): continue   # a road crosses the footprint (no layer tag): leave that cell open as a passage
                    n_b += place_box(name, cx + u * math.cos(r) - v * math.sin(r), cy + u * math.sin(r) + v * math.cos(r), yaw, cl, cw, hh, z)
    world.tick(); print(f'buildings: {len(buildings)} footprints + {len(parts)} parts -> {n_b} mesh pieces', flush=True)

# ---------------- shop fronts, bus stops, lamps ----------------
def facade_toward_road(poly):
    """Return (point, outward yaw) of the footprint edge midpoint closest to a driving lane, else None."""
    best = None; cs = list(poly.exterior.coords)
    for (x1, y1), (x2, y2) in zip(cs, cs[1:]):
        e = math.hypot(x2 - x1, y2 - y1)
        if e < 6: continue
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2; nx, ny = (y2 - y1) / e, -(x2 - x1) / e
        if poly.contains(Point(mx + nx * 1.5, my + ny * 1.5)): nx, ny = -nx, -ny
        d = road_dist(mx + nx * 2.0, my + ny * 2.0)
        if best is None or d < best[0]: best = (d, (x1, y1), (x2, y2), (nx, ny))
    if best is None or best[0] > 18: return None
    return best
SHOP_SETS = {'supermarket': ['shoppingcart', 'shoppingcart', 'shoppingtrolley', 'advertisement', 'vendingmachine', 'bin', 'plantpot04'], 'restaurant': ['plastictable', 'plasticchair', 'plasticchair', 'plantpot06', 'advertisement'],
             'cafe': ['plastictable', 'plasticchair', 'plantpot03', 'advertisement'], 'pub': ['bench01', 'plantpot05', 'advertisement', 'bin'], 'bank': ['atm', 'bench02', 'plantpot01'], 'fast_food': ['vendingmachine', 'bin', 'advertisement'],
             'clothes': ['advertisement', 'plantpot02', 'bench03'], 'default': ['advertisement', 'bench01', 'plantpot07', 'trashcan01']}
n_shop = 0
for t, poly in buildings:
    key = t.get('shop') or t.get('amenity') or ('default' if t.get('name') and t.get('building') in ('retail', 'commercial') else None)
    if key is None: continue
    f = facade_toward_road(poly)
    if f is None: continue
    _, (x1, y1), (x2, y2), (nx, ny) = f; e = math.hypot(x2 - x1, y2 - y1); ux, uy = (x2 - x1) / e, (y2 - y1) / e; yaw_out = math.degrees(math.atan2(ny, nx))
    items = SHOP_SETS.get(key, SHOP_SETS['default']); n_items = min(len(items), max(2, int(e / 6)))
    for i in range(n_items):
        d = -e / 2 + (e / (n_items + 1)) * (i + 1); px, py = (x1 + x2) / 2 + ux * d + nx * 1.6, (y1 + y2) / 2 + uy * d + ny * 1.6
        if on_road(px, py, 0.3): continue
        if spawn_prop(items[i % len(items)], px, py, yaw_out + rng.uniform(-10, 10)) is not None: n_shop += 1
    for i in range(max(1, int(e / 9))):   # awnings above the facade
        d = -e / 2 + (e / (max(1, int(e / 9)) + 1)) * (i + 1); sc = 1.8 if key in ('supermarket',) else 1.3
        spawn_mesh('SM_Awning02', (x1 + x2) / 2 + ux * d + nx * 1.0, (y1 + y2) / 2 + uy * d + ny * 1.0, yaw_out - 90, sc, z=2.8)
n_pt = 0
for nid, t in node_tags.items():
    if nid not in nodes: continue
    x, y = nodes[nid]
    if t.get('highway') == 'bus_stop':
        wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.5), project_to_road=True)
        if wp is None or on_road(x, y, 0.5): continue
        if spawn_prop('busstop', x, y, wp.transform.rotation.yaw) is not None: n_pt += 1
    elif t.get('amenity') in ('bench',) and not on_road(x, y):
        if spawn_prop('bench01', x, y, rng.uniform(0, 360)) is not None: n_pt += 1
    elif t.get('amenity') in ('waste_basket', 'post_box') and not on_road(x, y):
        if spawn_prop('trashcan02' if t['amenity'] == 'waste_basket' else 'mailbox', x, y, rng.uniform(0, 360)) is not None: n_pt += 1
    elif t.get('amenity') == 'bicycle_parking' and not on_road(x, y):
        if spawn_mesh('SM_BikeParking', x, y, rng.uniform(0, 360), 1.0) is not None: n_pt += 1
    elif t.get('natural') == 'tree' and not on_road(x, y) and not in_building(x, y):
        spawn_mesh(rng.choice(['SM_Maple_M_v1', 'SM_Acer_02']), x, y, rng.uniform(0, 360), rng.uniform(0.7, 1.0)); n_pt += 1
n_lamp = 0
for pts in main_roads:   # street lamps every ~35 m on both sides of the main roads
    acc = 17.0
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        seg = math.hypot(x2 - x1, y2 - y1)
        if seg < 1e-3: continue
        ux, uy = (x2 - x1) / seg, (y2 - y1) / seg; d = acc
        while d < seg:
            for side in (1, -1):
                wp = cmap.get_waypoint(carla.Location(x=float(x1 + ux * d), y=float(y1 + uy * d), z=0.5), project_to_road=True)
                off = (wp.lane_width * 2 + 1.5) if wp else 8.0
                lx, ly = x1 + ux * d - uy * off * side, y1 + uy * d + ux * off * side
                if on_road(lx, ly, 0.8) or in_building(lx, ly, 0.8): continue
                if spawn_mesh('SM_parkLightV2', lx, ly, math.degrees(math.atan2(uy, ux)) + (90 if side > 0 else -90), 1.0) is not None: n_lamp += 1
            d += 35.0
        acc = d - seg
world.tick(); print(f'shop-front props {n_shop}, OSM point props {n_pt}, lamps {n_lamp}', flush=True)


# ---------------- lane paint (debug lines, persistent): edges, centre lines, crosswalk zebras, parking bays ----------------
WHITE, YELLOW = carla.Color(12, 12, 12), carla.Color(14, 11, 2)   ; n_lines = 0   # debug lines are emissive: dim values render as normal paint; n_lines = 0
DARK = carla.Color(7, 7, 7)
def seg(ax, ay, bx, by, col, th=0.12):
    global n_lines
    if elevated and any(p.contains(Point((ax + bx) / 2, (ay + by) / 2)) for p in elevated): return   # no paint under decks: auto-exposure in the shade makes emissive lines glare
    world.debug.draw_line(carla.Location(x=float(ax), y=float(ay), z=0.06), carla.Location(x=float(bx), y=float(by), z=0.06), thickness=th, color=col, life_time=0.0); n_lines += 1
if a.paint:
    for wp in cmap.generate_waypoints(3.0):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction: continue
        nxt = wp.next(3.0)
        if not nxt: continue
        t0, t1 = wp.transform, nxt[0].transform; r0, r1 = t0.get_right_vector(), t1.get_right_vector(); hw = wp.lane_width / 2
        rl = wp.get_right_lane(); ll = wp.get_left_lane()
        if rl is None or rl.lane_type != carla.LaneType.Driving: seg(t0.location.x + r0.x * hw, t0.location.y + r0.y * hw, t1.location.x + r1.x * hw, t1.location.y + r1.y * hw, WHITE)
        if ll is None or ll.lane_type != carla.LaneType.Driving: seg(t0.location.x - r0.x * hw, t0.location.y - r0.y * hw, t1.location.x - r1.x * hw, t1.location.y - r1.y * hw, WHITE)
        elif ll.lane_id * wp.lane_id < 0 and wp.lane_id < 0: seg(t0.location.x - r0.x * hw, t0.location.y - r0.y * hw, t1.location.x - r1.x * hw, t1.location.y - r1.y * hw, YELLOW)
        elif ll.lane_id * wp.lane_id > 0 and int(wp.s / 3) % 2 == 0: seg(t0.location.x - r0.x * hw, t0.location.y - r0.y * hw, t1.location.x - r1.x * hw, t1.location.y - r1.y * hw, WHITE)
    cw = cmap.get_crosswalks(); grp = []
    for loc in cw:
        if grp and abs(loc.x - grp[0].x) < 1e-3 and abs(loc.y - grp[0].y) < 1e-3:
            if len(grp) >= 4:
                pg = Polygon([(p.x, p.y) for p in grp]); mrr = pg.minimum_rotated_rectangle; c4 = list(mrr.exterior.coords)[:4]
                e1 = math.hypot(c4[1][0] - c4[0][0], c4[1][1] - c4[0][1]); e2 = math.hypot(c4[2][0] - c4[1][0], c4[2][1] - c4[1][1])
                if e1 >= e2: ax_, ay_, bx_, by_ = c4[0], c4[1], c4[3], c4[2]   # long edges a->b and d->c
                else: ax_, ay_, bx_, by_ = c4[1], c4[2], c4[0], c4[3]
                n = max(2, int(max(e1, e2) / 1.3))
                for i in range(n):
                    f = (i + 0.5) / n
                    if i % 2 == 0: seg(ax_[0] + (ay_[0] - ax_[0]) * f, ax_[1] + (ay_[1] - ax_[1]) * f, bx_[0] + (by_[0] - bx_[0]) * f, bx_[1] + (by_[1] - bx_[1]) * f, WHITE, 0.55)
            grp = []
        else: grp.append(loc)
    print(f'lane paint: {n_lines} segments, crosswalks {len(cw)} points', flush=True)

# ---------------- painted markings: a real lane-marking mesh (St_Road_TileRoad_LaneMarkingBroken: ~3 m yellow dash at scale 1) ----------------
DASH_LEN = 3.0; n_dash = 0; _dashes = []
def paint_dash(x, y, yaw, length=3.0, z=0.0):
    global n_dash
    if any(math.hypot(x - dx, y - dy) < 0.6 and min(abs((yaw - dyaw) % 180.0), 180.0 - abs((yaw - dyaw) % 180.0)) < 15.0 for dx, dy, dyaw in _dashes): return   # same line already painted
    _dashes.append((x, y, yaw))
    if spawn_mesh('St_Road_TileRoad_LaneMarkingBroken', x, y, yaw, length / DASH_LEN, z=z + 0.012) is not None: n_dash += 1
def surface_z(x, y):   # every paved surface (roads and lots) is at z = 0
    wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.5), project_to_road=True, lane_type=carla.LaneType.Parking)
    return 0.0
def bay_marks(x, y, yaw, length=4.8, width=2.6):
    r = math.radians(yaw); vx, vy = -math.sin(r), math.cos(r); z = surface_z(x, y)
    for sgn in (1, -1): paint_dash(x + vx * sgn * width / 2, y + vy * sgn * width / 2, yaw, length, z)
lots_poly = [Polygon(p) for t, p in ways if t.get('amenity') == 'parking' and len(p) >= 4 and Polygon(p).is_valid]
if os.path.exists(f'{SITE}/regions.json'):
    lots_poly += [Polygon(r['poly']).buffer(0) for r in json.load(open(f'{SITE}/regions.json'))['regions'].get('parking lot', []) if r['score'] >= 0.3 and r['area_m2'] >= 300]
lots_poly = [p for p in lots_poly if not p.is_empty]
def in_lot(x, y): pt = Point(x, y); return any(p.contains(pt) for p in lots_poly)
if not a.no_marks:   # lane dashes: centre line of two-way roads and separators of same-direction lanes, one 3 m dash every 9 m (not in junctions)
    for wp in cmap.generate_waypoints(9.0):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction or wp.lane_id > 0 and wp.get_left_lane() is not None and wp.get_left_lane().lane_id < 0: continue
        ll = wp.get_left_lane()
        if ll is None or ll.lane_type != carla.LaneType.Driving: continue
        t0 = wp.transform; rv = t0.get_right_vector(); hw = wp.lane_width / 2
        paint_dash(t0.location.x - rv.x * hw, t0.location.y - rv.y * hw, t0.rotation.yaw, 3.0, 0.0)
    print(f'lane dashes: {n_dash}', flush=True)

# ---------------- trees from satellite detections, bushes in grass ----------------
det = json.load(open(f'{SITE}/detections.json'))['detections']; TREES = ['SM_Maple_M_v1', 'SM_Maple_M_v1', 'SM_Acer_02', 'SM_Beech', 'SM_TreePine_1', 'SM_Maple_L_v1']
n_tree = 0
for d in det.get('tree', []):
    if d['score'] < 0.4: continue
    crown = max(2.5, min(16.0, (d['length_m'] + d['width_m']) / 2)); x, y = d['x'], d['y']
    if on_road(x, y, 0.2) or in_building(x, y, 0.5): continue
    name = rng.choice(TREES) if crown > 5 else rng.choice(['SM_Maple_M_v1', 'SM_TreePine_2', 'SM_Acer_02'])
    s = max(0.45, min(1.6, crown / ((ASSETS[name]['size'][0] + ASSETS[name]['size'][1]) / 2)))
    if spawn_mesh(name, x, y, rng.uniform(0, 360), s) is not None: n_tree += 1
n_bush = 0
for g in grass:
    minx, miny, maxx, maxy = g.bounds
    for _ in range(max(1, int(g.area / 120))):
        x, y = rng.uniform(minx, maxx), rng.uniform(miny, maxy)
        if not g.contains(Point(x, y)) or on_road(x, y, 0.5) or in_building(x, y): continue
        if spawn_mesh(rng.choice(['SM_Bush_L_v1', 'SM_Hedge_01']), x, y, rng.uniform(0, 360), rng.uniform(0.7, 1.2)) is not None: n_bush += 1
world.tick(); print(f'trees {n_tree} (from {len(det.get("tree", []))} detections), bushes {n_bush}', flush=True)

# ---------------- cars: parked from detections, ego in the Target lot, Traffic Manager NPCs ----------------
cars_bp = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and not any(k in b.id for k in ('carlacola', 'firetruck', 'ambulance', 'sprinter', 'truck', 'bus', 'cybertruck', 't2', 'isetta', 'microlino'))]
def car_bp(role):
    b = rng.choice(cars_bp)
    if b.has_attribute('color'): b.set_attribute('color', rng.choice(b.get_attribute('color').recommended_values))
    b.set_attribute('role_name', role); return b
target = next((poly for t, poly in buildings if t.get('name') == 'Target'), None); tcx, tcy = (target.centroid.x, target.centroid.y) if target is not None else (0.0, 0.0)
car_det = [d for d in det.get('car', []) if d['score'] >= 0.45 and 3.4 <= d['length_m'] <= 6.6 and d['width_m'] <= 3.8]
def park_yaw(x, y, bay_yaw, nose_in=True):
    """Which way a car in this bay faces.
    Parallel bays (bay aligned with the lane): the car follows the lane's travel direction, so parked cars face the way traffic goes.
    Perpendicular / angled bays: the car is square to the aisle, nose pointing away from it (nose_in) or towards it (reverse-parked).
    With no driving lane within 35 m the bay heading is kept as is."""
    wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.3), project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None: return bay_yaw
    lt = wp.transform.location; lane_yaw = wp.transform.rotation.yaw
    if math.hypot(lt.x - x, lt.y - y) > 35.0: return bay_yaw
    def axis_diff(p, q): e = abs((p - q) % 180.0); return min(e, 180.0 - e)
    if axis_diff(lane_yaw, bay_yaw) < 30.0: return lane_yaw   # parallel parking: travel direction of the lane, whichever way the bay was measured
    ax, ay = lt.x - x, lt.y - y; r = math.radians(bay_yaw)   # vector from the bay towards the aisle
    towards = (math.cos(r) * ax + math.sin(r) * ay) > 0      # does bay_yaw point at the aisle?
    return (bay_yaw + 180.0 if towards else bay_yaw) if nose_in else (bay_yaw if towards else bay_yaw + 180.0)
CLEAR = 1.0   # metres between a parked car / bay and any building outline
def car_ok(x, y): return not on_road(x, y, -0.3) and not in_building(x, y, CLEAR)
def ang_diff(a_, b_): d = abs((a_ - b_) % 180.0); return min(d, 180.0 - d)
PITCH = 2.7; rows = []; bays = []; slot_of = {}   # detection index -> (row, k) once snapped to the grid
lot_dets = [(i, d) for i, d in enumerate(car_det) if car_ok(d['x'], d['y']) and in_lot(d['x'], d['y'])]
# 1. one heading per neighbourhood (detections jitter by ~10 deg), drop the odd one out
heads = {}
for i, d in lot_dets:
    nb = [q['yaw'] for _, q in lot_dets if math.hypot(q['x'] - d['x'], q['y'] - d['y']) < 25.0 and ang_diff(q['yaw'], d['yaw']) < 25.0]
    ref = nb[0]; heads[i] = sorted(((q - ref + 90.0) % 180.0) - 90.0 for q in nb)[len(nb) // 2] + ref
    far = [q for _, q in lot_dets if math.hypot(q['x'] - d['x'], q['y'] - d['y']) < 15.0]
    if len(far) >= 3 and len(nb) < len(far) / 2: heads[i] = None
# 2. rows: same heading, centres on one line across the bays
for i, d in lot_dets:
    if heads[i] is None: continue
    yaw = heads[i]; r = math.radians(yaw); u = (math.cos(r), math.sin(r)); t = d['x'] * u[0] + d['y'] * u[1]
    for row in rows:
        if ang_diff(row['yaw'], yaw) < 6.0 and abs(row['t'] - t) < 1.6 and any(math.hypot(d['x'] - q[0], d['y'] - q[1]) < 40.0 for q in row['pts']):
            row['pts'].append((d['x'], d['y'])); row['ids'].append(i); row['t'] = sum(p[0] * u[0] + p[1] * u[1] for p in row['pts']) / len(row['pts']); break
    else: rows.append({'yaw': yaw, 't': t, 'pts': [(d['x'], d['y'])], 'ids': [i]})
# 3. each row resampled on its line at a fixed pitch; detections snap to the nearest slot; the row extends while still inside the lot
for row in rows:
    r = math.radians(row['yaw']); u = (math.cos(r), math.sin(r)); v = (-u[1], u[0]); t = row['t']
    s0 = row['pts'][0][0] * v[0] + row['pts'][0][1] * v[1]; ks = {}
    for i, (px_, py_) in zip(row['ids'], row['pts']):
        k = round(((px_ * v[0] + py_ * v[1]) - s0) / PITCH)
        while k in ks: k += 1
        ks[k] = i
    kmin, kmax = min(ks), max(ks); occupied = set(ks)
    for sign, start in ((1, kmax + 1), (-1, kmin - 1)):
        k = start
        while abs(k - start) < 12:
            bx, by = u[0] * t + v[0] * (s0 + k * PITCH), u[1] * t + v[1] * (s0 + k * PITCH)
            if not in_lot(bx, by) or not car_ok(bx, by): break
            occupied.add(k); k += sign
    for k in sorted(occupied):
        bx, by = u[0] * t + v[0] * (s0 + k * PITCH), u[1] * t + v[1] * (s0 + k * PITCH)
        if k in ks and not car_ok(bx, by): continue
        bays.append((bx, by, row['yaw']))
        if k in ks: slot_of[ks[k]] = (bx, by, row['yaw'])
if not a.no_marks:
    for x, y, yaw in bays: bay_marks(x, y, yaw)
print(f'parking bays: {len(lot_dets)} detected cars in lots -> {len(rows)} rows, {len(bays)} bays ({len(slot_of)} cars snapped to the grid); dashes {n_dash}', flush=True)
# 4. parked cars: lot cars at their grid slot (centred between the lines, uniform spacing), street-parked cars at the detected pose
parked = []; parked_pos = []
for i, d in enumerate(car_det):
    if i % a.parked_stride: continue
    if i in slot_of: x, y, yaw = slot_of[i]
    elif i in heads: continue   # lot detection that did not fit a row (odd heading): skip rather than park across the lines
    else: x, y, yaw = d['x'], d['y'], d['yaw']
    if not car_ok(x, y) or any(math.hypot(x - px, y - py) < 2.4 for px, py in parked_pos): continue
    v = world.try_spawn_actor(car_bp('parked'), carla.Transform(carla.Location(x=float(x), y=float(y), z=0.3), carla.Rotation(yaw=park_yaw(x, y, float(yaw), rng.random() < 0.75))))
    if v is not None: v.apply_control(carla.VehicleControl(hand_brake=True)); parked.append(v); parked_pos.append((x, y))
world.tick()
# ground-level garage bays under elevated stores (the satellite cannot see them): perpendicular bays along the aisles, ~60 % occupied
n_under = 0
for poly in elevated:
    for wp in cmap.generate_waypoints(6.0):
        l = wp.transform.location
        if wp.lane_type != carla.LaneType.Driving or not poly.contains(Point(l.x, l.y)) or wp.lane_id > 0: continue
        rv = wp.transform.get_right_vector(); yaw_b = wp.transform.rotation.yaw + 90
        for sgn in (1, -1):
            bx, by = l.x + rv.x * sgn * (wp.lane_width / 2 + 2.8), l.y + rv.y * sgn * (wp.lane_width / 2 + 2.8)
            if not poly.contains(Point(bx, by)) or on_road(bx, by, 0.6) or any(math.hypot(bx - px, by - py) < 2.6 for px, py in parked_pos): continue
            if not a.no_marks: bay_marks(bx, by, yaw_b, 4.8, 2.6)
            if rng.random() < 0.6:
                v = world.try_spawn_actor(car_bp('parked'), carla.Transform(carla.Location(x=float(bx), y=float(by), z=0.3), carla.Rotation(yaw=park_yaw(bx, by, float(yaw_b), rng.random() < 0.75))))
                if v is not None: v.apply_control(carla.VehicleControl(hand_brake=True)); parked.append(v); parked_pos.append((bx, by)); n_under += 1
world.tick(); print(f'parked under elevated stores: {n_under}', flush=True)
# ego: driving waypoint in the lot next to Target (outside the building, many detected cars around, not on a main road)
cands = []
for wp in cmap.generate_waypoints(4.0):
    l = wp.transform.location
    if (in_building(l.x, l.y, 2.0) and not any(p.contains(Point(l.x, l.y)) for p in elevated)) or math.hypot(l.x - tcx, l.y - tcy) > 90: continue
    near = sum(1 for px, py in parked_pos if math.hypot(l.x - px, l.y - py) < 25)
    if near >= 6 and not any(math.hypot(l.x - px, l.y - py) < 4.0 for px, py in parked_pos): cands.append((near, -math.hypot(l.x - tcx, l.y - tcy), wp))
cands.sort(key=lambda c: (c[0], c[1]), reverse=True)
if not cands:   # no anchor building: any aisle near the site centre with parked cars around it
    for wp in cmap.generate_waypoints(4.0):
        l = wp.transform.location
        if in_building(l.x, l.y, 1.0) or math.hypot(l.x, l.y) > 140: continue
        near = sum(1 for px, py in parked_pos if math.hypot(l.x - px, l.y - py) < 25)
        if near >= 3 and not any(math.hypot(l.x - px, l.y - py) < 4.0 for px, py in parked_pos): cands.append((near, -math.hypot(l.x, l.y), wp))
    cands.sort(key=lambda c: (c[0], c[1]), reverse=True)
ego_wp = cands[0][2] if cands else cmap.get_waypoint(carla.Location(x=0.0, y=0.0, z=0.5), project_to_road=True)
ego_tf = ego_wp.transform; ego_tf.location.z += 0.4
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '10,10,10')
def try_ego(wp):
    tf = wp.transform; tf.location.z += 0.4; l = tf.location
    if any(math.hypot(l.x - cx_, l.y - cy_) < 4.0 for cx_, cy_ in columns): return None, tf
    e = world.try_spawn_actor(eb, tf)
    if e is None: return None, tf
    for _ in range(10): world.tick()   # settle: a vehicle spawned into geometry gets pushed or falls and is destroyed by the engine
    try:
        if e.get_location().z > -0.5 and e.get_location().distance(l) < 3.0: return e, tf
        e.destroy()
    except RuntimeError: pass
    return None, tf
ego = None
for cand in (cands or [(0, 0, ego_wp)]):
    ego, ego_tf = try_ego(cand[2])
    if ego is not None: break
assert ego is not None, 'ego spawn failed'
cars = []; all_wps = cmap.generate_waypoints(10.0); rng.shuffle(all_wps)
for wp in all_wps:
    if len(cars) >= a.cars: break
    l = wp.transform.location
    if l.distance(ego_tf.location) < 30 or any(math.hypot(l.x - px, l.y - py) < 6 for px, py in parked_pos): continue
    tf = wp.transform; tf.location.z += 0.4; v = world.try_spawn_actor(car_bp('npc'), tf)
    if v is None: continue
    v.set_autopilot(True, tm.get_port()); tm.ignore_lights_percentage(v, 0); tm.vehicle_percentage_speed_difference(v, rng.uniform(-15, 15)); cars.append(v)
world.tick()

# ---------------- pedestrians on the OSM footways ----------------
wbps = list(lib.filter('walker.pedestrian.*')); walkers = []
def wbp():
    b = rng.choice(wbps)
    if b.has_attribute('is_invincible'): b.set_attribute('is_invincible', 'false')
    return b
paths = [p for p in footways if sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(p, p[1:])) > 20]
def densify(p, step=2.0):
    out = [p[0]]
    for (x1, y1), (x2, y2) in zip(p, p[1:]):
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / step))
        out += [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(1, n + 1)]
    return out
paths = [densify(p) for p in paths]
for i in range(a.walkers):
    if not paths: break
    pi = rng.randrange(len(paths)); pts = paths[pi]; k = rng.randrange(len(pts)); x, y = pts[k]
    if in_building(x, y) or on_road(x, y, 0.2) or math.hypot(x - ego_tf.location.x, y - ego_tf.location.y) < 8: continue
    w = world.try_spawn_actor(wbp(), carla.Transform(carla.Location(x=float(x), y=float(y), z=0.6), carla.Rotation(yaw=rng.uniform(0, 360))))
    if w is None: continue
    walkers.append(dict(actor=w, path=pi, k=k, dir=rng.choice((1, -1)), speed=rng.uniform(0.8, 1.6), wait=rng.randint(0, 80)))
for _ in range(8): world.tick()
for v in parked:
    try: v.set_simulate_physics(False)
    except RuntimeError: pass   # settled on the ground: frozen (no per-tick physics for 200+ cars)
print(f'ready: {len(parked)} parked (from {len(car_det)} detections), {len(cars)} traffic cars, {len(walkers)} walkers, {len(props)} props; ego at ({ego_tf.location.x:.0f},{ego_tf.location.y:.0f})', flush=True)
def step_walkers():
    cmds = []
    for wk in walkers:
        w = wk['actor']
        if wk.get('dead') or not w.is_alive: continue
        if wk['wait'] > 0: wk['wait'] -= 1; cmds.append(carla.command.ApplyWalkerControl(w.id, carla.WalkerControl(speed=0.0))); continue
        pts = paths[wk['path']]; t = pts[wk['k']]
        try: loc = w.get_location()
        except RuntimeError: wk['dead'] = True; continue   # run over and destroyed server-side (is_alive is not updated for that)
        if math.hypot(t[0] - loc.x, t[1] - loc.y) < 1.0:
            wk['k'] += wk['dir']
            if wk['k'] < 0 or wk['k'] >= len(pts) or rng.random() < 0.005: wk['dir'] *= -1; wk['k'] = max(0, min(len(pts) - 1, wk['k'] + wk['dir'])); wk['wait'] = rng.randint(10, 60)
            t = pts[wk['k']]
        d = carla.Vector3D(t[0] - loc.x, t[1] - loc.y, 0.0); n = max(math.hypot(d.x, d.y), 0.1)
        if n > 12: wk['k'] = min(range(len(pts)), key=lambda j: math.hypot(pts[j][0] - loc.x, pts[j][1] - loc.y))   # pushed off the path: re-anchor
        cmds.append(carla.command.ApplyWalkerControl(w.id, carla.WalkerControl(direction=carla.Vector3D(d.x / n, d.y / n, 0.0), speed=wk['speed'])))
    if cmds: client.apply_batch(cmds)

# ---------------- HUD: chase cam + satellite minimap ----------------
W, H = 1300, 660; CW, BW = 880, 420; img = {}
pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption(f'{os.path.basename(SITE)} - geo-reconstructed CARLA scene'); font = pygame.font.SysFont('monospace', 17, bold=True)
sat = pygame.image.load(f'{SITE}/sat.png'); SPX = sat_meta['size_px'][0]; MPP = sat_meta['m_per_px']; MINI = 420; sat_scaled = pygame.transform.smoothscale(sat, (MINI, MINI)); sat_zoom_px = int(90 / MPP)   # 180 m window
cams = []
for name, (loc, rot, w_, h_, fov) in {'chase': (carla.Location(x=-6.5, z=3.0), carla.Rotation(pitch=-12), CW, H, 90)}.items():
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(w_)); cb.set_attribute('image_size_y', str(h_)); cb.set_attribute('fov', str(fov))
    cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im, n=name: img.__setitem__(n, pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)))); cams.append(cam)
def draw_minimap(ego_tf, kmh):
    l = ego_tf.location; px, py = l.x / MPP + SPX / 2, l.y / MPP + SPX / 2   # CARLA x east -> image column, y south -> image row
    x0, y0 = int(px - sat_zoom_px), int(py - sat_zoom_px); sub = pygame.Surface((2 * sat_zoom_px, 2 * sat_zoom_px)); sub.fill((20, 20, 20))
    sub.blit(sat, (0, 0), pygame.Rect(x0, y0, 2 * sat_zoom_px, 2 * sat_zoom_px)); zoom = pygame.transform.smoothscale(sub, (MINI, MINI)); screen.blit(zoom, (CW, 0))
    sc = MINI / (2 * sat_zoom_px)
    for v in cars + parked:
        try: vl = v.get_location()
        except RuntimeError: continue
        qx, qy = CW + (vl.x / MPP + SPX / 2 - x0) * sc, (vl.y / MPP + SPX / 2 - y0) * sc
        if 0 <= qx - CW < MINI and 0 <= qy < MINI: pygame.draw.circle(screen, (255, 140, 0) if v.attributes.get('role_name') == 'npc' else (200, 200, 255), (int(qx), int(qy)), 3)
    cx, cy = CW + MINI // 2, MINI // 2; r = math.radians(ego_tf.rotation.yaw)
    pygame.draw.polygon(screen, (0, 255, 0), [(cx + 10 * math.cos(r), cy + 10 * math.sin(r)), (cx + 7 * math.cos(r + 2.5), cy + 7 * math.sin(r + 2.5)), (cx + 7 * math.cos(r - 2.5), cy + 7 * math.sin(r - 2.5))])
    screen.blit(sat_scaled, (CW, MINI)); pygame.draw.rect(screen, (0, 255, 0), pygame.Rect(CW + int(px * MINI / SPX) - 3, MINI + int(py * MINI / SPX) - 3, 6, 6))
    pygame.draw.rect(screen, (255, 255, 255), pygame.Rect(CW, 0, MINI, MINI), 1)
t_loop = time.time(); clock = pygame.time.Clock(); ms_last = 50; steer = 0.0; ctl = carla.VehicleControl(); tick_n = 0; autopilot = a.test
if autopilot: ego.set_autopilot(True, tm.get_port())
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYUP and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_p: autopilot = not autopilot; ego.set_autopilot(autopilot, tm.get_port())
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        if not autopilot:
            gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
            ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
            inc = 5e-4 * ms_last; smax = 0.7 if kmh < 50 else max(0.25, 0.7 - (kmh - 50) * (0.45 / 70))
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-smax, steer - inc)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(smax, steer + inc)
            else: steer = 0.0
            ctl.steer = round(steer, 2); ego.apply_control(ctl)
        ms_last = clock.tick_busy_loop(20) if not a.test else 50; world.tick(); tick_n += 1
        step_walkers()
        screen.fill((15, 15, 15))
        if 'chase' in img: screen.blit(img['chase'], (0, 0))
        etf = ego.get_transform(); draw_minimap(etf, kmh)
        hud = [f'{kmh:5.1f} km/h {"REVERSE " if ctl.reverse else ""} ({etf.location.x:.0f},{etf.location.y:.0f})  traffic {len(cars)}  parked {len(parked)}  walkers {len(walkers)}   ' + ('AUTOPILOT (P = manual)' if autopilot else 'W/S A/D  Shift gentle  Q reverse  Space handbrake  P autopilot  ESC quit')]
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 255, 0)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if a.test and a.snapshot and tick_n in (60, 300, 600): pygame.image.save(screen, a.snapshot.replace('.png', f'_{tick_n}.png')); print('snapshot', tick_n, flush=True)
        if a.test and tick_n >= 600: print(f'test done: {tick_n} ticks in {time.time() - t_loop:.0f}s = {tick_n / (time.time() - t_loop):.1f} ticks/s', flush=True); break
except KeyboardInterrupt: pass
except RuntimeError as e:
    print('ego lost:', e, flush=True)
except Exception:
    import traceback; traceback.print_exc()
finally:
    pygame.quit()
    for cam in cams:
        try: cam.stop()
        except Exception: pass
    time.sleep(0.3)
    try:
        tm.set_synchronous_mode(False); st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    print('bye', flush=True); os._exit(0)
