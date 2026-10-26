"""Captioned figure set that explains the site-reconstruction pipeline: make_pipeline_figures.py SITE_DIR OUT_DIR [kkmall_site_dir]
Overlays are regenerated from the site data; CARLA captures are taken from selected_runs (run geo_scene.py --test, aerial.py, photo.py first)."""
import sys, os, json, math, shutil, textwrap, numpy as np, cv2, xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
from osmutil import parse_osm; from pyproj import Proj; from shapely.geometry import Polygon; import carla
D, OUT = sys.argv[1], sys.argv[2]; KK = sys.argv[3] if len(sys.argv) > 3 else None; os.makedirs(OUT, exist_ok=True)
FONT = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22); FONTB = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 26)
def caption(img_bgr, title, lines, out, width=1400):
    lines = [w_ for l in lines for w_ in textwrap.wrap(l, 108)]
    h, w = img_bgr.shape[:2]; s = width / w; img = cv2.resize(img_bgr, (width, int(h * s))); pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    band = Image.new('RGB', (width, 44 + 30 * len(lines) + 16), (250, 250, 250)); d = ImageDraw.Draw(band); d.text((16, 8), title, font=FONTB, fill=(20, 20, 20))
    for i, l in enumerate(lines): d.text((16, 46 + 30 * i), l, font=FONT, fill=(40, 40, 40))
    canvas = Image.new('RGB', (width, pil.height + band.height), (250, 250, 250)); canvas.paste(pil, (0, 0)); canvas.paste(band, (0, pil.height)); canvas.save(out); print('wrote', out)
meta = json.load(open(f'{D}/sat.json')); mpp = meta['m_per_px']; W = meta['size_px'][0]; half = meta['half_m']
proj = Proj(proj='utm', zone=int((meta['lon0'] + 180) // 6) + 1, datum='WGS84'); e0, n0 = proj(meta['lon0'], meta['lat0'])
ways, nodes, ntags = parse_osm(f'{D}/osm.xml', lambda lat, lon: (lambda e, n: (e - e0, -(n - n0)))(*proj(lon, lat)))
def px(x, y): return int(x / mpp + W / 2), int(y / mpp + W / 2)
sat = cv2.imread(f'{D}/sat.png')
# 01 site selection: imagery with the chosen square and centre
img = sat.copy(); cv2.rectangle(img, (4, 4), (W - 5, W - 5), (60, 255, 60), 6); cv2.circle(img, (W // 2, W // 2), 10, (60, 255, 60), -1)
caption(img, '01  Pick the site', [f'Search a place or click the map in the web app (site_app.py). Centre {meta["lat0"]:.6f}, {meta["lon0"]:.6f}; square {2 * half:.0f} m.',
        'Everything downstream is expressed in metres from this centre: x east, y south (the CARLA frame).'], f'{OUT}/01_site_selection.png')
# 02 satellite
caption(sat, '02  Satellite imagery', [f'ESRI World Imagery tiles, zoom {meta["zoom"]}, {mpp:.3f} m per pixel, stitched and cropped to the square ({W} px).',
        'Used for detection only (cars, trees, lots); it is never pasted into CARLA, since spawned props cannot be textured.'], f'{OUT}/02_satellite.png')
# 03 OSM
img = sat.copy(); nb = nr = nl = nf = 0
for t, p in ways:
    if t.get('amenity') == 'parking' and len(p) >= 4: cv2.polylines(img, [np.array([px(x, y) for x, y in p])], True, (0, 255, 255), 2); nl += 1
for t, p in ways:
    if t.get('highway') in ('footway', 'path', 'steps', 'pedestrian'): cv2.polylines(img, [np.array([px(x, y) for x, y in p])], False, (255, 0, 255), 1); nf += 1
    elif 'highway' in t: cv2.polylines(img, [np.array([px(x, y) for x, y in p])], False, (255, 200, 0), 2); nr += 1
for t, p in ways:
    if 'building' in t and len(p) >= 4: cv2.polylines(img, [np.array([px(x, y) for x, y in p])], True, (0, 0, 255), 2); nb += 1
caption(img, '03  OpenStreetMap vector data (Overpass)', [f'Roads with class and name (orange, {nr}), footways (magenta, {nf}), building footprints with tags (red, {nb}), parking polygons (yellow, {nl}).',
        'Projected to UTM metres with pyproj; the site centre is the origin. Multipolygon relations are assembled into rings.'], f'{OUT}/03_osm_vectors.png')
# 04 SAM3 cars/trees, 05 regions
det = json.load(open(f'{D}/detections.json'))['detections']
caption(cv2.imread(f'{D}/sat_detections.png'), '04  SAM3 text prompts: "car" and "tree"', [f'{len(det["car"])} cars (yellow boxes: position, length, width, heading) and {len(det["tree"])} trees (green circles: crown diameter).',
        'Tiles of 136 m upsampled 2x so a car is ~40 px; detections de-duplicated across tiles and converted to metres.'], f'{OUT}/04_sam3_cars_trees.png')
reg = json.load(open(f'{D}/regions.json'))['regions']
caption(cv2.imread(f'{D}/sat_regions.png'), '05  SAM3 region prompts on the whole image', [f'"parking lot" (yellow, {len(reg["parking lot"])}), "grass" (green, {len(reg["grass"])}), "water" (blue, {len(reg["water"])}), "building" (red, {len(reg["building"])}).',
        'Lot masks pave lots that OSM does not map. Building masks are roofs, displaced by parallax, so they are only used for height estimation.'], f'{OUT}/05_sam3_regions.png')
# 06 OpenDRIVE lanes
m = carla.Map('t', open(f'{D}/site_lots.xodr').read()); img = sat.copy(); nd = npk = 0
for rd in ET.parse(f'{D}/site_lots.xodr').getroot().findall('road'):   # parking strips (not returned by generate_waypoints): rectangles from the xodr
    if rd.get('name') != 'parking_lot': continue
    g = rd.find('planView/geometry'); x0, y0, hdg, L = float(g.get('x')), -float(g.get('y')), -float(g.get('hdg')), float(g.get('length')); Wd = float(rd.find('lanes/laneSection/right/lane/width').get('a'))
    ux, uy = math.cos(hdg), math.sin(hdg); nx, ny = -uy, ux   # right-hand side in the y-south frame
    quad = np.array([px(x0, y0), px(x0 + ux * L, y0 + uy * L), px(x0 + ux * L + nx * Wd, y0 + uy * L + ny * Wd), px(x0 + nx * Wd, y0 + ny * Wd)]); ov = img.copy(); cv2.fillPoly(ov, [quad], (0, 200, 255)); img = cv2.addWeighted(ov, 0.35, img, 0.65, 0); npk += 1
for wp in m.generate_waypoints(2.0):
    l = wp.transform.location; cv2.circle(img, px(l.x, l.y), 2, (255, 80, 80), -1); nd += 1
caption(img, '06  OSM -> OpenDRIVE road network (carla.Osm2Odr + sanitizer + lot paving)', ['Blue: driving-lane centre lines every 2 m (service roads and parking aisles included; SUMO\'s "restricted" lanes rewritten to driving).',
        'Yellow: 9 m "parking" lane strips covering every lot polygon (OSM + satellite masks) so lots render as asphalt.',
        'Sanitizer: roads < 1.5 m stretched, crosswalk objects past the road end removed (both crashed the server).'], f'{OUT}/06_opendrive_lanes.png')
# 07 asset catalogue
if os.path.exists('selected_runs/geo_cat_sheet.png'):
    caption(cv2.imread('selected_runs/geo_cat_sheet.png'), '07  CARLA\'s own cooked meshes, spawned by path with static.prop.mesh', ['Row 1-2: gas station strip, mall, block, apartment towers, houses, terraced house, mansion, parking structure, church (all measured: size + pivot in geo_assets.json).',
        'Row 3: candidate deck pieces seen from below: flat ground tiles are invisible from underneath, so decks use storey modules instead.'], f'{OUT}/07_asset_catalogue.png')
# 08 building rules (text figure over OSM overlay)
img = sat.copy()
for t, p in ways:
    if 'building' in t and len(p) >= 4:
        P = Polygon(p); kind = t.get('building'); col = (0, 200, 255) if t.get('layer', '0').isdigit() and int(t.get('layer', '0')) >= 2 else (255, 0, 255) if kind in ('church',) or t.get('amenity') == 'place_of_worship' else (0, 255, 0) if kind in ('apartments', 'house', 'residential') else (0, 0, 255)
        cv2.polylines(img, [np.array([px(x, y) for x, y in p])], True, col, 2)
        if P.area > 1200: c = P.centroid; cv2.putText(img, f"{kind} lv{t.get('building:levels', '?')}", (px(c.x, c.y)[0] - 50, px(c.x, c.y)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
caption(img, '08  Building rules from OSM tags', ['Red: retail/commercial/office -> storey modules (rectangles as a grid, irregular outlines as modules laid along the exact polygon edges).',
        'Green: apartments/houses -> apartment/house meshes, stacked for towers. Magenta: church mesh. Cyan: layer>=2 with roads under it -> columns + deck + store on top.',
        'Height = building:levels x 3.4 m or height tag; building:part gives podium + towers; underground buildings are skipped.'], f'{OUT}/08_building_rules.png')
# 09-12 CARLA captures
S = os.path.basename(D.rstrip('/'))
if os.path.exists(f'selected_runs/geo_{S}_aerial.png'):
    a = cv2.resize(cv2.imread(f'selected_runs/geo_{S}_aerial.png'), (W, W)); caption(np.hstack([sat, a]), '09  Result from above: satellite (left) vs CARLA (right), same framing, north up',
        ['Roads, lots, footprints, parked cars and trees land where the data says. Ground outside roads and lots stays beige (no grass/water material can be spawned).',
         'Capture: geo_scenes/aerial.py (a camera 294 m above the centre looking straight down).'], f'{OUT}/09_aerial_vs_satellite.png')
for fn, num, title, lines in (('geo_photo_target_ext.png', '10', '10  Street level: the elevated store', ['OSM layer=2 + aisles under the footprint -> columns, a storey-module deck and the store built on top; the aisles stay drivable.']),
                              ('geo_photo_under.png', '11', '11  Under the deck: generated garage bays', ['The satellite cannot see under the roof, so bays are generated along the aisles and 60 % filled with parked cars.']),
                              ('geo_photo_street.png', '12', '12  Street level: shop fronts, trees, lamps, traffic', ['Shop/amenity facades facing a road get awnings, benches, vending machines, carts or tables; lamps every 35 m on main roads; Traffic Manager cars; pedestrians on OSM footways.']),
                              (f'geo_{S}_300.png', '13', '13  The driving window', ['Chase camera + satellite minimap (zoomed, with traffic and parked-car dots) + site overview. Keys: W/S A/D, Shift gentle, Q reverse, Space, P autopilot, ESC.'])):
    if os.path.exists(f'selected_runs/{fn}'): caption(cv2.imread(f'selected_runs/{fn}'), title, lines, f'{OUT}/{num}_{fn.replace("geo_", "").replace(f"{S}_", "")}')
# 14 parallax caveat (KK Mall)
if KK and os.path.exists(f'{KK}/heights_debug.png') and os.path.exists('selected_runs/kkmall/overview.png'):
    caption(cv2.imread('selected_runs/kkmall/overview.png'), '14  Limits seen at a dense CBD (KK Mall, Shenzhen)', ['Top: satellite vs CARLA. The imagery is off-nadir, so tall roofs are displaced from their ground footprints; CARLA builds at the OSM footprint, which is the true geometry.',
        'OSM maps the mall as one outline without tower parts, so the towers on the podium cannot be built; untagged towers get type-default heights unless heights.json is supplied.',
        'Bottom: oblique CARLA view (the KK100 tower from its building:part, 100 levels) and street views of the curved podium built along the exact outline.'], f'{OUT}/14_limits_parallax_kkmall.png')
