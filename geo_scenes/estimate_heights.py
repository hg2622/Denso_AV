"""Building heights from satellite parallax: in off-nadir imagery a roof is displaced from its ground footprint by  d = H * tan(theta).
We match each OSM footprint to the SAM3 'building' mask (the roof), measure the displacement, calibrate tan(theta) and the common
displacement direction on buildings that carry building:levels / height tags, and estimate the others.
   estimate_heights.py SITE_DIR  -> SITE_DIR/heights.json ({name: height_m}) + heights_debug.png"""
import sys, os, json, math, numpy as np, cv2
from pyproj import Proj
from shapely.geometry import Polygon
from shapely import affinity
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from osmutil import parse_osm
D = sys.argv[1]; meta = json.load(open(f'{D}/sat.json')); mpp = meta['m_per_px']; W = meta['size_px'][0]
proj = Proj(proj='utm', zone=int((meta['lon0'] + 180) // 6) + 1, datum='WGS84'); e0, n0 = proj(meta['lon0'], meta['lat0'])
ways, _, _ = parse_osm(f'{D}/osm.xml', lambda lat, lon: (lambda e, n: (e - e0, -(n - n0)))(*proj(lon, lat)))
regs = [Polygon(r['poly']).buffer(0) for r in json.load(open(f'{D}/regions.json'))['regions']['building'] if r['area_m2'] > 150]
LEVEL_H = 3.4
def tagged_height(t):
    if t.get('height'):
        try: return float(t['height'].split()[0])
        except ValueError: pass
    if t.get('building:levels'):
        try: return float(t['building:levels']) * LEVEL_H
        except ValueError: pass
    return None
obs = []   # (tags, footprint, roof, shift vector)
for t, p in ways:
    if 'building' not in t or len(p) < 4 or t.get('location') == 'underground' or t.get('building:levels') == '0': continue
    fp = Polygon(p).buffer(0)
    if not fp.is_valid or fp.area < 200: continue
    best = None
    for rf in regs:   # roof mask with similar area whose centroid is within 80 m, best shape overlap after shifting it back
        if not (0.5 < rf.area / fp.area < 2.0) or fp.centroid.distance(rf.centroid) > 100: continue
        dx, dy = rf.centroid.x - fp.centroid.x, rf.centroid.y - fp.centroid.y; iou = fp.intersection(affinity.translate(rf, -dx, -dy)).area / fp.union(affinity.translate(rf, -dx, -dy)).area
        if iou > 0.45 and (best is None or iou > best[0]): best = (iou, rf, (dx, dy))
    if best: obs.append((t, fp, best[1], best[2]))
cal = [(math.hypot(*sh), tagged_height(t), sh) for t, fp, rf, sh in obs if tagged_height(t) and tagged_height(t) >= 15 and math.hypot(*sh) > 2]
out = {}; dbg = cv2.imread(f'{D}/sat.png')
def px(x, y): return int(x / mpp + W / 2), int(y / mpp + W / 2)
if len(cal) >= 3:
    ux = sum(sh[0] / d for d, h, sh in cal); uy = sum(sh[1] / d for d, h, sh in cal); n = math.hypot(ux, uy); ux, uy = ux / n, uy / n   # mean displacement direction
    cal = [c for c in cal if (c[2][0] * ux + c[2][1] * uy) / c[0] > math.cos(math.radians(35))] or cal   # drop mismatched roofs (direction disagrees)
    ux = sum(sh[0] / d for d, h, sh in cal); uy = sum(sh[1] / d for d, h, sh in cal); n = math.hypot(ux, uy); ux, uy = ux / n, uy / n
    k = float(np.median([(sh[0] * ux + sh[1] * uy) / h for d, h, sh in cal]))   # tan(theta): shift along the common direction per metre of height
    if not (0.05 < k < 1.2): print(f'implausible tan(theta)={k:.2f}: no estimates'); cal = []
else: cal = []
if cal:
    print(f'calibrated on {len(cal)} tagged buildings: tan(theta)={k:.3f} ({math.degrees(math.atan(k)):.1f} deg off-nadir), direction ({ux:.2f},{uy:.2f})')
    for t, fp, rf, sh in obs:
        if tagged_height(t): continue
        d = sh[0] * ux + sh[1] * uy   # displacement along the common direction (negative = inconsistent -> ignore)
        if d < 1.5: continue
        h = max(6.0, min(150.0, d / k)); name = t.get('name') or f'osm@{fp.centroid.x:.0f},{fp.centroid.y:.0f}'; out[name] = round(h, 1)
        cv2.polylines(dbg, [np.array([px(x, y) for x, y in fp.exterior.coords])], True, (0, 0, 255), 2); cv2.polylines(dbg, [np.array([px(x, y) for x, y in rf.exterior.coords])], True, (255, 0, 0), 1)
        c = fp.centroid; cv2.arrowedLine(dbg, px(c.x, c.y), px(c.x + sh[0], c.y + sh[1]), (0, 255, 255), 2); cv2.putText(dbg, f'{h:.0f}m', px(c.x, c.y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
else: print(f'only {len(cal)} tagged tall buildings matched a roof mask (need 3): no estimates')
json.dump(out, open(f'{D}/heights.json', 'w'), indent=1, ensure_ascii=False); cv2.imwrite(f'{D}/heights_debug.png', dbg)
print(f'{len(obs)} footprints matched a roof mask, {len(out)} heights estimated:', out)
