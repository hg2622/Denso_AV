"""One-command site reconstruction from public map data:
   python geo_scenes/build_site.py NAME LAT LON [--half 170] [--zoom 19] [--skip-detect]
Fetches OSM (Overpass) + ESRI World Imagery for a (2*half) m square around (LAT, LON), converts OSM -> OpenDRIVE (with service roads /
parking aisles, sanitized, lots paved), and runs SAM3 on the satellite image for parked cars and trees. Then drive it with
   python geo_scene.py --site geo_scenes/NAME"""
import sys, os, json, math, io, time, argparse, subprocess, urllib.request, urllib.parse
from PIL import Image
ap = argparse.ArgumentParser(); ap.add_argument('name'); ap.add_argument('lat', type=float); ap.add_argument('lon', type=float); ap.add_argument('--half', type=float, default=170.0); ap.add_argument('--zoom', type=int, default=19)
ap.add_argument('--road-margin', type=float, default=120.0, help='extra metres of OSM around the site so roads continue past the edge'); ap.add_argument('--skip-detect', action='store_true'); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__)); D = f'{HERE}/{a.name}'; os.makedirs(D, exist_ok=True); lat0, lon0 = a.lat, a.lon
UA = {'User-Agent': 'carla-scene-builder/0.1'}
# 1. OSM
if not os.path.exists(f'{D}/osm.xml'):
    dlat = (a.half + a.road_margin) / 111320.0; dlon = (a.half + a.road_margin) / (111320.0 * math.cos(math.radians(lat0)))
    bbox = f'{lat0 - dlat:.6f},{lon0 - dlon:.6f},{lat0 + dlat:.6f},{lon0 + dlon:.6f}'
    q = f'[out:xml][timeout:120];(node({bbox});way({bbox});relation({bbox})["type"="multipolygon"];);(._;>;);out body;'   # multipolygons only: a route relation would drag in ways across the whole region
    MIRRORS = ['https://overpass-api.de/api/interpreter', 'https://overpass.osm.ch/api/interpreter', 'https://overpass.kumi.systems/api/interpreter',
               'https://lz4.overpass-api.de/api/interpreter', 'https://z.overpass-api.de/api/interpreter', 'https://overpass.private.coffee/api/interpreter']
    xml = None
    for attempt in range(3):            # rotate through the public Overpass mirrors; the main one returns 502 when overloaded
        for url in MIRRORS:
            try:
                req = urllib.request.Request(url, data=urllib.parse.urlencode({'data': q}).encode(), headers=UA)
                xml = urllib.request.urlopen(req, timeout=180).read()
                if b'<osm' not in xml[:400]: raise RuntimeError('not an OSM response')
                open(f'{D}/osm.xml', 'wb').write(xml); print('osm.xml', len(xml) // 1000, 'kB  bbox', bbox, ' from', url.split('/')[2]); break
            except Exception as e: print(f'overpass {url.split("/")[2]}: {str(e)[:60]}'); xml = None
        if xml: break
        time.sleep(15)
    if not xml: raise SystemExit('BUILD FAILED: no Overpass mirror answered (OpenStreetMap servers overloaded or no network). Try again in a few minutes.')
# 2. satellite
if not os.path.exists(f'{D}/sat.png'):
    z = a.zoom; n = 2 ** z; mpp = 156543.03392 * math.cos(math.radians(lat0)) / n
    def px(lat, lon): return (lon + 180) / 360 * n * 256, (1 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2 * n * 256
    cx, cy = px(lat0, lon0); hp = a.half / mpp; x0, y0, x1, y1 = cx - hp, cy - hp, cx + hp, cy + hp
    tx0, ty0, tx1, ty1 = int(x0 // 256), int(y0 // 256), int(x1 // 256), int(y1 // 256); canvas = Image.new('RGB', ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            req = urllib.request.Request(f'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{ty}/{tx}', headers=UA)
            for attempt in range(5):   # tile servers return the odd 502; retry with backoff
                try: canvas.paste(Image.open(io.BytesIO(urllib.request.urlopen(req, timeout=30).read())), ((tx - tx0) * 256, (ty - ty0) * 256)); break
                except Exception as e:
                    if attempt == 4: raise SystemExit(f'BUILD FAILED: imagery tile {z}/{ty}/{tx} unavailable after 5 tries ({str(e)[:60]})')
                    time.sleep(3 * (attempt + 1))
    crop = canvas.crop((int(x0 - tx0 * 256), int(y0 - ty0 * 256), int(x1 - tx0 * 256), int(y1 - ty0 * 256))); crop.save(f'{D}/sat.png'); crop.resize((600, 600)).save(f'{D}/sat_small.png')
    json.dump({'lat0': lat0, 'lon0': lon0, 'half_m': a.half, 'zoom': z, 'm_per_px': mpp, 'size_px': list(crop.size), 'note': 'image centre = (lat0, lon0); x right = east, y down = south = CARLA frame'}, open(f'{D}/sat.json', 'w'), indent=1)
    print('sat.png', crop.size, 'm/px %.3f' % mpp)
# 3. SAM3 detections (cars, trees, lot / grass / water / building masks)
if not a.skip_detect and not (os.path.exists(f'{D}/detections.json') and os.path.exists(f'{D}/regions.json')):
    subprocess.run(['/home/all/miniconda3/envs/sam3/bin/python', f'{HERE}/detect_sat.py', D], check=True)
# 4. OpenDRIVE (service roads + OSM lots + satellite lots paved)
subprocess.run(['/home/all/miniconda3/envs/b2d/bin/python', f'{HERE}/build_xodr.py', D], check=True)
subprocess.run(['/home/all/miniconda3/envs/b2d/bin/python', f'{HERE}/estimate_heights.py', D])   # 5. heights from roof parallax (needs regions.json)
print(f'site ready: {D}\n  python geo_scene.py --site {D}')
