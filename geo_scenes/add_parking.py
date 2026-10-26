"""Append OSM parking-lot polygons (amenity=parking) to an OpenDRIVE as wide single-lane 'parking' roads, so the lots are
paved like the roads instead of showing the bare ground. Usage: add_parking.py osm.xml in.xodr out.xodr lat0 lon0"""
import sys, math, xml.etree.ElementTree as ET
from pyproj import Proj
from shapely.geometry import Polygon
osm_f, src, dst, lat0, lon0 = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5]); LON0 = lon0
proj = Proj(proj='utm', zone=int((LON0 + 180) // 6) + 1, datum='WGS84'); e0, n0 = proj(lon0, lat0)
import os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from osmutil import parse_osm
ways, _, _ = parse_osm(osm_f, lambda lat, lon: (lambda e, n: (e - e0, n - n0))(*proj(lon, lat)))   # xodr frame: x east, y north
lots = []
for t, pts in ways:
    if t.get('amenity') != 'parking' or t.get('parking') in ('multi-storey', 'underground', 'rooftop'): continue
    if len(pts) >= 4 and Polygon(pts).is_valid and Polygon(pts).area > 100: lots.append((Polygon(pts), 0.0))
rj = os.path.join(os.path.dirname(osm_f), 'regions.json'); n_sam = 0
if os.path.exists(rj):   # lots the satellite shows but OSM does not map (SAM3 'parking lot' masks; frame x east, y south -> flip y)
    osm_union = [p for p, _ in lots]
    for reg in json.load(open(rj))['regions'].get('parking lot', []):
        if reg['score'] < 0.3 or reg['area_m2'] < 300: continue
        P = Polygon([(x, -y) for x, y in reg['poly']]).buffer(0)
        if P.is_empty or not P.is_valid: continue
        if any(P.intersection(q).area > 0.6 * P.area for q in osm_union): continue
        lots.append((P, 0.0)); n_sam += 1
print(f'lots: {len(lots) - n_sam} from OSM, {n_sam} from satellite masks')
tree = ET.parse(src); r = tree.getroot(); rid = max(int(x.get('id')) for x in r.findall('road')) + 1000; n = 0
STRIP = 9.0
from shapely import affinity
def add_road(x0, y0, hdg, L, W, elev=0.0):   # exactly the road level: lanes and lots form one surface with no step
    global rid, n
    road = ET.SubElement(r, 'road', name='parking_lot', length=f'{L:.4f}', id=str(rid), junction='-1'); ET.SubElement(road, 'link')
    ET.SubElement(road, 'type', s='0', type='town'); pv = ET.SubElement(road, 'planView')
    ET.SubElement(ET.SubElement(pv, 'geometry', s='0', x=f'{x0:.4f}', y=f'{y0:.4f}', hdg=f'{hdg:.6f}', length=f'{L:.4f}'), 'line')
    ET.SubElement(ET.SubElement(road, 'elevationProfile'), 'elevation', s='0', a=f'{elev:.3f}', b='0', c='0', d='0'); ET.SubElement(road, 'lateralProfile')
    ls = ET.SubElement(ET.SubElement(road, 'lanes'), 'laneSection', s='0')
    c = ET.SubElement(ET.SubElement(ls, 'center'), 'lane', id='0', type='none', level='true'); ET.SubElement(c, 'link')
    l = ET.SubElement(ET.SubElement(ls, 'right'), 'lane', id='-1', type='parking', level='true'); ET.SubElement(l, 'link'); ET.SubElement(l, 'width', sOffset='0', a=f'{W:.4f}', b='0', c='0', d='0')
    rid += 1; n += 1
for poly, elev in lots:   # cover the exact polygon with parallel strips along its long axis (each strip clipped to the polygon)
    if poly.geom_type != 'Polygon': poly = max(poly.geoms, key=lambda g: g.area)
    mrr = poly.minimum_rotated_rectangle; cs = list(mrr.exterior.coords)[:4]
    e1 = math.hypot(cs[1][0] - cs[0][0], cs[1][1] - cs[0][1]); e2 = math.hypot(cs[2][0] - cs[1][0], cs[2][1] - cs[1][1])
    hdg = math.atan2(cs[1][1] - cs[0][1], cs[1][0] - cs[0][0]) if e1 >= e2 else math.atan2(cs[2][1] - cs[1][1], cs[2][0] - cs[1][0])
    cx, cy = mrr.centroid.x, mrr.centroid.y; local = affinity.rotate(affinity.translate(poly, -cx, -cy), -math.degrees(hdg), origin=(0, 0))
    minu, minv, maxu, maxv = local.bounds; nstrip = max(1, math.ceil((maxv - minv) / STRIP)); sw = (maxv - minv) / nstrip
    for i in range(nstrip):
        v_lo, v_hi = minv + i * sw, minv + (i + 1) * sw
        part = local.intersection(Polygon([(minu - 1, v_lo), (maxu + 1, v_lo), (maxu + 1, v_hi), (minu - 1, v_hi)]))
        if part.is_empty or part.area < 20: continue
        pu0, _, pu1, _ = part.bounds
        # reference line along the strip's upper edge (v_hi) heading +u: the lane lies to the right (towards lower v)
        x0 = cx + pu0 * math.cos(hdg) - v_hi * math.sin(hdg); y0 = cy + pu0 * math.sin(hdg) + v_hi * math.cos(hdg)
        add_road(x0, y0, hdg, pu1 - pu0, sw + 0.3, elev)
tree.write(dst, xml_declaration=True, encoding='UTF-8'); print(f'added {n} parking strips for {len(lots)} lots -> {dst}')
