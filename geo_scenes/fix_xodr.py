"""Make an Osm2Odr-generated OpenDRIVE safe for CARLA's generate_opendrive_world:
roads shorter than MIN_LEN (OSM junction nodes a few cm apart) crash the server with 's <= road->GetLength()'; they are
stretched symmetrically to MIN_LEN (tiny overlap into the neighbouring junctions is harmless). Usage: fix_xodr.py in.xodr out.xodr"""
import sys, math, xml.etree.ElementTree as ET
MIN_LEN = 1.5
src, dst = sys.argv[1:3]; tree = ET.parse(src); r = tree.getroot(); n = 0
for rd in r.findall('road'):
    L = float(rd.get('length'))
    if L >= MIN_LEN: continue
    geos = rd.find('planView').findall('geometry')
    if len(geos) != 1 or geos[0].find('line') is None: continue
    g = geos[0]; x, y, h = float(g.get('x')), float(g.get('y')), float(g.get('hdg')); d = (MIN_LEN - L) / 2
    g.set('x', f'{x - d * math.cos(h):.8f}'); g.set('y', f'{y - d * math.sin(h):.8f}'); g.set('length', f'{MIN_LEN:.8f}'); rd.set('length', f'{MIN_LEN:.8f}'); n += 1
nres = 0
for ln in r.iter('lane'):   # SUMO exports service roads / parking aisles with vehicle-class restrictions as 'restricted' lanes: CARLA ignores those
    if ln.get('type') == 'restricted': ln.set('type', 'driving'); nres += 1
print(f'restricted -> driving lanes: {nres}')
# Connecting roads INSIDE a junction sometimes carry a sidewalk lane (SUMO builds the kerb through the junction). CARLA raises every
# sidewalk 15 cm, so that kerb crosses the area cars actually turn through and the ego hits a step mid-intersection. Flatten those only:
# street kerbs outside junctions keep their normal height, and the lanes are kept (deleting lanes segfaults the mesh generator).
njs = 0
for rd in r.findall('road'):
    if rd.get('junction', '-1') == '-1': continue
    for ln in rd.iter('lane'):
        if ln.get('type') != 'sidewalk': continue
        for h in ln.findall('height'): ln.remove(h)
        ET.SubElement(ln, 'height', sOffset='0', inner='0.0', outer='0.0')
        for wd in ln.findall('width'): wd.set('a', '0.01'); wd.set('b', '0'); wd.set('c', '0'); wd.set('d', '0')   # CARLA ignores <height> on junction connecting roads, so shrink the kerb to 1 cm wide instead
        njs += 1
print(f'sidewalk kerbs flattened inside junctions: {njs}')
nobj = 0
for rd in r.findall('road'):   # crosswalk objects placed past the road end (Osm2Odr rounding) -> get_crosswalks()/mesh generation assert
    L = float(rd.get('length'))
    for objs in rd.findall('objects'):
        for o in list(objs):
            s_ = float(o.get('s', '0')); ln = float(o.get('length', '0'))
            if s_ < 0 or s_ > L + 1e-9: objs.remove(o); nobj += 1
            elif s_ + ln / 2 > L: o.set('s', f'{max(0.0, L - ln / 2):.8f}')
print(f'removed {nobj} objects outside their road')
tree.write(dst, xml_declaration=True, encoding='UTF-8'); print(f'stretched {n} roads shorter than {MIN_LEN} m -> {dst}')
