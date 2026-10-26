"""Shared OSM parsing: ways + multipolygon-relation outer rings, projected with a caller-supplied (lat, lon) -> (x, y) function."""
import xml.etree.ElementTree as ET
def parse_osm(path, ll2xy):
    root = ET.parse(path).getroot()
    nodes = {n.get('id'): ll2xy(float(n.get('lat')), float(n.get('lon'))) for n in root.findall('node')}
    node_tags = {n.get('id'): {t.get('k'): t.get('v') for t in n.findall('tag')} for n in root.findall('node')}
    way_nodes = {}; ways = []
    for w in root.findall('way'):
        refs = [nd.get('ref') for nd in w.findall('nd') if nd.get('ref') in nodes]; way_nodes[w.get('id')] = refs
        tags = {t.get('k'): t.get('v') for t in w.findall('tag')}
        if len(refs) >= 2: ways.append((tags, [nodes[r] for r in refs]))
    for rel in root.findall('relation'):   # multipolygon outers (split outer ways are chained end to end)
        tags = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if tags.get('type') != 'multipolygon': continue
        outers = [way_nodes[m.get('ref')] for m in rel.findall('member') if m.get('type') == 'way' and m.get('role') in ('outer', '') and m.get('ref') in way_nodes]
        rings = []; pool = [list(o) for o in outers if len(o) >= 2]
        while pool:
            ring = pool.pop(0); changed = True
            while changed and ring[0] != ring[-1]:
                changed = False
                for i, seg in enumerate(pool):
                    if seg[0] == ring[-1]: ring += seg[1:]; pool.pop(i); changed = True; break
                    if seg[-1] == ring[-1]: ring += seg[::-1][1:]; pool.pop(i); changed = True; break
                    if seg[-1] == ring[0]: ring = seg[:-1] + ring; pool.pop(i); changed = True; break
                    if seg[0] == ring[0]: ring = seg[::-1][:-1] + ring; pool.pop(i); changed = True; break
            if len(ring) >= 4 and ring[0] == ring[-1]: rings.append(ring)   # open chains (members outside the bbox) are dropped
        for ring in rings: ways.append((dict(tags), [nodes[r] for r in ring]))
    return ways, nodes, node_tags
