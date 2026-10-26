"""Approx route length (m, sum of waypoint gaps) and scenario count per route.  python route_lengths.py <xml>"""
import sys, math, xml.etree.ElementTree as ET
from collections import Counter
for r in ET.parse(sys.argv[1]).getroot().iter('route'):
    pts = [(float(p.get('x')), float(p.get('y'))) for p in r.find('waypoints').iter('position')]
    L = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
    sc = Counter(s.get('type') for s in r.iter('scenario'))
    print(f"{r.get('id'):>6}  {r.get('town'):8s}  {L/1000:6.2f} km  {sum(sc.values()):3d} scenarios  {', '.join(f'{k}x{v}' if v>1 else k for k,v in sc.most_common(4))}{' ...' if len(sc)>4 else ''}")
