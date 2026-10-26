"""List Bench2Drive routes: id, town, scenario types.  python list_routes.py [xml] [filter-substring]"""
import sys, xml.etree.ElementTree as ET
xml = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else '/home/all/carla_ws/Bench2Drive/leaderboard/data/bench2drive220.xml'
flt = sys.argv[2].lower() if len(sys.argv) > 2 else ''
for r in ET.parse(xml).getroot().iter('route'):
    types = [s.get('type') for s in r.iter('scenario')]
    line = f"{r.get('id'):>6}  {r.get('town'):8s}  {' + '.join(types)}"
    if flt in line.lower(): print(line)
