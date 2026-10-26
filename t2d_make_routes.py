"""Make random 'free-run' route files for Think2Drive (standard leaderboard XML: start + goal key points, no scenarios).
Works without a CARLA server: reads the town's OpenDRIVE file and samples far-apart lane waypoints.
  python t2d_make_routes.py --map Town04 --n 3 --dist 600 --out CaRL/Think2Drive/freeroam_routes
"""
import argparse, os, random, sys, xml.etree.ElementTree as ET
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
import carla
ap = argparse.ArgumentParser(); ap.add_argument('--map', default='Town04'); ap.add_argument('--n', type=int, default=3)
ap.add_argument('--dist', type=float, default=600); ap.add_argument('--out', default='/home/all/carla_ws/CaRL/Think2Drive/freeroam_routes')
a = ap.parse_args()
xodr = next(f for f in [f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/{a.map}.xodr', f'/home/all/carla_ws/carla/CarlaUE4/Content/Carla/Maps/{a.map}/OpenDrive/{a.map}.xodr'] if os.path.exists(f))
m = carla.Map(a.map, open(xodr).read())
wps = [w for w in m.generate_waypoints(20.0) if w.lane_type == carla.LaneType.Driving and not w.is_junction]
os.makedirs(a.out, exist_ok=True)
for f in os.listdir(a.out): os.remove(os.path.join(a.out, f))
for i in range(a.n):
    s = random.choice(wps); far = [w for w in wps if w.transform.location.distance(s.transform.location) > a.dist] or sorted(wps, key=lambda w: -w.transform.location.distance(s.transform.location))[:20]
    pts = [s.transform.location, random.choice(far).transform.location]
    root = ET.Element('routes'); r = ET.SubElement(root, 'route', id=str(i), town=a.map); ET.SubElement(r, 'weathers')
    w_el = ET.SubElement(r, 'waypoints')
    for p in pts: ET.SubElement(w_el, 'position', x=f'{p.x:.1f}', y=f'{p.y:.1f}', z=f'{p.z + 0.5:.1f}')
    ET.SubElement(r, 'scenarios')
    path = os.path.join(a.out, f'freeroam_{a.map}_{i:02d}.xml'); ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    print(path, f'{pts[0].distance(pts[1]):.0f} m straight-line')
