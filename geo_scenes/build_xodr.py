"""OSM -> OpenDRIVE for a site, in the site frame (x east, y north, origin lat0/lon0), including service roads (parking aisles):
   build_xodr.py SITE_DIR   (reads osm.xml + sat.json, writes site_raw.xodr, then site.xodr (sanitized) and site_lots.xodr (lots paved))"""
import sys, json, subprocess, os; sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla
from pyproj import Proj
site = sys.argv[1]; meta = json.load(open(f'{site}/sat.json')); LON0 = meta['lon0']; proj = Proj(proj='utm', zone=int((LON0 + 180) // 6) + 1, datum='WGS84'); e0, n0 = proj(meta['lon0'], meta['lat0'])
st = carla.Osm2OdrSettings(); st.use_offsets = True; st.offset_x = -e0; st.offset_y = -n0; st.center_map = False; st.default_lane_width = 3.5; st.generate_traffic_lights = False
st.proj_string = f'+proj=utm +zone={int((LON0 + 180) // 6) + 1} +datum=WGS84 +units=m +no_defs'
st.set_osm_way_types(['motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified', 'residential', 'living_street', 'service', 'busway'])
xodr = carla.Osm2Odr.convert(open(f'{site}/osm.xml').read(), st); open(f'{site}/site_raw.xodr', 'w').write(xodr); print('converted', len(xodr) // 1000, 'kB')
here = os.path.dirname(os.path.abspath(__file__)); py = sys.executable
subprocess.run([py, f'{here}/fix_xodr.py', f'{site}/site_raw.xodr', f'{site}/site.xodr', site], check=True)
subprocess.run([py, f'{here}/add_parking.py', f'{site}/osm.xml', f'{site}/site.xodr', f'{site}/site_lots.xodr', str(meta['lat0']), str(meta['lon0'])], check=True)
m = carla.Map('t', open(f'{site}/site_lots.xodr').read()); print('client check: crosswalks', len(m.get_crosswalks()), 'waypoints', len(m.generate_waypoints(5.0)))
