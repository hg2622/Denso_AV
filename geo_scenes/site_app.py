"""Map search -> bird's-eye preview -> one-click build -> drive, as a local web app (stdlib only, no Google key needed):
    python geo_scenes/site_app.py [--port 8765]      then open http://localhost:8765
Search a place (OpenStreetMap Nominatim), pan/zoom the satellite view (ESRI World Imagery, the same tiles the pipeline uses), click to
set the site centre, pick the square size, press BUILD (~1 min: OSM + imagery + SAM3 + OpenDRIVE) and then DRIVE (starts the CARLA
server if needed and geo_scene.py on the local display). STOP ends the scene and the server."""
import argparse, json, os, re, subprocess, threading, time, urllib.parse, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE); PY = '/home/all/miniconda3/envs/b2d/bin/python'
UA = {'User-Agent': 'carla-scene-builder/0.1'}
STATE = {'build': None, 'log': [], 'site': None, 'drive': None, 'server': None, 'phase': 'idle'}
def log(s): STATE['log'].append(s); print(s, flush=True)
def run_build(name, lat, lon, half):
    STATE['phase'] = 'building'; STATE['log'] = []; site = f'{HERE}/{name}'; t0 = time.time()
    for f in ('osm.xml', 'sat.png', 'detections.json', 'regions.json', 'site_lots.xodr'):   # rebuild when the centre moved
        p = f'{site}/{f}'
        if os.path.exists(p):
            try:
                m = json.load(open(f'{site}/sat.json'))
                if abs(m['lat0'] - lat) > 1e-6 or abs(m['lon0'] - lon) > 1e-6 or abs(m['half_m'] - half) > 0.5: os.remove(p)
            except Exception: os.remove(p)
    p = subprocess.Popen(['python3', f'{HERE}/build_site.py', name, str(lat), str(lon), '--half', str(half)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=WS)
    for line in p.stdout:
        if re.search(r'Warning|pj_obj|Options|pkg_resources|timm|proj=|georef|Using default|^\s*$', line): continue
        log(line.rstrip())
    p.wait(); ok = p.returncode == 0 and os.path.exists(f'{site}/site_lots.xodr')
    log(f'{"BUILD DONE" if ok else "BUILD FAILED"} in {time.time() - t0:.0f} s'); STATE['site'] = name if ok else None; STATE['phase'] = 'built' if ok else 'idle'
def server_up():
    return subprocess.run(['pgrep', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True).returncode == 0
def run_drive(name):
    STATE['phase'] = 'starting'; env = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':1'))
    if not server_up():
        log('starting the CARLA server (Epic) ...'); STATE['server'] = subprocess.Popen(['bash', f'{WS}/carla_server.sh'], stdout=open(f'{WS}/logs/carla_2000.log', 'w'), stderr=subprocess.STDOUT, cwd=WS, env=env)
        for _ in range(90):
            time.sleep(1)
            if subprocess.run([PY, '-c', "import sys; sys.path.insert(0,'/home/all/carla_ws/carla/PythonAPI/carla'); import carla; carla.Client('localhost',2000).set_timeout(5.0); carla.Client('localhost',2000).get_server_version()"], capture_output=True).returncode == 0: break
    log(f'launching geo_scene.py --site geo_scenes/{name} on DISPLAY {env["DISPLAY"]} (window: W/S A/D, Q reverse, P autopilot, ESC quit)')
    STATE['drive'] = subprocess.Popen([PY, f'{WS}/geo_scene.py', '--site', f'geo_scenes/{name}'], stdout=open(f'{WS}/logs/geo_scene.log', 'w'), stderr=subprocess.STDOUT, cwd=WS, env=env); STATE['phase'] = 'driving'
    STATE['drive'].wait(); log('scene ended'); STATE['phase'] = 'built'
def stop_all():
    subprocess.run(['pkill', '-f', 'geo_scen[e].py']); time.sleep(2); subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng']); STATE['phase'] = 'built' if STATE['site'] else 'idle'; log('scene and server stopped')
HTML = r'''<!doctype html><html><head><meta charset="utf-8"><title>CARLA site builder</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"><script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>body{margin:0;font:14px system-ui;display:flex;height:100vh}#map{flex:1}#side{width:380px;padding:12px;background:#1d1f24;color:#eee;overflow:auto}
input,select,button{font:14px system-ui;padding:6px;margin:2px 0}button{cursor:pointer}#log{font:12px monospace;white-space:pre-wrap;background:#111;padding:6px;height:40vh;overflow:auto}
.res{cursor:pointer;padding:3px;border-bottom:1px solid #333}.res:hover{background:#333}#go{background:#2a7;color:#fff;border:0;padding:8px 14px}#drv{background:#27a;color:#fff;border:0;padding:8px 14px}#stp{background:#a33;color:#fff;border:0;padding:8px 14px}</style></head>
<body><div id="map"></div><div id="side"><h3>Search → bird's-eye → build → drive</h3>
<input id="q" size="34" placeholder="place, address or 'lat, lon'"><button onclick="search()">Search</button><div id="results"></div>
<p>Centre: <span id="c">click the map</span><br>Size: <select id="half"><option value="120">240 m</option><option value="170" selected>340 m</option><option value="220">440 m</option><option value="280">560 m</option></select>
&nbsp;Name: <input id="name" size="12" value="site1"></p>
<button id="go" onclick="build()">BUILD (~1 min)</button> <button id="drv" onclick="drive()">DRIVE</button> <button id="stp" onclick="stopAll()">STOP</button>
<p id="phase">idle</p><div id="log"></div></div>
<script>
const map=L.map('map').setView([40.4609,-79.9214],17);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Esri World Imagery'}).addTo(map);
L.tileLayer('https://{s}.basemaps.cartocdn.com/light_only_labels/{z}/{x}/{y}{r}.png',{maxZoom:19,opacity:0.9}).addTo(map);
let centre=null,box=null,marker=null;
function setCentre(lat,lon){centre=[lat,lon];document.getElementById('c').textContent=lat.toFixed(6)+', '+lon.toFixed(6);draw();}
function draw(){if(!centre)return;const half=+document.getElementById('half').value;const dlat=half/111320,dlon=half/(111320*Math.cos(centre[0]*Math.PI/180));
 if(box)map.removeLayer(box);if(marker)map.removeLayer(marker);box=L.rectangle([[centre[0]-dlat,centre[1]-dlon],[centre[0]+dlat,centre[1]+dlon]],{color:'#2f6',weight:2,fill:false}).addTo(map);marker=L.circleMarker(centre,{radius:5,color:'#2f6'}).addTo(map);}
map.on('click',e=>setCentre(e.latlng.lat,e.latlng.lng));document.getElementById('half').onchange=draw;
async function search(){const q=document.getElementById('q').value.trim();const m=q.match(/^\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*$/);
 if(m){setCentre(+m[1],+m[2]);map.setView(centre,17);return;}
 const r=await fetch('/search?q='+encodeURIComponent(q));const js=await r.json();const d=document.getElementById('results');d.innerHTML='';
 js.forEach(x=>{const e=document.createElement('div');e.className='res';e.textContent=x.display_name;e.onclick=()=>{setCentre(+x.lat,+x.lon);map.setView(centre,17);d.innerHTML='';};d.appendChild(e);});
 if(!js.length)d.textContent='no result';}
async function build(){if(!centre){alert('click the map to set the centre');return;}
 await fetch('/build',{method:'POST',body:JSON.stringify({name:document.getElementById('name').value,lat:centre[0],lon:centre[1],half:+document.getElementById('half').value})});}
async function drive(){await fetch('/drive',{method:'POST'});}
async function stopAll(){await fetch('/stop',{method:'POST'});}
async function poll(){const r=await fetch('/status');const s=await r.json();document.getElementById('phase').textContent='phase: '+s.phase+(s.site?'   site: '+s.site:'');
 const l=document.getElementById('log');l.textContent=s.log.join('\n');l.scrollTop=l.scrollHeight;setTimeout(poll,1000);}
poll();document.getElementById('q').addEventListener('keydown',e=>{if(e.key==='Enter')search();});
</script></body></html>'''
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def send(self, body, ctype='application/json'):
        b = body if isinstance(body, bytes) else body.encode(); self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        u = urllib.parse.urlparse(self.path); qs = urllib.parse.parse_qs(u.query)
        if u.path == '/': return self.send(HTML, 'text/html; charset=utf-8')
        if u.path == '/status': return self.send(json.dumps({'phase': STATE['phase'], 'site': STATE['site'], 'log': STATE['log'][-200:]}))
        if u.path == '/search':
            q = qs.get('q', [''])[0]
            try:
                req = urllib.request.Request('https://nominatim.openstreetmap.org/search?' + urllib.parse.urlencode({'q': q, 'format': 'json', 'limit': 8}), headers=UA)
                res = json.loads(urllib.request.urlopen(req, timeout=20).read()); return self.send(json.dumps([{'display_name': r['display_name'], 'lat': r['lat'], 'lon': r['lon']} for r in res]))
            except Exception as e: return self.send(json.dumps([]))
        self.send_response(404); self.end_headers()
    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0)); body = json.loads(self.rfile.read(n) or b'{}')
        if self.path == '/build':
            if STATE['phase'] in ('building', 'starting'): return self.send('{"ok":false}')
            name = re.sub(r'[^A-Za-z0-9_-]', '_', body.get('name', 'site1')) or 'site1'
            threading.Thread(target=run_build, args=(name, float(body['lat']), float(body['lon']), float(body.get('half', 170))), daemon=True).start(); return self.send('{"ok":true}')
        if self.path == '/drive':
            if not STATE['site'] or STATE['phase'] in ('building', 'starting', 'driving'): log('nothing built yet or already running'); return self.send('{"ok":false}')
            threading.Thread(target=run_drive, args=(STATE['site'],), daemon=True).start(); return self.send('{"ok":true}')
        if self.path == '/stop': stop_all(); return self.send('{"ok":true}')
        self.send_response(404); self.end_headers()
if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--port', type=int, default=8765); a = ap.parse_args()
    me = os.getpid(); r = subprocess.run(['pgrep', '-f', r'^\S*python[0-9.]* \S*site_app\.py'], capture_output=True, text=True)   # replace a stale instance: only python processes running this file (never ourselves, never a shell wrapper)
    for pid in r.stdout.split():
        if int(pid) != me: subprocess.run(['kill', pid])
    ThreadingHTTPServer.allow_reuse_address = True
    for attempt in range(10):
        try: srv = ThreadingHTTPServer(('0.0.0.0', a.port), H); break
        except OSError: time.sleep(1)
    else: raise SystemExit(f'port {a.port} is busy: pkill -f "site_ap[p].py" or use --port')
    print(f'open http://localhost:{a.port}', flush=True); srv.serve_forever()
