"""Record a 3rd-person video (chase cam + top-down) of whatever hero car exists on a CARLA server, until the hero disappears.
  python record_hero.py --port 8848 --out run.mp4 [--fps 20]
Attaches as a passive client (works in lock-step worlds: one frame per world tick). Stops on Ctrl-C/SIGTERM or when the
hero actor is gone for >5 s. Written with the b2d/lead env (needs carla + opencv).
"""
import argparse, os, sys, time, math, signal
import numpy as np, cv2
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla
ap = argparse.ArgumentParser(); ap.add_argument('--port', type=int, default=2000); ap.add_argument('--out', required=True)
ap.add_argument('--fps', type=float, default=20.0); ap.add_argument('--wait', type=float, default=600, help='seconds to wait for a hero to appear')
ap.add_argument('--label', default=''); ap.add_argument('--meta', default='', help='write per-frame hero/camera poses + collision events (JSONL) for annotate_run.py'); ap.add_argument('--multi', action='store_true', help='keep going after the hero disappears: one video per hero appearance (<out>_00.mp4, _01.mp4, ...)')
a = ap.parse_args()
stop = [False]; signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True)); signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
c = carla.Client('localhost', a.port); c.set_timeout(20.0)
def find_hero(w):
    for v in w.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') == 'hero': return v
    return None
def live_port(hint):
    # the Bench2Drive evaluators relaunch their CARLA on another port when the first attempt fails (seen: 8848 -> 8850);
    # take the hint if a server listens on it, else the most recently started CarlaUE4 server
    import subprocess, re
    try: out = subprocess.run(['pgrep', '-fa', 'CarlaUE4-Linux-Shippin[g]'], capture_output=True, text=True).stdout
    except Exception: return hint
    found = [(int(l.split()[0]), int(m.group(1))) for l in out.splitlines() for m in [re.search(r'-carla-rpc-port=(\d+)', l)] if m]
    if not found: return hint
    if any(pt == hint for _, pt in found): return hint
    return max(found)[1]
def wait_hero(timeout):
    t0 = time.time(); n_try = 0
    while time.time() - t0 < timeout and not stop[0]:
        n_try += 1
        try:
            port = live_port(a.port)
            if port != a.port: print(f'recorder: using CARLA on port {port} (asked for {a.port})', flush=True); a.port = port
            c = carla.Client('localhost', a.port); c.set_timeout(2.0)    # fresh client each attempt: a client created while the server restarts stays dead
            c.get_server_version()                                     # cheap probe (2 s): a server still loading its map must not be hammered with 10 s get_world()
            c.set_timeout(10.0); w = c.get_world()                     # calls - every abandoned call occupies a CARLA RPC worker and the server stops answering
            try: w.wait_for_tick(seconds=1.0)      # sync worlds: a fresh client sees actors only after a tick
            except RuntimeError: pass
            acts = w.get_actors().filter('vehicle.*'); hero = find_hero(w)
            if hero is not None: return w, hero
            if n_try % 10 == 0: print(f'recorder: waiting for hero ({int(time.time()-t0)} s): map {w.get_map().name.split("/")[-1]}, {len(acts)} vehicles', flush=True)
        except RuntimeError as e:
            if n_try % 10 == 0: print(f'recorder: server not ready ({str(e)[:50]})', flush=True)
            time.sleep(3.0); continue
        time.sleep(0.5)
    return None, None

def record(w, hero, out):
    print(f'recorder: hero {hero.id} on port {a.port} -> {out}', flush=True)
    W, H, BW = 800, 450, 450
    specs = {'chase': (carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), W, H), 'bev': (carla.Location(z=48.0), carla.Rotation(pitch=-90), BW, H)}
    frames, cams = {}, []; lib = w.get_blueprint_library()
    for name, (loc, rot, cw, ch) in specs.items():
        b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(cw)); b.set_attribute('image_size_y', str(ch)); b.set_attribute('fov', '90')
        cam = w.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
        cam.listen(lambda img, n=name: frames.__setitem__(n, (img.frame, np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3].copy())))
        cams.append(cam)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    meta = open(a.meta, 'w') if a.meta else None; events = []
    if meta is not None:
        import json
        col = w.spawn_actor(lib.find('sensor.other.collision'), carla.Transform(), attach_to=hero)
        col.listen(lambda ev: events.append({'frame': ev.frame, 'other': ev.other_actor.type_id, 'x': ev.other_actor.get_location().x, 'y': ev.other_actor.get_location().y})); cams.append(col)
        meta.write(json.dumps({'kind': 'cams', 'chase': {'w': W, 'h': H, 'fov': 90}, 'bev': {'w': BW, 'h': H, 'fov': 90}}) + '\n')
    # raw frames go to an MJPG .avi (readable even if this process is aborted by a CARLA stream thread when the evaluator kills its
    # server - an unfinalised mp4 is not); the .mp4 is produced from it at a clean exit, annotate_run.py falls back to the .avi otherwise
    avi = os.path.splitext(out)[0] + '.avi'
    vw = cv2.VideoWriter(avi, cv2.VideoWriter_fourcc(*'MJPG'), a.fps, (W + BW, H)); n = 0; gone_since = None; last_frame = -1
    try:
        while not stop[0]:
            try: snap = w.wait_for_tick(seconds=5.0)
            except RuntimeError:
                if gone_since is None: gone_since = time.time()
                elif time.time() - gone_since > 15:
                    # stale world handle (the evaluator loaded another map / restarted its server): reconnect instead of quitting,
                    # the batch scripts treat a recorder exit as the end of the run
                    for cam in cams:
                        try: cam.stop(); cam.destroy()
                        except Exception: pass
                    cams.clear(); w2, hero2 = wait_hero(90)
                    if hero2 is None: break
                    w, hero = w2, hero2; lib = w.get_blueprint_library(); frames.clear(); last_frame = -1
                    for name, (loc, rot, cw, ch) in specs.items():
                        b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(cw)); b.set_attribute('image_size_y', str(ch)); b.set_attribute('fov', '90')
                        cam = w.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
                        cam.listen(lambda img, n=name: frames.__setitem__(n, (img.frame, np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3].copy()))); cams.append(cam)
                    if meta is not None:
                        col = w.spawn_actor(lib.find('sensor.other.collision'), carla.Transform(), attach_to=hero)
                        col.listen(lambda ev: events.append({'frame': ev.frame, 'other': ev.other_actor.type_id, 'x': ev.other_actor.get_location().x, 'y': ev.other_actor.get_location().y})); cams.append(col)
                    print(f'recorder: reconnected, hero {hero.id}', flush=True); gone_since = None
                continue
            try: alive = hero.is_alive and find_hero(w) is not None
            except RuntimeError: alive = False
            if not alive:
                gone_since = gone_since or time.time()
                for cam in cams:   # stop + destroy the streams NOW: if the evaluator shuts its server down while a stream is open, the stream thread aborts the whole process (core dump, unfinalised mp4)
                    try: cam.stop(); cam.destroy()
                    except Exception: pass
                cams.clear()
                if time.time() - gone_since > 3:
                    # the evaluators sometimes replace the ego right after spawning it: re-acquire instead of stopping
                    for cam in cams:
                        try: cam.stop(); cam.destroy()
                        except Exception: pass
                    cams.clear(); w2, hero2 = wait_hero(45)
                    if hero2 is None: break
                    w, hero = w2, hero2; lib = w.get_blueprint_library(); frames.clear(); last_frame = -1
                    for name, (loc, rot, cw, ch) in specs.items():
                        b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(cw)); b.set_attribute('image_size_y', str(ch)); b.set_attribute('fov', '90')
                        cam = w.spawn_actor(b, carla.Transform(loc, rot), attach_to=hero)
                        cam.listen(lambda img, n=name: frames.__setitem__(n, (img.frame, np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3].copy()))); cams.append(cam)
                    if meta is not None:
                        col = w.spawn_actor(lib.find('sensor.other.collision'), carla.Transform(), attach_to=hero)
                        col.listen(lambda ev: events.append({'frame': ev.frame, 'other': ev.other_actor.type_id, 'x': ev.other_actor.get_location().x, 'y': ev.other_actor.get_location().y})); cams.append(col)
                    print(f'recorder: re-acquired hero {hero.id}', flush=True); gone_since = None
                continue
            gone_since = None
            if snap.frame == last_frame or 'chase' not in frames or 'bev' not in frames: continue
            _tw = time.time()   # the camera images for this frame arrive a moment after the tick: wait (<=0.25 s) so image and state are the same frame
            while (frames['chase'][0] < snap.frame or frames['bev'][0] < snap.frame) and time.time() - _tw < 0.25: time.sleep(0.005)
            last_frame = snap.frame
            canvas = np.zeros((H, W + BW, 3), dtype=np.uint8); canvas[:, :W] = frames['chase'][1]; canvas[:, W:] = frames['bev'][1]
            v = hero.get_velocity(); ctl = hero.get_control()
            txt = f'{a.label}  {3.6*math.hypot(v.x, v.y):5.1f} km/h  thr {ctl.throttle:.2f} brk {ctl.brake:.2f} steer {ctl.steer:+.2f}  t={snap.timestamp.elapsed_seconds:.1f}s'
            cv2.putText(canvas, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4); cv2.putText(canvas, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if meta is not None:
                ht = hero.get_transform(); rec = {'kind': 'frame', 'i': n, 'frame': snap.frame, 'img_frame': int(frames['chase'][0]), 'bev_frame': int(frames['bev'][0]), 't': snap.timestamp.elapsed_seconds, 'speed': 3.6*math.hypot(v.x, v.y), 'throttle': ctl.throttle, 'brake': ctl.brake, 'steer': ctl.steer,
                                                  'hero': ht.get_matrix(), 'chase': cams[0].get_transform().get_inverse_matrix(), 'bev': cams[1].get_transform().get_inverse_matrix()}
                while events: e = events.pop(0); e['kind'] = 'collision'; e['i'] = n; e['t'] = snap.timestamp.elapsed_seconds; meta.write(json.dumps(e) + '\n')
                meta.write(json.dumps(rec) + '\n')
            vw.write(canvas); n += 1
    finally:
        vw.release()
        if meta is not None: meta.close()
        try:
            import subprocess as _sp
            if n > 0 and _sp.run([__import__('shutil').which('ffmpeg') or '/home/all/miniconda3/envs/sam3/bin/ffmpeg', '-y', '-loglevel', 'error', '-i', avi, '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p', out], capture_output=True).returncode == 0: os.remove(avi)
        except Exception as e: print('recorder: mp4 conversion failed, keeping the .avi:', e, flush=True)
        for cam in cams:
            try: cam.stop(); cam.destroy()
            except Exception: pass
        print(f'recorder: wrote {n} frames -> {out}', flush=True)
    return n

k = 0
while not stop[0]:
    w, hero = wait_hero(a.wait if k == 0 else 600)
    if hero is None:
        if k == 0: print('recorder: no hero appeared', flush=True); sys.exit(1)
        break
    out = a.out if not a.multi else a.out.replace('.mp4', f'_{k:02d}.mp4')
    record(w, hero, out); k += 1
    if not a.multi: break
