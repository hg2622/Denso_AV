"""Aerial QA capture of the currently loaded geo scene: python geo_scenes/aerial.py out.png [half_m=170] [cx=0] [cy=0] [pitch=-90]"""
import sys, time, math; sys.path.insert(0,'/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np
out = sys.argv[1]; half = float(sys.argv[2]) if len(sys.argv) > 2 else 170.0; cx = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0; cy = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0; pitch = float(sys.argv[5]) if len(sys.argv) > 5 else -90.0
c = carla.Client('localhost', 2000); c.set_timeout(60); w = c.get_world(); lib = w.get_blueprint_library()
bp = lib.find('sensor.camera.rgb'); bp.set_attribute('image_size_x', '1500'); bp.set_attribute('image_size_y', '1500'); bp.set_attribute('fov', '60')
z = half / math.tan(math.radians(30))
cam = w.spawn_actor(bp, carla.Transform(carla.Location(x=cx, y=cy, z=z), carla.Rotation(pitch=pitch, yaw=-90)))   # yaw -90 -> image up = north (CARLA -y)
got = []; cam.listen(lambda im: got.append(im))
sync = w.get_settings().synchronous_mode
for _ in range(60):
    if sync: w.tick()
    else: time.sleep(0.05)
    if len(got) >= 3: break
im = got[-1]; arr = np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3]
import cv2; cv2.imwrite(out, arr); cam.stop(); cam.destroy(); print('saved', out, 'from z', round(z))
