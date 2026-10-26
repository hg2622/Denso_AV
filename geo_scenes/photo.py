"""Ground-level QA photos of the loaded scene: python geo_scenes/photo.py out.png x y yaw [pitch=-5] [z=2.5] [fov=90]"""
import sys, time; sys.path.insert(0,'/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, cv2
out, x, y, yaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); pitch = float(sys.argv[5]) if len(sys.argv) > 5 else -5.0; z = float(sys.argv[6]) if len(sys.argv) > 6 else 2.5; fov = sys.argv[7] if len(sys.argv) > 7 else '90'
c = carla.Client('localhost', 2000); c.set_timeout(60); w = c.get_world(); bp = w.get_blueprint_library().find('sensor.camera.rgb'); bp.set_attribute('image_size_x', '1100'); bp.set_attribute('image_size_y', '560'); bp.set_attribute('fov', fov)
cam = w.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw))); got = []; cam.listen(lambda im: got.append(im))
for _ in range(60):
    w.tick() if w.get_settings().synchronous_mode else time.sleep(0.05)
    if len(got) >= 3: break
im = got[-1]; cv2.imwrite(out, np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3]); cam.stop(); cam.destroy(); print('saved', out)
