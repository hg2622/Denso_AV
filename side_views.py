"""Small second window with two camera views of the hero car, side by side.
Works next to drive.sh (free roam) or a Bench2Drive episode — anything that has a hero vehicle.
  python side_views.py [VIEW1] [VIEW2] [--w 420] [--h 236]
views: left (left mirror), right (right mirror), rear, front, bev (top-down), chase
Ctrl+C or close the window to stop (cameras are removed).
"""
import argparse, sys, time
import numpy as np, carla, pygame

VIEWS = {  # name: (location offset in car frame, rotation)
    'left':  (carla.Location(x=0.5, y=-1.0, z=1.1), carla.Rotation(yaw=210)),
    'right': (carla.Location(x=0.5, y=1.0, z=1.1),  carla.Rotation(yaw=150)),
    'rear':  (carla.Location(x=-2.2, z=1.3),        carla.Rotation(yaw=180)),
    'front': (carla.Location(x=1.5, z=1.5),         carla.Rotation()),
    'bev':   (carla.Location(z=35.0),               carla.Rotation(pitch=-90)),
    'chase': (carla.Location(x=-6.0, z=3.0),        carla.Rotation(pitch=-12)),
}
ap = argparse.ArgumentParser()
ap.add_argument('views', nargs='*', default=['left', 'right'])
ap.add_argument('--w', type=int, default=420); ap.add_argument('--h', type=int, default=236)
ap.add_argument('--fov', type=float, default=90); ap.add_argument('--port', type=int, default=2000)
args = ap.parse_args()
views = args.views[:2] if len(args.views) >= 2 else (args.views + ['right'])[:2]
for v in views:
    if v not in VIEWS: sys.exit(f'unknown view {v}; choose from {list(VIEWS)}')

client = carla.Client('localhost', args.port); client.set_timeout(30.0); world = client.get_world()
try: world.wait_for_tick(seconds=10.0)   # sync-mode worlds: no actors visible before the first tick
except RuntimeError: pass
def find_hero():
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == 'hero': return a
    return None
hero = None
print('waiting for a hero vehicle ...', end='', flush=True)
while hero is None:
    hero = find_hero(); time.sleep(0.5)
print(f' found {hero.type_id} id={hero.id}')

frames = {}
def cb(name):
    def f(img):
        a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, :3][:, :, ::-1]
        frames[name] = a.swapaxes(0, 1).copy()  # pygame wants (w, h, 3)
    return f
bp = world.get_blueprint_library().find('sensor.camera.rgb')
bp.set_attribute('image_size_x', str(args.w)); bp.set_attribute('image_size_y', str(args.h)); bp.set_attribute('fov', str(args.fov))
cams = []
for name in views:
    loc, rot = VIEWS[name]
    cam = world.spawn_actor(bp, carla.Transform(loc, rot), attach_to=hero); cam.listen(cb(name)); cams.append(cam)

pygame.init()
screen = pygame.display.set_mode((args.w * 2 + 4, args.h)); pygame.display.set_caption(f'{views[0]} | {views[1]}')
font = pygame.font.SysFont('monospace', 16, bold=True); clock = pygame.time.Clock()
try:
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE): running = False
        if not hero.is_alive:  # hero respawned (Backspace in manual_control) -> re-attach
            for c in cams: c.stop(); c.destroy()
            cams.clear(); hero = None
            while hero is None: hero = find_hero(); time.sleep(0.3)
            for name in views:
                loc, rot = VIEWS[name]
                cam = world.spawn_actor(bp, carla.Transform(loc, rot), attach_to=hero); cam.listen(cb(name)); cams.append(cam)
        screen.fill((0, 0, 0))
        for i, name in enumerate(views):
            if name in frames:
                surf = pygame.surfarray.make_surface(frames[name])
                if name in ('left', 'right'): surf = pygame.transform.flip(surf, True, False)  # mirror image
                screen.blit(surf, (i * (args.w + 4), 0))
            screen.blit(font.render(name, True, (255, 255, 0)), (i * (args.w + 4) + 6, 4))
        pygame.display.flip(); clock.tick(30)
except KeyboardInterrupt:
    pass
finally:
    for c in cams:
        if c.is_alive: c.stop(); c.destroy()
    pygame.quit()
