"""List actors with role_name=hero on the running server, plus counts of NPC vehicles/walkers."""
import carla
c = carla.Client('localhost', 2000); c.set_timeout(10.0); w = c.get_world()
try: w.wait_for_tick(seconds=10.0)
except RuntimeError: pass
actors = w.get_actors()
heroes = [a for a in actors.filter('vehicle.*') if a.attributes.get('role_name') == 'hero']
print('map:', w.get_map().name.split('/')[-1])
for h in heroes:
    l = h.get_location(); print(f'HERO: {h.type_id} id={h.id} at ({l.x:.0f}, {l.y:.0f})')
if not heroes: print('no hero vehicle')
print('vehicles:', len(actors.filter('vehicle.*')) - len(heroes), ' walkers:', len(actors.filter('walker.pedestrian.*')))
if heroes:
    hl = heroes[0].get_location()
    near = [a for a in actors.filter('vehicle.*') if a.attributes.get('role_name') != 'hero' and a.get_location().distance(hl) < 300 and not a.attributes.get('role_name','').startswith('dormant')]
    print('NPC vehicles within 300 m of hero:', len(near))
