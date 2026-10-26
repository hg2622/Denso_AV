"""CARLA decoration for mall.scenic. Geometry is intentionally approximate."""
from pathlib import Path
import json
import math
import carla
from scenic.simulators.carla.simulator import CarlaSimulator
from scenic.domains.driving.simulators import DrivingSimulator

ROOT = Path(__file__).resolve().parent

class MallSimulator(CarlaSimulator):
    def __init__(self, carla_map, map_path, address='127.0.0.1', port=2000,
                 timeout=60, render=False, timestep=0.05,
                 geometry='blocks', mesh_path=''):
        # Same interface as Scenic's CARLA adapter, but disable generated walls:
        # a perimeter wall would incorrectly prevent the entrance crossing.
        DrivingSimulator.__init__(self)
        self.environment_actors=[]
        if carla_map is not None:
            raise ValueError('This scene uses its supplied OpenDRIVE map')
        self.client=carla.Client(address,port)
        self.client.set_timeout(timeout)
        self.world=self.client.generate_opendrive_world(
            Path(map_path).read_text(),
            carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0,
                wall_height=0.0, additional_width=0.6, smooth_junctions=True,
                enable_mesh_visibility=True))
        self.timestep=timestep
        self.tm=self.client.get_trafficmanager(port+6000)
        self.tm.set_synchronous_mode(True)
        settings=self.world.get_settings()
        settings.synchronous_mode=True
        settings.fixed_delta_seconds=timestep
        self.world.apply_settings(settings)
        self.render=render
        self.record=''
        self.scenario_number=0
        try:
            if geometry == 'mesh':
                if not mesh_path:
                    raise ValueError('mesh mode requires --mesh-path; see README.md')
                self._mesh(mesh_path, 1, 0, 0, 0)
            elif geometry == 'blocks':
                self._blocks()
            elif geometry != 'roads-only':
                raise ValueError('Unknown geometry mode: '+geometry)
            self.world.get_spectator().set_transform(carla.Transform(
                carla.Location(x=0,y=-80,z=650),carla.Rotation(pitch=-90,yaw=-90)))
        except Exception:
            self.destroy()
            raise

    def _mesh(self, path, scale, x, y, z):
        try:
            bp=self.world.get_blueprint_library().find('static.prop.mesh')
        except IndexError as exc:
            raise RuntimeError('This CARLA build lacks static.prop.mesh. Use a compatible build '
                               'or import the environment into a CARLA map in Unreal.') from exc
        bp.set_attribute('mesh_path',path)
        bp.set_attribute('scale',str(scale))
        actor=self.world.try_spawn_actor(bp,carla.Transform(carla.Location(x=x,y=-y,z=z)))
        if actor is None:
            raise RuntimeError('Could not spawn environment mesh: '+path)
        self.environment_actors.append(actor)
        actor.set_simulate_physics(False)
        return actor

    def _blocks(self):
        # The engine's Cube has 1 m sides; uniform scale yields square blocks.
        # This asset must be cooked into the installed CARLA package.
        cube='/Engine/BasicShapes/Cube.Cube'
        probe=self._mesh(cube,20,0,0,10)
        self.world.tick()
        extent=probe.bounding_box.extent
        if min(extent.x,extent.y,extent.z) < 0.1:
            raise RuntimeError('The engine Cube mesh is unavailable in this package. '
                               'Import mall_environment.obj and use --geometry mesh --mesh-path PATH; '
                               'see README.md. No full scene has been created.')
        data=json.loads((ROOT/'layout.json').read_text())
        # Large cubes sunk underground provide a flat surface at z=-0.02 m.
        for x in range(-350,351,100):
            for y in range(-250,451,100): self._mesh(cube,100,x,y,-50.02)
        # One probe already occupies a center cell; skip that cell below.
        for b in data['buildings']:
            nx=math.ceil(b['w']/20); ny=math.ceil(b['d']/20)
            for ix in range(nx):
                for iy in range(ny):
                    x=b['x']-b['w']/2+10+ix*20
                    y=b['y']-b['d']/2+10+iy*20
                    if abs(x)<0.01 and abs(y)<0.01: continue
                    self._mesh(cube,20,x,y,10)
        # In block mode the 30 m deep entrance wing rounds up to 40 m.
        # Painted parking stalls are provided by the optional imported OBJ.

    def destroy(self):
        for actor in reversed(self.environment_actors):
            if actor.is_alive: actor.destroy()
        self.environment_actors=[]
        super().destroy()
