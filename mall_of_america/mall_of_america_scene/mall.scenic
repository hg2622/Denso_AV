# Scenic 3.1.1. Run through run.py; or use scenic --2d mall.scenic --simulate.
import json
from pathlib import Path
from mall_support import MallSimulator

param map = localPath('mall.xodr')
param carla_map = None
param render = 0
param timestep = 0.05
param weather = 'ClearNoon'
param ego_speed = 6.0
param trigger_distance = 30.0
param pedestrian_speed = 1.3
param duration = 60.0
param mesh_path = ''
param geometry = 'blocks'
param parked_stride = 4

model scenic.simulators.carla.model

# Override the stock simulator to add the environment when the map loads.
simulator MallSimulator(
    carla_map=None, map_path=globalParameters.map,
    address=globalParameters.address, port=int(globalParameters.port),
    timeout=60, render=bool(globalParameters.render), timestep=globalParameters.timestep,
    geometry=globalParameters.geometry, mesh_path=globalParameters.mesh_path)

behavior EntranceDrive():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ego_speed)
    interrupt when (distance from self to shopper) < 18 and -231 < shopper.position.y < -219:
        while (distance from self to shopper) < 18 and -231 < shopper.position.y < -219:
            take SetThrottleAction(0), SetBrakeAction(1)

behavior TriggeredCrossing():
    while (distance from self to ego) > globalParameters.trigger_distance:
        wait
    while self.position.y < -217:
        take SetWalkingDirectionAction(0), SetWalkingSpeedAction(globalParameters.pedestrian_speed)
    take SetWalkingSpeedAction(0)
    while True:
        wait

ego = new Car at -80 @ -226.75,
    facing -90 deg,
    with blueprint 'vehicle.tesla.model3',
    with rolename 'hero',
    with behavior EntranceDrive()

shopper = new Pedestrian at 40 @ -232,
    facing 0 deg,
    with blueprint 'walker.pedestrian.0001',
    with regionContainedIn None,
    with behavior TriggeredCrossing()

# Opposing traffic and a car ahead, all using the connected road loop.
new Car at 140 @ -223.25, facing 90 deg,
    with blueprint 'vehicle.tesla.model3',
    with behavior FollowLaneBehavior(target_speed=5)
new Car at 20 @ -226.75, facing -90 deg,
    with blueprint 'vehicle.tesla.model3',
    with behavior FollowLaneBehavior(target_speed=7)

_layout = json.loads(Path(localPath('layout.json')).read_text())
# Parked vehicles are fixed and do not require a lane.
for _p in _layout['parked_cars'][::int(globalParameters.parked_stride)]:
    new Car at _p['x'] @ _p['y'], facing _p['heading'] deg,
        with blueprint 'vehicle.tesla.model3',
        with regionContainedIn None,
        with snapToGround False,
        with physics False

terminate after globalParameters.duration seconds
