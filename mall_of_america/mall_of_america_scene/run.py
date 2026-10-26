"""Launch a single Scenic episode against an already running CARLA server."""
from pathlib import Path
import argparse
import random
import scenic

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1')
    p.add_argument('--port',type=int,default=2000)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--ego-speed',type=float,default=6)
    p.add_argument('--trigger-distance',type=float,default=30)
    p.add_argument('--parked-stride',type=int,default=4,help='1=164 parked cars; 4=41 cars')
    p.add_argument('--geometry',choices=['blocks','mesh','roads-only'],default='blocks')
    p.add_argument('--mesh-path',default='',help='Imported Unreal static mesh asset path')
    p.add_argument('--render',action='store_true',help='Open Scenic ego camera pygame window')
    p.add_argument('--check',action='store_true',help='Compile and sample without connecting to CARLA')
    a=p.parse_args()
    if a.seconds<=0 or a.ego_speed<=0 or a.trigger_distance<=0 or a.parked_stride<1:
        p.error('seconds, speed, trigger distance and stride must be positive')
    random.seed(a.seed)
    scenario=scenic.scenarioFromFile(str(Path(__file__).with_name('mall.scenic')),
        mode2D=True,params=dict(address=a.host,port=a.port,duration=a.seconds,
            ego_speed=a.ego_speed,trigger_distance=a.trigger_distance,
            parked_stride=a.parked_stride,geometry=a.geometry,mesh_path=a.mesh_path,
            render=int(a.render)))
    scene,iterations=scenario.generate(maxIterations=100)
    print(f'Sampled {len(scene.objects)} actors in {iterations} iteration(s).')
    if a.check:
        print('Static check passed; no CARLA server was contacted.'); return
    simulator=None
    try:
        # This replaces the current server world with the supplied OpenDRIVE map.
        simulator=scenario.getSimulator()
        result=simulator.simulate(scene,maxSteps=int(a.seconds/0.05)+1,maxIterations=1)
        if result is None: raise RuntimeError('Scenic rejected/failed the simulation; inspect output above.')
        print('Episode finished:',result.result.terminationReason)
    finally:
        if simulator is not None: simulator.destroy()

if __name__=='__main__': main()
