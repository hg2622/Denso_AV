"""Meaningful offline checks; does not claim a running simulator was tested."""
from pathlib import Path
import json
import random
import carla
import scenic
import trimesh

ROOT=Path(__file__).resolve().parent

def main():
    network=carla.Map('mall', (ROOT/'mall.xodr').read_text())
    waypoints=network.generate_waypoints(5)
    ids={w.road_id for w in waypoints}
    assert ids==set(range(1,9)), ids
    assert len(network.get_topology())==16
    # Walk both directions around the full loop and verify no road end stops us.
    for lane_id in [-1,1]:
        wp=next(w for w in waypoints if w.road_id==1 and w.lane_id==lane_id)
        visited=set()
        for _ in range(500):
            visited.add(wp.road_id)
            nxt=wp.next(5)
            assert len(nxt)==1, (wp.road_id,wp.s,len(nxt))
            wp=nxt[0]
        assert visited==ids, visited
    print(f'PASS CARLA map parser: {len(waypoints)} waypoints, 16 topology links, both loop directions connected.')
    data=json.loads((ROOT/'layout.json').read_text())
    for b in data['buildings']:
        for w in waypoints:
            x=w.transform.location.x; y=-w.transform.location.y
            assert not (abs(x-b['x'])<b['w']/2+1.5 and abs(y-b['y'])<b['d']/2+1.5), b['name']
    print('PASS authored building footprints clear all sampled driving centerlines by vehicle half-width.')
    mesh=trimesh.load(ROOT/'mall_environment.obj',force='mesh',process=False)
    assert mesh.is_winding_consistent
    assert mesh.volume>0
    print(f'PASS OBJ load and face orientation: {len(mesh.faces)} triangles; positive signed volume.')
    for stride,expected in [(4,45),(1,168)]:
        for seed in [1,7,42]:
            random.seed(seed)
            scenario=scenic.scenarioFromFile(str(ROOT/'mall.scenic'),mode2D=True,params={'parked_stride':stride})
            scene,_=scenario.generate(maxIterations=100)
            assert len(scene.objects)==expected,(stride,len(scene.objects))
            assert scene.objects[-1].position.z==0
    print('PASS Scenic 3.1.1 compilation and sampling: three seeds, both parking densities, 45/168 actors.')
    print('NOT TESTED: CARLA server execution, cooked mesh availability, Unreal import, physics, control, collisions.')

if __name__=='__main__':main()
