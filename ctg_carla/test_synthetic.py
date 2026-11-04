"""Synthetic sanity + latency test for the CTG++ wrapper: a straight 3-lane road, 8 cars at 10 m/s with 3 s of history."""
import sys, time, math, numpy as np
sys.path.insert(0, '/home/all/carla_ws/ctg_carla')
from ctg_model import CTGPlusPlus, LaneGraph, DrivableRaster, HIST_T, DT
stride = int(sys.argv[1]) if len(sys.argv) > 1 else 1; nsamp = int(sys.argv[2]) if len(sys.argv) > 2 else 5
lanes = []
for k, yl in enumerate((-3.5, 0.0, 3.5)):
    xs = np.arange(-150, 300, 1.0); lanes.append(np.stack([xs, np.full_like(xs, yl), np.zeros_like(xs), np.zeros_like(xs)], -1))
lg = LaneGraph(lanes)
img = np.zeros((int(60 / 0.5), int(450 / 0.5)), np.float32); img[int((30 - 5.25) / 0.5):int((30 + 5.25) / 0.5), :] = 1.0   # 10.5 m wide road
dr = DrivableRaster(img, origin_xy=(-150.0, -30.0))
rng = np.random.default_rng(0); agents = []
for i in range(8):
    x0 = rng.uniform(-20, 40); y0 = [-3.5, 0, 3.5][i % 3]; v = rng.uniform(6, 12)
    tt = (np.arange(HIST_T) - (HIST_T - 1)) * DT
    hist = np.stack([x0 + v * tt, np.full(HIST_T, y0), np.zeros(HIST_T), np.full(HIST_T, v), np.zeros(HIST_T)], -1)
    agents.append(dict(hist=hist, extent=(4.5, 1.9, 1.6)))
t = time.time(); m = CTGPlusPlus(num_samples=nsamp, diffusion_stride=stride); print(f'model loaded {time.time()-t:.1f}s  stride={stride} samples={nsamp}')
t = time.time(); b = m.build_batch(agents, lg, dr); torch_sync = __import__('torch').cuda.synchronize; torch_sync(); print(f'batch built {time.time()-t:.2f}s; image {tuple(b["image"].shape)} lanes {tuple(b["extras"]["closest_lane_point"].shape)} nan-lane-frac {float(np.isnan(b["extras"]["closest_lane_point"].cpu().numpy()).mean()):.2f}')
for label, guid in (('no guidance', None),
                    ('avoid+map', [dict(name='agent_collision', weight=1.0, params=dict(num_disks=5, buffer_dist=0.2, decay_rate=0.9, guide_moving_speed_th=0.5, excluded_agents=None), agents=None),
                                   dict(name='map_collision', weight=1.0, params=dict(num_points_lw=(10, 10), decay_rate=0.9, guide_moving_speed_th=0.5), agents=None)]),
                    ('car2 hits car0', [dict(name='gptcollision', weight=1.0, params=dict(target_ind=2, ref_ind=0, collision_radius=1.0), agents=None)])):
    m.set_guidance(guid, b); m.reset()
    for rep in range(2):
        t = time.time(); out = m.plan(b); torch_sync(); dt_ = time.time() - t
    p = out['positions']; d = np.linalg.norm(p[:, -1], axis=-1)
    print(f'{label:16s} latency {dt_:.2f}s | end displacement per agent (m): {np.round(d, 1)} | mean lateral |y| at 5s: {np.abs(p[:, -1, 1]).mean():.2f}')
    if label == 'car2 hits car0':
        # distance between car 2 and car 0 over the horizon, in world frame
        W = b['world_from_agent'][0].cpu().numpy()
        def world(i): return p[i] @ W[i][:2, :2].T + W[i][:2, 2]
        dd = np.linalg.norm(world(2) - world(0), axis=-1); print(f'   car2-car0 distance over horizon: start {dd[0]:.1f} m -> min {dd.min():.1f} m at t={dd.argmin()*DT:.1f}s (initial gap {np.linalg.norm(agents[2]["hist"][-1,:2]-agents[0]["hist"][-1,:2]):.1f} m)')
