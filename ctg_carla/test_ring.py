import sys, numpy as np, torch
sys.path.insert(0, '/home/all/carla_ws/ctg_carla')
from ctg_model import CTGPlusPlus, LaneGraph, DrivableRaster, HIST_T, DT
# curved road: a 60 m radius ring, 3 cars following it at 8 m/s -> the correct action is a constant yaw rate (~0.13 rad/s)
R = 60.0; th = np.linspace(-np.pi, np.pi, 400, endpoint=False)
lane = np.stack([R*np.cos(th), R*np.sin(th), np.zeros_like(th), th + np.pi/2], -1); lg = LaneGraph([lane])
img = np.zeros((300, 300), np.float32); yy, xx = np.mgrid[0:300, 0:300]; rr = np.hypot((xx-150)*0.5, (yy-150)*0.5); img[(rr > R-3) & (rr < R+3)] = 1; dr = DrivableRaster(img, origin_xy=(-75.0, -75.0))
agents = []
for k, phi0 in enumerate((0.0, 0.4, 0.8)):
    v = 8.0; tt = (np.arange(HIST_T) - (HIST_T-1)) * DT; phi = phi0 + v/R*tt
    agents.append(dict(hist=np.stack([R*np.cos(phi), R*np.sin(phi), phi + np.pi/2, -v*np.sin(phi), v*np.cos(phi)], -1), extent=(4.5, 1.9, 1.6)))
m = CTGPlusPlus(num_samples=3, diffusion_stride=1); b = m.build_batch(agents, lg, dr); pol = m.policy
r = m.plan(b)
pos = r['positions']; print('RING via plan: agent0 end pos', pos[0, -1].round(1).tolist(), '(ideal x=%.1f y=%.1f)' % (R*np.sin(8*5.2/R), R*(1-np.cos(8*5.2/R))), '| yaw at end %.2f rad (ideal %.2f)' % (r['yaws'][0, -1], 8*5.2/R))
