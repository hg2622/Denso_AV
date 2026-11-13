"""Layout schematic of the AttentionTwin physical system: one Level-2 vehicle with the driver in the cabin, the vehicle
sensors on the CAN bus, and the environment within the ~100 m perception range. Pure matplotlib."""
import sys, os, math
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge, Circle, FancyBboxPatch, FancyArrowPatch
out = sys.argv[1] if len(sys.argv) > 1 else 'figs'; os.makedirs(out, exist_ok=True)

fig, (ax, az) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={'width_ratios': [1.25, 1]})
# ---- left: the vehicle and its surroundings, top view, metres
L, W = 4.7, 1.85
for rr, lab, (tx, ty) in ((100, '100 m perception range', (-58, 41)), (50, '50 m', (36, -36)), (20, '20 m', (14, -15))):
    ax.add_patch(Circle((0, 0), rr, fill=False, ec='gray', ls='--', lw=0.9)); ax.text(tx, ty, lab, fontsize=8, color='gray', ha='left')
# exterior camera coverage (Tesla-like: 3 forward, 2 forward-side, 2 rear-side, 1 rear)
for ang, fov, rng, col in ((0, 120, 60, '#1f77b4'), (0, 50, 100, '#1f77b4'), (55, 90, 60, '#1f77b4'), (-55, 90, 60, '#1f77b4'), (140, 80, 50, '#1f77b4'), (-140, 80, 50, '#1f77b4'), (180, 110, 40, '#1f77b4')):
    ax.add_patch(Wedge((0, 0), rng, ang - fov / 2, ang + fov / 2, fc=col, alpha=0.06, ec=col, lw=0.4))
ax.add_patch(Rectangle((-L / 2, -W / 2), L, W, fc='#e8e8e8', ec='k', lw=1.2, zorder=5))
# other road users (illustrative)
for (x, y, c, lab) in ((28, 0, 'tab:blue', 'lead vehicle: gap, headway, TTC'), (-18, 3.5, 'tab:blue', 'following vehicle'), (14, -9, 'tab:orange', 'pedestrian at kerb: distance, crossing intent'), (35, 12, 'tab:blue', 'cross traffic at junction')):
    ax.plot(x, y, 's' if c == 'tab:blue' else 'o', color=c, ms=9, mec='k', zorder=6); ax.annotate(lab, (x, y), xytext=(x + 4, y + 7), fontsize=8.5, color=c, arrowprops=dict(arrowstyle='-', color=c, lw=0.7), bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=c))
ax.annotate('lane geometry, speed limit,\nsignals (map + perception)', (0, 0), xytext=(-48, 28), fontsize=8.5, color='#555', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='#555'))
ax.annotate('weather, lighting, time of day\n(weather API + cameras)', (0, 0), xytext=(-48, -38), fontsize=8.5, color='#555', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='#555'))
ax.annotate('', xy=(12, 0), xytext=(4, 0), arrowprops=dict(arrowstyle='->', lw=1.4)); ax.text(5, 1.8, 'heading', fontsize=8)
ax.set_xlim(-60, 60); ax.set_ylim(-45, 45); ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
ax.set_title('Environment: one vehicle and everything within its perception range', fontsize=11)

# ---- right: cabin + vehicle sensing, zoomed top view
az.add_patch(Rectangle((-L / 2, -W / 2), L, W, fc='#e8e8e8', ec='k', lw=1.5)); az.add_patch(Rectangle((-L / 2 + 0.9, -W / 2 + 0.12), 2.6, W - 0.24, fc='#cfd8e3', ec='k', lw=0.8))
az.plot(0.45, -0.4, 'o', color='#8c564b', ms=16, mec='k'); az.text(0.45, -0.4, 'D', color='white', ha='center', va='center', fontsize=9, weight='bold')
items = [((0.9, 0.0), 'tab:red', 'Cabin IR camera\n(rear-view mirror): gaze,\nhead pose, eyelid closure', (2.3, -2.3)),
         ((0.9, -0.4), 'tab:orange', 'Steering torque sensor:\nhands on wheel', (-0.2, -2.4)),
         ((0.45, -0.4), '#8c564b', 'Driver (left seat):\nseat occupancy, belt', (-2.9, -1.7)),
         ((-1.2, 0.0), 'tab:green', 'CAN bus: speed, accel.,\nsteer, brake, throttle,\nautomation mode, alerts', (-3.0, 1.6)),
         ((2.2, 0.0), 'tab:blue', 'Exterior cameras (8) +\nperception: agents, lanes,\nsignals, TTC', (1.4, 2.2)),
         ((-2.1, 0.0), 'tab:blue', 'Rear camera', (-3.0, -0.6))]
for (x, y), c, lab, (lx, ly) in items:
    az.plot(x, y, 'o', color=c, ms=9, mec='k', zorder=5); az.annotate(lab, (x, y), xytext=(lx, ly), fontsize=8.3, color=c, va='center', arrowprops=dict(arrowstyle='-', color=c, lw=0.8, shrinkB=5), bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=c, alpha=0.95))
az.add_patch(Wedge((0.9, 0.0), 1.4, 150, 230, fc='tab:red', alpha=0.15, ec='tab:red'))
az.add_patch(Wedge((2.2, 0.0), 2.2, -30, 30, fc='tab:blue', alpha=0.10, ec='tab:blue'))
az.set_xlim(-4.6, 4.6); az.set_ylim(-3.0, 3.4); az.set_aspect('equal'); az.set_xlabel('x [m]'); az.set_ylabel('y [m]')
az.set_title('Vehicle and cabin: where the driver and vehicle states are sensed', fontsize=11)
fig.tight_layout(); fig.savefig(f'{out}/layout.png', dpi=160); plt.close(fig)

# ---- coupling diagram: three states -> demand vs attention -> decisions
fig, ax = plt.subplots(figsize=(13, 4.2)); ax.axis('off')
def box(x, y, w, h, txt, col, fs=9.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.01', fc=col, ec='k')); ax.text(x + w / 2, y + h / 2, txt, ha='center', va='center', fontsize=fs)
box(0.01, 0.62, 0.20, 0.30, 'DRIVER STATE\ngaze zone, eyes-off-road time,\nPERCLOS, head pose, hands torque', '#f6c6c6')
box(0.01, 0.30, 0.20, 0.28, 'VEHICLE STATE\nspeed, accel., steering,\nbrake, automation mode', '#cfd8e3')
box(0.01, 0.00, 0.20, 0.27, 'ENVIRONMENT STATE\nagents, gap, TTC, pedestrians,\nlane, signals, weather, light', '#cde6c7')
box(0.30, 0.52, 0.20, 0.30, 'ATTENTION SUPPLY\nhow much attention\nthe driver is giving', '#f6c6c6')
box(0.30, 0.12, 0.20, 0.30, 'SCENE DEMAND\nhow much attention\nthe moment requires', '#cde6c7')
box(0.58, 0.30, 0.18, 0.34, 'ATTENTION RISK\nSCORE\ndemand - supply,\nweighted by the\ndriver\'s baseline', '#dbe7ff', 9.0)
box(0.82, 0.58, 0.17, 0.30, 'FAST LOOP (s)\nescalate alert:\nvisual, chime, haptic,\nslow down', '#fff2cc')
box(0.82, 0.08, 0.17, 0.30, 'SLOW LOOP (days)\n30-day score with a\nreason per point;\nrecalibrate weights', '#fff2cc')
for (x0, y0, x1, y1) in ((0.21, 0.77, 0.30, 0.67), (0.21, 0.44, 0.30, 0.60), (0.21, 0.44, 0.30, 0.30), (0.21, 0.13, 0.30, 0.25), (0.50, 0.67, 0.58, 0.52), (0.50, 0.27, 0.58, 0.42), (0.76, 0.52, 0.82, 0.70), (0.76, 0.42, 0.82, 0.25)):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='->', mutation_scale=14, lw=1.2))
ax.set_xlim(0, 1); ax.set_ylim(0, 0.95); ax.set_title('How the three measured states couple into the decisions the twin supports', fontsize=11.5)
fig.savefig(f'{out}/coupling.png', dpi=160, bbox_inches='tight'); plt.close(fig)
print('saved layout.png, coupling.png ->', out)
