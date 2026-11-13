"""Schematic of the ego sensor rig (top and side view) and of the twin's spatial hierarchy. Pure matplotlib, no CARLA."""
import sys, os
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Rectangle, Wedge, FancyBboxPatch, FancyArrowPatch
out = sys.argv[1] if len(sys.argv) > 1 else 'figs'; os.makedirs(out, exist_ok=True)

# ---------------------------------------------------------------- sensor rig on a Lincoln MKZ (4.9 x 1.9 x 1.5 m)
L, W, H = 4.9, 1.9, 1.5
S = {'RGB / depth / seg cameras': ((1.5, 0.0, 2.0), 'tab:blue', 90, 'cam'), 'LiDAR 64 ch': ((0.0, 0.0, 2.5), 'tab:green', 360, 'lidar'),
     'Radar': ((2.3, 0.0, 0.8), 'tab:red', 60, 'radar'), 'GNSS': ((0.0, 0.0, 2.0), 'tab:purple', 0, 'gnss'), 'IMU': ((0.0, 0.0, 1.0), 'tab:orange', 0, 'imu'),
     'Collision / lane-invasion': ((0.0, 0.0, 0.5), 'gray', 0, 'col')}
fig, (at, asd) = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={'width_ratios': [1.15, 1]})
at.add_patch(Rectangle((-L / 2, -W / 2), L, W, fc='#e8e8e8', ec='k', lw=1.5)); at.add_patch(Rectangle((-L / 2 + 0.9, -W / 2 + 0.15), 2.3, W - 0.3, fc='#cfd8e3', ec='k', lw=0.8))
at.annotate('', xy=(L / 2 + 1.2, 0), xytext=(L / 2 + 0.2, 0), arrowprops=dict(arrowstyle='->', lw=1.5)); at.text(L / 2 + 0.3, 0.25, 'forward (+x)', fontsize=9)
slots = {'cam': (3.2, 3.4), 'lidar': (-4.6, 3.6), 'radar': (4.6, -3.2), 'gnss': (-4.6, 1.6), 'imu': (-4.6, -1.6), 'col': (-4.6, -3.6)}
for name, ((x, y, z), c, fov, kind) in S.items():
    if fov == 90: at.add_patch(Wedge((x, y), 6.0, -45, 45, fc=c, alpha=0.12, ec=c))
    if fov == 60: at.add_patch(Wedge((x, y), 7.0, -30, 30, fc=c, alpha=0.12, ec=c))
    if fov == 360: at.add_patch(plt.Circle((x, y), 6.5, fc=c, alpha=0.06, ec=c, ls='--'))
    at.plot(x, y, 'o', color=c, ms=9, mec='k', zorder=5)
    lx, ly = slots[kind]; at.annotate(f'{name}\n({x:+.1f}, {y:+.1f}, z {z:.1f}) m', xy=(x, y), xytext=(lx, ly), fontsize=8, color=c, ha='left', va='center',
                                      arrowprops=dict(arrowstyle='-', color=c, lw=0.8, shrinkB=5), bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=c, alpha=0.9))
at.set_xlim(-5, 10); at.set_ylim(-4.6, 4.6); at.set_aspect('equal'); at.set_xlabel('x [m]'); at.set_ylabel('y [m]'); at.set_title('Sensor rig, top view (ego frame, origin at the vehicle centre)')
asd.add_patch(Rectangle((-L / 2, 0), L, 0.75, fc='#e8e8e8', ec='k', lw=1.5)); asd.add_patch(Rectangle((-L / 2 + 0.6, 0.75), 3.0, 0.75, fc='#cfd8e3', ec='k', lw=1.0))
for wx in (-1.6, 1.6): asd.add_patch(plt.Circle((wx, 0.0), 0.35, fc='#333'))
sslots = {'cam': (3.0, 3.6), 'lidar': (-3.2, 3.6), 'radar': (4.2, 0.4), 'gnss': (-3.2, 2.3), 'imu': (-3.2, 1.2), 'col': (-3.2, 0.3)}
for name, ((x, y, z), c, fov, kind) in S.items():
    asd.plot(x, z, 'o', color=c, ms=9, mec='k', zorder=5); lx, lz = sslots[kind]
    asd.annotate(name.split(' /')[0].split(' 64')[0], xy=(x, z), xytext=(lx, lz), fontsize=8, color=c, va='center', arrowprops=dict(arrowstyle='-', color=c, lw=0.8, shrinkB=5), bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=c, alpha=0.9))
asd.add_patch(Wedge((1.5, 2.0), 5.5, -40, 5, fc='tab:blue', alpha=0.10, ec='tab:blue')); asd.add_patch(Wedge((0.0, 2.5), 5.5, -25, 10, fc='tab:green', alpha=0.10, ec='tab:green'))
asd.axhline(0, color='k', lw=0.8); asd.set_xlim(-4, 8); asd.set_ylim(-0.6, 4.2); asd.set_aspect('equal'); asd.set_xlabel('x [m]'); asd.set_ylabel('z [m]'); asd.set_title('Side view: mounting heights and vertical fields of view')
fig.tight_layout(); fig.savefig(f'{out}/rig.png', dpi=160); plt.close(fig)

# ---------------------------------------------------------------- spatial hierarchy of the twin
fig, ax = plt.subplots(figsize=(15, 4.6)); ax.axis('off')
boxes = [(0.01, 'SITE\n240 to 560 m square\nUTM metres\nx east, y south', '#f3e5ab'), (0.21, 'ROAD NETWORK\nOpenDRIVE lanes\n3.5 m wide, junctions,\nsignals, parking strips', '#cde6c7'),
         (0.41, 'STATIC SCENE\nbuildings at tagged\nheight, trees, parked\ncars, street props', '#cfd8e3'), (0.61, 'DYNAMIC AGENTS\nego + 20 to 60 vehicles\n+ 30 to 60 pedestrians\n+ signal phases', '#f6c6c6'),
         (0.81, 'EGO SENSOR RIG\ncameras, LiDAR,\nradar, GNSS, IMU,\ncontact sensors', '#d9c6f6')]
for x, txt, col in boxes:
    ax.add_patch(FancyBboxPatch((x, 0.22), 0.175, 0.56, boxstyle='round,pad=0.01', fc=col, ec='k')); ax.text(x + 0.0875, 0.5, txt, ha='center', va='center', fontsize=9.2)
for x in (0.19, 0.39, 0.59, 0.79): ax.add_patch(FancyArrowPatch((x, 0.5), (x + 0.02, 0.5), arrowstyle='->', mutation_scale=16, lw=1.5))
ax.text(0.5, 0.1, 'contains  ->  contains  ->  moves on  ->  is observed by          (each level has its own coordinate frame; all are tied to the site origin)', ha='center', fontsize=9.5, style='italic')
ax.text(0.5, 0.92, 'Spatial hierarchy of the twin: from the real site down to a single measurement', ha='center', fontsize=12, weight='bold')
ax.set_xlim(0, 1); ax.set_ylim(0, 1); fig.savefig(f'{out}/hierarchy.png', dpi=160, bbox_inches='tight'); plt.close(fig)
print('saved rig.png, hierarchy.png ->', out)
