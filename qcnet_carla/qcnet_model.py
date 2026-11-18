"""QCNet (Zhou et al., CVPR 2023, Argoverse 2 checkpoint) fed from CARLA.

Builds the same heterogeneous graph the Argoverse 2 loader builds (datasets/argoverse_v2_dataset.py), but from CARLA's
OpenDRIVE map and the simulator's actor states, then runs the released model and returns 6 trajectory modes per agent in
the world frame.  Frames: Argoverse is right-handed (x east, y north); CARLA is left-handed (y flipped), so every CARLA
pose is converted with  x, -y, -yaw  on the way in and back on the way out.
"""
import math
import os
import sys
from typing import Dict, List

import numpy as np
import torch

QC_ROOT = '/home/all/carla_ws/QCNet'
sys.path.insert(0, QC_ROOT)
from torch_geometric.data import Batch, HeteroData  # noqa: E402
from predictors import QCNet  # noqa: E402

HIST, FUT, DT = 50, 60, 0.1          # 5 s history, 6 s future at 10 Hz (Argoverse 2 protocol the checkpoint was trained on)
AGENT_TYPES = ['vehicle', 'pedestrian', 'motorcyclist', 'cyclist', 'bus', 'static', 'background', 'construction', 'riderless_bicycle', 'unknown']
POLY_TYPES = ['VEHICLE', 'BIKE', 'BUS', 'PEDESTRIAN']
POLY_IS_INT = [True, False, None]
POINT_TYPES = ['DASH_SOLID_YELLOW', 'DASH_SOLID_WHITE', 'DASHED_WHITE', 'DASHED_YELLOW', 'DOUBLE_SOLID_YELLOW', 'DOUBLE_SOLID_WHITE',
               'DOUBLE_DASH_YELLOW', 'DOUBLE_DASH_WHITE', 'SOLID_YELLOW', 'SOLID_WHITE', 'SOLID_DASH_WHITE', 'SOLID_DASH_YELLOW',
               'SOLID_BLUE', 'NONE', 'UNKNOWN', 'CROSSWALK', 'CENTERLINE']
SIDES = ['LEFT', 'RIGHT', 'CENTER']
P2P = ['NONE', 'PRED', 'SUCC', 'LEFT', 'RIGHT']


def to_av(x, y, yaw_deg):
    """CARLA (left-handed, yaw in degrees) -> Argoverse (right-handed, heading in radians)."""
    return x, -y, -math.radians(yaw_deg)


def marking_name(marking) -> str:
    """CARLA LaneMarking -> Argoverse mark type name."""
    import carla
    t, c = marking.type, marking.color
    yellow = c == carla.LaneMarkingColor.Yellow
    M = carla.LaneMarkingType
    if t == M.Broken: return 'DASHED_YELLOW' if yellow else 'DASHED_WHITE'
    if t == M.Solid: return 'SOLID_YELLOW' if yellow else 'SOLID_WHITE'
    if t == M.SolidSolid: return 'DOUBLE_SOLID_YELLOW' if yellow else 'DOUBLE_SOLID_WHITE'
    if t == M.BrokenBroken: return 'DOUBLE_DASH_YELLOW' if yellow else 'DOUBLE_DASH_WHITE'
    if t == M.BrokenSolid: return 'DASH_SOLID_YELLOW' if yellow else 'DASH_SOLID_WHITE'
    if t == M.SolidBroken: return 'SOLID_DASH_YELLOW' if yellow else 'SOLID_DASH_WHITE'
    if t == M.NONE: return 'NONE'
    return 'UNKNOWN'


class CarlaMapGraph:
    """Lane segments (chunks of <= chunk_len m of each OpenDRIVE lane) and crosswalks as Argoverse-style polygons, with
    PRED / SUCC / LEFT / RIGHT edges, all in the Argoverse frame."""

    def __init__(self, cmap, step: float = 2.0, chunk_len: float = 30.0):
        import carla
        self.polys: List[Dict] = []
        wps = cmap.generate_waypoints(step)
        lanes: Dict = {}
        for wp in wps:
            if wp.lane_type != carla.LaneType.Driving: continue
            lanes.setdefault((wp.road_id, wp.section_id, wp.lane_id), []).append(wp)
        key_to_chunks: Dict = {}
        for key, group in lanes.items():
            group.sort(key=lambda w: w.s if w.lane_id < 0 else -w.s)
            n_per = max(2, int(chunk_len / step))
            chunks = [group[i:i + n_per + 1] for i in range(0, max(1, len(group) - 1), n_per)]
            chunks = [c for c in chunks if len(c) >= 2]
            ids = []
            for c in chunks:
                self.polys.append(self._lane_poly(c, key)); ids.append(len(self.polys) - 1)
            key_to_chunks[key] = ids
        # connectivity
        for key, ids in key_to_chunks.items():
            if not ids: continue   # a lane sampled by a single waypoint has no chunk
            for a, b in zip(ids[:-1], ids[1:]):
                self.polys[b]['pred'].append(a); self.polys[a]['succ'].append(b)
            last_wp = self.polys[ids[-1]]['wps'][-1]
            for nxt in last_wp.next(step):
                nk = (nxt.road_id, nxt.section_id, nxt.lane_id)
                if nk in key_to_chunks and key_to_chunks[nk] and nk != key:
                    j = key_to_chunks[nk][0]; self.polys[ids[-1]]['succ'].append(j); self.polys[j]['pred'].append(ids[-1])
            for i in ids:
                mid = self.polys[i]['wps'][len(self.polys[i]['wps']) // 2]
                for side, nb in (('left', mid.get_left_lane()), ('right', mid.get_right_lane())):
                    if nb is None or nb.lane_type != carla.LaneType.Driving or nb.lane_id * mid.lane_id < 0: continue
                    nk = (nb.road_id, nb.section_id, nb.lane_id)
                    if not key_to_chunks.get(nk): continue
                    nl = nb.transform.location; best = min(key_to_chunks[nk], key=lambda j: np.min(np.hypot(self.polys[j]['center_c'][:, 0] - nl.x, self.polys[j]['center_c'][:, 1] - nl.y)))
                    self.polys[i][side + '_nb'] = best
        self._crosswalks(cmap)
        for p in self.polys: p.pop('wps', None)
        self.centers = np.array([p['center'][0, :2] for p in self.polys])

    def _lane_poly(self, wps, key):
        pts = np.array([to_av(w.transform.location.x, w.transform.location.y, w.transform.rotation.yaw) for w in wps])
        z = np.array([w.transform.location.z for w in wps])
        center = np.stack([pts[:, 0], pts[:, 1], z], -1)
        h = pts[:, 2]; nx, ny = -np.sin(h), np.cos(h); hw = np.array([w.lane_width / 2 for w in wps])
        left = center.copy(); left[:, 0] += nx * hw; left[:, 1] += ny * hw
        right = center.copy(); right[:, 0] -= nx * hw; right[:, 1] -= ny * hw
        return dict(center=center, left=left, right=right, left_type=marking_name(wps[0].left_lane_marking), right_type=marking_name(wps[0].right_lane_marking),
                    ptype='VEHICLE', is_int=wps[len(wps) // 2].is_junction, pred=[], succ=[], left_nb=None, right_nb=None, key=key, wps=wps,
                    center_c=np.array([[w.transform.location.x, w.transform.location.y] for w in wps]))

    def _crosswalks(self, cmap):
        pts = cmap.get_crosswalks(); polys, cur = [], []
        for p in pts:
            if cur and abs(p.x - cur[0].x) < 1e-3 and abs(p.y - cur[0].y) < 1e-3:
                polys.append(cur); cur = []
            else: cur.append(p)
        for poly in polys:
            P = np.array([[*to_av(p.x, p.y, 0)[:2], p.z] for p in poly])
            if len(P) < 4: continue
            P = P[:4]
            if np.linalg.norm(P[1] - P[0]) >= np.linalg.norm(P[2] - P[1]): e1, e2 = P[[0, 1]], P[[3, 2]]
            else: e1, e2 = P[[1, 2]], P[[0, 3]]
            start, end = (e1[0] + e2[0]) / 2, (e1[-1] + e2[-1]) / 2
            n = max(2, int(math.ceil(np.linalg.norm(end - start) / 2.0)) + 1)
            c = np.linspace(0, 1, n)[:, None]; center = start[None] * (1 - c) + end[None] * c
            l = e1 if ((e1[0] + e1[-1]) / 2 - start)[0] * (end - start)[1] - ((e1[0] + e1[-1]) / 2 - start)[1] * (end - start)[0] > 0 else e2
            r = e2 if l is e1 else e1
            def dense(a, n): t = np.linspace(0, 1, n)[:, None]; return a[0][None] * (1 - t) + a[-1][None] * t
            L, R = dense(l, n), dense(r, n)
            for center_, left_, right_ in ((center, L, R), (center[::-1], R[::-1], L[::-1])):
                self.polys.append(dict(center=center_, left=left_, right=right_, left_type='CROSSWALK', right_type='CROSSWALK', ptype='PEDESTRIAN', is_int=None, pred=[], succ=[], key=None))

    def tensors(self, sel: np.ndarray):
        """Map tensors for the selected polygon indices (Argoverse loader semantics)."""
        idx = {old: new for new, old in enumerate(sel)}
        n = len(sel); pos = torch.zeros(n, 3); ori = torch.zeros(n); hgt = torch.zeros(n); ptype = torch.zeros(n, dtype=torch.uint8); pint = torch.zeros(n, dtype=torch.uint8)
        pp, po, pm, ph, pt, ps, p2pl = [], [], [], [], [], [], []
        e_idx, e_type = [], []
        for new, old in enumerate(sel):
            p = self.polys[old]; c, l, r = p['center'], p['left'], p['right']
            pos[new] = torch.from_numpy(c[0]).float(); ori[new] = math.atan2(c[1, 1] - c[0, 1], c[1, 0] - c[0, 0]); hgt[new] = c[1, 2] - c[0, 2]
            ptype[new] = POLY_TYPES.index(p['ptype']); pint[new] = POLY_IS_INT.index(p['is_int'])
            vec = {k: v[1:] - v[:-1] for k, v in (('l', l), ('r', r), ('c', c))}
            pp.append(np.concatenate([l[:-1], r[:-1], c[:-1]]))
            po.append(np.concatenate([np.arctan2(vec['l'][:, 1], vec['l'][:, 0]), np.arctan2(vec['r'][:, 1], vec['r'][:, 0]), np.arctan2(vec['c'][:, 1], vec['c'][:, 0])]))
            pm.append(np.concatenate([np.linalg.norm(vec['l'][:, :2], axis=-1), np.linalg.norm(vec['r'][:, :2], axis=-1), np.linalg.norm(vec['c'][:, :2], axis=-1)]))
            ph.append(np.concatenate([vec['l'][:, 2], vec['r'][:, 2], vec['c'][:, 2]]))
            nl, nr, nc = len(vec['l']), len(vec['r']), len(vec['c'])
            pt.append(np.concatenate([np.full(nl, POINT_TYPES.index(p['left_type'])), np.full(nr, POINT_TYPES.index(p['right_type'])), np.full(nc, POINT_TYPES.index('CENTERLINE'))]))
            ps.append(np.concatenate([np.full(nl, 0), np.full(nr, 1), np.full(nc, 2)]))
            p2pl.append(np.full(nl + nr + nc, new))
            for k, name in (('pred', 'PRED'), ('succ', 'SUCC')):
                for j in p.get(k, []):
                    if j in idx: e_idx.append((idx[j], new)); e_type.append(P2P.index(name))
            for k, name in (('left_nb', 'LEFT'), ('right_nb', 'RIGHT')):
                j = p.get(k)
                if j is not None and j in idx: e_idx.append((idx[j], new)); e_type.append(P2P.index(name))
        t = lambda a, dt=torch.float: torch.as_tensor(np.concatenate(a), dtype=dt)
        md = {'map_polygon': dict(num_nodes=n, position=pos, orientation=ori, height=hgt, type=ptype, is_intersection=pint),
              'map_point': dict(num_nodes=int(sum(len(a) for a in pp)), position=t(pp), orientation=t(po), magnitude=t(pm), height=t(ph), type=t(pt, torch.uint8), side=t(ps, torch.uint8)),
              ('map_point', 'to', 'map_polygon'): dict(edge_index=torch.stack([torch.arange(int(sum(len(a) for a in pp))), t(p2pl, torch.long)])),
              ('map_polygon', 'to', 'map_polygon'): dict(edge_index=torch.tensor(e_idx, dtype=torch.long).T.reshape(2, -1), type=torch.tensor(e_type, dtype=torch.uint8))}
        return md


class QCNetCarla:
    def __init__(self, ckpt: str = f'{QC_ROOT}/ckpt/QCNet_AV2.ckpt', device: str = 'cuda'):
        self.device = device
        self.model = QCNet.load_from_checkpoint(ckpt, map_location='cpu').to(device).eval()

    @torch.no_grad()
    def predict(self, agents: List[Dict], graph: CarlaMapGraph, ego_idx: int, map_radius: float = 200.0):
        """agents: list of dict(id, type ('vehicle'|'pedestrian'|...), hist (HIST, 5) [x, y, heading, vx, vy] in the Argoverse
        frame with NaN where unobserved).  Returns world-frame trajectories (N, 6, FUT, 2) and mode probabilities (N, 6)."""
        N = len(agents)
        pos = torch.zeros(N, HIST + FUT, 3); head = torch.zeros(N, HIST + FUT); vel = torch.zeros(N, HIST + FUT, 3)
        valid = torch.zeros(N, HIST + FUT, dtype=torch.bool); pred_mask = torch.zeros(N, HIST + FUT, dtype=torch.bool)
        atype = torch.zeros(N, dtype=torch.uint8); cat = torch.zeros(N, dtype=torch.uint8)
        for i, ag in enumerate(agents):
            h = np.asarray(ag['hist'], dtype=np.float32); ok = ~np.isnan(h[:, 0])
            pos[i, :HIST, :2] = torch.from_numpy(np.nan_to_num(h[:, :2])); head[i, :HIST] = torch.from_numpy(np.nan_to_num(h[:, 2])); vel[i, :HIST, :2] = torch.from_numpy(np.nan_to_num(h[:, 3:5]))
            v = torch.from_numpy(ok)
            valid[i, :HIST] = v; cur = bool(v[HIST - 1])
            valid[i, 1:HIST] = valid[i, :HIST - 1].clone() & valid[i, 1:HIST].clone(); valid[i, 0] = False     # vector_repr
            pred_mask[i, HIST:] = cur
            atype[i] = AGENT_TYPES.index(ag.get('type', 'vehicle')); cat[i] = 2 if ok.all() else 1
        data = {'agent': dict(num_nodes=N, av_index=int(ego_idx), valid_mask=valid, predict_mask=pred_mask, id=[str(a['id']) for a in agents],
                              type=atype, category=cat, position=pos, heading=head, velocity=vel)}
        ego_xy = pos[ego_idx, HIST - 1, :2].numpy()
        sel = np.where(np.linalg.norm(graph.centers - ego_xy[None], axis=-1) < map_radius)[0]
        data.update(graph.tensors(sel))
        # TargetBuilder equivalent (the repo's transform predates the current PyG BaseTransform API): the model's forward
        # does not read 'target', but it is kept so the object matches what the loader produces.
        data['agent']['target'] = torch.zeros(N, FUT, 4)
        hd = HeteroData(data); batch = Batch.from_data_list([hd]).to(self.device)
        out = self.model(batch)
        loc = out['loc_refine_pos'][..., :2].cpu()           # (N, K, FUT, 2) in each agent's frame at the current step
        pi = torch.softmax(out['pi'], dim=-1).cpu()          # (N, K)
        origin = pos[:, HIST - 1, :2]; th = head[:, HIST - 1]; c, s = th.cos(), th.sin()
        wx = origin[:, None, None, 0] + loc[..., 0] * c[:, None, None] - loc[..., 1] * s[:, None, None]
        wy = origin[:, None, None, 1] + loc[..., 0] * s[:, None, None] + loc[..., 1] * c[:, None, None]
        return torch.stack([wx, wy], -1).numpy(), pi.numpy(), len(sel)
