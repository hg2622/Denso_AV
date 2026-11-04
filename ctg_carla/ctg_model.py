"""CTG++ (NVlabs/CTG, scene-level diffusion traffic model, CoRL 2023) as a stand-alone multi-agent trajectory generator.

Simulator-agnostic: you give it the scene (agent states + 3 s of history, lane centre-lines, a drivable-area raster) and it
returns a 5.2 s trajectory for every agent, optionally steered by guidance losses ("make vehicle 3 collide with vehicle 0",
"keep 10-30 m behind", "follow the lane", target speed, collision avoidance ...). The CARLA side lives in ctg_carla.py.

Frames: right-handed, x forward / y left / yaw counter-clockwise in radians (nuScenes convention). CARLA is left-handed, the
CARLA adapter flips y and yaw before calling in here. The released checkpoint is agent-centric: the batch holds one row per
agent, each in its OWN current frame, and the model computes the pairwise relations itself.
"""
import math
import os
import sys
from typing import Dict, List, Optional

import numpy as np
import torch

CTG_ROOT = os.environ.get('CTG_ROOT', '/home/all/carla_ws/CTG')
for p in (CTG_ROOT, f'{CTG_ROOT}/trajdata/src', f'{CTG_ROOT}/Pplan', f'{CTG_ROOT}/stlcg/src', f'{CTG_ROOT}/stlcg'):
    if p not in sys.path:
        sys.path.insert(0, p)

from tbsim.utils.config_utils import get_experiment_config_from_file  # noqa: E402
from tbsim.utils.trajdata_utils import (get_modality_shapes, set_global_trajdata_batch_env,  # noqa: E402
                                        set_global_trajdata_batch_raster_cfg, add_scene_dim_to_agent_data)
from tbsim.algos.algos import SceneDiffuserTrafficModel  # noqa: E402
from tbsim.utils.batch_utils import set_global_batch_type  # noqa: E402
from tbsim.utils import guidance_loss as GL  # noqa: E402
import tbsim.utils.trajdata_utils as _TU  # noqa: E402
_TU.load_vec_map = lambda *a, **k: None   # the guidance batch extractor loads a trajdata vector map by name; no loss uses it

# Extra guidance losses that exist in the repo but are only reachable through the GPT path; expose them by name.
for _name, _cls in (('front_collision', 'FrontCollisionLoss'), ('collide_left_side', 'CollideLeftSideLoss'),
                    ('stay_away', 'StayAwayLoss'), ('follow_lane', 'FollowLaneLoss'),
                    ('change_to_left_lane', 'ChangeToLeftLaneLoss'), ('lane_following', 'LaneFollowingLoss'),
                    ('keep_distance', 'KeepDistanceLoss'), ('collision', 'CollisionLoss')):
    if hasattr(GL, _cls):
        GL.GUIDANCE_FUNC_MAP.setdefault(_name, getattr(GL, _cls))

VEHICLE_TYPE = 3          # l5kit type id for "car", what the checkpoint was trained with
HIST_T = 31               # 30 past frames + current
FUT_T = 52                # 5.2 s horizon
DT = 0.1
RASTER = 224
PIXEL_SIZE = 0.5          # m / px
EGO_CENTER = (-0.5, 0.0)  # agent sits 25 % from the left edge, vertically centred (trajdata convention)
S_SEG, S_POINT = 15, 80   # lane segments x points per agent
VEC_MAP = dict(map_max_dist=80.0, max_heading_error=0.25 * math.pi, ahead_threshold=-40.0, dist_weight=1.0, heading_weight=0.1)


def _tf(yaw: float, x: float, y: float) -> np.ndarray:
    """3x3 homogeneous transform: world_from_agent for an agent at (x, y) heading yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]], dtype=np.float64)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class LaneGraph:
    """Lane centre-lines as polylines of (x, y, z, heading) in world frame, with the kd-tree lookup CTG++ expects."""

    def __init__(self, polylines: List[np.ndarray]):
        from scipy.spatial import cKDTree
        self.lanes = [np.asarray(p, dtype=np.float64) for p in polylines if len(p) >= 2]
        pts = np.concatenate([p[:, :3] for p in self.lanes], axis=0)
        self.headings = np.concatenate([p[:, 3] for p in self.lanes], axis=0)
        self.owner = np.concatenate([np.full(len(p), i) for i, p in enumerate(self.lanes)], axis=0)
        self.tree = cKDTree(pts)
        self.pts = pts

    def current_lanes(self, xyzh: np.ndarray, max_dist: float, max_heading_err: float) -> List[int]:
        """Indices of lanes with a point within max_dist whose heading matches, sorted by distance (trajdata semantics)."""
        inds = np.asarray(self.tree.query_ball_point(xyzh[:3], max_dist))
        if len(inds) == 0:
            return []
        herr = np.abs(wrap(xyzh[3] - self.headings[inds]))
        ok = inds[herr < max_heading_err]
        if len(ok) == 0:
            return []
        d = np.linalg.norm(self.pts[ok] - xyzh[:3], axis=-1)
        lane_ids, first = np.unique(self.owner[ok], return_index=True)
        order = np.argsort(d[first])
        return lane_ids[order].tolist()

    def closest_lane_points(self, x: float, y: float, yaw: float, agent_from_world: np.ndarray) -> np.ndarray:
        """(S_SEG, S_POINT, 3) lane points (x, y, heading) in the agent frame, NaN-padded, as the model was trained on."""
        p = VEC_MAP
        query = np.array([x, y, 0.0, wrap(yaw)])
        lane_ids = self.current_lanes(query, p['map_max_dist'], p['max_heading_error'])[:S_SEG]
        out = np.full((S_SEG, S_POINT, 3), np.nan, dtype=np.float32)
        R, t = agent_from_world[:2, :2], agent_from_world[:2, 2]
        ayaw = math.atan2(agent_from_world[1, 0], agent_from_world[0, 0])
        for k, lid in enumerate(lane_ids):
            lane = self.lanes[lid]
            dist = p['dist_weight'] * np.linalg.norm(lane[:, :2] - query[:2], axis=-1) + p['heading_weight'] * np.abs(lane[:, 3] - query[3])
            order = np.argsort(dist)
            local = lane[order][:, [0, 1, 3]].copy()
            local[:, :2] = local[:, :2] @ R.T + t
            local[:, 2] = wrap(local[:, 2] + ayaw)
            local = local[local[:, 0] > p['ahead_threshold']][:S_POINT]
            out[k, :len(local)] = local
        return out


class DrivableRaster:
    """World-aligned drivable-area raster (1 = drivable) at PIXEL_SIZE; produces the per-agent 224x224 crop."""

    def __init__(self, drivable: np.ndarray, origin_xy: np.ndarray, pixel_size: float = PIXEL_SIZE):
        self.img = drivable.astype(np.float32)      # row = y, col = x, origin at the bottom-left corner origin_xy
        self.origin = np.asarray(origin_xy, dtype=np.float64)
        self.px = pixel_size

    def crop(self, agent_from_world: np.ndarray, raster_from_agent: np.ndarray) -> np.ndarray:
        import cv2
        # raster(agent) <- agent <- world <- global raster pixels
        world_from_gpix = np.array([[self.px, 0, self.origin[0]], [0, self.px, self.origin[1]], [0, 0, 1.0]])
        gpix_from_rpix = np.linalg.inv(raster_from_agent @ agent_from_world @ world_from_gpix)
        # cv2.warpAffine maps destination pixels through the inverse matrix when WARP_INVERSE_MAP is set
        return cv2.warpAffine(self.img, gpix_from_rpix[:2].astype(np.float32), (RASTER, RASTER),
                              flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP, borderValue=0.0)


class CTGPlusPlus:
    def __init__(self, ckpt_dir: str = f'{CTG_ROOT}/ckpt/ctg++8_9,10edge', device: str = 'cuda', num_samples: int = 5,
                 diffusion_stride: int = 1):
        self.device = torch.device(device)
        cfg = get_experiment_config_from_file(f'{ckpt_dir}/config.json')
        set_global_batch_type('trajdata')
        set_global_trajdata_batch_env('nusc_trainval')
        set_global_trajdata_batch_raster_cfg(cfg.env.rasterizer)
        shapes = get_modality_shapes(cfg)
        ck = sorted(os.listdir(f'{ckpt_dir}/checkpoints'))[-1]
        self.model = SceneDiffuserTrafficModel.load_from_checkpoint(
            f'{ckpt_dir}/checkpoints/{ck}', algo_config=cfg.algo, modality_shapes=shapes,
            registered_name=cfg['registered_name'], map_location='cpu').to(self.device).eval()
        self.cfg = cfg
        self.num_samples = num_samples
        # 'current_speed' mode only needs the agents' current speed, which build_batch writes into the placeholder
        # future (extras.full_fut_traj[:, 0, 2]).  NOTE: this must stay enabled: the policy keeps its own copy of the
        # flag, and with the wrapper's copy off the mask is None and `x_recon[None] = 0` zeroes EVERY agent's action.
        self.model.disable_control_on_stationary = 'current_speed'
        self.n_image_ch = shapes['image'][0]
        self.model.use_ema = True    # the evaluation default (the 52 all-zero EMA tensors are legitimately zero in the raw policy too)
        self.model.nets['policy'].disable_control_on_stationary = 'current_speed'
        self.model.ema_policy.disable_control_on_stationary = 'current_speed'
        self.policy = self.model.ema_policy if self.model.use_ema else self.model.nets['policy']
        self.raster_from_agent = np.array([[1 / PIXEL_SIZE, 0, (1 + EGO_CENTER[0]) / 2 * RASTER],
                                           [0, 1 / PIXEL_SIZE, (1 + EGO_CENTER[1]) / 2 * RASTER], [0, 0, 1.0]])
        self.step_index = 0
        # the two parameter sets the scene editor passes (scene_edit_config defaults); stride > 1 skips diffusion steps
        self.model.set_guidance_optimization_params(dict(optimizer='adam', lr=0.3, grad_steps=1, perturb_th=None))
        self.model.set_diffusion_specific_params(dict(stride=max(1, int(diffusion_stride)), apply_guidance_intermediate=True,
                                                      apply_guidance_output=False,
                                                      final_step_opt_params=dict(optimizer='adam', lr=0.3, grad_steps=1, perturb_th=1)))

    # ------------------------------------------------------------------ batch
    def build_batch(self, agents: List[Dict], lane_graph: LaneGraph, drivable: Optional[DrivableRaster]) -> Dict:
        """agents: list of dicts with keys
             hist: (HIST_T, 5) world-frame rows [x, y, yaw, vx, vy] oldest -> current; NaN rows = unavailable
             extent: (length, width, height) in metres
           Returns the scene batch (with the batch dim) ready for the model."""
        M = len(agents)
        hist_pos = np.zeros((M, HIST_T, 2), np.float32); hist_yaw = np.zeros((M, HIST_T, 1), np.float32)
        hist_spd = np.zeros((M, HIST_T), np.float32); hist_ok = np.zeros((M, HIST_T), bool)
        agent_hist = np.zeros((M, HIST_T, 8), np.float32)
        centroid = np.zeros((M, 2), np.float32); yaw = np.zeros((M,), np.float32); extent = np.zeros((M, 3), np.float32)
        a_from_w = np.zeros((M, 3, 3), np.float32); w_from_a = np.zeros((M, 3, 3), np.float32)
        lanes = np.zeros((M, S_SEG, S_POINT, 3), np.float32); images = np.zeros((M, self.n_image_ch, RASTER, RASTER), np.float32)
        for i, ag in enumerate(agents):
            h = np.asarray(ag['hist'], dtype=np.float64)
            assert h.shape == (HIST_T, 5), h.shape
            x, y, th = h[-1, 0], h[-1, 1], h[-1, 2]
            W = _tf(th, x, y); A = np.linalg.inv(W)
            ok = ~np.isnan(h[:, 0])
            rel = (h[:, :2] - [x, y]) @ A[:2, :2].T
            rel_yaw = wrap(h[:, 2] - th)
            vel = h[:, 3:5] @ A[:2, :2].T
            rel[~ok] = 0; rel_yaw[~ok] = 0; vel[~ok] = 0
            spd = np.linalg.norm(h[:, 3:5], axis=-1); spd[~ok] = 0
            acc = np.zeros_like(vel); acc[1:] = (vel[1:] - vel[:-1]) / DT; acc[~ok] = 0
            hist_pos[i], hist_yaw[i, :, 0], hist_spd[i], hist_ok[i] = rel, rel_yaw, spd, ok
            agent_hist[i] = np.concatenate([rel, vel, acc, np.sin(rel_yaw)[:, None], np.cos(rel_yaw)[:, None]], axis=-1)
            centroid[i], yaw[i], extent[i] = (x, y), th, ag['extent']
            a_from_w[i], w_from_a[i] = A, W
            lanes[i] = lane_graph.closest_lane_points(x, y, th, A)
            if drivable is not None:
                images[i, -3] = drivable.crop(A, self.raster_from_agent)
        curr_speed = hist_spd[:, -1]
        raster_from_agent = np.repeat(self.raster_from_agent[None].astype(np.float32), M, 0)
        agent_from_raster = np.linalg.inv(raster_from_agent)
        raster_from_world = raster_from_agent @ a_from_w
        # neighbours of each agent: all other agents, expressed in that agent's frame (used by the non-edge encoders)
        Q = max(M - 1, 1)
        n_pos = np.zeros((M, Q, HIST_T, 2), np.float32); n_yaw = np.zeros((M, Q, HIST_T, 1), np.float32)
        n_spd = np.zeros((M, Q, HIST_T), np.float32); n_ok = np.zeros((M, Q, HIST_T), bool); n_ext = np.zeros((M, Q, 3), np.float32)
        n_type = np.zeros((M, Q), np.int64)
        for i in range(M):
            others = [j for j in range(M) if j != i]
            for q, j in enumerate(others):
                T_ij = a_from_w[i] @ w_from_a[j]
                p = hist_pos[j] @ T_ij[:2, :2].T + T_ij[:2, 2]
                n_pos[i, q] = p * hist_ok[j][:, None]; n_yaw[i, q, :, 0] = wrap(hist_yaw[j, :, 0] + yaw[j] - yaw[i]) * hist_ok[j]
                n_spd[i, q] = hist_spd[j]; n_ok[i, q] = hist_ok[j]; n_ext[i, q] = extent[j]; n_type[i, q] = VEHICLE_TYPE
        t = lambda a, dt=torch.float32: torch.as_tensor(a, dtype=dt, device=self.device)
        fut_ph = np.zeros((M, 200, 8), np.float32); fut_ok = np.zeros((M, 200), bool)
        fut_ph[:, 0, 2] = curr_speed; fut_ok[:, 0] = True   # lets get_stationary_mask('current_speed') work without GT future
        obs = dict(
            image=t(images), maps=None, map_names=['carla'] * M,
            drivable_map=t(images[:, -3]),
            target_positions=t(np.zeros((M, FUT_T, 2), np.float32)), target_yaws=t(np.zeros((M, FUT_T, 1), np.float32)),
            target_availabilities=t(np.ones((M, FUT_T), bool), torch.bool),
            history_positions=t(hist_pos), history_yaws=t(hist_yaw), history_speeds=t(hist_spd),
            history_availabilities=t(hist_ok, torch.bool), curr_speed=t(curr_speed),
            centroid=t(centroid), yaw=t(yaw), type=t(np.full((M,), VEHICLE_TYPE), torch.int64), extent=t(extent),
            raster_from_agent=t(raster_from_agent), agent_from_raster=t(agent_from_raster), raster_from_world=t(raster_from_world),
            agent_from_world=t(a_from_w), world_from_agent=t(w_from_a),
            all_other_agents_history_positions=t(n_pos), all_other_agents_history_yaws=t(n_yaw),
            all_other_agents_history_speeds=t(n_spd), all_other_agents_history_availabilities=t(n_ok, torch.bool),
            all_other_agents_extents=t(n_ext), all_other_agents_types=t(n_type, torch.int64),
            agent_hist=t(agent_hist), scene_index=t(np.zeros((M,), np.int64), torch.int64),
            extras=dict(closest_lane_point=t(lanes),
                        full_fut_traj=t(fut_ph), full_fut_valid=t(fut_ok, torch.bool)),
        )
        scene = add_scene_dim_to_agent_data(obs)
        scene['num_agents'] = torch.tensor([M], device=self.device)
        self._last_agent_obs = obs   # guidance losses are initialised from the agent-centric view (B = M agents)
        return scene

    # ------------------------------------------------------------------ guidance
    def set_guidance(self, guidance: Optional[List[Dict]], batch: Dict):
        """guidance: list of {'name', 'weight', 'params', 'agents'} dicts (one scene). None clears it."""
        self.model.clear_guidance()
        if guidance:
            self.model.set_guidance([guidance], getattr(self, '_last_agent_obs', None))

    # ------------------------------------------------------------------ inference
    @torch.no_grad()
    def plan(self, batch: Dict) -> Dict[str, np.ndarray]:
        """Returns positions (M, FUT_T, 2) and yaws (M, FUT_T) in each agent's OWN current frame, plus all samples
        (N, M, FUT_T, 2). Convert to world with world_from_agent of that agent."""
        no_guidance = self.policy.current_perturbation_guidance.current_guidance is None
        with torch.enable_grad():   # guidance needs gradients inside the sampler even though weights are frozen
            # step_index=0 always: every call is a fresh scene (agents are re-selected), and the wrapper only recomputes
            # the stationary mask at step 0, so a stale mask from a scene of different size would crash the sampler.
            action, info = self.model.get_action(batch, num_action_samples=self.num_samples, step_index=0,
                                                 guide_as_filter_only=no_guidance)   # sampler asserts a guidance object otherwise
        pos = action.positions[0].detach().cpu().numpy()            # (M, T, 2)
        yaw = action.yaws[0].detach().cpu().numpy()[..., 0]         # (M, T)
        samples = info['action_samples']['positions'][0].detach().cpu().numpy()   # (N, M, T, 2)
        return dict(positions=pos, yaws=yaw, samples=samples)

    def reset(self):
        self.step_index = 0
