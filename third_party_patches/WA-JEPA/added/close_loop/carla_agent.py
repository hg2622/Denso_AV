"""WA-JEPA as a CARLA / Bench2Drive leaderboard agent (local wiring, zero-shot: the model was trained on nuPlan / NAVSIM).

Mirrors close_loop/hugsim_planner.py: four cameras in the model's slot order (cam_l0, cam_f0, cam_r0, cam_b0) mounted like the
nuPlan rig (LEAD's NUPLAN_CAMERA_CALIBRATION: FOV 63.71, 16:9), 4 history frames 0.5 s apart, ego_status = [command one-hot(4),
vx, vy, ax, ay], history trajectory as relative SE2 poses (x forward, y left, right-handed), output 8 waypoints at 2 Hz in the
same frame. The waypoints are turned into controls with a PID (desired speed from the first waypoints, steering toward an aim
point ~1 s ahead). The model is queried every INFER_EVERY ticks (CARLA runs at 20 Hz) and the last plan is reused in between.
Env: WAJEPA_CKPT (checkpoint dir or model_state_dict.pt), WAJEPA_CFG (yaml; default configs/wa_jepa_hugsim.yaml),
AGENT_TRACE (per-tick jsonl for annotate_run.py --y-left), WAJEPA_MAX_KMH (speed cap, default 60), WAJEPA_INFER_EVERY (default 4)."""
import json, math, os, sys, time
from collections import deque
import numpy as np, carla, cv2
from leaderboard.autoagents import autonomous_agent
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path: sys.path.insert(0, REPO)


def get_entry_point(): return 'WAJEPAAgent'


# nuPlan rig (LEAD constants, metres / degrees; positions are relative to the rear axle -> +1.4 m to CARLA's vehicle centre)
CAMS = {'cam_l0': dict(x=0.26 + 1.4, y=-0.15, z=1.96, yaw=-56.0), 'cam_f0': dict(x=0.34 + 1.4, y=0.0, z=1.95, yaw=0.0),
        'cam_r0': dict(x=0.36 + 1.4, y=0.16, z=1.95, yaw=54.3), 'cam_b0': dict(x=-1.84 + 1.4, y=0.0, z=1.91, yaw=180.0)}
CAM_W, CAM_H, CAM_FOV = 512, 288, 63.71   # the model resizes to 512x256 anyway; 4 x 960x540 made CARLA (Town12, Low) crawl at ~1 s per tick
WARMUP_TICKS = int(os.environ.get('WAJEPA_WARMUP_TICKS', '100'))   # drive straight at fixed throttle first so the history the model sees is a moving car (a standing-still history makes it creep)
INFER_EVERY = int(os.environ.get('WAJEPA_INFER_EVERY', '4'))
MAX_KMH = float(os.environ.get('WAJEPA_MAX_KMH', '60'))
CTRL = os.environ.get('WAJEPA_CTRL', 'pid')   # 'pid' (local PID on throttle/brake/steer) or 'hugsim' (HUGSIM's iLQR + kinematic bicycle, the ego is moved with its exact velocities)
HUGSIM_DT = 0.25; HUGSIM_L = 2.7; HUGSIM_MAX_STEER = math.pi / 3.0
if CTRL == 'hugsim': INFER_EVERY = 5   # HUGSIM plans at 4 Hz


def rel_se2(origin, poses):
    """close_loop/hugsim_planner.convert_absolute_to_relative_se2_array (right-handed x fwd, y left, yaw ccw)."""
    poses = np.asarray(poses, dtype=np.float64).copy(); ox, oy, oh = origin; th = -oh
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    rel = poses - np.array([[ox, oy, oh]]); rel[:, :2] = rel[:, :2] @ rot.T
    rel[:, 2] = np.arctan2(np.sin(rel[:, 2]), np.cos(rel[:, 2])); return rel


class PID:
    def __init__(self, kp, ki, kd, n=20): self.kp, self.ki, self.kd = kp, ki, kd; self.w = deque([0.0] * n, maxlen=n)
    def step(self, e):
        self.w.append(e); return self.kp * e + self.ki * float(np.mean(self.w)) + self.kd * (self.w[-1] - self.w[-2])


class WAJEPAAgent(autonomous_agent.AutonomousAgent):
    def setup(self, path_to_conf_file, *args, **kwargs):
        self.track = autonomous_agent.Track.SENSORS
        import torch
        from omegaconf import OmegaConf
        from models.factory import build_world_model
        from training.checkpoint import load_checkpoint
        self.torch = torch; self.device = 'cuda'
        cfg_path = os.environ.get('WAJEPA_CFG', os.path.join(REPO, 'configs', 'wa_jepa_hugsim.yaml'))
        ckpt = os.environ.get('WAJEPA_CKPT', os.path.join(REPO, 'ckpt'))
        if os.path.isdir(ckpt): ckpt = os.path.join(ckpt, 'model_state_dict.pt')
        cfg = OmegaConf.load(cfg_path); OmegaConf.set_struct(cfg, False)
        cfg.model.require_pretrained = False; cfg.model.vjepa2_ckpt = None; cfg.model.init_from_checkpoint = None; cfg.model.require_init_from_checkpoint = False
        self.cfg = OmegaConf.to_container(cfg, resolve=True); m = self.cfg['model']
        self.cams = [str(c).lower() for c in m['camera_names']]; self.in_h, self.in_w = int(m['input_hw'][0]), int(m['input_hw'][1])
        self.n_hist = int(m['num_history_frames']); self.ego_dim = int(m['ego_status_dim']); self.horizon = int(m['trajectory_horizon'])
        self.stride = int(round(0.5 / 0.05))   # 2 Hz model history on a 20 Hz simulator
        t0 = time.time(); self.model = build_world_model(self.cfg).to(self.device)
        load_checkpoint(ckpt, self.model, map_location=self.device, log_fn=print); self.model.eval()
        print(f'[WA-JEPA] loaded {ckpt} in {time.time() - t0:.0f} s, cuda mem {torch.cuda.memory_allocated() / 1e9:.2f} GB, cams {self.cams}, infer every {INFER_EVERY} ticks, cap {MAX_KMH:.0f} km/h', flush=True)
        self.buf = deque(maxlen=self.stride * (self.n_hist - 1) + 1)
        self.step = -1; self.initialized = False; self.last_control = carla.VehicleControl(brake=1.0); self.last_traj = None; self.last_cmd = 1
        self.speed_pid = PID(1.0, 0.25, 0.3); self.turn_pid = PID(1.0, 0.4, 0.2); self.infer_ms = 0.0; self.last_plan_pose = None
        if CTRL == 'hugsim':
            from third_party.hugsim_ctrl.lqr import plan2control; self._plan2control = plan2control
            self.k_v = 0.0; self.k_steer = 0.0; self.k_acc = 0.0; self.k_rate = 0.0; self.k_yaw = None   # kinematic state (HUGSIM: velo, steer, last accel / steer rate)
            print('[WA-JEPA] controller = HUGSIM iLQR + kinematic bicycle (plan tracked every 0.25 s, ego driven with the model velocities)', flush=True)

    def _init(self):
        # map lat/lon reference solved from the first route point (as the DriveMoE agent does), so GPS -> CARLA metres is exact
        from scipy.optimize import fsolve
        E = 6378137.0
        try:
            locx, locy = self._global_plan_world_coord[0][0].location.x, self._global_plan_world_coord[0][0].location.y
            lon, lat = self._global_plan[0][0]['lon'], self._global_plan[0][0]['lat']
            def eqs(v):
                x, y = v
                return [lon * math.cos(x * math.pi / 180) - (locx * x * 180) / (math.pi * E) - math.cos(x * math.pi / 180) * y,
                        math.log(math.tan((lat + 90) * math.pi / 360)) * E * math.cos(x * math.pi / 180) + locy - math.cos(x * math.pi / 180) * E * math.log(math.tan((90 + x) * math.pi / 360))]
            self.lat_ref, self.lon_ref = (float(v) for v in fsolve(eqs, [0, 0]))
        except Exception as e: print('[WA-JEPA] lat/lon ref failed:', e, flush=True); self.lat_ref, self.lon_ref = 0.0, 0.0
        import importlib.util
        spec = importlib.util.spec_from_file_location('dm_planner', '/home/all/carla_ws/DriveMoE/src/agent/team_code/planner.py'); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        self._route_planner = mod.RoutePlanner(7.5, 25.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref); self._route_planner.set_route(self._global_plan, True)
        self.initialized = True

    def gps_to_location(self, gps):
        E = 6378137.0; lat, lon = float(gps[0]), float(gps[1]); scale = math.cos(self.lat_ref * math.pi / 180.0)
        my = math.log(math.tan((lat + 90) * math.pi / 360.0)) * (E * scale); mx = (lon * (math.pi * E * scale)) / 180.0
        y = scale * E * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my; x = mx - scale * self.lon_ref * math.pi * E / 180.0
        return np.array([x, y])

    def sensors(self):
        s = []
        for name, c in CAMS.items():
            s.append({'type': 'sensor.camera.rgb', 'x': c['x'], 'y': c['y'], 'z': c['z'], 'roll': 0.0, 'pitch': 0.0, 'yaw': c['yaw'],
                      'width': CAM_W, 'height': CAM_H, 'fov': CAM_FOV, 'id': name})
        s += [{'type': 'sensor.other.gnss', 'x': -1.4, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'sensor_tick': 0.01, 'id': 'GPS'},
              {'type': 'sensor.other.imu', 'x': -1.4, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'sensor_tick': 0.05, 'id': 'IMU'},
              {'type': 'sensor.speedometer', 'reading_frequency': 20, 'id': 'SPEED'}]
        return s

    def _img(self, bgra):
        rgb = cv2.cvtColor(bgra[:, :, :3], cv2.COLOR_BGR2RGB); rgb = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        t = self.torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float() / 255.0; return t.mul(2.0).sub(1.0).clamp(-1.0, 1.0)

    def _command(self, gps):
        try:
            near = self._route_planner.run_step(gps)   # (pos, RoadOption)
            opt = near[1]; name = str(opt).split('.')[-1]
            self.last_cmd = {'LEFT': 0, 'RIGHT': 2, 'CHANGELANELEFT': 0, 'CHANGELANERIGHT': 2}.get(name, 1)
        except Exception: pass
        return self.last_cmd

    def run_step(self, input_data, timestamp, sensors=None):
        self.step += 1
        if not self.initialized: self._init()
        gps = self.gps_to_location(input_data['GPS'][1][:2])   # CARLA map metres (x east, y south)
        compass = float(input_data['IMU'][1][-1]); compass = 0.0 if math.isnan(compass) else compass
        yaw_carla = compass - math.pi / 2.0   # leaderboard compass -> CARLA yaw (rad)
        speed = float(input_data['SPEED'][1]['speed']); acc = input_data['IMU'][1][:3]
        imgs = self.torch.stack([self._img(input_data[c][1]) for c in self.cams], dim=0)   # [V,3,H,W]
        # right-handed pose for the model: x forward east, y LEFT, yaw ccw  (CARLA is left-handed: y right, yaw cw)
        pose = np.array([float(gps[0]), -float(gps[1]), -yaw_carla], dtype=np.float64)
        self.buf.append({'imgs': imgs, 'pose': pose})
        cmd = self._command(gps)
        if self.step % INFER_EVERY == 0 or self.last_traj is None:
            last = len(self.buf) - 1; idx = [max(0, last - self.stride * (self.n_hist - 1 - k)) for k in range(self.n_hist)]
            hist_imgs = self.torch.stack([self.buf[i]['imgs'] for i in idx], dim=0)
            poses = np.stack([self.buf[i]['pose'] for i in idx]); hist_traj = rel_se2(poses[-1], poses).astype(np.float32)
            st = np.zeros(self.ego_dim, dtype=np.float32); st[min(cmd, 3)] = 1.0; st[4] = speed; st[5] = 0.0; st[6] = float(acc[0]); st[7] = 0.0
            feats = {'history_images': hist_imgs.unsqueeze(0).to(self.device), 'ego_status': self.torch.from_numpy(st).unsqueeze(0).to(self.device),
                     'history_trajectory': self.torch.from_numpy(hist_traj).unsqueeze(0).to(self.device)}
            t0 = time.time()
            with self.torch.no_grad(), self.torch.autocast('cuda', dtype=self.torch.bfloat16):
                pred = self.model.predict_trajectory(feats)
            self.infer_ms = (time.time() - t0) * 1000.0
            self.last_traj = pred.squeeze(0).float().cpu().numpy().astype(np.float64)   # [8,3] x fwd, y left, yaw (2 Hz)
            self.last_plan_pose = pose
        # express the plan relative to the CURRENT pose (the plan is reused for a few ticks while the car moves)
        traj = self.last_traj.copy()
        if self.last_plan_pose is not None:
            ox, oy, oh = self.last_plan_pose; c, s_ = math.cos(oh), math.sin(oh)
            world = np.stack([ox + c * traj[:, 0] - s_ * traj[:, 1], oy + s_ * traj[:, 0] + c * traj[:, 1], oh + traj[:, 2]], axis=1)
            traj = rel_se2(pose, world)
        wps = traj[:, :2]   # x forward, y left (metres), 0.5 s apart
        if CTRL == 'hugsim':
            control = self._hugsim_control(wps, speed, yaw_carla)
            if os.environ.get('AGENT_TRACE'):
                with open(os.environ['AGENT_TRACE'], 'a') as f:
                    f.write(json.dumps({'step': self.step, 't': float(timestamp), 'speed': speed, 'desired': float(self.k_v), 'throttle': float(max(self.k_acc, 0.0) / 3.0), 'brake': float(max(-self.k_acc, 0.0) / 3.0),
                                        'steer': float(self.k_steer / HUGSIM_MAX_STEER), 'route': np.round(wps, 2).tolist(), 'speed_wps': [], 'text': f'HUGSIM ctrl: acc {self.k_acc:+.2f} steer {math.degrees(self.k_steer):+.1f} deg  cmd {["left", "straight", "right", "?"][min(cmd, 3)]}  infer {self.infer_ms:.0f} ms',
                                        'cmd': int(cmd), 'infer_ms': round(self.infer_ms, 1), 'acc': float(self.k_acc), 'steer_rate': float(self.k_rate)}) + '\n')
            return control
        # --- controls: desired speed from the first second of the plan, steering toward the aim point ~1 s ahead
        # desired speed = the fastest average speed the plan reaches within its first 2 s (an accelerating plan means "go faster")
        desired = min(MAX_KMH / 3.6, max(float(np.linalg.norm(wps[k])) / (0.5 * (k + 1)) for k in range(4)))
        aim_i = next((i for i, w in enumerate(wps) if np.linalg.norm(w) >= max(4.0, 1.0 * speed)), len(wps) - 1); aim = wps[aim_i]
        angle = math.degrees(math.atan2(-aim[1], max(aim[0], 0.1))) / 90.0   # y left -> CARLA steer sign (positive = right)
        if speed < 0.05: angle = 0.0
        steer = float(np.clip(self.turn_pid.step(angle), -1.0, 1.0))
        brake = desired < 0.4 or speed > desired + 1.5          # brake only on a real speed reduction; a plan slightly below the current speed just coasts
        throttle = 0.0 if (brake or desired < speed) else float(np.clip(self.speed_pid.step(np.clip(desired - speed, 0.0, 3.0)), 0.0, 0.75))
        control = carla.VehicleControl(steer=steer, throttle=throttle, brake=1.0 if brake else 0.0)
        if self.step < 10: control = carla.VehicleControl(brake=1.0)
        elif self.step < 10 + WARMUP_TICKS and desired >= speed - 1.0: control = carla.VehicleControl(throttle=0.7, steer=steer * 0.3)   # warm-up: build up speed (model steering lightly); the model's own brake (plan slower than the car) ends the warm-up
        self.last_control = control
        if os.environ.get('AGENT_TRACE'):
            with open(os.environ['AGENT_TRACE'], 'a') as f:
                f.write(json.dumps({'step': self.step, 't': float(timestamp), 'speed': speed, 'desired': float(desired), 'throttle': control.throttle, 'brake': control.brake,
                                    'steer': control.steer, 'route': np.round(wps, 2).tolist(), 'speed_wps': [], 'text': f'cmd {["left", "straight", "right", "?"][min(cmd, 3)]}  infer {self.infer_ms:.0f} ms',
                                    'cmd': int(cmd), 'infer_ms': round(self.infer_ms, 1)}) + '\n')
        return control

    def _hugsim_control(self, wps, speed, yaw_carla):
        """HUGSIM's closed loop, line for line: traj2control (heading-fixed) -> iLQR (acc, steer rate) every 0.25 s, then the kinematic
        bicycle integrated at the simulator tick. The ego is moved with the model's velocities (physics stays on so CARLA still
        detects collisions); throttle/brake/steer are not used. Frame: x forward, y RIGHT, yaw positive to the right = CARLA's."""
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        hero = CarlaDataProvider.get_hero_actor()
        if hero is None: return carla.VehicleControl(brake=1.0)
        if self.k_yaw is None: self.k_yaw = yaw_carla; self.k_v = speed
        if self.step < 10: self.k_v = 0.0; return carla.VehicleControl(brake=1.0)
        if self.step % INFER_EVERY == 0:   # new plan -> new (acc, steer_rate), as HUGSIM does once per 0.25 s step
            fr = wps[:, 0]; rt = -wps[:, 1]                       # model y is LEFT -> HUGSIM/CARLA right
            stats = np.zeros((len(wps) + 1, 5)); stats[1:, 0] = fr; stats[1:, 1] = rt
            pa, pb = 0.0, 0.0
            for i in range(len(wps)):                            # heading-fixed traj2control: arctan2(d_right, d_forward)
                rot = math.atan2(rt[i] - pa, fr[i] - pb); rot = rot - math.pi if rot > math.pi / 2 else (rot + math.pi if rot < -math.pi / 2 else rot)
                stats[i + 1, 2] = rot; pa, pb = rt[i], fr[i]
            try: acc, rate = self._plan2control(stats, np.array([0.0, 0.0, 0.0, self.k_v, self.k_steer]))
            except Exception as e: print('[WA-JEPA] iLQR failed:', e, flush=True); acc, rate = -1.0, 0.0
            self.k_acc, self.k_rate = float(np.clip(acc, -3.0, 3.0)), float(np.clip(rate, -0.4, 0.4))
        dt = 0.05                                                 # HUGSIM integrates with dt = 0.25 once per step; here the same ODE at the 20 Hz tick
        self.k_v = max(0.0, min(MAX_KMH / 3.6, self.k_v + self.k_acc * dt)); self.k_steer = float(np.clip(self.k_steer + self.k_rate * dt, -HUGSIM_MAX_STEER, HUGSIM_MAX_STEER))
        yaw_rate = self.k_v * math.tan(self.k_steer) / HUGSIM_L; self.k_yaw = yaw_carla + yaw_rate * dt
        hero.set_target_velocity(carla.Vector3D(self.k_v * math.cos(self.k_yaw), self.k_v * math.sin(self.k_yaw), hero.get_velocity().z))
        hero.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, math.degrees(yaw_rate)))
        return carla.VehicleControl(throttle=0.0, brake=0.0, steer=float(np.clip(self.k_steer / HUGSIM_MAX_STEER, -1, 1)))   # steer only turns the wheels visually

    def destroy(self):
        self.model = None
        try: self.torch.cuda.empty_cache()
        except Exception: pass
