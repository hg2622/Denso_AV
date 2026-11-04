#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    : manual.py
@Date    : 2026/11/02

Human-driven ego: YOU drive while the fine-tuned CBV policy (RIFT / Pluto / GRPO / ...) takes over nearby NPCs and
tries to create trouble around you. Drop-in replacement for the rule-based egos, so every CBV config works unchanged.

Window: chase camera + HUD (speed, gear, which keys do what, how many NPCs the policy currently controls).
Keys: W/S or UP/DOWN throttle / brake, A/D or LEFT/RIGHT steer, SPACE handbrake, Q toggle reverse,
      SHIFT drive gently, TAB cycle camera (chase / bonnet / top-down), ESC end the route.
The route is still scored: the leaderboard records your driving score and the CBV metrics exactly as for a model ego.
"""
import math
import os
from typing import Dict

import numpy as np

from rift.ego.base_policy import EgoBasePolicy


class Manual(EgoBasePolicy):
    name = 'manual'
    type = 'unlearnable'

    CAMS = [('chase', (-6.5, 0.0, 2.8), (-12.0, 0.0)),
            ('bonnet', (0.6, 0.0, 1.25), (0.0, 0.0)),
            ('top-down', (0.0, 0.0, 28.0), (-89.0, 0.0))]

    def __init__(self, config, logger):
        super().__init__(config, logger)
        self.logger = logger
        self.num_scenario = config['num_scenario']
        self.width = int(config.get('window_width', 1280))
        self.height = int(config.get('window_height', 720))
        self.steer_rate = float(config.get('steer_rate', 1.8))      # how fast steering winds on, 1/s
        self.steer_return = float(config.get('steer_return', 3.0))  # how fast it self-centres, 1/s
        self._cam_idx = 0
        self._steer = 0.0
        self._reverse = False
        self._display = None
        self._camera = None
        self._image = None
        self._ego = None
        self._quit = False
        self._pygame = None
        self._font = None
        self._clock = None
        self._cbv_count = 0

    # ---------------------------------------------------------------- setup
    def _ensure_window(self):
        if self._display is not None:
            return
        os.environ.setdefault('SDL_VIDEO_WINDOW_POS', '60,60')
        import pygame
        self._pygame = pygame
        pygame.init()
        pygame.font.init()
        self._display = pygame.display.set_mode((self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('RIFT - you drive, the policy drives the NPCs')
        self._font = pygame.font.SysFont('monospace', 17, bold=True)
        self._clock = pygame.time.Clock()

    def _attach_camera(self, ego):
        import carla
        if self._camera is not None:
            try:
                self._camera.stop(); self._camera.destroy()
            except Exception:
                pass
            self._camera = None
        world = ego.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.width))
        bp.set_attribute('image_size_y', str(self.height))
        bp.set_attribute('fov', '100')
        _, (x, y, z), (pitch, yaw) = self.CAMS[self._cam_idx]
        tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw))
        self._camera = world.spawn_actor(bp, tf, attach_to=ego)
        self._image = None
        self._camera.listen(self._on_image)

    def _on_image(self, image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        self._image = arr[:, :, :3][:, :, ::-1]   # BGRA -> RGB

    def set_ego_and_route(self, ego_vehicles, info, sampled_scenario_configs):
        self.ego_vehicles = ego_vehicles
        self._ego = ego_vehicles[0] if ego_vehicles else None
        self._steer = 0.0
        self._reverse = False
        self._quit = False
        if self._ego is not None:
            self._ensure_window()
            self._attach_camera(self._ego)

    # ---------------------------------------------------------------- input
    def _read_keys(self, dt):
        pg = self._pygame
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self._quit = True
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE:
                    self._quit = True
                elif event.key == pg.K_q:
                    self._reverse = not self._reverse
                elif event.key == pg.K_TAB:
                    self._cam_idx = (self._cam_idx + 1) % len(self.CAMS)
                    if self._ego is not None:
                        self._attach_camera(self._ego)
        keys = pg.key.get_pressed()
        gentle = keys[pg.K_LSHIFT] or keys[pg.K_RSHIFT]
        throttle = (0.45 if gentle else 1.0) if (keys[pg.K_w] or keys[pg.K_UP]) else 0.0
        brake = 1.0 if (keys[pg.K_s] or keys[pg.K_DOWN]) else 0.0
        if keys[pg.K_SPACE]:
            brake = 1.0
        left = keys[pg.K_a] or keys[pg.K_LEFT]
        right = keys[pg.K_d] or keys[pg.K_RIGHT]
        if left and not right:
            self._steer = max(-1.0, self._steer - self.steer_rate * dt)
        elif right and not left:
            self._steer = min(1.0, self._steer + self.steer_rate * dt)
        else:   # self-centre
            step = self.steer_return * dt
            self._steer = 0.0 if abs(self._steer) < step else self._steer - math.copysign(step, self._steer)
        return throttle, self._steer, brake

    # ---------------------------------------------------------------- render
    def _render(self, speed_kmh):
        pg = self._pygame
        if self._image is not None:
            surface = pg.surfarray.make_surface(self._image.swapaxes(0, 1))
            self._display.blit(surface, (0, 0))
        else:
            self._display.fill((20, 20, 24))
        lines = [
            f"{speed_kmh:5.1f} km/h   {'REVERSE' if self._reverse else 'DRIVE  '}   steer {self._steer:+.2f}   "
            f"view {self.CAMS[self._cam_idx][0]}",
            f"NPCs under policy control: {self._cbv_count}",
            "W/S throttle-brake   A/D steer   SHIFT gentle   Q reverse   SPACE handbrake   TAB view   ESC end route",
        ]
        for i, text in enumerate(lines):
            label = self._font.render(text, True, (255, 230, 90))
            bg = pg.Surface((label.get_width() + 10, label.get_height() + 4))
            bg.set_alpha(140)
            bg.fill((0, 0, 0))
            self._display.blit(bg, (8, 8 + 24 * i))
            self._display.blit(label, (13, 10 + 24 * i))
        pg.display.flip()

    def _count_cbvs(self):
        """How many nearby NPCs the CBV policy has taken over right now (drawn from the shared data provider)."""
        if self._ego is None:
            return 0
        try:
            from rift.scenario.tools.carla_data_provider import CarlaDataProvider
            return len(CarlaDataProvider.get_CBVs_by_ego(self._ego.id) or {})
        except Exception:
            return 0

    # ---------------------------------------------------------------- step
    def get_action(self, obs, infos, deterministic=False) -> Dict[str, np.ndarray]:
        actions = {}
        if self._display is None and self.ego_vehicles:
            self._ensure_window()
            self._attach_camera(self.ego_vehicles[0])

        dt = (self._clock.tick(60) / 1000.0) if self._clock else 0.05
        throttle, steer, brake = (0.0, 0.0, 1.0)
        if os.environ.get('RIFT_AUTODRIVE') == '1':
            # self-test: stand still for ~6 s (which used to trigger "Agent got blocked"), then drive forward
            self._auto_t = getattr(self, '_auto_t', 0) + 1
            # drive like a cautious human: pause at the start, then a modest throttle (the case that used to engage no NPCs)
            throttle, steer, brake = ((0.0, 0.0, 0.0) if self._auto_t < 60 else (0.35, 0.0, 0.0))
            n = self._count_cbvs()
            self._auto_peak = max(getattr(self, '_auto_peak', 0), n)
            if self._auto_t % 100 == 0:
                print(f'[autodrive] tick {self._auto_t}  NPCs now: {n}  peak so far: {self._auto_peak}', flush=True)
        elif self._display is not None:
            throttle, steer, brake = self._read_keys(dt)

        for info in infos:
            # [throttle, steer, brake, reverse] - convert_rule_action() passes the 4th element through as the gear
            actions[info['env_id']] = [throttle, steer, brake, float(self._reverse)]
        self._cbv_count = self._count_cbvs()

        if self._ego is not None:
            try:
                v = self._ego.get_velocity()
                speed = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            except Exception:
                speed = 0.0
            if self._display is not None:
                self._render(speed)

        if self._quit:
            raise KeyboardInterrupt('ESC pressed: ending the route')
        return {'ego_actions': actions}

    # ---------------------------------------------------------------- teardown
    def clean_up(self):
        if self._camera is not None:
            try:
                self._camera.stop(); self._camera.destroy()
            except Exception:
                pass
            self._camera = None
        self._image = None

    def finish(self):
        self.clean_up()
        if self._display is not None and self._pygame is not None:
            try:
                self._pygame.quit()
            except Exception:
                pass
            self._display = None
