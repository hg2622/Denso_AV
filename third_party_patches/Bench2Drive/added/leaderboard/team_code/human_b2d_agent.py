"""Human-driven Bench2Drive agent: the stock leaderboard HumanAgent (pygame, WASD)
plus the route drawn in the world as small dim waypoint dots so you know where to go, and a
speed / distance-to-goal / elapsed-time HUD. Scored exactly like any other agent."""
import math
import carla
import pygame
from pygame.locals import K_DOWN, K_LEFT, K_RIGHT, K_SPACE, K_UP, K_a, K_d, K_s, K_w, K_q, K_LSHIFT, K_RSHIFT
from leaderboard.autoagents.human_agent import HumanAgent, KeyboardControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


def get_entry_point():
    return "HumanB2DAgent"


import os, sys
sys.path.insert(0, '/home/all/carla_ws')
from virtual_wheel import VirtualWheel
WHEEL = VirtualWheel() if os.environ.get('MOUSE_STEER', '0') != '0' else None   # off by default; MOUSE_STEER=1 enables
THROTTLE = 1.0        # stock HumanAgent uses 0.8
THROTTLE_SHIFT = 0.5  # hold Shift for gentle throttle (easier to hold a lane / creep)
STEER_RATE = 5e-4     # stock 3e-4; steer per ms held
STEER_MAX = 0.7


class FastKeyboardControl(KeyboardControl):
    """Same keys as the stock controller, full throttle, faster steering, Shift = half throttle."""

    skip = False; quit = False

    def _parse_vehicle_keys(self, keys, milliseconds):
        for event in pygame.event.get():
            if WHEEL is not None and pygame.display.get_surface() is not None: WHEEL.handle_event(event, pygame.display.get_surface())
            if event.type == pygame.KEYUP and event.key == K_q:
                self._control.gear = 1 if self._control.reverse else -1
                self._control.reverse = self._control.gear < 0
            if event.type == pygame.KEYUP and event.key == pygame.K_n: self.skip = True   # N = give up this route, go to the next one
            if event.type == pygame.QUIT or (event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE): self.quit = True
        gentle = keys[K_LSHIFT] or keys[K_RSHIFT]
        self._control.throttle = (THROTTLE_SHIFT if gentle else THROTTLE) if (keys[K_UP] or keys[K_w]) else 0.0
        inc = STEER_RATE * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache = max(-STEER_MAX, self._steer_cache - inc)
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache = min(STEER_MAX, self._steer_cache + inc)
        else:
            self._steer_cache = 0.0
        self._control.steer = round(self._steer_cache, 2)
        self._control.brake = 1.0 if (keys[K_DOWN] or keys[K_s]) else 0.0
        self._control.hand_brake = bool(keys[K_SPACE])
        # virtual steering wheel (shared module; MOUSE_STEER=0 disables)
        if WHEEL is not None:
            WHEEL.update()
            if WHEEL.active: self._steer_cache = max(-STEER_MAX, min(STEER_MAX, WHEEL.steer)); self._control.steer = round(self._steer_cache, 3)
            if WHEEL.brake_btn: self._control.brake, self._control.throttle = 1.0, 0.0


class HumanB2DAgent(HumanAgent):

    def setup(self, path_to_conf_file):
        super().setup(path_to_conf_file.split("+")[0])  # Bench2Drive appends "+<route name>"
        self._controller = FastKeyboardControl(path_to_conf_file.split("+")[0])
        # side mirrors: stock HumanAgent supports them but leaves them off. Enabling adds Left/Right cameras in
        # sensors() and draws them as insets in the bottom corners of the window. MIRRORS=0 env disables.
        import os
        from leaderboard.autoagents.human_agent import HumanInterface
        if os.environ.get('MIRRORS', '1') != '0':
            self._left_mirror = self._right_mirror = True
            self._side_scale = 0.28
            self._hic = HumanInterface(self.camera_width, self.camera_height, self._side_scale, True, True)
        self._route_drawn = False
        self._font = pygame.font.SysFont("monospace", 22, bold=True)
        self._t0 = None
        self._route_info = ''   # "NN/44 ScenarioType (route id, Town)" when TOUR_ROUTES (tour44_routes.txt) is set

    def _find_route_info(self):
        f = os.environ.get('TOUR_ROUTES', '')
        if not f or not self._global_plan_world_coord: return
        try:
            import xml.etree.ElementTree as ET
            xml = os.environ.get('TOUR_XML', '/home/all/carla_ws/Bench2Drive/leaderboard/data/tour44.xml')
            starts = {r.get('id'): r.find('waypoints').find('position') for r in ET.parse(xml).getroot().findall('route')}
            p0 = self._global_plan_world_coord[0][0].location
            best = min(starts, key=lambda k: math.hypot(float(starts[k].get('x')) - p0.x, float(starts[k].get('y')) - p0.y))
            for l in open(f):
                if l.startswith('#'): continue
                n, t, rid, town = l.split()
                if rid == best: self._route_info = f'{n}/44 {t}  (route {rid}, {town})'; print(f'[tour] {self._route_info}', flush=True); return
        except Exception as e: print('[tour] route info failed:', e, flush=True)

    def _draw_route(self):
        world = CarlaDataProvider.get_world()
        pts = [tf.location for tf, _ in self._global_plan_world_coord]
        # small, dim waypoint dots on the road (no bright arrows/lines in your view); one dot every ~5 m
        last = None
        for p in pts:
            if last is not None and p.distance(last) < 5.0: continue
            world.debug.draw_point(p + carla.Location(z=0.15), size=0.08, color=carla.Color(60, 110, 60), life_time=0); last = p
        world.debug.draw_point(pts[-1] + carla.Location(z=1.0), size=0.3, color=carla.Color(200, 40, 40), life_time=0)
        self._goal = pts[-1]
        self._route_drawn = True

    def run_step(self, input_data, timestamp):
        if not self._route_drawn and self._global_plan_world_coord:
            self._draw_route(); self._find_route_info()
        if self._controller.quit: print('[tour] ESC pressed - quitting (resume later with the same command)', flush=True); raise KeyboardInterrupt('ESC pressed')
        if self._controller.skip: self._controller.skip = False; raise RuntimeError('route skipped with N')
        if self._t0 is None:
            self._t0 = timestamp
        control = super().run_step(input_data, timestamp)
        # HUD on top of the pygame display drawn by HumanInterface
        ego = CarlaDataProvider.get_hero_actor()
        if ego is not None:
            v = ego.get_velocity(); speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
            dist = ego.get_location().distance(self._goal) if self._route_drawn else float("nan")
            if int(timestamp * 20) % 20 == 0: print(f"[HUD] t={timestamp - self._t0:.1f}s speed={speed:.1f} km/h goal={dist:.1f} m ctrl thr={control.throttle:.2f} brk={control.brake:.2f} str={control.steer:.2f}", flush=True)
            nxt = ''
            if self._route_drawn:
                loc = ego.get_location(); plan = self._global_plan_world_coord
                i = min(range(len(plan)), key=lambda k: plan[k][0].location.distance(loc))
                # one BRIGHT green marker ~20 m ahead along the route (moves with you), redrawn every tick
                j, d = i, 0.0
                while j + 1 < len(plan) and d < 20.0: d += plan[j][0].location.distance(plan[j + 1][0].location); j += 1
                ahead = plan[j][0].location
                CarlaDataProvider.get_world().debug.draw_point(ahead + carla.Location(z=1.0), size=0.35, color=carla.Color(0, 255, 0), life_time=0.12)
                for tf, cmd in plan[i:]:
                    name = str(cmd).split('.')[-1]
                    if name not in ('LANEFOLLOW', 'VOID'): nxt = f"next: {name.replace('CHANGELANE', 'LANE CHANGE ')} in {tf.location.distance(loc):.0f} m"; break
                else: nxt = 'next: follow the lane to the goal'
            lines = [f"{speed:5.1f} km/h", f"goal {dist:6.1f} m", f"t {timestamp - self._t0:6.1f} s", nxt, self._route_info,
                     "BRIGHT green = 20 m ahead on the route, dim dots = route, red = goal   W/S A/D  Shift=gentle  Q reverse  Space handbrake",
                     "N = skip this route   ESC = quit (resume later with the same command)"]
            surf = self._hic._display
            for i, txt in enumerate(lines):
                surf.blit(self._font.render(txt, True, (255, 255, 0)), (10, 10 + 26 * i))
            if WHEEL is not None: WHEEL.draw(surf)
            pygame.display.flip()
        return control
