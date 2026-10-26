"""On-screen steering wheel for pygame windows (CARLA manual_control and the Bench2Drive human agent).
Click inside the wheel and drag around its centre: the wheel turns with your hand like a real one.
Lock-to-lock is LOCK_DEG (default 900 deg, i.e. +-450 = full steer). Released -> self-centres.
Pedals stay on keys (W/S) — or right mouse button anywhere = brake.
"""
import math, time, os
import pygame

LOCK_DEG = float(os.environ.get('WHEEL_LOCK_DEG', '900'))   # total lock-to-lock, like a road car
RETURN_RATE = float(os.environ.get('WHEEL_RETURN_DEG_S', '360'))  # self-centring speed, deg/s, when released

class VirtualWheel:
    def __init__(self, radius=90, margin=24):
        self.r, self.margin = radius, margin
        self.angle = 0.0          # current wheel rotation, deg (+ = clockwise = steer right)
        self.grabbed = False
        self._last_hand = None    # last mouse angle around centre while grabbed
        self._t = time.time()
        self.brake_btn = False

    # ---- geometry ----
    def center(self, surf):
        w, h = surf.get_size(); return (w // 2, h - self.margin - self.r)
    def _hand_angle(self, pos, c):
        return math.degrees(math.atan2(pos[1] - c[1], pos[0] - c[0]))

    # ---- input ----
    def handle_event(self, event, surf):
        c = self.center(surf)
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and math.dist(event.pos, c) <= self.r * 1.25:
                self.grabbed = True; self._last_hand = self._hand_angle(event.pos, c)
            elif event.button == 3: self.brake_btn = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1: self.grabbed = False; self._last_hand = None
            elif event.button == 3: self.brake_btn = False
        elif event.type == pygame.MOUSEMOTION and self.grabbed:
            a = self._hand_angle(event.pos, c); d = a - self._last_hand
            d = (d + 180) % 360 - 180                      # shortest signed delta
            self.angle = max(-LOCK_DEG / 2, min(LOCK_DEG / 2, self.angle + d)); self._last_hand = a

    def update(self):
        now = time.time(); dt = now - self._t; self._t = now
        if not self.grabbed and self.angle != 0.0:        # self-centre
            step = RETURN_RATE * dt
            self.angle = 0.0 if abs(self.angle) <= step else self.angle - math.copysign(step, self.angle)

    @property
    def steer(self):  # -1..1
        return max(-1.0, min(1.0, self.angle / (LOCK_DEG / 2)))
    @property
    def active(self):  # wheel is driving the car (grabbed or not yet centred)
        return self.grabbed or abs(self.angle) > 0.5

    # ---- drawing ----
    def draw(self, surf):
        c = self.center(surf); r = self.r; a = math.radians(self.angle)
        rim = (235, 235, 235) if self.grabbed else (190, 190, 190)
        pygame.draw.circle(surf, (30, 30, 30), c, r + 8); pygame.draw.circle(surf, rim, c, r, 10)
        for k in range(3):                                 # three spokes, rotated with the wheel
            t = a + math.radians(90 + 120 * k)
            pygame.draw.line(surf, rim, c, (c[0] + r * 0.92 * math.cos(t), c[1] + r * 0.92 * math.sin(t)), 8)
        pygame.draw.circle(surf, (60, 60, 60), c, 22)
        top = (c[0] + r * math.cos(a - math.pi / 2), c[1] + r * math.sin(a - math.pi / 2))  # red marker at the "12 o'clock" of the rim
        pygame.draw.circle(surf, (220, 40, 40), (int(top[0]), int(top[1])), 9)
        pygame.draw.line(surf, (255, 255, 0), (c[0], c[1] - r - 16), (c[0], c[1] - r - 4), 3)   # fixed straight-ahead mark
        f = pygame.font.SysFont('monospace', 14, bold=True)
        surf.blit(f.render(f'{self.angle:+5.0f} deg  steer {self.steer:+.2f}', True, (255, 255, 0)), (c[0] - 80, c[1] + r + 12))
