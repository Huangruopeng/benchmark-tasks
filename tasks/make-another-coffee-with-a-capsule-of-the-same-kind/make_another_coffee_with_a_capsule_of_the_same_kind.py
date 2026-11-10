"""On the station table stands a capsule coffee machine: a black body with a silver top, a capsule holder in the top on
the right, a clear water tank at the left end, a round brew button, and a white cup on the drip tray under its spout. A
used capsule is stuck in the holder (pierced capsules stick). To the right of the machine stand a green recycling tin
for used capsules, with a round hole in its lid, and a wooden tray holding three new capsules of three different kinds
(colours).

Task: make another coffee of the same kind. Take the used capsule out of the holder (hold the machine down with the
other hand, by its water tank: the capsule sticks, and the light machine lifts with it otherwise) and drop it into the
recycling tin through the hole in its lid. Only then take, from the tray, the new capsule of the same kind (the same
colour) as the one you threw away, put it in the holder, and press the brew button. Do not move any other capsule. Then
take both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=make_another_coffee_with_a_capsule_of_the_same_kind.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the machine: body frame origin on the table under the middle of its footprint; length along y -------------------
MCX, MCY = 0.315, 0.025
M_X, M_Y, M_H = 0.120, 0.250, 0.070  # the body: depth (x), width (y), height of its top
M_MASS = 0.30
HOLD_DY = -0.075  # the capsule holder's middle, from the machine's middle (y)
HOLD_DX = 0.005
HOLE_R = 0.036  # half the holder's (square) hole: roomy, so a capsule let go a little off-centre drops in
POCKET = 0.012  # the holder's depth
TANK_DY, TANK = 0.084, (0.070, 0.050, 0.045)  # the water tank's middle (y) and size (x, y, height above the top)
BUTTON_DY, BUTTON_DX, BUTTON_R = -0.010, -0.035, 0.012
SPOUT_DY = -0.075
CUP_H = 0.034
# -- capsules: body origin at the middle of the bottom; the foil lid on top ---------------------------------------------
CAP_BOT_R, CAP_TOP_R, CAP_H, RIM_R, RIM_T = 0.021, 0.024, 0.034, 0.028, 0.003
CAP_MASS = 0.030
HOLDER = np.array([MCX + HOLD_DX, MCY + HOLD_DY, TOP + M_H - POCKET + 0.001])  # a seated capsule's origin
KINDS = {"red": (0.78, 0.10, 0.10), "blue": (0.12, 0.30, 0.75), "gold": (0.85, 0.65, 0.15)}
USED_KIND = "blue"
TRAY_X, TRAY_Y = (0.265, 0.325, 0.385), -0.195  # the new capsules' places on the tray (red, gold, blue)
TRAY_ORDER = ("red", "gold", "blue")
# -- the recycling tin ----------------------------------------------------------------------------------------------
BIN_X, BIN_Y, BIN_NX, BIN_NY, BIN_H = 0.330, -0.345, 0.150, 0.090, 0.085
BIN_HOLE, BIN_HOLE_X = 0.033, 0.035  # the lid's square hole: half its size, its middle along x (over the back part)
# -- rules ------------------------------------------------------------------------------------------------------------
MOVED = 0.015  # a capsule moved this far from its place has been taken
SEAT_XY, SEAT_Z, SEAT_TILT = 0.018, 0.008, 15.0
PRESS_XY, PRESS_DZ = 0.022, 0.012  # a TCP this close over the button, this low: pressed
BREW_S = 3.0

MACHINE_C, SILVER, DARK, TANK_C, WATER_C = (0.07, 0.07, 0.08), (0.72, 0.73, 0.75), (0.16, 0.16, 0.17), \
    (0.75, 0.85, 0.92), (0.55, 0.72, 0.90)
CUP_C, COFFEE_C, CREMA_C, TRAY_C, BIN_C, FOIL = (0.96, 0.96, 0.95), (0.25, 0.14, 0.07), (0.62, 0.42, 0.22), \
    (0.62, 0.45, 0.28), (0.15, 0.45, 0.25), (0.82, 0.83, 0.85)
BUTTON_C, BUTTON_LIT = (0.30, 0.30, 0.32), (0.95, 0.95, 0.80)
TOP_C, LID_C = (0.20, 0.20, 0.22), (0.58, 0.70, 0.80)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _machine() -> Body:
    """The machine: body, silver top with the holder ring, tank, button, spout, drip tray, a cup on it."""
    s = []
    hx, hy = HOLD_DX, HOLD_DY
    s.append(BoxShape((0.0, 0.0, (M_H - POCKET) / 2), (M_X, M_Y, M_H - POCKET), MACHINE_C))  # its top: the pocket floor
    y0, y1 = -M_Y / 2, M_Y / 2

    def ring(z0, z1, color):  # four boxes round the holder's square hole, from z0 to z1
        zc, zh = (z0 + z1) / 2, z1 - z0
        s.append(BoxShape((0.0, (hy + HOLE_R + y1) / 2, zc), (M_X, y1 - hy - HOLE_R, zh), color))
        s.append(BoxShape((0.0, (y0 + hy - HOLE_R) / 2, zc), (M_X, hy - HOLE_R - y0, zh), color))
        s.append(BoxShape(((hx + HOLE_R + M_X / 2) / 2, hy, zc), (M_X / 2 - hx - HOLE_R, 2 * HOLE_R, zh), color))
        s.append(BoxShape(((hx - HOLE_R - M_X / 2) / 2, hy, zc), (hx - HOLE_R + M_X / 2, 2 * HOLE_R, zh), color))

    ring(M_H - POCKET, M_H - 0.006, MACHINE_C)
    ring(M_H - 0.006, M_H, TOP_C)  # the top, dark charcoal
    for k in range(20):  # a brushed steel trim ring round the holder
        a = 2 * math.pi * k / 20
        s.append(BoxShape((hx + 0.043 * math.cos(a), hy + 0.043 * math.sin(a), M_H + 0.0006), (0.014, 0.005, 0.001),
                          SILVER, yaw=a + math.pi / 2, collide=False))
    s.append(BoxShape((-M_X / 2 + 0.004, 0.0, M_H + 0.0006), (0.006, M_Y - 0.004, 0.001), SILVER, collide=False))
    s.append(BoxShape((hx, hy, M_H - POCKET + 0.0005), (2 * HOLE_R - 0.002, 2 * HOLE_R - 0.002, 0.001), DARK,
                      collide=False))  # the pocket's dark floor
    s.append(CylShape((hx, hy, M_H + 0.0005), 0.030, 0.001, DARK, collide=False))  # the holder's dark round seat
    # the water tank (clear blue, water inside) at the left end
    tx, ty, tz = TANK
    s.append(BoxShape((0.0, TANK_DY, M_H + tz / 2), (tx, ty, tz), TANK_C))
    s.append(BoxShape((0.0, TANK_DY, M_H + tz * 0.35), (tx - 0.004, ty - 0.004, tz * 0.6), WATER_C, collide=False))
    s.append(BoxShape((0.0, TANK_DY, M_H + tz + 0.003), (tx + 0.004, ty + 0.004, 0.006), LID_C))  # its tinted lid
    s.append(BoxShape((tx / 2 - 0.008, TANK_DY, M_H + tz + 0.0065), (0.008, ty - 0.010, 0.001), DARK, collide=False))
    # the brew button (drawn ring; the button itself is a separate shape, shown lit after brewing)
    s.append(CylShape((BUTTON_DX, BUTTON_DY, M_H + 0.002), BUTTON_R + 0.004, 0.004, DARK, collide=False))
    s.append(CylShape((BUTTON_DX, BUTTON_DY, M_H + 0.004), BUTTON_R, 0.004, BUTTON_C, collide=False))
    # the spout at the front under the holder, the drip tray and the cup
    s.append(BoxShape((-M_X / 2 - 0.018, SPOUT_DY, M_H - 0.010), (0.036, 0.040, 0.016), MACHINE_C))
    s.append(CylShape((-M_X / 2 - 0.030, SPOUT_DY, M_H - 0.022), 0.006, 0.008, SILVER, collide=False))
    s.append(BoxShape((-M_X / 2 - 0.035, SPOUT_DY, 0.006), (0.070, 0.090, 0.012), SILVER))  # the drip tray
    for k in range(5):
        s.append(BoxShape((-M_X / 2 - 0.035, SPOUT_DY - 0.032 + 0.016 * k, 0.0125), (0.060, 0.004, 0.001), DARK,
                          collide=False))
    cx = -M_X / 2 - 0.032
    s.append(CylShape((cx, SPOUT_DY, 0.012 + CUP_H / 2), 0.024, CUP_H, CUP_C))  # the cup (closed: its coffee is drawn)
    s.append(CylShape((cx, SPOUT_DY, 0.012 + CUP_H + 0.0005), 0.021, 0.001, (0.88, 0.88, 0.86), collide=False))
    s.append(BoxShape((cx - 0.002, SPOUT_DY + 0.030, 0.012 + CUP_H / 2), (0.008, 0.012, 0.020), CUP_C, collide=False))
    return Body.build("machine", pos=(MCX, MCY, TOP), mass=M_MASS, shapes=s, color=MACHINE_C, friction=(1.2, 1.0),
                      com=(0.0, 0.02, 0.030))


# drawn shapes the task file shows while it runs (the index of each in the "fx" body)
_FX: dict = {}


def _fx() -> Body:
    """Drawn only, never colliding: the lit brew button, the coffee stream, and the coffee rising in the cup."""
    s = []
    bx, by = MCX + BUTTON_DX, MCY + BUTTON_DY
    _FX["button"] = [len(s)]
    s.append(CylShape((bx, by, TOP + M_H + 0.0065), BUTTON_R + 0.0005, 0.0012, BUTTON_LIT, collide=False))
    cx, cy = MCX - M_X / 2 - 0.032, MCY + SPOUT_DY
    _FX["stream"] = [len(s)]
    s.append(CylShape((cx + 0.002, cy, TOP + 0.012 + CUP_H + 0.008), 0.0025, 0.020, COFFEE_C, collide=False))
    _FX["level"] = []
    for k in range(4):  # the coffee level rising in four steps
        _FX["level"].append(len(s))
        s.append(CylShape((cx, cy, TOP + 0.012 + CUP_H + 0.0012 + 0.0002 * k), 0.0205, 0.001,
                          COFFEE_C if k < 3 else CREMA_C, collide=False))
    return Body.build("fx", pos=(0.0, 0.0, 0.0), mass=0.1, kinematic=True, shapes=s, color=COFFEE_C)


def _capsule(name: str, kind: str, pos) -> Body:
    """A coffee capsule: a tapered cup in its kind's colour, a rim, a silver foil lid with a printed ring."""
    c = KINDS[kind]
    s = [CylShape((0.0, 0.0, CAP_H * 0.25), CAP_BOT_R, CAP_H * 0.5, c),
         CylShape((0.0, 0.0, CAP_H * 0.70), CAP_TOP_R, CAP_H * 0.4, c),
         CylShape((0.0, 0.0, CAP_H - RIM_T / 2), RIM_R, RIM_T, c),
         CylShape((0.0, 0.0, CAP_H + 0.0003), RIM_R - 0.003, 0.0006, FOIL, collide=False),
         CylShape((0.0, 0.0, CAP_H + 0.0007), RIM_R - 0.011, 0.0006, c, collide=False)]
    return Body.build(name, pos=tuple(float(v) for v in pos), mass=CAP_MASS, shapes=s, color=c, friction=(1.0, 0.9),
                      damping=(0.05, 0.2))


def _tray() -> Body:
    """A wooden tray with a shallow round pocket for each capsule."""
    x0, x1 = min(TRAY_X) - 0.040, max(TRAY_X) + 0.040
    s = [BoxShape(((x0 + x1) / 2 - TRAY_X[1], 0.0, 0.004), (x1 - x0, 0.080, 0.008), TRAY_C)]
    for x in TRAY_X:
        s.append(CylShape((x - TRAY_X[1], 0.0, 0.0085), RIM_R + 0.003, 0.001, (0.50, 0.35, 0.20), collide=False))
    return Body.build("tray", pos=(TRAY_X[1], TRAY_Y, TOP), mass=0.3, kinematic=True, shapes=s, color=TRAY_C,
                      friction=(0.8, 0.7))


def _tin() -> Body:
    """The recycling tin: four walls, a lid with a square hole over its back part, and inside a slick ramp that slides
    a capsule dropped through the hole forward, under the solid front part of the lid (out of sight)."""
    w, nx, ny, h = 0.004, BIN_NX, BIN_NY, BIN_H
    s = [BoxShape((0.0, 0.0, w / 2), (nx + 2 * w, ny + 2 * w, w), BIN_C)]
    for sg in (-1.0, 1.0):
        s.append(BoxShape((sg * (nx / 2 + w / 2), 0.0, h / 2), (w, ny + 2 * w, h), BIN_C))
        s.append(BoxShape((0.0, sg * (ny / 2 + w / 2), h / 2), (nx, w, h), BIN_C))
    hx0, hx1 = BIN_HOLE_X - BIN_HOLE, BIN_HOLE_X + BIN_HOLE  # the hole along x; across, it is BIN_HOLE either side
    s.append(BoxShape(((-nx / 2 - w + hx0) / 2, 0.0, h + w / 2), (hx0 + nx / 2 + w, ny + 2 * w, w), BIN_C))  # front
    if nx / 2 + w - hx1 > 0.001:
        s.append(BoxShape(((hx1 + nx / 2 + w) / 2, 0.0, h + w / 2), (nx / 2 + w - hx1, ny + 2 * w, w), BIN_C))
    for sg in (-1.0, 1.0):
        s.append(BoxShape((BIN_HOLE_X, sg * (BIN_HOLE + ny / 2 + w) / 2, h + w / 2),
                          (2 * BIN_HOLE, ny / 2 + w - BIN_HOLE, w), BIN_C))
    s.append(BoxShape((BIN_HOLE_X, 0.0, h + w + 0.0005), (2 * BIN_HOLE + 0.006, 2 * BIN_HOLE + 0.006, 0.001),
                      (0.10, 0.30, 0.16), collide=False))  # a dark rim round the hole
    # the ramp inside: high at the back, low at the front
    ang = math.radians(-25.0)
    s.append(BoxShape((0.0, 0.0, 0.020), (nx / math.cos(ang) - 0.002, ny - 0.002, 0.004), (0.10, 0.10, 0.10),
                      quat_xyzw=(0.0, math.sin(ang / 2), 0.0, math.cos(ang / 2))))
    # a recycling mark on the lid's front part (three arrows, drawn as three bars)
    for k in range(3):
        a = k * 2 * math.pi / 3
        s.append(BoxShape((-0.040 + 0.012 * math.cos(a), 0.012 * math.sin(a), h + w + 0.0005), (0.014, 0.004, 0.001),
                          (0.85, 0.95, 0.85), yaw=a + math.pi / 2, collide=False))
    return Body.build("tin", pos=(BIN_X, BIN_Y, TOP), mass=0.5, kinematic=True, shapes=s, color=BIN_C,
                      friction=(0.15, 0.10), slick=True)


OBJECTS = [_machine(), _tray(), _tin(), _fx(), _capsule("used", USED_KIND, HOLDER)] + \
    [_capsule(f"new_{k}", k, (x, TRAY_Y, TOP + 0.0085)) for x, k in zip(TRAY_X, TRAY_ORDER)]
# the used capsule is stuck in the holder: a joint that breaks under a firm pull (machine held down)
STUCK = hinge("stuck", "machine", "used", anchor=tuple(HOLDER.tolist()), axis="Z", limits_deg=(-0.5, 0.5),
              break_force=6.0, break_torque=0.6)

# hide the drawn effects at spawn, and remember where they are
_PRIM: dict = {}
_author_glues_plain = bodykit._author_glues


def _author_glues(stage, prim_path: str, spec) -> None:
    _author_glues_plain(stage, prim_path, spec)
    if spec.name == "fx":
        from pxr import UsdGeom

        _PRIM["fx"] = prim_path
        for idx in _FX.values():
            for i in idx:
                prim = stage.GetPrimAtPath(f"{prim_path}/body/s{i}")
                if prim:
                    UsdGeom.Imageable(prim).MakeInvisible()


bodykit._author_glues = _author_glues
_SHOWN: dict = {}


def _show(key: str, i: int, on: bool) -> None:
    if _SHOWN.get((key, i)) == on or "fx" not in _PRIM:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(f"{_PRIM['fx']}/body/s{_FX[key][i]}")
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()
    _SHOWN[(key, i)] = on


# -- followed after every control step --------------------------------------------------------------------------------

NEW = [f"new_{k}" for k in TRAY_ORDER]
_START = {f"new_{k}": np.array([x, TRAY_Y, TOP + 0.0085]) for x, k in zip(TRAY_X, TRAY_ORDER)}
MATCH = f"new_{USED_KIND}"


def _fresh() -> dict:
    return {"steps": 0, "wrong": False, "early": False, "binned": False, "seated": False, "brewed_at": None,
            "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("COFFEE_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _SHOWN.clear()
    for key, idx in _FX.items():
        for i in range(len(idx)):
            _show(key, i, False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _in_tin(p) -> bool:
    return abs(p[0] - BIN_X) <= BIN_NX / 2 and abs(p[1] - BIN_Y) <= BIN_NY / 2 and p[2] <= TOP + BIN_H - 0.005


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    t = _EP["steps"] * 0.05
    pm, rm = _pose(base, origin, "machine")
    pu, _ = _pose(base, origin, "used")
    if _in_tin(pu):
        _EP["binned"] = True
    hold = np.array([pm[0], pm[1], pm[2]]) + rm @ np.array([HOLD_DX, HOLD_DY, M_H - POCKET + 0.001])
    news = {n: _pose(base, origin, n) for n in NEW}
    for n, (p, r) in news.items():
        if float(np.linalg.norm(p - _START[n])) > MOVED:
            if n != MATCH:
                _EP["wrong"] = True  # a capsule of another kind was taken
            elif not _EP["binned"]:
                _EP["early"] = True  # the new capsule was taken before the used one was in the tin
    p, r = news[MATCH]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    seated = (float(np.linalg.norm(p[:2] - hold[:2])) <= SEAT_XY and abs(p[2] - hold[2]) <= SEAT_Z
              and tilt <= SEAT_TILT)
    _EP["seated"] = seated
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    button = pm + rm @ np.array([BUTTON_DX, BUTTON_DY, M_H + 0.006])
    pressed = any(float(np.linalg.norm(tc[:2] - button[:2])) <= PRESS_XY and tc[2] <= button[2] + PRESS_DZ
                  for tc in tcps)
    if pressed and seated and _EP["brewed_at"] is None and not (_EP["wrong"] or _EP["early"]):
        _EP["brewed_at"] = t
    if _EP["brewed_at"] is not None:
        dt = t - _EP["brewed_at"]
        _show("button", 0, True)
        _show("stream", 0, dt < BREW_S)
        for k in range(4):
            _show("level", k, dt >= (k + 1) * BREW_S / 4)
    tilt_m = math.degrees(math.acos(max(-1.0, min(1.0, float(rm[2, 2])))))
    _EP["s"] = {"machine_down": pm[2] - TOP < 0.004 and tilt_m < 3.0}
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        tl, trc = tcps
        print(f"[coffee] t={t:5.2f} machine={np.round((pm - [0, 0, TOP]) * 1000).astype(int).tolist()} tilt={tilt_m:.1f}"
              f" used={np.round((pu - [0, 0, TOP]) * 1000).astype(int).tolist()} binned={int(_EP['binned'])}"
              f" match={np.round((p - [0, 0, TOP]) * 1000).astype(int).tolist()} seated={int(seated)}"
              f" wrong={int(_EP['wrong'])} early={int(_EP['early'])} brewed={_EP['brewed_at']}"
              f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
              f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a capsule of another kind was taken from the tray, or the right one was taken before the used capsule
    was in the tin. Otherwise 1.0 with the coffee brewed (the right capsule seated in the holder and the brew button
    pressed), the capsule still seated, the machine standing on the table and both arms back where they started, at the
    end; 0.75 with the coffee brewed; 0.5 with the right capsule seated in the holder; 0.25 with the used capsule in the
    tin; else 0.0."""
    if _EP["wrong"] or _EP["early"]:
        return 0.0
    if _EP["brewed_at"] is not None:
        if _EP["seated"] and _EP["s"].get("machine_down") and _arms_home():
            return 1.0
        return 0.75
    if _EP["seated"]:
        return 0.5
    return 0.25 if _EP["binned"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the machine and the new capsule still: the trial ends there."""
    return score(objects) >= 1.0 and objects["machine"].resting() and objects[MATCH].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the water tank's top from above and holds the machine down. The right takes the used capsule by
# its top and pulls it up out of the holder (the stuck joint gives), carries it over the tin and drops it through the
# hole. The left lets go of the tank and goes home. The right takes the blue capsule from the tray, carries it over the
# holder, lowers it in and lets go, then presses the brew button and goes home.

OVER = 0.035
LEAD_S, LEAD_STEPS = 3.0, 15
CAP_FINGER = -0.036  # open round a capsule's rim (a wider pre-open shoves it further as the fingers close)
CAP_GRIP_DZ = 0.004  # the TCP this far below the used capsule's foil (the fingers round its rim and upper body)
NEW_GRIP_DZ = 0.018  # the new capsule is taken deeper in the jaw (the tray leaves room below)
DROP_DZ = 0.010  # the new capsule is let go with its underside this far above its seat (already down in the hole)
RELEASE_FINGER = -0.036  # open past the rim (5.6 cm) to let go
TANK_FINGER = -0.034  # 3 mm wider than the tank each side: a wider pre-open shoves the machine as it closes
YAW_R, YAW_L = 0.0, 0.0
PLACE_DY = 0.021  # a capsule hangs this far off the TCP along the closing axis (-y): aim the TCP this much +y
MACHINE_SHIFT = np.array([0.004, -0.006, 0.0])  # where the left's grip on the tank leaves the machine (measured)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena.constants import READY

    rr, rl = grip_rotation(YAW_R), grip_rotation(YAW_L)
    L, R = [], []
    # the left: hold the machine down by its water tank
    tank = np.array([MCX, MCY + TANK_DY, TOP + M_H + TANK[2] - 0.012])
    L += _joint_path("left", np.array(READY, float), _good_ik("left", tank + [0, 0, 0.035], rl), "over the water tank",
                     TANK_FINGER)
    L.append(_w(1.2, tank, TANK_FINGER, "down round the tank's top", rl))
    L.append(_w(0.6, tank + [0, 0, -0.004], FINGER_CLOSED, "hold the machine down by its tank", rl))
    t_left_holding = LEAD_S + 1.8
    # the right: pull the used capsule out, drop it in the tin
    cap_top = HOLDER + [0.0, 0.0, CAP_H]
    grip_used = cap_top + [0.0, 0.0, -CAP_GRIP_DZ + 0.004]
    R += _joint_path("right", np.array(READY, float), _good_ik("right", grip_used + [0, 0, OVER], rr),
                     "over the used capsule", CAP_FINGER)
    R.append(_w(t_left_holding - LEAD_S + 0.4, grip_used + [0, 0, OVER], CAP_FINGER, "wait for the machine to be held",
                rr))
    R.append(_w(1.2, grip_used, CAP_FINGER, "down round the used capsule", rr))
    R.append(_w(0.6, grip_used, FINGER_CLOSED, "take it", rr))
    R.append(_w(1.5, grip_used + [0, 0, 0.035], FINGER_CLOSED, "pull it up out of the holder", rr))
    hole = np.array([BIN_X + BIN_HOLE_X, BIN_Y, TOP + BIN_H])
    over_hole = hole + [0.0, 0.0, CAP_H + 0.020]
    R.append(_w(1.0, grip_used + [0, 0, 0.050], FINGER_CLOSED, "lift it clear", rr))
    R.append(_w(2.0, over_hole, FINGER_CLOSED, "carry it over the tin's hole", rr))
    R.append(_w(0.8, over_hole + [0, 0, -0.015], FINGER_CLOSED, "lower it into the hole", rr))
    R.append(_w(0.8, over_hole + [0, 0, -0.015], CAP_FINGER, "drop it into the tin", rr))
    R.append(_w(0.8, over_hole + [0, 0, 0.030], CAP_FINGER, "rise clear", rr))
    t_dropped = sum(w.duration for w in R)
    # the left lets go and goes home
    L.append(_w(t_dropped - sum(w.duration for w in L), tank + [0, 0, -0.004], FINGER_CLOSED,
                "keep holding until the capsule is out", rl))
    L.append(_w(0.6, tank, TANK_FINGER, "let go of the tank", rl))
    L.append(_w(0.8, tank + [0, 0, 0.035], TANK_FINGER, "rise clear", rl))
    L += _joint_path("left", _good_ik("left", tank + [0, 0, 0.035], rl), np.array(READY, float), "back home",
                     TANK_FINGER)
    # the right takes the blue capsule, puts it in the holder, presses brew
    k = TRAY_ORDER.index(USED_KIND)
    new_top = np.array([TRAY_X[k], TRAY_Y, TOP + 0.0085 + CAP_H])
    grip_new = new_top + [0.0, 0.0, -NEW_GRIP_DZ]
    R.append(_w(2.0, grip_new + [0, 0, OVER], CAP_FINGER, "over the new capsule of the same kind (blue)", rr))
    R.append(_w(1.2, grip_new, CAP_FINGER, "down round it", rr))
    R.append(_w(0.6, grip_new, FINGER_CLOSED, "take it", rr))
    R.append(_w(1.2, grip_new + [0, 0, 0.070], FINGER_CLOSED, "lift it", rr))
    seat_grip = HOLDER + MACHINE_SHIFT + [0.0, PLACE_DY, CAP_H - NEW_GRIP_DZ + DROP_DZ]
    R.append(_w(2.6, seat_grip + [0, 0, 0.040], FINGER_CLOSED, "carry it over the holder", rr))
    R.append(_w(1.2, seat_grip, FINGER_CLOSED, "lower it over the holder", rr))
    R.append(_w(0.8, seat_grip, RELEASE_FINGER, "let go: it drops into the holder", rr))
    R.append(_w(0.8, seat_grip + [0, 0, 0.040], RELEASE_FINGER, "rise clear", rr))
    button = np.array([MCX + BUTTON_DX, MCY + BUTTON_DY, TOP + M_H + 0.006])
    R.append(_w(1.2, button + [0, 0, 0.040], FINGER_CLOSED, "over the brew button", rr))
    R.append(_w(0.8, button + [0, 0, 0.004], FINGER_CLOSED, "press the brew button", rr))
    R.append(_w(0.6, button + [0, 0, 0.004], FINGER_CLOSED, "hold it a moment", rr))
    R.append(_w(0.8, button + [0, 0, 0.050], FINGER_CLOSED, "let it up", rr))
    R += _joint_path("right", _good_ik("right", button + [0, 0, 0.050], rr), np.array(READY, float), "back home",
                     FINGER_CLOSED)
    tr = sum(w.duration for w in R)
    tl = sum(w.duration for w in L)
    end = max(tr, tl) + 4.0
    R.append(_w(end - tr, R[-1].pos, FINGER_CLOSED, "hold still: the coffee runs into the cup", R[-1].yaw))
    L.append(_w(end - tl, L[-1].pos, TANK_FINGER, "hold still", L[-1].yaw))
    return {"left": L, "right": R}
