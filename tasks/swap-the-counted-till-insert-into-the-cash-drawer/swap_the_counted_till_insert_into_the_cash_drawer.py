"""A shop counter at shift change. A steel POS cash drawer (411 x 415 x 110 mm, the common 16 x 16 inch size) stands on
the counter with a POS tablet on top of it, its drawer sprung open toward the arms as the till leaves it after the POS
fires the release. In the drawer sits the outgoing till insert: a black ABS tray, 361 x 306 x 60 mm, with five bill
bays under black hold-down clips at the back and five coin cups at the front, about 1.5 kg with its coins and notes.
On a grey rubber mat to the right of the drawer lies the counted insert for the next shift, the same model, coins
toward the arms; the mat to the left of the drawer is empty.

Task (the instruction gives the goal only): take the old insert out and leave it on the empty left mat, put the
counted insert into the drawer, coins at the front, without spilling coins (no insert tipped past 25 degrees or
dropped), shut the drawer so its catch holds it, and take both arms back to where they started.

Why two hands: the loaded insert is 361 mm wide and 1.5 kg and has no handle. A YAM pinch on one of its 4 mm wall tops
holds only about 10 N before the V-jaw squeezes the wall out, so one hand can't take its 15 N; held off its middle it
also tips (about 2 N m about the grip against the ~0.2-0.4 N m a pinch resists). No one arm reaches both mats (each is
39 cm off the centre line). A cashier lifts it with both hands too.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_the_counted_till_insert_into_the_cash_drawer.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation, slider  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the till insert (APG Vasario 5 bill / 5 coin class): body frame x toward its back (bills), y left, z up from the
#    underside of its two runners; the coins are at its front (-x) ------------------------------------------------------
TL, TW, TH = 0.306, 0.361, 0.060  # depth (x), width (y), height (z) at the rim
RUN_H = 0.008  # its two runners: the under-till storage gap
FLOOR_T, WALL_T = 0.0025, 0.0025
BEAD_T, BEAD_H = 0.004, 0.008  # the internal walls' top edge
DRAFT = math.radians(2.5)  # the outer walls lean in toward the floor
CW_X = -TL / 2 + 0.120  # the cross wall between the coin row (front, 120 mm) and the bill row (back)
DIV_Y = (-0.108, -0.036, 0.036, 0.108)  # the dividers between the five cups / bays
TILL_SHELL = 0.83  # kg: the ABS tray with its steel hold-down bar and clips
COINS = 0.65  # kg of coins in the five cups (a usual $40 float: rolls of quarters, dimes, nickels and pennies)
NOTES = 0.05  # kg of notes under the clips
TILL_MASS = TILL_SHELL + COINS + NOTES
# -- the cash drawer: cabinet fixed on the counter, the drawer (steel pan + front) on roller slides -------------------
CAB_W, CAB_D, CAB_H, FEET = 0.411, 0.415, 0.110, 0.013
TRAVEL = 0.345  # how far the drawer stands out when sprung open
FACE_T, FACE_W, FACE_Z0, FACE_Z1 = 0.015, 0.405, 0.014, 0.105  # the drawer front: thickness, width, bottom, top
PAN_GAP = 0.005  # the insert's play in the pan, each side, at its rim
PAN_IN_L, PAN_IN_W = TL + 2 * PAN_GAP, TW + 2 * PAN_GAP
PAN_WALL = 0.002
PAN_FLOOR = 0.026  # the pan floor's top above the counter
PAN_TOP = PAN_FLOOR + TH  # the pan's side walls: flush with a seated insert's rim
DRAWER_MASS = 1.8
SPRING_OPEN, SPRING_SHUT = 6.0, 12.0  # N the ejection spring pushes with, drawer fully out / pushed home
SPRING_K = (SPRING_SHUT - SPRING_OPEN) / TRAVEL
SPRING_T = -SPRING_OPEN / SPRING_K  # its target (m from the open position, toward open)
CATCH = 0.002  # the catch holds the drawer this far short of fully home
HOME_AT = 0.001  # pushed within this of fully home, the catch engages
# -- layout (world, m; z above the counter) ------------------------------------------------------------------------
XC = 0.22  # the inserts' cross walls, in the drawer and on the mats
TILL_X = XC - CW_X  # the inserts' centres
PAN_X0 = TILL_X - TL / 2 - PAN_GAP  # the pan's inside front face, drawer open
X_FRONT = PAN_X0 + TRAVEL  # the cabinet's front edge
MAT_Y = 0.39  # the mats' centres: left (empty) +, right (counted insert) -
MAT_W, MAT_L, MAT_T = 0.37, 0.36, 0.003
# -- scoring ---------------------------------------------------------------------------------------------------------
TIP_DEG = 25.0  # an insert tipped further spills coins out of their cups
DROP_SPEED = 0.6  # m/s: an insert landing this fast fell 2 cm or more
SEAT_TOL = 0.003  # seated: each corner of its underside within this of the pan floor
YAW_TOL = 10.0  # deg: coins at the front
ON_MAT_TILT = 5.0
HOME_WITHIN = 0.05
STILL_MM = 0.5
SUCCESS_HOLD_S = 1.0

ABS_BLACK, CLIP_BLACK = (0.07, 0.07, 0.08), (0.03, 0.03, 0.035)
STEEL_BAR, NOTE_GREEN, NOTE_EDGE = (0.70, 0.71, 0.72), (0.55, 0.68, 0.52), (0.40, 0.52, 0.40)
SLIP_WHITE, SLIP_INK = (0.96, 0.96, 0.94), (0.35, 0.35, 0.40)
SILVER, COPPER = (0.72, 0.73, 0.75), (0.72, 0.42, 0.25)
CAB_BLACK, PAN_GREY, RAIL = (0.10, 0.10, 0.11), (0.20, 0.20, 0.21), (0.82, 0.82, 0.80)
MAT_GREY, MAT_EDGE = (0.42, 0.44, 0.47), (0.33, 0.35, 0.38)
TABLET, SCREEN, SCREEN_UI = (0.14, 0.14, 0.15), (0.06, 0.09, 0.13), (0.20, 0.45, 0.70)


def _q(axis, ang):
    s = math.sin(ang / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2))


def _till_shapes(counted: bool):
    """The insert, from its parts: runners, floor, drafted outer walls with a rim, the cross wall and four dividers with
    a beaded top edge; coins, notes, clips and the steel hold-down bar are drawn on (they don't collide)."""
    h = TH - RUN_H  # the walls' height above the runners
    lean = h * math.tan(DRAFT)
    s = [BoxShape((0.0, sy * 0.150, RUN_H / 2), (TL - 0.03, 0.006, RUN_H), ABS_BLACK) for sy in (-1, 1)]
    s.append(BoxShape((0.0, 0.0, RUN_H + FLOOR_T / 2), (TL - 2 * lean, TW - 2 * lean, FLOOR_T), ABS_BLACK))
    zc = RUN_H + h / 2
    for sy in (-1, 1):  # the end walls, tops leaning out
        s.append(BoxShape((0.0, sy * (TW / 2 - lean / 2 - WALL_T / 2), zc), (TL - 0.004, WALL_T, h), ABS_BLACK,
                          quat_xyzw=_q((1, 0, 0), -sy * DRAFT)))
    for sx in (-1, 1):  # front and back walls
        s.append(BoxShape((sx * (TL / 2 - lean / 2 - WALL_T / 2), 0.0, zc), (WALL_T, TW - 0.004, h), ABS_BLACK,
                          quat_xyzw=_q((0, 1, 0), sx * DRAFT)))
    rim = 0.004
    for sy in (-1, 1):
        s.append(BoxShape((0.0, sy * (TW / 2 - rim / 2), TH - 0.0015), (TL, rim, 0.003), ABS_BLACK))
    for sx in (-1, 1):
        s.append(BoxShape((sx * (TL / 2 - rim / 2), 0.0, TH - 0.0015), (rim, TW, 0.003), ABS_BLACK))
    zin0 = RUN_H + FLOOR_T
    wall_h = TH - BEAD_H - zin0
    inner_l, inner_w = TL - 2 * WALL_T - 2 * lean, TW - 2 * WALL_T - 2 * lean
    s.append(BoxShape((CW_X, 0.0, zin0 + wall_h / 2), (WALL_T, inner_w, wall_h), ABS_BLACK))
    s.append(BoxShape((CW_X, 0.0, TH - BEAD_H / 2), (BEAD_T, inner_w, BEAD_H), ABS_BLACK))
    for y in DIV_Y:
        s.append(BoxShape((0.0, y, zin0 + wall_h / 2), (inner_l, WALL_T, wall_h), ABS_BLACK))
        s.append(BoxShape((0.0, y, TH - BEAD_H / 2), (inner_l, BEAD_T, BEAD_H), ABS_BLACK))
    # drawn on (no mass of their own; the insert's mass and centre of mass are set for the loaded insert): the coins
    # (quarters, dimes, nickels, pennies, and a spare cup of quarters) and the notes under their clips. The counted
    # insert is made up neatly: every coin cup full to the same level, every bay an even stack of notes, and a white
    # count slip under one clip. The used one is uneven: stacks of all thicknesses, one bay nearly empty, coin
    # cups at different levels, one almost empty.
    cups = [-0.144, -0.072, 0.0, 0.072, 0.144]
    coin = [(0.0121, 0.0018, SILVER), (0.0090, 0.0014, SILVER), (0.0106, 0.0020, SILVER), (0.0095, 0.0015, COPPER),
            (0.0121, 0.0018, SILVER)]
    xf = -TL / 2 + 0.060  # the middle of the coin cups' floors
    if counted:
        piles = [(dx, dy) for dx in (-0.030, 0.0, 0.030) for dy in (-0.0135, 0.0135)]
        for y, (r, t, c) in zip(cups, coin):
            n = round(0.024 / t)  # full and level: every cup's piles stand 24 mm
            for dx, dy in piles:
                s.append(CylShape((xf + dx, y + dy, zin0 + n * t / 2), r, n * t, c, collide=False))
        notes = [0.008] * 5
    else:
        fills = [(9, 7, 5, 4), (3, 2, 0, 0), (12, 10, 8, 6), (1, 0, 0, 0), (6, 5, 3, 2)]  # coins per pile, cup by cup
        for y, (r, t, c), fill in zip(cups, coin, fills):
            for (dx, dy), n in zip(((-0.010, -0.012), (0.012, 0.011), (-0.028, 0.012), (0.030, -0.009)), fill):
                if n:
                    s.append(CylShape((xf + dx, y + dy, zin0 + n * t / 2), r, n * t, c, collide=False))
        notes = [0.007, 0.002, 0.010, 0.0006, 0.004]
    # the used insert's notes lie untidily: shifted along their bays, two slightly askew, the nearly empty bay's last
    # note folded and pushed back so most of the bay's floor shows
    shift = [0.0] * 5 if counted else [0.004, -0.012, 0.0, 0.050, -0.006]
    skew = [0.0] * 5 if counted else [0.0, 0.02, 0.0, 0.0, 0.0]
    length = [0.150] * 5 if counted else [0.150, 0.150, 0.150, 0.078, 0.150]  # the near-empty bay: one folded note
    bx = CW_X + 0.012 + 0.080
    for y, h, dx, a, ln in zip(cups, notes, shift, skew, length):
        s.append(BoxShape((bx + dx, y, zin0 + h / 2), (ln, 0.062, h), NOTE_GREEN, yaw=a, collide=False))
        # the stack's front face, darker, so its thickness shows
        s.append(BoxShape((bx + dx - ln / 2 + 0.001, y, zin0 + h / 2), (0.002, 0.062, h), NOTE_EDGE, yaw=a,
                          collide=False))
        s.append(BoxShape((bx + 0.010, y, zin0 + h + 0.0015), (0.120, 0.022, 0.003), CLIP_BLACK, collide=False))
        s.append(BoxShape((TL / 2 - 0.020, y, zin0 + 0.016), (0.020, 0.026, 0.024), CLIP_BLACK, collide=False))
    if counted:  # the count slip, tucked under the clip of the bay nearest the coins' spare cup
        ys, hs = cups[4], notes[4]
        s.append(BoxShape((bx - 0.015, ys, zin0 + hs + 0.0004), (0.100, 0.066, 0.0006), SLIP_WHITE, collide=False))
        for k in range(4):  # its printed lines
            s.append(BoxShape((bx - 0.050 + 0.016 * k, ys + 0.021, zin0 + hs + 0.0008), (0.002, 0.018, 0.0002),
                              SLIP_INK, collide=False))
    s.append(BoxShape((TL / 2 - 0.010, 0.0, TH + 0.0008), (0.022, TW - 0.040, 0.0016), STEEL_BAR, collide=False))
    for sy in (-1, 1):  # the rear tabs
        s.append(BoxShape((TL / 2 + 0.004, sy * (TW / 2 - 0.035), TH - 0.004), (0.008, 0.016, 0.006), ABS_BLACK,
                          collide=False))
    return s


def _till_com():
    """The insert's centre of mass in its body frame: the tray's walls and floor, the steel bar at the back, the coins
    low in the front cups, the notes at the back."""
    parts = [(0.70, (0.0, 0.0, 0.028)), (TILL_SHELL - 0.70, (TL / 2 - 0.040, 0.0, 0.040)),
             (COINS, (-TL / 2 + 0.050, 0.0, RUN_H + FLOOR_T + 0.008)), (NOTES, (CW_X + 0.090, 0.0, 0.016))]
    m = sum(p[0] for p in parts)
    return tuple(float(v) for v in sum(np.array(c) * w for w, c in parts) / m)


TILL_COM = _till_com()


def _till(name, y, z) -> Body:
    return Body.build(name, pos=(TILL_X, y, TOP + z), mass=TILL_MASS, shapes=_till_shapes(name == "new_till"),
                      color=ABS_BLACK, friction=(1.0, 0.9), com=TILL_COM, solver_iterations=24, damping=(0.0, 0.05))


def _cabinet() -> Body:
    """The drawer's steel cabinet with the POS tablet on its stand on top. Origin: the middle of its front edge, on the
    counter."""
    d, w, h = CAB_D, CAB_W, CAB_H
    s = []
    for fx in (0.03, d - 0.03):
        for fy in (-w / 2 + 0.03, w / 2 - 0.03):
            s.append(CylShape((fx, fy, FEET / 2), 0.010, FEET, CLIP_BLACK))
    s += [BoxShape((d / 2, 0.0, FEET + 0.001), (d, w, 0.002), CAB_BLACK),  # base
          BoxShape((d / 2, 0.0, h - 0.001), (d, w, 0.002), CAB_BLACK),  # top
          BoxShape((d - 0.001, 0.0, (FEET + h) / 2), (0.002, w, h - FEET), CAB_BLACK)]  # back
    for sy in (-1, 1):
        s.append(BoxShape((d / 2, sy * (w / 2 - 0.001), (FEET + h) / 2), (d, 0.002, h - FEET), CAB_BLACK))
        s.append(BoxShape((0.010, sy * (w / 2 - 0.003), (FEET + h) / 2), (0.020, 0.004, h - FEET), CAB_BLACK))
    s.append(BoxShape((0.010, 0.0, h - 0.003), (0.020, w, 0.006), CAB_BLACK))  # the lip above the opening
    # the POS tablet on its stand, on the cabinet's top toward the back
    s += [BoxShape((0.28, 0.0, h + 0.004), (0.14, 0.14, 0.008), TABLET),
          BoxShape((0.30, 0.0, h + 0.060), (0.025, 0.040, 0.105), TABLET),
          BoxShape((0.285, 0.0, h + 0.150), (0.012, 0.255, 0.170), TABLET, quat_xyzw=_q((0, 1, 0), math.radians(-20))),
          BoxShape((0.278, 0.0, h + 0.148), (0.002, 0.230, 0.150), SCREEN, collide=False,
                   quat_xyzw=_q((0, 1, 0), math.radians(-20)))]
    for k in range(3):
        s.append(BoxShape((0.2765 + 0.004 * (k - 1) * math.sin(math.radians(20)), -0.06 + 0.06 * k,
                           h + 0.148 + 0.011 * (k - 1)), (0.0006, 0.045, 0.030), SCREEN_UI, collide=False,
                          quat_xyzw=_q((0, 1, 0), math.radians(-20))))
    return Body.build("cash_drawer", pos=(X_FRONT, 0.0, TOP), mass=8.0, kinematic=True, color=CAB_BLACK, shapes=s,
                      friction=(0.5, 0.4))


def _drawer() -> Body:
    """The drawer: a steel pan on roller slides behind its front panel (with the lock). Origin: the middle of the pan's
    floor (its top face)."""
    L, W = PAN_IN_L, PAN_IN_W
    t = PAN_WALL
    wall_h = PAN_TOP - PAN_FLOOR
    s = [BoxShape((0.0, 0.0, -0.001), (L + 2 * t, W + 2 * t, 0.002), PAN_GREY)]
    for sy in (-1, 1):
        s.append(BoxShape((0.0, sy * (W / 2 + t / 2), wall_h / 2), (L + 2 * t, t, wall_h), PAN_GREY))
        s.append(BoxShape((0.010, sy * (W / 2 + t + 0.006), 0.012), (L, 0.010, 0.016), RAIL, collide=False))
    s.append(BoxShape((L / 2 + t / 2, 0.0, wall_h / 2), (t, W + 2 * t, wall_h), PAN_GREY))
    z0, z1 = FACE_Z0 - PAN_FLOOR, FACE_Z1 - PAN_FLOOR
    s.append(BoxShape((-L / 2 - FACE_T / 2, 0.0, (z0 + z1) / 2), (FACE_T, FACE_W, z1 - z0), CAB_BLACK))
    s.append(CylShape((-L / 2 - FACE_T - 0.002, 0.0, (z0 + z1) / 2 + 0.006), 0.0075, 0.004, RAIL, collide=False))
    s.append(BoxShape((-L / 2 - FACE_T - 0.0005, 0.0, (z0 + z1) / 2 + 0.006), (0.0012, 0.0015, 0.008), CLIP_BLACK,
                      collide=False))
    for sy in (-1, 1):  # the two media slots in the front
        s.append(BoxShape((-L / 2 - FACE_T - 0.0005, sy * 0.10, z1 - 0.025), (0.001, 0.120, 0.003), CLIP_BLACK,
                          collide=False))
    return Body.build("drawer", pos=(PAN_X0 + L / 2, 0.0, TOP + PAN_FLOOR), mass=DRAWER_MASS, shapes=s,
                      color=PAN_GREY, friction=(0.35, 0.30), slick=True, solver_iterations=24, damping=(0.0, 0.0))


def _mat(name, y) -> Body:
    s = [BoxShape((0.0, 0.0, MAT_T / 2), (MAT_L, MAT_W, MAT_T), MAT_GREY)]
    for sx in (-1, 1):
        s.append(BoxShape((sx * (MAT_L / 2 - 0.006), 0.0, MAT_T + 0.0002), (0.004, MAT_W - 0.008, 0.0004), MAT_EDGE,
                          collide=False))
    return Body.build(name, pos=(TILL_X, y, TOP), mass=0.5, kinematic=True, color=MAT_GREY, shapes=s,
                      friction=(0.9, 0.8))


OBJECTS = [_cabinet(), _drawer(), _mat("left_mat", MAT_Y), _mat("right_mat", -MAT_Y),
           _till("old_till", 0.0, PAN_FLOOR + 0.0008), _till("new_till", -MAT_Y, MAT_T + 0.0008)]

# the drawer's roller slide: 0 sprung open, TRAVEL pushed home; the ejection spring pushes it open with 6 N fully out,
# 12 N pushed home
slider("slide", "cash_drawer", "drawer", anchor=(PAN_X0 + PAN_IN_L / 2, 0.0, TOP + PAN_FLOOR), axis="X",
       limits=(0.0, TRAVEL), stiffness=SPRING_K, damping=2.0, target=SPRING_T, max_force=60.0)

_J: dict = {}
_author_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    if spec.name == "drawer":
        _J["env"] = prim_path.rsplit("/", 1)[0]
        _J.pop("lo", None)


bodykit._author_sliders = _author_sliders


def _catch(lo: float) -> None:
    """Move the slide's lower stop (m from the open position): the catch holding the drawer near home."""
    if "lo" not in _J:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS["slide"])
        _J["lo"] = UsdPhysics.PrismaticJoint(prim).GetLowerLimitAttr()
    _J["lo"].Set(float(lo))


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"latched": False, "spill": "", "s": {}, "tcps": None, "steps": 0, "prev": {}, "hist": [], "still": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TILL_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    if "env" in _J:
        _catch(0.0)


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    d = base.scene[name].data
    p = as_numpy(d.root_link_pose_w)[0]
    v = as_numpy(d.root_com_vel_w)[0] if hasattr(d, "root_com_vel_w") else as_numpy(d.root_vel_w)[0]
    return p[:3] - origin, rotation(p[3:7]), v[:3]


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


CORNERS = [np.array([sx * (TL / 2 - 0.015), sy * (0.150), 0.0]) for sx in (-1, 1) for sy in (-1, 1)]


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    pd, rd, _vd = _pose(base, origin, "drawer")
    u = float(pd[0] - (PAN_X0 + PAN_IN_L / 2))  # how far the drawer has been pushed in from fully open
    if not _EP["latched"] and u >= TRAVEL - HOME_AT:
        _EP["latched"] = True
        _catch(TRAVEL - CATCH)  # the catch drops in: the drawer can't spring back out
    tills = {}
    for name in ("old_till", "new_till"):
        p, r, v = _pose(base, origin, name)
        tilt = _tilt(r)
        if not _EP["spill"] and tilt > TIP_DEG:
            _EP["spill"] = f"{name} tipped {tilt:.0f} deg: coins out of their cups"
        prev = _EP["prev"].get(name)
        if not _EP["spill"] and prev is not None and prev > DROP_SPEED and v[2] > -0.1:
            _EP["spill"] = f"{name} dropped: landed at {prev:.2f} m/s"
        _EP["prev"][name] = float(-v[2])
        tills[name] = (p, r, tilt)
    # the old insert on the left mat: flat on it, inside its outline
    p, r, tilt = tills["old_till"]
    lo = np.array([TILL_X - MAT_L / 2, MAT_Y - MAT_W / 2])
    hi = np.array([TILL_X + MAT_L / 2, MAT_Y + MAT_W / 2])
    feet = [p + r @ c for c in CORNERS]
    on_mat = (all(lo[0] <= f[0] <= hi[0] and lo[1] <= f[1] <= hi[1] for f in feet)
              and all(abs(f[2] - (TOP + MAT_T)) <= 0.005 for f in feet) and tilt <= ON_MAT_TILT)
    # the new insert seated in the drawer: each corner of its underside on the pan floor, square, coins at the front
    p, r, tilt = tills["new_till"]
    rel = [rd.T @ (p + r @ c - pd) for c in CORNERS]
    yaw = math.degrees(math.atan2(float((rd.T @ r)[1, 0]), float((rd.T @ r)[0, 0])))
    in_pan = all(abs(q[0]) <= PAN_IN_L / 2 and abs(q[1]) <= PAN_IN_W / 2 for q in rel)
    seated = in_pan and all(abs(float(q[2])) <= SEAT_TOL for q in rel) and abs(yaw) <= YAW_TOL
    shut = _EP["latched"] and u >= TRAVEL - CATCH - 0.0015
    # still: the drawer and both inserts moved under 0.5 mm over the last second (a slide resting on its stop reads a
    # small speed in PhysX, so displacement is used, not velocity)
    _EP["hist"].append(np.concatenate([[u], tills["old_till"][0], tills["new_till"][0]]))
    _EP["hist"] = _EP["hist"][-21:]
    h = np.array(_EP["hist"])
    _EP["still"] = len(h) == 21 and float(np.abs(h - h[-1]).max()) < STILL_MM / 1000.0
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"on_mat": on_mat, "seated": seated, "shut": shut, "home": _arms_home(), "u": u, "yaw": yaw,
                "seat_mm": [round(float(q[2]) * 1000, 1) for q in rel]}
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        tl, tr = _EP["tcps"]
        mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000).astype(int).tolist()  # noqa: E731
        po, ro, to = tills["old_till"]
        pn, rn, tn = tills["new_till"]
        print(f"[till] t={_EP['steps'] / 20:5.1f}s u={u * 1000:6.1f} latched={_EP['latched']} old={mm(po)} tilt={to:4.1f}"
              f" new={mm(pn)} tilt={tn:4.1f} on_mat={on_mat} seated={seated} seat={_EP['s']['seat_mm']} yaw={yaw:.1f}"
              f" spill='{_EP['spill']}' L={mm(tl)} R={mm(tr)} home={_EP['s']['home']} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if either insert was ever tipped past 25 degrees or dropped (landing faster than 0.6 m/s, a fall of 2 cm or
    more): coins spilled (latched). Otherwise, read at the end: 1.0 the old insert flat on the left mat, the counted one
    seated in the drawer coins at the front, the drawer shut on its catch, and both arms back where they started; 0.75
    the same without the arms home; 0.5 the old insert on the mat and the counted one seated; 0.25 the old insert on
    the left mat; else 0.0."""
    s = _EP["s"]
    if not s or _EP["spill"]:
        return 0.0
    if not s["on_mat"]:
        return 0.0
    if not s["seated"]:
        return 0.25
    if not s["shut"]:
        return 0.5
    return 1.0 if s["home"] else 0.75


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
    """Full score with the drawer and both inserts still (each moved under 0.5 mm over the last second): the trial ends
    there."""
    return score(objects) >= 1.0 and _EP["still"]


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands pinch the insert's cross wall (between the coin row and the bill row, next to its centre of mass), fingers
# closing along x, so the carry along y runs across the grips, never along their closing axis. The hand that crosses
# the centre line pinches 15 cm off the insert's middle with its grip leaned 40 deg toward the mat so it can reach; the
# outboard hand pinches 6 cm off the middle (taking 71% of the weight), starts straight down and leans over to 30 deg
# as it carries outward (its pads turn on the wall's face). They lift the old insert straight up until it clears the
# pan's walls, carry it level to the left mat and set it down. Then they take the counted insert the same way from the
# right mat (the hands' roles swapped), carry it over the drawer, lower it in (its drafted walls find the pan) and let
# go. Both hands push the drawer front home until the catch holds it, and go home.

PINCH_UP = 0.004  # the TCP above the rim when pinching the wall's beaded top, straight down
PINCH_UP_LEAN = -0.002  # ... and leaned 45 deg, where the tips reach less far down
PRE = -0.025  # fingers part-open before pinching
CROSS_DY, OUT_DY = 0.150, 0.060  # grip points off the insert's middle, on its cross wall (the crossing hand takes 29%)
LEAN = 40.0
OUT_LEANS = (0.0, 5.0, 15.0, 25.0, 30.0)
STRETCH_UP = 0.012  # the crossing hand, fully stretched at a mat, sags more: aim it this much higher there  # the outboard hand's lean: lifting, then at each quarter of the carry out
LIFT_CLEAR = 0.016  # the underside carried this far above the pan's walls
SAG = 0.015  # the loaded arms sag this much
PUSH_Z, PUSH_Y = 0.055, 0.10  # both hands push the drawer front, 10 cm each side of its middle
AIM_OUT = 0.015  # the stretched crossing hand falls short: carry the old insert this much further out
PUSH_PITCH = 25.0  # the pushing grips lean forward so their fingertips lead
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5, 0.5), (-1.4, 0.0, 1.4),
                                  (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > math.radians(25.0):
                break
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


def grip_rot(arm: str, lean_toward_left: float) -> np.ndarray:
    """A grip closing along x, leaned ``lean_toward_left`` deg about x (positive: the fingers point toward +y)."""
    yaw = math.pi / 2 if arm == "right" else -math.pi / 2
    return _rx(math.radians(lean_toward_left)) @ grip_rotation(yaw)


def pinch_up(lean: float) -> float:
    return PINCH_UP + (PINCH_UP_LEAN - PINCH_UP) * min(abs(lean), 45.0) / 45.0


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, n=1):
        p0, _f, rot = cur[arm]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def both(t, dpos, finger, label, n=1):
        for arm in ("left", "right"):
            go(arm, t, cur[arm][0] + np.asarray(dpos, float), finger, label, n)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        last = plan[arm][-1]
        plan[arm].append(Waypoint(t, last.pos, last.finger, label, last.yaw))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def joint_to(arm, pos, rot, finger, label):
        p0, _f, r0 = cur[arm]
        q0 = np.array(READY, float) if p0 is None else _good_ik(arm, p0, r0)
        plan[arm].extend(_joint_path(arm, q0, _good_ik(arm, pos, rot), label, finger))
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def to_home(arm, label="back to where it started"):
        p, f, rot = cur[arm]
        plan[arm].extend(_joint_path(arm, _good_ik(arm, p, rot), np.array(READY, float), label, f))
        cur[arm] = (None, f, None)

    cur["left"] = (None, FINGER_OPEN, None)
    cur["right"] = (None, FINGER_OPEN, None)

    def carry(cross_arm, out_arm, y0, z_floor, sign, z_lift, leans, label):
        """Both hands carry the insert (middle at y0, underside z_floor) along y by sign * MAT_Y at height z_lift,
        the crossing hand leaned LEAN toward the move, the outboard one through ``leans``."""
        for k in range(1, 5):
            dy = sign * (MAT_Y + AIM_OUT) * k / 4
            for arm, off, lean in ((cross_arm, -sign * CROSS_DY, sign * LEAN), (out_arm, sign * OUT_DY, leans[k])):
                rot = grip_rot(arm, lean)
                p = np.array([XC, y0 + off + dy, z_lift + pinch_up(lean)])
                plan[arm].append(Waypoint(1.0, tuple(float(v) for v in p), FINGER_CLOSED, label, rot))
                cur[arm] = (p, FINGER_CLOSED, rot)

    up = np.array([0.0, 0.0, 0.06])
    z_lift = TOP + PAN_TOP + LIFT_CLEAR + TH + SAG  # the rim's height while carried (before the TCP's offset)
    # -- the old insert out of the drawer, onto the left mat -----------------------------------------------------------
    rim = TOP + PAN_FLOOR + TH
    rc, ro = grip_rot("right", LEAN), grip_rot("left", OUT_LEANS[0])
    c = np.array([XC, -CROSS_DY, rim + pinch_up(LEAN)])
    o = np.array([XC, OUT_DY, rim + pinch_up(OUT_LEANS[0])])
    joint_to("right", c + up, rc, PRE, "over the old insert's cross wall, leaned toward the left")
    joint_to("left", o + up, ro, PRE, "over its cross wall")
    sync()
    go("right", 1.2, c, PRE, "down astride the cross wall")
    go("left", 1.2, o, PRE, "down astride the cross wall")
    both(0.8, (0, 0, 0), FINGER_CLOSED, "pinch")
    both(0.4, (0, 0, 0), FINGER_CLOSED, "pinch")
    both(2.5, (0, 0, z_lift - rim), FINGER_CLOSED, "lift it straight up out of the pan", n=3)
    carry("right", "left", 0.0, PAN_FLOOR, 1.0, z_lift, OUT_LEANS, "carry it level to the left mat")
    rim_mat = TOP + MAT_T + TH
    both(2.0, (0, 0, (rim_mat + 0.002) - z_lift), FINGER_CLOSED, "set it down on the mat", n=2)
    both(0.8, (0, 0, -0.002), PRE, "let go")
    both(0.8, up, PRE, "rise")
    sync()
    for arm in ("left", "right"):
        to_home(arm, "back toward the start")
    sync()
    # -- the counted insert from the right mat into the drawer -----------------------------------------------------------
    back = tuple(reversed(OUT_LEANS))
    rc, ro = grip_rot("left", -LEAN), grip_rot("right", -back[0])
    c = np.array([XC, -MAT_Y + CROSS_DY, rim_mat + pinch_up(LEAN)])
    o = np.array([XC, -MAT_Y - OUT_DY, rim_mat + pinch_up(back[0])])
    joint_to("right", o + up, ro, PRE, "over the counted insert's cross wall, leaned toward the right")
    joint_to("left", c + up, rc, PRE, "over its cross wall, leaned toward the right")
    sync()
    go("left", 1.2, c, PRE, "down astride the cross wall")
    go("right", 1.2, o, PRE, "down astride the cross wall")
    both(0.8, (0, 0, 0), FINGER_CLOSED, "pinch")
    both(0.4, (0, 0, 0), FINGER_CLOSED, "pinch")
    go("left", 3.5, cur["left"][0] + [0, 0, z_lift - rim_mat + STRETCH_UP], FINGER_CLOSED, "lift it", n=4)
    go("right", 3.5, cur["right"][0] + [0, 0, z_lift - rim_mat], FINGER_CLOSED, "lift it", n=4)
    # carrying toward +y: the left (crossing the centre line on the way back) leads, leaned -45 deg toward where it came
    # from; the right unleans from -45 to 0 as it comes in
    for k in range(1, 5):
        dy = MAT_Y * k / 4
        for arm, off, lean in (("left", CROSS_DY, -LEAN), ("right", -OUT_DY, -back[k])):
            rot = grip_rot(arm, lean)
            up_k = STRETCH_UP * (1.0 - k / 4) if arm == "left" else 0.0
            p = np.array([XC, -MAT_Y + off + dy, z_lift + pinch_up(lean) + up_k])
            plan[arm].append(Waypoint(1.5, tuple(float(v) for v in p), FINGER_CLOSED, "carry it level over the drawer",
                                      rot))
            cur[arm] = (p, FINGER_CLOSED, rot)
    both(1.5, (0, 0, (TOP + PAN_TOP + 0.006 + TH) - z_lift), FINGER_CLOSED, "lower it to the pan", n=2)
    both(3.0, (0, 0, (PAN_FLOOR - 0.004) - (PAN_TOP + 0.006) - 0.006), FINGER_CLOSED,
         "lower it in: its walls find the pan", n=4)
    both(0.8, (0, 0, 0), PRE, "let go")
    both(0.8, up, PRE, "rise")
    sync()
    # -- push the drawer home with both hands ---------------------------------------------------------------------------
    both(0.8, (0, 0, 0.05), PRE, "rise clear of the drawer")
    face = PAN_X0 - FACE_T  # the drawer front's face, open
    c, s_ = math.cos(math.radians(PUSH_PITCH)), math.sin(math.radians(PUSH_PITCH))
    pitch = np.array([[c, 0.0, -s_], [0.0, 1.0, 0.0], [s_, 0.0, c]])  # about y: the fingers point forward
    for arm, sy in (("right", -1.0), ("left", 1.0)):
        joint_to(arm, np.array([face - 0.026, sy * PUSH_Y, TOP + PUSH_Z + 0.10]), pitch @ grip_rotation(0.0),
                 FINGER_CLOSED, "over the front of the drawer")
    sync()
    both(1.5, (0, 0, -0.10), FINGER_CLOSED, "down in front of it", n=2)
    both(5.0, (TRAVEL + 0.045, 0, 0), FINGER_CLOSED, "push the drawer home: its catch drops in", n=6)
    both(1.0, (0, 0, 0), FINGER_CLOSED, "hold it home")
    both(1.0, (-0.04, 0, 0), FINGER_CLOSED, "let go: the catch holds it")
    both(0.8, (0, 0, 0.06), FINGER_CLOSED, "rise")
    for arm in ("left", "right"):
        to_home(arm)
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
