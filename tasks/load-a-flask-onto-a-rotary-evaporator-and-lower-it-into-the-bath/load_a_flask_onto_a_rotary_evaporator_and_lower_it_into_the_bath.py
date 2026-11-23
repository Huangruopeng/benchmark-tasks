"""A benchtop rotary evaporator (Heidolph Hei-VAP Core type, motor lift, vertical G3 condenser) stands on the lab bench
in front of the arms, its lift up, its EASY CLIP flask clamp open. A 100 mL round-bottom flask (NS 24/40) holding 40 mL
of extract stands in a cork ring beside it.

Task: put the flask on the evaporator and lower it into the water bath, ready to run.

What the physics asks for: the flask's ground socket goes onto the vapour tube's ground cone, which points down toward
the bath at 30 degrees: it slides on only along that axis, held straight. A dry ground joint does not carry the flask
on that slope: let go before the clamp is shut and it slides off the cone and falls into the bath. The EASY CLIP's
clamping lever lies over the joint and closes, with a click, only when its hook can drop behind the flask's collar,
i.e. once the socket is fully home (the maker's procedure). So one hand holds the flask home on the cone while the other
presses the lever down; then the lift's DOWN button, held, takes the flask 95 mm down into the bath (25 mm/s) to the
lift's immersion stop.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _qy(a) -> tuple:
    """(x, y, z, w) of a turn ``a`` (rad) about y."""
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


# -- layout --------------------------------------------------------------------------------------------------------------
DROP = 0.060  # the lab bench under the evaporator is 60 mm below the arms' deck (the station table)
B = TOP - DROP
HX0, HX1, HY0, HY1 = 0.090, 0.800, -0.220, 0.160  # the bench area (a cut-out in the station table)
TILT = math.radians(30.0)  # the vapour tube's axis, below horizontal (the Hei-VAP's tilt range is 20-80 degrees)
A = np.array([math.cos(TILT), 0.0, math.sin(TILT)])  # along the axis, from the flask up toward the drive
N = np.array([-math.sin(TILT), 0.0, math.cos(TILT)])  # the axis's upward normal

# the flask (100 mL round bottom, NS 24/40 socket): origin at the bulb's centre, the neck along +z
BULB_R = 0.032
NECK_R0, NECK_R1 = 0.0125, 0.0150  # the socket's bore and outside
NECK_Z0, NECK_Z1 = 0.026, 0.078
COLLAR_R, COLLAR_Z0 = 0.0170, 0.078  # the bead round the socket's mouth
MOUTH_Z = 0.082
FLASK_MASS = 0.100  # 58 g of glass and 40 mL of extract
GRIP_Z = 0.044  # where the hand holds the neck, just above the bulb

# the seat: where the flask's mouth is when it is home on the cone, lift up; the rotary head's origin
GRIP_SEAT = np.array([0.330, 0.0, TOP + 0.130])  # the hand's grip on the neck of the flask, home, lift up
BULB_SEAT = GRIP_SEAT - GRIP_Z * A
S0 = BULB_SEAT + MOUTH_Z * A  # the mouth, home
CONE_L, CONE_R = 0.040, 0.0070  # the ground cone reaches 40 mm into the socket (collision core of the cone)
TRAVEL = 0.095  # the lift's travel down to its immersion stop
LIFT_SPEED = 0.025  # m/s, the maker's figure

# the heating bath (Hei-VAP: inner 253 mm, outer 291 mm across) under the flask, on the bench
BATH_C = (float(BULB_SEAT[0]), 0.0)
BATH_RI, BATH_RO = 0.1265, 0.1455
BATH_RIM = 0.120  # above the bench
BATH_FLOOR = 0.025
WATER = 0.085  # the water's surface above the bench (3.0 L)

# the cork ring the flask starts in, and the evaporator's control panel, on the arms' deck
CORK_C = (0.320, 0.220)
CORK_RI, CORK_RO, CORK_H = 0.025, 0.045, 0.025
FLASK0 = (CORK_C[0], CORK_C[1], TOP + CORK_H + math.sqrt(BULB_R ** 2 - CORK_RI ** 2))
PANEL_C = (0.175, 0.235)
PANEL_TOP = 0.045
BTN_R, BTN_TRAVEL, BTN_SWITCH = 0.008, 0.0035, 0.002
BTN_Z0 = TOP + PANEL_TOP + 0.008  # a key cap standing 4-12 mm above the panel
BTN_TOP = BTN_Z0 + 0.004
BTN_DOWN = (PANEL_C[0] - 0.030, PANEL_C[1] + 0.022)
BTN_UP = (PANEL_C[0] - 0.030, PANEL_C[1] - 0.022)

# the EASY CLIP: a ring on the vapour tube, its lever hinged on top (axis along y), its hook dropping behind the collar
HINGE = S0 + 0.030 * A + 0.026 * N
LEVER_L = 0.036
LEVER_OPEN = 35.0  # degrees, as left open
LEVER_SPRING = 0.004  # N m per degree: about 5 N on the thumb pad to shut it
CLICK_DEG = 3.0  # within this of shut, with the flask home: it clicks shut

# -- the rules -------------------------------------------------------------------------------------------------------
CATCH_LAT, CATCH_DEG = 0.0055, 10.0  # the socket goes onto the cone this close to the axis and this straight
HOME_S = 0.004  # home on the cone: within 4 mm of fully on
OFF_S = 0.046  # this far off: off the cone
LOWER_MM = 4.0  # the lift at its stop: within 4 mm
DROP_Z = TOP + 0.035  # the bulb's centre below this, off the cone and out of the cork ring: dropped
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

GLASS, GLASS_D = (0.80, 0.86, 0.90), (0.62, 0.70, 0.76)
EXTRACT = (0.86, 0.72, 0.38)
FROST = (0.90, 0.92, 0.93)
HEID, HEID_D = (0.93, 0.93, 0.92), (0.20, 0.21, 0.23)
STEEL, WATER_C = (0.70, 0.72, 0.74), (0.45, 0.62, 0.80)
EPOXY, CORK = (0.12, 0.12, 0.13), (0.70, 0.52, 0.33)
CLIP_C, RED = (0.15, 0.30, 0.65), (0.75, 0.15, 0.12)


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _ring(cx, cy, r0, r1, z0, z1, color, n=24, collide=True) -> list:
    out = []
    for k in range(n):
        th = 2 * math.pi * (k + 0.5) / n
        w = 2 * r1 * math.tan(math.pi / n) + 0.001
        rm = (r0 + r1) / 2
        out.append(BoxShape((cx + rm * math.cos(th), cy + rm * math.sin(th), (z0 + z1) / 2), (r1 - r0, w, z1 - z0),
                            color, yaw=th, collide=collide))
    return out


def _tilted_box(centre, along, cross_y, cross_n, color, collide=True) -> BoxShape:
    """A box whose long side lies along the vapour tube's axis (A), ``cross_y`` wide and ``cross_n`` thick."""
    return BoxShape(tuple(float(v) for v in centre), (along, cross_y, cross_n), color, collide=collide,
                    quat_xyzw=_qy(-TILT))


def _tilted_cyl(p0, p1, r, color, collide=True, n=12) -> list:
    """A tube round the axis from p0 to p1 (both on a line along A), radius r: ``n`` flat panels (drawn only) round a
    square core that collides when ``collide``."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    c, length = (p0 + p1) / 2, float(np.linalg.norm(p1 - p0))
    out = []
    t = max(0.0015, 0.2 * r)
    for k in range(n):
        th = 2 * math.pi * (k + 0.5) / n
        rm = (r - t / 2) * math.cos(math.pi / n)
        off = rm * (math.cos(th) * np.array([0.0, 1.0, 0.0]) + math.sin(th) * N)
        w = 2 * r * math.tan(math.pi / n)
        q = bodykit._quat_from_matrix(_ry(-TILT) @ _rx(th))
        out.append(BoxShape(tuple((c + off).tolist()), (length, w, t), color, collide=False, quat_xyzw=q))
    out.append(BoxShape(tuple(c.tolist()), (length, r * 1.40, r * 1.40), color, collide=collide, quat_xyzw=_qy(-TILT)))
    return out


# -- the station table, cut out where the lab bench is ---------------------------------------------------------------

def _table_pieces(station) -> list:
    cx, cy, cz = station.TABLE_CENTER
    sx, sy, sz = station.TABLE_SIZE
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    out = []

    def slab(name, xa, xb, ya, yb):
        out.append((name, (xb - xa, yb - ya, sz), ((xa + xb) / 2, (ya + yb) / 2, cz)))

    slab("table", x0, HX0, y0, y1)
    slab("table_back", HX1, x1, y0, y1)
    slab("table_right", HX0, HX1, y0, HY0)
    slab("table_left", HX0, HX1, HY1, y1)
    return out


def _cut_table(scene) -> None:
    if getattr(scene, "_rv_cut", False):
        return
    plain = scene._station_room
    from yam_arena import station

    def room():
        out = []
        for obj in plain():
            if getattr(obj, "name", None) == "table":
                out += [scene._box(n, size, pos, scene.STATION_TABLE_COLOR, cls=scene._Table)
                        for n, size, pos in _table_pieces(station)]
            else:
                out.append(obj)
        return out

    scene._station_room = room
    scene._rv_cut = True


_plain_scene_patch = bodykit._TARGETS["yam_arena.scene"]


def _scene_patch(module) -> None:
    _plain_scene_patch(module)
    _cut_table(module)


bodykit._TARGETS["yam_arena.scene"] = _scene_patch
if "yam_arena.scene" in sys.modules:
    _cut_table(sys.modules["yam_arena.scene"])


# -- bodies ------------------------------------------------------------------------------------------------------------

def _bench() -> Body:
    """The lab bench (black epoxy top) in the table's cut-out, the evaporator's base unit and lift column, the heating
    bath with its water, the cork ring and the control panel on the arms' deck. Fixed. Origin at (0, 0, B)."""
    sh = []
    z = lambda v: v - B  # noqa: E731  (world height -> body frame)
    sh.append(_box((HX0 + 0.001, HY0 + 0.001, -0.030), (HX1 - 0.001, HY1 - 0.001, 0.0), EPOXY))
    # the base unit (base plate under the bath, housing at the back) and the lift column
    sh.append(_box((0.110, -0.150, 0.0), (0.700, 0.150, 0.012), HEID))
    sh.append(_box((0.470, -0.130, 0.012), (0.700, 0.130, 0.090), HEID))
    sh.append(_box((0.469, -0.110, 0.030), (0.470, 0.110, 0.075), HEID_D, collide=False))
    sh.append(_box((0.600, -0.050, 0.120), (0.690, 0.050, z(S0[2]) + 0.170), HEID))
    sh.append(_box((0.598, -0.030, 0.140), (0.600, 0.030, z(S0[2]) + 0.150), HEID_D, collide=False))
    # the heating bath: steel pot in a white housing, water inside
    cx, cy = BATH_C
    sh += _ring(cx, cy, BATH_RI, BATH_RO, 0.012, BATH_RIM, HEID, n=36)
    sh += _ring(cx, cy, BATH_RI - 0.002, BATH_RI, BATH_FLOOR, BATH_RIM, STEEL, n=36, collide=False)
    sh.append(CylShape((cx, cy, (0.012 + BATH_FLOOR) / 2), BATH_RI, BATH_FLOOR - 0.012, STEEL))
    sh.append(CylShape((cx, cy, WATER - 0.0005), BATH_RI - 0.002, 0.001, WATER_C, collide=False))
    sh.append(_box((cx - BATH_RO - 0.004, -0.030, 0.030), (cx - BATH_RO + 0.002, 0.030, 0.070), HEID_D, collide=False))
    # the cork ring on the deck
    sh += _ring(CORK_C[0], CORK_C[1], CORK_RI, CORK_RO, z(TOP), z(TOP) + CORK_H, CORK, n=20)
    # the control panel on the deck: a grey console with a display and the lift's two arrow buttons
    px, py = PANEL_C
    sh.append(_box((px - 0.070, py - 0.060, z(TOP)), (px + 0.070, py + 0.060, z(TOP) + PANEL_TOP), HEID_D))
    sh.append(_box((px + 0.002, py - 0.045, z(TOP) + PANEL_TOP), (px + 0.060, py + 0.045, z(TOP) + PANEL_TOP + 0.001),
                   (0.05, 0.08, 0.10), collide=False))
    for (bx, by), mark in ((BTN_DOWN, -1), (BTN_UP, 1)):
        sh += _ring(bx, by, BTN_R + 0.001, BTN_R + 0.004, z(TOP) + PANEL_TOP, z(TOP) + PANEL_TOP + 0.002, (0.35, 0.36,
                    0.38), n=12, collide=False)
        sh.append(_box((bx - 0.014 + mark * 0.0, by - 0.002 + mark * 0.018, z(TOP) + PANEL_TOP),
                       (bx - 0.008, by + 0.002 + mark * 0.018, z(TOP) + PANEL_TOP + 0.0008), (0.9, 0.9, 0.9), False))
    return Body.build("bench", pos=(0.0, 0.0, B), mass=30.0, kinematic=True, shapes=sh, color=EPOXY,
                      friction=(0.6, 0.5))


def _rv_head() -> Body:
    """The rotary head on the lift: the drive housing, its arm to the lift column, the glass vapour tube ending in the
    NS 24/40 ground cone, the EASY CLIP's ring, the vertical condenser and receiving flask above. Origin: the cone's
    base, where the flask's mouth sits when home (S0)."""
    sh = []
    o = S0
    rel = lambda p: tuple((np.asarray(p, float) - o).tolist())  # noqa: E731
    housing = S0 + 0.175 * A
    sh.append(_tilted_box(rel(housing), 0.130, 0.110, 0.110, HEID))
    sh.append(_tilted_box(rel(housing - 0.067 * A), 0.004, 0.090, 0.090, HEID_D, collide=False))
    # the arm from the housing back to the column carriage
    sh.append(_box(rel(housing + [0.050, -0.040, 0.020]), rel([0.600, 0.040, housing[2] + 0.075]), HEID))
    sh.append(_box(rel([0.596, -0.055, housing[2] - 0.010]), rel([0.700, 0.055, housing[2] + 0.090]), HEID))
    # the vapour tube from the housing to the clip ring, the clip ring, the ground cone
    sh += _tilted_cyl(rel(S0 + 0.024 * A), rel(housing - 0.065 * A), 0.013, GLASS, collide=True, n=16)
    sh += _tilted_cyl(rel(S0 + 0.006 * A), rel(S0 + 0.026 * A), 0.021, CLIP_C, collide=True, n=20)
    sh.append(_tilted_box(rel(S0 - 0.014 * A), 0.028, 2 * CONE_R, 2 * CONE_R, FROST))  # the cone's core
    sh.append(_tilted_box(rel(S0 - 0.034 * A), 0.012, 0.009, 0.009, FROST))  # and its narrower tip
    for t in (0.004, 0.012, 0.020, 0.028, 0.036):  # the frosted cone's look (11.4-12 mm radius), not colliding
        r = 0.012 - 0.0006 * t / 0.004
        sh += _tilted_cyl(rel(S0 - t * A + 0.004 * A), rel(S0 - t * A - 0.004 * A), r, FROST, collide=False, n=14)
    # the vertical condenser (G3) above the housing, and the receiving flask hanging below its outlet
    top = housing + 0.060 * N
    sh.append(CylShape(rel(top + [0.0, 0.0, 0.220]), 0.045, 0.400, GLASS_D, collide=False))
    sh.append(CylShape(rel(top + [0.0, 0.0, 0.425]), 0.020, 0.030, GLASS_D, collide=False))
    sh.append(SphereShape(rel(housing + [0.070, -0.110, -0.040]), 0.060, GLASS_D, collide=False))
    sh.append(_box(rel(housing + [0.040, -0.120, 0.010]), rel(housing + [0.090, -0.040, 0.030]), GLASS_D, False))
    return Body.build("rv_head", pos=tuple(S0.tolist()), mass=3.0, shapes=sh, color=HEID, friction=(0.5, 0.4),
                      solver_iterations=32, damping=(0.5, 0.5))


def _lever() -> Body:
    """The EASY CLIP's clamping lever: a 36 mm arm hinged on top of the clip ring, lying along the joint when shut, a
    hook at its tip that drops behind the flask's collar, a ribbed thumb pad on top. Origin on the hinge line; its
    shapes as shut (the joint's 0), it starts turned open."""
    sh = []
    tip = -LEVER_L * A
    sh.append(_tilted_box(tuple((0.5 * tip).tolist()), LEVER_L + 0.006, 0.012, 0.005, CLIP_C))
    pad = tip + 0.006 * A + 0.004 * N
    sh.append(_tilted_box(tuple(pad.tolist()), 0.016, 0.020, 0.004, CLIP_C))
    for k in (-1, 0, 1):
        sh.append(_tilted_box(tuple((pad + 0.005 * k * A + 0.0022 * N).tolist()), 0.0015, 0.020, 0.0008,
                              (0.10, 0.20, 0.45), collide=False))
    sh.append(_tilted_box(tuple((tip - 0.0020 * A - 0.0050 * N).tolist()), 0.004, 0.012, 0.010, CLIP_C))  # the hook
    return Body.build("lever", pos=tuple(HINGE.tolist()), mass=0.025, shapes=sh, color=CLIP_C, friction=(0.8, 0.7),
                      solver_iterations=32, start_xyzw=_qy(math.radians(LEVER_OPEN)), damping=(0.0, 0.5))


def _button(name, xy) -> Body:
    sh = [CylShape((0.0, 0.0, 0.0), BTN_R, 0.008, (0.88, 0.88, 0.90))]
    return Body.build(name, pos=(xy[0], xy[1], BTN_Z0), mass=0.010, shapes=sh, color=(0.88, 0.88, 0.90),
                      friction=(0.9, 0.8), solver_iterations=16)


def _flask() -> Body:
    """A 100 mL round-bottom flask with an NS 24/40 socket: the bulb, the hollow neck (12.5 mm bore), the bead round the
    mouth; 40 mL of amber extract in the bulb."""
    sh = [SphereShape((0.0, 0.0, 0.0), BULB_R, EXTRACT)]  # the glass bulb, the amber extract showing through
    n = 12
    for k in range(n):
        th = 2 * math.pi * (k + 0.5) / n
        rm = (NECK_R0 + NECK_R1) / 2
        w = 2 * NECK_R1 * math.tan(math.pi / n) + 0.0008
        sh.append(BoxShape((rm * math.cos(th), rm * math.sin(th), (NECK_Z0 + NECK_Z1) / 2),
                           (NECK_R1 - NECK_R0, w, NECK_Z1 - NECK_Z0), GLASS, yaw=th))
        rc = (NECK_R0 + COLLAR_R) / 2
        wc = 2 * COLLAR_R * math.tan(math.pi / n) + 0.0008
        sh.append(BoxShape((rc * math.cos(th), rc * math.sin(th), (COLLAR_Z0 + MOUTH_Z) / 2),
                           (COLLAR_R - NECK_R0, wc, MOUTH_Z - COLLAR_Z0), FROST, yaw=th))
    return Body.build("flask", pos=FLASK0, mass=FLASK_MASS, shapes=sh, color=GLASS, friction=(1.0, 0.9),
                      solver_iterations=32, com=(0.0, 0.0, -0.008), damping=(0.0, 0.1))


OBJECTS = [_bench(), _rv_head(), _lever(), _button("btn_down", BTN_DOWN), _button("btn_up", BTN_UP), _flask()]

bodykit.slider("lift", "bench", "rv_head", anchor=tuple(S0.tolist()), axis="Z", limits=(-TRAVEL - 0.002, 0.002),
               stiffness=40000.0, damping=800.0, target=0.0, max_force=800.0)
bodykit.hinge("clip", "rv_head", "lever", anchor=tuple(HINGE.tolist()), axis="Y", limits_deg=(-1.0, 38.0),
              spring=LEVER_SPRING, damping=0.0002, target_deg=LEVER_OPEN + 1.0)
for _n, _xy in (("btn_down", BTN_DOWN), ("btn_up", BTN_UP)):
    bodykit.slider(f"{_n}_spring", "bench", _n, anchor=(_xy[0], _xy[1], BTN_Z0), axis="Z",
                   limits=(-BTN_TRAVEL, 0.0), stiffness=1500.0, damping=3.0, target=0.004, max_force=50.0)
bodykit.thread("socket", "rv_head", "flask", anchor0=(0.0, 0.0, 0.0), axis0=tuple((-A).tolist()),
               anchor1=(0.0, 0.0, MOUTH_Z), axis1=(0.0, 0.0, -1.0), slide_limits=(-0.003, 0.050), turn_damping=0.02,
               slide_stiffness=150.0, slide_damping=3.0, slide_max_force=8.0)

# -- followed after every control step --------------------------------------------------------------------------------


def _fresh() -> dict:
    return {"on": False, "latched": False, "dropped": False, "why": "", "tcps": None, "lift": 0.0, "s": None,
            "lever": LEVER_OPEN, "lifted": False, "was_on": False, "home_ever": False, "clicked_at": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("RV_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin):
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


def _socket_state(fp, fr, hp, hr):
    """The flask's mouth against the cone, in the head's frame: (lateral offset m, angle deg, s m off home)."""
    mouth = fp + fr @ np.array([0.0, 0.0, MOUTH_Z])
    out_dir = hr @ (-A)  # the axis, outward
    rel = mouth - hp
    s = float(rel @ out_dir)
    off = rel - s * out_dir
    lat = float(np.linalg.norm(off))
    _EP["off_yn"] = (float(off @ (hr @ np.array([0.0, 1.0, 0.0]))), float(off @ (hr @ N)))
    fa = fr @ np.array([0.0, 0.0, -1.0])  # the flask's axis, mouth toward bulb
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(fa @ out_dir)))))
    return lat, ang, s


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    fp, fr = _pose(base, origin, "flask")
    hp, hr = _pose(base, origin, "rv_head")
    lp, lr = _pose(base, origin, "lever")
    rel = hr.T @ lr
    lever = math.degrees(math.atan2(rel[0, 2], rel[0, 0]))
    lat, ang, s = _socket_state(fp, fr, hp, hr)
    lift = float(hp[2] - S0[2])
    # buttons: the lift runs while one is held in past its switch point
    down = float(_pose(base, origin, "btn_down")[0][2] - BTN_Z0) < -BTN_SWITCH
    up = float(_pose(base, origin, "btn_up")[0][2] - BTN_Z0) < -BTN_SWITCH
    tgt = _EP.get("lift_target", 0.0)
    if down and not up:
        tgt = max(-TRAVEL, tgt - LIFT_SPEED / 20.0)
    elif up and not down:
        tgt = min(0.0, tgt + LIFT_SPEED / 20.0)
    if tgt != _EP.get("lift_target", 0.0):
        bodykit.set_slider_drive("lift", target=tgt)
    _EP["lift_target"] = tgt
    # the socket on the cone
    if not _EP["on"]:
        if lat <= CATCH_LAT and ang <= CATCH_DEG and -0.003 <= s <= CONE_L - 0.008 and not _EP["latched"]:
            bodykit.set_thread_on("socket", True)
            bodykit.set_slide_target("socket", max(0.0, s))
            _EP["on"], _EP["was_on"] = True, True
    else:
        if _EP["latched"]:
            bodykit.set_slide_target("socket", -0.004)  # held home against the end of the cone
        else:
            bodykit.set_slide_target("socket", max(0.0, s))
            if s > OFF_S:
                bodykit.set_thread_on("socket", False)
                _EP["on"] = False
    home = _EP["on"] and s <= HOME_S
    if home:
        _EP["home_ever"] = True
    # the clamp: clicks shut only over a socket that is home
    if not _EP["latched"] and lever <= CLICK_DEG and home:
        bodykit.set_hinge_drive("clip", target_deg=0.0, spring=0.08, damping=0.002)
        _EP["latched"], _EP["clicked_at"] = True, _DEBUG["steps"]
    if _EP["latched"] and lever > 12.0:  # prised open again
        bodykit.set_hinge_drive("clip", target_deg=LEVER_OPEN + 1.0, spring=LEVER_SPRING, damping=0.0002)
        _EP["latched"] = False
    # out of the cork ring; dropped
    in_ring = math.hypot(fp[0] - CORK_C[0], fp[1] - CORK_C[1]) < 0.015 and fp[2] < FLASK0[2] + 0.010
    if fp[2] > FLASK0[2] + 0.020:
        _EP["lifted"] = True
    if not _EP["on"] and fp[2] < DROP_Z and not in_ring and not _EP["dropped"]:
        _EP["dropped"], _EP["why"] = True, f"flask dropped (bulb centre at {1000 * (fp[2] - TOP):.0f} mm)"
    _EP.update({"lever": lever, "lift": lift, "s": s, "lat": lat, "ang": ang, "home": home})
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000, 1).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            yn = [round(v * 1000, 1) for v in _EP.get("off_yn", (0, 0))]
            print(f"[rv] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]} flask={mm(fp - [0, 0, TOP])}"
                  f" lat={lat * 1000:.1f} yn={yn} ang={ang:.1f} s={s * 1000:.1f} on={_EP['on']} lever={lever:.1f}"
                  f" latched={_EP['latched']} lift={lift * 1000:.1f} down={down} L={mm(tl - [0, 0, TOP])}"
                  f" R={mm(tr - [0, 0, TOP])} dropped={_EP['dropped']} {_EP['why']} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once the flask has been dropped (off the cone and fallen below 35 mm above the deck, out of its cork ring:
    into the bath, onto the bench or the deck). Else, at the end, on live states: 1.0: the flask home on the vapour
    tube's cone (within 4 mm of fully on), the EASY CLIP clicked shut over its collar, the lift down at its immersion
    stop (within 4 mm), so the flask sits in the bath, and both arms back where they started; 0.75: the same with an
    arm not back; 0.5: the flask home and clamped, the lift not down; 0.25: the flask on the cone (the socket onto it),
    not clamped; else 0.0."""
    if _EP["s"] is None or _EP["dropped"]:
        return 0.0
    clamped = _EP["on"] and _EP["latched"] and _EP["home"]
    if clamped:
        if abs(_EP["lift"] + TRAVEL) * 1000 <= LOWER_MM:
            return 1.0 if _arms_home() else 0.75
        return 0.5
    return 0.25 if _EP["on"] else 0.0


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
    """Full score with the flask, the head and the lever at rest: the trial ends once that has held SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("flask", "rv_head", "lever"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand holds the flask by its neck from the side (fingers pointing +y, closing along x), lifts it out of the
# cork ring, turns it with a wrist roll until its neck lies along the vapour tube's axis, and slides its socket onto the
# cone along that axis, home. The left hand presses the clamp's lever down onto the joint, square to it, past the
# click. The right hand lets go and leaves; the left holds the lift's DOWN button until the flask is in the bath at the
# stop. Both arms go home.

SIDE = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])  # fingers toward +y, closing along x
ROT_R0 = SIDE
ROT_R = _ry(TILT * 2.0) @ SIDE  # rolled 60 degrees: the neck (upright before) along A
ROT_L = _ry(-TILT) @ grip_rotation(0.0)  # fingers square to the lever (along -N), closing along y
ROT_LB = grip_rotation(0.0)
R_OPEN = -float(os.environ.get("RV_ROPEN", 0.021))
PRE_S = float(os.environ.get("RV_PRES", 0.050))  # the socket's mouth this far out along the axis before sliding on
PUSH_OVER = float(os.environ.get("RV_PUSHOVER", 0.010))  # the hand aims this far past home (arm lag)
AIM = np.array([float(os.environ.get("RV_AIMX", 0.0)), float(os.environ.get("RV_AIMY", 0.0)),
                float(os.environ.get("RV_AIMZ", 0.0))])  # the grip's measured offset from the neck's axis
PRESS_OVER = float(os.environ.get("RV_PRESSOVER", 0.012))
PRESS_AIM = np.array([float(os.environ.get("RV_PAIMX", 0.0)), 0.0, float(os.environ.get("RV_PAIMZ", 0.0))])
LEAD_S, LEAD_STEPS = 3.0, 15


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
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger, t=LEAD_S, steps=LEAD_STEPS) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(t / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_CLOSED, r)
    moved = set()
    one = os.environ.get("RV_ONEHAND")  # checks only: "left" or "right" has that arm work alone, the other at home
    chk = os.environ.get("RV_CHECK", "")

    def go(arm, t, pos, finger, label, n=1, rot=None):
        p0, _, r0 = cur[arm]
        r1 = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r1))
        cur[arm] = (p1, finger, r1)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync():
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), "wait for the other hand")

    def lead(arm, pos, rot, label, finger, t=LEAD_S):
        q0 = np.array(READY, float) if arm not in moved else _good_ik(arm, cur[arm][0], cur[arm][2])
        moved.add(arm)
        plan[arm].extend(_joint_path(arm, q0, _good_ik(arm, pos, rot), label, finger, t=t))
        cur[arm] = (np.asarray(pos, float), finger, rot)

    if chk == "clipfirst":  # checks only: the lever pressed shut before the flask is on the cone
        lever_tip0 = HINGE - (LEVER_L - 0.006) * A + 0.0085 * N + PRESS_AIM
        lead("left", lever_tip0 + 0.013 * N + 0.050 * N, ROT_L, "over the clamp's lever", FINGER_CLOSED)
        go("left", 1.0, lever_tip0 + 0.013 * N + 0.025 * N, FINGER_CLOSED, "onto the open lever")
        go("left", 1.4, lever_tip0 + 0.013 * N - PRESS_OVER * N, FINGER_CLOSED, "press it shut, no flask on", n=3)
        wait("left", 0.5)
        go("left", 0.8, lever_tip0 + 0.013 * N + 0.050 * N, FINGER_CLOSED, "lift off")
        sync()
    # 1. the right hand takes the flask by its neck, from the side
    neck0 = np.array(FLASK0) + [0.0, 0.0, GRIP_Z]
    lead("right", np.array([0.220, -0.100, TOP + 0.140]), ROT_R0, "turn the hand to grip from the side", R_OPEN)
    go("right", 1.6, np.array([0.240, 0.120, TOP + 0.140]), R_OPEN, "across in front of the evaporator", n=3)
    go("right", 1.0, neck0 - [0.0, 0.060, 0.0], R_OPEN, "down beside the flask's neck, fingers open", n=2)
    go("right", 1.2, neck0, R_OPEN, "fingers round the neck, just above the bulb")
    go("right", 0.8, neck0, FINGER_CLOSED, "take the flask by its neck")
    wait("right", 0.3)
    go("right", 1.2, neck0 + [0.0, 0.0, 0.090], FINGER_CLOSED, "lift it straight out of the cork ring", n=2)
    # 2. turn it so its neck lies along the vapour tube's axis, mouth below the cone, and slide it on
    grip_home = GRIP_SEAT + AIM
    pre = grip_home - PRE_S * A
    p0, _, _ = cur["right"]
    p1 = pre - 0.030 * A + np.array([0.0, 0.0, 0.020])
    steps = 8
    for j in range(1, steps + 1):
        f = j / steps
        plan["right"].append(Waypoint(3.2 / steps, tuple(float(v) for v in p0 + (p1 - p0) * f), FINGER_CLOSED,
                                      "carry it over and turn it to the tube's slope, below the cone",
                                      _ry(2.0 * TILT * f) @ SIDE))
    cur["right"] = (p1, FINGER_CLOSED, ROT_R)
    go("right", 1.0, pre, FINGER_CLOSED, "socket in line with the cone")
    if chk != "noseat":
        push = grip_home + PUSH_OVER * A if chk != "short" else grip_home - 0.008 * A  # short: 8 mm short of home
        go("right", 2.0, push, FINGER_CLOSED, "slide the socket up the cone along its axis, home", n=4)
        wait("right", 0.6, "keep it pressed home")
    # 3. the left hand presses the clamp's lever down onto the joint, square to it, past its click
    lever_tip_shut = HINGE - (LEVER_L - 0.006) * A + 0.0085 * N + PRESS_AIM  # 2 mm over the thumb pad, shut
    tcp_off = 0.013 * N  # the closed fingertips reach 13 mm past the TCP
    sync()
    if chk not in ("noclip", "clipfirst") and one != "right":
        lead("left", lever_tip_shut + tcp_off + 0.050 * N, ROT_L, "over the clamp's lever, square to the joint",
             FINGER_CLOSED)
        go("left", 1.0, lever_tip_shut + tcp_off + 0.025 * N, FINGER_CLOSED, "onto the open lever's thumb pad")
        go("left", 1.4, lever_tip_shut + tcp_off - PRESS_OVER * N, FINGER_CLOSED, "press the lever down past its click",
           n=3)
        wait("left", 0.5, "hold it shut a moment")
        go("left", 0.8, lever_tip_shut + tcp_off + 0.050 * N, FINGER_CLOSED, "lift off the lever")
    sync()
    # 4. the right hand lets go of the flask (the clamp holds it) and leaves
    hold = cur["right"][0]
    go("right", 0.6, hold, R_OPEN, "let go of the flask")
    go("right", 0.8, hold - [0.0, 0.070, 0.0], R_OPEN, "draw the fingers back off the neck")
    go("right", 0.8, hold - [0.0, 0.070, -0.060], FINGER_CLOSED, "up, clear")
    sync()
    # 5. the left hand holds the lift's DOWN button until the flask is down in the bath
    if chk != "nolower" and one != "right":
        btn = np.array([BTN_DOWN[0], BTN_DOWN[1], BTN_TOP + 0.013])  # TCP with the fingertips on the key
        press = float(os.environ.get("RV_BTNPRESS", 0.014))
        arm = "left"
        lead(arm, btn + [0.0, 0.0, 0.060], ROT_LB, "over the lift's DOWN button", FINGER_CLOSED)
        go(arm, 0.8, btn - [0.0, 0.0, press], FINGER_CLOSED, "press the DOWN button in")
        wait(arm, TRAVEL / LIFT_SPEED + 1.2, "hold it: the lift takes the flask down into the bath to its stop")
        go(arm, 0.6, btn + [0.0, 0.0, 0.060], FINGER_CLOSED, "let the button go")
    # 6. both arms go home and hold still
    for a in ("left", "right"):
        if a not in moved:
            p, r = ik.tcp_pose(np.array(READY, float), station.BASES[a])
            plan[a].append(Waypoint(1.0, tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r))
            continue
        p, f, r = cur[a]
        plan[a] += _joint_path(a, _good_ik(a, p, r), np.array(READY, float), "back to where it started", f)
        last = plan[a][-1]
        plan[a].append(Waypoint(3.0, last.pos, f,
                                "hold still: the trial ends once the full score has held 1 s with both arms home",
                                last.yaw))
    if one in ("left", "right"):  # checks only: the other arm stays home all trial
        other = "left" if one == "right" else "right"
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[other])
        plan[other] = [Waypoint(max(dur(one), 1.0), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    return plan
