"""A 1.5 litre inside-fitting (inner-lid) aluminium pressure cooker stands on the kitchen counter with rice and water in
it, its long body handle pointing left and toward the robot. Its oval lid lies top up on the counter to the right, the
lid handle pointing toward the robot at right angles to the body handle.

The cooker is closed the way its maker describes: the lid, which is larger than the oval mouth, goes in tilted and
crosswise, is turned in line with the body handle and lowered inside; the lid's handle bar is set on the pivot at the
rim, the lid is pulled up against the underside of the rim, and the locking loop that hangs from the lid handle's end is
pushed in under the body handle's end so the lid stays up against the rim.

Task: close the pressure cooker, ready to go on the stove.

Run it:  inspect-robots run --embodiment yam_arena -E task=close_an_inside_fitting_pressure_cooker.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     plate, rotation, segment)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402,F401
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()


def _quat_from_matrix(m) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, by Shepperd's method (bodykit's own picks wrong signs near 180 deg)."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        k = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * k, (m[2, 1] - m[1, 2]) / k, (m[0, 2] - m[2, 0]) / k, (m[1, 0] - m[0, 1]) / k
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        k = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / k, 0.25 * k, (m[0, 1] + m[1, 0]) / k, (m[0, 2] + m[2, 0]) / k
    elif m[1, 1] > m[2, 2]:
        k = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / k, (m[0, 1] + m[1, 0]) / k, 0.25 * k, (m[1, 2] + m[2, 1]) / k
    else:
        k = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / k, (m[0, 2] + m[2, 0]) / k, (m[1, 2] + m[2, 1]) / k, 0.25 * k
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


bodykit._quat_from_matrix = _quat_from_matrix


def _qf(m) -> tuple[float, float, float, float]:
    """A start orientation as plain floats (Isaac Lab's pose conversion refuses NumPy float64 scalars)."""
    return tuple(float(v) for v in _quat_from_matrix(m))

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PHYSICS_DT = 1.0 / 240.0

# -- the cooker body (metres, in its own frame: origin at the middle of its base, on the counter; the body handle
#    points along -x). A 1.5 litre Hawkins-Classic-type inner-lid cooker: aluminium, 3.25 mm thick. -----------------
CX, CY = 0.340, -0.160  # where the cooker stands (world); turned so its body handle points left (+y)
COOKER_YAW = -math.pi / 4  # the body handle points left and toward the robot (45 deg)
R_BASE = 0.065  # flat base 130 mm across (maker's catalogue)
R_OUT = 0.0795  # body 159 mm across
WALL_T = 0.0032
R_IN = R_OUT - WALL_T
Z_CURVE = 0.022  # the rounded bottom edge reaches the straight wall here
Z_LIP0, Z_LIP1 = 0.102, 0.105  # the rim's inward lip: underside, top (the rim top is 105 mm above the counter)
MOUTH_X, MOUTH_Y = 0.060, 0.069  # the oval mouth's half-axes: 120 x 138 mm, its long axis across the handle
Z_FOOD = 0.026  # rice and water, 0.40 kg, 22 mm deep
BODY_MASS, FOOD_MASS = 1.04, 0.40
# the body handle (bakelite) on its bracket, and the fixed pivot the lid's handle bar rests on
BH_X0, BH_X1 = -0.2355, -0.105  # the bakelite handle, from its end to the bracket
BH_W = 0.026
BH_Z0, BH_Z1 = 0.0745, 0.1025
BH_LOW = 0.0795  # the handle's lower layer (BH_Z0..BH_LOW) is cut away where the loop's crossbar hooks in
BR_X0 = -R_OUT  # the bracket from the wall to the handle
BR_Z0 = 0.080
PIV_X = -0.093  # the pivot: a 6 x 10.5 mm steel post on the bracket
PIV_TOP = 0.1165
# -- the lid (in its own frame: origin at the middle of the lid plate's top face; seated, its axes are the cooker's) --
LID_X, LID_Y = 0.064, 0.073  # the lid's half-axes: 128 x 146 mm, 4 mm larger than the mouth all round
PLATE_T = 0.0025
GASKET_W, GASKET_TOP, GASKET_BOT = 0.005, 0.0015, -0.004  # the rubber gasket on the lid's edge
LID_SEAT_Z = Z_LIP0 - GASKET_TOP  # the lid's origin when seated (cooker frame): its gasket against the lip
BAR_Z0, BAR_Z1 = 0.016, 0.024  # the steel handle bar, 10 x 8 mm, from the vent tube out to the lid handle
BAR_W = 0.010
BAR_X1 = -0.1505  # the bar ends in the lid handle
LH_OVER = 0.025  # the lid handle reaches this far past the body handle's end, so the loop hangs clear of it
LH_X0, LH_W, LH_Z0 = BH_X0 - LH_OVER, 0.022, 0.004  # the lid handle (bakelite): 110 mm long, 22 mm wide
TUBE_R, TUBE_H = 0.005, 0.034
LID_MASS = 0.21
LID_COM = (-0.038, 0.0, 0.005)
# -- the locking loop: a steel wire loop hinged at the lid handle's end, hanging down past the body handle's end ------
HINGE_X, HINGE_Z = LH_X0 - 0.003, 0.017  # in the lid's frame
LOOP_L = 0.060  # hinge to the crossbar's centre
LOOP_HALF_W = 0.022  # its legs 44 mm apart (outside), straddling both handles
WIRE = 0.0035
WIRE_R = WIRE / 2
LOOP_MASS = 0.008
LATCH_DEG = (36.0, 48.0)  # the crossbar is hooked in under the body handle's end between these swings (deg inward)

START_INSIDE = bool(os.environ.get("PC_START_INSIDE"))  # debug only: start with the lid seated (tests the latch)
RC_ = np.array([[math.cos(COOKER_YAW), -math.sin(COOKER_YAW), 0.0], [math.sin(COOKER_YAW), math.cos(COOKER_YAW), 0.0],
                [0.0, 0.0, 1.0]])
LID_IN = np.array([CX, CY, TOP]) + RC_ @ np.array([0.0, 0.0, LID_SEAT_Z])
LID0_ROT = np.array([[math.cos(COOKER_YAW + math.pi / 2), -math.sin(COOKER_YAW + math.pi / 2), 0.0],
                     [math.sin(COOKER_YAW + math.pi / 2), math.cos(COOKER_YAW + math.pi / 2), 0.0],
                     [0.0, 0.0, 1.0]])  # lying with its handle toward the robot, at right angles to the body handle
LID0 = np.array([0.420, -0.380, TOP - GASKET_BOT])  # the lid on the counter (origin), unrotated: its handle toward
# the robot, at right angles to the body handle

ALU, ALU_DARK, ALU_LIGHT = (0.78, 0.79, 0.81), (0.62, 0.63, 0.66), (0.86, 0.87, 0.89)
BAKELITE, RUBBER, STEEL = (0.07, 0.07, 0.08), (0.10, 0.10, 0.11), (0.66, 0.67, 0.70)
FOOD = (0.93, 0.91, 0.82)

SUCCESS_HOLD_S = 1.0


# -- shapes -----------------------------------------------------------------------------------------------------

def _ring(n, r_in, r_out, z_in, z_out, t, color, collide=True) -> list:
    """A ring (or cone) of n flat panels t thick, from radius r_in at height z_in to r_out at z_out (its inner face)."""
    width = 2 * max(r_in, r_out) * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        u = np.array([math.cos(a), math.sin(a), 0.0])
        p1 = r_in * u + [0.0, 0.0, z_in]
        p2 = r_out * u + [0.0, 0.0, z_out]
        d = p2 - p1
        nrm = np.cross(d, np.cross([0.0, 0.0, 1.0], u))
        if np.dot(nrm, u) < 0:
            nrm = -nrm
        nrm = nrm / np.linalg.norm(nrm)
        out.append(plate(tuple(p1 + nrm * t / 2), tuple(p2 + nrm * t / 2), width, t, tuple(nrm), color, collide))
    return out


def _quat_y(phi: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(phi / 2), 0.0, math.cos(phi / 2))


def _ellipse(a_x, a_y, t):
    """A point on the ellipse with half-axes a_x (along x), a_y (along y), and its outward unit normal."""
    p = np.array([a_x * math.cos(t), a_y * math.sin(t)])
    n = np.array([math.cos(t) / a_x, math.sin(t) / a_y])
    return p, n / np.linalg.norm(n)


def _cooker() -> Body:
    s = [CylShape((0.0, 0.0, 0.002), R_BASE, 0.004, ALU_DARK)]  # the flat base
    s += _ring(48, R_BASE - 0.002, R_IN, 0.004, Z_CURVE, WALL_T, ALU)  # the rounded bottom edge
    for k in range(48):  # the straight wall
        a = 2 * math.pi * (k + 0.5) / 48
        u = (math.cos(a), math.sin(a), 0.0)
        m = R_IN + WALL_T / 2
        s.append(plate((m * u[0], m * u[1], Z_CURVE - 0.002), (m * u[0], m * u[1], Z_LIP0), 2 * R_OUT * math.tan(math.pi / 48)
                       + 0.001, WALL_T, u, ALU))
    # the lip round the oval mouth: 72 pieces, each from the mouth's edge out along its normal to the wall
    n_lip = 72
    for k in range(n_lip):
        t = 2 * math.pi * k / n_lip
        p, nrm = _ellipse(MOUTH_X, MOUTH_Y, t)
        b = float(p @ nrm)
        length = -b + math.sqrt(b * b - (p @ p) + R_OUT ** 2)  # to the outer wall along the normal
        c = p + nrm * length / 2
        s.append(BoxShape((float(c[0]), float(c[1]), (Z_LIP0 + Z_LIP1) / 2), (length + 0.0005, 0.0075, Z_LIP1 - Z_LIP0),
                          ALU, yaw=math.atan2(nrm[1], nrm[0])))
        e = p + nrm * 0.0012  # the rolled edge of the mouth (drawn)
        s.append(BoxShape((float(e[0]), float(e[1]), Z_LIP1 + 0.0004), (0.0024, 0.0062, 0.0008), ALU_LIGHT,
                          yaw=math.atan2(nrm[1], nrm[0]), collide=False))
    s.append(CylShape((0.0, 0.0, (0.004 + Z_FOOD) / 2), R_IN - 0.0015, Z_FOOD - 0.004, FOOD))  # rice and water
    # the body handle's bracket and the pivot post on it
    s.append(BoxShape(((BR_X0 + BH_X1) / 2, 0.0, (BR_Z0 + BH_Z1) / 2), (abs(BH_X1 - BR_X0), 0.024, BH_Z1 - BR_Z0),
                      STEEL))
    s.append(BoxShape((PIV_X, 0.0, (BH_Z1 + PIV_TOP) / 2), (0.006, 0.0105, PIV_TOP - BH_Z1), STEEL))  # flat top
    for xr in (-0.086, -0.099):  # rivets (drawn)
        s.append(CylShape((xr, 0.0, BR_Z0 + 0.008), 0.003, 0.0255, ALU_LIGHT, collide=False, axis="Y"))
    # the bakelite body handle; its lower layer is cut away round the hook pocket for the loop's crossbar
    hx, hz = _hinge_seated()
    pk0 = hx + (LOOP_L - WIRE_R) * math.sin(math.radians(31.0)) - 0.0005  # the pocket, from here ...
    pk1 = hx + (LOOP_L + WIRE_R) * math.sin(math.radians(LATCH_DEG[1])) + 0.0003  # ... to its end wall
    s.append(BoxShape(((BH_X0 + BH_X1) / 2, 0.0, (BH_LOW + BH_Z1) / 2), (BH_X1 - BH_X0, BH_W, BH_Z1 - BH_LOW), BAKELITE))
    s.append(BoxShape(((BH_X0 + pk0) / 2, 0.0, (BH_Z0 + BH_LOW) / 2), (pk0 - BH_X0, BH_W, BH_LOW - BH_Z0), BAKELITE))
    s.append(BoxShape(((pk1 + BH_X1) / 2, 0.0, (BH_Z0 + BH_LOW) / 2), (BH_X1 - pk1, BH_W, BH_LOW - BH_Z0), BAKELITE))
    # the pocket's roof: an arc round the loop's hinge (lid seated), 0.3 mm clear of the crossbar
    r_roof = LOOP_L - WIRE_R - 0.0003
    a0, a1 = math.radians(34.0), math.radians(LATCH_DEG[1] + 1.5)
    n_r = 4
    for k in range(n_r):
        th = a0 + (a1 - a0) * (k + 0.5) / n_r
        rc = r_roof - 0.003
        c = (hx + rc * math.sin(th), 0.0, hz - rc * math.cos(th))
        s.append(BoxShape(c, (r_roof * (a1 - a0) / n_r + 0.0015, BH_W, 0.006), BAKELITE, quat_xyzw=_quat_y(-th)))
    for xs in (-0.215, -0.135):  # the handle's two screws (drawn)
        s.append(CylShape((xs, 0.0, BH_Z1 - 0.012), 0.0035, BH_W + 0.001, STEEL, collide=False, axis="Y"))
    mass = BODY_MASS + FOOD_MASS
    return Body.build("cooker", pos=(CX, CY, TOP), mass=mass, shapes=s, color=ALU, friction=(0.40, 0.35), yaw=COOKER_YAW,
                      com=(-0.012, 0.0, 0.043), solver_iterations=32, contact_offset=0.002)


def _hinge_seated() -> tuple[float, float]:
    """The loop's hinge (x, z) in the cooker's frame with the lid seated."""
    return HINGE_X, LID_SEAT_Z + HINGE_Z


def _lid() -> Body:
    s = []
    # the lid plate: strips across its short axis, each inside the oval (the gasket ring covers the steps at the edge)
    n = 16
    w = 2 * LID_X / n
    for k in range(n):
        x0, x1 = -LID_X + k * w, -LID_X + (k + 1) * w
        xe = max(abs(x0), abs(x1)) - 0.0005
        half = LID_Y * math.sqrt(max(0.0, 1.0 - (xe / LID_X) ** 2))
        if half < 0.004:
            xe = min(abs(x0), abs(x1))
            half = LID_Y * math.sqrt(max(0.0, 1.0 - (xe / LID_X) ** 2)) - 0.004
        s.append(BoxShape(((x0 + x1) / 2, 0.0, -PLATE_T / 2), (w + 0.0004, 2 * half, PLATE_T), ALU))
    # the vent tube in the middle, its bracket under the bar, the safety valve under the bar
    s.append(CylShape((0.0, 0.0, TUBE_H / 2), TUBE_R, TUBE_H, STEEL))
    s.append(CylShape((0.0, 0.0, TUBE_H - 0.002), TUBE_R + 0.0012, 0.004, ALU_LIGHT, collide=False))
    s.append(BoxShape((-0.001, 0.0, (BAR_Z0 + BAR_Z1) / 2), (0.016, 0.014, BAR_Z1 - BAR_Z0), STEEL))
    s.append(CylShape((0.0, 0.0, -PLATE_T - 0.0005), 0.008, 0.001, STEEL, collide=False))  # the vent tube's nut
    s.append(CylShape((-0.040, 0.0, 0.004), 0.0055, 0.008, ALU_LIGHT))  # the safety valve
    s.append(BoxShape(((-0.009 + BAR_X1) / 2, 0.0, (BAR_Z0 + BAR_Z1) / 2), (-0.009 - BAR_X1, BAR_W, BAR_Z1 - BAR_Z0), STEEL))
    s.append(BoxShape(((LH_X0 + BAR_X1) / 2, 0.0, (LH_Z0 + BAR_Z1) / 2), (BAR_X1 - LH_X0, LH_W, BAR_Z1 - LH_Z0), BAKELITE))
    s.append(CylShape((LH_X0 + 0.012, 0.0, (LH_Z0 + BAR_Z1) / 2), 0.0028, LH_W + 0.0008, STEEL, collide=False,
                      axis="Y"))  # a rivet (drawn)
    s.append(BoxShape((HINGE_X + 0.001, 0.0, HINGE_Z), (0.004, 2 * LOOP_HALF_W - 0.006, 0.005), STEEL,
                      collide=False))  # the loop's knuckle at the handle's end (drawn)
    for k in range(48):  # the black rubber gasket on the lid's edge (the outer edge of the lid's outline)
        t = 2 * math.pi * k / 48
        p, nrm = _ellipse(LID_X, LID_Y, t)
        c = p - nrm * GASKET_W / 2
        dp = np.array([-LID_X * math.sin(t), LID_Y * math.cos(t)])
        seg = float(np.linalg.norm(dp)) * 2 * math.pi / 48 + 0.0015
        s.append(BoxShape((float(c[0]), float(c[1]), (GASKET_TOP + GASKET_BOT) / 2),
                          (GASKET_W, seg, GASKET_TOP - GASKET_BOT), RUBBER, yaw=math.atan2(nrm[1], nrm[0])))
    if START_INSIDE:  # debug only: the lid starts seated in the cooker (with nothing holding it, it swings down)
        return Body.build("lid", pos=tuple(LID_IN), mass=LID_MASS, shapes=s, color=ALU, friction=(0.8, 0.7),
                          com=LID_COM, start_xyzw=_qf(RC_), solver_iterations=32, contact_offset=0.0015)
    return Body.build("lid", pos=tuple(LID0), mass=LID_MASS, shapes=s, color=ALU, friction=(0.8, 0.7), com=LID_COM,
                      start_xyzw=_qf(LID0_ROT), solver_iterations=32, contact_offset=0.0015)


LOOP_START_DEG = 72.0  # on the counter the loop lies folded out, resting on the counter beyond the handle


def _loop() -> Body:
    """The locking loop, its origin on its hinge line; built hanging straight down."""
    s = []
    for sy in (1.0, -1.0):
        s.append(segment((0.0, sy * (LOOP_HALF_W - WIRE_R), 0.0), (0.0, sy * (LOOP_HALF_W - WIRE_R), -LOOP_L), WIRE, STEEL))
    s.append(segment((0.0, -(LOOP_HALF_W - WIRE_R), -LOOP_L), (0.0, LOOP_HALF_W - WIRE_R, -LOOP_L), WIRE, STEEL))
    s.append(segment((0.0, -(LOOP_HALF_W - WIRE_R), 0.0), (0.0, LOOP_HALF_W - WIRE_R, 0.0), WIRE, STEEL,
                     collide=False))  # the hinge pin
    hinge_w = LID0 + LID0_ROT @ np.array([HINGE_X, 0.0, HINGE_Z])
    q = _qf(LID0_ROT @ rotation(_quat_y(math.radians(LOOP_START_DEG))))
    if START_INSIDE:
        hinge_w = LID_IN + RC_ @ np.array([HINGE_X, 0.0, HINGE_Z])
        q = _qf(RC_)
    return Body.build("loop", pos=tuple(hinge_w), mass=LOOP_MASS, shapes=s, color=STEEL, friction=(0.5, 0.4),
                      com=(0.0, 0.0, -0.032), start_xyzw=q, solver_iterations=32, contact_offset=0.0012,
                      damping=(0.0, 0.2))


bodykit.hinge("loop_hinge", "lid", "loop", anchor=tuple((LID_IN if START_INSIDE else LID0) + np.array([HINGE_X, 0.0, HINGE_Z])),
              axis="Y",
              limits_deg=(-50.0, 80.0), damping=2.0e-5)

OBJECTS = [_cooker(), _lid(), _loop()]
MOVING = ["cooker", "lid", "loop"]

# note each body's prim path (for the contact views)
_J: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name in ("cooker", "lid", "loop"):
        from pxr import PhysxSchema

        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body"))
        api.CreateThresholdAttr(0.0)
    _J[spec.name] = f"{prim_path}/body"
    _J["env"] = prim_path.rsplit("/", 1)[0]


bodykit._author_sliders = _author_sliders

PADS = [(arm, tip) for arm in ("LeftArm", "RightArm") for tip in ("tip_left", "tip_right")]
_CT = {"views": None, "failed": ""}


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _views():
    """PhysX contact views, made once: (0) the lid and the loop against the cooker; (1) the lid, the loop and the cooker
    against the four finger pads."""
    need = {"cooker", "lid", "loop", "env"}
    if _CT["views"] is None and not _CT["failed"] and need <= _J.keys():
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            env, links = _J["env"], asset_link_paths()
            pads = [f"{env}/{arm}{links[tip]}" for arm, tip in PADS]
            sim = PhysxManager.get_physics_sim_view()
            body = sim.create_rigid_contact_view([_J["lid"], _J["loop"]], filter_patterns=[[_J["cooker"]]] * 2,
                                                 max_contact_data_count=0)
            pad = sim.create_rigid_contact_view([_J["lid"], _J["loop"], _J["cooker"]], filter_patterns=[pads] * 3,
                                                max_contact_data_count=0)
            _CT["views"] = (body, pad)
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
    return _CT["views"]


def _contacts():
    """((lid on cooker, loop on cooker) N, (3, 4) pad force magnitudes on the lid, the loop and the cooker) or None."""
    v = _views()
    if v is None:
        return None
    try:
        a = np.linalg.norm(_np(v[0].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(2, 3), axis=1)
        b = np.linalg.norm(_np(v[1].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(3, 4, 3), axis=2)
    except Exception as e:  # noqa: BLE001
        _CT["failed"] = "read: " + repr(e)[:300]
        return None
    return a, b


# -- followed after every control step --------------------------------------------------------------------------
# Seated: each check is one a real trial makes on the closed cooker (the README gives the real instrument for each).
ALIGN_OFF = 0.013  # in line: the middle of the lid handle's end within 13 mm sideways of the body handle's centre line
GAP_MAX = 0.004  # handles together: the lid handle's underside within 4 mm of the body handle's top at its end
DEPTH_SEAT = Z_LIP1 - LID_SEAT_Z  # 4.5 mm: the lid's top face below the rim's top with the lid seated
DEPTH_TOL = 0.002  # ... within 2 mm of that at each of the four depth points
DEPTH_IN = 0.005  # the depth points: 5 mm in from the mouth's edge, on its diagonals (clear of the bar and the valve)
N_CHECK = 24  # the witness ring: points round the gasket's outer edge, 18 mm apart, ...
SEAT_OVERLAP = 0.001  # ... each at least 1 mm in under the rim (outside the mouth's edge) ...
SEAT_TOL = 0.002  # ... and within 2 mm of the rim's underside (the rigid gasket; a real one is squeezed onto it)
# Latch hold: once both hands have been off the closed, latched lid for LIFT_AFTER_S, the scorer lifts the lid handle's
# end straight up with LIFT_N for LIFT_S (a real trial does it with a spring scale); it must stay seated throughout.
LIFT_N = 1.0  # N: 100 g on a spring scale (the sim latch, held by friction at the pivot, lets go at about 1.8 N)
LIFT_S = 1.0
LIFT_AFTER_S = 0.5
LIFT_RAMP = float(os.environ.get("PC_LIFT_RAMP", "0"))  # debug only: ramp the lift at this many N/s to find what
# the latch holds
LIFT_LOCAL = np.array([LH_X0 + 0.010, 0.0, LH_Z0])  # where: under the lid handle, 10 mm in from its end (lid frame)
TIP_DEG = 25.0
FLOOR_Z = 0.30
CLEAR = 0.05  # at the end each gripper is at least this far from the lid handle's end and from the loop
GRIP_N = 0.5


def _gasket_points() -> np.ndarray:
    """N_CHECK points on the gasket's top at the lid's outer edge (lid frame)."""
    pts = []
    for k in range(N_CHECK):
        p, _ = _ellipse(LID_X, LID_Y, 2 * math.pi * k / N_CHECK)
        pts.append((p[0], p[1], GASKET_TOP))
    return np.array(pts)


def _depth_points() -> np.ndarray:
    """The four depth points (x, y) in the cooker's frame: 5 mm in from the mouth's edge on its diagonals."""
    out = []
    for deg in (45.0, 135.0, 225.0, 315.0):
        p, nrm = _ellipse(MOUTH_X, MOUTH_Y, math.radians(deg))
        out.append(p - nrm * DEPTH_IN)
    return np.array(out)


GASKET_PTS = _gasket_points()
DEPTH_PTS = _depth_points()


def _fresh() -> dict:
    return {"inside": False, "seated": False, "latched": False, "gap": 1.0, "loop_deg": 0.0, "hands_off": False,
            "tipped": False, "fallen": False, "steps": 0, "err": "", "max_tilt": 0.0, "seat_info": "",
            "off_n": 0, "test": None, "held": False, "tests": 0, "test_fails": 0, "lift_on": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PC_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _in_mouth(x, y, shrink=0.0) -> bool:
    return (x / (MOUTH_X - shrink)) ** 2 + (y / (MOUTH_Y - shrink)) ** 2 < 1.0


def _seal_state(cp, cr, lp, lr):
    """(seated as the four checks find it, wholly inside, info, the handles' gap): all in the cooker's frame.
    In line: the middle of the lid handle's end within ALIGN_OFF sideways of the body handle's centre line. Handles
    together: the gap at the body handle's end within GAP_MAX. Depth: at each depth point the lid's top face DEPTH_SEAT
    +/- DEPTH_TOL below the rim's top. Witness ring: every gasket point in under the rim and close to its underside."""
    pts = (lp + GASKET_PTS @ lr.T - cp) @ cr      # world -> cooker frame
    under = []
    for x, y, z in pts:
        under.append((not _in_mouth(x, y, -SEAT_OVERLAP)) and abs(z - Z_LIP0) <= SEAT_TOL and math.hypot(x, y) < R_IN)
    o = (lp - cp) @ cr  # the lid's top face: through its origin, square to its z axis
    nz = cr.T @ lr[:, 2]
    depths = [Z_LIP1 - (o[2] - (nz[0] * (x - o[0]) + nz[1] * (y - o[1])) / nz[2]) for x, y in DEPTH_PTS]
    depth_ok = all(abs(d - DEPTH_SEAT) <= DEPTH_TOL for d in depths)
    end_c = (lp + lr @ np.array([LH_X0, 0.0, (LH_Z0 + BAR_Z1) / 2]) - cp) @ cr
    align = abs(float(end_c[1]))
    end_l = (lp + lr @ np.array([BH_X0 + 0.006, 0.0, LH_Z0]) - cp) @ cr
    gap = float(end_l[2] - BH_Z1)
    seated = all(under) and depth_ok and align <= ALIGN_OFF and gap <= GAP_MAX
    # wholly inside: every point of the lid's outline below the rim's underside and inside the wall
    outline = []
    for k in range(16):
        p, _ = _ellipse(LID_X, LID_Y, 2 * math.pi * k / 16)
        for z in (GASKET_TOP, GASKET_BOT):
            outline.append((p[0], p[1], z))
    ol = (lp + np.array(outline) @ lr.T - cp) @ cr
    inside = bool(np.all(ol[:, 2] < Z_LIP0 + 0.0005) and np.all(np.hypot(ol[:, 0], ol[:, 1]) < R_IN + 0.001))
    info = (f"ring={sum(under)}/{N_CHECK} z-lip=[{(pts[:, 2] - Z_LIP0).min() * 1000:.1f},"
            f"{(pts[:, 2] - Z_LIP0).max() * 1000:.1f}]mm depth={np.round(np.array(depths) * 1000, 1).tolist()}mm "
            f"align={align * 1000:.1f}mm gap={gap * 1000:.1f}mm")
    return seated, inside, info, gap


def _lift(base, origin, lp, lr, on: bool, newtons: float = LIFT_N) -> None:
    """The scorer's latch-hold test: LIFT_N straight up under the lid handle's end while ``on``, else nothing."""
    import torch

    asset = base.scene["lid"]
    at = lp + lr @ LIFT_LOCAL + origin  # world, with this environment's origin
    f = torch.tensor([0.0, 0.0, newtons if on else 0.0], dtype=torch.float32, device=asset.device).view(1, 1, 3)
    pos = torch.tensor(at, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f, positions=pos, is_global=True)
    _EP["lift_on"] = on


def _latch_test(base, origin, lp, lr, e) -> None:
    """Once both hands have been off the seated, latched lid for LIFT_AFTER_S, lift the lid handle's end with LIFT_N for
    LIFT_S. It passes if the lid stays seated and latched with the hands off all the while (the handles within GAP_MAX
    is part of seated). The pass holds only while the lid stays that way; any change and the next closing is tested
    afresh."""
    closed = e["seated"] and e["latched"] and e["hands_off"]
    t = e["test"]
    if t is not None:  # the lift is on
        t["n"] += 1
        t["max_gap"] = max(t["max_gap"], e["gap"])
        f_n = LIFT_RAMP * t["n"] / 20 if LIFT_RAMP else LIFT_N
        if not closed:
            _lift(base, origin, lp, lr, False)
            e["test"], e["held"], e["off_n"] = None, False, 0
            e["test_fails"] += 1
            if _DEBUG["on"]:
                print(f"[cooker] lift test FAILED after {t['n'] / 20:.2f}s at {f_n:.2f} N (max gap "
                      f"{t['max_gap'] * 1000:.1f} mm)", file=sys.stderr, flush=True)
        elif t["n"] >= int(round((8.0 if LIFT_RAMP else LIFT_S) * 20)):
            _lift(base, origin, lp, lr, False)
            e["test"], e["held"] = None, True
            if _DEBUG["on"]:
                print(f"[cooker] lift test passed at {f_n:.2f} N (max gap {t['max_gap'] * 1000:.1f} mm)",
                      file=sys.stderr, flush=True)
        else:
            _lift(base, origin, lp, lr, True, f_n)  # (kept under the handle's end as the lid moves)
            if _DEBUG["on"] and LIFT_RAMP and t["n"] % 4 == 0:
                print(f"[cooker] lift {f_n:.2f} N gap={e['gap'] * 1000:.2f}mm loop={e['loop_deg']:.1f} "
                      f"{e['seat_info']}", file=sys.stderr, flush=True)
        return
    if not closed:
        e["off_n"], e["held"] = 0, False
        return
    e["off_n"] += 1
    if not e["held"] and e["off_n"] >= int(round(LIFT_AFTER_S * 20)):
        e["test"] = {"n": 0, "max_gap": e["gap"]}
        e["tests"] += 1
        _lift(base, origin, lp, lr, True, 0.0 if LIFT_RAMP else LIFT_N)


@on_step
def _follow(env) -> None:
    try:
        _follow_inner(env)
    except Exception as exc:  # noqa: BLE001  (never let the follower end a trial)
        if not _EP.get("err"):
            _EP["err"] = repr(exc)[:300]
            print(f"[cooker] follower error: {_EP['err']}", file=sys.stderr, flush=True)


def _follow_inner(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    e = _EP
    e["steps"] += 1
    cp, cr = _pose(base, origin, "cooker")
    lp, lr = _pose(base, origin, "lid")
    op, orr = _pose(base, origin, "loop")
    tcps = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        k = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][k]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    ct = _contacts()
    lid_n, loop_n = (0.0, 0.0) if ct is None else (float(ct[0][0]), float(ct[0][1]))
    pads = np.zeros((3, 4)) if ct is None else ct[1]
    tilt = _tilt(cr)
    e["max_tilt"] = max(e["max_tilt"], tilt)
    if tilt > TIP_DEG:
        e["tipped"] = True
    if cp[2] < FLOOR_Z or lp[2] < FLOOR_Z:
        e["fallen"] = True
    seated, inside, info, gap = _seal_state(cp, cr, lp, lr)
    e["seated"] = bool(seated and lid_n > 0.01)
    e["seat_info"] = info
    e["gap"] = gap
    if inside:
        e["inside"] = True
    # the loop's swing about its hinge, against the lid (deg; + = swung in under the body handle)
    rel = lr.T @ orr
    loop_deg = -math.degrees(math.atan2(rel[0, 2], rel[2, 2]))
    e["loop_deg"] = loop_deg
    e["latched"] = bool(LATCH_DEG[0] - 2.0 <= loop_deg <= LATCH_DEG[1] + 2.0 and loop_n > 0.01)
    hand_end = lp + lr @ np.array([LH_X0, 0.0, 0.010])
    gripped = bool(pads.max() > GRIP_N)
    e["hands_off"] = (not gripped) and all(float(np.linalg.norm(t - hand_end)) >= CLEAR and
                                           float(np.linalg.norm(t - op)) >= CLEAR for t in tcps)
    _latch_test(base, origin, lp, lr, e)
    if _DEBUG["on"] and e["steps"] % 5 == 0:
        lid_c = (lp - cp) @ cr
        grip_w = lp + lr @ GRIP_LOCAL
        print(f"[cooker] t={e['steps'] / 20:5.2f}s lid_in_cooker={np.round(lid_c * 1000, 1).tolist()}mm "
              f"lid_tilt={_tilt(lr):5.1f} rel_yaw={math.degrees(math.atan2((cr.T @ lr)[1, 0], (cr.T @ lr)[0, 0])):6.1f} "
              f"inside={e['inside']} seated={e['seated']} {info} lidN={lid_n:5.2f} loopN={loop_n:5.2f} "
              f"loop={loop_deg:5.1f} latched={e['latched']} "
              f"pads={np.round(pads.max(axis=1), 1).tolist()} off={e['hands_off']} off_n={e['off_n']} "
              f"lift={e['lift_on']} tests={e['tests']} fails={e['test_fails']} held={e['held']} "
              f"lift_maxgap={(e['test'] or {}).get('max_gap', float('nan')) * 1000:.1f}mm "
              f"grip-TCPR={np.round((grip_w - tcps[1]) * 1000, 1).tolist()} lid-vs-hand={_lid_vs_hand(base, lr):5.1f}deg "
              f"TCPL_c={np.round((tcps[0] - cp) @ cr * 1000, 1).tolist()} "
              f"leg_c={np.round((op + orr @ np.array([0.0, -(LOOP_HALF_W - WIRE_R), -PUSH_R]) - cp) @ cr * 1000, 1).tolist()} "
              f"cooker={np.round((cp - [CX, CY, TOP]) * 1000, 1).tolist()} ctilt={tilt:4.1f} "
              f"tipped={e['tipped']} fallen={e['fallen']} score={score(None)}", file=sys.stderr, flush=True)


def _lid_vs_hand(base, lr) -> float:
    """How far (deg) the lid has turned in the right hand from the planned grip."""
    robot = base.scene["right_robot"]
    k = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
    g = rotation(as_numpy(robot.data.body_link_pose_w)[0][k][3:7])
    d = (lr @ GRIP_ROT).T @ g
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(d) - 1) / 2))))


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 for good once the cooker has tipped more than 25 degrees or the cooker or the lid has fallen off the counter.
    Otherwise, at the end: 1.0 with the lid seated (in line, handles together, the four depths, the witness ring), the
    locking loop hooked in under the body handle's end, both hands off, and the latch-hold lift passed on this closing;
    0.5 with the lid seated; 0.25 if the lid was ever wholly inside the cooker below the rim; else 0.0."""
    e = _EP
    if e["tipped"] or e["fallen"]:
        return 0.0
    if e["seated"]:
        return 1.0 if (e["latched"] and e["hands_off"] and e["held"]) else 0.5
    return 0.25 if e["inside"] else 0.0


def success(objects) -> bool:
    """Full score with the cooker, the lid and the loop still: the trial ends there."""
    if score(objects) < 1.0:
        return False
    return all(objects[n].resting() for n in MOVING)


# -- scripted demo --------------------------------------------------------------------------------------------------
# The right hand takes the lid by its handle bar next to the vent tube, lifts it and carries it crosswise over the
# mouth. Slowly, pausing at each step, it tilts the lid's far end down into the mouth and under the far side of the rim,
# turns the lid in line with the body handle while lowering it inside, and sets the handle bar on the pivot. It lets go
# (the lid swings down inside) and presses the lid handle down onto the body handle, which turns the lid up about the
# pivot against the rim's underside. The left hand pushes the locking loop in under the body handle's end. The right
# lifts off: the lid stays up, held by the loop. Both arms go home.

def _rot(axis, a) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def _frame(d, c) -> np.ndarray:
    """A TCP rotation whose approach (gripper z) is d and whose fingers close along c."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = np.asarray(c, float) - d * (np.asarray(c, float) @ d)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(d, c), d])


GRIP_LOCAL = np.array([-0.032, 0.0, (BAR_Z0 + BAR_Z1) / 2])  # the right's TCP on the bar, 32 mm out from the tube
GRIP_LEAN = math.radians(float(os.environ.get("PC_LEAN", "0")))  # the right's grip leans toward the lid handle
GRIP_ROT = _frame((math.sin(GRIP_LEAN), 0.0, -math.cos(GRIP_LEAN)), (0.0, -1.0, 0.0))  # in the lid's frame: closing
# across the bar
BAR_FINGER = -0.011  # open about 22 mm round the 10 mm bar
COOKER = np.array([CX, CY, TOP])
RC = _rot((0, 0, 1), COOKER_YAW)  # the cooker's frame -> the world
LEAD_S, LEAD_STEPS = 3.0, 15
GEO_TO_LID = _rot((0, 0, 1), -math.pi / 2)  # the planning frame's lid axes -> the lid's own axes

# The lid's way in, planned offline: the lid's centre (cooker frame, mm, at its mid-plane) and its turn (deg): yaw about
# the vertical, roll about the handle bar (+ dips the far end), pitch about the lid's long axis. Yaw 0 is crosswise
# (the handle pointing right), -90 in line with the body handle.
_PLANNED = [
    (0.0, 0.0, 125.0, 0.0, 0.0, 0.0),
    (-10.0, 5.6, 125.0, 1.0, 8.2, 6.6),
    (-19.6, 5.2, 126.1, -7.2, 20.8, 15.5),
    (-4.9, 2.2, 107.4, -20.0, 33.0, -1.8),
    (8.0, -3.5, 86.3, -31.2, 39.4, -13.9),
    (7.6, -0.6, 76.2, -47.0, 29.3, -16.1),
    (7.2, 1.1, 67.8, -75.1, 17.9, -19.8),
    (4.8, 0.5, 73.6, -81.0, 13.7, -18.4),
    (0.0, 0.0, 72.1, -90.0, 0.0, -22.0),
]
LID_PATH = list(_PLANNED)  # 3.1 mm clear of the rim, wall and pivot all the way in the offline geometry check (the end hangs 3.3 mm off the wall)
PATH_S = [0.0, 3.0, 3.5, 4.0, 4.0, 3.5, 4.0, 2.5, 3.0]  # seconds to reach each key (slow: the soft arm lags)
PATH_SUB = 3  # waypoints between keys
PATH_HOLD = 0.8  # the hand pauses this long at each key so the soft arm catches up
# measured placement corrections at each key (cooker frame: mm along x, y, z and deg of yaw), aimed against
PATH_CORR = [(0.0, 0.0, 0.0, 0.0), (-2.5, -0.5, -0.4, 0.0), (-3.0, 0.5, 1.0, 0.0), (-5.4, -0.9, 1.3, 0.0),
             (-6.1, -1.2, 2.1, 0.0), (-5.6, -0.6, 0.2, 0.0), (-3.9, 0.4, 0.4, 0.0), (-3.1, 0.6, 0.2, 0.0),
             (-4.3, -0.7, 0.4, -3.0)]
PIVOT_PITCH = 22.0  # at the path's end the lid hangs this far (lid side down) about the pivot, the bar 12 mm above it


def _geo_rot(yaw, roll, pitch) -> np.ndarray:
    return _rot((0, 0, 1), math.radians(yaw)) @ _rot((0, 1, 0), math.radians(roll)) @ _rot((1, 0, 0), math.radians(pitch))


def _lid_from_geo(key) -> tuple[np.ndarray, np.ndarray]:
    """(lid origin, lid rotation) in the world for a planning key."""
    c = np.asarray(key[:3], float) / 1000.0
    rg = _geo_rot(*key[3:6])
    r = rg @ GEO_TO_LID.T
    return COOKER + RC @ (c + rg @ np.array([0.0, 0.0, 0.001])), RC @ r


def _corrected(p, r, corr) -> tuple[np.ndarray, np.ndarray]:
    """A lid pose moved by a measured correction (cooker frame: mm along x, y, z; deg of yaw about the vertical)."""
    return p + RC @ (np.asarray(corr[:3], float) / 1000.0), _rot((0, 0, 1), math.radians(corr[3])) @ r


def _tcp_for_lid(p, r, aim=True) -> tuple[np.ndarray, np.ndarray]:
    if not aim:
        return p + r @ GRIP_LOCAL, r @ GRIP_ROT
    return p + r @ GRIP_LOCAL, _rot((0, 0, 1), AIM_YAW) @ r @ GRIP_ROT


def _pivot_lid(pitch_deg, lift=0.0) -> tuple[np.ndarray, np.ndarray]:
    """The lid in line with the body handle, turned pitch_deg (lid side down) about the pivot with the bar resting on
    it, raised by lift (m)."""
    r = _rot((0, 1, 0), math.radians(pitch_deg))
    pivot = np.array([PIV_X, 0.0, PIV_TOP])
    contact_local = np.array([PIV_X, 0.0, BAR_Z0])  # the bar's underside point over the pivot, lid seated
    p = pivot - r @ contact_local + np.array([0.0, 0.0, lift])
    return COOKER + RC @ p, RC @ r


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.9, -0.3, 0.3, 0.9), (0.6, 1.4, 2.2), (0.5, 1.5), (-1.2, 0.0, 1.2),
                                  (-1.4, 0.0, 1.4), (-1.2, 0.0, 1.2)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=120)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm, q0, q1, label, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm, pos, rot, label, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _ready(arm):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return np.asarray(p, float), r


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _slerp_rot(r0, r1, f) -> np.ndarray:
    d = r0.T @ r1
    ang = math.acos(max(-1.0, min(1.0, (np.trace(d) - 1) / 2)))
    if ang < 1e-9:
        return r0
    axis = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / (2 * math.sin(ang))
    return r0 @ _rot(axis, ang * f)


AIM = np.array([0.0021, 0.0008, 0.0055])  # the right's measured placement error with the lid held, aimed against
AIM_YAW = math.radians(-2.0)  # ... and its measured yaw error (the lid hung 2 deg off)
AIM_XY = AIM * np.array([1.0, 1.0, 0.0])
LIFT_AIM = 0.0  # the right holds the lid where it seated (pulling higher tips it in the grip)
OVER_PITCH = 3.5  # ... and turns it this far past level (deg): the lid lags about 3 deg in the grip as it turns


PRESS_LOCAL = np.array([-0.153, 0.0, BAR_Z1])  # the right presses the lid handle's top here (lid frame, lid seated)
PRESS_DEPTH = 0.007  # ... aiming this far below its seated height
PRESS_LEAN = math.radians(30.0)


def _press_pose() -> tuple[np.ndarray, np.ndarray]:
    top = COOKER + RC @ (np.array([0.0, 0.0, LID_SEAT_Z]) + PRESS_LOCAL)
    d = RC @ np.array([-math.sin(PRESS_LEAN), 0.0, -math.cos(PRESS_LEAN)])  # down, leaning toward the handle's end
    rot = _frame(d, RC @ np.array([0.0, 1.0, 0.0]))  # closed fingers spread across the handle
    return top - np.array([0.0, 0.0, PRESS_DEPTH]) - 0.0128 * d, rot


def _right_plan() -> tuple[list[Waypoint], float]:
    """The right's waypoints and the time at which the lid is held up against the rim, waiting for the loop."""
    up = np.array([0.0, 0.0, 1.0])
    p0, r0 = np.asarray(LID0, float), LID0_ROT
    gp, gr = _tcp_for_lid(p0, r0, aim=False)
    out = _lead_in("right", gp + 0.06 * up, gr, "over the lid's handle bar", BAR_FINGER)
    out += [_w(1.5, gp, BAR_FINGER, "down round the bar next to the vent tube", gr),
            _w(0.8, gp, FINGER_CLOSED, "take the lid by its bar", gr),
            _w(1.2, gp + 0.05 * up, FINGER_CLOSED, "lift it off the counter", gr)]
    sp, sr = _lid_from_geo(LID_PATH[0])  # crosswise over the mouth: the lid as it lay on the counter
    tp, tr = _tcp_for_lid(sp, sr)
    hi_p, hi_r = _tcp_for_lid(np.array([p0[0], p0[1], sp[2]]), r0)
    out += [_w(1.5, hi_p + AIM, FINGER_CLOSED, "up above the rim's height", hi_r),
            _w(2.5, tp + AIM, FINGER_CLOSED, "over the mouth, crosswise, its handle toward the robot", tr)]
    for i in range(1, len(LID_PATH)):
        k0 = np.array(LID_PATH[i - 1], float) + np.r_[PATH_CORR[i - 1][:3], PATH_CORR[i - 1][3], 0.0, 0.0]
        k1 = np.array(LID_PATH[i], float) + np.r_[PATH_CORR[i][:3], PATH_CORR[i][3], 0.0, 0.0]
        for j in range(1, PATH_SUB + 1):
            p, r = _lid_from_geo(k0 + (k1 - k0) * j / PATH_SUB)
            tp, tr = _tcp_for_lid(p, r)
            out.append(_w(PATH_S[i] / PATH_SUB, tp + AIM, FINGER_CLOSED, "work the lid in under the rim", tr))
        out.append(_w(PATH_HOLD, tp + AIM, FINGER_CLOSED, "work the lid in under the rim", tr))
    # the bar onto the pivot; let go (the lid swings down inside, hanging from the bar on the pivot); then press the lid
    # handle down onto the body handle, which turns the lid up about the pivot against the rim's underside
    # the bar onto the pivot; then, keeping it there, the lid turned up about the pivot until it is against the rim's
    # underside (which brings the lid handle down onto the body handle), and held there pulled up a little
    # (once the bar rests on the pivot the post carries the lid, so the arm's sag is no longer aimed against)
    for pitch, lift, aim, t, label in ((PIVOT_PITCH, 0.012, AIM, 1.5, "the handle bar over the pivot (let the arm settle)"),
                                       (PIVOT_PITCH, -0.004, AIM, 1.5, "set the bar in the pivot's notch"),
                                       (PIVOT_PITCH * 0.6, -0.004, AIM_XY, 1.5, "turn the lid up about the pivot"),
                                       (PIVOT_PITCH * 0.25, -0.004, AIM_XY, 1.5, "... toward the rim"),
                                       (0.0, -0.003, AIM_XY, 1.2, "level"),
                                       (-OVER_PITCH, 0.0, AIM_XY, 1.2, "the lid up against the rim, handles together"),
                                       (-OVER_PITCH, LIFT_AIM, AIM_XY, 1.0, "hold it up against the rim")):
        p, r = _corrected(*_pivot_lid(pitch, lift), PATH_CORR[-1])
        tp, tr = _tcp_for_lid(p, r)
        out.append(_w(t, tp + aim, FINGER_CLOSED, label, tr))
    return out, sum(w.duration for w in out)


LEAN = math.radians(25.0)
AIM_L = np.array([0.0, 0.023, 0.0])  # the left's measured placement error here (cooker frame), aimed against
PUSH_R = 0.050  # the left pushes the loop's front leg this far down from its hinge
PUSH_DEG = (-14.0,)  # ... starting just outside it
PUSH_END_X, PUSH_END_Z = -0.267, 0.072  # the push ends with the left's TCP here (cooker frame): further shoves the lid


def _left_push_poses():
    """The left's TCP poses along its push on the loop's front leg (world), and its rotation."""
    hx, hz = _hinge_seated()
    d = np.array([math.sin(LEAN), 0.0, -math.cos(LEAN)])  # pointing down and a little forward
    rot = RC @ _frame(d, (0.0, -1.0, 0.0))  # fingers closed, spread across the handles: a 35 mm wide pusher
    face = RC @ np.array([math.cos(LEAN), 0.0, math.sin(LEAN)])  # the closed fingers' face toward the loop, 12 mm from
    # the TCP's line
    y = -(LOOP_HALF_W - 0.0035) - 0.0175  # the pusher's inner edge 3.5 mm in over the front leg, clear of the handles
    out = []
    for deg in PUSH_DEG:
        th = math.radians(deg)
        leg = np.array([hx + PUSH_R * math.sin(th), y, hz - PUSH_R * math.cos(th)])
        out.append(COOKER + RC @ (leg + AIM_L) - (0.012 + WIRE_R) * face - 0.008 * (RC @ d))
    s_c = RC.T @ (out[0] - COOKER)
    end = COOKER + RC @ np.array([PUSH_END_X, s_c[1], PUSH_END_Z])  # measured: the loop is fully in with the TCP here
    return [out[0], end], rot




def _release_bar(p, r, label) -> list[Waypoint]:
    p = np.asarray(p, float)
    up = np.array([0.0, 0.0, 0.028])
    out = [_w(0.8, p, BAR_FINGER, label, r), _w(1.0, p + up, BAR_FINGER, "up off the lid", r)]
    return out + _go_home("right", p + up, r, BAR_FINGER)


def _two_hand_plans() -> dict:
    right, t_hold = _right_plan()
    ready_p, ready_r = _ready("left")
    (s, e), rot = _left_push_poses()
    pre = s + np.array([-0.03, 0.0, 0.05])
    left = [_w(t_hold - LEAD_S, ready_p, FINGER_CLOSED, "wait at the start", ready_r)]
    left += _lead_in("left", pre, rot, "over the handles' end", FINGER_CLOSED)
    left += [_w(1.5, s, FINGER_CLOSED, "down beside the loop", rot),
             _w(2.0, e, FINGER_CLOSED, "push the loop in under the body handle's end", rot),
             _w(0.3, e, FINGER_CLOSED, "hold", rot),
             _w(1.0, s, FINGER_CLOSED, "back out", rot),
             _w(1.0, pre, FINGER_CLOSED, "up", rot)]
    left += _go_home("left", pre, rot, FINGER_CLOSED)
    t_push_done = t_hold + 1.5 + 2.0 + 0.3 + 2.0  # (the right keeps holding while the left backs out)
    hold_p, hold_r = right[-1].pos, right[-1].yaw
    right.append(_w(t_push_done - sum(w.duration for w in right) + 0.5, hold_p, FINGER_CLOSED,
                    "hold the handles together while the loop goes in", hold_r))
    right += _release_bar(hold_p, hold_r, "let go of the bar: the loop holds the lid up")
    total = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    right.append(_w(total - sum(w.duration for w in right), right[-1].pos, BAR_FINGER, "hold still", right[-1].yaw))
    left.append(_w(total - sum(w.duration for w in left), left[-1].pos, FINGER_CLOSED, "hold still", left[-1].yaw))
    return {"left": left, "right": right}


def _one_hand_plans() -> dict:
    """The one-hand check (PC_ONEHAND=1): the right does its part exactly as in the demo and lets go; the left never
    moves."""
    right, t_hold = _right_plan()
    hold_p, hold_r = right[-1].pos, right[-1].yaw
    right.append(_w(4.0, hold_p, FINGER_CLOSED, "hold", hold_r))
    right += _release_bar(hold_p, hold_r, "let go of the bar")
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, BAR_FINGER, "hold still", right[-1].yaw))
    ready_p, ready_r = _ready("left")
    return {"left": [_w(total, ready_p, FINGER_CLOSED, "stay", ready_r)], "right": right}


def _inside_plans() -> dict:
    """Debug (PC_START_INSIDE=1): the lid starts seated and swings down; the right presses the lid handle down and the
    left pushes the loop in; then both go home."""
    up = np.array([0.0, 0.0, 1.0])
    hp, hr = _press_pose()
    right = _lead_in("right", hp + 0.03 * up, hr, "over the lid handle", FINGER_CLOSED)
    right += [_w(1.5, hp, FINGER_CLOSED, "press the lid handle down onto the body handle", hr),
              _w(0.8, hp, FINGER_CLOSED, "hold it down", hr)]
    t_hold = sum(w.duration for w in right)
    ready_p, ready_r = _ready("left")
    (s0, e), rot = _left_push_poses()
    pre = s0 + np.array([-0.03, 0.0, 0.05])
    left = [_w(t_hold - LEAD_S, ready_p, FINGER_CLOSED, "wait", ready_r)]
    left += _lead_in("left", pre, rot, "over the handles' end", FINGER_CLOSED)
    left += [_w(1.5, s0, FINGER_CLOSED, "down beside the loop", rot), _w(2.4, e, FINGER_CLOSED, "push", rot),
             _w(0.6, e, FINGER_CLOSED, "hold", rot),
             _w(1.0, s0, FINGER_CLOSED, "back out", rot), _w(1.0, pre, FINGER_CLOSED, "up", rot)]
    left += _go_home("left", pre, rot, FINGER_CLOSED)
    right.append(_w(t_hold + 1.5 + 2.4 + 0.6 + 0.5 - sum(w.duration for w in right), hp, FINGER_CLOSED, "hold", hr))
    right += [_w(1.2, hp + 0.03 * up, FINGER_CLOSED, "lift off", hr)]
    right += _go_home("right", hp + 0.03 * up, hr, FINGER_CLOSED)
    total = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    right.append(_w(total - sum(w.duration for w in right), right[-1].pos, FINGER_CLOSED, "hold still", right[-1].yaw))
    left.append(_w(total - sum(w.duration for w in left), left[-1].pos, FINGER_CLOSED, "hold still", left[-1].yaw))
    return {"left": left, "right": right}


def demo():
    if START_INSIDE:
        return _inside_plans()
    return _one_hand_plans() if os.environ.get("PC_ONEHAND") else _two_hand_plans()
