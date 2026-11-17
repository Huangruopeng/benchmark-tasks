"""A HIWIN 16 x 10 mm rolled ball screw, cut to 400 mm with its end journals machined, lies loose in two V-blocks on a
granite surface plate, its floating-side journal (10 mm across, 11 mm long) overhanging the near V-block toward the
left. Its ball nut (HIWIN R16-10K3-FSCDIN) lies beside it in an open box, still on the plastic assembly tube it ships
on: the tube keeps the nut's balls in until the nut is on the screw.

Task (the instruction gives the goal only): move the ball nut onto the screw, fully onto the thread, without losing its
balls, and put the tube back in the box. The maker's way: slide the tube's bore over the screw's end journal until the
tube butts against the thread start, slide the nut along the tube to the thread start and screw it on with a little
pressure and a rotating movement; only once it is fully on the thread does the tube come off.

What the physics asks for: the screw rolls in its V-blocks with about 0.01 N m and slides along them with about 1.3 N,
while the nut's wipers and grease drag about 0.04 N m against the screw: turned with nothing holding the screw, the
nut takes the screw round with it and never threads on. While any of the nut's ball track is between the tube and the
thread, the tube must stay butted (gap under one ball, 3.175 mm) and in line (within half a ball): otherwise the balls
drop out (score 0).

Run it:  inspect-robots run --embodiment yam_arena -E task=move_a_ball_nut_from_its_shipping_arbor_onto_a_ball_screw.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_hinge_drive, set_slide_target, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the granite surface plate (12 x 18 x 3 in), its long side along y ---------------------------------------------
PX0, PX1, PY0, PY1, PT = 0.215, 0.520, -0.360, 0.097, 0.0762
PZ = TOP + PT  # the plate's top
# -- the V-blocks: hardened steel, 89 mm tall, a 90-degree V 54 mm wide at the top ----------------------------------
VB_H, VB_V, VB_X, VB_L = 0.0889, 0.054, 0.124, 0.070  # height, V opening, across (x), along the screw (y)
VB_A, VB_B = (-0.140, -0.070), (-0.255, -0.185)  # their y spans
# -- the ball screw: HIWIN rolled, 16 x 10 mm (outer 14.7 mm, core 12.5 mm), 400 mm long with machined ends -------------
SX = 0.360  # its axis, x
SR = 0.00735  # thread outer radius
SZ = PZ + VB_H - VB_V / 2 + SR * math.sqrt(2.0)  # its axis height, resting in the V-blocks
Y_SH = -0.010  # the thread start (shoulder), y; the screw's origin is its centre
JR, JL = 0.005, 0.011  # the floating-side (BF12) journal: 10 mm across, 11 mm long
THREAD_L = 0.336
FIX_L1, FIX_R1, FIX_L2, FIX_R2 = 0.036, 0.006, 0.017, 0.005  # the fixed side (BK12): 12 mm bearing seat, 10 mm spigot
SCREW_MASS = 0.43
LEAD = 0.010  # 10 mm per turn
MARK_N = 0.050  # the pen line on the thread: 50 mm from the thread start (the nut's length plus 5 mm)
# -- the ball nut: HIWIN R16-10K3-FSCDIN, flange 48 x 10 mm, body 28 mm, 45 mm long, 0.20 kg; flange end leading ---
NUT_L, NUT_FL, NUT_FR, NUT_BR = 0.045, 0.010, 0.024, 0.014
NUT_MASS = 0.20  # the whole nut: its shell plus the hidden thread carrier inside it
CARRIER_MASS = 0.06
BALL_D = 0.003175  # the nut's balls (1/8 in)
# -- the assembly tube (arbor): 12.3 mm outside (0.2 mm under the core), 10.5 mm bore, 55 mm long with a rear collar ---
A_RO, A_RI, A_L, A_COL_R, A_COL_L = 0.00615, 0.00525, 0.055, 0.010, 0.003
A_CHAMFER = 0.0009  # the bore's mouth is chamfered 45 degrees through the wall
ARBOR_MASS = 0.010
S_START = -(A_L - A_COL_L - NUT_L)  # the nut's leading face sits this far (7 mm) behind the tube's end, on its collar
# -- the open box the nut ships in, on the plate in front of the screw's end ----------------------------------------
BXN, BYN = 0.470, -0.006  # the tube's leading end as it lies in the box (x, y)
NZB = PZ + 0.040  # the nut's axis as it lies in the box
BOX_W, BOX_Y0, BOX_Y1, BOX_H = 0.090, BYN - 0.012, BYN + 0.070, 0.020
# -- the lint-free cloth under the screw's end (it catches any ball) ---------------------------------------------------
CLOTH = (0.300, 0.420, -0.065, 0.050, 0.002)
# -- the rules ----------------------------------------------------------------------------------------------------------
GAP_LOSS = BALL_D  # tube's end this far from the thread start with the nut between them: the balls drop into the gap
OFF_LOSS = BALL_D / 2  # or its axis this far off the screw's at its end
TILT_LOSS = 5.0  # or tilted this much (deg)
S_RUNOFF = 0.004  # the nut's leading face this far past the tube's end with no thread there: its first balls are out
CATCH_N = -0.0005  # the nut's leading face this close to the thread start (tube butted): the thread start stops it
T_DRAG, SPIN_K, SPIN_SLIP = 0.04, 0.02, 2.0  # nut-on-screw drag (N m); its stick-slip spring (N m/deg), slip (deg)
NA_K, NA_D, NA_MAX, NA_SLIP = 2000.0, 2.0, 1.0, 0.001  # the wipers' drag on the tube: about 1 N
SLIDE_K, SLIDE_D, SLIDE_MAX = 8000.0, 20.0, 60.0  # the thread: the nut held where its turns put it
ON_N = NUT_L + 0.001  # the nut's trailing face past the thread start: fully on the thread, off the tube
ENGAGED_N = 0.010  # one full turn onto the thread
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

GRANITE, GRANITE_S = (0.11, 0.11, 0.12), (0.30, 0.30, 0.32)
STEEL, STEEL_D, STEEL_B = (0.70, 0.72, 0.75), (0.38, 0.39, 0.42), (0.86, 0.87, 0.89)
VB_C, VB_D = (0.62, 0.64, 0.67), (0.30, 0.31, 0.33)
BLACK, RED = (0.06, 0.06, 0.07), (0.85, 0.10, 0.08)
TUBE_C, TUBE_D = (0.92, 0.93, 0.90), (0.55, 0.57, 0.55)
CARD, CARD_D, FOAM = (0.66, 0.52, 0.34), (0.50, 0.38, 0.24), (0.32, 0.33, 0.36)
CLOTH_C = (0.96, 0.96, 0.95)


def _qy(a: float) -> tuple:
    """The rotation about y by ``a`` (xyzw)."""
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _q_of(m: np.ndarray) -> tuple:
    from yam_arena import ik

    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.asarray(m, float)))


def _plate_shape(p1, p2, width: float, thickness: float, normal, color, collide: bool = True) -> BoxShape:
    """bodykit.plate with a robust matrix-to-quaternion conversion (bodykit's mis-turns some normals near -x)."""
    a, b = np.asarray(p1, float), np.asarray(p2, float)
    d = b - a
    length = float(np.linalg.norm(d))
    d /= length
    n = np.asarray(normal, float) - np.dot(normal, d) * d
    n /= np.linalg.norm(n)
    t = np.cross(d, n)
    return BoxShape(tuple(((a + b) / 2).tolist()), (thickness, width, length), color, collide=collide,
                    quat_xyzw=_q_of(np.column_stack([n, t, d])))


# -- the bodies -----------------------------------------------------------------------------------------------------

def _plate() -> Body:
    """The granite surface plate, with the cloth on it and the V-blocks standing on it (all fixed)."""
    cx, cy = (PX0 + PX1) / 2, (PY0 + PY1) / 2
    w, d = PX1 - PX0, PY1 - PY0
    shapes = [BoxShape((0.0, 0.0, -PT / 2), (w, d, PT), GRANITE)]
    rng = np.random.default_rng(7)
    for _ in range(90):  # the granite's grain
        shapes.append(BoxShape((rng.uniform(-w / 2 + 0.005, w / 2 - 0.005), rng.uniform(-d / 2 + 0.005, d / 2 - 0.005),
                                0.0002), (0.003, 0.003, 0.0004), GRANITE_S, collide=False, yaw=rng.uniform(0, 3)))
    x0, x1, y0, y1, ct = CLOTH
    shapes.append(BoxShape(((x0 + x1) / 2 - cx, (y0 + y1) / 2 - cy, ct / 2), (x1 - x0, y1 - y0, ct), CLOTH_C))
    for (ya, yb) in (VB_A, VB_B):  # each V-block: a body, two cheeks either side of the V, and the V's two faces
        yc, ln = (ya + yb) / 2 - cy, yb - ya
        h0 = VB_H - VB_V / 2
        shapes.append(BoxShape((SX - cx, yc, h0 / 2), (VB_X, ln, h0), VB_C))
        for s in (-1.0, 1.0):
            xs = SX - cx + s * (VB_V / 2 + (VB_X - VB_V) / 4)
            shapes.append(BoxShape((xs, yc, (h0 + VB_H) / 2), ((VB_X - VB_V) / 2, ln, VB_H - h0), VB_C))
            p1 = np.array([SX - cx, yc, h0])
            p2 = np.array([SX - cx + s * VB_V / 2, yc, VB_H])
            nrm = np.array([-s, 0.0, 1.0]) / math.sqrt(2.0)
            shapes.append(_plate_shape(p1 - 0.003 * nrm, p2 - 0.003 * nrm, ln, 0.006, nrm, VB_C))
            # the dark ground V faces and the clamp slot on each cheek (pictures)
            shapes.append(_plate_shape(p1 + 0.0002 * nrm, p2 + 0.0002 * nrm, ln - 0.002, 0.0004, nrm, VB_D,
                                        collide=False))
            shapes.append(BoxShape((xs + s * (VB_X - VB_V) / 4 + s * 0.0002, yc, VB_H - 0.015), (0.0004, 0.012, 0.012),
                                   VB_D, collide=False))
    return Body.build("plate", pos=(cx, cy, PZ), mass=30.0, kinematic=True, color=GRANITE, shapes=shapes,
                      friction=(0.15, 0.12), contact_offset=0.001)


def _box() -> Body:
    """The nut's open cardboard box with a grey foam insert: ridges under the flange, the tube's end and its collar
    hold the nut level with the body free. Its end toward the arms is folded down. Origin: on the floor's middle."""
    cy = (BOX_Y0 + BOX_Y1) / 2
    ln, t = BOX_Y1 - BOX_Y0, 0.002
    shapes = [BoxShape((0.0, 0.0, t / 2), (BOX_W, ln, t), CARD)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (BOX_W / 2 - t / 2), 0.0, BOX_H / 2), (t, ln, BOX_H), CARD))
        shapes.append(BoxShape((s * (BOX_W / 2 + 0.012), 0.0, 0.0008), (0.024, ln, 0.0016), CARD_D))  # opened flaps
    shapes.append(BoxShape((0.0, -ln / 2 + t / 2, BOX_H / 2), (BOX_W, t, BOX_H), CARD))  # the far (-y) end
    shapes.append(BoxShape((0.0, ln / 2 - t / 2, 0.004), (BOX_W, t, 0.008), CARD))  # the near end, folded down
    shapes.append(BoxShape((0.0, ln / 2 + 0.010, 0.0008), (BOX_W, 0.020, 0.0016), CARD_D))
    zb = NZB - PZ  # the nut's axis above the box floor
    rel = lambda y: y - cy  # noqa: E731
    for (y0, y1, r, dx) in ((BYN + 0.0005, BYN + 0.0050, A_RO, 0.004),  # under the tube's end
                            (BYN + 0.0085, BYN + 0.0155, NUT_FR, 0.012),  # under the flange
                            (BYN + 0.0525, BYN + 0.0545, A_COL_R, 0.004)):  # under the collar
        top = zb - math.sqrt(r * r - dx * dx)
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((s * (dx + 0.002), rel((y0 + y1) / 2), (t + top) / 2), (0.004, y1 - y0, top - t),
                                   FOAM))
    shapes.append(BoxShape((0.0, rel(BYN + 0.034), t + 0.002), (BOX_W - 0.006, 0.030, 0.004), FOAM))
    return Body.build("box", pos=(BXN, cy, PZ), mass=0.05, kinematic=True, color=CARD, shapes=shapes,
                      friction=(0.8, 0.7), contact_offset=0.001)


def _screw() -> Body:
    """The ball screw, axis along y: its thread (rings mark the 10 mm lead), the floating-side journal toward +y with
    its chamfered tip and snap-ring groove, the fixed-side bearing seat and spigot toward -y, and a red pen line on the
    thread 50 mm from its start. Origin: the thread start (shoulder) on the axis."""
    shapes = [CylShape((0.0, -THREAD_L / 2, 0.0), SR, THREAD_L, STEEL, axis="Y")]
    k = 0
    while 0.004 + k * LEAD < THREAD_L - 0.004:
        shapes.append(CylShape((0.0, -0.004 - k * LEAD, 0.0), SR + 0.00008, 0.0022, STEEL_D, collide=False, axis="Y"))
        shapes.append(CylShape((0.0, -0.009 - k * LEAD, 0.0), SR + 0.00008, 0.0008, STEEL_D, collide=False, axis="Y"))
        k += 1
    shapes.append(CylShape((0.0, -MARK_N, 0.0), SR + 0.00015, 0.0016, RED, collide=False, axis="Y"))
    jl = JL - 0.0008
    shapes.append(CylShape((0.0, jl / 2, 0.0), JR, jl, STEEL_B, axis="Y"))  # the journal
    shapes.append(CylShape((0.0, jl + 0.0002, 0.0), JR - 0.0004, 0.0004, STEEL_B, axis="Y"))  # its chamfered tip
    shapes.append(CylShape((0.0, jl + 0.0006, 0.0), JR - 0.0008, 0.0004, STEEL_B, axis="Y"))
    shapes.append(CylShape((0.0, 0.00915 + 0.0006, 0.0), JR + 0.00005, 0.0011, STEEL_D, collide=False, axis="Y"))
    y = -THREAD_L
    shapes.append(CylShape((0.0, y - FIX_L1 / 2, 0.0), FIX_R1, FIX_L1, STEEL_B, axis="Y"))  # bearing seat, M12 thread
    for j in range(12):
        shapes.append(CylShape((0.0, y - FIX_L1 + 0.001 + j * 0.001, 0.0), FIX_R1 + 0.00005, 0.0004, STEEL_D,
                               collide=False, axis="Y"))
    shapes.append(CylShape((0.0, y - FIX_L1 - FIX_L2 / 2, 0.0), FIX_R2, FIX_L2, STEEL_B, axis="Y"))  # drive spigot
    return Body.build("screw", pos=(SX, Y_SH, SZ), mass=SCREW_MASS, color=STEEL, shapes=shapes, friction=(0.15, 0.12),
                      com=(0.0, -0.184, 0.0), solver_iterations=48, contact_offset=0.001, damping=(0.0, 0.02))


def _stave(phi: float, r0: float, r1: float, y0: float, y1: float, color) -> BoxShape:
    """One stave of a tube along y between radii r0..r1 and y0..y1, at angle phi about y (in the x-z plane)."""
    rm = (r0 + r1) / 2
    w = 2 * r1 * math.tan(math.pi / 12) + 0.0002
    return BoxShape((rm * math.cos(phi), (y0 + y1) / 2, rm * math.sin(phi)), (r1 - r0, y1 - y0, w), color,
                    quat_xyzw=_qy(-phi))


def _arbor() -> Body:
    """The assembly tube: a white plastic tube with a 10.5 mm bore (collision: twelve staves, so the journal can go
    in), its bore's mouth chamfered, a collar at its rear end. Origin: the leading end's centre; it runs along +y."""
    shapes = []
    for k in range(12):
        phi = k * math.pi / 6
        shapes.append(_stave(phi, A_RI, A_RO, A_CHAMFER, A_L - A_COL_L, None))
        # the chamfer: a slanted stave whose inner face runs from the bore (A_CHAMFER deep) out to the rim (at the end)
        er = np.array([math.cos(phi), 0.0, math.sin(phi)])
        ey = np.array([0.0, 1.0, 0.0])
        n_out = (er + ey) / math.sqrt(2.0)
        s_dir = (ey - er) / math.sqrt(2.0)
        th = 0.0005
        mid = ((A_RI + A_RO) / 2) * er + (A_CHAMFER / 2) * ey + (th / 2) * n_out
        m = np.stack([n_out, s_dir, np.cross(n_out, s_dir)], axis=1)
        shapes.append(BoxShape(tuple(mid.tolist()), (th, A_CHAMFER * math.sqrt(2.0), 2 * A_RO * math.tan(math.pi / 12)),
                               None, quat_xyzw=_q_of(m)))
    yc = A_L - A_COL_L / 2
    shapes.append(CylShape((0.0, yc, 0.0), A_COL_R, A_COL_L, TUBE_C, axis="Y"))  # the collar
    tl = A_L - A_COL_L
    shapes.append(CylShape((0.0, tl / 2, 0.0), A_RO - 0.00005, tl - 0.0002, TUBE_C, collide=False, axis="Y"))
    shapes.append(CylShape((0.0, 0.0002, 0.0), A_RI + 0.0002, 0.0003, TUBE_D, collide=False, axis="Y"))  # the bore
    shapes.append(CylShape((0.0, A_L + 0.0001, 0.0), 0.004, 0.0002, TUBE_D, collide=False, axis="Y"))
    return Body.build("arbor", pos=(BXN, BYN, NZB), mass=ARBOR_MASS, color=TUBE_C, shapes=shapes, friction=(0.5, 0.4),
                      com=(0.0, 0.03, 0.0), solver_iterations=48, contact_offset=0.0005)


def _carrier() -> Body:
    """The thread's carrier: a tiny hidden body at the nut's leading face. The nut spins on it (a revolute joint with
    the thread's drag); once the thread has caught, a slide joint holds it on the screw where the nut's turns put it."""
    return Body.build("carrier", pos=(BXN, BYN - S_START, NZB), mass=CARRIER_MASS, color=STEEL,
                      shapes=[CylShape((0.0, 0.006, 0.0), 0.0065, 0.010, None, axis="Y")], ignore=("screw", "arbor"),
                      com=(0.0, 0.006, 0.0), solver_iterations=64, contact_offset=0.0005)


def _nut() -> Body:
    """The ball nut, axis along y, its flange leading (toward -y): flange 48 x 10 mm with six bolt holes on 38 mm,
    body 28 mm, black wipers in the bores at both ends. Origin: the leading (flange) face's centre."""
    shapes = [CylShape((0.0, NUT_FL / 2, 0.0), NUT_FR, NUT_FL, STEEL_B, axis="Y"),
              CylShape((0.0, NUT_FL + (NUT_L - NUT_FL) / 2, 0.0), NUT_BR, NUT_L - NUT_FL, STEEL, axis="Y")]
    for y in (-0.0002, NUT_L + 0.0002):
        shapes.append(CylShape((0.0, y, 0.0), 0.0092, 0.0004, BLACK, collide=False, axis="Y"))  # the wipers
        shapes.append(CylShape((0.0, y + (0.0001 if y < 0 else -0.0001), 0.0), 0.0072, 0.0006, (0.12, 0.12, 0.13),
                               collide=False, axis="Y"))
    for k in range(6):
        a = math.pi / 6 + k * math.pi / 3
        for y in (-0.0002, NUT_FL + 0.0002):
            shapes.append(CylShape((0.019 * math.cos(a), y, 0.019 * math.sin(a)), 0.00275, 0.0004, STEEL_D,
                                   collide=False, axis="Y"))
    shapes.append(CylShape((0.0, NUT_FL + 0.003, 0.0), NUT_BR + 0.0002, 0.0012, STEEL_D, collide=False, axis="Y"))
    shapes.append(BoxShape((0.0, NUT_FL / 2, NUT_FR - 0.0002), (0.006, 0.006, 0.0006), BLACK, collide=False))  # lube
    return Body.build("nut", pos=(BXN, BYN - S_START, NZB), mass=NUT_MASS - CARRIER_MASS, color=STEEL, shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, 0.019, 0.0), ignore=("screw", "arbor", "carrier"),
                      solver_iterations=64, contact_offset=0.001)


N_BALLS = 8
PARK = (0.36, 0.0, TOP - 0.10)  # where the spilled balls wait, out of sight under the table


def _balls() -> Body:
    """The balls a spill drops onto the plate: eight steel balls (pictures only), parked under the table."""
    rng = np.random.default_rng(3)
    shapes = []
    for _ in range(N_BALLS):
        shapes.append(SphereShape((rng.uniform(-0.025, 0.025), rng.uniform(-0.02, 0.02), BALL_D / 2), BALL_D / 2,
                                  STEEL_B, collide=False))
    shapes.append(SphereShape((0.0, 0.0, BALL_D / 2), 0.0005, None, collide=False))
    return Body.build("balls", pos=PARK, mass=0.01, kinematic=True, color=STEEL_B, shapes=shapes)


# the nut on its tube: it keeps on the tube's axis, spins freely, and slides along it against the wipers' drag (~1 N);
# the tube's collar stops it at the rear
thread("on_tube", "arbor", "nut", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(S_START, 0.120), turn_damping=0.0005, slide_stiffness=NA_K,
       slide_damping=NA_D, slide_max_force=NA_MAX, play=0.0002, tilt_deg=1.0, always_on=True)
# the nut spins on the carrier against the thread's drag (a stick-slip spring capped at T_DRAG)
hinge("spin", "carrier", "nut", anchor=(BXN, BYN - S_START, NZB), axis="Y", spring=SPIN_K, damping=0.0,
      target_deg=0.0)
# the thread: the carrier slides along the screw (switched on once the thread catches; its target follows the turns)
slider("thread", "screw", "carrier", anchor=(SX, Y_SH, SZ), axis="Y", limits=(-THREAD_L + NUT_L, 0.005),
       stiffness=SLIDE_K, damping=SLIDE_D, target=0.0, max_force=SLIDE_MAX)

_author_threads_plain = bodykit._author_threads
_author_hinges_plain = bodykit._author_hinges
_author_sliders_plain = bodykit._author_sliders


def _author_threads(stage, prim_path: str, spec) -> None:
    """bodykit's threads, then: the nut-on-tube drag starts where the nut sits (on the collar), not at the far limit."""
    _author_threads_plain(stage, prim_path, spec)
    if spec.name == "nut":
        from pxr import UsdPhysics

        UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/on_tube"), "transX").GetTargetPositionAttr().Set(
            float(S_START))


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, then: the spin joint's spring is capped at the thread's drag (Coulomb-like friction)."""
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name == "nut":
        from pxr import UsdPhysics

        UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/spin"), "angular").GetMaxForceAttr().Set(float(T_DRAG))


def _author_sliders(stage, prim_path: str, spec) -> None:
    """The thread's slide joint, authored here: its frames are the screw's thread start and the carrier's origin (the
    carrier starts in the box, far from the screw), and it starts switched off."""
    if spec.name != "carrier":
        _author_sliders_plain(stage, prim_path, spec)
        return
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/thread")
    joint.CreateBody0Rel().SetTargets([f"{parent}/screw/body"])
    joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr("Y")
    joint.CreateLowerLimitAttr(float(-THREAD_L + NUT_L))
    joint.CreateUpperLimitAttr(0.005)
    joint.CreateJointEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(SLIDE_K))
    drive.CreateDampingAttr(float(SLIDE_D))
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateMaxForceAttr(float(SLIDE_MAX))
    bodykit._SLIDER_PATHS["thread"] = f"{prim_path}/thread"
    bodykit._SLIDER_ATTRS.pop("thread", None)


bodykit._author_threads = _author_threads
bodykit._author_hinges = _author_hinges
bodykit._author_sliders = _author_sliders

OBJECTS = [_plate(), _box(), _screw(), _arbor(), _carrier(), _nut(), _balls()]


def _joint_attr(path_key: str, getter):
    from isaaclab.sim.utils.stage import get_current_stage

    return getter(get_current_stage().GetPrimAtPath(path_key))


_ATTRS: dict = {}


def _thread_attrs():
    if "thread" not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS["thread"])
        j = UsdPhysics.PrismaticJoint(prim)
        _ATTRS["thread"] = (j.GetJointEnabledAttr(), UsdPhysics.DriveAPI(prim, "linear").GetTargetPositionAttr(),
                            j.GetLowerLimitAttr(), j.GetUpperLimitAttr())
    return _ATTRS["thread"]


def _set_thread(on: bool, n: float | None = None) -> None:
    """Switch the thread on (holding the nut's leading face ``n`` m onto the thread) or off."""
    en, tgt, lo, hi = _thread_attrs()
    if n is not None:
        y = -float(n)
        tgt.Set(y)
        lo.Set(y - 0.0005)
        hi.Set(y + 0.0005)
    if bool(en.Get()) != bool(on):
        en.Set(bool(on))


def _set_on_tube(on: bool) -> None:
    bodykit.set_thread_on("on_tube", on)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"phase": "tube", "lost": None, "n_catch": 0.0, "turns": 0.0, "theta": None, "n_tgt": 0.0,
            "na_hold": S_START, "spin_tgt": 0.0, "screw_fell": False, "nut_fell": False, "spill": None,
            "tcps": None, "obs": {}, "s": {}, "steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BN_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    try:
        _set_thread(False, 0.0)
        _set_on_tube(True)
        set_slide_target("on_tube", S_START)
        set_hinge_drive("spin", target_deg=0.0)
    except KeyError:
        pass


def _grippers(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        r = rotation(g[3:7])
        out.append((g[:3] - origin + r @ np.array(TCP_OFFSET), r))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _write_pose(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = [*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]
    asset.write_root_pose_to_sim(torch.tensor([pose], dtype=torch.float32, device=asset.device))


def _turn_about_y(r0, r1) -> float:
    """How far r1 is turned against r0 about r0's y axis (rad)."""
    m = r0.T @ r1
    return math.atan2(float(m[0, 2]), float(m[0, 0]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _surface_below(p) -> float:
    """The height of whatever lies under the point p (the cloth, the plate or the bench)."""
    x0, x1, y0, y1, ct = CLOTH
    if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
        return PZ + ct
    if PX0 <= p[0] <= PX1 and PY0 <= p[1] <= PY1:
        return PZ
    return TOP


def _lose(why: str, where) -> None:
    if _EP["lost"] is None:
        _EP["lost"] = why
        _EP["spill"] = {"from": np.asarray(where, float).copy(), "k": 0}
        _set_on_tube(False)
        _set_thread(False)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    ps, rs = _pose(base, origin, "screw")
    pn, rn = _pose(base, origin, "nut")
    pa, ra = _pose(base, origin, "arbor")
    pc, rc = _pose(base, origin, "carrier")
    u = rs @ np.array([0.0, -1.0, 0.0])  # into the screw, from its free end
    n = float((pn - ps) @ u)  # the nut's leading face, onto the thread from its start (m)
    ua = ra @ np.array([0.0, -1.0, 0.0])
    a_rel = pa - ps
    gap = -float(a_rel @ u)  # the tube's end short of the thread start (m)
    a_off = float(np.linalg.norm(a_rel + gap * u))  # its end off the screw's axis
    a_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ua @ u)))))
    s = float((pn - pa) @ ua)  # the nut's leading face past the tube's end, along the tube
    lined = a_off <= OFF_LOSS and a_tilt <= TILT_LOSS
    butted = gap <= GAP_LOSS and lined
    # -- the stick-slip drags: the nut on the tube (its wipers), the nut on the carrier (the thread) -------------------
    if _EP["phase"] in ("tube", "thread") and _EP["lost"] is None and abs(s - _EP["na_hold"]) > NA_SLIP:
        _EP["na_hold"] = s
        set_slide_target("on_tube", s)
    spin = math.degrees(_turn_about_y(rc, rn))
    if abs(_wrap(math.radians(spin - _EP["spin_tgt"]))) > math.radians(SPIN_SLIP):
        _EP["spin_tgt"] = spin
        set_hinge_drive("spin", target_deg=spin)
    # -- the thread ---------------------------------------------------------------------------------------------------
    theta = _turn_about_y(rs, rn)
    if _EP["lost"] is None:
        if _EP["phase"] == "tube":
            if butted and n >= CATCH_N:  # the nut has slid up to the thread start: it stops there and can screw on
                _EP.update(phase="thread", n_catch=n, turns=0.0, theta=theta, n_tgt=n)
                _set_thread(True, n)
            elif s > S_RUNOFF:  # off the tube's end with no thread there
                _lose("the nut ran off the tube's end", pa)
        else:
            d = -_wrap(theta - _EP["theta"]) if _EP["theta"] is not None else 0.0
            _EP["theta"] = theta
            _EP["turns"] += d / (2 * math.pi)
            tgt = _EP["n_catch"] + LEAD * _EP["turns"]
            if tgt < _EP["n_catch"] - 0.002:  # screwed back off the thread start, onto the tube
                _set_thread(False)
                _EP.update(phase="tube", theta=None)
                _set_on_tube(True)
            else:
                if abs(tgt - _EP["n_tgt"]) > 0.00005:
                    _EP["n_tgt"] = tgt
                    _set_thread(True, tgt)
                if _EP["phase"] == "thread" and n >= ON_N:  # fully on the thread: off the tube
                    _EP["phase"] = "on"
                    _set_on_tube(False)
                elif _EP["phase"] == "on" and n < ON_N - 0.001:  # back over the joint: onto the tube again
                    if butted:
                        _EP["phase"] = "thread"
                        _set_on_tube(True)
                    else:
                        _lose("the nut was screwed back off the thread with no tube butted", ps + u * 0.0)
                if _EP["phase"] == "thread" and not butted:  # balls between the tube and the thread: they drop out
                    _lose(f"the tube parted from the screw (gap {gap * 1000:.1f} mm, off {a_off * 1000:.1f} mm, "
                          f"tilt {a_tilt:.1f} deg)", ps - u * max(gap, 0.0) / 2)
    # -- drops ------------------------------------------------------------------------------------------------------------
    s_ends = [ps + rs @ np.array([0.0, 0.0, 0.0]), ps + rs @ np.array([0.0, -THREAD_L - FIX_L1, 0.0])]
    if any(e[2] < _surface_below(e) + 0.012 for e in s_ends):
        _EP["screw_fell"] = True  # off its V-blocks onto the plate or the bench
    in_box = abs(pn[0] - BXN) < BOX_W / 2 and BOX_Y0 < pn[1] < BOX_Y1 + 0.01
    if not in_box and pn[2] < _surface_below(pn) + 0.020:
        _EP["nut_fell"] = True
    # -- the spill: the balls fall onto what is below -------------------------------------------------------------------
    sp = _EP["spill"]
    if sp is not None and sp["k"] <= 8:
        f = sp["from"]
        z0 = _surface_below(f)
        t = sp["k"] / 8.0
        p = np.array([f[0], f[1], f[2] + (z0 - f[2]) * t * t])
        _write_pose(base, origin, "balls", p)
        sp["k"] += 1
    # -- the result's parts ---------------------------------------------------------------------------------------------
    off_screw = gap > JL + 0.002 or a_off > 0.004  # the tube's bore is off the journal
    arbor_in_box = (abs(pa[0] - BXN) < BOX_W / 2 and BOX_Y0 - 0.005 < pa[1] < BOX_Y1 and pa[2] < NZB + 0.020
                    and off_screw)
    in_vblocks = all(abs((ps + rs @ np.array([0.0, yy - Y_SH, 0.0]))[2] - SZ) < 0.004 and
                     abs((ps + rs @ np.array([0.0, yy - Y_SH, 0.0]))[0] - SX) < 0.006
                     for yy in (sum(VB_A) / 2, sum(VB_B) / 2))
    _EP["tcps"] = [g[0] for g in _grippers(base, origin)]
    grips = _grippers(base, origin)
    _EP["obs"] = {"screw": (ps, rs), "nut": (pn, rn), "arbor": (pa, ra), "left": grips[0], "right": grips[1],
                  "n": n, "gap": gap}
    _EP["s"] = {"phase": _EP["phase"], "n": n, "arbor_in_box": arbor_in_box, "in_vblocks": in_vblocks}
    if _DEBUG["on"] and (_EP["steps"] % (1 if os.environ.get("BN_FAST") else 5) == 0):
        mm = lambda v: np.round(np.asarray(v) * 1000).astype(int).tolist()  # noqa: E731
        fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
              for a in ("left_robot", "right_robot")]
        print(f"[bn] t={_EP['steps'] / 20:6.2f} ph={_EP['phase']} lost={_EP['lost']} n={n * 1000:6.1f}"
              f" tgt={_EP['n_tgt'] * 1000:6.1f} turns={_EP['turns']:5.2f} s={s * 1000:6.1f} gap={gap * 1000:6.1f}"
              f" aoff={a_off * 1000:4.1f} atilt={a_tilt:4.1f} screw={mm(ps)} rollS={math.degrees(_turn_about_y(np.eye(3), rs)):7.1f}"
              f" nut={mm(pn)} arbor={mm(pa)} L={mm(grips[0][0])} R={mm(grips[1][0])}"
              f" fing={[round(v, 1) for v in fj]} fell={_EP['screw_fell']},{_EP['nut_fell']}"
              f" inbox={arbor_in_box} vb={in_vblocks} score={score(None)}"
              f" spin={spin:7.1f} spin_tgt={_EP['spin_tgt']:7.1f} car_vs_screw={math.degrees(_turn_about_y(rs, rc)):7.1f}"
              f" thr_on={_thread_attrs()[0].Get()}", file=sys.stderr, flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if the nut's balls were lost (latched) or the screw or the nut fell. Otherwise, at the end: 1.0: the nut fully
    on the thread up to the pen line (its leading end 50 mm or more onto it), the tube off the screw and back in its
    box, the screw lying in both V-blocks and both arms back where they started; 0.75: the nut up to the pen line;
    0.5: the nut screwed one full turn (10 mm) onto the thread; 0.25: the nut at the thread start with the tube butted
    against the screw's end; else 0.0."""
    s = _EP["s"]
    if not s or _EP["lost"] is not None or _EP["screw_fell"] or _EP["nut_fell"]:
        return 0.0
    if s["phase"] == "tube":
        return 0.0
    n = s["n"]
    if n >= MARK_N:
        if s["arbor_in_box"] and s["in_vblocks"] and _arms_home():
            return 1.0
        return 0.75
    if n >= ENGAGED_N:
        return 0.5
    return 0.25


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
    """Full score with the screw, the nut and the tube at rest: the trial ends once that has held for 1 s."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("screw", "nut", "arbor"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand holds the screw by its thread beyond the far V-block. The left takes the nut by its body, from behind
# (gripper pointing along the screw), lifts it out of its box and brings the tube's bore in line with the screw's
# journal, measuring where the tube's end is and correcting; it pushes the tube over the journal until it butts, then on
# until the nut has slid along the tube to the thread start. It screws the nut on, turning its wrist 220 degrees a
# stroke and regripping, following the nut along the screw, until the nut's leading end is past the pen line. It then
# takes the tube by its rear end, draws it off the journal and lays it in the box. Both arms go home.

LEAD_S, LEAD_STEPS = 3.0, 15
GRIP_IN = 0.004  # the left TCP this far in from the nut body's trailing face
BODY_OPEN, BODY_HOLD = -0.020, FINGER_CLOSED
TUBE_OPEN, TUBE_HOLD = -0.012, -0.0035  # a gentle grip on the 12.3 mm plastic tube
TUBE_GRIP = 0.014  # the left TCP this far ahead of the tube's collar
PICK_ROLL = 90.0  # the nut is taken from its box with the fingers either side of its body
ROLL0, ROLL1 = -20.0, 200.0
ADV_EST = 0.0058  # measured: a 220-degree stroke screws the nut on about 5.8 mm (the wrist lags its last degrees)
DONE_N = MARK_N + 0.001  # once the nut is this far on, the remaining strokes are skipped  # each stroke turns the wrist this far (deg): 220 degrees, 6.1 mm of thread
STROKE_S, RELEASE_S, BACK_S, REGRIP_S = 2.2, 0.35, 1.2, 0.45
PRE_GAP = 0.015  # the tube's end this far out from the journal's tip before it goes on
PUSH_PAST = 0.012  # the arm lags under the wipers' drag: aim this far past the thread start
R_HOLD_Y = -0.300  # the right hand holds the screw's thread here, beyond the far V-block
R_FINGER = -0.014


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def point_left(roll_deg: float) -> np.ndarray:
    """The left grip pointing -y (along the screw, into it), turned ``roll_deg`` about that: + screws the nut on.
    Roll 0 closes the fingers up and down, 90 sideways."""
    return _rx(-math.pi / 2) @ grip_rotation(0.0) @ _rz(math.radians(roll_deg))


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.6, 0.0, 0.6), (1.2, 1.8), (0.8, 1.4), (-1.2, -0.4, 0.4), (-0.6, 0.0, 0.6),
                                  (-1.2, 0.0, 1.2)):
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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


class _Latch:
    """A value worked out from the simulation's state the first time the demo needs it (scripted demos may read the
    simulator's state), with a fallback for when there is none (the offline check)."""

    def __init__(self, fn, fallback):
        self._fn, self._fallback, self._v = fn, fallback, None

    def get(self):
        if self._v is None:
            v = None
            if _EP.get("obs"):
                try:
                    v = self._fn(_EP["obs"])
                except Exception as e:  # noqa: BLE001
                    print(f"[bn] demo latch failed: {e!r}", file=sys.stderr, flush=True)
            if v is None and _DEBUG["on"]:
                print("[bn] demo latch: no simulator state, using the nominal target", file=sys.stderr, flush=True)
            self._v = v if v is not None else self._fallback()
        return self._v


class _LiveWaypoint:
    """A waypoint whose target (position, rotation and, optionally, finger) comes from a latch, read when the arm
    starts toward it."""

    def __init__(self, duration, fn, finger, label):
        self.duration, self._fn, self._finger, self.label = float(duration), fn, finger, label
        self._v = None

    def _get(self):
        if self._v is None:
            v = self._fn()
            f = v[2] if len(v) > 2 else self._finger
            self._v = (tuple(float(x) for x in np.asarray(v[0], float)), np.asarray(v[1], float), float(f))
        return self._v

    @property
    def finger(self):
        return self._get()[2]

    @property
    def pos(self):
        return self._get()[0]

    @property
    def yaw(self):
        return self._get()[1]


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _f, r0 = cur[arm]
        p1 = np.asarray(pos, float)
        r1 = r0 if rot is None else rot
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r1))
        cur[arm] = (p1, finger, r1)

    def live(arm, t, fn, finger, label):
        plan[arm].append(_LiveWaypoint(t, fn, finger, label))
        cur[arm] = (None, finger, None)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        last = plan[arm][-1]
        if isinstance(last, _LiveWaypoint):  # hold wherever that one turns out to be
            plan[arm].append(_LiveWaypoint(t, lambda w=last: (w.pos, w.yaw, w.finger), last._finger, label))
        else:
            plan[arm].append(Waypoint(t, last.pos, last.finger, label, last.yaw))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, finger, label, rot):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def to_home(arm, pos, rot, finger, label="back to where it started"):
        plan[arm] += _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), label, finger)

    L, R = "left", "right"
    up = np.array([0.0, 0.0, 1.0])
    ey = np.array([0.0, 1.0, 0.0])

    # -- the right hand holds the screw by its thread beyond the far V-block ----------------------------------------
    r_rot = grip_rotation(math.pi / 2)  # straight down, closing along x (across the screw)
    r_hold = np.array([SX, R_HOLD_Y, SZ])
    from_home(R, r_hold + 0.025 * up, R_FINGER, "over the screw beyond the far V-block", r_rot)
    go(R, 1.0, r_hold, R_FINGER, "down round the screw")
    go(R, 0.6, r_hold, FINGER_CLOSED, "hold the screw: it must not roll or slide")

    # -- the left takes the nut by its body, from behind, and lifts it out of the box -------------------------------
    rot0 = point_left(PICK_ROLL)
    body_tcp = np.array([BXN, BYN - S_START + NUT_L - GRIP_IN, NZB])
    from_home(L, body_tcp + 0.045 * ey + 0.02 * up, BODY_OPEN, "behind the nut in its box", rot0)
    go(L, 0.8, body_tcp + 0.045 * ey, BODY_OPEN, "down behind the nut")
    go(L, 1.0, body_tcp, BODY_OPEN, "round the nut's body")
    go(L, 0.6, body_tcp, BODY_HOLD, "take the nut")
    go(L, 0.8, body_tcp + 0.030 * up, BODY_HOLD, "lift it out of the box, level")
    # the tube's end 15 mm out from the journal's tip, on the screw's axis (nominal: the grip's own offset unknown)
    tcp_from_end = -S_START + NUT_L - GRIP_IN  # the TCP this far behind the tube's end, along the axis
    j_tip = np.array([SX, Y_SH + JL, SZ])
    pre = j_tip + (PRE_GAP + tcp_from_end) * ey
    go(L, 1.6, pre + 0.010 * up, BODY_HOLD, "carry it to the screw's end", n=2)
    go(L, 0.8, pre, BODY_HOLD, "the tube in line with the screw")
    wait(L, 0.6, "hold still: see where the tube's end is")

    def aim(obs, standoff):
        """Where to send the TCP (and how to turn it) so the tube's end comes to ``standoff`` m out from the journal's
        tip, on the screw's axis and in line with it."""
        ps, rs = obs["screw"]
        pa, ra = obs["arbor"]
        pt, rt = obs["left"]
        u = rs @ np.array([0.0, -1.0, 0.0])
        ua = ra @ np.array([0.0, -1.0, 0.0])
        tip = ps + rs @ np.array([0.0, JL, 0.0])
        want = tip - standoff * u
        ax = np.cross(ua, u)
        sa = float(np.linalg.norm(ax))
        corr = np.eye(3)
        if sa > 1e-6:
            from yam_arena.urdf import axis_angle_matrix

            corr = axis_angle_matrix(ax / sa, math.asin(min(1.0, sa)))
        cmd_p, cmd_r = cur_cmd["p"], cur_cmd["r"]
        err = pt - cmd_p  # how far the arm sits from where it was sent
        new_p = want - err - corr @ (pa - pt)
        return new_p, corr @ cmd_r

    cur_cmd = {"p": pre, "r": rot0}
    lat1 = _Latch(lambda obs: aim(obs, 0.006), lambda: (pre - 0.009 * ey, rot0))
    live(L, 0.8, lat1.get, BODY_HOLD, "line the tube up with the journal (measured), 6 mm out")
    wait(L, 0.6, "hold still: see where the tube's end is now")

    def aim2(obs):
        p1, r1 = lat1.get()
        cur_cmd["p"], cur_cmd["r"] = np.asarray(p1, float), np.asarray(r1, float)
        return aim(obs, 0.002)

    lat2 = _Latch(aim2, lambda: (pre - 0.013 * ey, rot0))
    live(L, 0.6, lat2.get, BODY_HOLD, "and again, 2 mm out")
    # push it on: the tube's bore goes over the journal and butts; then the nut slides along it to the thread start
    push_to = _Latch(lambda obs: (np.asarray(lat2.get()[0]) + (0.002 + JL + (-S_START) + PUSH_PAST) *
                                  (obs["screw"][1] @ np.array([0.0, -1.0, 0.0])), lat2.get()[1]),
                     lambda: (pre - (0.015 + JL - S_START + PUSH_PAST) * ey, rot0))
    live(L, 2.4, push_to.get, BODY_HOLD, "push the tube over the journal, and the nut on to the thread start")
    wait(L, 0.4, "the nut is at the thread start")

    # -- screw it on: stroke, let go, turn back, take it again ----------------------------------------------------------
    sync("the right holds the screw")

    def grip_at(obs, roll):
        """The left TCP on the nut's body, GRIP_IN from its trailing face, on the screw's axis, turned ``roll``; and
        whether the nut is already far enough on (the remaining strokes are then skipped)."""
        ps, rs = obs["screw"]
        u = rs @ np.array([0.0, -1.0, 0.0])
        n = obs["n"]
        p = ps + (n - NUT_L + GRIP_IN) * u
        return p, roll, bool(n >= DONE_N)

    def nominal(n):
        return np.array([SX, Y_SH - (n - NUT_L + GRIP_IN), SZ])

    strokes = []  # (start roll, nominal advance): enough to pass the pen line by 4 mm at the measured advance
    n_est, first = 0.0, True
    while n_est < MARK_N + 0.004 and len(strokes) < 12:
        r0 = PICK_ROLL if first else ROLL0
        strokes.append((r0, LEAD * (ROLL1 - r0) / 360.0))
        n_est += ADV_EST * (ROLL1 - r0) / (ROLL1 - ROLL0)
        first = False

    n_nom = 0.0
    for k, (r0, adv) in enumerate(strokes):
        if k > 0:
            go_l = _Latch(lambda obs: grip_at(obs, ROLL1), lambda nn=n_nom: (nominal(nn), ROLL1, False))
            plan[L].append(_LiveWaypoint(RELEASE_S, lambda g=go_l: (g.get()[0], point_left(ROLL1)), BODY_OPEN,
                                         "let go"))
            for j in range(1, 12):  # turned back the way it came (a slerp would take the short way round)
                plan[L].append(_LiveWaypoint(BACK_S / 11, lambda g=go_l, j=j: (
                    g.get()[0], point_left(ROLL1 if g.get()[2] else ROLL1 + (ROLL0 - ROLL1) * j / 11), BODY_OPEN),
                    BODY_OPEN, "turn the hand back"))
            plan[L].append(_LiveWaypoint(REGRIP_S, lambda g=go_l: (
                g.get()[0], point_left(ROLL1 if g.get()[2] else ROLL0), BODY_OPEN if g.get()[2] else BODY_HOLD),
                BODY_HOLD, "take the nut again"))
            base = go_l
        else:
            base = _Latch(lambda obs: grip_at(obs, PICK_ROLL), lambda: (nominal(0.0), PICK_ROLL, False))
            plan[L].append(_LiveWaypoint(0.3, lambda b=base: (b.get()[0], point_left(PICK_ROLL)), BODY_HOLD,
                                         "on the nut, in line"))
        m = 11
        for j in range(1, m + 1):
            f = j / m
            plan[L].append(_LiveWaypoint(
                STROKE_S / m,
                lambda b=base, f=f, r0=r0, adv=adv: (
                    (np.asarray(b.get()[0]), point_left(ROLL1), BODY_OPEN) if b.get()[2] else
                    (np.asarray(b.get()[0]) - f * adv * ey, point_left(r0 + (ROLL1 - r0) * f), BODY_HOLD)),
                BODY_HOLD, f"screw the nut on ({k + 1} of {len(strokes)}; skipped once it is past the pen line)"))
        n_nom += adv
    # -- the tube comes off: the left lets go, takes the tube by its rear end, draws it off and lays it in the box -----
    last = _Latch(lambda obs: grip_at(obs, ROLL1), lambda: (nominal(n_nom), ROLL1))
    plan[L].append(_LiveWaypoint(RELEASE_S, lambda: (last.get()[0], point_left(ROLL1)), BODY_OPEN, "let go"))

    def tube_grip(obs):
        pa, ra = obs["arbor"]
        ua = ra @ np.array([0.0, -1.0, 0.0])
        return pa - (A_L - A_COL_L - TUBE_GRIP) * ua

    tg = _Latch(tube_grip, lambda: np.array([SX, Y_SH + A_L - A_COL_L - TUBE_GRIP, SZ]))
    plan[L].append(_LiveWaypoint(1.6, lambda: (tg.get(), point_left(90.0)), BODY_OPEN,
                                 "back along the screw to the tube's rear end, the hand turned"))
    plan[L].append(_LiveWaypoint(0.5, lambda: (tg.get(), point_left(90.0)), TUBE_HOLD, "take the tube"))
    plan[L].append(_LiveWaypoint(1.2, lambda: (np.asarray(tg.get()) + 0.022 * ey, point_left(90.0)), TUBE_HOLD,
                                 "draw it straight off the journal"))
    box_tcp = np.array([BXN, BYN + A_L - A_COL_L - TUBE_GRIP, NZB + 0.006])
    plan[L].append(_LiveWaypoint(1.0, lambda: (np.asarray(tg.get()) + 0.022 * ey + 0.03 * up, point_left(90.0)),
                                 TUBE_HOLD, "lift it clear"))
    cur[L] = (box_tcp + 0.03 * up, TUBE_HOLD, point_left(90.0))
    plan[L].append(Waypoint(1.4, tuple((box_tcp + 0.03 * up).tolist()), TUBE_HOLD, "carry it over the box",
                            point_left(90.0)))
    go(L, 0.8, box_tcp, TUBE_HOLD, "lower it into the box")
    go(L, 0.4, box_tcp, TUBE_OPEN, "let it go")
    go(L, 0.6, box_tcp + 0.03 * ey + 0.02 * up, TUBE_OPEN, "back out")
    to_home(L, cur[L][0], cur[L][2], TUBE_OPEN)
    # -- the right lets go of the screw once the tube is off, and both go home ------------------------------------------
    wait(R, dur(L) - dur(R) - LEAD_S - 1.4 - 2.0, "hold the screw")
    go(R, 0.5, r_hold, R_FINGER, "let go of the screw")
    go(R, 0.8, r_hold + 0.025 * up, R_FINGER, "rise")
    to_home(R, cur[R][0], cur[R][2], R_FINGER)
    sync("home")
    for arm in (L, R):
        last_w = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last_w.pos, last_w.finger,
                                  "hold still: the trial ends once the full score has held 1 s", last_w.yaw))
    if os.environ.get("BN_ONE_HAND"):  # the one-hand check: the left does its part, the right never moves
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[R])
        plan[R] = [Waypoint(dur(L), tuple(float(v) for v in p), FINGER_OPEN, "stay home", r)]
    if os.environ.get("BN_STOP"):  # debugging: cut the demo short after this many seconds
        t_stop = float(os.environ["BN_STOP"])
        for arm in (L, R):
            acc, keep = 0.0, []
            for w in plan[arm]:
                if acc >= t_stop:
                    break
                keep.append(w)
                acc += w.duration
            plan[arm] = keep
    return plan
