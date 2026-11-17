"""On a lab bench stands a Bio-Rad Mini-PROTEAN Tetra electrode assembly in its open position, its two green clamping
arms swung out. Two Mini-PROTEAN TGX precast gel cassettes rest on its gel supports, one on each side, short plates
facing in, each leaning 30 degrees away from the centre as the maker's manual describes.

Task: clamp both gels into the electrode assembly, ready to run, and finish with both arms back where they started.

What the physics asks for: a cassette let go of falls back to its 30-degree lean. A clamping arm slides over the
cassettes' side edges only while both cassettes lie flat on the green gaskets at its end; otherwise it stops 20 degrees
short of closed against the leaning cassette's edge. Closing an arm takes about 5.6 N at its top until it passes over
centre and locks. One hand has to squeeze both cassettes flat on the gaskets while the other locks the first arm (the
manual's own two-hand step).

Run it:  inspect-robots run --embodiment yam_arena -E task=clamp_two_precast_gels_into_an_sds_page_electrode_assembly.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the bench: a black epoxy-resin lab bench top --------------------------------------------------------------------
BX0, BX1, BY0, BY1, BENCH_H = 0.150, 0.520, -0.270, 0.270, 0.010
CT = TOP + BENCH_H
# -- the electrode assembly (frame origin: the middle of its base, on the bench) --------------------------------------
FX, FY = 0.300, 0.000
CORE_HX = 0.0125  # half the core's thickness between the gasket seats
GASKET_T = 0.0015  # the green silicone U gasket stands this proud of the core
G = CORE_HX + GASKET_T  # the gasket face: where a seated cassette's short plate lies
BASE_HX, BASE_HY, BASE_H = 0.036, 0.064, 0.006
ZB = 0.014  # a seated cassette's bottom above the bench
POST_Y0, POST_Y1 = 0.037, 0.0495  # the core's end posts
POST_TOP = ZB + 0.090
LEG_Y0, LEG_Y1 = 0.039, 0.047  # the gasket's legs (the U's sides)
NOTCH_Z = ZB + 0.0765  # the notch at the top of each gasket leg: the short plate's top edge sits just below it
SHOULDER_T = 0.0020  # the gasket's shoulder over the notch, proud of the gasket face
SUPPORT_Y = 0.030  # the gel supports: two on each side
SUPPORT_W, SUPPORT_L, SLOPE = 0.012, 0.016, math.radians(12.0)  # their seats slope down outward
LEAN = math.radians(30.0)  # a cassette resting on the supports leans this far out (maker's manual)
# -- the cassettes: Mini-PROTEAN TGX, 8.5 x 10 cm, short plate inward ----------------------------------------------
CAS_W, CAS_H, SHORT_H, CAS_T = 0.100, 0.085, 0.075, 0.0055
PLATE_T, GEL_T = 0.00225, 0.0010
CAS_MASS = 0.040
# -- the clamping arms: green C-shaped clamps on pivots at the base of each end --------------------------------------
PIV_Y, PIV_Z = 0.054, 0.010  # pivot axis (along x) at each end
BACK_T = 0.005  # the arm's back plate: inner face at PIV_Y - 0.0025 = 0.0515 (1.5 mm past the cassette edge)
WALL_X0 = G + CAS_T + 0.0003  # the arm's two clamping walls: their inner faces 0.3 mm outside the cassettes
WALL_T = 0.004
LEAD_IN = 0.0035  # the walls' lead-in: their leading 4 mm slope out this far
WALL_IN = 0.012  # the walls reach this far in from the pivot line over the cassettes' side edges (8 mm overlap)
ARM_H = 0.088  # back plate top above the pivot
ARM_MASS = 0.030
OPEN_DEG = 35.0  # open position: swung out this far
CATCH_DEG = 20.0  # the walls meet the cassettes' edges here: past this only with both cassettes flat at that end
LOCK_DEG = 6.0  # over centre: within this of closed the arm snaps shut and locks
ARM_HOLD = 0.45  # N m: the arm's spring and lock torque (5.6 N at its thumb tab, 8 cm from the pivot)
K_WEAK, K_STRONG = 0.004, 0.05  # N m per degree
ARM_DAMP = 2.0e-4
# -- seating tolerances ---------------------------------------------------------------------------------------------
FLAT_MM = 1.0  # a seated cassette's short plate lies within 1 mm of the gasket face at bottom and top
NOTCH_BAND = (-0.0015, 0.0015)  # its bottom within 1.5 mm of the seat: the short plate's top 0-3 mm below the notch
JAM_MM = 3.0  # an arm slides on when both cassettes lie within 3 mm of the gasket at its end (its lead-in)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

EPOXY, EPOXY_V = (0.10, 0.10, 0.11), (0.16, 0.16, 0.17)
CORE, CORE_D, CORE_L = (0.27, 0.28, 0.30), (0.17, 0.18, 0.19), (0.42, 0.43, 0.46)
GREEN, GREEN_D = (0.22, 0.62, 0.30), (0.14, 0.44, 0.20)
RED, BLACK, PT = (0.80, 0.10, 0.10), (0.05, 0.05, 0.05), (0.85, 0.85, 0.88)
PLATE, PLATE_EDGE, GEL_C = (0.74, 0.82, 0.88), (0.40, 0.50, 0.58), (0.62, 0.74, 0.86)
PRINT_W, WELL_C, LINE_C = (0.97, 0.97, 0.97), (0.45, 0.55, 0.65), (0.15, 0.15, 0.18)
TANK_C, TANK_E, LID_C = (0.82, 0.88, 0.92), (0.60, 0.70, 0.78), (0.90, 0.90, 0.88)
BOX_C, BOX_G = (0.95, 0.95, 0.93), (0.20, 0.55, 0.30)


def _quat(m: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix (Shepperd's method, safe for half turns)."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (float(x / n), float(y / n), float(z / n), float(w / n))


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _box(lo, hi, color, collide=True, rot=None) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide,
                    quat_xyzw=None if rot is None else _quat(rot))


def _rbox(centre, size, rot, color, collide=True) -> BoxShape:
    return BoxShape(tuple(float(v) for v in centre), tuple(float(v) for v in size), color, collide=collide,
                    quat_xyzw=_quat(rot))


# -- the bench and scenery ----------------------------------------------------------------------------------------------

def _bench() -> Body:
    shapes = [_box((-(BX1 - BX0) / 2, -(BY1 - BY0) / 2, 0.0), ((BX1 - BX0) / 2, (BY1 - BY0) / 2, BENCH_H), EPOXY)]
    rng = np.random.default_rng(3)
    for _ in range(10):  # faint wear marks in the resin
        x0, y0 = rng.uniform(-0.16, 0.16), rng.uniform(-0.24, 0.24)
        shapes.append(BoxShape((x0, y0, BENCH_H + 0.0002), (rng.uniform(0.03, 0.08), 0.0012, 0.0004), EPOXY_V,
                               collide=False, yaw=rng.uniform(0, math.pi)))
    return Body.build("bench", pos=((BX0 + BX1) / 2, (BY0 + BY1) / 2, TOP), mass=1.0, kinematic=True, color=EPOXY,
                      shapes=shapes, friction=(0.6, 0.5))


TANK_X, TANK_Y = 0.445, -0.165
TANK_LX, TANK_LY, TANK_H = 0.120, 0.160, 0.150


def _tank() -> Body:
    """The Mini-PROTEAN Tetra mini tank (clear polycarbonate, 12 x 16 cm) with its lid set beside it: scenery."""
    hx, hy, w = TANK_LX / 2, TANK_LY / 2, 0.004
    shapes = [_box((-hx, -hy, 0.0), (hx, hy, 0.004), TANK_E)]
    for s in (-1.0, 1.0):
        shapes.append(_box((-hx, s * hy - w / 2, 0.0), (hx, s * hy + w / 2, TANK_H), None))
        shapes.append(_box((s * hx - w / 2, -hy, 0.0), (s * hx + w / 2, hy, TANK_H), None))
        for z in (0.0, TANK_H - 0.002):  # what shows of the clear walls: their edges, and the fill lines
            shapes.append(_box((-hx, s * hy - 0.0012, z), (hx, s * hy + 0.0012, z + 0.003), TANK_E, collide=False))
            shapes.append(_box((s * hx - 0.0012, -hy, z), (s * hx + 0.0012, hy, z + 0.003), TANK_E, collide=False))
        for t in (-1.0, 1.0):
            shapes.append(_box((s * hx - 0.0015, t * hy - 0.0015, 0.0), (s * hx + 0.0015, t * hy + 0.0015, TANK_H),
                               TANK_E, collide=False))
    for z, c in ((0.060, (0.85, 0.85, 0.85)), (0.085, (0.85, 0.85, 0.85))):
        shapes.append(_box((-hx - 0.0006, -0.030, z), (-hx + 0.0004, 0.030, z + 0.0015), c, collide=False))
    shapes.append(_box((hx - 0.020, hy - 0.004, TANK_H - 0.004), (hx - 0.004, hy, TANK_H), RED, collide=False))
    shapes.append(_box((hx - 0.020, -hy, TANK_H - 0.004), (hx - 0.004, -hy + 0.004, TANK_H), BLACK, collide=False))
    return Body.build("tank", pos=(TANK_X, TANK_Y, CT), mass=1.0, kinematic=True, color=TANK_E, shapes=shapes)


def _lid() -> Body:
    """The tank's lid with its red and black leads, lying upside down beside the tank: scenery."""
    shapes = [_box((-0.060, -0.084, 0.0), (0.060, 0.084, 0.012), LID_C),
              _box((-0.050, -0.074, 0.012), (0.050, 0.074, 0.018), (0.80, 0.80, 0.78))]
    for y, c in ((0.040, RED), (-0.040, BLACK)):
        shapes.append(CylShape((0.0, y, 0.022), 0.006, 0.010, c))
        shapes.append(_box((0.006, y - 0.002, 0.024), (0.055, y + 0.002, 0.028), c, collide=False))
    return Body.build("lid", pos=(0.445, 0.175, CT), mass=1.0, kinematic=True, color=LID_C, shapes=shapes)


def _gel_box() -> Body:
    """The opened box the precast gels came in: scenery."""
    shapes = [_box((-0.060, -0.075, 0.0), (0.060, 0.075, 0.035), BOX_C),
              _box((-0.061, -0.076, 0.010), (0.061, 0.076, 0.020), BOX_G, collide=False),
              _box((-0.050, -0.040, 0.035), (0.040, 0.040, 0.0356), (0.85, 0.86, 0.88), collide=False)]
    return Body.build("gel_box", pos=(0.215, 0.205, CT), mass=1.0, kinematic=True, color=BOX_C, shapes=shapes,
                      yaw=math.radians(8.0))


# -- the electrode assembly ---------------------------------------------------------------------------------------------

def _support_shapes(side: float) -> list:
    """One side's two gel supports: a seat sloping down outward from the gasket face, and a back rest that holds a
    resting cassette at its 30-degree lean. side: +1 the far (+x) side, -1 the near side."""
    out = []
    # where a resting cassette's bottom outer corner sits (u: out from the gasket face)
    uo, zo = CAS_T * math.cos(SLOPE), ZB - CAS_T * math.cos(SLOPE) * math.tan(SLOPE)
    rot_seat = _ry(side * SLOPE)
    rot_rest = _ry(side * LEAN)
    up = np.array([math.sin(LEAN), 0.0, math.cos(LEAN)])
    nout = np.array([math.cos(LEAN), 0.0, -math.sin(LEAN)])
    for sy in (-SUPPORT_Y, SUPPORT_Y):
        th = 0.004
        uc = (SUPPORT_L / 2) - 0.001
        top = np.array([uc, 0.0, ZB - uc * math.tan(SLOPE)])
        nseat = np.array([math.sin(SLOPE), 0.0, math.cos(SLOPE)])
        c = top - nseat * th / 2
        out.append(_rbox((side * (G + c[0]), sy, c[2]), (SUPPORT_L / math.cos(SLOPE), SUPPORT_W, th), rot_seat, CORE))
        # a low lip at the seat's outer side keeps a resting cassette's foot from sliding out
        lip_u0, lip_u1 = CAS_T + 0.0012, CAS_T + 0.0040
        x0, x1 = sorted((side * (G + lip_u0), side * (G + lip_u1)))
        out.append(_box((x0, sy - SUPPORT_W / 2, ZB - 0.004), (x1, sy + SUPPORT_W / 2, ZB - lip_u0 * math.tan(SLOPE)
                                                                                      + 0.0020), CORE))
        # the back rest, along a resting cassette's outer face, reaching above its centre of mass
        s0, s1 = 0.020, 0.050
        cr = np.array([uo, 0.0, zo]) + up * (s0 + s1) / 2 + nout * (0.0015 + 0.0003)
        out.append(_rbox((side * (G + cr[0]), sy, cr[2]), (0.003, SUPPORT_W, s1 - s0), rot_rest, CORE))
        # the support's foot down to the base, and the back rest's strut
        for u0, u1, ztop in ((0.0, 0.012, ZB - 0.007), (0.0165, 0.0205, zo + 0.017)):
            x0, x1 = sorted((side * (G + u0), side * (G + u1)))
            out.append(_box((x0, sy - SUPPORT_W / 2 + 0.002, BASE_H), (x1, sy + SUPPORT_W / 2 - 0.002, ztop), CORE_D))
    return out


def _module() -> Body:
    """The electrode assembly: a glass-filled PBT core with an open inner chamber, a green silicone U gasket on each
    face with a notch at the top of each leg, platinum electrodes, red and black banana plugs, gel supports, and the
    pivots of the two clamping arms. Origin: the middle of its base, on the bench."""
    s = []
    s.append(_box((-BASE_HX, -BASE_HY, 0.0), (BASE_HX, BASE_HY, BASE_H), CORE_D))  # the base
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * POST_Y0, sy * POST_Y1))
        s.append(_box((-CORE_HX, y0, BASE_H), (CORE_HX, y1, POST_TOP), CORE))  # an end post
        s.append(_box((-CORE_HX + 0.002, y0 + 0.002, POST_TOP), (CORE_HX - 0.002, y1 - 0.002, POST_TOP + 0.010), CORE_L))
        # the pivot boss for the clamping arm
        s.append(CylShape((0.0, sy * PIV_Y, PIV_Z), 0.0055, 2 * (WALL_X0 + WALL_T) + 0.004, CORE_D, axis="X",
                          collide=False))
        s.append(_box((-0.010, sy * PIV_Y - 0.004, 0.0), (0.010, sy * PIV_Y + 0.004, PIV_Z), CORE_D, collide=False))
    s.append(_box((-CORE_HX, -POST_Y0, BASE_H), (CORE_HX, POST_Y0, ZB + 0.010), CORE))  # the bottom crossbar
    # banana plugs: the anode (red) at the right-hand end (-y), the cathode (black) at the left (+y)
    for sy, c in ((-1.0, RED), (1.0, BLACK)):
        yc = sy * (POST_Y0 + POST_Y1) / 2
        s.append(CylShape((0.0, yc, POST_TOP + 0.013), 0.0045, 0.006, c))
        s.append(CylShape((0.0, yc, POST_TOP + 0.020), 0.0018, 0.008, (0.75, 0.68, 0.30), collide=False))
    # gaskets: a U on each face (legs up the end posts, bottom along the crossbar), a shoulder over each leg's notch
    for sx in (-1.0, 1.0):
        x0, x1 = sorted((sx * CORE_HX, sx * G))
        for sy in (-1.0, 1.0):
            y0, y1 = sorted((sy * LEG_Y0, sy * LEG_Y1))
            s.append(_box((x0, y0, ZB + 0.004), (x1, y1, NOTCH_Z), GREEN))
            xs0, xs1 = sorted((sx * CORE_HX, sx * (G + SHOULDER_T)))
            s.append(_box((xs0, y0, NOTCH_Z), (xs1, y1, NOTCH_Z + 0.0065), GREEN))
        s.append(_box((x0, -LEG_Y1, ZB + 0.004), (x1, LEG_Y1, ZB + 0.010), GREEN))
        # the gel supports and back rests
        s.extend(_support_shapes(sx))
        # the lower electrode: a platinum wire along the base on each side (a picture only)
        s.append(CylShape((sx * (CORE_HX + 0.003), 0.0, BASE_H + 0.001), 0.0004, 0.080, PT, axis="Y", collide=False))
    # the upper electrode in the inner chamber (a picture only), and a dark inner-chamber floor
    s.append(CylShape((0.0, 0.0, ZB + 0.014), 0.0004, 0.068, PT, axis="Y", collide=False))
    s.append(_box((-CORE_HX + 0.001, -POST_Y0, ZB + 0.010), (CORE_HX - 0.001, POST_Y0, ZB + 0.0105), CORE_D,
                  collide=False))
    return Body.build("module", pos=(FX, FY, CT), mass=0.2, kinematic=True, color=CORE, shapes=s,
                      friction=(1.0, 0.9))


def _arm(name: str, sy: float) -> Body:
    """A green clamping arm: a C-shaped clamp (back plate and two walls) on a pivot at the base of one end, with a
    ribbed thumb tab at its top. Origin: on the pivot axis; drawn closed (its walls over the cassettes' side edges)."""
    s = []
    b0, b1 = sorted((sy * -BACK_T / 2, sy * BACK_T / 2))
    hw = WALL_X0 + WALL_T
    s.append(_box((-hw, b0, 0.006), (hw, b1, ARM_H), GREEN))  # the back plate
    for sx in (-1.0, 1.0):
        xi0, xi1 = sorted((sx * WALL_X0, sx * (WALL_X0 + WALL_T)))
        y0, y1 = sorted((sy * -BACK_T / 2, sy * -(WALL_IN - 0.004)))
        s.append(_box((xi0, y0, 0.022), (xi1, y1, 0.082), GREEN))  # a clamping wall
        # its leading edge: a lead-in sloping out 3.5 mm over its last 4 mm, which wedges a cassette onto the gasket
        a = np.array([sx * WALL_X0, sy * -(WALL_IN - 0.004)])
        b = np.array([sx * (WALL_X0 + LEAD_IN), sy * -WALL_IN])
        d = b - a
        n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        if n[0] * sx < 0:
            n = -n
        c = (a + b) / 2 + n * 0.001
        s.append(_rbox((c[0], c[1], 0.052), (float(np.linalg.norm(d)) + 0.0006, 0.002, 0.060),
                       _rz(math.atan2(d[1], d[0])), GREEN))
        s.append(_box((min(sx * (WALL_X0 + LEAD_IN), sx * (WALL_X0 + WALL_T)),
                       min(sy * -WALL_IN, sy * -(WALL_IN - 0.004)), 0.022),
                      (max(sx * (WALL_X0 + LEAD_IN), sx * (WALL_X0 + WALL_T)),
                       max(sy * -WALL_IN, sy * -(WALL_IN - 0.004)), 0.082), GREEN, collide=False))
    t0, t1 = sorted((sy * -0.003, sy * 0.007))
    s.append(_box((-0.020, t0, ARM_H - 0.004), (0.020, t1, ARM_H + 0.006), GREEN))  # the thumb tab
    for k in range(4):
        z = ARM_H - 0.002 + k * 0.0022
        r0, r1 = sorted((sy * 0.007, sy * 0.0078))
        s.append(_box((-0.018, r0, z), (0.018, r1, z + 0.0008), GREEN_D, collide=False))
    s.append(CylShape((0.0, 0.0, 0.0), 0.0045, 2 * hw, GREEN_D, axis="X", collide=False))  # its pivot
    s.append(_box((-hw, b0, 0.0), (hw, b1, 0.006), GREEN, collide=False))
    start = _rx(-sy * math.radians(OPEN_DEG))
    return Body.build(name, pos=(FX, FY + sy * PIV_Y, CT + PIV_Z), mass=ARM_MASS, shapes=s, color=GREEN,
                      friction=(0.25, 0.2), slick=True, ignore=("module",), solver_iterations=32,
                      damping=(0.0, 0.2), start_xyzw=_quat(start), com=(0.0, 0.0, 0.045))


def _cassette_rest(side: float):
    """A cassette's resting pose on its supports, leaning LEAN out: (origin, rotation). Its origin is the middle of
    its short plate's bottom edge, its local +x out through the long plate, +z up."""
    uo, zo = CAS_T * math.cos(SLOPE), ZB - CAS_T * math.cos(SLOPE) * math.tan(SLOPE)
    ui, zi = uo - CAS_T * math.cos(LEAN), zo + CAS_T * math.sin(LEAN)  # its bottom inner corner
    r = _ry(LEAN)
    if side < 0:
        r = _rz(math.pi) @ r
    origin = np.array([FX + side * (G + ui), FY, CT + zi + 0.0004])
    return origin, r


def _cassette(name: str, side: float) -> Body:
    """A Mini-PROTEAN TGX precast gel cassette: a short plate (inside) and a long plate (outside) with the 1 mm gel
    between, comb and tape off. Collides as one 5.5 mm slab up to the short plate's top, and the long plate above it."""
    hw = CAS_W / 2
    s = [_box((0.0, -hw, 0.0), (CAS_T, hw, SHORT_H), None),
         _box((PLATE_T + GEL_T, -hw, SHORT_H), (CAS_T, hw, CAS_H), None)]
    # what shows: the plates' faces and edges, the gel, its wells, the long plate's printing
    s.append(_box((CAS_T - 0.0004, -hw + 0.0005, 0.0005), (CAS_T, hw - 0.0005, CAS_H - 0.0005), PLATE, collide=False))
    s.append(_box((0.0, -hw + 0.0005, 0.0005), (0.0004, hw - 0.0005, SHORT_H - 0.0005), PLATE, collide=False))
    s.append(_box((CAS_T - 0.0002, -0.043, 0.004), (CAS_T + 0.0003, 0.043, 0.072), GEL_C, collide=False))
    for e in (-hw, hw - 0.0012):
        s.append(_box((0.0, e, 0.0), (CAS_T + 0.0002, e + 0.0012, SHORT_H), PLATE_EDGE, collide=False))
        s.append(_box((PLATE_T + GEL_T, e, SHORT_H), (CAS_T + 0.0002, e + 0.0012, CAS_H), PLATE_EDGE, collide=False))
    s.append(_box((0.0, -hw, SHORT_H - 0.0010), (PLATE_T + GEL_T, hw, SHORT_H), PLATE_EDGE, collide=False))
    s.append(_box((PLATE_T + GEL_T, -hw, CAS_H - 0.0010), (CAS_T + 0.0002, hw, CAS_H), PLATE_EDGE, collide=False))
    for k in range(10):  # the ten wells under the short plate's top, numbered on the long plate
        y = -0.0405 + k * 0.009
        s.append(_box((CAS_T - 0.0001, y - 0.0025, 0.060), (CAS_T + 0.0004, y + 0.0025, 0.0745), WELL_C, collide=False))
        s.append(_box((CAS_T - 0.0001, y - 0.0007, 0.077), (CAS_T + 0.0005, y + 0.0007, 0.081), PRINT_W, collide=False))
    s.append(_box((CAS_T - 0.0001, -0.043, 0.009), (CAS_T + 0.0005, 0.043, 0.0098), LINE_C, collide=False))
    s.append(_box((CAS_T - 0.0001, 0.020, 0.003), (CAS_T + 0.0005, 0.044, 0.0065), PRINT_W, collide=False))
    origin, r = _cassette_rest(side)
    return Body.build(name, pos=tuple(origin.tolist()), mass=CAS_MASS, shapes=s, color=PLATE, friction=(0.8, 0.7),
                      solver_iterations=32, damping=(0.0, 0.1), start_xyzw=_quat(r),
                      com=(CAS_T / 2, 0.0, CAS_H * 0.47))


# the arms on their pivots: the +y ("left-hand end") arm opens to -OPEN_DEG about x, the -y arm to +OPEN_DEG
hinge("arm_p_pivot", "module", "arm_p", anchor=(FX, FY + PIV_Y, CT + PIV_Z), axis="X", limits_deg=(-OPEN_DEG, 0.0),
      spring=K_WEAK, damping=ARM_DAMP, target_deg=-OPEN_DEG)
hinge("arm_m_pivot", "module", "arm_m", anchor=(FX, FY - PIV_Y, CT + PIV_Z), axis="X", limits_deg=(0.0, OPEN_DEG),
      spring=K_WEAK, damping=ARM_DAMP, target_deg=OPEN_DEG)

_author_hinges_plain = bodykit._author_hinges


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, then: each arm's drive is capped at ARM_HOLD (its spring and its over-centre lock)."""
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name in ("arm_p", "arm_m"):
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/{spec.name}_pivot")
        UsdPhysics.DriveAPI(prim, "angular").GetMaxForceAttr().Set(float(ARM_HOLD))


bodykit._author_hinges = _author_hinges

OBJECTS = [_bench(), _tank(), _lid(), _gel_box(), _module(), _arm("arm_p", 1.0), _arm("arm_m", -1.0),
           _cassette("gel_near", -1.0), _cassette("gel_far", 1.0)]

ARMS = {"arm_p": 1.0, "arm_m": -1.0}  # arm -> its end (+y / -y)
GELS = {"gel_near": -1.0, "gel_far": 1.0}  # cassette -> its side (-x / +x)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"state": {a: "open" for a in ARMS}, "jam": {a: None for a in ARMS}, "dropped": False,
            "both_flat_seen": False, "tcps": None, "s": {}, "wrist_gap_min": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SDS_DEBUG")), "steps": 0}
_LIMITS: dict = {}


def _set_upper(arm: str, closed_side_limit: float) -> None:
    """Move the closed-side limit of an arm's pivot (degrees): 0 free to close, +-CATCH_DEG jammed."""
    if _EP["jam"][arm] == closed_side_limit:
        return
    if arm not in _LIMITS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        joint = UsdPhysics.RevoluteJoint(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS[f"{arm}_pivot"]))
        _LIMITS[arm] = (joint.GetLowerLimitAttr(), joint.GetUpperLimitAttr())
    lo, hi = _LIMITS[arm]
    if ARMS[arm] > 0:  # +y arm: open at -OPEN_DEG, closed at 0 (the upper limit)
        hi.Set(float(closed_side_limit))
    else:
        lo.Set(float(closed_side_limit))
    _EP["jam"][arm] = closed_side_limit


def _arm_drive(arm: str, state: str) -> None:
    sy = ARMS[arm]
    if state == "open":
        bodykit.set_hinge_drive(f"{arm}_pivot", target_deg=-sy * OPEN_DEG, spring=K_WEAK, damping=ARM_DAMP)
    elif state == "resist":
        bodykit.set_hinge_drive(f"{arm}_pivot", target_deg=-sy * OPEN_DEG, spring=K_STRONG, damping=ARM_DAMP)
    else:  # locked: pressed against its closed stop
        bodykit.set_hinge_drive(f"{arm}_pivot", target_deg=sy * 4.0, spring=K_STRONG, damping=ARM_DAMP)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for arm in ARMS:
        _arm_drive(arm, "open")
        _set_upper(arm, 0.0)


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append((g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET), g[:3] - origin))
    return out


def _gel(p, r, side: float) -> dict:
    """Where a cassette is against its gasket: how far its short plate lies off the gasket face at the bottom and at the
    top of the short plate, at each end (mm), how far it leans out (deg), its bottom against its seat (m), and whether it
    faces the right way (short plate in)."""
    out = np.array([side, 0.0, 0.0])
    facing = float(r[:, 0] @ out)
    lean = math.degrees(math.atan2(float(side * r[0, 2]), float(r[2, 2])))
    off = {}
    for ey in (-0.045, 0.045):
        for ez, key in ((0.0, "b"), (SHORT_H, "t")):
            q = p + r @ np.array([0.0, ey, ez])
            off[(ey > 0, key)] = (side * (q[0] - FX) - G) * 1000.0
    bottom = p + r @ np.array([0.0, 0.0, 0.0])
    corners = [p + r @ np.array([u, y, z]) for u in (0.0, CAS_T) for y in (-CAS_W / 2, CAS_W / 2) for z in (0.0, CAS_H)]
    low = min(c[2] for c in corners)
    return {"facing": facing, "lean": lean, "off": off, "dz": float(bottom[2] - (CT + ZB)),
            "dy": float(p[1] - FY), "low": float(low), "p": p}


def _flat_at(g: dict, end: float | None, tol_mm: float) -> bool:
    """A cassette lying flat on its gasket (within tol_mm at the bottom and at the short plate's top), facing the right
    way, at one end (+1/-1) or both (None)."""
    if g["facing"] < 0.95 or abs(g["dy"]) > 0.004 or not (-0.002 < g["dz"] < 0.004):
        return False
    ends = (True, False) if end is None else (end > 0,)
    return all(-0.6 < g["off"][(e, k)] < tol_mm for e in ends for k in ("b", "t"))


def _seated(g: dict) -> bool:
    """Seated: flat on the gasket within FLAT_MM all round and its short plate's top in the notch band."""
    return _flat_at(g, None, FLAT_MM) and NOTCH_BAND[0] <= g["dz"] <= NOTCH_BAND[1]


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    gels = {}
    for name, side in GELS.items():
        p, r = _pose(base, origin, name)
        gels[name] = _gel(p, r, side)
        if gels[name]["low"] < CT + 0.004 or abs(gels[name]["dy"]) > 0.08:
            _EP["dropped"] = True  # fell off its supports onto the bench
    flat_end = {e: all(_flat_at(g, e, JAM_MM) for g in gels.values()) for e in (-1.0, 1.0)}
    seated = {n: _seated(g) for n, g in gels.items()}
    if all(_flat_at(g, None, FLAT_MM) for g in gels.values()):
        _EP["both_flat_seen"] = True
    arms = {}
    for arm, sy in ARMS.items():
        _, ra = _pose(base, origin, arm)
        theta = math.degrees(math.atan2(float(ra[2, 1]), float(ra[1, 1])))
        phi = -sy * theta  # how far open (0 closed, OPEN_DEG open)
        arms[arm] = phi
        st = _EP["state"][arm]
        if st == "locked":
            if phi > LOCK_DEG + 3.0:
                st = "resist"
        elif phi <= LOCK_DEG:
            st = "locked"
        elif phi <= CATCH_DEG + 0.5:
            st = "resist"
        else:
            st = "open"
        if st != _EP["state"][arm]:
            _arm_drive(arm, st)
            _EP["state"][arm] = st
        # the walls meet the cassettes' edges at CATCH_DEG: they slide on only over cassettes lying flat at this end
        if phi >= CATCH_DEG - 0.5:
            _set_upper(arm, 0.0 if flat_end[sy] else -sy * CATCH_DEG)
    locked = {a: _EP["state"][a] == "locked" and arms[a] <= 2.5 for a in ARMS}
    tcps = _tcps(base, origin)
    _EP["tcps"] = [t[0] for t in tcps]
    gap = float(np.linalg.norm(tcps[0][1] - tcps[1][1]))
    _EP["s"] = {"locked": locked, "seated": seated, "both_seated": all(seated.values()),
                "n_locked": sum(locked.values())}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            mm = lambda v: np.round((np.asarray(v) - [FX, FY, CT]) * 1000).astype(int).tolist()  # noqa: E731
            gtxt = " ".join(
                f"{n[4:]}:lean={g['lean']:5.1f} dz={g['dz'] * 1000:4.1f} dy={g['dy'] * 1000:4.1f} "
                f"off=[{','.join(f'{v:4.1f}' for v in g['off'].values())}] seat={seated[n]}"
                for n, g in gels.items())
            print(f"[sds] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]} "
                  f"arms={{p:{arms['arm_p']:5.1f} {_EP['state']['arm_p']} jam={_EP['jam']['arm_p']}, "
                  f"m:{arms['arm_m']:5.1f} {_EP['state']['arm_m']} jam={_EP['jam']['arm_m']}}} {gtxt} "
                  f"flat_end={flat_end} L={mm(tcps[0][0])} R={mm(tcps[1][0])} wrists={gap * 100:.1f}cm "
                  f"drop={_EP['dropped']} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: both arms locked, both cassettes seated, and both robot arms back where they started; 0.75: both arms
    locked with both cassettes seated; 0.5: one arm locked with both cassettes seated; 0.25: both cassettes lay flat on
    their gaskets at the same moment at some point; 0.0 otherwise, and 0.0 whenever a cassette has fallen onto the
    bench. Judged at the end on live states, except the 0.25 tier and the fall (whole trial)."""
    s = _EP["s"]
    if not s or _EP["dropped"]:
        return 0.0
    if s["both_seated"] and s["n_locked"] == 2:
        return 1.0 if _arms_home() else 0.75
    if s["both_seated"] and s["n_locked"] == 1:
        return 0.5
    return 0.25 if _EP["both_flat_seen"] else 0.0


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
    """Full score with both cassettes and both clamping arms at rest: the trial ends once that has held 1 s."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("gel_near", "gel_far", "arm_p", "arm_m"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand pushes the near cassette up flat against its gasket. The left hand comes down over both cassettes,
# its jaw open across them, and closes: its far finger brings the far cassette in, and it squeezes the pair flat on the
# gaskets. The right lets go of the near cassette and pushes the right-hand clamping arm up over the cassettes' edges
# until it locks. The left lets go and pushes the other arm shut. Both arms go back to where they started.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.060
F = np.array([FX, FY, CT])  # the frame origin in world coordinates

# right hand: push the near cassette flat (closing along y, so its thin side pushes along x)
R_PUSH_Y = -0.030
R_PUSH_Z = ZB + 0.066  # TCP height (fingertips 13 mm lower, clear of the back rests)
R_PUSH_X0 = -(G + 0.066)  # in front of the leaning cassette's top
R_PUSH_X1 = -(G + CAS_T) - 0.012 + 0.006  # the finger's side on the flat cassette's face, aimed 6 mm in
# left hand: the squeeze across both cassettes (closing along x)
L_SQ_Y = 0.036
L_SQ_X0 = 0.019  # jaw fully open: far fingertip 4 mm outside the far cassette's top corner, near one 8.5 mm out
L_SQ_Z = ZB + 0.068 + 0.013  # fingertips 5.6 mm below a resting cassette's top corner
SQ_HOLD = -0.0170  # finger target for the squeeze: 2.5 mm inside the pair's faces each side (about 12 N)
# pushes on the arms: the thin side of closed fingers (closing along x) on the arm's back plate near its top
ARM_PUSH_Z = PIV_Z + 0.072


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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lean(rot: np.ndarray, deg: float) -> np.ndarray:
    """A grip leaned about world x: positive tips the wrist toward +y."""
    return _rx(-math.radians(deg)) @ rot


ROT_R_PUSH = _ry(-math.radians(3.0)) @ _lean(grip_rotation(0.0), -25.0)  # right: closing along y, wrist toward
# -y, fingertips 3 degrees ahead so they stand upright under the push
ROT_L_SQ = _lean(grip_rotation(-math.pi / 2), 25.0)  # left: closing along x, its wrist leaned toward +y
ROT_R_ARM = _lean(grip_rotation(math.pi / 2), -20.0)
ROT_L_ARM = _lean(grip_rotation(-math.pi / 2), 20.0)


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    one_hand = os.environ.get("SDS_ONE_HAND", "")

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            f = finger if not isinstance(finger, tuple) else finger[0] + (finger[1] - finger[0]) * j / n
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), f, label, rot))
        cur[arm] = (p1, finger if not isinstance(finger, tuple) else finger[1], rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def w(x, y, z):
        return F + np.array([x, y, z])

    # 1. the right hand pushes the near cassette up flat against its gasket
    r0 = w(R_PUSH_X0, R_PUSH_Y, R_PUSH_Z)
    plan["right"] += _joint_path("right", np.array(READY, float), _good_ik("right", r0 + [0, 0, 0.05], ROT_R_PUSH),
                                 "over the near cassette's front", FINGER_CLOSED)
    cur["right"] = (r0 + [0, 0, 0.05], FINGER_CLOSED, ROT_R_PUSH)
    go("right", 1.0, r0, FINGER_CLOSED, "down in front of the leaning near cassette")
    go("right", 2.0, w(R_PUSH_X1, R_PUSH_Y, R_PUSH_Z), FINGER_CLOSED, "push it up flat against its gasket", n=4)

    # 2. the left hand comes down over both cassettes, its jaw open across them, and squeezes them flat together
    sq0 = w(L_SQ_X0, L_SQ_Y, L_SQ_Z)
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", sq0 + [0, 0, 0.06], ROT_L_SQ),
                                "over both cassettes", FINGER_OPEN)
    cur["left"] = (sq0 + [0, 0, 0.06], FINGER_OPEN, ROT_L_SQ)
    wait("left", dur("right") - dur("left"))
    go("left", 1.4, sq0, FINGER_OPEN, "down: near finger outside the near cassette, far finger beyond the far one")
    # close while moving in, so the near finger stays where it is and the far finger brings the far cassette in
    go("left", 2.4, w(0.0, L_SQ_Y, L_SQ_Z), (FINGER_OPEN, -0.024), "close: bring the far cassette in flat", n=8)
    go("left", 0.8, w(0.0, L_SQ_Y, L_SQ_Z), SQ_HOLD, "squeeze both cassettes flat on the gaskets")
    wait("right", dur("left") - dur("right"), "hold the near cassette")
    # 3. the right lets go of the near cassette and pushes the right-hand arm (-y end) up over the edges until it locks
    go("right", 0.6, w(R_PUSH_X1 - 0.030, R_PUSH_Y, R_PUSH_Z), FINGER_CLOSED, "let go of the near cassette")
    go("right", 0.8, w(R_PUSH_X1 - 0.030, R_PUSH_Y, R_PUSH_Z + 0.06), FINGER_CLOSED, "rise")
    phi0 = math.radians(OPEN_DEG)
    top_y = -(PIV_Y + 0.007 * math.cos(phi0) + (ARM_H + 0.006) * math.sin(phi0))  # the open arm's outermost corner
    m0 = w(0.0, top_y - 0.012 - 0.008, ARM_PUSH_Z)
    go("right", 1.8, m0 + [0, 0, 0.05], FINGER_CLOSED, "out beyond the right-hand clamping arm", rot=ROT_R_ARM, n=3)
    go("right", 1.0, m0, FINGER_CLOSED, "down beside its top")
    m1 = w(0.0, -(PIV_Y + BACK_T / 2) - 0.012 + 0.022, ARM_PUSH_Z + 0.008)
    go("right", 2.6, m1, FINGER_CLOSED, "push the arm up over the cassettes' edges until it locks", n=6)
    wait("right", 0.4)
    go("right", 0.8, w(0.0, -(PIV_Y + 0.040), ARM_PUSH_Z + 0.050), FINGER_CLOSED, "back off")
    # 4. the left lets go of the squeeze and pushes the left-hand arm (+y end) shut the same way
    wait("left", dur("right") - dur("left"), "squeeze until the first arm is locked")
    go("left", 0.6, w(0.0, L_SQ_Y, L_SQ_Z), -0.030, "let go of the cassettes")
    go("left", 0.8, w(0.0, L_SQ_Y, L_SQ_Z + 0.050), -0.030, "rise clear of the cassettes")
    p0 = w(0.0, -top_y + 0.012 + 0.008, ARM_PUSH_Z)
    go("left", 1.8, p0 + [0, 0, 0.05], FINGER_CLOSED, "out beyond the left-hand clamping arm", rot=ROT_L_ARM, n=3)
    go("left", 1.0, p0, FINGER_CLOSED, "down beside its top")
    p1 = w(0.0, (PIV_Y + BACK_T / 2) + 0.012 - 0.022, ARM_PUSH_Z + 0.008)
    go("left", 2.6, p1, FINGER_CLOSED, "push the arm up over the cassettes' edges until it locks", n=6)
    wait("left", 0.4)
    go("left", 0.8, w(0.0, PIV_Y + 0.040, ARM_PUSH_Z + 0.050), FINGER_CLOSED, "back off")

    # 5. both arms go home and hold still
    wait("right", dur("left") - dur("right"))
    for arm in ("left", "right"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    if one_hand in ("left", "right"):  # the one-hand check: the other hand never moves
        from yam_arena import ik, station

        other = "left" if one_hand == "right" else "right"
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[other])
        plan[other] = [Waypoint(dur(one_hand), tuple(float(v) for v in p), FINGER_CLOSED, "never moves", r)]
    return plan
