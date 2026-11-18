"""Couple a trailer's lights: the tractor's 7-way plug (SAE J560) is parked in its holder on the tractor side, on its
cord. On the trailer's front wall is the 7-way socket, its spring-closed cover shut over it.

Task: plug the tractor's 7-way plug into the trailer's socket all the way, and let the socket's cover close on it so
it latches the plug in.

What it asks for: the cover is spring-closed and hangs over the socket's mouth, so one hand has to hold it up while
the other brings the plug in under it, square to the socket and with its key in the keyway, and pushes it fully home.
Let go, the cover drops onto the plug; its lip catches behind the plug's top latch ear only if the plug is fully
home (a plug left a few millimetres short has the lip land on top of the ear instead, and it is not latched).

Run it:  inspect-robots run --embodiment yam_arena -E task=connect_a_trailer_7_way_plug.py --instruction "..." \\
           --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, MeshShape, allow_matrix_grips, as_numpy, ball, hinge,  # noqa: E402
                     on_reset, on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the trailer's socket: face of its rim in the plane x = X_F, axis along +x, centre (Y_C, Z_C) ---------------------
X_F, Y_C, Z_C = 0.330, 0.000, TOP + 0.190
BORE_R = 0.0205  # the socket's bore (the plug's barrel is 38.5 mm across)
DEPTH = 0.045  # the plug's face stops this far in (against the contact insert)
WALL_T = 0.004
KEY_W = 0.0100  # the keyway, at the socket's left (+y)
HINGE = np.array([X_F - 0.0030, Y_C, Z_C + 0.0330])  # the cover's hinge line, along y, above the mouth
LID_L, LID_W, LID_T = 0.0340, 0.060, 0.0030  # the cover: hinge to lip
LIP_H = 0.0070  # its latch lip, at the free end
TAB_L = 0.0120  # the thumb lip, sticking out along the cover's free edge
LID_OPEN_MAX = 125.0
# -- the tractor's plug (Erich Jaeger 211107 class: 123 mm long, barrel 38.5 mm, 58 mm across its latch ears) -----------
P_LEN, P_R = 0.123, 0.01925
BARREL_L = 0.059  # its barrel, from the face back to the ears
EAR_L, EAR_R, EAR_W = 0.019, 0.029, 0.016  # the two latch ears (top and bottom): length along the plug, reach, width
EAR_FLAT = 0.006  # the top one's flat, behind its 13 mm ramp
BODY_L, BODY_R = 0.026, 0.0210
GROM_R = 0.0110
KEY_RIB = (0.004, 0.036, 0.0030, 0.0050)  # from the face: start, end; how far proud; its width
BAND_AT = 0.045  # a yellow band on the barrel this far back from the face: hidden once the plug is home
P_MASS = 0.18
# -- where the plug starts: parked in its holder on the tractor side --------------------------------------------------
X_H, Y_H = 0.300, -0.170  # the holder's mouth (plane x = X_H), its axis at (Y_H, Z_C)
PARK = 0.030  # the plug's face is this far into the holder
# -- the cord from the plug's back to the back of the cab -----------------------------------------------------------------
ANCHOR = np.array([0.200, -0.300, TOP + 0.030])
CORD_R, CORD_N, CORD_MASS = 0.007, 11, 0.20
# -- the rules ------------------------------------------------------------------------------------------------------------
HOME = DEPTH - 0.0015  # fully home: the face within 1.5 mm of the contact insert (the yellow band hidden)
IN_BORE = 0.010
RELEASED = 0.030
SUCCESS_HOLD_S = 1.0

ALU, ALU_D, BLACK, GREY = (0.78, 0.79, 0.80), (0.55, 0.56, 0.58), (0.06, 0.06, 0.07), (0.30, 0.31, 0.33)
PAINT, YELLOW, RED, GREEN, WHITE = (0.85, 0.86, 0.88), (0.95, 0.80, 0.10), (0.75, 0.12, 0.10), (0.20, 0.55, 0.30), (0.93, 0.93, 0.92)
STEEL, CORD_C = (0.70, 0.71, 0.73), (0.10, 0.10, 0.11)


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _ring(centre_x0, centre_x1, yc, zc, r_in, t, n, color, gap_at=None, gap_w=0.0, rel=lambda p: p):
    """A tube along x as n flat staves (inner radius r_in, wall t), optionally with a slot of gap_w at angle gap_at
    (rad, 0 = +y, pi/2 = +z)."""
    out = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        a = (a0 + a1) / 2
        width = 2 * (r_in + t / 2) * math.tan(math.pi / n) + 0.0006
        if gap_at is not None:
            d = (a - gap_at + math.pi) % (2 * math.pi) - math.pi
            half = gap_w / 2 / (r_in + t / 2)
            if abs(d) < half + math.pi / n:
                continue  # the slot (staves near it are dropped; the slot's sides are added below)
        c = np.array([(centre_x0 + centre_x1) / 2, yc + (r_in + t / 2) * math.cos(a), zc + (r_in + t / 2) * math.sin(a)])
        q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))  # turned about x by a
        out.append(BoxShape(tuple(rel(c).tolist()), (centre_x1 - centre_x0, t, width), color, quat_xyzw=q))
    if gap_at is not None:
        for s in (-1.0, 1.0):  # the slot's two sides and its floor (the keyway runs back to the insert)
            a = gap_at + s * (gap_w / 2 + 0.0015) / (r_in + t / 2)
            c = np.array([(centre_x0 + centre_x1) / 2, yc + (r_in + t / 2) * math.cos(a), zc + (r_in + t / 2) * math.sin(a)])
            q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
            out.append(BoxShape(tuple(rel(c).tolist()), (centre_x1 - centre_x0, t, 0.003), color, quat_xyzw=q))
        a = gap_at
        c = np.array([(centre_x0 + centre_x1) / 2, yc + (r_in + t + 0.0025) * math.cos(a), zc + (r_in + t + 0.0025) * math.sin(a)])
        q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
        out.append(BoxShape(tuple(rel(c).tolist()), (centre_x1 - centre_x0, 0.002, gap_w + 0.006), color, quat_xyzw=q))
    return out


# -- the trailer: its front wall and the socket ------------------------------------------------------------------------
SOCK_O = np.array([X_F, Y_C, Z_C])


def _trailer() -> Body:
    """The trailer's front wall (a painted steel panel standing on the table's far side) with the 7-way socket's
    flange, its bore and keyway, the contact insert at the bottom, and the cover's hinge lugs. Kinematic."""
    o = SOCK_O
    rel = lambda p: np.asarray(p, float) - o  # noqa: E731
    def holed(x0, x1, y0, y1, z0, z1, h, color):  # a plate with a square hole of half-size h round the socket's axis
        return [_box(rel((x0, y0, z0)), rel((x1, y1, Z_C - h)), color),
                _box(rel((x0, y0, Z_C + h)), rel((x1, y1, z1)), color),
                _box(rel((x0, y0, Z_C - h)), rel((x1, Y_C - h, Z_C + h)), color),
                _box(rel((x0, Y_C + h, Z_C - h)), rel((x1, y1, Z_C + h)), color)]
    sh = holed(X_F + 0.008, X_F + 0.011, -0.20, 0.20, TOP, TOP + 0.42, 0.0262, PAINT)  # the nose wall (2 in hole)
    sh.append(_box(rel((X_F + 0.0074, -0.20, TOP + 0.395)), rel((X_F + 0.0079, 0.20, TOP + 0.42)), RED, False))
    # the flange (two-hole mount, holes 74.4 mm apart) round the socket's mouth, and its two screws
    sh += holed(X_F, X_F + 0.008, Y_C - 0.048, Y_C + 0.048, Z_C - 0.031, Z_C + 0.031, BORE_R + WALL_T, GREY)
    for s in (-1.0, 1.0):
        sh.append(CylShape(tuple(rel((X_F - 0.0005, Y_C + s * 0.0372, Z_C)).tolist()), 0.0045, 0.002, STEEL,
                           collide=False, axis="X"))
    sh += _ring(X_F + 0.002, X_F + DEPTH + 0.004, Y_C, Z_C, BORE_R, WALL_T, 20, GREY, gap_at=0.0, gap_w=KEY_W, rel=rel)
    for k in range(24):  # the mouth's lead-in: a 45-degree funnel from 26 mm down to the bore, the keyway left open
        a = 2 * math.pi * (k + 0.5) / 24
        if abs((a + math.pi) % (2 * math.pi) - math.pi) < 0.30:
            continue
        rad = np.array([0.0, math.cos(a), math.sin(a)])
        p1 = SOCK_O + np.array([-0.005, 0.0, 0.0]) + rad * (BORE_R + 0.0065)
        p2 = SOCK_O + np.array([0.0025, 0.0, 0.0]) + rad * (BORE_R + 0.0003)
        sh.append(bodykit.plate(tuple(rel(p1).tolist()), tuple(rel(p2).tolist()), 2 * (BORE_R + 0.004) * math.tan(math.pi / 24) + 0.001,
                                0.002, tuple((rad + np.array([-1.0, 0.0, 0.0])).tolist()), GREY))
    sh.append(CylShape(tuple(rel((X_F + DEPTH + 0.0075, Y_C, Z_C)).tolist()), BORE_R + 0.001, 0.015, BLACK, axis="X"))
    for k in range(7):  # the contact insert's seven sockets, seen down the bore
        a = 2 * math.pi * k / 6
        y, z = (0.0, 0.0) if k == 6 else (0.011 * math.cos(a), 0.011 * math.sin(a))
        sh.append(CylShape(tuple(rel((X_F + DEPTH - 0.0002, Y_C + y, Z_C + z)).tolist()), 0.0026, 0.0006, (0.70, 0.55, 0.25),
                           collide=False, axis="X"))
    for s in (-1.0, 1.0):  # the cover's hinge lugs
        sh.append(_box(rel((X_F - 0.006, Y_C + s * 0.026 - 0.003, HINGE[2] - 0.004)),
                       rel((X_F, Y_C + s * 0.026 + 0.003, HINGE[2] + 0.004)), GREY, False))
    sh.append(CylShape(tuple(rel(HINGE).tolist()), 0.0018, 0.058, STEEL, collide=False, axis="Y"))
    return Body.build("trailer", pos=tuple(o.tolist()), mass=20.0, kinematic=True, color=PAINT, shapes=sh,
                      friction=(0.4, 0.35))


def _lid() -> Body:
    """The socket's spring-closed cover, shut: a plate hanging from the hinge over the mouth, its latch lip along the
    free edge pointing into the mouth, and a thumb tab below it. Origin on the hinge line."""
    o = HINGE
    rel = lambda p: np.asarray(p, float) - o  # noqa: E731
    xf = HINGE[0] - 0.0015  # the plate's back face
    sh = [_box(rel((xf - LID_T, Y_C - LID_W / 2, HINGE[2] - LID_L)), rel((xf, Y_C + LID_W / 2, HINGE[2] + 0.002)), GREY)]
    sh.append(_box(rel((xf, Y_C - 0.015, HINGE[2] - LID_L)), rel((xf + LIP_H, Y_C + 0.015, HINGE[2] - LID_L + 0.003)),
                   GREY))  # the latch lip
    sh.append(_box(rel((xf - LID_T - TAB_L, Y_C - LID_W / 2, HINGE[2] - LID_L - 0.0035)),
                   rel((xf - LID_T, Y_C + LID_W / 2, HINGE[2] - LID_L + 0.0025)), GREY))  # the thumb lip, full width
    sh.append(_box(rel((xf - LID_T - 0.0004, Y_C - 0.010, HINGE[2] - 0.022)), rel((xf - LID_T, Y_C + 0.010, HINGE[2] - 0.010)),
                   WHITE, False))  # the maker's label
    return Body.build("lid", pos=tuple(o.tolist()), mass=0.04, shapes=sh, color=GREY, friction=(0.15, 0.12), slick=True,
                      solver_iterations=32, damping=(0.0, 0.5))


# the cover's spring: closing, preloaded (its target sits past shut)
hinge("cover", "trailer", "lid", anchor=tuple(HINGE.tolist()), axis="Y", limits_deg=(0.0, LID_OPEN_MAX),
      spring=0.0004, damping=0.0002, target_deg=-25.0)


# -- the plug (its frame: origin at the face's centre, x into the socket) ----------------------------------------------
PLUG_START = np.array([X_H + PARK, Y_H, Z_C])


def _plug_shapes(rel):
    sh = [CylShape(tuple(rel((-(BARREL_L + 0.004) / 2, 0, 0)).tolist()), P_R, BARREL_L - 0.004, BLACK, axis="X"),
          CylShape(tuple(rel((-0.002, 0, 0)).tolist()), P_R - 0.0012, 0.004, BLACK, axis="X")]  # the nose, chamfered
    sh.append(CylShape(tuple(rel((0.0004, 0, 0)).tolist()), P_R - 0.001, 0.0008, (0.15, 0.15, 0.16), collide=False,
                       axis="X"))
    for k in range(7):  # the face's seven pins
        a = 2 * math.pi * k / 6
        y, z = (0.0, 0.0) if k == 6 else (0.011 * math.cos(a), 0.011 * math.sin(a))
        sh.append(CylShape(tuple(rel((0.001, y, z)).tolist()), 0.0022, 0.001, (0.80, 0.62, 0.30), collide=False, axis="X"))
    k0, k1, kh, kw = KEY_RIB
    sh.append(_box(rel((-k1, P_R - 0.0005, -kw / 2)), rel((-k0 - kw / 2, P_R + kh, kw / 2)), BLACK))  # the key rib (+y)
    d = kw / 2 ** 0.5  # its front, pointed (a square turned 45 degrees) so it finds the keyway
    sh.append(BoxShape(tuple(rel((-k0 - kw / 2, P_R + (kh - 0.0005) / 2, 0.0)).tolist()), (d, kh + 0.0005, d), BLACK,
                       quat_xyzw=(0.0, math.sin(math.pi / 8), 0.0, math.cos(math.pi / 8))))
    sh.append(CylShape(tuple(rel((-BAND_AT - 0.0015, 0, 0)).tolist()), P_R + 0.0002, 0.003, YELLOW, collide=False,
                       axis="X"))  # the yellow band
    # the latch boss on top: ramped in front (so the cover's lip rides up over it), square at the back; the ear below
    ramp_l = EAR_L - EAR_FLAT
    r0, x0 = P_R - 0.004, -BARREL_L + 0.002  # the ramp starts inside the barrel, so its front edge is not a step
    sh.append(bodykit.plate((x0, 0.0, r0), (-BARREL_L - ramp_l, 0.0, EAR_R - 0.0015), EAR_W, 0.003,
                            (-(EAR_R - 0.0015 - r0), 0.0, ramp_l + x0 + BARREL_L), BLACK))
    sh.append(_box(rel((-BARREL_L - EAR_L, -EAR_W / 2, P_R)), rel((-BARREL_L - ramp_l, EAR_W / 2, EAR_R)), BLACK))
    sh.append(_box(rel((-BARREL_L - ramp_l + 0.001, -EAR_W / 2, P_R)), rel((-BARREL_L - 0.007, EAR_W / 2, P_R + 0.004)),
                   BLACK))  # under the ramp (kept below its surface)
    sh.append(_box(rel((-BARREL_L - EAR_L, -EAR_W / 2, -EAR_R)), rel((-BARREL_L, EAR_W / 2, -P_R)), BLACK))
    for s in (-1.0, 1.0):
        sh.append(CylShape(tuple(rel((-BARREL_L - EAR_L / 2, 0, s * 0.024)).tolist()), 0.0035, EAR_W + 0.0004,
                           (0.25, 0.25, 0.27), collide=False, axis="Y"))
    sh.append(CylShape(tuple(rel((-BARREL_L - EAR_L / 2, 0, 0)).tolist()), P_R - 0.0003, EAR_L, BLACK, axis="X"))
    sh.append(CylShape(tuple(rel((-BARREL_L - EAR_L - BODY_L / 2, 0, 0)).tolist()), BODY_R, BODY_L, BLACK, axis="X"))
    gl = P_LEN - BARREL_L - EAR_L - BODY_L
    for j in range(4):  # the grommet's ribs
        sh.append(CylShape(tuple(rel((-BARREL_L - EAR_L - BODY_L - gl * (j + 0.5) / 4, 0, 0)).tolist()),
                           GROM_R + (0.0012 if j % 2 == 0 else 0.0), gl / 4, BLACK, axis="X"))
    return sh


def _plug() -> Body:
    o = PLUG_START
    rel = lambda p: np.asarray(p, float)  # noqa: E731  (shapes are given in the plug's own frame)
    return Body.build("plug", pos=tuple(o.tolist()), mass=P_MASS, shapes=_plug_shapes(rel), color=BLACK,
                      friction=(1.0, 0.9), solver_iterations=32, com=(-0.060, 0.0, 0.0))


# -- the holder on the tractor side, and the cord's hanger ---------------------------------------------------------------

def _tractor() -> Body:
    """The plug's holder (a lidless 7-way parking cup with a keyway) on a bracket post, and the cord's hanger post at
    the back of the cab. Kinematic."""
    o = np.array([X_H, Y_H, Z_C])
    rel = lambda p: np.asarray(p, float) - o  # noqa: E731
    sh = _ring(X_H, X_H + PARK + 0.003, Y_H, Z_C, BORE_R, WALL_T, 20, (0.18, 0.18, 0.20), gap_at=0.0, gap_w=KEY_W,
               rel=rel)
    sh.append(CylShape(tuple(rel((X_H + PARK + 0.0045, Y_H, Z_C)).tolist()), BORE_R + WALL_T, 0.003,
                       (0.18, 0.18, 0.20), axis="X"))
    sh.append(_box(rel((X_H + PARK + 0.006, Y_H - 0.015, TOP)), rel((X_H + PARK + 0.016, Y_H + 0.015, Z_C + 0.020)),
                   (0.18, 0.18, 0.20)))  # the bracket post
    sh.append(_box(rel((X_H + PARK - 0.03, Y_H - 0.05, TOP)), rel((X_H + PARK + 0.04, Y_H + 0.05, TOP + 0.006)),
                   (0.20, 0.20, 0.22)))  # its foot
    a = ANCHOR
    sh.append(_box(rel((a[0] - 0.012, a[1] - 0.012, TOP)), rel((a[0] + 0.012, a[1] + 0.012, a[2] + 0.015)),
                   (0.20, 0.20, 0.22)))  # the hanger post
    return Body.build("tractor", pos=tuple(o.tolist()), mass=10.0, kinematic=True, color=(0.2, 0.2, 0.22), shapes=sh,
                      friction=(0.5, 0.4))


def _cord_path(n: int):
    """The cord's knots from the plug's back to its tie-off: it hangs straight down from the plug, lies along the
    table in an S toward the tractor side, and rises to the tie-off post; n equal segments."""
    p0 = PLUG_START + np.array([-P_LEN, 0.0, 0.0])
    lay = TOP + CORD_R + 0.004
    poly = np.array([p0, p0 + np.array([-0.030, 0.0, -0.075]), [p0[0] - 0.040, p0[1] - 0.030, lay],
                     [0.080, -0.220, lay], [0.110, -0.360, lay], [0.170, -0.360, lay], ANCHOR])
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    out = []
    for k in range(n + 1):
        d = seg[-1] * k / n
        i = min(int(np.searchsorted(seg, d, side="right")) - 1, len(poly) - 2)
        t = (d - seg[i]) / (seg[i + 1] - seg[i])
        out.append(poly[i] + (poly[i + 1] - poly[i]) * t)
    return out


CORD_KNOTS = _cord_path(CORD_N)


def _cyl_mesh(a, b, r, n=10):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax = (b - a) / np.linalg.norm(b - a)
    u = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(ax, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    pts = [p + (u * math.cos(2 * math.pi * i / n) + v * math.sin(2 * math.pi * i / n)) * r for p in (a, b) for i in range(n)]
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    for i in range(1, n - 1):
        faces += [(0, i + 1, i), (n, n + i, n + i + 1)]
    return tuple(map(tuple, pts)), tuple(faces)


def _cord(k: int) -> Body:
    p0, p1 = CORD_KNOTS[k - 1], CORD_KNOTS[k]
    pts, faces = _cyl_mesh(np.zeros(3), p1 - p0, CORD_R)
    return Body.build(f"cord{k}", pos=tuple(p0.tolist()), mass=CORD_MASS / CORD_N, color=CORD_C,
                      shapes=[MeshShape(pts, faces, CORD_C, approx="convexHull")], friction=(0.35, 0.3),
                      solver_iterations=24, damping=(0.2, 0.5),
                      ignore=("plug",) if k == 1 else (f"cord{k - 1}",))


CORD_K, CORD_D = 0.03, 0.002  # the cord's bending stiffness at each knot (N m/rad) and damping (N m s/rad)
_c = CORD_KNOTS
ball("c0", "plug", "cord1", anchor=tuple(_c[0].tolist()), axis=tuple((_c[1] - _c[0]).tolist()), swing_stiffness=CORD_K,
     twist_stiffness=CORD_K, swing_damping=CORD_D, twist_damping=CORD_D)
for _k in range(1, CORD_N):
    ball(f"c{_k}", f"cord{_k}", f"cord{_k + 1}", anchor=tuple(_c[_k].tolist()), axis=tuple((_c[_k + 1] - _c[_k]).tolist()),
         swing_stiffness=CORD_K, twist_stiffness=CORD_K, swing_damping=CORD_D, twist_damping=CORD_D)
ball(f"c{CORD_N}", "tractor", f"cord{CORD_N}", anchor=tuple(_c[CORD_N].tolist()),
     axis=tuple((_c[CORD_N] - _c[CORD_N - 1]).tolist()), swing_damping=CORD_D, twist_damping=CORD_D)

OBJECTS = [_trailer(), _lid(), _tractor(), _plug(), *[_cord(k) for k in range(1, CORD_N + 1)]]


# -- followed after every control step ------------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"s": {}, "tcps": None, "steps": 0, "last": None}


_EP = _fresh()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


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


def lid_angle(r_lid) -> float:
    """How far the cover is open (degrees; 0 shut), from its rotation."""
    down = r_lid @ np.array([0.0, 0.0, -1.0])
    return math.degrees(math.atan2(-down[0], -down[2]))


def measure(pp, rp, pl, rl, tcps) -> dict:
    """The coupling, from the plug's and the cover's poses and the two fingertip centres."""
    face = pp
    axis = rp @ np.array([1.0, 0.0, 0.0])
    depth = float(face[0] - X_F)
    off = float(np.hypot(face[1] - Y_C, face[2] - Z_C))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0])))))
    # the cover's lip (its lowest point, at the free end) against the top latch ear's back face
    lip_x0 = pl + rl @ (np.array([HINGE[0] - 0.0015, Y_C, HINGE[2] - LID_L]) - HINGE)  # the lip's root, inner edge
    lip_tip = pl + rl @ (np.array([HINGE[0] - 0.0015 + LIP_H, Y_C, HINGE[2] - LID_L]) - HINGE)
    lip_low = min(float(lip_x0[2]), float(lip_tip[2]))
    lip_front = max(float(lip_x0[0]), float(lip_tip[0]))
    ear_back = float((pp + rp @ np.array([-BARREL_L - EAR_L, 0.0, 0.0]))[0])
    ear_top = float((pp + rp @ np.array([-BARREL_L - EAR_L / 2, 0.0, EAR_R]))[2])
    latched = lip_low < ear_top - 0.002 and lip_front <= ear_back + 0.0015 and lid_angle(rl) < 100.0
    mid = pp + rp @ np.array([-BARREL_L - EAR_L - BODY_L / 2, 0.0, 0.0])
    tip = pl + rl @ (np.array([HINGE[0] - 0.003, Y_C, HINGE[2] - LID_L]) - HINGE)
    near = min(min(float(np.linalg.norm(t - mid)), float(np.linalg.norm(t - tip))) for t in tcps) if tcps else 1.0
    up = rp @ np.array([0.0, 0.0, 1.0])
    pitch = math.degrees(math.atan2(float(axis[2]), float(axis[0])))
    yaw = math.degrees(math.atan2(float(axis[1]), float(axis[0])))
    roll = math.degrees(math.atan2(-float(up[1]), float(up[2])))
    return {"depth": depth, "off": off, "ang": ang, "roll": roll, "pitch": pitch, "yaw": yaw, "lid": lid_angle(rl), "home": depth >= HOME and off < 0.002 and ang < 4.0,
            "in_bore": depth >= IN_BORE and off < 0.004, "latched": latched, "released": near >= RELEASED,
            "lip_gap": lip_front - ear_back, "lip_low": lip_low - ear_top}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pp, rp = _pose(base, origin, "plug")
    pl, rl = _pose(base, origin, "lid")
    _EP["tcps"] = _tcps(base, origin)
    s = measure(pp, rp, pl, rl, _EP["tcps"])
    last = _EP.get("last")
    s["still"] = last is not None and float(np.linalg.norm(pp - last[0])) < 0.0003 and abs(s["lid"] - last[1]) < 0.3
    _EP["last"] = (pp, s["lid"])
    _EP["s"] = s
    _EP["steps"] += 1


# -- scoring ----------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """Judged at the end, on live states. 1.0: the plug fully home (its face against the contact insert: the yellow
    band hidden), the cover down on it with its lip behind the plug's top latch ear, and neither hand on the plug or
    the cover; 0.75: the same, still held; 0.5: home but not latched; 0.25: the plug's barrel in the socket's bore,
    not home; else 0.0."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["home"]:
        if s["latched"]:
            return 1.0 if s["released"] else 0.75
        return 0.5
    return 0.25 if s["in_bore"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand, fingers closed, gets under the cover's thumb lip and swings the cover up about its hinge to about
# 100 degrees, and holds it there. The right hand takes the plug by its body from behind and above, draws it out of
# its holder, brings it square in front of the socket under the raised cover and pushes it fully home, and holds it
# there. The left slides out from under the cover sideways, so the cover drops behind the plug's latch ear; then the
# right lets go. Both go home.

LEAD_S, LEAD_STEPS = 3.0, 15
GRIP_PITCH = math.radians(65.0)  # the right hand comes down onto the plug's body from behind, 65 degrees down
R_OPEN = -0.030  # the right's pads 60 mm apart round the 42 mm body
LID_HOLD = 102.0  # the cover is held open this far (degrees): its lip then clears the plug's latch ear
PAD_W, PAD_T = 0.020, 0.010  # the left's closed pads, flat: half their width (along x), half their thickness
PRESS = 0.020  # the paddle is aimed this far into the cover (the arm gives)
PUSH_PAST = 0.008  # the plug is pushed this far past home (the arm gives)
NOSE_IN = 0.016  # first, the plug's face aimed this far in, under the raised cover
SAG = np.array([0.0, 0.005, 0.016])  # the right arm gives this much under the plug and its cord: aim this far off


def _frame(approach, close) -> np.ndarray:
    """A TCP rotation (world): fingers pointing along ``approach``, closing along ``close`` (the gripper's x axis, as
    in yam_arena: grip_rotation(0) closes along world y). Each fingertip is 10 mm thick along ``close`` and tapers
    from about 40 mm wide (along the third axis) to 15 mm at its tip, 12.8 mm past the TCP."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - np.dot(close, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _good_ik(arm: str, pos, rot, near=None) -> np.ndarray:
    """An IK solution for the TCP pose: the one closest to ``near`` (joint angles) among those with some room to the
    joint limits, else the one with the most room."""
    import itertools

    from yam_arena import ik, station

    sols = []
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.3, 1.9), (1.0, 1.7), (-1.0, 0.0), (0.0,), (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            sols.append((q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
    if not sols:
        raise ValueError(f"no IK solution for the {arm} hand at {np.round(pos, 4)}")
    if near is not None:
        roomy = [s for s in sols if s[1] > 0.15] or sols
        return min(roomy, key=lambda s: float(np.linalg.norm(s[0] - near)))[0]
    return max(sols, key=lambda s: s[1])[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs=LEAD_S, steps=LEAD_STEPS) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(secs / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, r)

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def lead(arm, pos, rot, finger, label, secs=LEAD_S):
        p0, _, r0 = cur[arm]
        q0 = np.array(READY, float) if not plan[arm] else _good_ik(arm, p0, r0, near=np.array(READY, float))
        plan[arm] += _joint_path(arm, q0, _good_ik(arm, pos, rot, near=q0), label, finger, secs)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        if p[0] > 0.24:  # back away from the trailer first, where the hand can
            back = np.array([0.22, p[1], p[2] + 0.01])
            try:
                _good_ik(arm, back, r)
                go(arm, 1.0, back, f, "back away from the trailer", n=3)
                p, f, r = cur[arm]
            except ValueError:
                pass
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r, near=np.array(READY, float)), np.array(READY, float), label, f)
        cur[arm] = (np.asarray(plan[arm][-1].pos, float), f, plan[arm][-1].yaw)

    # the left: its closed fingertips as a paddle (20 mm thick, pointing at the plug's path from the left, the
    # gripper's body clear of the wall) come up under the cover's thumb lip and lift the cover round its hinge to a
    # little past level, the paddle under the cover's left edge, clear of the plug's path; the cover rests on it,
    # pressed by its spring
    rot_l = _frame((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
    xf = HINGE[0] - 0.0015
    lid_pts = [np.array([xf, Y_C, HINGE[2] - t]) for t in np.linspace(0.0, LID_L, 35)]  # the plate's inner face
    for x in np.linspace(xf - LID_T - TAB_L, xf - LID_T, 7):  # the tab's underside
        lid_pts.append(np.array([x, Y_C, HINGE[2] - LID_L - 0.0035]))
    for x in np.linspace(xf, xf + LIP_H, 5):  # the lip's lower face
        lid_pts.append(np.array([x, Y_C, HINGE[2] - LID_L]))
    xc = X_F - 0.026
    def paddle_under(deg, y_tip, press=0.0):
        pts = np.array([HINGE + _rot_y(deg) @ (c - HINGE) for c in lid_pts])
        span = pts[np.abs(pts[:, 0] - xc) <= PAD_W]
        return np.array([xc, y_tip + 0.0128, float(span[:, 2].min()) - PAD_T - 0.001 + press])
    y_tab = Y_C + 0.023  # pad tips under the cover's left edge (the gripper's body then clears the flange)
    p_ready = cur["left"][0]
    go("left", 1.2, p_ready + np.array([-0.120, -0.020, -0.020]), FINGER_CLOSED, "back, clear of the trailer", n=3)
    lead("left", paddle_under(0.0, y_tab) + np.array([-0.030, 0.060, -0.030]), rot_l, FINGER_CLOSED, "beside and below the tab")
    go("left", 1.0, paddle_under(0.0, y_tab, -0.030), FINGER_CLOSED, "under the cover's lip, 3 cm below it", n=2)
    wait("left", 0.5, "steady")
    # straight up: the cover's lip rides on the paddle's top, so the cover opens as the paddle rises
    z0 = paddle_under(0.0, y_tab)[2]
    z1 = paddle_under(LID_HOLD, y_tab)[2]
    for z in np.linspace(z0, z1, 16)[1:]:
        go("left", 0.45, np.array([xc, y_tab + 0.0128, z + PRESS]), FINGER_CLOSED, "lift the cover: the paddle straight up")
    wait("left", 0.4, "hold the cover up")

    # the right: the plug from its holder, square to the socket, fully home
    a = np.array([math.cos(GRIP_PITCH), 0.0, -math.sin(GRIP_PITCH)])
    rot_r = _frame(a, (0.0, -1.0, 0.0))  # pads either side of the plug
    g = 0.100  # the grip point, back from the plug's face (on its body): the fingers stay clear of the left's paddle
    face0 = PLUG_START
    grip0 = face0 - np.array([g, 0.0, 0.0])
    lead("right", grip0 - a * 0.045, rot_r, R_OPEN, "behind and above the plug's body")
    go("right", 1.2, grip0 + np.array([0.0, 0.0, -0.004]), R_OPEN, "down round the plug's body", n=3)
    wait("right", 0.8, "steady")
    go("right", 0.6, grip0 + np.array([0.0, 0.0, -0.004]), FINGER_CLOSED, "take the plug")
    go("right", 1.2, grip0 - np.array([0.042, 0.0, 0.0]), FINGER_CLOSED, "draw it out of its holder", n=3)
    face_pre = np.array([X_F - 0.014, Y_C, Z_C])
    grip_pre = face_pre - np.array([g, 0.0, 0.0])
    go("right", 1.0, grip0 - np.array([0.042, 0.0, -0.012]), FINGER_CLOSED, "lift it clear")
    wait("right", max(0.0, dur("left") - dur("right") + 1.0), "wait for the cover to be up")
    go("right", 2.0, grip_pre + SAG + np.array([0.0, 0.0, 0.012]), FINGER_CLOSED, "over to the socket", n=4)
    go("right", 0.8, grip_pre + SAG, FINGER_CLOSED, "square in front of the socket, under the cover")
    wait("right", max(1.0, dur("left") - dur("right")), "steady")
    grip_nose = np.array([X_F + NOSE_IN, Y_C, Z_C]) - np.array([g, 0.0, 0.0]) + SAG
    go("right", 1.5, grip_nose, FINGER_CLOSED, "start it into the socket under the cover", n=3)
    wait("right", 0.4, "hold it there")
    grip_home = np.array([X_F + DEPTH + PUSH_PAST, Y_C, Z_C]) - np.array([g, 0.0, 0.0]) + SAG
    go("right", 2.0, grip_home, FINGER_CLOSED, "push it in, fully home, under the raised cover", n=4)
    wait("right", 0.6, "hold it home")
    # the left slides out sideways from under the cover: the cover drops behind the plug's latch ear
    wait("left", dur("right") - dur("left"), "hold the cover up")
    p_l = cur["left"][0]
    go("left", 0.8, p_l + np.array([0.0, 0.060, -0.004]), FINGER_CLOSED, "slide out sideways from under the cover", n=2)
    go("left", 0.6, p_l + np.array([0.0, 0.060, 0.030]), FINGER_CLOSED, "clear")
    home("left")
    # the right lets go once the cover is down, and backs off
    wait("right", dur("left") - dur("right") - LEAD_S + 0.6, "hold it home while the cover latches")
    go("right", 0.6, grip_home, R_OPEN, "let go")
    go("right", 1.0, grip_home - a * 0.050, R_OPEN, "back off")
    home("right")
    end = max(dur("left"), dur("right")) + 2.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
