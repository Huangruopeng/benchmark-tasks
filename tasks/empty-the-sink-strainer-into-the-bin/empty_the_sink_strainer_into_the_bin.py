"""A stainless single-bowl kitchen sink (25 x 22 in top-mount, bowl 21 x 15-3/4 in and 8 in deep) is set into the
counter (the station table, cut out under it) on the right, with a gooseneck tap on its back deck. In its 3-1/2 in
drain sits a fixed-post strainer basket holding nine food scraps (carrot and potato peel, rice, peas, broccoli, a piece
of apple core) that cling to it as wet scraps do: turned over, they stay in it; a firm shake frees them. A 1.8 litre
food-waste caddy, its lid off, stands on the counter just left of the sink. A real trial uses stand-ins printed in ABS
at the scraps' sizes and masses, which the simulated scraps copy, dipped in a cornstarch paste that makes them cling.

Task: empty the scraps from the strainer into the caddy and put the strainer back in the drain, without letting any
scraps go down the drain or onto the counter.

Run it:  inspect-robots run --embodiment yam_arena -E task=empty_the_sink_strainer_into_the_bin.py \\
           --instruction "..." --scorer task_score ...
Checks:  SINK_TEST=noshake (the demo, turned over without a shake: the scraps stay stuck); SINK_TEST=binbasket (the
         basket set down in the caddy with the scraps still stuck in it: none counts); SINK_TEST=gentle (the
         previous revision's gentle shakes instead); SINK_TEST=shakes (every bond kept on, a series of shakes: the
         cling's loads, with SINK_DEBUG=1); SINK_TEST=reseat (the full basket put back in the drain and lifted out
         again first); SINK_ONEHAND=1 (the right hand alone); python empty_the_sink_strainer_into_the_bin.py (the drain
         rule's geometry and the cling rule, no simulator).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation, segment)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()


def _quat_from_matrix(m) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, by Shepperd's method. bodykit's own conversion picks wrong signs for
    rotations near 180 deg (a wall facing -x came out tilted the wrong way and left a slot at the caddy's floor), so
    plate() is given this one."""
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

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PHYSICS_DT = 1.0 / 240.0

# -- the sink (metres, world frame): a 25 x 22 in top-mount single bowl, 20 gauge stainless -------------------------
RIM_X0, RIM_X1 = 0.110, 0.669  # the rim's outer edge, front to back: 559 mm (22 in)
RIM_Y = 0.3175  # ... side to side: +-317.5 mm (25 in)
RIM_T = 0.003  # the rim stands 3 mm proud of the counter
BX0, BX1 = 0.148, 0.548  # the bowl's inside, front to back: 400 mm (15-3/4 in); a 38 mm front rim, a 121 mm back deck
BY = 0.2665  # ... side to side: +-266.5 mm (21 in)
DEPTH = 0.200  # the bowl is 200 mm (7-7/8 in) deep, rim to floor
FLOOR = TOP - DEPTH
WALL = 0.010  # collision thickness of the bowl's walls and floor (outward; the real steel is 0.9 mm)
CHAMFER = 0.025  # the bowl's coved corners and floor edge, drawn as 25 mm chamfers
CUT = 0.010  # the counter is cut out this far round the bowl's inside
SINK_Y = -0.300  # the sink's middle line (y): the right half of the counter, so the caddy beside it is near the middle
DRAIN = np.array([(BX0 + BX1) / 2, SINK_Y])  # the drain in the middle of the floor
# -- the strainer body (3-1/2 in kitchen basket strainer): flange 114 mm across, bore 82 mm, 79 mm deep ----------------
FLANGE_R, BORE_R, BODY_DEPTH = 0.057, 0.041, 0.079
FLANGE_TOP = 0.0015  # the flange stands 1.5 mm proud of the sink floor at its edge ...
FLANGE_IN = -0.004  # ... and slopes down to 4 mm below it at the bore (a lead-in)
CROSSBAR = -0.049  # the top of the crossbar the basket's stem rests on, below the sink floor
THROAT = CROSSBAR - 0.013  # the body narrows to the tailpiece (1-1/2 in OD) here
TAIL_R = 0.0175
HUB_R, HUB_H = 0.0055, 0.005  # the crossbar's hub: a socket 11 mm across and 5 mm deep that the post stands in
# -- the strainer basket (a fixed-post replacement basket, 3-1/8 in): origin at the centre of its cup's underside -----
B_RIM_R = 0.0397  # 79.4 mm across its rolled rim
B_FLOOR_R = 0.030  # the cup tapers to a 60 mm floor
B_RIM_Z = 0.028  # the rim's top, above the cup's underside (a 28 mm deep cup)
B_WALL_T = 0.0015
SEAL_R, SEAL_H = 0.026, 0.006  # the rubber seal under the floor
STEM_R = 0.003
STEM_L = 0.010  # the post below the seal
POST_R = 0.003
KNOB_R, KNOB_Z0, KNOB_Z1 = 0.008, 0.032, 0.041  # the knurled knob: 16 mm across, its top 13 mm above the rim
B_MASS = 0.068
SEATED_Z = FLOOR + CROSSBAR + SEAL_H + STEM_L  # the basket's origin when its stem rests on the crossbar
B_START = np.array([DRAIN[0], DRAIN[1], SEATED_Z])
SIDES = 24
# -- the caddy (1.8 l sink-side food-waste caddy, ABS): origin at the centre of its underside ---------------------
CAD_L, CAD_W, CAD_H = 0.221, 0.127, 0.095  # outside at the top (its lid makes it 107 mm tall when shut)
CAD_L0, CAD_W0 = 0.205, 0.111  # outside at the bottom (tapered walls)
CAD_T = 0.0025
CAD_MASS = 0.42
CAD_XY = np.array([0.295, SINK_Y + RIM_Y + 0.024 + CAD_L / 2])  # 24 mm left of the sink's rim, its long side along y
# -- the tap: a single-lever gooseneck on the back deck, its spout swung 50 deg to the left ----------------------
TAP_XY = np.array([0.610, SINK_Y])
SPOUT_REACH, SPOUT_TOP, SPOUT_OUT = 0.21, 0.405, 0.25
SPOUT_AZ = math.radians(180.0 - 50.0)  # the spout points over the bowl, swung toward the left

STEEL, STEEL_FLOOR, STEEL_DARK = (0.74, 0.76, 0.79), (0.67, 0.69, 0.72), (0.30, 0.31, 0.33)
CHROME, RUBBER, HOLE = (0.70, 0.71, 0.74), (0.08, 0.08, 0.09), (0.12, 0.12, 0.13)
CADDY, CADDY_IN, LID = (0.48, 0.64, 0.57), (0.36, 0.48, 0.43), (0.53, 0.69, 0.62)
CARROT, POTATO, RICE, PEA, BROC, APPLE, SEED = ((0.93, 0.47, 0.13), (0.78, 0.66, 0.45), (0.96, 0.95, 0.90),
                                                 (0.38, 0.62, 0.20), (0.20, 0.42, 0.18), (0.95, 0.90, 0.72),
                                                 (0.30, 0.18, 0.10))

SUCCESS_HOLD_S = 1.0
HOME_WITHIN = 0.05


# -- the counter: the station table, cut out under the sink -------------------------------------------------------

def _table_pieces(station) -> list[tuple[str, tuple, tuple]]:
    """The station table as four slabs round the sink's cut-out: (name, size, centre)."""
    cx, cy, cz = station.TABLE_CENTER
    sx, sy, sz = station.TABLE_SIZE
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    hx0, hx1, hy0, hy1 = BX0 - CUT, BX1 + CUT, SINK_Y - BY - CUT, SINK_Y + BY + CUT
    out = []

    def slab(name, xa, xb, ya, yb):
        out.append((name, (xb - xa, yb - ya, sz), ((xa + xb) / 2, (ya + yb) / 2, cz)))

    slab("table", x0, hx0, y0, y1)  # in front of the sink (keeps the room's name for the table)
    slab("table_back", hx1, x1, y0, y1)
    slab("table_right", hx0, hx1, y0, hy0)
    slab("table_left", hx0, hx1, hy1, y1)
    return out


def _cut_table(scene) -> None:
    """Swap the station's table for four slabs that leave a hole for the sink's bowl. Changes only the scene built for
    this task file (yam_arena itself is not edited)."""
    if getattr(scene, "_sink_cut", False):
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
    scene._sink_cut = True


_plain_scene_patch = bodykit._TARGETS["yam_arena.scene"]


def _scene_patch(module) -> None:
    _plain_scene_patch(module)
    _cut_table(module)


bodykit._TARGETS["yam_arena.scene"] = _scene_patch
if "yam_arena.scene" in sys.modules:
    _cut_table(sys.modules["yam_arena.scene"])


# -- shapes -----------------------------------------------------------------------------------------------------

def _ring(n, r_in, r_out, z_in, z_out, t, color, collide=True, centre=(0.0, 0.0, 0.0)) -> list:
    """A ring (or cone) of n flat panels t thick, from radius r_in at height z_in to r_out at z_out (its top surface),
    about a vertical axis through centre. Each panel is as wide as the widest chord, so neighbours overlap."""
    cx, cy, cz = centre
    width = 2 * max(r_in, r_out) * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = np.array([math.cos(a), math.sin(a), 0.0])
        p1 = np.array([cx, cy, cz]) + r_in * u + [0.0, 0.0, z_in]
        p2 = np.array([cx, cy, cz]) + r_out * u + [0.0, 0.0, z_out]
        d = p2 - p1
        nrm = np.cross(d, np.cross([0.0, 0.0, 1.0], u))  # in the radial plane, square to the panel, upward
        if nrm[2] < 0:
            nrm = -nrm
        if abs(nrm[2]) < 1e-9 and np.dot(nrm, u) < 0:
            nrm = -nrm
        nrm = nrm / np.linalg.norm(nrm)
        out.append(plate(tuple(p1 - nrm * t / 2), tuple(p2 - nrm * t / 2), width, t, tuple(nrm), color, collide))
    return out


def _wall(n, r, z0, z1, t, color, collide=True, centre=(0.0, 0.0, 0.0)) -> list:
    """A vertical cylinder wall of n panels, its inside at radius r."""
    cx, cy, cz = centre
    width = 2 * (r + t) * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        m = r + t / 2
        out.append(plate((cx + m * u[0], cy + m * u[1], cz + z0), (cx + m * u[0], cy + m * u[1], cz + z1), width, t, u,
                         color, collide))
    return out


def _box(xa, xb, ya, yb, za, zb, color, origin, collide=True) -> BoxShape:
    ox, oy, oz = origin
    return BoxShape(((xa + xb) / 2 - ox, (ya + yb) / 2 - oy, (za + zb) / 2 - oz), (xb - xa, yb - ya, zb - za), color,
                    collide=collide)


def _sink() -> Body:
    """The bowl and its rim, a fixed body. Origin: on the counter at the drain's centre. (Shapes are laid out about
    the sink's own middle line, y = 0, then the body is placed at SINK_Y.)"""
    o = (DRAIN[0], 0.0, TOP)
    s = []
    hole = BORE_R + 0.0005  # the floor stops at the bore; the strainer's flange covers the corners
    # the floor (top at FLOOR), round a square hole at the drain
    s += [_box(BX0, BX1, -BY, -hole, FLOOR - WALL, FLOOR, STEEL_FLOOR, o),
          _box(BX0, BX1, hole, BY, FLOOR - WALL, FLOOR, STEEL_FLOOR, o),
          _box(BX0, DRAIN[0] - hole, -hole, hole, FLOOR - WALL, FLOOR, STEEL_FLOOR, o),
          _box(DRAIN[0] + hole, BX1, -hole, hole, FLOOR - WALL, FLOOR, STEEL_FLOOR, o)]
    # the four walls, rim to floor
    s += [_box(BX0 - WALL, BX0, -BY - WALL, BY + WALL, FLOOR - WALL, TOP, STEEL, o),
          _box(BX1, BX1 + WALL, -BY - WALL, BY + WALL, FLOOR - WALL, TOP, STEEL, o),
          _box(BX0, BX1, -BY - WALL, -BY, FLOOR - WALL, TOP, STEEL, o),
          _box(BX0, BX1, BY, BY + WALL, FLOOR - WALL, TOP, STEEL, o)]
    # coves: 45 deg panels along the floor's edges and up the four corners
    c = CHAMFER
    for (p, q, n_) in (((BX0, -BY), (BX1, -BY), (0.0, 1.0)), ((BX0, BY), (BX1, BY), (0.0, -1.0)),
                       ((BX0, -BY), (BX0, BY), (1.0, 0.0)), ((BX1, -BY), (BX1, BY), (-1.0, 0.0))):
        a = np.array([p[0], p[1], FLOOR]) + np.array([n_[0], n_[1], 0.0]) * c / 2 + [0.0, 0.0, c / 2]
        b = np.array([q[0], q[1], FLOOR]) + np.array([n_[0], n_[1], 0.0]) * c / 2 + [0.0, 0.0, c / 2]
        nrm = np.array([n_[0], n_[1], 1.0]) / math.sqrt(2)
        a -= nrm * 0.002
        b -= nrm * 0.002
        s.append(plate(tuple(a - o), tuple(b - o), c * math.sqrt(2), 0.004, tuple(nrm), STEEL_FLOOR))
    for cx_, cy_, sx, sy in ((BX0, -BY, 1, 1), (BX0, BY, 1, -1), (BX1, -BY, -1, 1), (BX1, BY, -1, -1)):
        nrm = np.array([sx, sy, 0.0]) / math.sqrt(2)
        m = np.array([cx_ + sx * c / 2, cy_ + sy * c / 2, 0.0]) - nrm * 0.002
        s.append(plate((m[0] - o[0], m[1] - o[1], FLOOR + c / 2 - o[2]), (m[0] - o[0], m[1] - o[1], TOP - o[2]),
                       c * math.sqrt(2), 0.004, tuple(nrm), STEEL))
    # the rim on the counter, round the bowl
    s += [_box(RIM_X0, BX0 - WALL, -RIM_Y, RIM_Y, TOP, TOP + RIM_T, STEEL, o),
          _box(BX1 + WALL, RIM_X1, -RIM_Y, RIM_Y, TOP, TOP + RIM_T, STEEL, o),
          _box(BX0 - WALL, BX1 + WALL, -RIM_Y, -BY - WALL, TOP, TOP + RIM_T, STEEL, o),
          _box(BX0 - WALL, BX1 + WALL, BY + WALL, RIM_Y, TOP, TOP + RIM_T, STEEL, o)]
    # a drawn edge where the rim turns down into the bowl
    for xa, xb, ya, yb in ((BX0 - WALL, BX0 + 0.001, -BY, BY), (BX1 - 0.001, BX1 + WALL, -BY, BY),
                           (BX0, BX1, -BY - WALL, -BY + 0.001), (BX0, BX1, BY - 0.001, BY + WALL)):
        s.append(_box(xa, xb, ya, yb, TOP + RIM_T - 0.0005, TOP + RIM_T + 0.0005, (0.86, 0.87, 0.89), o,
                      collide=False))
    return Body.build("sink", pos=(DRAIN[0], DRAIN[1], TOP), mass=12.0, kinematic=True, color=STEEL, shapes=s, friction=(0.45, 0.4))


def _strainer_body() -> Body:
    """The drain's strainer body, a fixed body: flange, bore, crossbar, throat and the tailpiece below. Origin: at the
    drain's centre, on the sink floor."""
    s = []
    s += _ring(SIDES, BORE_R, FLANGE_R + 0.003, FLANGE_IN, FLANGE_TOP, 0.002, CHROME)  # the flange (a lead-in)
    s += _wall(SIDES, BORE_R, CROSSBAR - 0.006, FLANGE_IN + 0.001, 0.002, STEEL_DARK)  # the bore
    s += _ring(SIDES, TAIL_R, BORE_R + 0.001, THROAT, CROSSBAR - 0.006, 0.002, STEEL_DARK)  # its floor, to the throat
    s += [BoxShape((0.0, 0.0, CROSSBAR - 0.002), (2 * BORE_R, 0.006, 0.004), STEEL_DARK),  # the crossbar
          BoxShape((0.0, 0.0, CROSSBAR - 0.002), (0.006, 2 * BORE_R, 0.004), STEEL_DARK)]
    s += _ring(8, HUB_R, HUB_R + 0.003, CROSSBAR + HUB_H, CROSSBAR + HUB_H + 0.002, 0.002, STEEL_DARK)  # its hub's lip
    s += _wall(8, HUB_R, CROSSBAR - 0.001, CROSSBAR + HUB_H, 0.002, STEEL_DARK)  # the hub: a socket for the post
    s += _wall(16, TAIL_R, THROAT - 0.12, THROAT, 0.002, (0.70, 0.55, 0.25))  # the tailpiece, under the counter
    return Body.build("strainer_body", pos=(DRAIN[0], DRAIN[1], FLOOR), mass=1.0, kinematic=True, color=STEEL_DARK,
                      shapes=s, friction=(0.45, 0.4))


def _basket() -> Body:
    """The strainer basket: a tapered perforated steel cup with a rolled rim, a post through its middle with a knurled
    knob on top, and a rubber seal with the post's stem below. Origin: the centre of the cup's underside."""
    s = [CylShape((0.0, 0.0, 0.001), B_FLOOR_R, 0.002, STEEL)]  # the floor
    s += _ring(SIDES, B_FLOOR_R - B_WALL_T, B_RIM_R - 0.003, 0.002, B_RIM_Z - 0.002, B_WALL_T, STEEL)  # the wall
    s += _wall(SIDES, B_RIM_R - 0.0032, B_RIM_Z - 0.003, B_RIM_Z, 0.0032, STEEL)  # the rolled rim
    s += [CylShape((0.0, 0.0, -SEAL_H / 2), SEAL_R, SEAL_H, RUBBER),  # the seal
          CylShape((0.0, 0.0, -SEAL_H - STEM_L / 2), STEM_R, STEM_L, CHROME),  # the stem below it
          CylShape((0.0, 0.0, (0.002 + KNOB_Z0) / 2), POST_R, KNOB_Z0 - 0.002, CHROME),  # the post
          CylShape((0.0, 0.0, (KNOB_Z0 + KNOB_Z1) / 2), KNOB_R, KNOB_Z1 - KNOB_Z0, CHROME)]  # the knob
    for k in range(16):  # the knob's knurling (drawn)
        a = 2 * math.pi * k / 16
        s.append(BoxShape((KNOB_R * math.cos(a), KNOB_R * math.sin(a), (KNOB_Z0 + KNOB_Z1) / 2),
                          (0.0012, 0.0012, KNOB_Z1 - KNOB_Z0 - 0.002), (0.62, 0.63, 0.65), yaw=a, collide=False))
    for ring_r, n in ((0.012, 8), (0.021, 14)):  # the floor's perforations (drawn)
        for k in range(n):
            a = 2 * math.pi * (k + 0.5) / n
            s.append(CylShape((ring_r * math.cos(a), ring_r * math.sin(a), 0.00205), 0.0018, 0.0002, HOLE, collide=False))
    for z, n in ((0.009, 20), (0.019, 22)):  # the wall's perforations (drawn on the inside)
        r = B_FLOOR_R + (B_RIM_R - 0.003 - B_FLOOR_R) * (z - 0.002) / (B_RIM_Z - 0.004) - B_WALL_T - 0.0002
        for k in range(n):
            a = 2 * math.pi * (k + 0.25) / n
            s.append(BoxShape((r * math.cos(a), r * math.sin(a), z), (0.0004, 0.0034, 0.0034), HOLE, yaw=a,
                              collide=False))
    return Body.build("basket", pos=tuple(B_START), mass=B_MASS, shapes=s, color=STEEL, friction=(0.4, 0.35),
                      com=(0.0, 0.0, 0.008), solver_iterations=32, contact_offset=0.0015)


# the food scraps: (name, kind, local position in the basket, yaw, mass kg)
SCRAPS = [
    ("carrot_peel_1", "peel", (0.0, -0.0135, 0.0032), 0.0, 0.00058, (0.034, 0.011, 0.0015), CARROT),
    ("carrot_peel_2", "peel", (0.0, 0.0130, 0.0032), 0.0, 0.00046, (0.030, 0.010, 0.0015), CARROT),
    ("pea_1", "pea", (-0.020, 0.0, 0.0062), 0.0, 0.00027, 0.004, PEA),
    ("pea_2", "pea", (0.020, 0.0, 0.0062), 0.0, 0.00027, 0.004, PEA),
    ("potato_peel", "peel", (0.0, -0.0128, 0.0061), 0.0, 0.00075, (0.026, 0.018, 0.0015), POTATO),
    ("rice_1", "rice", (0.0005, 0.0160, 0.0095), 0.0, 0.00080, 0.0045, RICE),
    ("broccoli", "broc", (-0.0125, -0.0100, 0.0165), 0.0, 0.0012, None, BROC),
    ("apple_core", "apple", (0.0140, 0.0100, 0.0215), 0.0, 0.0030, (0.022, 0.014, 0.012), APPLE),
    ("rice_2", "rice", (0.0080, -0.0140, 0.0140), 0.6, 0.00055, 0.004, RICE),
]
SCRAP_NAMES = [n for n, *_ in SCRAPS]

# -- cling: wet, starchy scraps stick to the basket ---------------------------------------------------------------
# Each scrap is held to the basket where it lies by a bond (a PhysX fixed joint), which lets go for good once the
# cling it would have to supply exceeds CLING_G times the scrap's weight. The force the basket must put on a bonded
# scrap is m (a - g): its weight plus whatever it takes to make the scrap follow the basket. In the basket's frame, a
# push into the cup's floor (+z, toward the mouth) is carried by contact, and sideways shear up to CLING_MU times that
# push by friction; the rest, any pull away from the floor and the shear beyond friction, is the cling's load. It is
# averaged over CLING_AVG physics steps (12.5 ms). Held still upright the load is 0; on its side (90 deg) or upside
# down it is 1.0 x the weight, so tipping alone leaves the scraps stuck. Lifting it, carrying it and turning it over
# stay well under CLING_G; a firm shake (a quick stroke stopped short) or a knock goes over, and the scraps come
# loose. Calibrated to the real stand-ins' cling check at each session: on its side and upside down, held still,
# every piece stays; at one firm shake, every piece falls.
G = 9.81
CLING_G = 2.0
CLING_MU = 0.40  # the scraps' friction on the basket's steel
CLING_AVG = 3
CLING_LOOSE_M, CLING_LOOSE_DEG = 0.0015, 6.0  # (debug) a bonded scrap this far off its place has slipped
_TEST = os.environ.get("SINK_TEST", "")
_SCRAP_BOND = {n: (np.asarray(loc, float), yaw, m) for n, _k, loc, yaw, m, *_ in SCRAPS}
_CLING_LIMIT = math.inf if _TEST == "shakes" else CLING_G  # SINK_TEST=shakes keeps every bond, to measure the loads
_CLING_PATHS: dict[str, str] = {}
_CLING_ATTRS: dict[str, object] = {}


def cling_load(f_basket) -> float:
    """The cling's load, in multiples of the scrap's weight, for f_basket: the force per unit mass the basket must put on
    the scrap, a - g, in the basket's frame (m/s2). A push into the floor (+z) is carried by contact and sideways shear
    up to CLING_MU times that push by friction; the pull away from the floor and the shear beyond friction remain."""
    push = float(f_basket[2])
    pull = max(0.0, -push)
    shear = max(0.0, math.hypot(float(f_basket[0]), float(f_basket[1])) - CLING_MU * max(0.0, push))
    return math.hypot(pull, shear) / G


def _scrap(name, kind, local, yaw, mass, dims, color) -> Body:
    pos = B_START + np.asarray(local)
    if kind == "peel":
        shapes = [BoxShape((0.0, 0.0, 0.0), dims, color)]
        fric = (0.4, 0.35)
    elif kind == "pea":
        shapes = [SphereShape((0.0, 0.0, 0.0), dims, color)]
        fric = (0.4, 0.35)
    elif kind == "rice":  # a clump of cooked rice: two lumps stuck together
        shapes = [SphereShape((-dims * 0.95, 0.0, 0.0), dims, color), SphereShape((dims * 0.95, 0.0, 0.0), dims, color)]
        fric = (0.45, 0.4)
    elif kind == "broc":  # a broccoli floret: its head and a short stalk along -x
        shapes = [SphereShape((0.0015, 0.0, 0.0), 0.0075, color),
                  CylShape((-0.0085, 0.0, 0.0), 0.0032, 0.007, (0.55, 0.70, 0.40), axis="X")]
        fric = (0.45, 0.4)
    else:  # a piece of apple core: flesh with the dark seeds showing
        shapes = [BoxShape((0.0, 0.0, 0.0), dims, color),
                  BoxShape((0.0, 0.0, dims[2] / 2 + 0.0002), (0.006, 0.0035, 0.0004), SEED, collide=False),
                  BoxShape((0.0065, 0.0, dims[2] / 2 + 0.0002), (0.005, 0.003, 0.0004), SEED, collide=False)]
        fric = (0.4, 0.35)
    return Body.build(name, pos=tuple(pos), mass=mass, shapes=shapes, yaw=yaw, color=color, friction=fric,
                      damping=(0.05, 0.2), contact_offset=0.002, solver_iterations=32)


def _caddy() -> Body:
    """The 1.8 l sink-side food-waste caddy (ABS, a steel handle folded down), its lid taken off. Built with its long
    side along its own x, then turned 90 deg so the long side runs along the counter's edge (y). Origin: the centre of
    its underside."""
    s = [BoxShape((0.0, 0.0, 0.0015), (CAD_L0 - 0.004, CAD_W0 - 0.004, 0.003), CADDY_IN)]  # the floor
    hl0, hl1, hw0, hw1 = CAD_L0 / 2, CAD_L / 2, CAD_W0 / 2, CAD_W / 2
    for sgn in (1, -1):  # tapered walls: the two long sides, then the two ends
        s.append(plate((0.0, sgn * (hw0 - CAD_T / 2), 0.0), (0.0, sgn * (hw1 - CAD_T / 2), CAD_H), CAD_L, CAD_T,
                       (0.0, sgn, 0.0), CADDY))
        s.append(plate((sgn * (hl0 - CAD_T / 2), 0.0, 0.0), (sgn * (hl1 - CAD_T / 2), 0.0, CAD_H), CAD_W, CAD_T,
                       (sgn, 0.0, 0.0), CADDY))
        # each wall is backed outside by an unseen 8 mm slab, so a fast pea can't pass through the thin plastic
        s.append(plate((0.0, sgn * (hw0 + 0.004), 0.0), (0.0, sgn * (hw1 + 0.004), CAD_H - 0.002), CAD_L + 0.016,
                       0.008, (0.0, sgn, 0.0), None))
        s.append(plate((sgn * (hl0 + 0.004), 0.0, 0.0), (sgn * (hl1 + 0.004), 0.0, CAD_H - 0.002), CAD_W + 0.016,
                       0.008, (sgn, 0.0, 0.0), None))
    # the folded steel handle on the front side (drawn)
    s.append(BoxShape((0.0, hw1 + 0.003, CAD_H - 0.02), (0.15, 0.003, 0.004), (0.78, 0.79, 0.80), collide=False))
    return Body.build("caddy", pos=(CAD_XY[0], CAD_XY[1], TOP + 0.0005), mass=CAD_MASS, shapes=s, color=CADDY,
                      yaw=math.pi / 2, friction=(0.6, 0.5), com=(0.0, 0.0, 0.035), solver_iterations=32,
                      contact_offset=0.004)


def _tap() -> Body:
    """A chrome single-lever gooseneck tap on the sink's back deck, a fixed body. Origin: its base on the deck."""
    u = np.array([math.cos(SPOUT_AZ), math.sin(SPOUT_AZ), 0.0])
    s = [CylShape((0.0, 0.0, 0.003), 0.030, 0.006, CHROME),  # the escutcheon
         CylShape((0.0, 0.0, 0.055), 0.017, 0.098, CHROME),  # the body
         CylShape((0.0, 0.0, 0.104), 0.0125, 0.012, CHROME),  # the swivel collar
         BoxShape((0.030, 0.0, 0.090), (0.060, 0.010, 0.008), CHROME)]  # the lever, pointing back
    # the gooseneck: up to SPOUT_TOP, over in a half circle of radius SPOUT_REACH/2, and down to the spray head
    rr = SPOUT_REACH / 2
    zc = SPOUT_TOP - rr
    pts = [np.array([0.0, 0.0, 0.11]), np.array([0.0, 0.0, zc])]
    for k in range(1, 15):
        t = math.pi * k / 14
        pts.append(u * (rr - rr * math.cos(t)) + [0.0, 0.0, zc + rr * math.sin(t)])
    for p, q in zip(pts[:-1], pts[1:]):
        s.append(segment(tuple(p), tuple(q), 0.020, CHROME))
    head = u * SPOUT_REACH
    s.append(CylShape((head[0], head[1], (zc + SPOUT_OUT) / 2 + 0.01), 0.013, zc - SPOUT_OUT, CHROME))
    s.append(CylShape((head[0], head[1], SPOUT_OUT + 0.001), 0.011, 0.002, (0.25, 0.25, 0.27), collide=False))
    return Body.build("tap", pos=(TAP_XY[0], TAP_XY[1], TOP + RIM_T), mass=1.5, kinematic=True, color=CHROME,
                      shapes=s, friction=(0.4, 0.35))


OBJECTS = [_sink(), _strainer_body(), _tap(), _caddy(), _basket()] + [_scrap(*sc) for sc in SCRAPS]
MOVING = ["basket", "caddy"] + SCRAP_NAMES

# note each body's prim path (for the contact views)
_J: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name in ("basket", "sink", "strainer_body", "tap"):  # contact views need contact reports on these
        from pxr import PhysxSchema

        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body"))
        api.CreateThresholdAttr(0.0)
    if spec.name in _SCRAP_BOND:  # the scrap's bond to the basket (spawned before the scraps)
        _author_cling(stage, prim_path, spec.name)
    _J[spec.name] = f"{prim_path}/body"
    _J["env"] = prim_path.rsplit("/", 1)[0]


def _author_cling(stage, prim_path: str, name: str) -> None:
    """A fixed joint from the basket to the scrap where it starts: the cling. _cling_step() switches it off for good once
    its load exceeds CLING_G. The two still collide (a scrap that has come loose lies loose in the cup, it does not pass
    through it). Every reset switches it back on, whole (bodykit re-arms the joints it lists)."""
    from pxr import Gf, UsdPhysics

    local, yaw, _mass = _SCRAP_BOND[name]
    parent = prim_path.rsplit("/", 1)[0]
    path = f"{prim_path}/cling"
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([f"{parent}/basket/body"])
    joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in local]))  # the basket starts at B_START, unrotated
    joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateJointEnabledAttr(True)
    joint.CreateCollisionEnabledAttr(True)
    joint.CreateExcludeFromArticulationAttr(True)
    _CLING_PATHS[name] = path
    _CLING_ATTRS.pop(name, None)
    if path not in bodykit._BREAKABLE_PATHS:
        bodykit._BREAKABLE_PATHS.append(path)


bodykit._author_sliders = _author_sliders

PADS = [(arm, tip) for arm in ("LeftArm", "RightArm") for tip in ("tip_left", "tip_right")]
FIXTURES = ["sink", "strainer_body", "tap"]
_CT = {"views": None, "failed": ""}
TOUCH_N = 0.05


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _views():
    """PhysX rigid-contact views, made once: (0) the basket against the four finger pads and the strainer body, with
    contact points; (1) the sink, the strainer body and the tap against every arm link."""
    need = {"basket", "strainer_body", "sink", "tap", "env"}
    if _CT["views"] is None and not _CT["failed"] and need <= _J.keys():
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            env, links = _J["env"], asset_link_paths()
            pads = [f"{env}/{arm}{links[tip]}" for arm, tip in PADS]
            every = [f"{env}/{arm}{p}" for arm in ("LeftArm", "RightArm") for p in links.values()]
            sim = PhysxManager.get_physics_sim_view()
            basket = sim.create_rigid_contact_view([_J["basket"]], filter_patterns=[pads + [_J["strainer_body"]]],
                                                   max_contact_data_count=4096)
            fixt = sim.create_rigid_contact_view([_J[n] for n in FIXTURES], filter_patterns=[every] * len(FIXTURES),
                                                 max_contact_data_count=0)
            _CT["views"] = (basket, fixt, len(every))
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
    return _CT["views"]


def _basket_contacts(origin):
    """(forces (5, 3): the four pads then the strainer body, on the basket; each pad's force-weighted contact centre,
    world, or None)."""
    v = _views()
    if v is None:
        return None
    try:
        forces = _np(v[0].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(5, 3)
        f, pts, _n, _sep, counts, starts = v[0].get_contact_data(dt=PHYSICS_DT)
        f, pts = _np(f).reshape(-1), _np(pts).reshape(-1, 3)
        counts, starts = _np(counts).reshape(5).astype(int), _np(starts).reshape(5).astype(int)
    except Exception as e:  # noqa: BLE001
        _CT["failed"] = "read: " + repr(e)[:300]
        return None
    centres = []
    for j in range(5):
        w, c = 0.0, np.zeros(3)
        for k in range(starts[j], min(starts[j] + counts[j], len(f))):
            w += abs(float(f[k]))
            c += abs(float(f[k])) * (pts[k] - origin)
        centres.append(c / w if w > 1e-6 else None)
    return forces, centres


def _fixture_touches() -> np.ndarray | None:
    """(3,): the largest contact force any arm link puts on the sink, the strainer body and the tap now (N)."""
    v = _views()
    if v is None:
        return None
    try:
        m = _np(v[1].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(len(FIXTURES), v[2], 3)
    except Exception:  # noqa: BLE001
        return None
    return np.linalg.norm(m, axis=2).max(axis=1)




# -- followed after every control step --------------------------------------------------------------------------
GRIP_N = 1.0  # a finger pad pressing on the basket with more than this is touching it
FALL_V = 0.6  # the basket dropping faster than this (m/s) with no finger on it: dropped
FLOOR_Z = 0.30  # anything below this is on the room's floor
COUNTER_Z = TOP + 0.015  # a scrap whose middle is this low outside the bowl and the caddy lies on the counter or rim
CLEAR = 0.05  # at the end each gripper is at least this far from the basket


def _fresh() -> dict:
    return {"tcps": None, "down_drain": [], "on_floor": [], "on_counter": [], "dropped": False, "lifted": False,
            "seated": True, "in_caddy": [], "hands_off": True, "max_pad": np.zeros(4), "max_fix": np.zeros(3),
            "carried": 0, "steps": 0, "err": "", "loose": {}, "peak": dict.fromkeys(SCRAP_NAMES, 0.0),
            "win": 0.0, "step_max": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SINK_DEBUG"))}
_PHYS = {"on": False, "hist": [], "dt": PHYSICS_DT}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _PHYS["hist"] = []


def _release(name: str) -> None:
    """Switch a scrap's bond off: it has come loose."""
    if name not in _CLING_ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        _CLING_ATTRS[name] = UsdPhysics.Joint(get_current_stage().GetPrimAtPath(_CLING_PATHS[name])).GetJointEnabledAttr()
    _CLING_ATTRS[name].Set(False)


def _install_cling(base) -> None:
    """Run _cling_step() after every physics step (the scene's buffers are updated once per physics step)."""
    if _PHYS["on"]:
        return
    scene = base.scene
    plain = scene.update
    _PHYS["dt"] = float(getattr(base, "physics_dt", PHYSICS_DT))

    def update(*args, **kwargs):
        plain(*args, **kwargs)
        try:
            _cling_step(scene)
        except Exception as exc:  # noqa: BLE001
            if not _EP.get("err"):
                _EP["err"] = "cling: " + repr(exc)[:300]
                print(f"[sink] cling error: {_EP['err']}", file=sys.stderr, flush=True)

    scene.update = update
    _PHYS["on"] = True


def _cling_step(scene) -> None:
    """Each scrap still bonded: its cling load (cling_load) from its acceleration averaged over the last CLING_AVG
    physics steps, in the basket's frame. Over _CLING_LIMIT, its bond is switched off."""
    e = _EP
    bonded = [n for n in SCRAP_NAMES if n not in e["loose"]]
    hist = _PHYS["hist"]
    if not bonded:
        return
    v = {n: as_numpy(scene[n].data.root_com_vel_w)[0][:3] for n in bonded}
    hist.append(v)
    if len(hist) <= CLING_AVG:
        return
    old = hist.pop(0)
    br = rotation(as_numpy(scene["basket"].data.root_link_pose_w)[0][3:7])
    t = e["steps"] / 20
    for n in bonded:
        if n not in old:
            continue
        a = (v[n] - old[n]) / (CLING_AVG * _PHYS["dt"])
        load = cling_load(br.T @ (a + [0.0, 0.0, G]))
        e["peak"][n] = max(e["peak"][n], load)
        e["win"] = max(e["win"], load)
        e["step_max"] = max(e["step_max"], load)
        if load > _CLING_LIMIT:
            e["loose"][n] = round(t, 2)
            _release(n)
            if _DEBUG["on"]:
                print(f"[sink-cling] t={t:5.2f}s {n} came loose at a load of {load:.2f} x its weight",
                      file=sys.stderr, flush=True)


def _bond_slip(bp, br, p, r, name) -> tuple[float, float]:
    """How far (m) and how much (deg) a scrap is off the place in the basket it is bonded to."""
    local, yaw, _m = _SCRAP_BOND[name]
    q = br.T @ (np.asarray(p, float) - bp)
    rel = (br.T @ r) @ _rot((0.0, 0.0, 1.0), yaw).T
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(rel)) - 1.0) / 2.0))))
    return float(np.linalg.norm(q - local)), ang


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _in_bore(p) -> bool:
    """A point inside the drain's bore (or the tailpiece below it), more than 6 mm below the sink floor. A scrap whose
    middle gets there while the basket is not seated has gone down the drain, unless it is inside the basket's cup."""
    return math.hypot(p[0] - DRAIN[0], p[1] - DRAIN[1]) < BORE_R and p[2] < FLOOR - 0.006


def _seated_pose(bp, br) -> bool:
    """The basket upright in the strainer body: its axis within 4 mm of the drain's, its stem down on the crossbar
    (its origin within 4 mm of the seated height) and tilted no more than 8 deg."""
    return (math.hypot(bp[0] - DRAIN[0], bp[1] - DRAIN[1]) <= 0.004 and abs(bp[2] - SEATED_Z) <= 0.004
            and _tilt(br) <= 8.0)


CUP_IN_R0, CUP_IN_R1 = B_FLOOR_R - B_WALL_T, B_RIM_R - 0.003  # the cup wall's inside: 28.5 mm at its foot (2 mm up)
# ... widening to 36.7 mm just under the rim (26 mm up)


def _in_basket(bp, br, p) -> bool:
    """A scrap's middle inside the basket's cup, in the basket's own frame (wherever the basket is and however it is
    turned): above the cup's underside, no more than 10 mm above its rim, and inside its tapered wall (plus 1 mm).
    Such a scrap is carried in the basket, and the drain rule never counts it, not while the basket is lifted out of
    the strainer body and not while it is put back in. A scrap beside the basket, between its wall and the bore, is
    not in it."""
    q = br.T @ (np.asarray(p, float) - bp)
    if not 0.0 < q[2] < B_RIM_Z + 0.010:
        return False
    f = (min(max(q[2], 0.002), B_RIM_Z - 0.002) - 0.002) / (B_RIM_Z - 0.004)
    return math.hypot(q[0], q[1]) < CUP_IN_R0 + (CUP_IN_R1 - CUP_IN_R0) * f + 0.001


def _in_caddy(cp, cr, p) -> bool:
    """A scrap's middle inside the caddy's live inner volume: between its tapered inner walls, above its floor and
    below its rim, in the caddy's own frame (it may have been nudged)."""
    q = cr.T @ (np.asarray(p, float) - cp)
    if not 0.003 < q[2] < CAD_H:
        return False
    f = q[2] / CAD_H
    hl = CAD_L0 / 2 + (CAD_L - CAD_L0) / 2 * f - CAD_T
    hw = CAD_W0 / 2 + (CAD_W - CAD_W0) / 2 * f - CAD_T
    return abs(q[0]) < hl and abs(q[1]) < hw


def _over_caddy(cp, cr, p) -> bool:
    q = cr.T @ (np.asarray(p, float) - cp)
    return abs(q[0]) < CAD_L / 2 + 0.005 and abs(q[1]) < CAD_W / 2 + 0.005


def _over_bowl(p) -> bool:
    return BX0 < p[0] < BX1 and SINK_Y - BY < p[1] < SINK_Y + BY


@on_step
def _follow(env) -> None:
    try:
        _follow_inner(env)
    except Exception as exc:  # noqa: BLE001  (never let the follower end a trial)
        if not _EP.get("err"):
            _EP["err"] = repr(exc)[:300]
            print(f"[sink] follower error: {_EP['err']}", file=sys.stderr, flush=True)


def _follow_inner(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    e = _EP
    e["steps"] += 1
    bp, br = _pose(base, origin, "basket")
    cp, cr = _pose(base, origin, "caddy")
    bvel = as_numpy(base.scene["basket"].data.root_com_vel_w)[0][:3]
    tcps = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        k = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][k]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    e["tcps"] = tcps
    bc = _basket_contacts(origin)
    pads = np.zeros(4) if bc is None else np.linalg.norm(bc[0][:4], axis=1)
    seat_n = 0.0 if bc is None else float(np.linalg.norm(bc[0][4]))
    e["max_pad"] = np.maximum(e["max_pad"], pads)
    fx = _fixture_touches()
    if fx is not None:
        e["max_fix"] = np.maximum(e["max_fix"], fx)
    held = bool(pads.max() > GRIP_N)
    seated = _seated_pose(bp, br) and seat_n > TOUCH_N
    e["seated"] = seated
    if bp[2] > SEATED_Z + 0.03:
        e["lifted"] = True
    if not held and not seated and bvel[2] < -FALL_V:
        e["dropped"] = True
    if bp[2] < FLOOR_Z:
        e["on_floor"] = sorted(set(e["on_floor"]) | {"basket"})
    _install_cling(base)
    scr, inside, carried, in_vol = {}, [], 0, 0
    for name in SCRAP_NAMES:
        p, r = _pose(base, origin, name)
        scr[name] = p
        if _DEBUG["on"] and name not in e["loose"]:
            off, ang = _bond_slip(bp, br, p, r, name)
            if off > CLING_LOOSE_M or ang > CLING_LOOSE_DEG:
                print(f"[sink-cling] t={e['steps'] / 20:5.2f}s WARNING {name} is bonded but {off * 1000:.1f} mm, "
                      f"{ang:.1f} deg off its place", file=sys.stderr, flush=True)
        in_b = _in_basket(bp, br, p)
        if not seated and _in_bore(p):
            if in_b:  # carried down in the bore inside the basket (lifting it out or putting it back): not lost
                carried += 1
            elif name not in e["down_drain"]:
                e["down_drain"].append(name)
        if p[2] < FLOOR_Z and name not in e["on_floor"]:
            e["on_floor"].append(name)
        if (p[2] < COUNTER_Z and not _over_bowl(p) and not _in_caddy(cp, cr, p) and not in_b
                and name not in e["on_counter"]):
            e["on_counter"].append(name)
        if _in_caddy(cp, cr, p):
            in_vol += 1
            if not in_b:  # a scrap still in the basket's cup is not in the caddy, even with the basket in the caddy
                inside.append(name)
    e["in_caddy"] = inside
    e["carried"] = max(e["carried"], carried)  # the most scraps seen in the bore inside the unseated basket at once
    e["hands_off"] = (not held) and all(float(np.linalg.norm(t - bp)) >= CLEAR for t in tcps)
    if _DEBUG["on"] and e["step_max"] > 1.25:
        print(f"[sink-load] t={e['steps'] / 20:5.2f}s max bond load this step {e['step_max']:.2f} x weight, "
              f"bonded {len(SCRAP_NAMES) - len(e['loose'])}", file=sys.stderr, flush=True)
    e["step_max"] = 0.0
    if _DEBUG["on"] and e["steps"] % 5 == 0:
        print(f"[sink-cling] t={e['steps'] / 20:5.2f}s bonded={len(SCRAP_NAMES) - len(e['loose'])} "
              f"window_load={e['win']:.2f} loose={e['loose']}", file=sys.stderr, flush=True)
        e["win"] = 0.0
        knob = bp + br @ np.array([0.0, 0.0, KNOB_GRIP_Z])
        cup = bp + br @ np.array([0.0, 0.0, CUP_GRIP_Z])
        cent = "" if bc is None else " ".join(
            "-" if c is None else f"{np.round(br.T @ (c - bp) * 1000, 1).tolist()}" for c in bc[1][:4])
        print(f"[sink] t={e['steps'] / 20:5.2f}s basket={np.round((bp - B_START) * 1000, 1).tolist()}mm "
              f"tilt={_tilt(br):5.1f} seated={seated} seatN={seat_n:5.2f} "
              f"pads=[{', '.join(f'{v:5.1f}' for v in pads)}]N padpts_local=[{cent}] "
              f"knob-TCPR={np.round((knob - tcps[1]) * 1000, 1).tolist()} "
              f"cup-TCPL={np.round((cup - tcps[0]) * 1000, 1).tolist()} "
              f"caddy={np.round((cp - [CAD_XY[0], CAD_XY[1], TOP + 0.0005]) * 1000, 1).tolist()}mm "
              f"caddy_tilt={_tilt(cr):4.1f} in_caddy={len(inside)} in_caddy_volume={in_vol} "
              f"carried_in_bore={carried}/{e['carried']} "
              f"drain={e['down_drain']} floor={e['on_floor']} "
              f"counter={e['on_counter']} dropped={e['dropped']} off={e['hands_off']} "
              f"fix={'?' if fx is None else np.round(fx, 2).tolist()} score={score(None)}", file=sys.stderr, flush=True)
        if e["steps"] % 20 == 0:
            print("[sink-scraps] " + " ".join(
                f"{n}={np.round(cr.T @ (p - cp) * 1000).astype(int).tolist()}" for n, p in scr.items())
                  + " (caddy frame, mm)", file=sys.stderr, flush=True)
            print("[sink-peak] " + " ".join(f"{n}={v:.2f}" for n, v in e["peak"].items())
                  + " (peak bond load while bonded, x weight)", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _latched() -> bool:
    e = _EP
    return bool(e["down_drain"] or e["on_floor"] or e["on_counter"] or e["dropped"])


def score(objects) -> float:
    """0.0 for good once a scrap has gone down the open drain (its middle in the bore while the basket is not seated,
    and not inside the basket's cup: scraps carried in the basket as it is lifted out or put back never count), a
    scrap has landed on the counter, a scrap or the basket has fallen to the floor, or the basket has been dropped.
    Otherwise, at the end: 1.0 with all nine scraps
    in the caddy, the basket seated back in the strainer body and both hands off it; 0.5 with all nine scraps in the
    caddy; 0.25 with at least five scraps in the caddy; else 0.0. A scrap in the caddy is one out of the basket: one
    still in the basket's cup never counts, even with the basket set in the caddy."""
    e = _EP
    if _latched():
        return 0.0
    n = len(e["in_caddy"])
    if n == len(SCRAP_NAMES):
        return 1.0 if (e["seated"] and e["hands_off"]) else 0.5
    return 0.25 if n >= 5 else 0.0


def success(objects) -> bool:
    """Full score with the basket, the caddy and every scrap still: the trial ends there."""
    if score(objects) < 1.0:
        return False
    return all(objects[n].resting() for n in MOVING)


# -- scripted demo --------------------------------------------------------------------------------------------------
# The right hand reaches into the bowl, grips the basket's knob and lifts the basket out of the strainer body and up
# out of the bowl, upright, and carries it toward the caddy. The left hand comes in from the front and takes the
# basket's cup across its rim; the right lets go. The left carries it over the caddy and turns it over (104 deg about
# its own axis); the scraps stay stuck. It shakes it twice, a quick jab toward the caddy and straight back, and the
# scraps come loose and fall in. It turns it back up and brings it back. The right takes the knob again, the left lets
# go, and the right lowers the basket into the strainer body, presses it home and lets go. Both arms go home.

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


KNOB_GRIP_Z = KNOB_Z1 - 0.002  # the right's TCP 2 mm below the knob's top
CUP_GRIP_Z = 0.010  # the left's TCP on the basket's axis, 10 mm up the cup (it grips across the cup)
BETA = math.radians(15.0)  # the left's grip points 15 deg down
GRIP_R = _frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))  # the right on the knob, in the basket's frame: down, closing on y
LEFT_D = np.array([math.cos(BETA), 0.0, -math.sin(BETA)])  # the left comes at the cup from the front
GRIP_L = _frame(LEFT_D, (0.0, -1.0, 0.0))  # ... its fingers closing across the cup, side to side
KNOB_FINGER = -0.014  # open about 28 mm round the 16 mm knob
CUP_R_MID = B_FLOOR_R + (B_RIM_R - 0.003 - B_FLOOR_R) * (CUP_GRIP_Z - 0.002) / (B_RIM_Z - 0.004)  # 35 mm
CUP_FINGER = -(CUP_R_MID - 0.004)  # a gentle grip: each pad aimed 4 mm inside the cup's wall (about 20 N)
TIP_AXIS = LEFT_D  # the basket is turned about the left hand's own axis: a wrist roll for the left
TIP = math.radians(104.0)  # turned over this far, its mouth faces right (toward the sink) and down
UP = np.eye(3)
DRAIN_KNOB = np.array([DRAIN[0], DRAIN[1], SEATED_Z + KNOB_GRIP_Z])
LIFT_H = TOP + 0.07  # the knob this high: the basket's stem clears the rim by 1.5 cm
HANDOVER = np.array([0.300, -0.040, TOP + 0.080])  # the cup's middle here when the left takes it
TIP_AT = np.array([0.300, 0.165, TOP + 0.180])  # the cup's middle here, over the caddy, while it is turned over
FIRM_D, FIRM_S, FIRM_HOLD_S = 0.05, 0.10, 0.5  # each firm shake: a jab 5 cm toward the basket's mouth in 0.1 s and
# straight back in 0.1 s, then held 0.5 s
PRESS = 0.012  # the right aims this far below the seated knob to press the basket home
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _hand(arm: str, cup_at, brot):
    """The TCP pose of one hand with the cup's middle at cup_at and the basket turned by brot."""
    cup_at = np.asarray(cup_at, float)
    if arm == "right":
        return cup_at + brot @ np.array([0.0, 0.0, KNOB_GRIP_Z - CUP_GRIP_Z]), brot @ GRIP_R
    return cup_at, brot @ GRIP_L


def _shakes(kind: str, tipped, mouth_h) -> list:
    """The shakes over the caddy, the basket turned over (cup middle at TIP_AT). "firm" (the demo): twice, a jab FIRM_D
    toward the mouth (level) in FIRM_S and straight back in FIRM_S; the hard stop and turn back at the far end is what
    shakes the scraps loose, and they drop into the caddy. "gentle" (a check): the previous
    revision's two slow 2.5 cm strokes. "none" (a check): no shake, held turned over as long. "calib"
    (SINK_TEST=shakes): a series of shakes, to measure the cling's loads."""
    up = np.array([0.0, 0.0, 1.0])
    out = []
    firm_s = 2 * (2 * FIRM_S + FIRM_HOLD_S)
    if kind == "none":
        return [(firm_s, TIP_AT, tipped, "held turned over, no shake")]
    if kind == "gentle":
        for _ in range(2):
            out += [(0.22, TIP_AT + 0.025 * mouth_h, tipped, "gentle shake"), (0.22, TIP_AT, tipped, "gentle shake")]
        return out + [(firm_s - 0.88, TIP_AT, tipped, "held turned over")]
    if kind == "firm":
        for k in range(2):
            out += [(FIRM_S, TIP_AT + FIRM_D * mouth_h, tipped, f"shake {k + 1}: a quick jab toward the caddy"),
                    (FIRM_S, TIP_AT, tipped, f"shake {k + 1}: and straight back"),
                    (FIRM_HOLD_S, TIP_AT, tipped, f"shake {k + 1}: held")]
        return out
    # "calib": each candidate once, then a pause
    for _ in range(2):
        out += [(0.22, TIP_AT + 0.025 * mouth_h, tipped, "C1 gentle"), (0.22, TIP_AT, tipped, "C1 gentle")]
    out.append((0.6, TIP_AT, tipped, "pause"))
    out += [(0.5, TIP_AT + 0.06 * up, tipped, "C2 V6/0.10 raise"), (0.10, TIP_AT, tipped, "C2 V6/0.10 down"),
            (0.6, TIP_AT, tipped, "pause")]
    for name, d, t in (("C3 M3/0.10", 0.03, 0.10), ("C4 M4/0.10", 0.04, 0.10), ("C5 M5/0.10", 0.05, 0.10),
                       ("C6 M4/0.15", 0.04, 0.15)):
        out += [(t, TIP_AT + d * mouth_h, tipped, f"{name} thrust"), (0.4, TIP_AT, tipped, f"{name} back"),
                (0.6, TIP_AT, tipped, "pause")]
    out += [(0.10, TIP_AT + 0.04 * mouth_h, tipped, "C7 M4/0.10 out-back"), (0.10, TIP_AT, tipped, "C7 back"),
            (0.6, TIP_AT, tipped, "pause")]
    return out


def _basket_path(shake: str = "firm"):
    """The shared part: (duration, cup middle, basket rotation, label) for each step both hands hold the basket."""
    tipped = _rot(TIP_AXIS, TIP)
    mouth = tipped @ np.array([0.0, 0.0, 1.0])
    mouth_h = np.array([mouth[0], mouth[1], 0.0]) / max(1e-9, math.hypot(mouth[0], mouth[1]))
    start = HANDOVER + [0.0, 0.0, 0.03]
    path = [(1.0, start, UP, "lift it")]
    for (f, label), g in zip(((1 / 3, "carry it toward the caddy, turning it"), (2 / 3, "over the caddy, turning it over"),
                              (1.0, "turned right over")), (0.75, 1.0, 1.0)):
        at = start + (TIP_AT - start) * np.array([f, f, g])  # up first, so it clears the caddy's near wall
        path.append((1.3, at, _rot(TIP_AXIS, TIP * f), label))
    path.append((0.6, TIP_AT, tipped, "turned over: the scraps cling"))
    path += _shakes(shake, tipped, mouth_h)
    path.append((1.0, TIP_AT, tipped, "hold: the last scraps fall into the caddy"))
    for f, g in ((2 / 3, 1.0), (1 / 3, 0.75), (0.0, 0.0)):
        at = start + (TIP_AT - start) * np.array([f, f, g])
        path.append((1.3, at, _rot(TIP_AXIS, TIP * f), "turn it back up, back toward the sink"))
    path.append((1.0, HANDOVER, UP, "down to where it was taken"))
    return path


def _lift_out() -> list[Waypoint]:
    """The right reaches into the bowl, grips the knob and lifts the basket, scraps and all, up out of the bowl."""
    above = DRAIN_KNOB + [0.0, 0.0, 0.06]
    r = _lead_in("right", above, GRIP_R, "over the drain", KNOB_FINGER)
    r += [_w(2.0, DRAIN_KNOB, KNOB_FINGER, "down round the strainer's knob", GRIP_R),
          _w(0.8, DRAIN_KNOB, FINGER_CLOSED, "grip the knob", GRIP_R),
          _w(1.2, DRAIN_KNOB + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift the basket out of the strainer body", GRIP_R),
          _w(2.0, [DRAIN_KNOB[0], DRAIN_KNOB[1], LIFT_H], FINGER_CLOSED, "up out of the bowl", GRIP_R)]
    return r


def _right_before(reseat: bool = False) -> list[Waypoint]:
    r = _lift_out()
    if reseat:  # the drain rule's check (SINK_TEST=reseat): put it back, scraps and all, and lift it out again
        r += [_w(2.5, DRAIN_KNOB + [0.0, 0.0, 0.010], FINGER_CLOSED, "check: lower it back in with the scraps", GRIP_R),
              _w(1.0, DRAIN_KNOB - [0.0, 0.0, PRESS], FINGER_CLOSED, "check: press it home, scraps and all", GRIP_R),
              _w(1.0, DRAIN_KNOB - [0.0, 0.0, PRESS], FINGER_CLOSED, "check: hold it seated", GRIP_R),
              _w(1.5, DRAIN_KNOB + [0.0, 0.0, 0.03], FINGER_CLOSED, "check: lift it out again", GRIP_R),
              _w(2.0, [DRAIN_KNOB[0], DRAIN_KNOB[1], LIFT_H], FINGER_CLOSED, "check: up out of the bowl again",
                 GRIP_R)]
    p, rot = _hand("right", HANDOVER, UP)
    r.append(_w(2.5, p, FINGER_CLOSED, "carry it toward the caddy", rot))
    return r


def _right_after() -> list[Waypoint]:
    over = np.array([DRAIN_KNOB[0], DRAIN_KNOB[1], LIFT_H])
    r = [_w(2.5, over, FINGER_CLOSED, "carry it back over the drain", GRIP_R),
         _w(2.0, DRAIN_KNOB + [0.0, 0.0, 0.045], FINGER_CLOSED, "lower it to just over the strainer body", GRIP_R),
         _w(0.6, DRAIN_KNOB + [0.0, 0.0, 0.045], FINGER_CLOSED, "let it settle over the drain", GRIP_R),
         _w(1.5, DRAIN_KNOB + [0.0, 0.0, 0.010], FINGER_CLOSED, "lower it into the strainer body", GRIP_R),
         _w(1.0, DRAIN_KNOB - [0.0, 0.0, PRESS], FINGER_CLOSED, "press it home", GRIP_R),
         _w(0.8, DRAIN_KNOB - [0.0, 0.0, PRESS], FINGER_CLOSED, "hold it pressed", GRIP_R),
         _w(0.8, DRAIN_KNOB - [0.0, 0.0, PRESS], KNOB_FINGER, "let go", GRIP_R),
         _w(1.5, over, KNOB_FINGER, "up out of the bowl", GRIP_R)]
    r += _go_home("right", over, GRIP_R, KNOB_FINGER)
    return r


RIGHT_WAIT = np.array([0.300, -0.170, TOP + 0.150])  # where the right waits, clear of the left's path, while the left
# empties the basket


def _two_hand_plans(reseat: bool = False) -> dict:
    """The demo. The right lifts the basket out by its knob and brings it toward the caddy; the left takes the cup from
    the front; the right lets go and moves clear. The left alone carries the basket over the caddy, turns it over,
    shakes it twice, turns it back up and brings it back. The right takes the knob again, the left lets go, and the
    right lowers the basket into the strainer body and presses it home. Both arms go home. With reseat (the check
    SINK_TEST=reseat) the right first puts the full basket back in the drain and lifts it out again."""
    right = _right_before(reseat)
    t = sum(w.duration for w in right)
    knob_p, knob_r = _hand("right", HANDOVER, UP)
    lp, lr = _hand("left", HANDOVER, UP)
    approach = lr[:, 2]
    pre = lp - 0.07 * approach
    ready_p, ready_r = _ready("left")
    left = [_w(t, ready_p, FINGER_OPEN, "wait at the start", ready_r)]
    stage = lp - 0.15 * approach + [0.0, 0.0, 0.06]
    left += _lead_in("left", stage, lr, "over in front of the basket", FINGER_OPEN)
    left += [_w(1.5, pre, FINGER_OPEN, "down in front of the cup", lr),
             _w(2.0, lp, FINGER_OPEN, "slide in round the cup", lr),
             _w(1.0, lp, CUP_FINGER, "take the cup", lr)]
    right.append(_w(sum(w.duration for w in left) - t, knob_p, FINGER_CLOSED, "hold it for the other hand", knob_r))
    # the right lets go and moves clear
    up = knob_p + [0.0, 0.0, 0.03]
    release = [_w(0.8, knob_p, KNOB_FINGER, "let go of the knob", knob_r),
               _w(1.0, up, KNOB_FINGER, "up off the knob", knob_r),
               _w(1.5, RIGHT_WAIT, KNOB_FINGER, "move clear", knob_r)]
    right += release
    left.append(_w(sum(w.duration for w in release), lp, CUP_FINGER, "hold the basket", lr))
    # the left alone empties it
    path = _basket_path(_shake_kind())
    for dur, cup_at, brot, label in path:
        p, rot = _hand("left", cup_at, brot)
        left.append(_w(dur, p, CUP_FINGER, label, rot))
    right.append(_w(sum(step[0] for step in path), RIGHT_WAIT, KNOB_FINGER, "wait clear while the basket is emptied",
                    knob_r))
    # the right takes the knob again
    regrip = [_w(1.5, up, KNOB_FINGER, "back over the knob", knob_r),
              _w(1.0, knob_p, KNOB_FINGER, "down round the knob", knob_r),
              _w(0.8, knob_p, FINGER_CLOSED, "take the knob", knob_r)]
    right += regrip
    left.append(_w(sum(w.duration for w in regrip), lp, CUP_FINGER, "hold it for the other hand", lr))
    # the left lets go and goes home
    out = lp - 0.10 * approach
    left += [_w(0.8, lp, FINGER_OPEN, "let go of the cup", lr),
             _w(1.4, out, FINGER_OPEN, "back out", lr),
             _w(1.2, out + [0.0, 0.0, 0.10], FINGER_OPEN, "up", lr)]
    left += _go_home("left", out + [0.0, 0.0, 0.10], lr, FINGER_OPEN)
    right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), knob_p, FINGER_CLOSED,
                    "wait for the other hand to clear", knob_r))
    right += _right_after()
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, KNOB_FINGER, "hold still", right[-1].yaw))
    left.append(_w(total - sum(w.duration for w in left), left[-1].pos, FINGER_OPEN, "hold still", left[-1].yaw))
    return {"left": left, "right": right}


def _one_hand_plans() -> dict:
    """The one-hand check (SINK_ONEHAND=1): the right alone lifts the basket by its knob, carries it over the caddy,
    turns it over, shakes it twice, brings it back and presses it home; the left never moves."""
    right = _right_before()
    right.append(_w(LEAD_S + 4.5, right[-1].pos, FINGER_CLOSED, "hold", right[-1].yaw))
    for dur, cup_at, brot, label in _basket_path(_shake_kind()):
        p, rot = _hand("right", cup_at, brot)
        right.append(_w(dur, p, FINGER_CLOSED, label, rot))
    right += _right_after()
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, KNOB_FINGER, "hold still", right[-1].yaw))
    ready_p, ready_r = _ready("left")
    return {"left": [_w(total, ready_p, FINGER_OPEN, "stay", ready_r)], "right": right}


def _bin_basket_plans() -> dict:
    """The check SINK_TEST=binbasket: the right lifts the basket out by its knob, carries it over the caddy, sets it
    down in the caddy (tilted 25 deg along the caddy, so the right can reach) with every scrap still stuck in it, lets
    go and goes home; the left never moves. The scraps are inside the caddy's volume but still in the basket's cup, so
    none counts: 0.0, not 0.5."""
    right = _lift_out()
    tilt = _rot((1.0, 0.0, 0.0), math.radians(25.0))
    rot = tilt @ GRIP_R
    xy = np.array([CAD_XY[0], CAD_XY[1] - 0.02])
    # its lowest point (the seal's edge) 6 mm above the caddy's floor
    low = min(float((tilt @ np.array(v))[2]) for v in ((0.0, sy, -SEAL_H) for sy in (-SEAL_R, SEAL_R)))
    in_bin = np.array([*xy, TOP + 0.0035 + 0.006 - low + float((tilt @ [0.0, 0.0, KNOB_GRIP_Z])[2])])
    over = np.array([*xy, TOP + 0.150])
    right += [_w(1.5, [DRAIN_KNOB[0], DRAIN_KNOB[1], TOP + 0.150], FINGER_CLOSED, "check: up over the drain", GRIP_R),
              _w(3.0, over, FINGER_CLOSED, "check: over the caddy", rot),
              _w(2.5, in_bin, FINGER_CLOSED, "check: down into the caddy", rot),
              _w(0.5, in_bin, FINGER_CLOSED, "check: hold", rot),
              _w(0.8, in_bin, KNOB_FINGER, "check: let go of it in the caddy", rot),
              _w(1.5, over, KNOB_FINGER, "check: up out of the caddy", rot)]
    right += _go_home("right", over, rot, KNOB_FINGER)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, KNOB_FINGER, "hold still", right[-1].yaw))
    ready_p, ready_r = _ready("left")
    return {"left": [_w(total, ready_p, FINGER_OPEN, "stay", ready_r)], "right": right}


def _shake_kind() -> str:
    """The demo's shakes: "firm"; the checks SINK_TEST=noshake ("none"), SINK_TEST=gentle and SINK_TEST=shakes
    ("calib") change them."""
    return {"noshake": "none", "gentle": "gentle", "shakes": "calib"}.get(_TEST, "firm")


def demo():
    if os.environ.get("SINK_ONEHAND"):
        return _one_hand_plans()
    if _TEST == "binbasket":
        return _bin_basket_plans()
    return _two_hand_plans(reseat=_TEST == "reseat")


# -- a check of the drain rule's geometry, no simulator needed: python empty_the_sink_strainer_into_the_bin.py ------

def _drain_rule_selftest() -> list[str]:
    """Which scrap positions the drain rule counts as lost, for poses set by hand (it runs while the basket is not
    seated). Prints one line per case."""
    def lost(bp, br, p, seated=False):
        return (not seated) and _in_bore(p) and not _in_basket(bp, br, p)

    local = [np.asarray(loc) for _n, _k, loc, *_ in SCRAPS]
    out, worst = [], 0
    for dz in np.arange(0.0, 0.0601, 0.002):  # the full basket lifted out or put back, upright or tilted
        for tilt in (0.0, 8.0, 15.0):
            r = _rot((1.0, 0.0, 0.0), math.radians(tilt))
            bp = B_START + [0.0, 0.0, dz]
            worst = max(worst, sum(lost(bp, r, bp + r @ q) for q in local))
            if dz in (0.0, 0.02) and tilt in (0.0, 15.0):
                out.append(f"basket {dz * 1000:2.0f} mm up, tilted {tilt:4.1f} deg: "
                           f"{sum(_in_bore(bp + r @ q) for q in local)} scraps in the bore, "
                           f"{sum(lost(bp, r, bp + r @ q) for q in local)} counted lost")
    out.append(f"basket 0-60 mm up, tilted 0-15 deg, all nine scraps in it: at most {worst} counted lost")
    bp = B_START + [0.0, 0.0, 0.010]
    for r_mm, z_mm in ((36.0, 5.0), (37.0, 2.0)):
        p = bp + [r_mm / 1000, 0.0, z_mm / 1000]
        out.append(f"pea beside the basket ({r_mm:.0f} mm from its axis, {z_mm:.0f} mm above its underside), basket 10 mm "
                   f"up: in the cup {_in_basket(bp, np.eye(3), p)}, counted lost {lost(bp, np.eye(3), p)}")
    p = np.array([DRAIN[0] + 0.02, DRAIN[1], FLOOR + CROSSBAR + 0.004])
    out.append(f"pea on the crossbar under the basket lifted 40 mm: counted lost "
               f"{lost(B_START + [0.0, 0.0, 0.040], np.eye(3), p)}")
    p = np.array([DRAIN[0], DRAIN[1], FLOOR - 0.03])
    out.append(f"pea 30 mm down the bore, basket away over the caddy: counted lost "
               f"{lost(np.array([0.30, 0.165, TOP + 0.18]), np.eye(3), p)}")
    # the caddy count: a scrap in the basket's cup never counts, even with the basket standing in the caddy
    cp, cr = np.array([CAD_XY[0], CAD_XY[1], TOP + 0.0005]), _rot((0.0, 0.0, 1.0), math.pi / 2)
    bp = cp + [0.0, 0.0, 0.003 + SEAL_H + STEM_L]
    in_cup = [bp + q for q in local]
    out.append(f"basket standing in the caddy, all nine scraps in its cup: {sum(_in_caddy(cp, cr, q) for q in in_cup)} "
               f"in the caddy's volume, {sum(_in_caddy(cp, cr, q) and not _in_basket(bp, np.eye(3), q) for q in in_cup)} "
               f"counted in the caddy")
    q = cp + [0.0, 0.07, 0.008]
    out.append(f"pea on the caddy's floor beside that basket: counted in the caddy "
               f"{_in_caddy(cp, cr, q) and not _in_basket(bp, np.eye(3), q)}")
    return out


def _cling_selftest() -> list[str]:
    """The cling rule's load (cling_load, in multiples of the weight) for a basket held still at several angles and
    moved in a few ways, and the force (mN) at which each scrap lets go. Prints one line per case."""
    out = []
    for tilt in (0.0, 45.0, 90.0, 104.0, 135.0, 180.0):  # held still, turned over about a level axis
        r = _rot((1.0, 0.0, 0.0), math.radians(tilt))
        out.append(f"held still, turned {tilt:5.1f} deg: load {cling_load(r.T @ np.array([0.0, 0.0, G])):.2f}")
    for label, a in (("upright, lifted at 2 g", (0.0, 0.0, 2 * G)), ("upright, pushed sideways at 0.4 g", (0.4 * G, 0, 0)),
                     ("upright, pushed sideways at 1.0 g", (G, 0.0, 0.0)), ("upright, dropped at 1.5 g", (0, 0, -1.5 * G))):
        out.append(f"{label}: load {cling_load(np.asarray(a, float) + [0.0, 0.0, G]):.2f}")
    r = _rot((1.0, 0.0, 0.0), math.pi)
    for k in (0.5, 1.0, 1.5):
        out.append(f"upside down, jerked up at {k:.1f} g (the scraps pulled from the floor): load "
                   f"{cling_load(r.T @ np.array([0.0, 0.0, G + k * G])):.2f}")
    out.append("lets go at (mN): " + ", ".join(f"{n} {CLING_G * m * G * 1000:.1f}" for n, (_l, _y, m) in
                                              _SCRAP_BOND.items()) + f" ({CLING_G} x the weight)")
    return out


if __name__ == "__main__":
    print("\n".join(_drain_rule_selftest()))
    print("\n".join(_cling_selftest()))
