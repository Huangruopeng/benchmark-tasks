"""On the bench stands a pneumatic lug crimper (an AM-70 hex-type press: a housing with its air cylinder, and a pair of
jaws in front carrying a hexagonal die), its jaws inside a clear guard. A 16 mm2 tube cable lug lies in the lower half
of the open 16 mm2 die, its barrel pointing left, its palm against a lug stop. The press's 5/2 pedal valve stands on the
bench to its right, and a short 16 mm2 cable with a stripped end lies on two rests to its left.

Task: crimp the lug onto the cable's end. One hand picks up the cable, pushes its stripped end through the guard's
feed hole into the lug's barrel, straight along it, until the insulation meets the barrel's mouth, and holds it there;
the other presses the pedal valve down and holds it while the press's jaws close and crimp, then lets it go, and the
jaws open. Then the first hand pulls the cable out of the press with the lug crimped on its end, holds it up to show it,
and lays it back on its rests, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=crimp_a_terminal_onto_a_cable_end.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the press: an AM-70 hex-type pneumatic lug crimper, 510 x 150 x 195 mm overall, its long axis pointing at the arms.
# Its jaws (in front, 10 mm thick) hold the 16 mm2 hexagonal die; the barrel's axis runs across them, along y. ---------
BX = 0.27  # the die's centre (the barrel's axis), forward from the line between the arm bases
AXIS_Z = 0.075  # the barrel's axis above the bench (estimated from the maker's photos)
JAW_T = 0.010  # the jaws' and the die's thickness along the barrel (estimated)
JAW_TIP = 0.22  # the jaws' front end (x)
HOUSING_X = (0.40, 0.73)  # the housing (air cylinder) behind the jaws: 0.22 to 0.73 is the press's 510 mm length
HOUSING_W, HOUSING_H, HANDLE_H = 0.15, 0.16, 0.035  # 150 mm wide; 160 mm housing + 35 mm handle = 195 mm tall
JAW_PIVOT = np.array([0.38, 0.0, AXIS_Z + 0.012])  # the upper jaw's pivot (relative to the bench under y = 0)
JAW_OPEN_DEG = 15.0  # the upper jaw open (front end up)
T_CLOSE, T_OPEN = 1.0, 1.0  # s the cylinder takes to close and to open the jaws (assumed: not published)

# -- the lug: a Klauke 3R8 tubular cable lug, Cu tinned, 16 mm2, M8: barrel 5.5 mm inside, 8.5 mm outside, 13 mm long;
# palm 15 mm wide, 29 mm overall, 8.5 mm hole. Its frame: origin at the barrel's mouth, x along the barrel into it. --
BAR_RI, BAR_T, BAR_L = 0.00275, 0.0015, 0.013
CHAMFER = 0.0005  # the barrel's internal chamfer at its mouth, 45 degrees (Klauke: "precise internal chamfer"; size
# assumed, not published)
PALM_L, PALM_W, PALM_T = 0.016, 0.015, 0.003  # 29 mm overall less the 13 mm barrel; 3 mm: the tube flattened
HOLE_R, HOLE_AT = 0.00425, 0.008  # the 8.5 mm stud hole, its centre 8 mm from the palm's end
MOUTH = np.array([BX, BAR_L / 2, TOP + AXIS_Z])  # the barrel centred on the die (y = 0): its mouth 6.5 mm left of it
BAR_DIR = np.array([0.0, -1.0, 0.0])  # into the barrel (to the right)
LUG_ROT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the lug's frame in the world (yaw -90 deg)
STOP_T = 0.016  # the lug stop: a 24 x 16 mm block standing on the bench against the palm's end

# -- the clear guard (3 mm polycarbonate) round the jaws, with a 22 mm square feed hole for the cable ---------------
GUARD_T = 0.003
GUARD_GAP = 0.018  # its left wall's inside face this far left of the barrel's mouth
GUARD_HOLE = 0.022
GUARD_X = (0.205, HOUSING_X[0])
GUARD_Y = (-0.048, MOUTH[1] + GUARD_GAP + GUARD_T)  # outside faces: right and left walls
GUARD_H = 0.16

# -- the pedal valve: a Camozzi 354N-925 (Series 3, 5/2, G1/4): 245.9 x 138 mm, 67.5 mm body, 152.9 mm at its high
# end. Its cover is hinged at the low end; pressing the high end down switches the valve: 17 N at 6 bar. It stands
# along x, its low end away from the arms and its high end toward them. -----------------------------------------------
PED_X0, PED_Y = 0.531, -0.24  # the cover's hinge (low end, far from the arms) and the pedal's centre line
PED_L, PED_W, PED_BODY, PED_TOPH = 0.2459, 0.138, 0.0675, 0.1529
PED_PRESS = 0.231  # where a hand presses: on the flat at the high end, this far from the hinge
PED_STOP_DEG, PED_SWITCH_DEG = 6.0, 5.0  # the cover's travel and where the valve switches (assumed: not published)
PED_SPRING, PED_SPRING_TO = 0.14, -26.0  # N m per degree, 26 deg past rest: 14.5 N at rest, 17.0 N at the switch
PED_MASS, PED_COM_X = 0.25, 0.12

# -- the cable: 16 mm2 single-core H07V-R; its frame: origin at the stripped tip, the cable along -x -----------------
STRIP_L, STRIP_R = 0.013, 0.00255  # 7 x 1.7 mm strands, 5.1 mm across, stripped to the barrel's 13 mm
CABLE_L, CABLE_R = 0.16, 0.0035  # 7 mm over the insulation
CABLE_MASS = 0.027  # 23 g of copper (16 mm2 x 16 cm x 8.96 g/cm3) and 4 g of PVC
CABLE_ROT = LUG_ROT  # on its rests the cable lies along y, its tip pointing right (toward the press)
TIP0 = np.array([0.20, 0.09])  # where the cable's tip lies on its rests
CABLE_Z = 0.03  # the cable's axis above the bench on its rests
REST_AT = (0.045, 0.13)  # the two V-rests, along the cable from its tip (its centre of mass, 8 cm, between them)

COPPER, BLACK, RED, STEEL, DARK, ORANGE, YELLOW, BLUE, CLEAR, BEIGE = (
    (0.80, 0.50, 0.25), (0.08, 0.08, 0.09), (0.75, 0.10, 0.10), (0.62, 0.63, 0.66), (0.20, 0.21, 0.23),
    (0.90, 0.42, 0.08), (0.95, 0.72, 0.10), (0.18, 0.36, 0.62), (0.80, 0.90, 0.97), (0.85, 0.82, 0.74))
EDGE = (0.70, 0.80, 0.88)  # the guard's edges, drawn in place of its clear panels

# -- scoring ----------------------------------------------------------------------------------------------------------
# a good crimp: the bare end this far into the 13 mm barrel when the jaws close (all but at most 2.5 mm of the copper
# in: the insulation, too wide for the barrel, within 2.5 mm of its mouth) and the cable within this angle of its axis
DEPTH = (0.0105, 0.0135)
ALIGN_DEG = 8.0
INSERTED = 0.0065  # the bare end pushed this far in (half the barrel) at some point
LET_GO = 0.04  # both TCPs this far from the pedal's press point: let go
OUT = 0.05  # the cable's tip this far from the barrel's mouth: taken out of the press (past the guard)
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home
ON_REST = (0.03, 0.015, 20.0)  # laid back on its rests: the tip this close to where it lay (across, up) and the cable
# within this many degrees of lying along its rests
SUCCESS_HOLD_S = 1.0
DT = 0.05  # s per control step (20 Hz)


def _press() -> Body:
    """The press's fixed parts: housing, handle, lower jaw and die, the clear guard; and the lug stop."""
    hx0, hx1 = HOUSING_X
    jaw_top = AXIS_Z - BAR_RI - BAR_T  # the lower die's seat: the barrel's underside
    shapes = [BoxShape(((hx0 + hx1) / 2, 0.0, 0.004), (hx1 - hx0, HOUSING_W + 0.01, 0.008), BLACK),  # base plate
              BoxShape(((hx0 + hx1) / 2, 0.0, 0.008 + (HOUSING_H - 0.008) / 2), (hx1 - hx0, HOUSING_W, HOUSING_H - 0.008),
                       ORANGE),  # housing (the air cylinder)
              BoxShape((hx0 + 0.06, 0.0, HOUSING_H + 0.01), (0.03, 0.03, 0.02), BLACK),  # handle posts
              BoxShape((hx1 - 0.06, 0.0, HOUSING_H + 0.01), (0.03, 0.03, 0.02), BLACK),
              BoxShape(((hx0 + hx1) / 2, 0.0, HOUSING_H + HANDLE_H - 0.0075), (hx1 - hx0 - 0.09, 0.03, 0.015), BLACK),
              BoxShape(((JAW_TIP + hx0) / 2, 0.0, (0.035 + jaw_top - 0.012) / 2),
                       (hx0 - JAW_TIP, JAW_T, jaw_top - 0.012 - 0.035), STEEL),  # lower jaw
              BoxShape((BX, 0.0, jaw_top - 0.006), (0.04, JAW_T, 0.012), DARK),  # lower half of the 16 mm2 die
              BoxShape((hx0 - 0.02, 0.0, 0.035 + 0.04), (0.04, 0.03, 0.08), STEEL)]  # jaw root at the housing
    # the clear guard: left wall with the feed hole, right wall, front, top
    gx0, gx1 = GUARD_X
    gy0, gy1 = GUARD_Y
    yl = gy1 - GUARD_T / 2
    hz0, hz1 = AXIS_Z - GUARD_HOLE / 2, AXIS_Z + GUARD_HOLE / 2
    hx_0, hx_1 = BX - GUARD_HOLE / 2, BX + GUARD_HOLE / 2
    panels = [(((gx0 + gx1) / 2, yl, hz0 / 2), (gx1 - gx0, GUARD_T, hz0)),  # left wall, round the feed hole
              (((gx0 + gx1) / 2, yl, (hz1 + GUARD_H) / 2), (gx1 - gx0, GUARD_T, GUARD_H - hz1)),
              (((gx0 + hx_0) / 2, yl, AXIS_Z), (hx_0 - gx0, GUARD_T, GUARD_HOLE)),
              (((hx_1 + gx1) / 2, yl, AXIS_Z), (gx1 - hx_1, GUARD_T, GUARD_HOLE)),
              (((gx0 + gx1) / 2, gy0 + GUARD_T / 2, GUARD_H / 2), (gx1 - gx0, GUARD_T, GUARD_H)),  # right wall
              ((gx0 + GUARD_T / 2, (gy0 + gy1) / 2, GUARD_H / 2), (GUARD_T, gy1 - gy0, GUARD_H)),  # front
              (((gx0 + gx1) / 2, (gy0 + gy1) / 2, GUARD_H + GUARD_T / 2), (gx1 - gx0, gy1 - gy0, GUARD_T))]  # lid
    shapes += [BoxShape(c, z, CLEAR, visible=False) for c, z in panels]  # clear: solid, but drawn only by its edges
    e = 0.003  # its edges and the feed hole's rim, shown only
    zt = GUARD_H + GUARD_T
    for y in (gy0 + e / 2, gy1 - e / 2):
        for z in (e / 2, zt - e / 2):
            shapes.append(BoxShape(((gx0 + gx1) / 2, y, z), (gx1 - gx0, e, e), EDGE, collide=False))
        shapes.append(BoxShape((gx0 + e / 2, y, zt / 2), (e, e, zt), EDGE, collide=False))
    for z in (e / 2, zt - e / 2):
        shapes.append(BoxShape((gx0 + e / 2, (gy0 + gy1) / 2, z), (e, gy1 - gy0, e), EDGE, collide=False))
    for z in (hz0 - e / 2, hz1 + e / 2):
        shapes.append(BoxShape((BX, gy1 - e / 2, z), (GUARD_HOLE + 2 * e, e + 0.0005, e), EDGE, collide=False))
    for x in (hx_0 - e / 2, hx_1 + e / 2):
        shapes.append(BoxShape((x, gy1 - e / 2, AXIS_Z), (e, e + 0.0005, GUARD_HOLE), EDGE, collide=False))
    # the lug stop against the palm's end
    end = MOUTH[1] - BAR_L - PALM_L
    shapes.append(BoxShape((BX, end - STOP_T / 2, (AXIS_Z + 0.006) / 2), (0.024, STOP_T, AXIS_Z + 0.006), DARK))
    return Body.build("press", pos=(0.0, 0.0, TOP), mass=18.0, shapes=shapes, kinematic=True, friction=(0.4, 0.3),
                      color=ORANGE)


def _upper_jaw() -> Body:
    """The upper jaw and the upper half of the die (shown only): it turns about its pivot as the cylinder drives it."""
    p = JAW_PIVOT
    top = AXIS_Z + BAR_RI + BAR_T  # the upper die closed: the barrel's top
    shapes = [BoxShape(((JAW_TIP + p[0] + 0.01) / 2 - p[0], 0.0, (top + 0.012 + top + 0.03) / 2 - p[2]),
                       (p[0] + 0.01 - JAW_TIP, JAW_T, 0.018), STEEL, collide=False),
              BoxShape((BX - p[0], 0.0, top + 0.006 - p[2]), (0.04, JAW_T, 0.012), DARK, collide=False),
              CylShape((0.0, 0.0, 0.0), 0.008, JAW_T + 0.004, BLACK, axis="Y", collide=False)]
    a = math.radians(JAW_OPEN_DEG) / 2
    return Body.build("upper_jaw", pos=(p[0], p[1], TOP + p[2]), mass=0.5, shapes=shapes, kinematic=True, color=STEEL,
                      start_xyzw=(0.0, math.sin(a), 0.0, math.cos(a)))


def _lug_shapes(r_in: float, collide: bool, solid: bool = False, chamfer: float = 0.0) -> list:
    """A tube lug in its frame (mouth at the origin, the barrel along +x): the barrel (panels round the x axis, ``r_in``
    inside, with a 45 degree internal ``chamfer`` at the mouth, or a solid rod when crushed empty) and its flat palm with
    the stud hole."""
    shapes = [BoxShape((BAR_L + PALM_L / 2, 0.0, 0.0), (PALM_L, PALM_W, PALM_T), COPPER, collide=collide),
              CylShape((BAR_L + PALM_L - HOLE_AT, 0.0, 0.0), HOLE_R, PALM_T + 0.0003, DARK, collide=False)]
    if solid:
        return shapes + [CylShape((BAR_L / 2, 0.0, 0.0), r_in + BAR_T, BAR_L, COPPER, axis="X", collide=collide)]
    n = 10
    tc = 0.0008  # the chamfer's panels: 0.8 mm thick
    for k in range(n):
        a = 2 * math.pi * k / n
        u = np.array([0.0, math.cos(a), math.sin(a)])
        r = r_in + BAR_T / 2
        shapes.append(plate(tuple(chamfer * np.array([1.0, 0.0, 0.0]) + r * u), tuple(BAR_L * np.array([1.0, 0.0, 0.0]) + r * u),
                            2 * r * math.tan(math.pi / n) + 0.0005, BAR_T, tuple(u), COPPER, collide=collide))
        if chamfer > 0.0:  # the chamfer: its face from radius r_in + chamfer at the mouth to r_in at depth chamfer
            nrm = (np.array([1.0, 0.0, 0.0]) + u) / math.sqrt(2.0)
            p0 = (r_in + chamfer) * u + nrm * tc / 2
            p1 = np.array([chamfer, 0.0, 0.0]) + r_in * u + nrm * tc / 2
            shapes.append(plate(tuple(p0), tuple(p1), 2 * (r_in + chamfer) * math.tan(math.pi / n) + 0.0008, tc,
                                tuple(nrm), COPPER, collide=collide))
    return shapes


HIDE = np.array([BX, 0.05, TOP - 0.03])  # under the bench
HIDE2 = np.array([BX, -0.05, TOP - 0.03])


def _lug() -> Body:
    return Body.build("lug", pos=tuple(MOUTH.tolist()), mass=0.009, shapes=_lug_shapes(BAR_RI, True, chamfer=CHAMFER),
                      kinematic=True, friction=(0.3, 0.25), slick=True, color=COPPER, yaw=-math.pi / 2)  # tinned, smooth


def _crimped() -> Body:
    """The lug crimped onto the copper (its barrel squeezed onto it): shown once crimped, following the cable."""
    return Body.build("crimped_lug", pos=tuple(HIDE.tolist()), mass=0.009, kinematic=True, color=COPPER,
                      shapes=_lug_shapes(STRIP_R + 0.0003, False), yaw=-math.pi / 2)


def _crushed() -> Body:
    """The lug crushed empty (the jaws closed with no copper in it): its barrel closed, nothing goes in any more."""
    return Body.build("crushed_lug", pos=tuple(HIDE2.tolist()), mass=0.009, kinematic=True, color=COPPER,
                      shapes=_lug_shapes(0.0008, True, solid=True), yaw=-math.pi / 2)


def _pedal_base() -> Body:
    shapes = [BoxShape((-PED_L / 2, 0.0, PED_BODY / 2), (PED_L, PED_W, PED_BODY), BLUE),
              BoxShape((-0.006, 0.0, PED_BODY + 0.004), (0.012, PED_W - 0.02, 0.008), BLUE)]  # hinge block
    return Body.build("pedal_base", pos=(PED_X0, PED_Y, TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.8, 0.7), color=BLUE)


def _pedal_cover() -> Body:
    """The pedal's cover, origin on its hinge (at the low end, on the body's top), rising toward -x to the flat at the
    high end."""
    rise = PED_TOPH - PED_BODY
    flat0 = PED_L - 0.03
    t = 0.004
    shapes = [plate((-0.004, 0.0, 0.0), (-flat0, 0.0, rise - t / 2), PED_W - 0.004, t, (rise, 0.0, flat0), YELLOW),
              BoxShape((-(flat0 + PED_L) / 2, 0.0, rise - t / 2), (PED_L - flat0, PED_W - 0.004, t), YELLOW),
              BoxShape((-(PED_L - 0.002), 0.0, rise / 2), (0.004, PED_W - 0.004, rise), YELLOW),  # its high end
              BoxShape((-(flat0 + PED_L) / 2, 0.035, rise + 0.001), (0.02, 0.012, 0.003), RED, collide=False)]  # flap
    for s in (-1, 1):  # side skirts
        shapes.append(BoxShape((-PED_L * 0.6, s * (PED_W / 2 - 0.002), rise * 0.45), (PED_L * 0.75, 0.004, rise * 0.5),
                               YELLOW, collide=False))
    return Body.build("pedal", pos=(PED_X0, PED_Y, TOP + PED_BODY), mass=PED_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=YELLOW, com=(-PED_COM_X, 0.0, rise * 0.5))


def _rest() -> Body:
    shapes = []
    h = CABLE_Z - CABLE_R
    for d in REST_AT:
        shapes += [BoxShape((0.0, d, h / 2), (0.03, 0.016, h), DARK),
                   BoxShape((0.007, d, h + 0.003), (0.006, 0.016, 0.006), DARK),  # lips either side of the cable
                   BoxShape((-0.007, d, h + 0.003), (0.006, 0.016, 0.006), DARK)]
    return Body.build("rest", pos=(TIP0[0], TIP0[1], TOP), mass=0.3, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=DARK)


def _cable() -> Body:
    shapes = [CylShape((-STRIP_L / 2, 0.0, 0.0), STRIP_R, STRIP_L, COPPER, axis="X"),
              CylShape((-STRIP_L - (CABLE_L - STRIP_L) / 2, 0.0, 0.0), CABLE_R, CABLE_L - STRIP_L, BLACK, axis="X")]
    return Body.build("cable", pos=(TIP0[0], TIP0[1], TOP + CABLE_Z + 0.0005), mass=CABLE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLACK, com=(-CABLE_L / 2, 0.0, 0.0), yaw=-math.pi / 2)


PRESS, PEDAL_BASE, PEDAL, REST, CABLE = _press(), _pedal_base(), _pedal_cover(), _rest(), _cable()
OBJECTS = [PRESS, PEDAL_BASE, PEDAL, REST, CABLE, _upper_jaw(), _lug(), _crimped(), _crushed()]
hinge("pedal_pin", "pedal_base", "pedal", anchor=(PED_X0, PED_Y, TOP + PED_BODY), axis="Y",  # pressed: negative
      limits_deg=(-PED_STOP_DEG, 0.0), spring=PED_SPRING, damping=0.002, target_deg=-PED_SPRING_TO)

LUG_AT = MOUTH.copy()  # the lug's origin as it lies in the die
PRESS_PT = np.array([PED_X0 - PED_PRESS, PED_Y, TOP + PED_TOPH])  # where a hand presses the pedal, at rest


# -- followed after every control step: the valve, the jaws and the crimp -------------------------------------------

def _fresh() -> dict:
    return {"crimped": False, "good": False, "depth": None, "align": None, "best_depth": -1.0, "tcps": None,
            "pedal_deg": 0.0, "valve": False, "jaw": 0.0, "on_cable": None, "tip": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CRIMP_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pedal_deg(p_rot) -> float:
    """The pedal cover's turn from rest (positive: pressed down, a negative turn about y)."""
    return -math.degrees(math.atan2(p_rot[0, 2], p_rot[0, 0]))


def _insertion(c_pos, c_rot) -> tuple[float, float, float]:
    """(how far the cable's tip is past the barrel's mouth, along the barrel; the cable's angle from the barrel's axis,
    degrees; the tip's distance from that axis)."""
    rel = np.asarray(c_pos, float) - MOUTH
    depth = float(rel @ BAR_DIR)
    axis = c_rot @ np.array([1.0, 0.0, 0.0])
    align = math.degrees(math.acos(max(-1.0, min(1.0, float(axis @ BAR_DIR)))))
    return depth, align, float(np.linalg.norm(rel - depth * BAR_DIR))


def _quat_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = s / 4
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return tuple(q)


def _place(base, origin, name, pos, rot) -> None:
    import torch

    asset = base.scene[name]
    w, x, y, z = _quat_wxyz(np.asarray(rot, float))
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), x, y, z, w]], dtype=torch.float32, device=asset.device))


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _crimp(base, origin, c_pos, c_rot, depth, off) -> None:
    """The jaws have closed: with the copper in the barrel, the lug is crimped onto it and from now on goes where the
    cable goes; with no copper in it, the barrel is crushed shut."""
    _place(base, origin, "lug", HIDE, LUG_ROT)
    if depth > 0.0 and off < BAR_RI:
        _EP["on_cable"] = (c_rot.T @ (LUG_AT - c_pos), c_rot.T @ LUG_ROT)  # the lug in the cable's frame
        _place(base, origin, "crimped_lug", LUG_AT, LUG_ROT)
    else:
        _place(base, origin, "crushed_lug", LUG_AT, LUG_ROT)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["cable"].data.root_link_pose_w)[0]
    pv = as_numpy(base.scene["pedal"].data.root_link_pose_w)[0]
    c_pos, c_rot = c[:3] - origin, rotation(c[3:7])
    depth, align, off = _insertion(c_pos, c_rot)
    if off < BAR_RI and not (_EP["crimped"] and _EP["on_cable"] is None):  # nothing goes into a crushed barrel
        _EP["best_depth"] = max(_EP["best_depth"], depth)
    _EP["pedal_deg"] = _pedal_deg(rotation(pv[3:7]))
    _EP["valve"] = _EP["pedal_deg"] >= PED_SWITCH_DEG  # the 5/2 valve: pressed, it drives the cylinder closed
    was = _EP["jaw"]
    _EP["jaw"] = min(1.0, was + DT / T_CLOSE) if _EP["valve"] else max(0.0, was - DT / T_OPEN)
    if _EP["jaw"] != was:
        _place(base, origin, "upper_jaw", JAW_PIVOT + [0.0, 0.0, TOP], _rot_y(JAW_OPEN_DEG * (1.0 - _EP["jaw"])))
    _EP["tcps"] = _tcps(base, origin)
    if not _EP["crimped"] and _EP["jaw"] >= 1.0:  # the jaws have closed fully: the crimp
        _EP["crimped"] = True
        _EP["depth"], _EP["align"] = depth, align
        _EP["good"] = DEPTH[0] <= depth <= DEPTH[1] and align <= ALIGN_DEG and off < BAR_RI
        _crimp(base, origin, c_pos, c_rot, depth, off)
    elif _EP["on_cable"] is not None:  # the crimped lug goes with the cable
        p_rel, r_rel = _EP["on_cable"]
        _place(base, origin, "crimped_lug", c_pos + c_rot @ p_rel, c_rot @ r_rel)
    _EP["tip"] = c_pos
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[crimp] t={_DEBUG['steps'] / 20:5.1f}s pedal={_EP['pedal_deg']:5.2f} valve={int(_EP['valve'])} "
                  f"jaw={_EP['jaw']:.2f} depth={depth * 1000:6.1f}mm align={align:5.1f} "
                  f"axis={np.round(c_rot[:, 0], 3).tolist()} off={off * 1000:5.2f}mm "
                  f"tipxz=({(c_pos[0] - MOUTH[0]) * 1000:.2f},{(c_pos[2] - MOUTH[2]) * 1000:.2f})mm "
                  f"crimped={_EP['crimped']} good={_EP['good']} "
                  f"at_crimp=({None if _EP['depth'] is None else round(_EP['depth'] * 1000, 1)}mm, "
                  f"{None if _EP['align'] is None else round(_EP['align'], 1)}deg) "
                  f"leftTCP-tip={np.round((tl - c_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-press={np.round((tr - PRESS_PT) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or all(float(np.linalg.norm(t - PRESS_PT)) >= LET_GO for t in tcps)
    tip = _EP["tip"]
    out = tip is not None and float(np.linalg.norm(tip - MOUTH)) >= OUT
    return {"crimped": _EP["crimped"] and _EP["on_cable"] is not None, "good": _EP["good"],
            "released": not _EP["valve"] and _EP["jaw"] <= 0.0 and let_go, "inserted": _EP["best_depth"] >= INSERTED,
            "out": out and _EP["on_cable"] is not None}


def _points(st) -> float:
    if st["good"] and st["released"] and st["out"]:
        return 1.0
    if st["crimped"]:
        return 0.5
    return 0.25 if st["inserted"] else 0.0


def score(objects) -> float:
    """1.0 for a good crimp (the bare end 10.5 to 13.5 mm into the 13 mm barrel and the cable within 8 degrees of its
    axis when the jaws closed) with the pedal let go and the jaws open again, the cable taken out of the press with the
    lug on it (its tip at least 5 cm from where the barrel's mouth was) and laid back on its rests, and both arms back
    where they started; 0.5 for any crimp with the bare end in the barrel; 0.25 if the end was pushed at least 6.5 mm
    into the barrel at some point; else 0.0."""
    pts = _points(_status())
    if pts >= 1.0 and not (_on_rest(objects) and _arms_home()):
        return 0.5  # crimped and taken out, but not laid back on its rests or the arms not back
    return pts


def _on_rest(objects) -> bool:
    """The cable laid back on its rests: its tip within 3 cm (across) and 1.5 cm (up or down) of where it lay, and the
    cable within 20 degrees of lying along the rests."""
    c = objects["cable"]
    tip, axis = np.asarray(c.pos, float), rotation(c.quat_xyzw)[:, 0]
    return (float(np.linalg.norm(tip[:2] - TIP0)) <= ON_REST[0] and abs(float(tip[2]) - (TOP + CABLE_Z)) <= ON_REST[1]
            and abs(float(axis[1])) >= math.cos(math.radians(ON_REST[2])))


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
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
    """Full score with the pedal and the cable still: the trial ends there."""
    return score(objects) >= 1.0 and objects["pedal"].resting() and objects["cable"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable 7.5 cm from its tip, fingers closing across it, lifts it off its
# rests, lines it up with the barrel 3 cm outside the guard and pushes the bare end straight in through the feed hole,
# aiming 2.5 mm past full depth so that the insulation meeting the barrel's mouth sets the depth, and holds it. The right
# hand comes straight down on the pedal's high end, fingers closed, presses it down to its stop and holds it while the
# jaws close (the crimp), then rises: the valve returns, the jaws open, and the right goes back to where it started. The
# left pulls the cable straight back out through the feed hole with the lug crimped on its end, holds it up to show it,
# lays it back on its rests and goes back to where it started.

GRIP_L = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the cable, which runs along y)
GRIP_R = grip_rotation(0.0)  # straight down, fingers closed, pressing the pedal
HOLD_AT = 0.075  # the left TCP this far back from the cable's tip (between the rests)
IN_PAST = 0.0025  # aim this far past full depth: the insulation stops the cable at the barrel's mouth
PRESS_BELOW = 0.07  # aim this far below the pedal's resting top (2.5 cm of travel, then the arm gives: about 19 N)
SAG = 0.006  # the held cable hangs this far below the left TCP (the jaw pushes it toward the fingertips): aim above
AIM = np.array([-0.0013, 0.0, -0.0039])  # trim of the left hand's aim at the barrel (x, z), from debug runs
PROBE_IN, PROBE_R, PROBE_STEPS, PROBE_S = 0.004, (0.0002, 0.0015), 24, 0.15  # the search for the barrel's mouth
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tcp_for_tip(tip_y: float) -> np.ndarray:
    """The left TCP holding the cable on the barrel's axis with its tip at ``tip_y``."""
    return np.array([BX, tip_y + HOLD_AT, TOP + AXIS_Z + SAG]) + AIM


def _left() -> tuple[list[Waypoint], float]:
    pick = np.array([TIP0[0], TIP0[1] + HOLD_AT, TOP + CABLE_Z + 0.0005])
    front = _tcp_for_tip(GUARD_Y[1] + 0.03)
    pre = _tcp_for_tip(MOUTH[1] + 0.005)
    at_mouth = _tcp_for_tip(MOUTH[1] + 0.0005)
    held = _tcp_for_tip(MOUTH[1] - BAR_L - IN_PAST)
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], GRIP_L, "over the cable", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP_L),
            _w(0.8, pick, FINGER_CLOSED, "grip it", GRIP_L),
            _w(0.8, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it off the rests", GRIP_L),
            _w(1.5, front, FINGER_CLOSED, "line it up with the barrel, 3 cm outside the guard", GRIP_L),
            _w(0.8, front, FINGER_CLOSED, "settle", GRIP_L),
            _w(1.2, pre, FINGER_CLOSED, "through the feed hole, the tip 5 mm from the barrel's mouth", GRIP_L),
            _w(0.6, pre, FINGER_CLOSED, "settle", GRIP_L),
            _w(1.0, at_mouth, FINGER_CLOSED, "the tip just short of the mouth", GRIP_L),
            _w(0.8, at_mouth, FINGER_CLOSED, "settle", GRIP_L)]
    # feel for the mouth: press lightly (aiming 4 mm past it) while the tip circles out to 1.5 mm off the axis; once the
    # tip finds the chamfer it slides in, and the barrel then holds it while the hand finishes the circles
    probe = _tcp_for_tip(MOUTH[1] - PROBE_IN)
    for k in range(PROBE_STEPS):
        r = PROBE_R[0] + (PROBE_R[1] - PROBE_R[0]) * k / (PROBE_STEPS - 1)
        a = 2 * math.pi * k / 8
        out.append(_w(PROBE_S, probe + [r * math.cos(a), 0.0, r * math.sin(a)], FINGER_CLOSED,
                      "feel for the barrel's mouth: small circles, pressing lightly", GRIP_L))
    out += [_w(0.5, probe, FINGER_CLOSED, "back on the axis", GRIP_L),
            _w(3.0, held, FINGER_CLOSED, "push the bare end straight in until the insulation meets the barrel", GRIP_L)]
    return out, sum(w.duration for w in out)


def _press_top(deg: float) -> np.ndarray:
    """The top of the pedal's flat (world) at the press point with the cover turned ``deg``."""
    p = np.array([-PED_PRESS, 0.0, PED_TOPH - PED_BODY]) @ _rot_y(-deg).T
    return np.array([PED_X0, PED_Y, TOP + PED_BODY]) + p


def _right(start: float) -> list[Waypoint]:
    above = _press_top(0.0) + [0.0, 0.0, 0.04]
    out = _lead_in("right", above, GRIP_R, "over the pedal", FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(start - t, 0.2), above, FINGER_CLOSED, "wait for the cable to be held in the barrel", GRIP_R))
    press = _press_top(0.0) - [0.0, 0.0, PRESS_BELOW]
    out += [_w(0.8, _press_top(0.0) + [0.0, 0.0, 0.005], FINGER_CLOSED, "onto the pedal", GRIP_R),
            _w(1.2, press, FINGER_CLOSED, "press the pedal down: the valve drives the jaws closed", GRIP_R),
            _w(T_CLOSE + 0.8, press, FINGER_CLOSED, "hold it down while the jaws close: the crimp", GRIP_R),
            _w(1.2, above, FINGER_CLOSED, "rise: the pedal springs up, the jaws open", GRIP_R)]
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


PULL_OUT = 0.085  # the left pulls the cable this far straight back out, through the guard's feed hole
SHOW_UP = 0.06  # ... and lifts it this much, to show the lug on its end


def demo():
    left, t_held = _left()
    right = _right(t_held + 0.3)
    t_r = sum(w.duration for w in right)
    right += _go_home("right", right[-1].pos, GRIP_R, FINGER_CLOSED)
    held = np.asarray(left[-1].pos)
    pulled = held + [0.0, PULL_OUT, 0.0]
    shown = pulled + [0.0, 0.0, SHOW_UP]
    rest = np.array([TIP0[0], TIP0[1] + HOLD_AT, TOP + CABLE_Z + SAG])  # the cable back on its rests
    left += [_w(max(t_r - t_held, 0.2) + T_OPEN + 0.3, held, FINGER_CLOSED, "hold the cable until the jaws are open",
                GRIP_L),
             _w(2.0, pulled, FINGER_CLOSED, "pull it straight out: the lug comes with it, crimped on", GRIP_L),
             _w(1.0, shown, FINGER_CLOSED, "hold it up to show the lug", GRIP_L),
             _w(1.5, shown, FINGER_CLOSED, "hold it up to show the lug", GRIP_L),
             _w(1.5, rest + [0.0, 0.0, 0.03], FINGER_CLOSED, "over its rests", GRIP_L),
             _w(0.8, rest, FINGER_CLOSED, "lay it on its rests", GRIP_L),
             _w(0.5, rest, FINGER_OPEN, "let go", GRIP_L),
             _w(0.8, rest + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", GRIP_L)]
    left += _go_home("left", rest + [0.0, 0.0, 0.05], GRIP_L, FINGER_OPEN)
    t_l = sum(w.duration for w in left)
    left.append(_w(3.0, left[-1].pos, FINGER_OPEN,
                   "hold still: the trial ends once the full score has held 1 s with both arms home", left[-1].yaw))
    t_r = sum(w.duration for w in right)
    right.append(_w(max(t_l - t_r, 0.5) + 3.0, right[-1].pos, FINGER_CLOSED, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
