"""On the bench lies a battery-powered hydraulic lug crimper (a CM-300G: 75 kN, 16-300 mm2 dies, 18 V), strapped on its
side in a plywood holder, its head turned so the head's opening faces up, inside a clear guard. A 16 mm2 tube cable lug
lies in the head's open 16 mm2 die, its barrel pointing at the arms, its palm in a lug holder. The crimper's grip, with
its trigger, points away from the arms on the right; a short 16 mm2 cable with a stripped end lies on two rests to the
left.

Task: crimp the lug onto the cable's end. One hand picks up the cable, pushes its stripped end through the guard's
feed hole into the lug's barrel, straight along it, until the insulation meets the barrel's mouth, and holds it there;
the other squeezes the crimper's trigger against its grip, as a hand does, and holds it while the ram closes the die on
the barrel; the ram then goes back by itself, and the hand lets go. Then the first hand pulls the cable out with the lug
crimped on its end, holds it up to show it, and lays it back on its rests, and both arms go back to where they started.

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

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the crimper: a CM-300G battery hydraulic crimper (75 kN, 16-300 mm2 dies, 17 mm stroke, 18 V 3.0 Ah), lying on its
# side in its holder with its axis across the bench (along y): its head on the left, its body, grip and battery to the
# right, the grip pointing away from the arms. Its head is turned on the tool's axis (as it is made to) so the head's
# opening faces up: the die closes along y, and the barrel runs along x through the head. Sizes are scaled from the
# maker's photo by the 18 V pack (115 mm long). A point (along the tool's axis toward the head, along its grip's
# up-direction, along its width) maps to the bench as x = BX - up, y = YD + along, z = AXIS_Z + width. ---------------
BX, YD = 0.30, 0.06  # the die's centre (the barrel's axis), forward from the line between the arm bases and to the left
AXIS_Z = 0.075  # the tool's axis and the die's centre above the bench, set by the holder
DIE_W = 0.010  # the die's width along the barrel (inside the 13 mm barrel)
STROKE = 0.017  # the ram's stroke (maker): the moving die starts 17 mm from closed
HEAD_T = 0.030  # the head's frame, front to back (along the barrel)
T_CLOSE, T_RETURN = 3.0, 2.0  # s for the ram to close the die and to go back (design values: tools of this class are
# listed at 3-6 s a crimp)
BASE_T = 0.018  # the holder's plywood base

# -- the trigger: a cordless-tool trigger at the top of the grip, squeezed toward the grip's back (-y here) ----------
TRIG_FRONT = 0.143  # its front face this far behind the die's centre along the tool's axis
TRIG_DEPTH = 0.017  # it stands 17 mm proud of the grip's front
TRIG_UP = (0.050, 0.076)  # its span below the tool's axis, along the grip
TRIG_W = 0.012  # its width
GRIP_BACK = 0.205  # the grip's back, behind the die's centre along the tool's axis
TRIG_TRAVEL, TRIG_SWITCH = 0.0085, 0.0020  # its full travel and where the motor switches on (as a cordless-tool trigger
# switch, e.g. Omron C3AW: on at 2.0 mm of 8.5 mm, at most 20 N)
TRIG_K, TRIG_PRE = 1200.0, 0.005  # N/m; pulled 5 mm past its rest stop: 6.0 N at rest, 8.4 N at the switch, 16.2 N at
# its stop (inside the 20 N such switches are rated for)

# -- the lug: a Klauke 3R8 tubular cable lug, Cu tinned, 16 mm2, M8: barrel 5.5 mm inside, 8.5 mm outside, 13 mm long;
# palm 15 mm wide, 29 mm overall, 8.5 mm hole. Its frame: origin at the barrel's mouth, x along the barrel into it. --
BAR_RI, BAR_T, BAR_L = 0.00275, 0.0015, 0.013
CHAMFER = 0.0005  # the barrel's internal chamfer at its mouth, 45 degrees (Klauke: "precise internal chamfer"; design
# value)
PALM_L, PALM_W, PALM_T = 0.016, 0.015, 0.003  # 29 mm overall less the 13 mm barrel; 3 mm: the tube flattened
HOLE_R, HOLE_AT = 0.00425, 0.008  # the 8.5 mm stud hole, its centre 8 mm from the palm's end
MOUTH = np.array([BX - BAR_L / 2, YD, TOP + AXIS_Z])  # the barrel centred on the die: its mouth 6.5 mm in front of it
BAR_DIR = np.array([1.0, 0.0, 0.0])  # into the barrel (away from the arms)
LUG_ROT = np.eye(3)  # the lug's frame in the world
STOP_L = 0.016  # the lug stop behind the palm's end

# -- the clear guard (3 mm polycarbonate) round the head, with a 22 mm square feed hole for the cable ---------------
GUARD_T = 0.003
GUARD_GAP = 0.018  # its front wall's inside face this far in front of the barrel's mouth
GUARD_HOLE = 0.022
GUARD_X = (MOUTH[0] - GUARD_GAP - GUARD_T, BX + 0.048)  # outside faces: front and back walls
GUARD_Y = (YD - 0.098, YD + 0.070)  # outside faces: right wall (the tool's neck passes through it) and left wall
GUARD_H = 0.150
NECK_R = 0.024

# -- the cable: 16 mm2 single-core H07V-R; its frame: origin at the stripped tip, the cable along -x -----------------
STRIP_L, STRIP_R = 0.013, 0.00255  # 7 x 1.7 mm strands, 5.1 mm across, stripped to the barrel's 13 mm
CABLE_L, CABLE_R = 0.16, 0.0035  # 7 mm over the insulation
CABLE_MASS = 0.027  # 23 g of copper (16 mm2 x 16 cm x 8.96 g/cm3) and 4 g of PVC
TIP0 = np.array([0.21, 0.16])  # where the cable's tip lies on its rests, pointing away from the arms
CABLE_Z = 0.03  # the cable's axis above the bench on its rests
REST_AT = (0.045, 0.13)  # the two V-rests, along the cable from its tip (its centre of mass, 8 cm, between them)

COPPER, BLACK, RED, STEEL, DARK, YELLOW, BEIGE, WHITE = (
    (0.80, 0.50, 0.25), (0.08, 0.08, 0.09), (0.75, 0.10, 0.10), (0.62, 0.63, 0.66), (0.20, 0.21, 0.23),
    (0.95, 0.72, 0.10), (0.80, 0.68, 0.48), (0.92, 0.92, 0.90))
CLEAR = (0.80, 0.90, 0.97)
EDGE = (0.70, 0.80, 0.88)  # the guard's edges, drawn in place of its clear panels

# -- scoring ----------------------------------------------------------------------------------------------------------
# a good crimp: the bare end this far into the 13 mm barrel when the die closes (all but at most 2.5 mm of the copper
# in: the insulation, too wide for the barrel, within 2.5 mm of its mouth) and the cable within this angle of its axis
DEPTH = (0.0105, 0.0135)
ALIGN_DEG = 8.0
INSERTED = 0.0065  # the bare end pushed this far in (half the barrel) at some point
LET_GO = 0.04  # both TCPs this far from the trigger: let go
OUT = 0.05  # the cable's tip this far from the barrel's mouth: taken out of the crimper (past the guard)
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home
ON_REST = (0.03, 0.015, 20.0)  # laid back on its rests: the tip this close to where it lay (across, up) and the cable
# within this many degrees of lying along its rests
SUCCESS_HOLD_S = 1.0
DT = 0.05  # s per control step (20 Hz)


def _w2(up: float, along: float) -> tuple[float, float]:
    """(x, y) on the bench of a point ``along`` the tool's axis (toward the head) and ``up`` its grip's up-direction."""
    return BX - up, YD + along


def _box(up0, up1, al0, al1, w0, w1, color, collide=True) -> BoxShape:
    """A box spanning ``up`` (grip's up-direction), ``al`` (along the axis) and ``w`` (width, i.e. height here)."""
    x0, y0 = _w2(up1, al0)
    x1, y1 = _w2(up0, al1)
    return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, AXIS_Z + (w0 + w1) / 2), (x1 - x0, y1 - y0, w1 - w0), color,
                    collide=collide)


def _tool() -> Body:
    """The fixed parts: the holder (base, saddles, straps, clamps), the crimper (head with its fixed die, neck, body,
    motor, grip, battery), the clear guard and the lug holder."""
    shapes = []
    # holder: 18 mm plywood base clamped to the bench at its far corners, saddles under the tool, two straps
    bx0, bx1, by0, by1 = BX - 0.10, BX + 0.25, YD - 0.25, YD + 0.08
    shapes.append(BoxShape(((bx0 + bx1) / 2, (by0 + by1) / 2, BASE_T / 2), (bx1 - bx0, by1 - by0, BASE_T), BEIGE))
    for yc in (by0 + 0.03, by1 - 0.03):  # C-clamps on the base's far edge (shown only)
        shapes += [BoxShape((bx1 - 0.02, yc, BASE_T + 0.006), (0.04, 0.02, 0.012), RED, collide=False),
                   BoxShape((bx1 + 0.004, yc, BASE_T / 2), (0.008, 0.02, 0.05), RED, collide=False)]
    sad = AXIS_Z - 0.0375 - BASE_T  # body and battery 75 mm wide: their underside 37.5 mm below the axis
    shapes += [BoxShape((BX, YD - 0.17, BASE_T + sad / 2), (0.06, 0.08, sad), BEIGE),  # under the body
               BoxShape((BX + 0.19, YD - 0.17, BASE_T + sad / 2), (0.05, 0.08, sad), BEIGE),  # under the battery
               BoxShape((BX - 0.06, YD - 0.17, BASE_T + (AXIS_Z - 0.024 - BASE_T) / 2),
                        (0.04, 0.08, AXIS_Z - 0.024 - BASE_T), BEIGE),  # under the motor
               BoxShape((BX, YD - 0.01, BASE_T + (AXIS_Z - 0.045 - BASE_T) / 2), (HEAD_T, 0.10, AXIS_Z - 0.045 - BASE_T),
                        BEIGE)]  # under the head
    for xs, ys, xl in ((BX, YD - 0.15, 0.115), (BX + 0.19, YD - 0.20, 0.07)):  # straps over the body and the battery
        shapes.append(BoxShape((xs, ys, AXIS_Z + 0.0385), (xl, 0.025, 0.002), BLACK, collide=False))
    # crimper head: frame (C back, top latch, bottom, ram housing), fixed die on the +y side
    h = HEAD_T / 2
    shapes += [BoxShape((BX, YD + 0.050, AXIS_Z), (HEAD_T, 0.020, 0.090), STEEL),  # C back
               BoxShape((BX, YD - 0.016, AXIS_Z + 0.040), (HEAD_T, 0.152, 0.010), STEEL),  # latch, over the top
               BoxShape((BX, YD - 0.016, AXIS_Z - 0.040), (HEAD_T, 0.152, 0.010), STEEL),  # bottom of the frame
               BoxShape((BX, YD - 0.076, AXIS_Z), (HEAD_T, 0.032, 0.090), STEEL),  # ram housing
               BoxShape((BX, YD + (BAR_RI + BAR_T + 0.040) / 2, AXIS_Z),
                        (DIE_W, 0.040 - BAR_RI - BAR_T, 0.028), DARK),  # fixed half of the 16 mm2 die
               BoxShape((BX, YD + 0.030, AXIS_Z), (HEAD_T - 0.004, 0.020, 0.034), STEEL),  # die holder
               BoxShape((BX - h - 0.0005, YD + 0.030, AXIS_Z + 0.008), (0.001, 0.008, 0.008), WHITE, collide=False)]
    # neck, body (gearbox), motor, grip with the trigger guard, battery mount and battery
    shapes += [CylShape((BX, YD - 0.103, AXIS_Z), NECK_R, 0.022, STEEL, axis="Y"),
               _box(-0.034, 0.034, -0.226, -0.114, -0.0375, 0.0375, YELLOW),  # body
               CylShape((BX - 0.060, YD - 0.170, AXIS_Z), 0.024, 0.112, BLACK, axis="Y"),  # motor
               _box(-0.124, -0.034, -GRIP_BACK, -TRIG_FRONT - TRIG_DEPTH, -0.020, 0.020, BLACK),  # grip
               _box(-TRIG_UP[0], -0.034, -0.170, -0.155, -0.015, 0.015, YELLOW),  # the grip's top, behind the trigger
               _box(-0.162, -0.124, -0.225, -0.130, -0.030, 0.030, YELLOW),  # battery mount
               _box(-0.224, -0.162, -0.228, -0.114, -0.0375, 0.0375, BLACK)]  # 18 V battery
    # the clear guard: front wall with the feed hole, back wall, left wall, right wall round the neck, lid
    gx0, gx1 = GUARD_X
    gy0, gy1 = GUARD_Y
    z0, zt = BASE_T, GUARD_H
    xf = gx0 + GUARD_T / 2
    hz0, hz1 = AXIS_Z - GUARD_HOLE / 2, AXIS_Z + GUARD_HOLE / 2
    hy0, hy1 = YD - GUARD_HOLE / 2, YD + GUARD_HOLE / 2
    ny0, ny1 = AXIS_Z - NECK_R - 0.002, AXIS_Z + NECK_R + 0.002  # the right wall's cut-out round the neck (z)
    nx0, nx1 = BX - NECK_R - 0.002, BX + NECK_R + 0.002
    yr = gy0 + GUARD_T / 2
    panels = [((xf, (gy0 + gy1) / 2, (z0 + hz0) / 2), (GUARD_T, gy1 - gy0, hz0 - z0)),  # front, below the hole
              ((xf, (gy0 + gy1) / 2, (hz1 + zt) / 2), (GUARD_T, gy1 - gy0, zt - hz1)),  # front, above it
              ((xf, (gy0 + hy0) / 2, AXIS_Z), (GUARD_T, hy0 - gy0, GUARD_HOLE)),  # front, beside it
              ((xf, (hy1 + gy1) / 2, AXIS_Z), (GUARD_T, gy1 - hy1, GUARD_HOLE)),
              ((gx1 - GUARD_T / 2, (gy0 + gy1) / 2, (z0 + zt) / 2), (GUARD_T, gy1 - gy0, zt - z0)),  # back
              (((gx0 + gx1) / 2, gy1 - GUARD_T / 2, (z0 + zt) / 2), (gx1 - gx0, GUARD_T, zt - z0)),  # left
              (((gx0 + gx1) / 2, yr, (z0 + ny0) / 2), (gx1 - gx0, GUARD_T, ny0 - z0)),  # right, round the neck
              (((gx0 + gx1) / 2, yr, (ny1 + zt) / 2), (gx1 - gx0, GUARD_T, zt - ny1)),
              (((gx0 + nx0) / 2, yr, AXIS_Z), (nx0 - gx0, GUARD_T, ny1 - ny0)),
              (((nx1 + gx1) / 2, yr, AXIS_Z), (gx1 - nx1, GUARD_T, ny1 - ny0)),
              (((gx0 + gx1) / 2, (gy0 + gy1) / 2, zt + GUARD_T / 2), (gx1 - gx0, gy1 - gy0, GUARD_T))]  # lid
    shapes += [BoxShape(c, s, CLEAR, visible=False) for c, s in panels]  # clear: solid, but drawn only by its edges
    e = 0.003  # its edges and the feed hole's rim, shown only
    ztop = zt + GUARD_T
    for x in (gx0 + e / 2, gx1 - e / 2):
        for z in (z0 + e / 2, ztop - e / 2):
            shapes.append(BoxShape((x, (gy0 + gy1) / 2, z), (e, gy1 - gy0, e), EDGE, collide=False))
        for y in (gy0 + e / 2, gy1 - e / 2):
            shapes.append(BoxShape((x, y, (z0 + ztop) / 2), (e, e, ztop - z0), EDGE, collide=False))
    for y in (gy0 + e / 2, gy1 - e / 2):
        for z in (z0 + e / 2, ztop - e / 2):
            shapes.append(BoxShape(((gx0 + gx1) / 2, y, z), (gx1 - gx0, e, e), EDGE, collide=False))
    for z in (hz0 - e / 2, hz1 + e / 2):
        shapes.append(BoxShape((gx0 + e / 2, YD, z), (e + 0.0005, GUARD_HOLE + 2 * e, e), EDGE, collide=False))
    for y in (hy0 - e / 2, hy1 + e / 2):
        shapes.append(BoxShape((gx0 + e / 2, y, AXIS_Z), (e + 0.0005, e, GUARD_HOLE), EDGE, collide=False))
    # the lug holder behind the head: a rest under the palm and a stop at its end
    p0 = BX + h  # the head's back face
    end = MOUTH[0] + BAR_L + PALM_L  # the palm's end
    pz = AXIS_Z - PALM_T / 2
    shapes += [BoxShape(((p0 + end) / 2 + 0.0005, YD, (BASE_T + pz) / 2), (end - p0 - 0.001, 0.020, pz - BASE_T), DARK),
               BoxShape((end + STOP_L / 2, YD, (BASE_T + AXIS_Z + 0.006) / 2), (STOP_L, 0.024, AXIS_Z + 0.006 - BASE_T),
                        DARK)]
    return Body.build("tool", pos=(0.0, 0.0, TOP), mass=6.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=YELLOW)


RAM0 = np.array([BX, YD - BAR_RI - BAR_T - STROKE, TOP + AXIS_Z])  # the moving die's face, open


def _ram() -> Body:
    """The moving half of the die on the ram (shown only): it closes 17 mm onto the barrel as the ram runs."""
    shapes = [BoxShape((0.0, -0.012, 0.0), (DIE_W, 0.024, 0.028), DARK, collide=False),
              BoxShape((0.0, -0.030, 0.0), (HEAD_T - 0.004, 0.016, 0.034), STEEL, collide=False)]
    return Body.build("ram", pos=tuple(RAM0.tolist()), mass=0.3, shapes=shapes, kinematic=True, color=STEEL)


TRIG_C = np.array([*_w2(-(TRIG_UP[0] + TRIG_UP[1]) / 2, -TRIG_FRONT - TRIG_DEPTH / 2), TOP + AXIS_Z])  # its centre


def _trigger() -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.0), (TRIG_UP[1] - TRIG_UP[0], TRIG_DEPTH, TRIG_W), BLACK)]
    return Body.build("trigger", pos=tuple(TRIG_C.tolist()), mass=0.02, shapes=shapes, friction=(1.0, 0.9), color=BLACK)


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


HIDE = np.array([BX, YD + 0.03, TOP - 0.03])  # under the bench
HIDE2 = np.array([BX, YD - 0.03, TOP - 0.03])


def _lug() -> Body:
    return Body.build("lug", pos=tuple(MOUTH.tolist()), mass=0.009, shapes=_lug_shapes(BAR_RI, True, chamfer=CHAMFER),
                      kinematic=True, friction=(0.3, 0.25), slick=True, color=COPPER)  # tinned, smooth


def _crimped() -> Body:
    """The lug crimped onto the copper (its barrel squeezed onto it): shown once crimped, following the cable."""
    return Body.build("crimped_lug", pos=tuple(HIDE.tolist()), mass=0.009, kinematic=True, color=COPPER,
                      shapes=_lug_shapes(STRIP_R + 0.0003, False))


def _crushed() -> Body:
    """The lug crushed empty (the die closed with no copper in it): its barrel closed, nothing goes in any more."""
    return Body.build("crushed_lug", pos=tuple(HIDE2.tolist()), mass=0.009, kinematic=True, color=COPPER,
                      shapes=_lug_shapes(0.0008, True, solid=True))


def _rest() -> Body:
    shapes = []
    hgt = CABLE_Z - CABLE_R
    for d in REST_AT:
        shapes += [BoxShape((-d, 0.0, hgt / 2), (0.016, 0.03, hgt), DARK),
                   BoxShape((-d, 0.007, hgt + 0.003), (0.016, 0.006, 0.006), DARK),  # lips either side of the cable
                   BoxShape((-d, -0.007, hgt + 0.003), (0.016, 0.006, 0.006), DARK)]
    return Body.build("rest", pos=(TIP0[0], TIP0[1], TOP), mass=0.3, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=DARK)


def _cable() -> Body:
    shapes = [CylShape((-STRIP_L / 2, 0.0, 0.0), STRIP_R, STRIP_L, COPPER, axis="X"),
              CylShape((-STRIP_L - (CABLE_L - STRIP_L) / 2, 0.0, 0.0), CABLE_R, CABLE_L - STRIP_L, BLACK, axis="X")]
    return Body.build("cable", pos=(TIP0[0], TIP0[1], TOP + CABLE_Z + 0.0005), mass=CABLE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLACK, com=(-CABLE_L / 2, 0.0, 0.0))


TOOL, TRIGGER, REST, CABLE = _tool(), _trigger(), _rest(), _cable()
OBJECTS = [TOOL, TRIGGER, REST, CABLE, _ram(), _lug(), _crimped(), _crushed()]
slider("trigger_slide", "tool", "trigger", anchor=tuple(TRIG_C.tolist()), axis="Y", limits=(-TRIG_TRAVEL, 0.0),
       stiffness=TRIG_K, damping=4.0, target=TRIG_PRE, max_force=200.0)  # squeezed: toward -y

LUG_AT = MOUTH.copy()  # the lug's origin as it lies in the die


# -- followed after every control step: the trigger, the ram and the crimp -------------------------------------------

def _fresh() -> dict:
    return {"crimped": False, "good": False, "depth": None, "align": None, "best_depth": -1.0, "tcps": None,
            "trig": 0.0, "on": False, "armed": True, "ram": 0.0, "back": False, "on_cable": None, "tip": None}


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


def _crimp(base, origin, c_pos, c_rot, depth, off) -> None:
    """The die has closed: with the copper in the barrel, the lug is crimped onto it and from now on goes where the
    cable goes; with no copper in it, the barrel is crushed shut."""
    _place(base, origin, "lug", HIDE, LUG_ROT)
    if depth > 0.0 and off < BAR_RI:
        _EP["on_cable"] = (c_rot.T @ (LUG_AT - c_pos), c_rot.T @ LUG_ROT)  # the lug in the cable's frame
        _place(base, origin, "crimped_lug", LUG_AT, LUG_ROT)
    else:
        _place(base, origin, "crushed_lug", LUG_AT, LUG_ROT)


def _ram_step() -> None:
    """The crimper: the trigger squeezed past its switch point runs the pump and the ram closes the die (T_CLOSE for
    the full stroke); let go before it closes, the ram stops where it is. Once the die has closed, the pressure valve
    opens and the ram goes back (T_RETURN), held trigger or not; it runs again only after the trigger is let go."""
    if _EP["back"]:
        _EP["ram"] = max(0.0, _EP["ram"] - DT / T_RETURN)
        _EP["back"] = _EP["ram"] > 0.0
    elif _EP["on"] and _EP["armed"]:
        _EP["ram"] = min(1.0, _EP["ram"] + DT / T_CLOSE)
    if not _EP["on"]:
        _EP["armed"] = True


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["cable"].data.root_link_pose_w)[0]
    tr = as_numpy(base.scene["trigger"].data.root_link_pose_w)[0]
    c_pos, c_rot = c[:3] - origin, rotation(c[3:7])
    depth, align, off = _insertion(c_pos, c_rot)
    if off < BAR_RI and not (_EP["crimped"] and _EP["on_cable"] is None):  # nothing goes into a crushed barrel
        _EP["best_depth"] = max(_EP["best_depth"], depth)
    _EP["trig"] = float(TRIG_C[1] - (tr[1] - origin[1]))  # how far the trigger is squeezed in
    _EP["on"] = _EP["trig"] >= TRIG_SWITCH
    was = _EP["ram"]
    _ram_step()
    if _EP["ram"] != was:
        _place(base, origin, "ram", RAM0 + [0.0, STROKE * _EP["ram"], 0.0], np.eye(3))
    _EP["tcps"] = _tcps(base, origin)
    if not _EP["crimped"] and _EP["ram"] >= 1.0:  # the die has closed fully: the crimp
        _EP["crimped"] = True
        _EP["back"], _EP["armed"] = True, False
        _EP["depth"], _EP["align"] = depth, align
        _EP["good"] = DEPTH[0] <= depth <= DEPTH[1] and align <= ALIGN_DEG and off < BAR_RI
        _crimp(base, origin, c_pos, c_rot, depth, off)
    elif _EP["ram"] >= 1.0:  # a later full closing: the ram goes back again
        _EP["back"], _EP["armed"] = True, False
    if _EP["on_cable"] is not None:  # the crimped lug goes with the cable
        p_rel, r_rel = _EP["on_cable"]
        _place(base, origin, "crimped_lug", c_pos + c_rot @ p_rel, c_rot @ r_rel)
    _EP["tip"] = c_pos
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr_ = _EP["tcps"]
            print(f"[crimp] t={_DEBUG['steps'] / 20:5.1f}s trig={_EP['trig'] * 1000:5.2f}mm on={int(_EP['on'])} "
                  f"ram={_EP['ram']:.2f} depth={depth * 1000:6.1f}mm align={align:5.1f} "
                  f"axis={np.round(c_rot[:, 0], 3).tolist()} off={off * 1000:5.2f}mm "
                  f"tipyz=({(c_pos[1] - MOUTH[1]) * 1000:.2f},{(c_pos[2] - MOUTH[2]) * 1000:.2f})mm "
                  f"crimped={_EP['crimped']} good={_EP['good']} "
                  f"at_crimp=({None if _EP['depth'] is None else round(_EP['depth'] * 1000, 1)}mm, "
                  f"{None if _EP['align'] is None else round(_EP['align'], 1)}deg) "
                  f"leftTCP-tip={np.round((tl - c_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-trig={np.round((tr_ - TRIG_C) * 1000).astype(int).tolist()} "
                  f"trigpos={np.round((tr[:3] - origin - TRIG_C) * 1000, 1).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or all(float(np.linalg.norm(t - TRIG_C)) >= LET_GO for t in tcps)
    tip = _EP["tip"]
    out = tip is not None and float(np.linalg.norm(tip - MOUTH)) >= OUT
    return {"crimped": _EP["crimped"] and _EP["on_cable"] is not None, "good": _EP["good"],
            "released": not _EP["on"] and _EP["ram"] <= 0.0 and let_go, "inserted": _EP["best_depth"] >= INSERTED,
            "out": out and _EP["on_cable"] is not None}


def _points(st) -> float:
    if st["good"] and st["released"] and st["out"]:
        return 1.0
    if st["crimped"]:
        return 0.5
    return 0.25 if st["inserted"] else 0.0


def score(objects) -> float:
    """1.0 for a good crimp (the bare end 10.5 to 13.5 mm into the 13 mm barrel and the cable within 8 degrees of its
    axis when the die closed) with the trigger let go and the die open again, the cable taken out of the crimper with
    the lug on it (its tip at least 5 cm from where the barrel's mouth was) and laid back on its rests, and both arms back
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
            and abs(float(axis[0])) >= math.cos(math.radians(ON_REST[2])))


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
    """Full score with the trigger and the cable still: the trial ends there."""
    return score(objects) >= 1.0 and objects["trigger"].resting() and objects["cable"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable 7.5 cm behind its tip, fingers closing across it, lifts it off its
# rests, lines it up with the barrel 3 cm in front of the guard and pushes the bare end straight in through the feed
# hole, aiming 2.5 mm past full depth so that the insulation meeting the barrel's mouth sets the depth, and holds it.
# The right hand comes down open round the crimper's grip, closes on the trigger and the grip's back (squeezing the
# trigger in, as a hand does) and holds it while the ram closes the die (the crimp); the ram goes back and the hand
# opens, rises and goes back to where it started. The left pulls the cable straight back out through the feed hole with
# the lug crimped on its end, holds it up to show it, lays it back on its rests and goes back to where it started.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closing along y (across the cable, which runs along x)
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y: on the trigger and the grip's back
HOLD_AT = 0.075  # the left TCP this far back from the cable's tip (between the rests)
IN_PAST = 0.0025  # aim this far past full depth: the insulation stops the cable at the barrel's mouth
SAG = 0.006  # the held cable hangs this far below the left TCP (the jaw pushes it toward the fingertips): aim above
AIM = np.array([0.0, -0.0007, -0.0040])  # trim of the left hand's aim at the barrel (y, z), from debug runs
NEAR = np.array([0.0, -0.0010, 0.0008])  # further trim once the tip is within 1 mm of the mouth (the arm drifts as
# it comes in), from debug runs
PROBE_IN, PROBE_R, PROBE_STEPS, PROBE_S = 0.004, (0.0002, 0.0015), 24, 0.15  # the search for the barrel's mouth
LEAD_S, LEAD_STEPS = 3.0, 15
SQ_AT = np.array([TRIG_C[0] + 0.004, YD - (TRIG_FRONT + GRIP_BACK) / 2, TOP + AXIS_Z + 0.006])  # the right TCP
# squeezing: 4 mm toward the trigger's far end, so the finger's upper part clears the tool's body


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


def _tcp_for_tip(tip_x: float) -> np.ndarray:
    """The left TCP holding the cable on the barrel's axis with its tip at ``tip_x``."""
    return np.array([tip_x - HOLD_AT, YD, TOP + AXIS_Z + SAG]) + AIM


def _left() -> tuple[list[Waypoint], float]:
    pick = np.array([TIP0[0] - HOLD_AT, TIP0[1], TOP + CABLE_Z + 0.0005])
    front = _tcp_for_tip(GUARD_X[0] - 0.03)
    pre = _tcp_for_tip(MOUTH[0] - 0.005)
    at_mouth = _tcp_for_tip(MOUTH[0] - 0.0005) + NEAR
    held = _tcp_for_tip(MOUTH[0] + BAR_L + IN_PAST) + NEAR
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], GRIP_L, "over the cable", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP_L),
            _w(0.8, pick, FINGER_CLOSED, "grip it", GRIP_L),
            _w(0.8, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it off the rests", GRIP_L),
            _w(1.5, front, FINGER_CLOSED, "line it up with the barrel, 3 cm in front of the guard", GRIP_L),
            _w(0.8, front, FINGER_CLOSED, "settle", GRIP_L),
            _w(1.2, pre, FINGER_CLOSED, "through the feed hole, the tip 5 mm from the barrel's mouth", GRIP_L),
            _w(0.6, pre, FINGER_CLOSED, "settle", GRIP_L),
            _w(1.0, at_mouth, FINGER_CLOSED, "the tip just short of the mouth", GRIP_L),
            _w(0.8, at_mouth, FINGER_CLOSED, "settle", GRIP_L)]
    # feel for the mouth: press lightly (aiming 4 mm past it) while the tip circles out to 1.5 mm off the axis; once the
    # tip finds the chamfer it slides in, and the barrel then holds it while the hand finishes the circles
    probe = _tcp_for_tip(MOUTH[0] + PROBE_IN) + NEAR
    for k in range(PROBE_STEPS):
        r = PROBE_R[0] + (PROBE_R[1] - PROBE_R[0]) * k / (PROBE_STEPS - 1)
        a = 2 * math.pi * k / 8
        out.append(_w(PROBE_S, probe + [0.0, r * math.cos(a), r * math.sin(a)], FINGER_CLOSED,
                      "feel for the barrel's mouth: small circles, pressing lightly", GRIP_L))
    out += [_w(0.5, probe, FINGER_CLOSED, "back on the axis", GRIP_L),
            _w(3.0, held, FINGER_CLOSED, "push the bare end straight in until the insulation meets the barrel", GRIP_L)]
    return out, sum(w.duration for w in out)


def _right(start: float) -> list[Waypoint]:
    above = SQ_AT + [0.0, 0.0, 0.045]
    out = _lead_in("right", above, GRIP_R, "over the crimper's grip", FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(start - t, 0.2), above, FINGER_OPEN, "wait for the cable to be held in the barrel", GRIP_R))
    out += [_w(1.0, SQ_AT, FINGER_OPEN, "down, open, round the trigger and the grip", GRIP_R),
            _w(0.8, SQ_AT, FINGER_CLOSED, "squeeze the trigger: the ram closes the die", GRIP_R),
            _w(T_CLOSE + 0.4, SQ_AT, FINGER_CLOSED, "hold it while the die closes: the crimp", GRIP_R),
            _w(0.6, SQ_AT, FINGER_OPEN, "let go: the ram goes back", GRIP_R),
            _w(1.0, above, FINGER_OPEN, "rise", GRIP_R)]
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
    right += _go_home("right", right[-1].pos, GRIP_R, FINGER_OPEN)
    held = np.asarray(left[-1].pos)
    pulled = held - [PULL_OUT, 0.0, 0.0]
    shown = pulled + [0.0, 0.0, SHOW_UP]
    rest = np.array([TIP0[0] - HOLD_AT, TIP0[1], TOP + CABLE_Z + SAG])  # the cable back on its rests
    left += [_w(max(t_r - t_held, 0.2) + T_RETURN, held, FINGER_CLOSED, "hold the cable until the die is open",
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
    right.append(_w(max(t_l - t_r, 0.5) + 3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
