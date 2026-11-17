"""On the station table stands an inert wrist stand: a dummy robot wrist on a pedestal, bolted to a board clamped to the
table, presenting a manual quick-change tool changer's robot side low over the table, its face toward the right arm,
as a parked robot's wrist would. The gripper tool has just been parked in the tool rack beside it; in the rack's other
slot waits a suction-cup tool. A locking ring sits round the flange, with a lever reaching out toward the arms.

Task: change the tool to the suction-cup tool. One hand takes it from the rack by its handle and holds its plate on the
flange's two dowels, their tips in its bushings' lead-in chamfers: the tool falls if let go before the ring has it. The
other presses the ring's lever down: past 8 degrees the claws' lips come over the plate's three lugs and draw it home
onto the flange, and from 25 degrees it is locked on. Then both hands let go, the tool hanging on the flange, and both
arms go back to where they started. The ring only takes a tool already seated on the
dowels, in a pose the full-size dowels and bushings allow (see _seat_check), while it is within 8 degrees of open (the
lugs pass between the lips only then); turned back below 25 degrees the tool is no longer locked, and back within 8
degrees of open it comes free.

Run it:  inspect-robots run --embodiment yam_arena -E task=lock_a_tool_onto_a_robots_quick_change_flange.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, glue, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the changer's robot side, on the stand's dummy wrist: the flange's axis runs along y; its face looks toward -y (the
# right arm). The sizes below are the changer's build (the README's "The changer as built" lists them, and the drawings
# and STL files are made from these constants); distances along the axis (y) are measured from the face, into the
# stand. Angles round the axis are seen facing the flange (from the tool side), counterclockwise from 12 o'clock, which
# is the way the ring turns to lock ----------------------------------------------------------------------------------
FX, FZ = 0.30, TOP + 0.085  # the flange's axis
FACE_Y = 0.02  # the flange's mating face
FACE_R, FACE_T = 0.037, 0.003  # the flange's front lip, whose front is the mating face (its back seats the ring's springs)
FLANGE_R = 0.032  # the journal behind the lip, on which the ring turns
RING_R, RING_Y = 0.044, (0.004, 0.022)  # the locking ring, just behind the face (its springs bridge y 0.003-0.004)
RETAINER_R, RETAINER_Y = 0.038, (0.022, 0.026)  # the retaining disc behind the ring; the wrist's end face is behind it
# -- the inert wrist stand: a printed dummy wrist 68 mm across (a small cobot's last link) on a printed pedestal, bolted
# to a plywood board that two C-clamps hold to the table. The changer bolts to the wrist's front face ----------------
WRIST_R, WRIST_Y = 0.034, (RETAINER_Y[1], 0.090)  # the dummy wrist (y from the flange's face)
BOARD = ((0.25, 0.45), (0.04, 0.22), 0.018)  # the board: x and y extents (world, m) and thickness
FOOT = (0.100, 0.100, 0.008)  # the pedestal's foot: across (x), back from the wrist's front face (y), thick
DOWEL_R, DOWEL_D, DOWEL_OUT = 0.022, 0.006, 0.006  # two locating dowels standing out of the face ...
DOWEL_AT = (0.0, 180.0)  # ... at 12 o'clock (round) and 6 o'clock (diamond)
SCREW_R, SCREW_CB, SCREW_AT = 0.025, 0.011, (45.0, 135.0, 225.0, 315.0)  # 4 x M6 into the wrist (ISO 9409-1-50-4-M6)
LUG_AT, LUG_SPAN = (0.0, 120.0, 240.0), 24.0  # the tool plate's three lugs (deg), on its rim ...
LUG_R, LUG_T = (0.030, 0.036), 0.003  # ... from the rim out to this, flush with the mating face, this thick
CLAW_AT, CLAW_SPAN = (60.0, 180.0, 300.0), 80.0  # the ring's three claws (with the ring open), each this long (deg)
CLAW_WALL_R, CLAW_LIP_R = (0.038, RING_R), (0.032, 0.038)  # a claw's wall, and the lip at its front reaching inward
CLAW_FRONT, LIP_BACK = -0.011, -0.0028  # the claws' front, and their lips' back (cam) face on its flat dwell ...
LIP_RAMP = (-0.009, -0.0036)  # ... which at the lip's leading end starts this far forward (the lead-in ramp), rising
# to the second over RAMP_IN - OPEN degrees, then to the dwell over LOCK - RAMP_IN (the clamp ramp)
LEVER_L = 0.11  # the lever: from the axis toward the arms (-x) ...
PAD_Y = 0.063  # ... then bent along +y to its pad, this far beyond the face
PAD = (0.045, 0.030, 0.008)
PAD_TOP = 0.006 + PAD[2]  # the pad's top above the lever's line
# the printed ring, claw ring, lever and pad with their springs, inserts and screws: mass and centre of mass (ring frame;
# computed from the STL files and the bought parts, README "Masses")
RING_MASS, RING_COM = 0.1404, (-0.0301, 0.0177, 0.0012)

# -- the ring turns on a hinge about the axis; pressing the lever down turns it. It stays where it is left (friction
# from its six springs' preload holds it): a spring holds it where it last stopped, and pushed SLIP past that it gives
# (the task file moves the hold to it) --------------------------------------------------------------------------------
RING_HOLD = 0.06  # N m per degree
SLIP = 2.0  # degrees
RING_DRAG = 0.002  # N m per degree/s
RING_MAX = 35.0  # the stop; the open stop is OPEN_STOP the other way
OPEN_STOP = 1.0
OPEN = (360.0 / len(CLAW_AT) - CLAW_SPAN - LUG_SPAN) / 2  # 8 deg: the lugs pass between the lips only this near open
RAMP_IN = 20.0  # turned this far, the lips' lead-in ramps have drawn a plate (on the flange) home ...
LOCK = 25.0  # ... and by this far their clamp ramps have pulled it tight: locked, the lugs on the lips' flat dwell

# -- the tools: each lies along y, its plate's face (the body's origin) toward +y, a square handle on its top -----------
PLATE_R, PLATE_T = 0.030, 0.008
BOSS_R, BOSS_T = 0.007, 0.004  # a boss on the plate's back round each bushing (the bushings are 10 mm long)
HANDLE = (0.016, 0.016, 0.035)
HANDLE_Y, HANDLE_Z = -0.05, 0.020  # the handle's middle along the tool, and its bottom above the axis
TOOL_R = 0.022  # the tools' bodies reach this far from the axis
TOOL_MASS = 0.12  # the suction tool; the gripper tool is 20 g heavier
# each tool's centre of mass (body frame), computed from its printed parts, bushings and steel ballast bar (README
# "Masses"): toward the plate from the handle, and below the axis, so it lies handle up
SUCTION_COM, GRIPPER_COM = (0.0, -0.0451, -0.0044), (0.0, -0.0429, -0.0064)
SUCTION_AT = np.array([0.30, -0.12, FZ])  # the suction tool's plate face, in the rack
GRIPPER_AT = np.array([0.18, -0.12, FZ])  # the parked gripper tool's
LOCKS = {"suction_tool": "suction_lock", "gripper_tool": "gripper_lock"}
SEAT = np.array([FX, FACE_Y, FZ])  # where a held tool's plate face is held: on the flange's face, centred and square
ANCHOR_AT = np.array([0.45, -0.30, TOP - 0.05])  # a hidden fixed body the lock holds a tool to

ORANGE, GREY, STEEL, DARK, BLACK, BLUE, WOOD, WHITE = (0.93, 0.50, 0.10), (0.45, 0.46, 0.48), \
    (0.72, 0.73, 0.75), (0.20, 0.21, 0.23), (0.06, 0.06, 0.07), (0.15, 0.35, 0.78), (0.62, 0.45, 0.28), \
    (0.95, 0.95, 0.93)

# -- scoring ----------------------------------------------------------------------------------------------------------
# Seated = on the dowels, in a pose the full-size dowels and bushings physically allow (the README's "What the dowels
# capture"). Each tool plate takes the dowels in two hardened bushings (DIN 179, 6 mm F7 bores: 6.010 to 6.022 mm),
# 22 mm out at 12 and 6 o'clock, behind a 1.5 x 45 degree lead-in chamfer. The round dowel (ISO 2338, 6 mm h8: 5.978 to
# 6.000 mm) runs in its bushing with 0.010 to 0.044 mm clearance, the diamond pin (DIN 6321 form C, 6 mm g6: 5.988 to
# 5.996 mm) with 0.014 to 0.034 mm on its lands. For each dowel, d is how far its tip (its deepest edge) has gone past
# the plate's face, e how far its axis is off the bushing's where it crosses the face, and t its tilt against the
# bushing's axis. The dowel's widest point, at depth d, reaches e + d tan t + 3 mm / cos t from the bushing's axis; the
# hole there is 3 mm + BORE_PLAY + (1.5 mm - d) wide in the chamfer, 3 mm + BORE_PLAY in the bore. So a dowel is clear
# of its bushing while e + d tan t + 3 mm (1/cos t - 1) <= BORE_PLAY + max(0, 1.5 mm - d): its axis may be up to
# 1.5 - d (+ 0.005) mm off in the chamfer, and only 0.005 mm off, with t <= about atan(0.005 / d), in the bore. Both
# dowels are checked in every direction as full 6.000 mm pins (the diamond pin's relief only adds room), at the
# tightest fit, the dowels' own end chamfers are left out, and the printed chamfer is made 0.2 mm larger than this one:
# every accepted pose fits every build.
FIT = (0.010e-3, 0.044e-3)  # the round dowel's running clearance in its bushing (on the diameter)
BORE_PLAY = FIT[0] / 2  # 0.005 mm: how far off its bore's axis a dowel can sit, at the tightest fit
BUSH_LEAD = 0.0015  # the bushings' lead-in chamfer: 1.5 mm wide (radially) and 1.5 mm deep
LUG_CATCH = -LIP_RAMP[0] + 0.001  # 10.0 mm: the lips' lead-in ramps (with their 1 mm chamfer) reach a lug front this
# far in front of the face. Seated also needs every lug front within it, the plate's face nowhere past the flange's,
# and each lug inside its gap between the lips (OPEN, 8 degrees either side, counting the plate's turn against the
# ring's and the sideways shift of a lug 30 mm out)
STOP_OUT = 0.0056  # the simulation's stand-in for the tips in the chamfers: the drawn dowels do not touch and the
# plate has no bores, so two collision-only stops, 6 mm across, 5.6 mm out of the face where the dowels are, stop a
# pushed plate with the dowels' tips 0.4 mm into its bushings' chamfers (hidden while the ring holds a tool)
STOPS_AT = np.array([FX, FACE_Y, FZ])
STOPS_AWAY = np.array([0.45, -0.20, TOP - 0.05])
LET_GO = 0.04  # each TCP at least this far from the tool's handle and the lever's pad
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _round(phi: float, r: float) -> tuple[float, float]:
    """(x, z) of a point ``r`` from the axis at ``phi`` degrees (seen facing the flange, counterclockwise from 12)."""
    a = math.radians(phi)
    return -r * math.sin(a), r * math.cos(a)


def _on_axis(phi: float, r: float, y: float) -> tuple[float, float, float]:
    """The point ``r`` from the axis at ``phi`` degrees and ``y`` along it (body frame)."""
    x, z = _round(phi, r)
    return x, y, z


def _arc_box(phi: float, r: tuple[float, float], y: tuple[float, float], span: float, color, at_r: float) -> BoxShape:
    """A visual block round the axis: centred at ``phi`` (deg), from r[0] to r[1] out and y[0] to y[1] along the axis,
    ``span`` degrees wide at radius ``at_r``."""
    x, z = _round(phi, (r[0] + r[1]) / 2)
    a = math.radians(phi)
    return BoxShape((x, (y[0] + y[1]) / 2, z), (2 * at_r * math.sin(math.radians(span) / 2), y[1] - y[0], r[1] - r[0]),
                    color, collide=False, quat_xyzw=(0.0, -math.sin(a / 2), 0.0, math.cos(a / 2)))


def _stand() -> Body:
    """The inert wrist stand with the changer's robot side, fixed; origin on the flange's axis at its face."""
    o = np.array([FX, FACE_Y, FZ])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    (bx0, bx1), (by0, by1), bt = BOARD
    floor = TOP + bt + FOOT[2]  # the pedestal stands on its foot, on the board
    wy0, wy1 = FACE_Y + WRIST_Y[0], FACE_Y + WRIST_Y[1]
    shapes = [BoxShape(at((bx0 + bx1) / 2, (by0 + by1) / 2, TOP + bt / 2), (bx1 - bx0, by1 - by0, bt), WOOD),
              BoxShape(at(FX, wy0 + FOOT[1] / 2, TOP + bt + FOOT[2] / 2), FOOT, GREY),  # the pedestal's foot
              BoxShape(at(FX, (wy0 + wy1) / 2, (floor + FZ) / 2), (2 * WRIST_R, wy1 - wy0, FZ - floor), GREY),
              CylShape(at(FX, (wy0 + wy1) / 2, FZ), WRIST_R, wy1 - wy0, GREY, axis="Y"),  # the dummy wrist
              *[shape for yc in (by0 + 0.03, by1 - 0.03) for shape in (  # two C-clamps on the board's far edge
                  BoxShape(at(bx1 - 0.015, yc, TOP + bt + 0.005), (0.03, 0.016, 0.01), DARK, collide=False),
                  BoxShape(at(bx1 + 0.006, yc, TOP + bt / 2), (0.012, 0.016, bt + 0.02), DARK, collide=False))],
              # the flange (its face at y = 0 here): the front lip, the journal the ring turns on, the retaining disc
              CylShape((0.0, FACE_T / 2, 0.0), FACE_R, FACE_T, STEEL, axis="Y"),
              CylShape((0.0, (FACE_T + RETAINER_Y[0]) / 2, 0.0), FLANGE_R, RETAINER_Y[0] - FACE_T, STEEL, axis="Y"),
              CylShape((0.0, sum(RETAINER_Y) / 2, 0.0), RETAINER_R, RETAINER_Y[1] - RETAINER_Y[0], GREY,
                       collide=False, axis="Y"),
              *[_arc_box(phi, (RETAINER_R - 0.0004, RETAINER_R + 0.0004), RETAINER_Y, 1.0, WHITE, RETAINER_R)
                for phi in (0.0, OPEN, LOCK)],  # marks on the disc: open, the end of the open window, locked
              *[CylShape(_on_axis(phi, DOWEL_R, -DOWEL_OUT / 2), DOWEL_D / 2, DOWEL_OUT, STEEL, collide=False,
                         axis="Y") for phi in DOWEL_AT],  # the dowels
              *[CylShape(_on_axis(phi, SCREW_R, -0.0003), SCREW_CB / 2, 0.0006, DARK, collide=False, axis="Y")
                for phi in SCREW_AT]]  # the mounting screws' counterbores
    return Body.build("stand", pos=tuple(o.tolist()), mass=5.0, shapes=shapes, kinematic=True, color=GREY,
                      friction=(0.6, 0.5))


def _anchor() -> Body:
    return Body.build("lock_anchor", pos=tuple(ANCHOR_AT.tolist()), mass=0.01, kinematic=True, color=DARK,
                      shapes=[SphereShape((0.0, 0.0, 0.0), 0.003, DARK, collide=False)])


def _ring() -> Body:
    """The locking ring with its three claws and its lever; origin on the flange's axis at its face."""
    yc = (RING_Y[0] + RING_Y[1]) / 2
    claws = []
    for c in CLAW_AT:  # each claw: its wall in four blocks, and the lip at its front reaching inward
        lead, ramp = c + CLAW_SPAN / 2, RAMP_IN - OPEN  # the lip's leading end, and its lead-in ramp (drawn as 2 steps)
        rest = (CLAW_SPAN - ramp) / 4
        for k in range(4):
            claws.append(_arc_box(c - CLAW_SPAN / 2 + CLAW_SPAN / 8 * (2 * k + 1), CLAW_WALL_R, (CLAW_FRONT, RING_Y[0]),
                                  CLAW_SPAN / 4 + 0.5, DARK, CLAW_WALL_R[1]))
            claws.append(_arc_box(c - CLAW_SPAN / 2 + rest * (k + 0.5), CLAW_LIP_R, (CLAW_FRONT, LIP_BACK), rest + 0.5,
                                  DARK, CLAW_LIP_R[1]))  # the dwell (and the 0.8 mm clamp ramp)
        for f in (0.25, 0.75):
            back = LIP_RAMP[1] + (LIP_RAMP[0] - LIP_RAMP[1]) * f
            claws.append(_arc_box(lead - ramp * (1 - f), CLAW_LIP_R, (CLAW_FRONT, back), ramp / 2 + 0.3, DARK,
                                  CLAW_LIP_R[1]))
    shapes = [CylShape((0.0, yc, 0.0), RING_R, RING_Y[1] - RING_Y[0], DARK, axis="Y"),
              _arc_box(0.0, (RING_R - 0.0004, RING_R + 0.0004), RING_Y, 2.6, WHITE, RING_R),  # its index line
              *claws,
              BoxShape((-(RING_R - 0.004 + LEVER_L) / 2, yc, 0.0), (LEVER_L - RING_R + 0.004, 0.012, 0.012), ORANGE),
              BoxShape((-LEVER_L, (yc + PAD_Y) / 2, 0.0), (0.012, PAD_Y - yc + 0.012, 0.012), ORANGE),
              BoxShape((-LEVER_L, PAD_Y, 0.006 + PAD[2] / 2), PAD, BLACK),
              BoxShape((-LEVER_L - PAD[0] / 2 + 0.003, PAD_Y, PAD_TOP + 0.003), (0.006, PAD[1], 0.006), BLACK)]  # a lip
    return Body.build("ring", pos=(FX, FACE_Y, FZ), mass=RING_MASS, shapes=shapes, friction=(1.0, 0.9), color=DARK,
                      com=RING_COM)


def _rack() -> Body:
    """Two slots, each a block under a tool's body with a lip either side; origin on the table."""
    shapes = []
    h = FZ - TOOL_R - TOP
    for at in (SUCTION_AT, GRIPPER_AT):
        x, yc = float(at[0]), float(at[1]) - 0.05
        shapes += [BoxShape((x, yc, h / 2), (0.062, 0.04, h), DARK),
                   *[BoxShape((x + s * 0.028, yc, h + 0.005), (0.006, 0.04, 0.010), DARK) for s in (-1.0, 1.0)]]
    shapes.append(BoxShape(((SUCTION_AT[0] + GRIPPER_AT[0]) / 2, float(SUCTION_AT[1]) - 0.05, 0.004),
                           (float(SUCTION_AT[0] - GRIPPER_AT[0]) + 0.08, 0.06, 0.008), GREY))
    return Body.build("rack", pos=(0.0, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, color=DARK,
                      friction=(0.6, 0.5))


def _plate_and_handle(color) -> list:
    """The tool plate with its three lugs, its two dowel bushings (marked on its face) and their bosses, and the
    handle. The plate is printed in one piece with the tool's body, in its colour."""
    return [CylShape((0.0, -PLATE_T / 2, 0.0), PLATE_R, PLATE_T, color, axis="Y"),
            *[_arc_box(phi, LUG_R, (-LUG_T, 0.0), LUG_SPAN, color, sum(LUG_R) / 2) for phi in LUG_AT],
            *[CylShape(_on_axis(phi, DOWEL_R, 0.0003), DOWEL_D / 2, 0.0006, DARK, collide=False, axis="Y")
              for phi in DOWEL_AT],
            *[CylShape(_on_axis(phi, DOWEL_R, -PLATE_T - BOSS_T / 2), BOSS_R, BOSS_T, color, collide=False, axis="Y")
              for phi in DOWEL_AT],
            BoxShape((0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] / 2), HANDLE, DARK)]


def _suction_tool() -> Body:
    shapes = [*_plate_and_handle(BLUE),
              CylShape((0.0, -0.0465, 0.0), TOOL_R, 0.077, BLUE, axis="Y"),  # the body
              CylShape((0.0, -0.090, 0.0), 0.008, 0.010, DARK, axis="Y"),  # the stem
              CylShape((0.0, -0.100, 0.0), 0.020, 0.010, BLACK, axis="Y")]  # the suction cup
    return Body.build("suction_tool", pos=tuple(SUCTION_AT.tolist()), mass=TOOL_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLUE, com=SUCTION_COM, ignore=("ring",))


def _gripper_tool() -> Body:
    shapes = [*_plate_and_handle(GREY),
              BoxShape((0.0, -0.040, 0.0), (0.044, 0.064, 0.044), GREY),  # the body
              *[BoxShape((s * 0.014, -0.088, 0.0), (0.008, 0.034, 0.020), DARK) for s in (-1.0, 1.0)]]  # its fingers
    return Body.build("gripper_tool", pos=tuple(GRIPPER_AT.tolist()), mass=TOOL_MASS + 0.02, shapes=shapes,
                      friction=(1.0, 0.9), color=GREY, com=GRIPPER_COM, ignore=("ring",))


def _stops() -> Body:
    """The stand-in for the dowels' tips in the bushings' chamfers: two 6 mm discs' worth of collision, inside the drawn
    dowels and 0.4 mm shorter; origin on the flange's axis at its face."""
    return Body.build("dowel_stops", pos=tuple(STOPS_AT.tolist()), mass=0.01, kinematic=True, color=STEEL,
                      shapes=[CylShape(_on_axis(phi, DOWEL_R, -STOP_OUT / 2), DOWEL_D / 2, STOP_OUT, STEEL, axis="Y")
                              for phi in DOWEL_AT], ignore=("ring",))


OBJECTS = [_stand(), _anchor(), _ring(), _stops(), _rack(), _suction_tool(), _gripper_tool()]
# the ring turns about the flange's axis; pressing the lever down turns it the negative way about +y
hinge("lock_ring", "stand", "ring", anchor=(FX, FACE_Y + (RING_Y[0] + RING_Y[1]) / 2, FZ), axis="Y",
      limits_deg=(-RING_MAX, OPEN_STOP), spring=RING_HOLD, damping=RING_DRAG, target_deg=0.0)
# the locks: off until the ring takes a seated tool (then the tool is held home on the flange, centred and square)
for _tool, _lock in LOCKS.items():
    glue(_lock, "lock_anchor", _tool, anchor=tuple(ANCHOR_AT.tolist()))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"held": 0.0, "tcps": None, "s": {}, "stops_away": False,
            "tools": {name: {"inserted": False, "caught": False, "ever": False, "ever_locked": False}
                      for name in LOCKS}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FLANGE_DEBUG")), "steps": 0}


def _quat_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


def _lock_frames() -> None:
    """Each lock holds its tool with its plate's face on the flange, centred and square (the ring draws it home)."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    for lock in LOCKS.values():
        joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[lock]))
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in SEAT - ANCHOR_AT]))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(np.eye(3))))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _lock_frames()
    for lock in LOCKS.values():
        set_glue_on(lock, False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _place(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _seat_error(pos, rot) -> tuple[float, float, float, float]:
    """How far a tool is from sitting on the flange: the gap from its plate's face to the flange's face (m), its
    middle off the axis (m), its axis tilted off the flange's (deg), and its turn about its axis (deg)."""
    gap = FACE_Y - float(pos[1])
    off = math.hypot(float(pos[0]) - FX, float(pos[2]) - FZ)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[1, 1])))))
    roll = math.degrees(math.atan2(float(rot[0, 2]), float(rot[2, 2])))
    return gap, off, tilt, roll


def _dowel_fit(pos, rot) -> list[dict]:
    """Each dowel against its bushing (12 o'clock, then 6): ``depth`` d, how far the dowel's deepest tip edge is past
    the plate's face; ``off`` e, how far its axis is off the bushing's where it crosses the face; ``tilt`` t (rad);
    ``need`` = e + d tan t + r (1/cos t - 1), how far its widest point at depth d reaches beyond its own 3 mm radius;
    ``room`` = BORE_PLAY + max(0, BUSH_LEAD - d), how much wider than 3 mm the hole is there; ``clear``: need <= room
    (lengths in m)."""
    pos = np.asarray(pos, float)
    out = []
    for phi in DOWEL_AT:
        bx, bz = _round(phi, DOWEL_R)
        root = rot.T @ (np.array([FX + bx, FACE_Y, FZ + bz]) - pos)  # where the dowel leaves the flange, plate frame
        v = rot.T @ np.array([0.0, -1.0, 0.0])  # along the dowel, out toward the plate (the bushing runs along -y)
        tilt = math.acos(max(-1.0, min(1.0, -float(v[1]))))
        depth = -float((root + DOWEL_OUT * v)[1]) + DOWEL_D / 2 * math.sin(tilt)
        hit = root + v * (-float(root[1]) / float(v[1]))  # where the dowel's axis crosses the plate's face
        off = math.hypot(float(hit[0]) - bx, float(hit[2]) - bz)
        need = off + depth * math.tan(tilt) + DOWEL_D / 2 * (1.0 / math.cos(tilt) - 1.0)
        room = BORE_PLAY + max(0.0, BUSH_LEAD - depth)
        out.append({"depth": depth, "off": off, "tilt": tilt, "need": need, "room": room, "clear": need <= room})
    return out


def _seat_check(pos, rot, turned: float) -> dict:
    """Whether a tool is seated on the dowels with the ring ``turned`` degrees: both dowels in their bushings (each tip
    past the plate's face and clear of the bushing, as :func:`_dowel_fit`), the plate's face nowhere past the flange's,
    every lug front within the lips' reach (LUG_CATCH) and every lug inside its gap between the lips."""
    gap, off, tilt, roll = _seat_error(pos, rot)
    dowels = _dowel_fit(pos, rot)
    reach = LUG_R[1] * math.sin(math.radians(tilt))  # how far the plate's edge (its lugs, 36 mm out) leans either way
    lug_turn = abs(-roll - turned) + math.degrees(math.asin(min(1.0, off / LUG_R[0])))  # roll is about +y: -roll is
    # the plate's turn as the ring's is counted (counterclockwise seen facing the flange)
    checks = {"dowels": all(d["depth"] > 0.0 and d["clear"] for d in dowels), "face": gap - reach >= 0.0,
              "catch": gap + LUG_T + reach <= LUG_CATCH, "lugs": lug_turn <= OPEN}
    return {"gap": gap, "tilt": tilt, "fits": dowels, "lug_turn": lug_turn, **checks, "seated": all(checks.values())}


def _seated(pos, rot, turned: float) -> bool:
    return _seat_check(pos, rot, turned)["seated"]


def _pad_top(turned: float) -> np.ndarray:
    """The middle of the lever pad's top with the ring turned ``turned`` degrees (lever pressed down)."""
    a = math.radians(turned)
    x, z = -LEVER_L, PAD_TOP
    return np.array([FX + x * math.cos(a) - z * math.sin(a), FACE_Y + PAD_Y, FZ + x * math.sin(a) + z * math.cos(a)])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    rr = rotation(as_numpy(base.scene["ring"].data.root_link_pose_w)[0][3:7])
    turned = -math.degrees(math.atan2(float(rr[0, 2]), float(rr[0, 0])))  # the lever pressed down: positive
    if abs(turned - _EP["held"]) > SLIP:  # pushed past its hold: the ring gives, and stays where it is left
        _EP["held"] = turned
        set_hinge_drive("lock_ring", target_deg=-turned)
    poses, checks = {}, {}
    for name, lock in LOCKS.items():
        t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        pos, rot = t[:3] - origin, rotation(t[3:7])
        poses[name] = (pos, rot)
        st = _EP["tools"][name]
        checks[name] = _seat_check(pos, rot, turned)
        seated = checks[name]["seated"]
        if st["caught"]:
            if turned < OPEN:  # turned back until the lugs line up with the gaps between the lips: the tool is free
                set_glue_on(lock, False)
                st["caught"] = st["inserted"] = False
        elif turned < OPEN:  # the ring open: a tool seated on the dowels can be taken. Its lugs must pass between the
            # lips to go on (seated); once on, the ring turning toward OPEN brings the lips' leading ends over them,
            # which is how it takes them, so staying on needs only the dowels, the face and the lips' reach
            c = checks[name]
            st["inserted"] = c["dowels"] and c["face"] and c["catch"] and (st["inserted"] or seated)
        else:  # the lips have reached the lugs: a tool seated a step ago and still on the dowels is held, drawn home
            c = checks[name]  # onto the face along the dowels and square (a real ring's ramps do it by RAMP_IN)
            if st["inserted"] and c["dowels"] and c["face"] and c["catch"]:
                set_glue_on(lock, True)
                st["caught"] = True
            st["inserted"] = False  # (the lips now cover the gaps: no other tool can go on)
        st["ever_locked"] = st["ever_locked"] or (st["caught"] and turned >= LOCK)
        st["ever"] = st["ever"] or st["inserted"]
    # the stops stand in for the dowel tips only until the ring holds a tool (it then sits on the dowels proper); they
    # come back once no tool is held and none is where they would be
    held = any(st["caught"] for st in _EP["tools"].values())
    if held and not _EP["stops_away"]:
        _place(base, origin, "dowel_stops", STOPS_AWAY)
        _EP["stops_away"] = True
    elif not held and _EP["stops_away"] and all(
            c["gap"] - LUG_R[1] * math.sin(math.radians(c["tilt"])) > STOP_OUT + 0.002
            or math.hypot(float(p[0]) - FX, float(p[2]) - FZ) > PLATE_R + DOWEL_R + DOWEL_D
            for (p, _), c in zip(poses.values(), checks.values())):
        _place(base, origin, "dowel_stops", STOPS_AT)
        _EP["stops_away"] = False
    _EP["tcps"] = _tcps(base, origin)
    pos, rot = poses["suction_tool"]
    handle = pos + rot @ np.array([0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] / 2])
    pad = _pad_top(turned)
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (handle, pad))
    _EP["s"] = {"turned": turned, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            gap, off, tilt, roll = _seat_error(pos, rot)
            c = checks["suction_tool"]
            dw = " ".join(f"d={d['depth'] * 1000:5.2f} e={d['off'] * 1000:5.3f} need={d['need'] * 1000:5.3f} "
                          f"room={d['room'] * 1000:5.3f}" for d in c["fits"])
            yaw = math.degrees(math.atan2(float(rot[0, 1]), float(rot[1, 1])))
            pitch = math.degrees(math.atan2(float(rot[2, 1]), float(rot[1, 1])))
            st = _EP["tools"]["suction_tool"]
            print(f"[flange] t={_DEBUG['steps'] / 20:5.1f}s ring={turned:5.1f}deg suction: gap={gap * 1000:6.2f}mm "
                  f"off={off * 1000:5.2f}mm (x {(pos[0] - FX) * 1000:5.2f} z {(pos[2] - FZ) * 1000:5.2f}) "
                  f"dowels[{dw}] tilt={tilt:5.2f} (yaw {yaw:5.2f} pitch {pitch:5.2f}) roll={roll:6.2f} "
                  f"lug_turn={c['lug_turn']:4.1f} checks={[k for k in ('dowels', 'face', 'catch', 'lugs') if c[k]]} "
                  f"seated={c['seated']} inserted={st['inserted']} caught={st['caught']} "
                  f"stops_away={_EP['stops_away']} gripper_tool_caught={_EP['tools']['gripper_tool']['caught']} "
                  f"clear={clear} "
                  f"rightTCP-handle={np.round((tr - handle) * 1000).astype(int).tolist()} "
                  f"leftTCP-pad={np.round((tl - pad) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tools = _EP["tools"]
    tight = float(_EP["s"].get("turned", 0.0)) >= LOCK  # the ring on the lips' flat dwell
    return {"locked": tools["suction_tool"]["caught"] and tight, "wrong": tools["gripper_tool"]["ever_locked"],
            "ever": tools["suction_tool"]["ever"], "clear": bool(_EP["s"].get("clear"))}


def _points(s) -> float:
    if s["wrong"]:
        return 0.0
    if s["locked"] and s["clear"]:
        return 1.0
    if s["locked"]:
        return 0.5
    return 0.25 if s["ever"] else 0.0


def score(objects) -> float:
    """0.0 if the gripper tool was ever locked on. Else 1.0 with the suction tool locked on the flange (the ring still
    turned 25 degrees or more), both hands let go (each TCP at least 4 cm from its handle and the lever's pad) and both
    arms back where they started; 0.5 with it locked on otherwise; 0.25 if it was ever seated on the dowels with the ring
    open; else 0.0."""
    pts = _points(_status())
    if pts >= 1.0 and not _arms_home():
        return 0.5  # locked on and let go, but the arms not back
    return pts


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
    """Full score with the suction tool still (hanging on the flange): the trial ends there."""
    return score(objects) >= 1.0 and objects["suction_tool"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the suction tool's handle, fingers closing across it, lifts it out of the rack,
# carries it to the flange and holds its plate on the dowels' tips. The left hand comes straight down on the lever's
# pad, fingers closed, and presses it down along its swing, 32 degrees: past 8 degrees the ring takes the tool and draws
# it home, and from 25 it is locked on. The right lets go and draws back, the left rises, the tool hanging on the
# flange, and both arms go back to where they started.

TOOL_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x, across the handle
HOLD_TWIST = -3.4  # the pads twist the held tool 3.4 deg about the vertical (measured): carried and held with the
# wrist turned back this much about the vertical, so its plate meets the flange square
_c, _s = math.cos(math.radians(HOLD_TWIST)), math.sin(math.radians(HOLD_TWIST))
HOLD_PITCH = 0.85  # pushed onto the dowels' tips, the held plate's face pitches face-down (its weight, 4.9 mm toward
# the plate from the handle, and the push at the handle, above the axis; measured): held with the wrist pitched back
# this much about x, so both tips reach their chamfers
_p = math.radians(HOLD_PITCH)
HOLD_ROLL = -0.65  # the pads also roll the held tool 0.65 deg about its axis (measured): held rolled back this much
# about y, which evens the two dowels' offsets and moves the plate 0.5 mm toward +x, against its drift
_r = math.radians(HOLD_ROLL)
HOLD_GRIP = (np.array([[math.cos(_r), 0.0, math.sin(_r)], [0.0, 1.0, 0.0], [-math.sin(_r), 0.0, math.cos(_r)]])
             @ np.array([[1.0, 0.0, 0.0], [0.0, math.cos(_p), -math.sin(_p)], [0.0, math.sin(_p), math.cos(_p)]])
             @ np.array([[_c, -_s, 0.0], [_s, _c, 0.0], [0.0, 0.0, 1.0]]) @ np.asarray(TOOL_GRIP, float))
PAD_GRIP = grip_rotation(0.0)  # straight down, fingers closed (closing along y)
IN_HANDLE = 0.013  # the right TCP this far below the handle's top
LIFT = 0.018  # out of the rack (its lips are 1 cm high)
SHORT = 0.02  # the plate brought level with the flange this far short of it, then on
PUSH_IN = 0.0025  # aimed this far past the dowels' tips (the arm stops 1-2 mm short of a pushed target)
AIM = np.array([0.0, 0.0, 0.0046])  # held so, the plate sits 4.6 mm low (measured): aim against it
TIPS = np.array([FX, FACE_Y - STOP_OUT, FZ])  # the plate's face on the dowels' tips (on the stops)
TIP = 0.013  # closed fingertips below the TCP
PRESS = 0.005  # aimed this far into the pad
TURN_TO = 32.0
FIRST_PRESS = 12.0  # the first press, made as the plate reaches the dowels' tips, takes the ring past 8 degrees
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _in_hand(plate_face) -> np.ndarray:
    """The right TCP holding a tool by its handle, the tool's plate face at ``plate_face``."""
    return np.asarray(plate_face, float) + [0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] - IN_HANDLE]


def _right(press_done: float) -> tuple[list[Waypoint], float]:
    pick = _in_hand(SUCTION_AT)
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], TOOL_GRIP, "over the suction tool's handle", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", TOOL_GRIP),
            _w(0.6, pick, FINGER_CLOSED, "take it", TOOL_GRIP),
            _w(1.0, pick + [0.0, 0.0, LIFT], FINGER_CLOSED, "lift it out of the rack", TOOL_GRIP)]
    on = _in_hand(TIPS + AIM + [0.0, PUSH_IN, 0.0])
    out += [_w(1.6, on + [0.0, -SHORT, LIFT], FINGER_CLOSED, "carry it to the flange, its plate facing it", HOLD_GRIP),
            _w(0.8, on + [0.0, -SHORT, 0.0], FINGER_CLOSED, "level with the flange", HOLD_GRIP),
            _w(0.8, on, FINGER_CLOSED, "its plate on the dowels' tips", HOLD_GRIP)]
    seated_at = sum(w.duration for w in out)
    back = on + [0.0, -0.03, 0.04]
    out += [_w(max(press_done + 0.8 - seated_at, 0.3), on, FINGER_CLOSED, "hold it on while the lever is pressed",
               HOLD_GRIP),
            _w(0.5, on, FINGER_OPEN, "let go: it hangs on the flange, locked", HOLD_GRIP),
            _w(1.0, back, FINGER_OPEN, "draw back", HOLD_GRIP)]
    out += _go_home("right", back, HOLD_GRIP, FINGER_OPEN)
    return out, seated_at


def _on_pad(turned: float, press: float) -> np.ndarray:
    return _pad_top(turned) + [0.0, 0.0, TIP - press]


def _left(seated_at: float, total: float) -> tuple[list[Waypoint], float]:
    over = _on_pad(0.0, -0.04)
    out = _lead_in("left", over, PAD_GRIP, "over the lever's pad", FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out += [_w(max(seated_at - 0.5 - t, 0.3), over, FINGER_CLOSED, "wait over the pad", PAD_GRIP),
            _w(0.8, _on_pad(0.0, 0.0), FINGER_CLOSED, "down onto the pad as the tool reaches the dowels' tips",
               PAD_GRIP),
            _w(0.6, _on_pad(FIRST_PRESS, PRESS), FINGER_CLOSED,
               "press the lever down at once: past 8 degrees the ring takes the tool", PAD_GRIP)]
    for k in range(1, 6):
        out.append(_w(0.5, _on_pad(FIRST_PRESS + (TURN_TO - FIRST_PRESS) * k / 5, PRESS), FINGER_CLOSED,
                      "press on: the ring draws the tool home and locks it on", PAD_GRIP))
    press_done = sum(w.duration for w in out)
    up = _on_pad(TURN_TO, -0.05)
    out += [_w(0.8, _on_pad(TURN_TO, PRESS), FINGER_CLOSED, "hold it there", PAD_GRIP),
            _w(1.0, up, FINGER_CLOSED, "rise", PAD_GRIP)]
    out += _go_home("left", up, PAD_GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, press_done


def demo():
    _, seated_at = _right(0.0)
    _, press_done = _left(seated_at, 0.0)
    right, _ = _right(press_done)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(seated_at, total)
    return {"left": left, "right": right}
