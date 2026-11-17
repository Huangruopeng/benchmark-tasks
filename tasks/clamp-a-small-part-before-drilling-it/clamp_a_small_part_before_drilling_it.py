"""An 8-inch bench drill press stands on the station table. On its table lies a waxed MDF spoil board, and on that a
loose aluminium plate with its punch mark 20 mm in front of the drill's bit. A screw hold-down clamp (a star-knob screw
with a rubber tip that turns with it, through an arm on a riser) stands over its front-left part.
The drill's motor has its own push-button switch (green start, red stop) on the front of the head; the quill is fed by
a three-spoke capstan handle on the right of the head, through a pinion on the quill's rack, against a return spring.

Task: drill through the plate at its mark and leave the drill switched off with its quill up. It must not move while
it is drilled and no part of either gripper may come within 5 cm of the bit or the chuck while they turn. The plate has
to be slid until its mark is under the bit. The clamp's tip turns with its screw and drags a loose plate round, off its
mark, unless the plate is held while the clamp is tightened to 85 N; an unclamped plate is caught by the bit and flung.

Run it:  inspect-robots run --embodiment yam_arena -E task=clamp_a_small_part_before_drilling_it.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_hinge_drive, set_slide_target, set_slider_drive, set_thread_on, slider,
                     thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the drill press: an 8-inch (203 mm swing) bench drill press, 1/3 hp, 13 mm keyed chuck, 50 mm quill travel ---------
# Heights are above the bench (the station table's top). x forward from the arm bases, y to the left.
SPX, SPY = 0.32, 0.0  # the spindle's axis
BASE_X, BASE_Y, BASE_H = (SPX - 0.165, SPX + 0.125), (-0.095, 0.095), 0.040  # cast-iron base, 290 x 190 x 40 mm
COL_X, COL_R, COL_TOP = SPX + 0.105, 0.023, 0.470  # 46 mm steel column, 105 mm behind the spindle
TABLE_X, TABLE_Y, TABLE_Z = (SPX - 0.08, SPX + 0.08), (-0.08, 0.08), (0.145, 0.160)  # 160 x 160 mm table, top 160 mm
HEAD_X, HEAD_Y, HEAD_Z = (SPX - 0.055, COL_X + 0.035), (-0.07, 0.07), (0.335, 0.475)  # the head casting
COVER_X, COVER_Y, COVER_Z = (SPX - 0.065, SPX + 0.235), (-0.085, 0.085), (0.475, 0.555)  # the belt and pulley cover
MOTOR_XY, MOTOR_R, MOTOR_Z = (SPX + 0.20, 0.0), 0.055, (0.36, 0.475)

# -- the work: a spoil board on the table, the plate on it ---------------------------------------------------------------
PX, PY = (SPX - 0.150, SPX + 0.080), (-0.100, 0.100)  # a waxed melamine-faced MDF spoil board, 230 x 200 x 10 mm,
# bolted to the table and overhanging its front by 7 cm
PT = TOP + TABLE_Z[1] + 0.010  # its top
PLATE_X, PLATE_Y, PLATE_T = (SPX - 0.105, SPX + 0.025), (-0.085, 0.065), 0.010  # 130 x 150 x 10 mm aluminium plate,
# where it is drilled: its punch mark under the bit
PLATE_START = 0.020  # the plate starts this far in front of that (toward the arms): its mark 20 mm in front of the bit
PLATE_MASS = 0.53
PLATE_C = np.array([(PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2])
PLATE_C0 = PLATE_C - np.array([PLATE_START, 0.0])  # where it starts
PLATE_TOP = PT + PLATE_T
MARK_IN_PLATE = np.array([SPX, SPY]) - PLATE_C  # the punch mark, from the plate's middle

# -- the quill, chuck and bit ------------------------------------------------------------------------------------------
CLEAR = 0.005  # the bit's tip 5 mm above the plate at rest (the table set so)
TIP0 = PLATE_TOP + CLEAR
BIT_R, BIT_L = 0.0025, 0.055  # a 5 mm HSS jobber drill, 55 mm out of the chuck
CHUCK_R, CHUCK_L = 0.023, 0.076  # 13 mm keyed chuck, 46 x 76 mm
NOSE_L = 0.005  # the spindle nose between the chuck and the quill
QUILL_R = 0.020
QUILL_Z0 = TIP0 + BIT_L + CHUCK_L + NOSE_L  # the quill's bottom at rest
QUILL_MASS, CHUCK_MASS = 0.95, 0.45
TRAVEL = 0.050  # the quill's full travel
DEPTH_STOP = 0.024  # the depth stop: the quill goes down at most 24 mm (the tip 9 mm into the 10 mm spoil board)
PINION_R = 0.015  # m of quill per radian of the feed handle: 94 mm per turn, the full 50 mm in 191 degrees
SPRING_F0, SPRING_K = 40.0, 530.0  # the return spring at the quill: 40 N at rest, rising 530 N/m (66.5 N at 50 mm)
GEAR_K, GEAR_D = 1.0e5, 60.0  # N/m and N s/m at the quill: the emulated rack and pinion's stiffness
DRIVE_MAX = 1.0e4
CUT_D = (50000.0, 8000.0)  # cutting thrust per m/s of feed, in the aluminium and in the MDF (N s/m)
TURNING = 1.0  # rad/s: the chuck counts as turning above this (about 10 rpm)
SPIN_RPM = 180.0  # the simulated chuck speed with the motor on (the real drill: 740-3140 rpm)
SPIN_TAU_ON, SPIN_TAU_OFF = 0.25, 1.0  # s: spin-up and run-down time constants (the chuck coasts about 3 s to a stop)
CHUCK_I = 0.5 * CHUCK_MASS * CHUCK_R ** 2

# -- the feed handle: a three-spoke capstan on the right of the head, its shaft the pinion's ----------------------------
HUB = np.array([SPX + 0.025, -0.110, TOP + 0.390])  # its axis (along y) and the spokes' plane
SPOKE_L = 0.140  # hub centre to knob centre
SPOKE_R, KNOB_R = 0.005, 0.0125  # 10 mm steel spokes, 25 mm black ball knobs
SPOKES0 = (-70.0, 50.0, 170.0)  # the spokes at rest, degrees from straight down (- toward the front)
CAPSTAN_MASS = 0.43

# -- the switch: a no-volt-release push-button switch box on the head's front ------------------------------------------
SW_X = (HEAD_X[0] - 0.040, HEAD_X[0])  # 40 mm deep box on the head's front face
SW_Y, SW_Z = (-0.075, -0.005), (TOP + 0.370, TOP + 0.430)
BTN_Z = TOP + 0.400
BTN_START_Y, BTN_STOP_Y = -0.022, -0.058
BTN_START_R, BTN_STOP_R = 0.010, 0.014  # green 20 mm start button, red 28 mm stop button
BTN_START_OUT, BTN_STOP_OUT = 0.007, 0.009  # how far each stands out of the box
BTN_TRAVEL, BTN_SWITCH = 0.004, 0.002  # each presses in 4 mm and switches at 2 mm
BTN_K, BTN_PRE = 600.0, 0.008  # its spring: 4.8 N to start moving, 7.2 N at the bottom

# -- the clamp: a screw hold-down built from stock parts. An aluminium riser (25 x 25 mm bar) stands left of the plate,
# bolted through the spoil board; on it an aluminium arm (25 x 12 mm bar) reaches over the plate's front-left part,
# tapped M8 for the clamp screw: a cast-iron star knob (40 mm across its lobes, on its own M8 x 40 steel stud)
# whose stud end carries a flat push-on rubber tip (19 mm across, 12.7 mm long), bonded so that it turns with it.
# Turning the knob clockwise (from above) drives the stud down PITCH per turn; once the tip is on the plate the rubber
# under the stud's end compresses and presses, and its friction turns the plate with it unless the plate is held.
# Origin of the screw (knob, stud and tip, one body): the middle of the tip's underside.
CLAMP_XY = np.array([SPX - 0.068, 0.040])
TIP_R, TIP_H = 0.009525, 0.0127  # the rubber tip, 3/4 in across and 1/2 in long
STUD_R, STUD_Z = 0.004, (0.0037, 0.0437)  # the M8 x 40 stud, its end 9 mm up inside the tip
PITCH = 0.00125  # m per turn: M8 x 1.25
GAP0 = 0.00125  # the tip starts one turn (1.25 mm) above the plate
SCREW0 = np.array([*CLAMP_XY, PLATE_TOP + GAP0])
ARM_Z = (0.018, 0.030)  # the arm (12 mm thick), in the screw's starting frame (from the tip's underside)
ARM_W = 0.025
RISER_Y = (0.067, 0.092)  # the riser (25 x 25 mm), world y, 2 mm left of the plate's left edge
KNOB_Z0 = STUD_Z[1]  # the knob's underside, in the screw's starting frame
KNOB_R, KNOB_H = 0.020, 0.025  # the star knob: 40 mm across its lobes, 25 mm tall
KNOB_HUB_R, KNOB_HUB_H = 0.007, 0.012  # its hub, 14 mm across and 12 mm tall, under the lobes
KNOB_CORE_R, LOBE_R, LOBES = 0.0105, 0.0050, 5  # the lobes' root circle; each lobe ends in a 10 mm round
SCREW_MASS = 0.073  # the knob (53 g) with its stud and the tip
SCREW_TRAVEL = 0.012  # the stud can go 12 mm down before the knob meets the arm
TIP_K = 3.0e5  # N/m: the rubber under the stud's 8 mm end compressing, with the arm's bending (computed)
SCREW_K, SCREW_D, SCREW_MAX = TIP_K, 100.0, 1000.0
THREAD_DRAG, THREAD_T0 = 2.0, 0.05  # the thread's own drag with no load: a stiff damper (N m s/rad) passing at most
# 0.05 N m, i.e. a light dry friction that a knob rubbed by an opening hand does not overcome
TIP_FRICTION = 0.63  # the rubber tip on the aluminium plate (it also carries the thread's own friction, see README)
TIGHT_N = 85.0  # clamped: the tip pressing at least this (a real knob tightened to 0.5 N m)
TIP_ON = 0.0005  # the tip's underside within this of the plate's top: on it

STEEL, ALU, DARK, RED, BLACK, GREEN = (0.55, 0.57, 0.60), (0.80, 0.81, 0.83), (0.20, 0.21, 0.23), \
    (0.85, 0.12, 0.10), (0.05, 0.05, 0.05), (0.10, 0.62, 0.20)
PAINT, PAINT_DARK, YELLOW, WHITE = (0.30, 0.42, 0.55), (0.22, 0.31, 0.41), (0.95, 0.78, 0.10), (0.92, 0.92, 0.90)
BOARD = (0.86, 0.85, 0.80)  # white melamine
ALU_DARK, IRON = (0.66, 0.67, 0.70), (0.24, 0.24, 0.25)  # aluminium bar stock; the cast-iron star knob

# -- scoring ----------------------------------------------------------------------------------------------------------
SAFE = 0.05  # every point of each gripper's hand and wrist at least this far from the bit and the chuck while they turn
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")  # the wrist bodies, the gripper and its fingers
ON_MARK = 0.005  # the hole (or, before drilling, the mark) within this of where it should be
THROUGH = PLATE_T + 0.0015  # the hole this deep: through the 10 mm plate and 1.5 mm into the spoil board
FORCED = 0.0005  # the bit pushed this far past the hole's bottom while not turning: forced into the part
QUILL_UP = 0.003  # the quill within this of its top stop: up
FLING = (0.6, 25.0)  # an unclamped plate the bit catches: flung at this speed (m/s) and spin (rad/s)
SUCCESS_HOLD_S = 1.0


def _cyl_between(p0, p1, r: float, color, collide: bool = True) -> list:
    """A round rod from p0 to p1 (body frame) as two square boxes turned 45 degrees about it (an octagonal section)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    x = d / length
    z = np.cross(x, [0.0, 1.0, 0.0]) if abs(x[1]) < 0.9 else np.cross(x, [1.0, 0.0, 0.0])
    z /= np.linalg.norm(z)
    out = []
    for turn in (0.0, math.pi / 4):
        zz = z * math.cos(turn) + np.cross(x, z) * math.sin(turn)
        m = np.column_stack([x, np.cross(zz, x), zz])
        out.append(BoxShape(tuple(((p0 + p1) / 2).tolist()), (length, 2 * r * 0.83, 2 * r * 0.83), color,
                            collide=collide, quat_xyzw=bodykit._quat_from_matrix(m)))
    return out


def _spoke_dir(beta_deg: float) -> np.ndarray:
    """A spoke's direction in the x-z plane, ``beta_deg`` from straight down (- toward the front)."""
    b = math.radians(beta_deg)
    return np.array([math.sin(b), 0.0, -math.cos(b)])


def _board() -> Body:
    """The waxed melamine-faced MDF spoil board on the drill's table: the bit drills through the plate into it."""
    shapes = [BoxShape(((PX[0] + PX[1]) / 2, (PY[0] + PY[1]) / 2, 0.005), (PX[1] - PX[0], PY[1] - PY[0], 0.010), BOARD)]
    return Body.build("board", pos=(0.0, 0.0, PT - 0.010), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.05, 0.05), slick=True, color=BOARD)


def _plate() -> Body:
    m = MARK_IN_PLATE
    shapes = [BoxShape((0.0, 0.0, 0.0), (PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_T), ALU),
              BoxShape((m[0], m[1], PLATE_T / 2 + 0.0002), (0.010, 0.0012, 0.0004), BLACK, collide=False),  # the mark
              BoxShape((m[0], m[1], PLATE_T / 2 + 0.0002), (0.0012, 0.010, 0.0004), BLACK, collide=False)]
    return Body.build("plate", pos=(*PLATE_C0, PT + PLATE_T / 2), mass=PLATE_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=ALU)


def _clamp() -> Body:
    """The clamp's riser and arm (fixed: the riser is bolted through the spoil board, the arm to the riser). Origin at
    the screw's starting origin."""
    bottom = PT - SCREW0[2]
    ry0, ry1 = RISER_Y[0] - CLAMP_XY[1], RISER_Y[1] - CLAMP_XY[1]
    arm_y0 = -ARM_W / 2  # the arm's end, 12.5 mm past the screw's axis
    shapes = [BoxShape((0.0, (ry0 + ry1) / 2, (bottom + ARM_Z[0]) / 2), (0.025, ry1 - ry0, ARM_Z[0] - bottom), ALU_DARK),
              BoxShape((0.0, (arm_y0 + ry1) / 2, (ARM_Z[0] + ARM_Z[1]) / 2), (ARM_W, ry1 - arm_y0, ARM_Z[1] - ARM_Z[0]),
                       ALU_DARK),
              CylShape((0.0, (ry0 + ry1) / 2, ARM_Z[1] + 0.0032), 0.0085, 0.0064, STEEL, collide=False)]  # M10 bolt
    return Body.build("clamp", pos=tuple(SCREW0.tolist()), mass=1.0, shapes=shapes, kinematic=True, color=ALU_DARK)


def _screw() -> Body:
    """The clamp screw: the star knob (cast iron), its steel stud and the rubber tip on the stud's end."""
    z0, z1 = KNOB_Z0 + KNOB_HUB_H, KNOB_Z0 + KNOB_H  # the lobes
    zc, h = (z0 + z1) / 2, z1 - z0
    shapes = [CylShape((0.0, 0.0, TIP_H / 2), TIP_R, TIP_H, BLACK),
              CylShape((0.0, 0.0, (TIP_H + KNOB_Z0) / 2), STUD_R, KNOB_Z0 - TIP_H, DARK),
              CylShape((0.0, 0.0, KNOB_Z0 + KNOB_HUB_H / 2), KNOB_HUB_R, KNOB_HUB_H, IRON),
              CylShape((0.0, 0.0, zc), KNOB_CORE_R, h, IRON)]
    for k in range(LOBES):  # the lobes (modelled as five), each ending in a round; one marked so the turning shows
        a = 2 * math.pi * k / LOBES + math.pi / 2
        u = np.array([math.cos(a), math.sin(a)])
        r_end = KNOB_R - LOBE_R
        shapes.append(BoxShape((*(u * r_end / 2).tolist(), zc), (r_end, 2 * LOBE_R, h), IRON, yaw=a))
        shapes.append(CylShape((*(u * r_end).tolist(), zc), LOBE_R, h, IRON))
        if k == 0:
            shapes.append(CylShape((*(u * r_end).tolist(), z1 + 0.0003), 0.003, 0.0006, WHITE, collide=False))
    return Body.build("screw", pos=tuple(SCREW0.tolist()), mass=SCREW_MASS, shapes=shapes,
                      friction=(TIP_FRICTION, TIP_FRICTION), slick=True, color=IRON, com=(0.0, 0.0, 0.045),
                      solver_iterations=32)


def _drill() -> Body:
    """The drill press's fixed parts: base, column, table, head, belt cover, motor, the capstan's boss and the switch
    box. Origin at the spindle's axis on the bench."""
    def box(x, y, z, color, collide=True):
        return BoxShape(((x[0] + x[1]) / 2 - SPX, (y[0] + y[1]) / 2 - SPY, (z[0] + z[1]) / 2),
                        (x[1] - x[0], y[1] - y[0], z[1] - z[0]), color, collide=collide)

    cx = COL_X - SPX
    shapes = [box(BASE_X, BASE_Y, (0.0, BASE_H), PAINT),
              CylShape((cx, 0.0, (BASE_H + COL_TOP) / 2), COL_R, COL_TOP - BASE_H, STEEL),
              CylShape((cx, 0.0, 0.120), 0.034, 0.060, PAINT),  # the table's collar on the column
              box((COL_X - 0.045, COL_X - 0.020), (-0.025, 0.025), (0.105, TABLE_Z[0]), PAINT),  # the table's arm
              box(TABLE_X, TABLE_Y, TABLE_Z, PAINT_DARK),  # the cast-iron table
              box((TABLE_X[0] + 0.012, TABLE_X[1] - 0.012), (TABLE_Y[0] + 0.012, TABLE_Y[1] - 0.012),
                  (TABLE_Z[0] - 0.025, TABLE_Z[0]), PAINT_DARK),  # its ribs
              box(HEAD_X, HEAD_Y, HEAD_Z, PAINT),
              box(COVER_X, COVER_Y, COVER_Z, PAINT),
              CylShape((MOTOR_XY[0] - SPX, MOTOR_XY[1], (MOTOR_Z[0] + MOTOR_Z[1]) / 2), MOTOR_R,
                       MOTOR_Z[1] - MOTOR_Z[0], DARK),
              CylShape((HUB[0] - SPX, (HEAD_Y[0] + HUB[1] + 0.008) / 2, HUB[2] - TOP), 0.020,
                       HEAD_Y[0] - HUB[1] - 0.008, PAINT_DARK, axis="Y"),  # the capstan's boss on the head
              box(SW_X, SW_Y, (SW_Z[0] - TOP, SW_Z[1] - TOP), BLACK),  # the switch box
              box((SW_X[0] - 0.0005, SW_X[0]), (SW_Y[0] + 0.006, SW_Y[1] - 0.006), (SW_Z[1] - TOP - 0.012,
                                                                                    SW_Z[1] - TOP - 0.004),
                  YELLOW, collide=False),  # its label strip
              box((HEAD_X[0] - 0.0005, HEAD_X[0]), (0.010, 0.050), (0.395, 0.407), WHITE, collide=False)]  # a plate
    return Body.build("drill", pos=(SPX, SPY, TOP), mass=20.0, shapes=shapes, kinematic=True, color=PAINT)


def _quill() -> Body:
    """The quill (it slides in the head on its rack) and the spindle's nose. Origin at the quill's bottom on the axis."""
    shapes = [CylShape((0.0, 0.0, 0.050), QUILL_R, 0.100, STEEL),
              CylShape((0.0, 0.0, -NOSE_L / 2), 0.015, NOSE_L, DARK)]
    return Body.build("quill", pos=(SPX, SPY, QUILL_Z0), mass=QUILL_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=STEEL, com=(0.0, 0.0, 0.03), solver_iterations=32)


def _chuck() -> Body:
    """The chuck and the bit, turning with the spindle. Origin at the bit's tip."""
    z0 = BIT_L
    shapes = [CylShape((0.0, 0.0, BIT_L / 2), BIT_R, BIT_L, STEEL, collide=False),  # the bit
              CylShape((0.0, 0.0, z0 + CHUCK_L * 0.55), CHUCK_R, CHUCK_L * 0.9, STEEL),  # the sleeve
              CylShape((0.0, 0.0, z0 + CHUCK_L * 0.05), 0.012, CHUCK_L * 0.1, DARK),  # the jaws' nose
              CylShape((0.0, 0.0, z0 + CHUCK_L * 0.35), CHUCK_R + 0.0005, 0.012, DARK, collide=False)]  # knurled ring
    for k in range(3):  # the three key holes and a white stripe show it turning
        a = 2 * math.pi * k / 3
        shapes.append(BoxShape((CHUCK_R * math.cos(a), CHUCK_R * math.sin(a), z0 + CHUCK_L * 0.7), (0.004, 0.006, 0.006),
                               BLACK, yaw=a, collide=False))
    shapes.append(BoxShape((CHUCK_R + 0.0003, 0.0, z0 + CHUCK_L * 0.55), (0.0012, 0.008, CHUCK_L * 0.85), WHITE,
                           collide=False))
    return Body.build("chuck", pos=(SPX, SPY, TIP0), mass=CHUCK_MASS, shapes=shapes, friction=(0.6, 0.5), color=STEEL,
                      com=(0.0, 0.0, z0 + CHUCK_L * 0.5), damping=(0.0, 0.0))


def _capstan() -> Body:
    """The feed handle: a hub on the pinion's shaft with three spokes and ball knobs. Origin on the shaft's axis."""
    shapes = [CylShape((0.0, 0.0, 0.0), 0.022, 0.016, PAINT_DARK, axis="Y"),
              CylShape((0.0, 0.016, 0.0), 0.014, 0.018, STEEL, axis="Y")]
    for b in SPOKES0:
        d = _spoke_dir(b)
        shapes += _cyl_between(d * 0.020, d * (SPOKE_L - KNOB_R + 0.002), SPOKE_R, STEEL)
        shapes.append(SphereShape(tuple((d * SPOKE_L).tolist()), KNOB_R, BLACK))
    return Body.build("capstan", pos=tuple(HUB.tolist()), mass=CAPSTAN_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=STEEL, com=(0.0, 0.0, 0.0), solver_iterations=32)


def _button(name: str, y: float, r: float, out: float, color) -> Body:
    """A push button on the switch box, origin at the box's front face on its axis; it stands ``out`` proud."""
    shapes = [CylShape((-out / 2, 0.0, 0.0), r, out, color, axis="X"),
              CylShape((0.004, 0.0, 0.0), r * 0.8, 0.008, color, axis="X", collide=False)]
    return Body.build(name, pos=(SW_X[0], y, BTN_Z), mass=0.01, shapes=shapes, friction=(1.0, 0.9), color=color)


def _marker(name: str, shape, at) -> Body:
    """A small fixed marker the task file moves into view (the hole drilled)."""
    return Body.build(name, pos=tuple(float(v) for v in at), mass=0.01, shapes=[shape], kinematic=True,
                      color=shape.color)


HIDE_HOLE = np.array([SPX, 0.30, TOP - 0.02])  # under the table
OBJECTS = [_board(), _plate(), _clamp(), _screw(), _drill(), _quill(), _chuck(), _capstan(),
           _button("btn_start", BTN_START_Y, BTN_START_R, BTN_START_OUT, GREEN),
           _button("btn_stop", BTN_STOP_Y, BTN_STOP_R, BTN_STOP_OUT, RED),
           _marker("hole", CylShape((0.0, 0.0, 0.0), BIT_R, 0.0006, BLACK, collide=False), HIDE_HOLE)]

thread("clamp_screw", "clamp", "screw", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(0.0, SCREW_TRAVEL), turn_damping=THREAD_DRAG, slide_stiffness=SCREW_K,
       slide_damping=SCREW_D, slide_max_force=SCREW_MAX, always_on=True)
_plain_author_threads = bodykit._author_threads


def _author_threads(stage, prim_path: str, spec) -> None:
    """bodykit's threads, with the clamp's slide drive aimed at its start (bodykit aims it at the far end of its
    travel, which would drive the stiff rubber tip into the plate while the scene settles)."""
    _plain_author_threads(stage, prim_path, spec)
    if spec.name == "screw":
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/clamp_screw")
        UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr().Set(0.0)
        UsdPhysics.DriveAPI(prim, "rotX").GetMaxForceAttr().Set(float(THREAD_T0))


bodykit._author_threads = _author_threads
# The quill slides in the head (z, - down) to the depth stop; the feed handle turns on the pinion's shaft (about +y: a
# feed turns it -). PhysX's rack-and-pinion joint does not run in this GPU scene, so both carry a stiff drive toward one
# shared gear position, re-aimed before every physics step (see _gear): a stiff, slightly springy mesh. The return
# spring and the cutting thrust act on the quill's drive.
_DEG = math.pi / 180.0
slider("feed", "drill", "quill", anchor=(SPX, SPY, QUILL_Z0), axis="Z", limits=(-DEPTH_STOP, 0.0), stiffness=GEAR_K,
       damping=GEAR_D, target=SPRING_F0 / GEAR_K, max_force=DRIVE_MAX)
hinge("pinion", "drill", "capstan", anchor=tuple(HUB.tolist()), axis="Y",
      limits_deg=(-math.degrees(DEPTH_STOP / PINION_R) - 3.0, 3.0), spring=GEAR_K * PINION_R ** 2 * _DEG,
      damping=GEAR_D * PINION_R ** 2 * _DEG, target_deg=0.0)
# the spindle turns the chuck: a velocity drive the switch turns on and off (see _motor)
hinge("spin", "quill", "chuck", anchor=(SPX, SPY, TIP0 + BIT_L + CHUCK_L), axis="Z",
      damping=CHUCK_I / SPIN_TAU_OFF * _DEG)
_plain_author_hinges = bodykit._author_hinges


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, with the spindle's drive given a target speed attribute from the start (the motor sets it)."""
    _plain_author_hinges(stage, prim_path, spec)
    if spec.name == "chuck":
        from pxr import UsdPhysics

        UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/spin"), "angular").CreateTargetVelocityAttr(0.0)


bodykit._author_hinges = _author_hinges
slider("start", "drill", "btn_start", anchor=(SW_X[0], BTN_START_Y, BTN_Z), axis="X", limits=(0.0, BTN_TRAVEL),
       stiffness=BTN_K, damping=2.0, target=-BTN_PRE, max_force=50.0)
slider("stop", "drill", "btn_stop", anchor=(SW_X[0], BTN_STOP_Y, BTN_Z), axis="X", limits=(0.0, BTN_TRAVEL),
       stiffness=BTN_K, damping=2.0, target=-BTN_PRE, max_force=50.0)


# -- the hands' surface points, for the clearance to the turning bit and chuck -----------------------------------------

_HAND: dict = {}


def _hand_points() -> dict[str, np.ndarray]:
    """Surface points of each hand link's collision meshes (vertices and triangle centres), in the link's frame."""
    if _HAND:
        return _HAND
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF

    def rpy_matrix(r, p, y):
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])

    root = ET.parse(YAM_URDF).getroot()
    for link in root.findall("link"):
        name = link.attrib["name"]
        if name not in HAND_LINKS:
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.attrib.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.attrib.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            mesh = col.find("geometry/mesh")
            raw = (pathlib.Path(YAM_URDF).parent / mesh.attrib["filename"]).read_bytes()
            n = int(np.frombuffer(raw[80:84], "<u4")[0])
            rec = np.frombuffer(raw[84:84 + 50 * n], np.dtype([("nv", "<f4", (12,)), ("a", "<u2")]))
            tri = rec["nv"][:, 3:].reshape(-1, 3, 3).astype(float)
            scale = np.array([float(v) for v in mesh.attrib.get("scale", "1 1 1").split()])
            p = np.concatenate([tri.reshape(-1, 3), tri.mean(axis=1)]) * scale
            pts.append(p @ rpy_matrix(*rpy).T + xyz)
        _HAND[name] = np.unique(np.round(np.concatenate(pts), 4), axis=0)
    return _HAND


def _clearance(base, origin, chuck_pos) -> list[float]:
    """Each arm's closest distance (m) from any point of its hand links to the turning parts: the bit (from its tip
    up) and the chuck with the spindle's nose."""
    pts = _hand_points()
    axis = chuck_pos[:2]
    tip = chuck_pos[2]
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        best = 1.0
        for i, n in enumerate(robot.body_names):
            key = n.rsplit("/", 1)[-1]
            if key not in pts:
                continue
            g = poses[i]
            p = pts[key] @ rotation(g[3:7]).T + (g[:3] - origin)
            rho = np.linalg.norm(p[:, :2] - axis, axis=1)
            h = p[:, 2]
            d = np.full(len(p), 1.0)
            for r, z0, z1 in ((BIT_R, tip, tip + BIT_L), (CHUCK_R, tip + BIT_L, tip + BIT_L + CHUCK_L + NOSE_L)):
                dr = np.maximum(rho - r, 0.0)
                dz = np.maximum(np.maximum(z0 - h, h - z1), 0.0)
                d = np.minimum(d, np.hypot(dr, dz))
            best = min(best, float(d.min()))
        out.append(best)
    return out


# -- followed before every physics step: the rack and pinion, the return spring, the cutting thrust --------------------

def _fresh() -> dict:
    return {"phi": None, "turns": 0.0, "target": 0.0, "press": 0.0, "press_max": 0.0, "tcps": None, "unsafe": False, "caught": False, "forced": False,
            "hole": 0.0, "hole_xy": None, "hole_ok": False, "motor": False, "cutting": None, "clear": [1.0, 1.0],
            "clear_min": [1.0, 1.0], "omega": 0.0, "st": {}, "gear_c": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("DRILL_DEBUG")), "steps": 0}
_ATTRS: dict = {}


def _spin_attrs():
    if "spin" not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS["spin"]), "angular")
        _ATTRS["spin"] = (drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr(0.0), drive.GetDampingAttr())
    return _ATTRS["spin"]


def _motor(on: bool) -> None:
    """The motor drives the spindle at its speed, or (off) lets the chuck run down."""
    vel, damp = _spin_attrs()
    vel.Set(float(-SPIN_RPM * 6.0) if on else 0.0)  # deg/s, clockwise seen from above
    damp.Set(float(CHUCK_I / (SPIN_TAU_ON if on else SPIN_TAU_OFF) * _DEG))
    _EP["motor"] = on


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _feed_state(base, origin) -> tuple[float, float]:
    """How far the quill is down (m) and how far the feed handle has turned, as quill travel (m)."""
    q, _ = _pose(base, origin, "quill")
    _, rc = _pose(base, origin, "capstan")
    turn = math.atan2(float(rc[0, 2]), float(rc[0, 0]))  # about +y
    return float(QUILL_Z0 - q[2]), -PINION_R * turn


def _gear(base) -> None:
    """The rack and pinion: before each physics step, aim the quill's and the feed handle's drives at the mean of where
    each says the gear is, so a torque on the handle reaches the quill as that torque over the pinion's radius, and the
    quill's spring and cutting thrust come back to the handle. The return spring pushes the quill up; while the turning
    bit cuts at the bottom of its hole, the quill's drive damps its feed by the cutting thrust."""
    origin = as_numpy(base.scene.env_origins[0])
    s_q, s_c = _feed_state(base, origin)
    c = (s_q + s_c) / 2.0
    _EP["gear_c"] = c
    _EP["knob_n"] = GEAR_K * PINION_R * (s_c - c) / SPOKE_L  # the push on a knob the mesh is passing on (N)
    _EP["knob_max"] = max(_EP.get("knob_max", 0.0), _EP["knob_n"])
    damp = GEAR_D
    cut = _EP.get("cutting")
    if cut is not None and _EP["omega"] > TURNING:
        depth = s_q - cut[0]  # how far the tip is below the plate's top
        if depth > _EP["hole"] - 0.0003 and cut[1]:
            damp += CUT_D[0] if depth < PLATE_T else CUT_D[1]
    set_slider_drive("feed", target=-c + (SPRING_F0 + SPRING_K * max(c, 0.0)) / GEAR_K, damping=damp)
    set_hinge_drive("pinion", target_deg=math.degrees(-c / PINION_R))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp_step(base) -> None:
    """The clamp's thread, before each physics step: the stud comes down PITCH per clockwise turn. Its slide drive is the
    stiffness of the rubber under the stud's end, so once the tip is on the plate each further turn presses harder."""
    origin = as_numpy(base.scene.env_origins[0])
    s_pos, s_rot = _pose(base, origin, "screw")
    phi = math.atan2(float(s_rot[1, 0]), float(s_rot[0, 0]))
    if _EP["phi"] is not None:
        _EP["turns"] -= _wrap(phi - _EP["phi"]) / (2 * math.pi)  # clockwise (from above) is +
    _EP["phi"] = phi
    target = min(max(PITCH * _EP["turns"], 0.0), SCREW_TRAVEL)
    set_slide_target("clamp_screw", target)
    _EP["target"] = target
    slide = float(SCREW0[2] - s_pos[2])  # how far the stud has come down
    _EP["press"] = min(SCREW_K * max(target - slide, 0.0), SCREW_MAX)  # the tip pressing on what is under it
    _EP["press_max"] = max(_EP["press_max"], _EP["press"])


def _hook_gear(base) -> None:
    scene = base.scene
    if getattr(scene, "_drill_gear", False):
        return
    plain = scene.write_data_to_sim

    def write_data_to_sim(*args, **kwargs):
        _gear(base)
        _clamp_step(base)
        return plain(*args, **kwargs)

    scene.write_data_to_sim = write_data_to_sim
    scene._drill_gear = True


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_thread_on("clamp_screw", True)
    set_slide_target("clamp_screw", 0.0)
    try:
        _motor(False)
        set_slider_drive("feed", target=SPRING_F0 / GEAR_K, damping=GEAR_D)
        set_hinge_drive("pinion", target_deg=0.0)
    except KeyError:
        pass


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _teleport(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _fling(base) -> None:
    import torch

    asset = base.scene["plate"]
    v = np.array([-FLING[0], 0.0, 0.05])
    asset.write_root_velocity_to_sim(torch.tensor([[*v.tolist(), 0.0, 0.0, FLING[1]]], dtype=torch.float32,
                                                  device=asset.device))


def _button_in(base, origin, name, y) -> float:
    p, _ = _pose(base, origin, name)
    return float(p[0] - SW_X[0])


def _state(b_pos, b_rot, s_pos, down, buttons) -> dict:
    """The clamp, the plate and the drill as they are."""
    slide = float(SCREW0[2] - s_pos[2])  # how far the stud has come down
    force = _EP["press"]
    pad = b_rot.T @ (s_pos - b_pos)  # the tip's underside, in the plate's frame
    on_plate = (abs(pad[0]) < (PLATE_X[1] - PLATE_X[0]) / 2 and abs(pad[1]) < (PLATE_Y[1] - PLATE_Y[0]) / 2
                and abs(pad[2] - PLATE_T / 2) < TIP_ON)
    clamped = on_plate and force >= TIGHT_N
    mark = b_pos + b_rot @ np.array([MARK_IN_PLATE[0], MARK_IN_PLATE[1], PLATE_T / 2])
    return {"slide": slide, "force": force, "on_plate": on_plate, "clamped": clamped, "down": down,
            "mark_off": float(np.linalg.norm(mark[:2] - np.array([SPX, SPY]))),
            "flat": float(b_rot[2, 2]) > math.cos(math.radians(10.0)), "buttons": buttons}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    _hook_gear(base)
    origin = as_numpy(base.scene.env_origins[0])
    b_pos, b_rot = _pose(base, origin, "plate")
    s_pos, s_rot = _pose(base, origin, "screw")
    k_pos, _ = _pose(base, origin, "chuck")  # the bit's tip
    _EP["tcps"] = _tcps(base, origin)
    # the switch: the start button pressed past its switch point starts the motor, the stop button stops it
    b_start, b_stop = _button_in(base, origin, "btn_start", BTN_START_Y), _button_in(base, origin, "btn_stop",
                                                                                     BTN_STOP_Y)
    if b_stop >= BTN_SWITCH and _EP["motor"]:
        _motor(False)
    elif b_start >= BTN_SWITCH and b_stop < BTN_SWITCH and not _EP["motor"]:
        _motor(True)
    omega = abs(float(as_numpy(base.scene["chuck"].data.root_ang_vel_w)[0][2]))
    _EP["omega"] = omega
    turning = omega > TURNING
    down = float(TIP0 - k_pos[2])
    st = _state(b_pos, b_rot, s_pos, down, (b_start, b_stop))
    _EP["st"] = st
    # safety: every point of each hand at least SAFE from the bit and the chuck while they turn
    clear = _clearance(base, origin, k_pos)
    _EP["clear"] = clear
    if turning:
        _EP["clear_min"] = [min(a, b) for a, b in zip(_EP["clear_min"], clear)]
        if min(clear) < SAFE:
            _EP["unsafe"] = True
    # drilling: the tip in the plate (its footprint, below its top)
    tip = b_rot.T @ (k_pos - b_pos)  # the tip in the plate's frame
    over_plate = abs(tip[0]) < (PLATE_X[1] - PLATE_X[0]) / 2 and abs(tip[1]) < (PLATE_Y[1] - PLATE_Y[0]) / 2
    depth = PLATE_T / 2 - float(tip[2])  # below the plate's top
    _EP["cutting"] = (down - depth, bool(st["clamped"]) and over_plate) if over_plate else None
    if not _EP["caught"] and over_plate and depth > _EP["hole"] + FORCED:
        if not turning:
            _EP["forced"] = True  # a bit that is not turning only presses on the plate
        elif not st["clamped"]:
            _EP["caught"] = True  # the bit catches a loose plate and flings it
            _fling(base)
        else:
            if _EP["hole_xy"] is None:
                _EP["hole_xy"] = np.array(tip[:2])
                _EP["hole_ok"] = float(np.linalg.norm(tip[:2] - MARK_IN_PLATE)) <= ON_MARK
                _teleport(base, origin, "hole", b_pos + b_rot @ np.array([tip[0], tip[1], PLATE_T / 2 + 0.0003]))
            _EP["hole"] = max(_EP["hole"], depth)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            s_q, s_c = _feed_state(base, origin)
            _, rc = _pose(base, origin, "capstan")
            print(f"[drill] t={_DEBUG['steps'] / 20:5.1f}s turns={_EP['turns']:5.2f} slide={st['slide'] * 1000:6.2f}mm "
                  f"target={_EP['target'] * 1000:6.2f}mm force={st['force']:5.1f}N max={_EP['press_max']:5.1f}N "
                  f"on={st['on_plate']} "
                  f"clamped={st['clamped']} plate={np.round(b_pos * 1000).astype(int).tolist()} "
                  f"plate_yaw={math.degrees(math.atan2(b_rot[1, 0], b_rot[0, 0])):6.1f} "
                  f"mark_off={st['mark_off'] * 1000:5.1f}mm quill={s_q * 1000:5.1f} handle={s_c * 1000:5.1f}mm "
                  f"handle_deg={-math.degrees(math.atan2(float(rc[0, 2]), float(rc[0, 0]))):6.1f} "
                  f"knob={_EP.get('knob_n', 0.0):5.1f}N knob_max={_EP.get('knob_max', 0.0):5.1f}N "
                  f"btn=({b_start * 1000:.1f},{b_stop * 1000:.1f})mm motor={_EP['motor']} omega={omega:5.1f} "
                  f"hole={_EP['hole'] * 1000:5.1f}mm hole_ok={_EP['hole_ok']} caught={_EP['caught']} "
                  f"forced={_EP['forced']} unsafe={_EP['unsafe']} clear=({clear[0] * 1000:.0f},{clear[1] * 1000:.0f})mm "
                  f"clear_min=({_EP['clear_min'][0] * 1000:.0f},{_EP['clear_min'][1] * 1000:.0f}) "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} rightTCP={np.round(tr * 1000).astype(int).tolist()} "
                  f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["st"]
    through = _EP["hole"] >= THROUGH and _EP["hole_ok"]
    return {"through": through, "unsafe": _EP["unsafe"], "spoilt": _EP["caught"] or _EP["forced"],
            "clamped": bool(st.get("clamped")),
            "in_place": bool(st.get("clamped")) and st.get("mark_off", 1.0) <= ON_MARK and bool(st.get("flat")),
            "quill_up": st.get("down", 1.0) < QUILL_UP, "off": not _EP["motor"] and _EP["omega"] <= TURNING}


def _points(s) -> float:
    if s["spoilt"]:
        return 0.0
    if not s["unsafe"]:
        if s["through"] and s["clamped"] and s["quill_up"] and s["off"]:
            return 1.0
        if s["through"] or s["in_place"]:
            return 0.5
    return 0.25 if s["clamped"] or s["through"] else 0.0


def score(objects) -> float:
    """Clamped: the clamp's rubber tip on the plate pressing at least 85 N (a knob tightened to about 0.5 N m). Drilled
    through: the turning bit cut 11.5 mm below the plate's top while it was clamped, through the
    10 mm plate into the spoil board.
    1.0 with a hole drilled through within 5 mm of the mark, no part of either hand within 5 cm of the bit or chuck
    while they turned, and at the end the plate still clamped, the quill up and the motor off with the chuck stopped;
    0.5 drilled through so (safely) but the drill left running or the quill down or the clamp loosened, or the plate
    clamped with its mark under the bit (within 5 mm) but not drilled through; 0.25 the plate clamped but off its mark,
    or clamped or drilled with a hand near the turning bit; 0.0 otherwise, or if the bit caught a loose plate or was
    pushed into the plate while not turning."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the plate, the quill, the feed handle, the clamp's screw and the chuck still: the trial ends once
    it has held 1 s."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("plate", "quill", "capstan", "screw", "chuck"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down on the plate's front edge, fingers closed, presses it onto the board and slides it back
# 20 mm, bringing its punch mark under the bit; it lifts off and comes down again on the plate's front-right part and
# keeps pressing. The left hand comes down on the clamp's star knob, fingers closing across its lobes, and turns it
# clockwise in strokes (grip, turn a quarter turn, let go, rise, turn back in the air, come down, grip again): four
# strokes bring the rubber tip down onto the plate, a fifth and a shorter sixth press it past 85 N. The left rises and
# goes home; the right lets go of the plate, presses the green start button, takes the feed handle's front knob, pulls
# the handle down, feeding the bit through the plate into the spoil board, and lets it back up under control. It lets
# go, presses the red stop button and goes home.

L_TILT = math.radians(45.0)  # the left hand on the clamp's knob leans out to the left and toward the arms
R_TILT = math.radians(55.0)  # the right hand pressing the plate leans toward the arms (clear of the head)
R_WRIST = 200.0  # degrees (seen from above): where the right wrist sits from its fingers, toward the arms
R_PRESS_XY = (SPX - 0.092, -0.072)  # where the right holds the plate (in place): its front-right part
R_PRESS = 0.030  # the right aims 3 cm below where its fingers meet the plate: the arm gives, pressing about 12 N
R_LEAN = 0.008  # and this far back toward the arms: a leaning hand slides forward as it presses
R_PUSH_XY = (PLATE_X[0] - PLATE_START + 0.015, PLATE_C[1] + 0.018)  # where the right slides the plate from: its front
# edge region, aimed so that its pressing fingertip (to the right of the TCP) is near the centre line and the slide
# turns the plate little
R_PUSH = 0.025  # how deep it aims there
R_SLIDE_OVER = (0.001, -0.006)  # the slide's end, against how the plate creeps as it is pressed and slid
R_SLIDE_S = 2.0
KNOB_IN_HAND = KNOB_Z0 + (KNOB_HUB_H + KNOB_H) / 2  # the left TCP this far above the screw's origin: level with the
# middle of the knob's lobes
KNOB_OPEN = FINGER_OPEN  # the left's fingers wide open round the 40 mm knob while it turns back
STROKE = math.radians(90.0)
STROKES = 6
LAST_STROKE = math.radians(45.0)  # the last stroke is shorter: a turn brings the tip down, the rest presses it
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 1.0, 0.5
LIFT, LIFT_S = 0.025, 0.4  # between strokes the open hand rises off the knob and turns back in the air
LEAD_S, LEAD_STEPS = 3.0, 15
CAP_GAMMA = math.radians(25.0)  # the right hand on the knob: pointing left, leaning this far toward the spoke's normal
FEED_BETA = (SPOKES0[0], SPOKES0[0] + 19.0, 15.0)  # the front spoke: at rest; the tip at the plate; the end of the pull
FEED_S = (1.5, 13.0)  # s for the approach and for the cut
PRESS_X = 0.019  # the right's closed fingertips aimed this far past a button's face (it gives under the spring)
SW_APPROACH = 0.035  # the right's TCP this far out from a button before and after pressing it


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot(axis, a: float) -> np.ndarray:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _frame(approach, closing) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float) - np.dot(closing, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# the left on the knob, in the middle of a stroke: fingers closing along 45 degrees (seen from above), leaned about the
# closing axis so the wrist sits at 135 degrees, out to the left and toward the arms, clear of the head. A stroke turns
# the hand from +45 degrees (fingers closing along y) to -45 degrees (along x)
_KNOB_CLOSE = np.array([math.cos(math.radians(225.0)), math.sin(math.radians(225.0)), 0.0])
_KNOB_WRIST = np.array([math.cos(math.radians(135.0)), math.sin(math.radians(135.0)), 0.0])
KNOB_GRIP = _frame(-math.sin(L_TILT) * _KNOB_WRIST + [0.0, 0.0, -math.cos(L_TILT)], _KNOB_CLOSE)
# the right pressing the plate: fingers closed, leaned about the closing axis so the wrist sits toward the arms
_PRESS_WRIST = np.array([math.cos(math.radians(R_WRIST)), math.sin(math.radians(R_WRIST)), 0.0])
PRESS_R = _frame(-math.sin(R_TILT) * _PRESS_WRIST + [0.0, 0.0, -math.cos(R_TILT)],
                 [math.cos(math.radians(R_WRIST + 90.0)), math.sin(math.radians(R_WRIST + 90.0)), 0.0])
# the right on a button: pointing forward, the fingers closing vertically (thin across, so it presses one button)
BUTTON_GRIP = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def _cap_grip(beta_deg: float) -> np.ndarray:
    """The right hand on the front knob with the spoke at ``beta_deg``: pointing left and leaned CAP_GAMMA toward the
    spoke's normal, the fingers closing across the spoke, which leaves the jaw sideways."""
    d = -_spoke_dir(beta_deg)  # knob to hub
    b = math.radians(beta_deg)
    n = np.array([math.cos(b), 0.0, math.sin(b)])
    a = math.cos(CAP_GAMMA) * np.array([0.0, 1.0, 0.0]) + math.sin(CAP_GAMMA) * n
    return _frame(a, np.cross(a, d))


def _knob_pos(beta_deg: float) -> np.ndarray:
    return HUB + SPOKE_L * _spoke_dir(beta_deg)


_IK_CACHE: dict = {}


def _good_ik(arm: str, pos, rot, seed=None) -> np.ndarray:
    """Joints for the TCP pose: from ``seed`` (the arm's joints just before) if that lands well inside the joint
    limits, else the solution with the most room over a grid of starts."""
    import itertools

    from yam_arena import ik, station

    key = (arm, tuple(np.round(pos, 5)), tuple(np.round(np.asarray(rot), 5).ravel()),
           None if seed is None else tuple(np.round(seed, 4)))
    if key in _IK_CACHE:
        return _IK_CACHE[key]

    def attempt(s):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(s, float),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            return q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        return None

    best = attempt(seed) if seed is not None else None
    if best is None or best[1] < 0.02:
        for s in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3, 2.0), (0.4, 1.0, 1.8), (-0.8, 0.0, 0.8),
                                   (-1.4, 0.0, 1.4), (0.0,)):
            got = attempt(s)
            if got is not None and (best is None or got[1] > best[1]):
                best = got
                if best[1] > 0.15:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {np.round(pos, 3)}")
    _IK_CACHE[key] = best[0]
    return best[0]


def _joint_move(arm: str, q0, pos, rot, label: str, finger, duration: float = LEAD_S, steps: int = LEAD_STEPS,
                seed=None) -> tuple[list[Waypoint], np.ndarray]:
    """Waypoints moving the arm from joints ``q0`` to the pose in a straight line in joint space."""
    from yam_arena import ik, station

    q0 = np.asarray(q0, float)
    q1 = _good_ik(arm, pos, rot, q0 if seed is None else seed)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(duration / steps, tuple(float(v) for v in p), finger, label, r))
    return out, q1


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _q_of(arm: str, w: Waypoint, seed) -> np.ndarray:
    return _good_ik(arm, w.pos, w.yaw, seed)


def _knob_at(turned: float) -> np.ndarray:
    """The left TCP on the knob after the screw has turned ``turned`` (rad); it stops on the plate."""
    down = min(PITCH * turned / (2 * math.pi), GAP0)
    return SCREW0 + [0.0, 0.0, KNOB_IN_HAND - down]


def _ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _left(times: dict) -> list[Waypoint]:
    """Tighten the clamp in quarter-turn strokes, then go home."""
    from yam_arena.constants import READY

    g_start, g_end = _rz(STROKE / 2) @ KNOB_GRIP, _rz(-STROKE / 2) @ KNOB_GRIP
    via = _knob_at(0.0) + [-0.06, 0.08, 0.08]  # out in front and to the left of the head
    p0, r0 = _ready_pose("left")
    out = [_w(times["left_go"], p0, FINGER_OPEN, "wait: the right hand is placing the plate", r0)]
    moves, q = _joint_move("left", READY, via, g_start, "toward the clamp, clear of the head", FINGER_OPEN, 2.0, 10)
    out += moves
    out += [_w(1.2, _knob_at(0.0) + [0.0, 0.0, 0.04], KNOB_OPEN, "over the clamp's knob", g_start),
            _w(1.0, _knob_at(0.0), KNOB_OPEN, "down round the knob", g_start),
            _w(0.6, _knob_at(0.0), FINGER_CLOSED, "grip it", g_start)]
    turned = 0.0
    for k in range(STROKES):
        stroke = LAST_STROKE if k == STROKES - 1 else STROKE
        for j in range(1, 5):  # a quarter of the stroke at a time
            out.append(_w(TURN_S / 4 * stroke / STROKE, _knob_at(turned + stroke * j / 4), FINGER_CLOSED,
                          f"turn it clockwise ({k + 1})", _rz(STROKE / 2 - stroke * j / 4) @ KNOB_GRIP))
        turned += stroke
        at = _knob_at(turned)
        g_end = _rz(STROKE / 2 - stroke) @ KNOB_GRIP
        out.append(_w(RELEASE_S, at, KNOB_OPEN, "let go", g_end))
        if k < STROKES - 1:
            up = at + [0.0, 0.0, LIFT]
            out.append(_w(LIFT_S, up, KNOB_OPEN, "rise off the knob", g_end))
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, up, KNOB_OPEN, "turn back in the air", _rz(-STROKE / 2 + STROKE * j / 4) @ KNOB_GRIP))
            out.append(_w(LIFT_S, at, KNOB_OPEN, "down round the knob", g_start))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the knob again", g_start))
    times["clamped"] = sum(w.duration for w in out)
    rise = _knob_at(turned) + [-0.03, 0.0, 0.03]
    out.append(_w(1.0, rise, FINGER_OPEN, "rise, toward the arms", g_end))
    q = _q_of("left", out[-1], _good_ik("left", _knob_at(turned), g_end, q))
    home = _ready_pose("left")
    moves, q = _joint_move("left", q, home[0], home[1], "home, well away from the drill", FINGER_OPEN, 2.5, 12)
    out += moves
    times["left_home"] = sum(w.duration for w in out)
    return out


def _press_button(y: float, out_len: float, label: str, wait: float = 0.0) -> list[Waypoint]:
    """The right hand to a button (a straight move in front of the switch box), press it in past its stop, back off."""
    away = np.array([SW_X[0] - out_len - SW_APPROACH, y, BTN_Z])
    pushed = np.array([SW_X[0] - out_len + PRESS_X - 0.0128, y, BTN_Z])
    moves = [_w(2.5, away, FINGER_CLOSED, "to the switch", BUTTON_GRIP)]
    if wait > 0.0:
        moves.append(_w(wait, away, FINGER_CLOSED, "wait: the left hand clear", BUTTON_GRIP))
    moves += [_w(1.0, pushed, FINGER_CLOSED, label, BUTTON_GRIP),
              _w(0.4, pushed, FINGER_CLOSED, "pressed", BUTTON_GRIP),
              _w(0.6, away, FINGER_CLOSED, "back off", BUTTON_GRIP)]
    return moves


def _right(times: dict) -> list[Waypoint]:
    """Hold the plate while the left clamps it; start the motor; feed the bit through with the handle and let it back
    up; stop the motor; go home."""
    from yam_arena.constants import READY

    press = np.array([R_PRESS_XY[0] - R_LEAN, R_PRESS_XY[1], PLATE_TOP + 0.013 - R_PRESS])
    touch = np.array([*R_PRESS_XY, PLATE_TOP + 0.013])
    push_touch = np.array([*R_PUSH_XY, PLATE_TOP + 0.013])
    push0 = push_touch + [-R_LEAN, 0.0, -R_PUSH]
    push1 = push0 + [PLATE_START + R_SLIDE_OVER[0], R_SLIDE_OVER[1], 0.0]
    via = push0 + [-0.08, -0.02, 0.07]  # in front of the drill, below the feed handle
    out, q = _joint_move("right", READY, via, PRESS_R, "toward the plate, below the feed handle", FINGER_CLOSED, 2.0, 10)
    out += [_w(1.2, push_touch + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the plate's front edge, on its centre line",
               PRESS_R),
            _w(1.0, push_touch, FINGER_CLOSED, "down onto it", PRESS_R),
            _w(0.6, push0, FINGER_CLOSED, "press it onto the board", PRESS_R),
            _w(R_SLIDE_S, push1, FINGER_CLOSED, "slide it back 20 mm: the punch mark under the bit", PRESS_R),
            _w(0.6, push1 + [0.0, 0.0, R_PUSH + 0.03], FINGER_CLOSED, "lift off: the plate stays", PRESS_R)]
    times["left_go"] = sum(w.duration for w in out) - 2.0  # the left sets off as the right lifts off
    out += [_w(1.2, touch + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the plate's front-right part", PRESS_R),
            _w(0.8, touch, FINGER_CLOSED, "down onto it", PRESS_R),
            _w(0.6, press, FINGER_CLOSED, "press it firmly onto the board: it holds the plate still", PRESS_R)]
    t = sum(w.duration for w in out)
    out.append(_w(max(times.get("clamped", 0.0) + 1.0 - t, 0.3), press, FINGER_CLOSED, "hold it while the clamp is tightened",
                  PRESS_R))
    up = press + [-0.06, -0.02, 0.08]
    out.append(_w(1.2, up, FINGER_CLOSED, "rise, back toward the arms", PRESS_R))
    t = sum(w.duration for w in out) + 2.5
    out += _press_button(BTN_START_Y, BTN_START_OUT, "press the green start button: the chuck turns",
                         max(times.get("left_home", 0.0) - t, 0.0))
    knob0 = _knob_pos(FEED_BETA[0])
    g0 = _cap_grip(FEED_BETA[0])
    out += [_w(2.5, knob0 - 0.03 * g0[:, 2], FINGER_OPEN, "beside the feed handle's front knob", g0),
            _w(1.0, knob0, FINGER_OPEN, "onto the knob", g0),
            _w(0.6, knob0, FINGER_CLOSED, "grip it", g0)]
    for (b0, b1), dur, label in (((FEED_BETA[0], FEED_BETA[1]), FEED_S[0], "pull the handle: the bit down to the plate"),
                                 ((FEED_BETA[1], FEED_BETA[2]), FEED_S[1], "feed it through the plate")):
        n = max(2, int(abs(b1 - b0) / 5.0))
        for k in range(1, n + 1):
            b = b0 + (b1 - b0) * k / n
            out.append(_w(dur / n, _knob_pos(b), FINGER_CLOSED, label, _cap_grip(b)))
    out.append(_w(0.8, _knob_pos(FEED_BETA[2]), FINGER_CLOSED, "through, into the spoil board", _cap_grip(FEED_BETA[2])))
    n = max(2, int(abs(FEED_BETA[2] - FEED_BETA[0]) / 5.0))
    for k in range(1, n + 1):
        b = FEED_BETA[2] + (FEED_BETA[0] - FEED_BETA[2]) * k / n
        out.append(_w(3.0 / n, _knob_pos(b), FINGER_CLOSED, "let the handle back up", _cap_grip(b)))
    out += [_w(0.5, knob0, FINGER_OPEN, "let go: the quill is up", g0),
            _w(0.8, knob0 - 0.03 * g0[:, 2], FINGER_OPEN, "back off", g0)]
    out += _press_button(BTN_STOP_Y, BTN_STOP_OUT, "press the red stop button: the chuck runs down")
    out.append(_w(1.5, up, FINGER_CLOSED, "down and away from the switch", PRESS_R))
    q = _q_of("right", out[-1], q)
    home = _ready_pose("right")
    moves, q = _joint_move("right", q, home[0], home[1], "home", FINGER_OPEN, 2.5, 12)
    out += moves
    return out


def demo():
    times: dict = {}
    _right(times)  # (for the time the left sets off)
    left = _left(times)
    right = _right(times)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 6.0
    for plan in (left, right):
        plan.append(_w(total - sum(w.duration for w in plan), plan[-1].pos, plan[-1].finger,
                       "hold still: the trial ends once the full score has held 1 s", plan[-1].yaw))
    return {"left": left, "right": right}
