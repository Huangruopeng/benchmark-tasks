"""The scene of clamp_a_small_part_before_drilling_it.py: an 8-inch bench drill press with its spoil board, the
aluminium plate and the screw hold-down clamp, built from bodykit shapes and joined by PhysX joints, plus what the task
file runs before every physics step to emulate what PhysX cannot do here (the rack and pinion, the clamp's thread, the
return spring and the cutting thrust) and each hand's clearance to the turning bit. The scoring is in the task file.

Distances in m. x forward from the arm bases, y to the left, z up; heights given as "above the bench" are above the
station table's top (TOP).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, as_numpy, hinge, on_reset, rotation,  # noqa: E402
                     set_hinge_drive, set_slide_target, set_slider_drive, set_thread_on, slider, thread)
from yam_arena.constants import TCP_OFFSET  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the drill press: an 8-inch (203 mm swing) bench drill press, 1/3 hp, 13 mm keyed chuck, 50 mm quill travel ---------
SPX, SPY = 0.32, 0.0  # the spindle's axis
BASE_X, BASE_Y, BASE_H = (SPX - 0.165, SPX + 0.125), (-0.095, 0.095), 0.040  # cast-iron base, 290 x 190 x 40 mm
COL_X, COL_R, COL_TOP = SPX + 0.105, 0.023, 0.470  # 46 mm steel column, 105 mm behind the spindle
TABLE_X, TABLE_Y, TABLE_Z = (SPX - 0.08, SPX + 0.08), (-0.08, 0.08), (0.145, 0.160)  # 160 x 160 mm table, top 160 mm
HEAD_X, HEAD_Y, HEAD_Z = (SPX - 0.055, COL_X + 0.035), (-0.07, 0.07), (0.335, 0.475)  # the head casting
COVER_X, COVER_Y, COVER_Z = (SPX - 0.065, SPX + 0.235), (-0.085, 0.085), (0.475, 0.555)  # the belt and pulley cover
MOTOR_XY, MOTOR_R, MOTOR_Z = (SPX + 0.20, 0.0), 0.055, (0.36, 0.475)

# -- the work: a waxed melamine-faced MDF spoil board (230 x 200 x 10 mm) on the table, the plate on it ------------------
PX, PY = (SPX - 0.150, SPX + 0.080), (-0.100, 0.100)
PT = TOP + TABLE_Z[1] + 0.010  # the board's top
PLATE_X, PLATE_Y, PLATE_T = (SPX - 0.105, SPX + 0.025), (-0.085, 0.065), 0.010  # 130 x 150 x 10 mm aluminium plate,
# where it is drilled: its punch mark under the bit
PLATE_START = 0.020  # the plate starts this far in front of that (toward the arms): its mark 20 mm in front of the bit
PLATE_MASS = 0.53
PLATE_C = np.array([(PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2])
PLATE_C0 = PLATE_C - np.array([PLATE_START, 0.0])  # where it starts
PLATE_TOP = PT + PLATE_T
MARK_IN_PLATE = np.array([SPX, SPY]) - PLATE_C  # the punch mark, from the plate's middle
# the layout: blue layout dye round the mark, two scribed lines crossing at it, a 90-degree centre-punch dimple 2.0 mm
# across (1.0 mm deep) where they cross, and a check circle scribed round it 8.0 mm across
DIMPLE_R = 0.0010
CHECK_R = 0.0040
DYE, SCRIBE, DIMPLE_DARK = (0.16, 0.32, 0.72), (0.85, 0.87, 0.90), (0.03, 0.03, 0.04)

# -- the quill, chuck and bit ------------------------------------------------------------------------------------------
CLEAR = 0.005  # the bit's tip 5 mm above the plate at rest (the table set so)
TIP0 = PLATE_TOP + CLEAR
BIT_R, BIT_L = 0.0025, 0.055  # a 5 mm HSS jobber drill (118-degree point), 55 mm out of the chuck
POINT_SLOPE = math.radians(31.0)  # its point's flanks, from the horizontal: (180 - 118) / 2
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
# tapped M8 for the clamp screw: a cast-iron star knob (40 mm across its lobes, on its own M8 x 40 steel stud) whose
# stud end carries a flat push-on rubber tip (19 mm across, 12.7 mm long), bonded so that it turns with it. Turning the
# knob clockwise (from above) drives the stud down PITCH per turn; once the tip is on the plate the rubber under the
# stud's end compresses and presses, and its friction turns the plate with it unless something holds the plate.
# Origin of the screw (knob, stud and tip, one body): the middle of the tip's underside.
CLAMP_XY = np.array([SPX - 0.068, 0.040])
TIP_R, TIP_H = 0.009525, 0.0127  # the rubber tip, 3/4 in across and 1/2 in long
STUD_R, STUD_Z = 0.004, (0.0037, 0.0437)  # the M8 x 40 stud, its end 9 mm up inside the tip
PITCH = 0.00125  # m per turn: M8 x 1.25
GAP0 = 0.00125  # the tip starts one turn (1.25 mm) above the plate
SCREW0 = np.array([*CLAMP_XY, PLATE_TOP + GAP0])
ARM_Z = (0.018, 0.030)  # the arm (12 mm thick), in the screw's starting frame (from the tip's underside)
ARM_W = 0.025
RISER_Y = (0.071, 0.096)  # the riser (25 x 25 mm), world y, 6 mm left of the plate's left edge (room to set the plate)
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
TIP_FRICTION = 0.63  # the rubber tip on the aluminium plate (it also carries the thread's own friction)
TIP_ON = 0.0005  # the tip's underside within this of the plate's top: on it

STEEL, ALU, DARK, RED, BLACK, GREEN = (0.55, 0.57, 0.60), (0.80, 0.81, 0.83), (0.20, 0.21, 0.23), \
    (0.85, 0.12, 0.10), (0.05, 0.05, 0.05), (0.10, 0.62, 0.20)
PAINT, PAINT_DARK, YELLOW, WHITE = (0.30, 0.42, 0.55), (0.22, 0.31, 0.41), (0.95, 0.78, 0.10), (0.92, 0.92, 0.90)
BOARD = (0.86, 0.85, 0.80)  # white melamine
ALU_DARK, IRON = (0.66, 0.67, 0.70), (0.24, 0.24, 0.25)  # aluminium bar stock; the cast-iron star knob
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")  # the wrist bodies, the gripper and its fingers
FLING = (0.6, 25.0)  # an unclamped plate the bit catches: flung at this speed (m/s) and spin (rad/s)


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


def spoke_dir(beta_deg: float) -> np.ndarray:
    """A spoke's direction in the x-z plane, ``beta_deg`` from straight down (- toward the front)."""
    b = math.radians(beta_deg)
    return np.array([math.sin(b), 0.0, -math.cos(b)])


def _board() -> Body:
    shapes = [BoxShape(((PX[0] + PX[1]) / 2, (PY[0] + PY[1]) / 2, 0.005), (PX[1] - PX[0], PY[1] - PY[0], 0.010), BOARD)]
    return Body.build("board", pos=(0.0, 0.0, PT - 0.010), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.05, 0.05), slick=True, color=BOARD)


def _plate() -> Body:
    """The plate with its layout: a patch of blue layout dye, two scribed lines crossing at the punch mark, the
    centre-punch dimple (dark: its cone in shadow) and the 8 mm check circle (all visual; the dimple's effect on the
    bit is the task file's seat rule)."""
    mx, my = MARK_IN_PLATE
    z = PLATE_T / 2
    shapes = [BoxShape((0.0, 0.0, 0.0), (PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_T), ALU),
              BoxShape((mx, my, z + 0.0001), (0.034, 0.034, 0.0002), DYE, collide=False),
              BoxShape((mx, my, z + 0.00025), (0.030, 0.0006, 0.0002), SCRIBE, collide=False),
              BoxShape((mx, my, z + 0.00025), (0.0006, 0.030, 0.0002), SCRIBE, collide=False),
              CylShape((mx, my, z + 0.0004), DIMPLE_R, 0.0002, DIMPLE_DARK, collide=False)]
    for k in range(24):  # the check circle
        a = 2 * math.pi * (k + 0.5) / 24
        shapes.append(BoxShape((mx + CHECK_R * math.cos(a), my + CHECK_R * math.sin(a), z + 0.00025),
                               (0.0005, 2 * math.pi * CHECK_R / 24 + 0.0002, 0.0002), SCRIBE, yaw=a, collide=False))
    return Body.build("plate", pos=(*PLATE_C0, PT + PLATE_T / 2), mass=PLATE_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=ALU)


def _clamp() -> Body:
    """The clamp's riser and arm (fixed: the riser is bolted through the spoil board, the arm to the riser)."""
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
    """The drill press's fixed parts. Origin at the spindle's axis on the bench."""
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
              box((SW_X[0] - 0.0005, SW_X[0]), (SW_Y[0] + 0.006, SW_Y[1] - 0.006),
                  (SW_Z[1] - TOP - 0.012, SW_Z[1] - TOP - 0.004), YELLOW, collide=False),  # its label strip
              box((HEAD_X[0] - 0.0005, HEAD_X[0]), (0.010, 0.050), (0.395, 0.407), WHITE, collide=False)]  # a plate
    return Body.build("drill", pos=(SPX, SPY, TOP), mass=20.0, shapes=shapes, kinematic=True, color=PAINT)


def _quill() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.050), QUILL_R, 0.100, STEEL),
              CylShape((0.0, 0.0, -NOSE_L / 2), 0.015, NOSE_L, DARK)]
    return Body.build("quill", pos=(SPX, SPY, QUILL_Z0), mass=QUILL_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=STEEL, com=(0.0, 0.0, 0.03), solver_iterations=32)


def _chuck() -> Body:
    """The chuck and the bit, turning with the spindle. Origin at the bit's tip. The bit has no collider: drilling and
    the bit's point on the plate are the task file's rules."""
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
    shapes = [CylShape((0.0, 0.0, 0.0), 0.022, 0.016, PAINT_DARK, axis="Y"),
              CylShape((0.0, 0.016, 0.0), 0.014, 0.018, STEEL, axis="Y")]
    for b in SPOKES0:
        d = spoke_dir(b)
        shapes += _cyl_between(d * 0.020, d * (SPOKE_L - KNOB_R + 0.002), SPOKE_R, STEEL)
        shapes.append(SphereShape(tuple((d * SPOKE_L).tolist()), KNOB_R, BLACK))
    return Body.build("capstan", pos=tuple(HUB.tolist()), mass=CAPSTAN_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=STEEL, com=(0.0, 0.0, 0.0), solver_iterations=32)


def _button(name: str, y: float, r: float, out: float, color) -> Body:
    shapes = [CylShape((-out / 2, 0.0, 0.0), r, out, color, axis="X"),
              CylShape((0.004, 0.0, 0.0), r * 0.8, 0.008, color, axis="X", collide=False)]
    return Body.build(name, pos=(SW_X[0], y, BTN_Z), mass=0.01, shapes=shapes, friction=(1.0, 0.9), color=color)


HIDE_HOLE = np.array([SPX, 0.30, TOP - 0.02])  # the hole's marker waits under the table until a hole is drilled
OBJECTS = [_board(), _plate(), _clamp(), _screw(), _drill(), _quill(), _chuck(), _capstan(),
           _button("btn_start", BTN_START_Y, BTN_START_R, BTN_START_OUT, GREEN),
           _button("btn_stop", BTN_STOP_Y, BTN_STOP_R, BTN_STOP_OUT, RED),
           Body.build("hole", pos=tuple(HIDE_HOLE.tolist()), mass=0.01, kinematic=True, color=BLACK,
                      shapes=[CylShape((0.0, 0.0, 0.0), BIT_R, 0.0008, BLACK, collide=False)])]

# -- joints -------------------------------------------------------------------------------------------------------------
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
# shared gear position, re-aimed before every physics step (see _gear).
_DEG = math.pi / 180.0
slider("feed", "drill", "quill", anchor=(SPX, SPY, QUILL_Z0), axis="Z", limits=(-DEPTH_STOP, 0.0), stiffness=GEAR_K,
       damping=GEAR_D, target=SPRING_F0 / GEAR_K, max_force=DRIVE_MAX)
hinge("pinion", "drill", "capstan", anchor=tuple(HUB.tolist()), axis="Y",
      limits_deg=(-math.degrees(DEPTH_STOP / PINION_R) - 3.0, 3.0), spring=GEAR_K * PINION_R ** 2 * _DEG,
      damping=GEAR_D * PINION_R ** 2 * _DEG, target_deg=0.0)
hinge("spin", "quill", "chuck", anchor=(SPX, SPY, TIP0 + BIT_L + CHUCK_L), axis="Z",
      damping=CHUCK_I / SPIN_TAU_OFF * _DEG)  # the spindle: a velocity drive the switch turns on and off (motor())
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

# -- each hand's clearance to the turning bit and chuck --------------------------------------------------------------

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

    for link in ET.parse(YAM_URDF).getroot().findall("link"):
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


def clearance(base, origin, chuck_pos) -> list[float]:
    """Each arm's closest distance (m) from any point of its hand links to the turning parts: the bit (from its tip
    up) and the chuck with the spindle's nose."""
    pts = _hand_points()
    axis, tip = chuck_pos[:2], chuck_pos[2]
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


# -- run before every physics step (1/240 s): the rack and pinion, the return spring, the cutting thrust, the thread -----

def fresh() -> dict:
    return {"phi": None, "turns": 0.0, "target": 0.0, "press": 0.0, "press_max": 0.0, "tcps": None, "unsafe": False,
            "caught": False, "hole": 0.0, "hole_xy": None, "hole_ok": False, "motor": False, "cutting": None,
            "clear": [1.0, 1.0], "clear_min": [1.0, 1.0], "omega": 0.0, "st": {}, "gear_c": 0.0, "seat_n": 0.0,
            "seat_max": 0.0, "located": False, "anchor": None}


EP = fresh()  # what the episode has done so far (the latches the score reads)
PRE_STEP: list = []  # further functions of ``base`` run before every physics step (the task file adds the seat rule)
_ATTRS: dict = {}


def _spin_attrs():
    if "spin" not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS["spin"]), "angular")
        _ATTRS["spin"] = (drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr(0.0), drive.GetDampingAttr())
    return _ATTRS["spin"]


def motor(on: bool) -> None:
    """The motor drives the spindle at its speed, or (off) lets the chuck run down."""
    vel, damp = _spin_attrs()
    vel.Set(float(-SPIN_RPM * 6.0) if on else 0.0)  # deg/s, clockwise seen from above
    damp.Set(float(CHUCK_I / (SPIN_TAU_ON if on else SPIN_TAU_OFF) * _DEG))
    EP["motor"] = on


def pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def feed_state(base, origin) -> tuple[float, float]:
    """How far the quill is down (m) and how far the feed handle has turned, as quill travel (m)."""
    q, _ = pose(base, origin, "quill")
    _, rc = pose(base, origin, "capstan")
    turn = math.atan2(float(rc[0, 2]), float(rc[0, 0]))  # about +y
    return float(QUILL_Z0 - q[2]), -PINION_R * turn


def _gear(base) -> None:
    """The rack and pinion: aim the quill's and the feed handle's drives at the mean of where each says the gear is, so
    a torque on the handle reaches the quill as that torque over the pinion's radius, and the quill's loads come back
    to the handle. The quill's loads: the return spring; the bit's point resting on the plate (EP["seat_n"], the task
    file's seat rule, motor off); while the turning bit cuts at the bottom of its hole, the cutting thrust (damping)."""
    origin = as_numpy(base.scene.env_origins[0])
    s_q, s_c = feed_state(base, origin)
    c = (s_q + s_c) / 2.0
    EP["gear_c"] = c
    EP["knob_n"] = GEAR_K * PINION_R * (s_c - c) / SPOKE_L  # the push on a knob the mesh is passing on (N)
    EP["knob_max"] = max(EP.get("knob_max", 0.0), EP["knob_n"])
    damp = GEAR_D
    cut = EP.get("cutting")
    if cut is not None and EP["omega"] > TURNING:
        depth = s_q - cut[0]  # how far the tip is below the plate's top
        if depth > EP["hole"] - 0.0003 and cut[1]:
            damp += CUT_D[0] if depth < PLATE_T else CUT_D[1]
    load = SPRING_F0 + SPRING_K * max(c, 0.0) + EP["seat_n"]
    set_slider_drive("feed", target=-c + load / GEAR_K, damping=damp)
    set_hinge_drive("pinion", target_deg=math.degrees(-c / PINION_R))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp_step(base) -> None:
    """The clamp's thread: the stud comes down PITCH per clockwise turn. Its slide drive is the stiffness of the rubber
    under the stud's end, so once the tip is on the plate each further turn presses harder."""
    origin = as_numpy(base.scene.env_origins[0])
    s_pos, s_rot = pose(base, origin, "screw")
    phi = math.atan2(float(s_rot[1, 0]), float(s_rot[0, 0]))
    if EP["phi"] is not None:
        EP["turns"] -= _wrap(phi - EP["phi"]) / (2 * math.pi)  # clockwise (from above) is +
    EP["phi"] = phi
    target = min(max(PITCH * EP["turns"], 0.0), SCREW_TRAVEL)
    set_slide_target("clamp_screw", target)
    EP["target"] = target
    slide = float(SCREW0[2] - s_pos[2])  # how far the stud has come down
    EP["press"] = min(SCREW_K * max(target - slide, 0.0), SCREW_MAX)  # the tip pressing on what is under it
    EP["press_max"] = max(EP["press_max"], EP["press"])


def hook_physics(base) -> None:
    """Run PRE_STEP, the gear and the thread before every physics step (wrapping the scene's write to the sim)."""
    scene = base.scene
    if getattr(scene, "_drill_gear", False):
        return
    plain = scene.write_data_to_sim

    def write_data_to_sim(*args, **kwargs):
        for fn in PRE_STEP:
            fn(base)
        _gear(base)
        _clamp_step(base)
        return plain(*args, **kwargs)

    scene.write_data_to_sim = write_data_to_sim
    scene._drill_gear = True


@on_reset
def _new_episode() -> None:
    EP.clear()
    EP.update(fresh())
    set_thread_on("clamp_screw", True)
    set_slide_target("clamp_screw", 0.0)
    try:
        motor(False)
        set_slider_drive("feed", target=SPRING_F0 / GEAR_K, damping=GEAR_D)
        set_hinge_drive("pinion", target_deg=0.0)
    except KeyError:
        pass


def tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def teleport(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def fling(base) -> None:
    """The turning bit catches a loose plate: it is flung, spinning."""
    import torch

    asset = base.scene["plate"]
    v = np.array([-FLING[0], 0.0, 0.05])
    asset.write_root_velocity_to_sim(torch.tensor([[*v.tolist(), 0.0, 0.0, FLING[1]]], dtype=torch.float32,
                                                  device=asset.device))


def plate_force(base, origin, force, at) -> None:
    """Apply ``force`` (world, N) to the plate at the point ``at`` (world, without the env origin) this physics step."""
    import torch

    asset = base.scene["plate"]
    f = torch.tensor(np.asarray(force, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    p = torch.tensor(np.asarray(at, float) + origin, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f, positions=p, is_global=True)


def button_in(base, origin, name) -> float:
    p, _ = pose(base, origin, name)
    return float(p[0] - SW_X[0])
