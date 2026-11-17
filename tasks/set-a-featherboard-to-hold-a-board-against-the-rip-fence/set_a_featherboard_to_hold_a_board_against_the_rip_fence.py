"""A contractor table saw, set up for a rip: its blade wound fully down, its cord unplugged, its rip fence locked
100 mm from the blade. A 1x6 pine board lies on the cast-iron top beside the fence, 15 mm off it. A mitre-slot
featherboard (orange polymer body with a row of flexible fingers and a short setup feather, on an aluminium
expanding bar in the left mitre slot, held by two T-knobs) sits loose in the slot, slid up beside the blade.

Task: set the featherboard so it holds the board against the fence for the rip, and finish with both arms back where
they started.

What the physics asks for: the featherboard's fingers are springs (3,500 N/m). Set by the maker's method, the short
setup feather just touches the board's edge and the fingers are flexed 3 mm (about 10.5 N). The setup feather is a
finger too (300 N/m): pressed harder, it bends with the rest, so the right preload is a window (fingers flexed 2 to 4
mm, about 7 to 14 N), not a hard stop. While its knobs are loose the featherboard slides on its bar and in the slot
against only about 1.5 N, so the fingers push it straight back off the board unless it is held there while a knob is
tightened. A knob clamps once it is turned tight (75 degrees from loose, against 0.3 N m); the rear knob also spreads
the bar in the slot.

Run it:  inspect-robots run --embodiment yam_arena -E task=set_a_featherboard_to_hold_a_board_against_the_rip_fence.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     plate, rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the saw: a contractor saw's cast-iron top, 27 in front to back, 85 cm above the floor ------------------------------
H_SAW = 0.090  # the top above the station table (the table is 76 cm above the floor)
ST = TOP + H_SAW  # the saw top's surface
TOP_T = 0.032  # the cast top's edge thickness
SX0, SX1, SY0, SY1 = 0.100, 0.786, -0.500, 0.340
X_BLADE, Y_BLADE = 0.420, -0.100  # where the blade comes up through the throat plate (its front edge), its plane
SLOT_X1 = X_BLADE + 0.170  # the throat plate's blade slot runs from X_BLADE to here
FENCE_Y, FENCE_W, FENCE_H = -0.200, 0.060, 0.064  # the rip fence's face (100 mm from the blade), its width, height
SLOT_Y, SLOT_W, SLOT_D = 0.025, 0.01905, 0.0095  # the left mitre slot, 3/4 x 3/8 in, 125 mm left of the blade
# -- the board: a 1x6 pine board, 18 in long ---------------------------------------------------------------------------
BOARD_L, BOARD_W, BOARD_T, BOARD_MASS = 0.457, 0.140, 0.019, 0.55
BOARD_X1 = X_BLADE - 0.025  # its front end, 1 in in front of the blade (set up for the rip)
BOARD_GAP0 = 0.015  # it starts this far off the fence
BOARD0 = np.array([BOARD_X1 - BOARD_L / 2, FENCE_Y + BOARD_GAP0 + BOARD_W / 2, ST + BOARD_T / 2])
EDGE_SET = FENCE_Y + BOARD_W  # the board's left edge once it is against the fence
# -- the featherboard, in its own frame: origin on the rear knob's axis, on the saw top, x along the slot (feed), y away
#    from the board. The fingers point -y and lean 30 degrees toward the blade (+x) -----------------------------------
FB_H, FB_FLOOR, FB_WALL = 0.017, 0.004, 0.006  # rim height, floor and wall thickness
FB_HEEL_X, FB_BACK_X1, FB_FRONT_X = -0.035, 0.200, 0.240  # heel end; back edge's front end; front corner at the roots
FB_BACK_Y, FB_ROOT_Y, FB_TIP_Y = 0.060, -0.036, -0.085  # back edge; finger-root wall's outer face; finger tips at rest
LEAN = math.radians(30.0)
N_FINGERS, PITCH, F_W, F_H = 14, 0.0085, 0.0045, 0.013
TIP_X = [0.205 - (N_FINGERS - i) * PITCH for i in range(N_FINGERS + 1)]  # finger tips along x; [0] is the setup feather
LEAD_X = TIP_X[-1] + F_W / 2  # the leading finger tip's front edge
D_SET = 0.003  # the setup feather is 3 mm shorter: just touching, the fingers are flexed this much
K_SETF, D_SETF = 300.0, 1.5  # the setup feather flexes like one finger of the row (a little stiffer: it is shorter)
K2_X = 0.105  # the front knob, along the bar from the rear one
SLOT_LO, SLOT_HI = -0.027, 0.012  # the body's knob slots run along y between these (body frame)
BAR_X0, BAR_L, BAR_W, BAR_T = -0.015, 0.1905, 0.0188, 0.0089  # the expanding bar (7-1/2 in), its split end at the rear
BAR_MASS, BODY_MASS, FINGERS_MASS, SETF_MASS, KNOB_MASS = 0.068, 0.160, 0.030, 0.010, 0.025
K_FINGERS, D_FINGERS = 3500.0, 8.0  # the fingers' stiffness: 10.5 N at the setup feather's 3 mm
# where it is set: the leading finger 4.5 cm in front of the blade, the fingers on the board's edge
SET_K1 = np.array([X_BLADE - 0.045 - LEAD_X, SLOT_Y, ST])
START_DX, START_DY = 0.095, 0.025  # it starts slid up beside the blade and pulled back on its bar
K1_0 = SET_K1 + [START_DX, 0.0, 0.0]  # the rear knob's axis at the start
FB0 = np.array([K1_0[0], EDGE_SET - FB_TIP_Y + START_DY, ST])  # the body's origin: tips 2.5 cm off the set board edge
assert abs((EDGE_SET - FB_TIP_Y) - SLOT_Y) < 1e-9  # set, the knob axis sits on the body's y = 0
K2_0 = K1_0 + [K2_X, 0.0, 0.0]
KNOB_W1, KNOB_W2 = math.radians(55.0), 0.0  # the T-knobs' wings at the start (loose)
# -- the rules ---------------------------------------------------------------------------------------------------------
F_LOOSE, F_LOCK, K_HOLD, D_HOLD = 1.5, 300.0, 5.0e4, 20.0  # loose: slides against 1.5 N; clamped: held
SLACK = 0.0002
T_LOOSE, T_TIGHT, K_KNOB, D_KNOB = 0.03, 0.30, 0.10, 0.002  # N m: a loose knob spins freely, a snug one takes 0.3
SNUG_DEG, TIGHT_DEG, LOOSEN_DEG, STOP_DEG = 40.0, 75.0, 70.0, 100.0
LEAD_WINDOW = (0.010, 0.075)  # the leading finger tip this far in front of the blade
TOUCH = 0.0002  # fingers flexed at least this much: on the board's edge
SET_WINDOW = (D_SET - 0.001, D_SET + 0.001)  # the setup feather within 1 mm of the board's edge, or bent at most 1 mm
FENCE_GAP = 0.001  # the board's fence-side edge within 1 mm of the fence at both ends
BOARD_MOVE = 0.050  # the board stays within 5 cm (along the fence) of where it lay
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

IRON, IRON_D, IRON_E = (0.47, 0.48, 0.50), (0.16, 0.16, 0.17), (0.36, 0.37, 0.39)
BLACK, STEEL, ALU = (0.07, 0.07, 0.08), (0.75, 0.76, 0.78), (0.80, 0.81, 0.83)
LAMINATE, CABINET, RAIL = (0.93, 0.92, 0.88), (0.30, 0.31, 0.34), (0.18, 0.18, 0.20)
TAPE, YELLOW, RED = (0.95, 0.90, 0.62), (0.95, 0.78, 0.10), (0.82, 0.10, 0.08)
PINE, GRAIN, KNOT, END = (0.88, 0.74, 0.52), (0.76, 0.58, 0.37), (0.55, 0.37, 0.20), (0.80, 0.64, 0.43)
ORANGE, ORANGE_D, WHITE = (0.98, 0.45, 0.06), (0.88, 0.38, 0.04), (0.95, 0.95, 0.94)


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


# -- the saw ------------------------------------------------------------------------------------------------------------
SAW_O = np.array([(SX0 + SX1) / 2, (SY0 + SY1) / 2, ST])  # the saw body's origin: middle of its top


def _saw() -> Body:
    """The cast-iron top with its left mitre slot, the throat plate over the lowered blade, the locked rip fence on its
    front rail, the stand below, the off switch and the unplugged cord. Only the top and the fence collide."""
    o = SAW_O
    w = lambda *p: np.asarray(p, float) - o  # noqa: E731  (world point -> body frame)
    s0, s1 = SLOT_Y - SLOT_W / 2, SLOT_Y + SLOT_W / 2
    shapes = [_box(w(SX0, SY0, ST - TOP_T), w(SX1, s0, ST), IRON),  # the top, either side of the mitre slot
              _box(w(SX0, s1, ST - TOP_T), w(SX1, SY1, ST), IRON),
              _box(w(SX0, s0, ST - TOP_T), w(SX1, s1, ST - SLOT_D), IRON_D)]  # the slot's floor
    for y in (s0, s1):  # the slot's walls, a shade darker
        shapes.append(_box(w(SX0, y - 0.0004, ST - SLOT_D), w(SX1, y + 0.0004, ST + 0.0001), IRON_E, False))
    for x in (SX0, SX1):  # the cast top's ground edges
        shapes.append(_box(w(x - 0.0006, SY0, ST - TOP_T), w(x + 0.0006, SY1, ST - 0.001), IRON_E, False))
    # the aluminium throat plate with its blade slot, flush with the top: the blade is wound down below it
    shapes.append(_box(w(X_BLADE - 0.080, Y_BLADE - 0.046, ST - 0.002), w(X_BLADE + 0.270, Y_BLADE + 0.046, ST + 0.0003),
                       (0.70, 0.71, 0.73), False))
    shapes.append(_box(w(X_BLADE, Y_BLADE - 0.005, ST - 0.002), w(SLOT_X1, Y_BLADE + 0.005, ST + 0.0006),
                       (0.015, 0.015, 0.015), False))
    for dx in (-0.065, 0.255):  # its levelling screws
        shapes.append(CylShape(tuple(w(X_BLADE + dx, Y_BLADE, ST + 0.0005).tolist()), 0.004, 0.0006, STEEL,
                               collide=False))
    # the rip fence, locked: a black steel tube with laminate faces, its head clamped on the front rail
    fx0, fx1 = SX0 - 0.002, SX1 + 0.010
    shapes.append(_box(w(fx0, FENCE_Y - FENCE_W, ST), w(fx1, FENCE_Y, ST + FENCE_H), BLACK))
    for y in (FENCE_Y, FENCE_Y - FENCE_W):
        sgn = 1.0 if y == FENCE_Y else -1.0
        shapes.append(_box(w(fx0, y - 0.0005 * sgn, ST + 0.002), w(fx1, y + 0.0035 * sgn, ST + FENCE_H - 0.002),
                           LAMINATE, False))
    head0 = FENCE_Y - FENCE_W - 0.008
    shapes.append(_box(w(0.066, head0, ST - 0.068), w(SX0 + 0.003, FENCE_Y + 0.008, ST + FENCE_H + 0.006), BLACK,
                       False))
    shapes.append(_box(w(0.058, FENCE_Y - 0.040, ST - 0.050), w(0.066, FENCE_Y - 0.020, ST + 0.010), RED, False))
    # the front rail with its tape scale, and the stand under the top
    shapes.append(_box(w(0.072, SY0 - 0.06, ST - 0.075), w(SX0, SY1, ST - 0.018), RAIL, False))
    shapes.append(_box(w(0.0715, SY0 - 0.06, ST - 0.030), w(0.0725, SY1, ST - 0.019), TAPE, False))
    for k in range(int((SY1 - SY0 + 0.06) / 0.0254)):
        y = SY0 - 0.06 + k * 0.0254
        shapes.append(_box(w(0.0710, y - 0.0005, ST - 0.029 + (0.0 if k % 4 else -0.004)), w(0.0716, y + 0.0005,
                           ST - 0.020), BLACK, False))
    shapes.append(_box(w(0.130, -0.400, TOP), w(0.760, 0.260, ST - TOP_T), CABINET, False))
    # the paddle switch on the rail, off, and the saw's cord, unplugged, lying on the bench
    shapes.append(_box(w(0.044, 0.170, ST - 0.072), w(0.070, 0.235, ST - 0.020), YELLOW, False))
    shapes.append(_box(w(0.038, 0.185, ST - 0.064), w(0.045, 0.220, ST - 0.028), RED, False))
    cord = [(0.42, 0.260, TOP + 0.030), (0.40, 0.300, TOP + 0.006), (0.33, 0.370, TOP + 0.006),
            (0.25, 0.385, TOP + 0.006), (0.19, 0.372, TOP + 0.006)]
    for a, b in zip(cord[:-1], cord[1:]):
        shapes.append(bodykit.segment(w(*a), w(*b), 0.009, BLACK, collide=False))
    pc = np.array(cord[-1])
    shapes.append(_box(w(*(pc + [-0.040, -0.011, -0.003])), w(*(pc + [0.0, 0.011, 0.016])), BLACK, False))
    for dy in (-0.006, 0.006):
        shapes.append(_box(w(*(pc + [-0.058, dy - 0.001, 0.004])), w(*(pc + [-0.040, dy + 0.001, 0.009])), STEEL,
                           False))
    return Body.build("saw", pos=tuple(o.tolist()), mass=50.0, kinematic=True, color=IRON, shapes=shapes,
                      friction=(0.30, 0.25))


# -- the board ----------------------------------------------------------------------------------------------------------

def _board() -> Body:
    L, W, T = BOARD_L, BOARD_W, BOARD_T
    shapes = [BoxShape((0.0, 0.0, 0.0), (L, W, T), PINE)]
    rng = np.random.default_rng(7)
    for _ in range(9):  # flat-sawn grain on the face
        y = rng.uniform(-W / 2 + 0.006, W / 2 - 0.006)
        x0, x1 = rng.uniform(-L / 2, -L / 2 + 0.12), rng.uniform(L / 2 - 0.12, L / 2)
        shapes.append(_box((x0, y - 0.0007, T / 2), (x1, y + 0.0007, T / 2 + 0.0003), GRAIN, False))
    for x, y, r in ((-0.11, 0.030, 0.007), (0.09, -0.035, 0.005)):
        shapes.append(CylShape((x, y, T / 2 + 0.0002), r, 0.0004, KNOT, collide=False))
    for x in (-L / 2, L / 2):  # end grain
        shapes.append(_box((x - 0.0006, -W / 2, -T / 2), (x + 0.0006, W / 2, T / 2), END, False))
    return Body.build("board", pos=tuple(BOARD0.tolist()), mass=BOARD_MASS, shapes=shapes, color=PINE,
                      friction=(0.35, 0.30), solver_iterations=24)


# -- the featherboard ---------------------------------------------------------------------------------------------------

def _bar() -> Body:
    """The aluminium expanding mitre bar in the slot, under the body: a long slot for the front knob's bolt, a split end
    at the rear spread by the rear knob's flat-head bolt. Origin on the rear knob's axis, on the saw top."""
    z0, z1 = -BAR_T - 0.0004, -0.0004
    shapes = [_box((BAR_X0, -BAR_W / 2, z0), (BAR_X0 + BAR_L, BAR_W / 2, z1), ALU)]
    shapes.append(_box((0.035, -0.0033, z1 - 0.0004), (0.165, 0.0033, z1 + 0.0002), (0.25, 0.25, 0.27), False))
    shapes.append(_box((BAR_X0 - 0.0005, -0.0004, z0), (0.012, 0.0004, z1 + 0.0002), (0.20, 0.20, 0.22), False))
    return Body.build("bar", pos=tuple(K1_0.tolist()), mass=BAR_MASS, shapes=shapes, color=ALU, friction=(0.3, 0.25),
                      solver_iterations=32, ignore=("saw", "board"))


def _feather() -> Body:
    """The featherboard's orange polymer body: a ribbed frame with a raised rim, two knob slots along y, and the
    maker's logo and feed arrow."""
    h, f, t = FB_H, FB_FLOOR, FB_WALL
    hx, bx1, fx, by, ry = FB_HEEL_X, FB_BACK_X1, FB_FRONT_X, FB_BACK_Y, FB_ROOT_Y
    z0 = 0.0005
    shapes = [_box((hx, ry, z0), (bx1, by, z0 + f), ORANGE_D),  # the floor
              _box((hx, by - t, z0), (bx1, by, h), ORANGE),  # the back edge
              _box((hx, ry, z0), (hx + t, by, h), ORANGE),  # the heel
              _box((hx, ry, z0), (fx, ry + t, h), ORANGE)]  # the finger-root wall
    # the front edge, slanting from the back edge's end to the front corner, and the floor under it
    shapes.append(plate((bx1, by - t / 2, (z0 + h) / 2), (fx - t / 2, ry + t / 2, (z0 + h) / 2), h - z0, t,
                        (1.0, (fx - bx1) / (by - ry), 0.0), ORANGE))
    for k in range(4):
        y0 = ry + k * (by - ry) / 4
        x1 = bx1 + (fx - bx1) * (1.0 - (k + 0.5) / 4)
        shapes.append(_box((bx1 - 0.001, y0, z0), (x1, y0 + (by - ry) / 4, z0 + f), ORANGE_D))
    for kx in (0.0, K2_X):  # the knob slots: a channel along y on each side of the bolt
        for s in (-1.0, 1.0):
            shapes.append(_box((kx + s * 0.0035 - (0.006 if s < 0 else 0.0), SLOT_LO - 0.004, z0),
                               (kx + s * 0.0035 + (0.006 if s > 0 else 0.0), SLOT_HI + 0.004, h), ORANGE))
        shapes.append(_box((kx - 0.0035, SLOT_LO, z0 + f), (kx + 0.0035, SLOT_HI, z0 + f + 0.0003), (0.25, 0.12, 0.03),
                           False))
    shapes.append(_box((0.020, ry, z0), (0.026, by, h - 0.004), ORANGE))  # cross ribs
    shapes.append(_box((0.150, ry, z0), (0.156, by, h - 0.004), ORANGE))
    shapes.append(CylShape((0.060, 0.022, z0 + f + 0.0003), 0.024, 0.0005, BLACK, collide=False))  # the logo
    shapes.append(CylShape((0.060, 0.022, z0 + f + 0.0006), 0.020, 0.0005, WHITE, collide=False))
    shapes.append(CylShape((0.060, 0.022, z0 + f + 0.0009), 0.008, 0.0005, (0.35, 0.35, 0.38), collide=False))
    shapes.append(_box((0.090, -0.020, z0 + f), (0.135, -0.016, z0 + f + 0.0005), WHITE, False))  # FEED ->
    for k in range(5):
        shapes.append(_box((0.135 + k * 0.002, -0.022 + k * 0.0012, z0 + f), (0.137 + k * 0.002, -0.014 - k * 0.0012,
                            z0 + f + 0.0005), WHITE, False))
    return Body.build("feather", pos=tuple(FB0.tolist()), mass=BODY_MASS, shapes=shapes, color=ORANGE,
                      friction=(0.5, 0.4), solver_iterations=32, ignore=("saw", "bar"))


def _fingers() -> Body:
    """The row of 14 flexible fingers, leaning toward the blade: one body sprung on the featherboard (it flexes as a
    whole). They collide along their tips."""
    shapes = []
    z = 0.0005 + F_H / 2
    for x in TIP_X[1:]:
        tip = np.array([x, FB_TIP_Y])
        root = np.array([x - (FB_ROOT_Y - FB_TIP_Y) * math.tan(LEAN), FB_ROOT_Y + 0.003])
        shapes.append(plate((root[0], root[1], z), (tip[0], tip[1] + F_W / 2 * math.sin(LEAN), z), F_H, F_W,
                            (1.0, math.tan(LEAN), 0.0), ORANGE, collide=False))
    shapes.append(_box((TIP_X[1] - 0.004, FB_TIP_Y, 0.002), (LEAD_X, FB_TIP_Y + 0.008, 0.0005 + F_H - 0.001), None,
                       True))  # the tips' contact line
    return Body.build("fingers", pos=tuple(FB0.tolist()), mass=FINGERS_MASS, shapes=shapes, color=ORANGE,
                      friction=(0.4, 0.35), solver_iterations=32, ignore=("saw", "bar", "setfeather"),
                      damping=(0.0, 0.1))


def _setfeather() -> Body:
    """The setup feather: a short finger at the infeed end of the row, D_SET shy of the fingers' tips, sprung on the
    featherboard on its own (it bends once pressed past just touching). It collides along its tip."""
    z0 = 0.0005
    tip = np.array([TIP_X[0], FB_TIP_Y + D_SET])
    root = np.array([tip[0] - (FB_ROOT_Y - tip[1]) * math.tan(LEAN), FB_ROOT_Y])
    shapes = [plate((root[0], root[1] + 0.003, z0 + F_H / 2), (tip[0], tip[1], z0 + F_H / 2), F_H, F_W + 0.001,
                    (1.0, math.tan(LEAN), 0.0), ORANGE, collide=False),
              _box((tip[0] - 0.004, tip[1], 0.002), (tip[0] + 0.004, tip[1] + 0.008, 0.002 + F_H - 0.002), None, True)]
    return Body.build("setfeather", pos=tuple(FB0.tolist()), mass=SETF_MASS, shapes=shapes, color=ORANGE,
                      friction=(0.4, 0.35), solver_iterations=32, ignore=("saw", "bar", "fingers"),
                      damping=(0.0, 0.1))


def _knob(name: str, pos, wing: float) -> Body:
    """A black T-knob on a 1/4-20 bolt into the bar: a hub and a 58 mm wing, a steel insert on top. Origin on its axis,
    on the body's rim."""
    shapes = [CylShape((0.0, 0.0, 0.005), 0.008, 0.010, BLACK),
              BoxShape((0.0, 0.0, 0.019), (0.058, 0.014, 0.018), BLACK, yaw=wing)]
    for s in (-1.0, 1.0):
        shapes.append(CylShape((s * 0.029 * math.cos(wing), s * 0.029 * math.sin(wing), 0.019), 0.007, 0.018, BLACK,
                               collide=False))
    shapes.append(CylShape((0.0, 0.0, 0.0283), 0.0045, 0.0008, STEEL, collide=False))
    shapes.append(CylShape((0.0, 0.0, 0.0006), 0.010, 0.0012, STEEL, collide=False))  # the washer
    shapes.append(CylShape((0.0, 0.0, -0.009), 0.0032, 0.016, STEEL, collide=False))  # the bolt, down into the bar
    return Body.build(name, pos=tuple(np.asarray(pos, float).tolist()), mass=KNOB_MASS, shapes=shapes, color=BLACK,
                      friction=(1.0, 0.9), solver_iterations=32, ignore=("saw", "board", "feather", "fingers", "setfeather"),
                      damping=(0.0, 0.2))


KNOB1_0, KNOB2_0 = K1_0 + [0.0, 0.0, FB_H], K2_0 + [0.0, 0.0, FB_H]

# the bar in the mitre slot: slides along it (x) against F_LOOSE until the rear knob spreads it
slider("slot", "saw", "bar", anchor=tuple(K1_0.tolist()), axis="X", limits=(SX0 + 0.005 - (K1_0[0] + BAR_X0), 0.25),
       stiffness=K_HOLD, damping=D_HOLD, target=0.0, max_force=F_LOOSE)
# the body on the bar: slides across it (y, along its knob slots) against F_LOOSE until a knob clamps it
slider("on_bar", "bar", "feather", anchor=tuple(K1_0.tolist()), axis="Y",
       limits=(-(START_DY + 0.010), 0.0005), stiffness=K_HOLD, damping=D_HOLD, target=0.0, max_force=F_LOOSE)
# the fingers flex: a spring pushing their tips out (-y) to their rest line
slider("flex", "feather", "fingers", anchor=tuple(FB0.tolist()), axis="Y", limits=(-0.0003, 0.008),
       stiffness=K_FINGERS, damping=D_FINGERS, target=0.0, max_force=200.0)
# the setup feather flexes too, on its own, once pressed past just touching
slider("setflex", "feather", "setfeather", anchor=tuple(FB0.tolist()), axis="Y", limits=(-0.0003, 0.008),
       stiffness=K_SETF, damping=D_SETF, target=0.0, max_force=50.0)
# the T-knobs turn on their bolts (clockwise from above, negative, tightens), stick-slip against T_LOOSE or T_TIGHT
hinge("k1", "bar", "knob1", anchor=tuple(KNOB1_0.tolist()), axis="Z", limits_deg=(-STOP_DEG, 3.0), spring=K_KNOB,
      damping=D_KNOB, target_deg=0.0)
hinge("k2", "bar", "knob2", anchor=tuple(KNOB2_0.tolist()), axis="Z", limits_deg=(-STOP_DEG, 3.0), spring=K_KNOB,
      damping=D_KNOB, target_deg=0.0)

_author_hinges_plain = bodykit._author_hinges


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, then: each knob's drive is capped (a loose knob's thread friction)."""
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name in ("knob1", "knob2"):
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/{'k1' if spec.name == 'knob1' else 'k2'}")
        UsdPhysics.DriveAPI(prim, "angular").GetMaxForceAttr().Set(float(T_LOOSE))


bodykit._author_hinges = _author_hinges

OBJECTS = [_saw(), _board(), _bar(), _feather(), _fingers(), _setfeather(), _knob("knob1", KNOB1_0, KNOB_W1),
           _knob("knob2", KNOB2_0, KNOB_W2)]


# -- followed after every control step ----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tgt": {"slot": 0.0, "on_bar": 0.0, "k1": 0.0, "k2": 0.0}, "cap": {}, "tight": {"k1": False, "k2": False},
            "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FEATHER_DEBUG")), "steps": 0}
_ATTRS: dict = {}


def _drive(name: str, target=None, cap=None) -> None:
    """Move a joint drive's target and/or its force (torque) cap. Sliders in m and N, knobs in degrees and N m."""
    if name not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        if name in bodykit._SLIDER_PATHS:
            path, kind = bodykit._SLIDER_PATHS[name], "linear"
        else:
            path, kind = bodykit._HINGE_PATHS[name], "angular"
        d = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(path), kind)
        _ATTRS[name] = (d.GetTargetPositionAttr(), d.GetMaxForceAttr())
    if target is not None:
        _ATTRS[name][0].Set(float(target))
        _EP["tgt"][name] = float(target)
    if cap is not None and _EP["cap"].get(name) != cap:
        _ATTRS[name][1].Set(float(cap))
        _EP["cap"][name] = cap


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for name, cap in (("slot", F_LOOSE), ("on_bar", F_LOOSE), ("k1", T_LOOSE), ("k2", T_LOOSE)):
        _drive(name, 0.0, cap)


def _stick(name: str, pos: float, cap: float, k: float, slack: float) -> None:
    """Stick-slip friction from a capped spring: once pushed past what the cap holds, the spring's anchor follows."""
    e = pos - _EP["tgt"][name]
    lim = cap / k
    if abs(e) > lim + slack:
        _drive(name, target=pos - math.copysign(lim, e))
    _drive(name, cap=cap)


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


def _yaw(r) -> float:
    return math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "board")
    pr, rr = _pose(base, origin, "bar")
    pf, rf = _pose(base, origin, "feather")
    pg, rg = _pose(base, origin, "fingers")
    ps, _ = _pose(base, origin, "setfeather")
    _, rk1 = _pose(base, origin, "knob1")
    _, rk2 = _pose(base, origin, "knob2")
    # the knobs: angle on their bolts (negative = tightened), stick-slip, tight past TIGHT_DEG
    ang = {}
    for name, rk in (("k1", rk1), ("k2", rk2)):
        a = _yaw(rr.T @ rk)
        ang[name] = a
        _stick(name, a, T_TIGHT if a <= -SNUG_DEG else T_LOOSE, K_KNOB, 0.5)
        if not _EP["tight"][name] and a <= -TIGHT_DEG:
            _EP["tight"][name] = True
        elif _EP["tight"][name] and a > -LOOSEN_DEG:
            _EP["tight"][name] = False
    t1, t2 = _EP["tight"]["k1"], _EP["tight"]["k2"]
    # the bar in the slot (x), the body across the bar (y): clamped by the knobs, otherwise loose (stick-slip)
    x_slot = float(pr[0] - K1_0[0])
    y_bar = float((rr.T @ (pf - pr))[1] - (FB0[1] - K1_0[1]))
    for name, pos, locked in (("slot", x_slot, t1), ("on_bar", y_bar, t1 or t2)):
        was = _EP["cap"].get(name) == F_LOCK
        if locked and not was:
            _drive(name, target=pos, cap=F_LOCK)
        elif not locked:
            if was:
                _drive(name, target=pos)
            _stick(name, pos, F_LOOSE, K_HOLD, SLACK)
    # what the score is built from
    flex = float((rf.T @ (pg - pf))[1])  # how far the fingers are flexed back
    sflex = float((rf.T @ (ps - pf))[1])  # how far the setup feather is bent back
    lead = float((pf + rf @ np.array([LEAD_X, FB_TIP_Y, 0.0]))[0])
    ahead = X_BLADE - lead
    gaps = []  # the board's fence-side edge: its gap to the fence at the saw's front edge and at the board's front end
    for x in (SX0 + 0.010, None):
        lx = BOARD_L / 2 - 0.005 if x is None else float((rb.T @ (np.array([x, pb[1], pb[2]]) - pb))[0])
        gaps.append(float((pb + rb @ np.array([lx, -BOARD_W / 2, 0.0]))[1] - FENCE_Y))
    front = max(float((pb + rb @ np.array([BOARD_L / 2, s * BOARD_W / 2, 0.0]))[0]) for s in (-1.0, 1.0))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[2, 2])))))
    board_ok = (tilt <= 3.0 and abs(float(pb[2]) - BOARD0[2]) <= 0.003 and front <= X_BLADE
                and abs(float(pb[0]) - BOARD0[0]) <= BOARD_MOVE)
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"ahead": LEAD_WINDOW[0] <= ahead <= LEAD_WINDOW[1], "touch": flex >= TOUCH,
                "on_fence": board_ok and max(gaps) <= FENCE_GAP and min(gaps) >= -0.002,
                "set": SET_WINDOW[0] <= flex <= SET_WINDOW[1], "locked": t1 and t2}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            mm = lambda v: np.round((np.asarray(v) - [0, 0, ST]) * 1000).astype(int).tolist()  # noqa: E731
            print(f"[fb] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]} k1={ang['k1']:6.1f}"
                  f" k2={ang['k2']:6.1f} tight={t1},{t2} x_slot={x_slot * 1000:6.1f} y_bar={y_bar * 1000:6.1f}"
                  f" flex={flex * 1000:5.2f} setf={sflex * 1000:5.2f} ahead={ahead * 1000:5.0f} gaps={[round(g * 1000, 1) for g in gaps]}"
                  f" board={mm(pb)} tilt={tilt:.1f} front={front * 1000:.0f} L={mm(tl)} R={mm(tr)}"
                  f" s={_EP['s']} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring ------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """All judged at the end, on live states. 1.0: the featherboard ahead of the blade (its leading finger tip 1 to
    7.5 cm in front of the blade's front edge), its fingers on the board's edge, the board flat against the fence at
    both ends (within 1 mm) and still where it lay, the fingers flexed 2 to 4 mm (the setup feather within 1 mm of
    the board's edge, or bent by it at most 1 mm), both knobs tight, and both arms back where they started; 0.75: the
    same with an arm not back;
    0.5: ahead of the blade, fingers on the board and the board against the fence; 0.25: ahead of the blade with its
    fingers on the board; else 0.0."""
    s = _EP["s"]
    if not s or not (s["ahead"] and s["touch"]):
        return 0.0
    if not s["on_fence"]:
        return 0.25
    if not (s["set"] and s["locked"]):
        return 0.5
    return 1.0 if _arms_home() else 0.75


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
    """Full score with everything that can move at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("board", "bar", "feather", "fingers",
                                                                         "setfeather", "knob1", "knob2"))


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The right hand pushes the board flat against the fence and backs off. The left takes the rear knob and slides the
# featherboard back along the slot until its leading finger is 4.5 cm in front of the blade, then pushes its body in
# toward the board until the setup feather touches and keeps it there while the right turns the rear knob tight. The
# right lets go and goes home, the left lets go of the body, turns the front knob tight, and goes home.

PUSH_FACE = 0.012  # a closed top-down gripper's thin face is this far from its TCP
PUSH_Z = ST + BOARD_T + 0.002  # pushing TCP height: the fingertips 11 mm below the board's top
KNOB_Z = ST + FB_H + 0.025  # gripping TCP height on a knob: 3 mm under its wing's top
KNOB_OPEN = -0.013  # about 2.6 cm open round the 14 mm wing
BOARD_PUSH_X = (SX0 + BOARD_X1) / 2  # push the board's edge midway along the part lying on the saw
BOARD_PAST = 0.018  # aim this far past the fence (the arm gives)
PRESS_X = SET_K1[0] + 0.175  # the left pushes the body's back edge here
PRESS_PAST = 0.046  # aim this far past where the setup feather touches (the arm gives about 4.5 cm at 10.5 N)
TURN_DEG = 105.0  # each knob is turned this far clockwise (it stops at 100: tight)
UP = np.array([0.0, 0.0, 0.04])
UP_K = np.array([0.0, 0.0, 0.020])  # over a knob: the fingertips 7 mm above its wing
CLEAR_K = np.array([0.0, 0.0, 0.026])  # risen off a knob before the wrist turns back: the fingertips 1 cm above it
LEAD_S, LEAD_STEPS = 3.0, 15

ROT_PUSH = grip_rotation(math.pi / 2)  # closing along x: the thin face pushes along y
ROT_PRESS = grip_rotation(-math.pi / 2)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.3, 1.9), (1.0, 1.7), (-1.0,), (0.0,), (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
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


def demo():
    from yam_arena.constants import READY

    from yam_arena import ik, station

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):  # where each arm starts
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, r)

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def turn(arm, t, deg0, deg1, finger, label, n=7):
        p, _, _ = cur[arm]
        for j in range(1, n + 1):
            r = grip_rotation(math.radians(deg0 + (deg1 - deg0) * j / n))
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p), finger, label, r))
        cur[arm] = (p, finger, grip_rotation(math.radians(deg1)))

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def lead_in(arm, pos, rot, finger, label):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        cur[arm] = (np.asarray(plan[arm][-1].pos, float), f, plan[arm][-1].yaw)

    # 1. the right pushes the board flat against the fence at its middle, then backs off
    edge0 = BOARD0[1] + BOARD_W / 2
    start = np.array([BOARD_PUSH_X, edge0 + PUSH_FACE + 0.010, PUSH_Z])
    lead_in("right", start + UP, ROT_PUSH, FINGER_CLOSED, "over the board's edge")
    go("right", 1.0, start, FINGER_CLOSED, "down beside the board's edge")
    go("right", 1.6, [BOARD_PUSH_X, EDGE_SET + PUSH_FACE - BOARD_PAST, PUSH_Z], FINGER_CLOSED,
       "push the board flat against the fence", n=3)
    wait("right", 0.4, "hold it there")
    go("right", 0.6, [BOARD_PUSH_X, EDGE_SET + PUSH_FACE + 0.015, PUSH_Z], FINGER_CLOSED, "back off")
    go("right", 0.8, [BOARD_PUSH_X, EDGE_SET + PUSH_FACE + 0.015, PUSH_Z] + UP, FINGER_CLOSED, "rise")
    home("right", "out of the way")

    # 2. the left takes the rear knob and slides the featherboard back along the slot, ahead of the blade
    k1_start = K1_0 + [0.0, 0.0, KNOB_Z - ST]
    k1_set = SET_K1 + [0.0, 0.0, KNOB_Z - ST]
    rot_k1 = grip_rotation(KNOB_W1 - math.pi)  # the left takes the wing turned half a turn (its wrist is freer)
    wait("left", dur("right") - LEAD_S)
    lead_in("left", k1_start + UP_K, rot_k1, KNOB_OPEN, "over the rear knob")
    go("left", 1.0, k1_start, KNOB_OPEN, "down round its wing")
    go("left", 0.6, k1_start, FINGER_CLOSED, "take the rear knob")
    go("left", 2.0, k1_set - [0.003, 0.0, 0.0], FINGER_CLOSED, "slide the featherboard back, ahead of the blade", n=3)
    go("left", 0.5, k1_set - [0.003, 0.0, 0.0], KNOB_OPEN, "let go")
    go("left", 0.6, k1_set + UP_K, KNOB_OPEN, "rise")

    # 3. the left pushes the body in toward the board until the setup feather touches, and holds it there
    back_y = FB0[1] + FB_BACK_Y + PUSH_FACE
    press0 = np.array([PRESS_X, back_y + 0.008, ST + 0.021])
    go("left", 1.4, press0 + UP, FINGER_CLOSED, "over the featherboard's back edge", rot=ROT_PRESS, n=2)
    go("left", 1.0, press0, FINGER_CLOSED, "down behind it")
    set_back = EDGE_SET + D_SET - FB_TIP_Y + FB_BACK_Y + PUSH_FACE  # the TCP with the setup feather touching
    go("left", 2.4, [PRESS_X, set_back - PRESS_PAST, press0[2]], FINGER_CLOSED,
       "push the featherboard in until the setup feather touches the board", n=4)
    wait("left", 0.6, "keep it pressed in")

    # 4. the right turns the rear knob tight (it spreads the bar and clamps the body) while the left keeps it pressed
    wait("right", dur("left") - dur("right") - 3.0)
    q_over = _good_ik("right", k1_set + UP_K, grip_rotation(KNOB_W1))
    plan["right"] += _joint_path("right", np.array(READY, float), q_over, "over the rear knob", KNOB_OPEN)
    cur["right"] = (k1_set + UP_K, KNOB_OPEN, grip_rotation(KNOB_W1))
    go("right", 1.0, k1_set, KNOB_OPEN, "down round its wing")
    go("right", 0.6, k1_set, FINGER_CLOSED, "take the rear knob")
    turn("right", 3.5, math.degrees(KNOB_W1), math.degrees(KNOB_W1) - TURN_DEG, FINGER_CLOSED,
         "turn it clockwise until it is tight")
    wait("right", 0.4, "tight")
    go("right", 0.6, k1_set, FINGER_OPEN, "let go: open wide")
    go("right", 0.6, k1_set + CLEAR_K, FINGER_OPEN, "rise clear of the wing")
    home("right")

    # 5. the left lets go of the body, turns the front knob tight and goes home
    wait("left", dur("right") - dur("left") - LEAD_S - 0.6, "keep it pressed in")
    go("left", 0.6, [PRESS_X, set_back + 0.015, press0[2]], FINGER_CLOSED, "let go of the featherboard")
    go("left", 0.6, [PRESS_X, set_back + 0.015, press0[2]] + UP, FINGER_CLOSED, "rise")
    k2_set = SET_K1 + [K2_X, 0.0, KNOB_Z - ST]
    go("left", 1.4, k2_set + UP_K, KNOB_OPEN, "over the front knob", rot=grip_rotation(KNOB_W2), n=2)
    go("left", 1.0, k2_set, KNOB_OPEN, "down round its wing")
    go("left", 0.6, k2_set, FINGER_CLOSED, "take the front knob")
    turn("left", 3.5, math.degrees(KNOB_W2), math.degrees(KNOB_W2) - TURN_DEG, FINGER_CLOSED,
         "turn it clockwise until it is tight")
    wait("left", 0.4, "tight")
    go("left", 0.6, k2_set, FINGER_OPEN, "let go: open wide")
    go("left", 0.6, k2_set + CLEAR_K, FINGER_OPEN, "rise clear of the wing")
    home("left")
    end = max(dur("left"), dur("right")) + 3.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still: the trial ends once the full score has held 1 s with both arms home")
    if os.environ.get("FEATHER_ONE_HAND"):  # one-hand check: the left does its part, the right never moves
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
        plan["right"] = [Waypoint(dur("left"), tuple(float(v) for v in p), FINGER_OPEN, "the right never moves", r)]
    return plan
