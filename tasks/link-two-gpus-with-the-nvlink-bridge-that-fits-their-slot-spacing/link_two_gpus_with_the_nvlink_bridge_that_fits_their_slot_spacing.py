"""On a blue antistatic mat stands a Streacom BC1 V2 open benchtable: a workstation motherboard on its aluminium
plate, with two NVIDIA RTX A6000 graphics cards (dual-slot, 267 x 111 mm) standing in its first and fourth x16 slots,
three slot pitches apart (60.96 mm), screwed to the bench's bracket bar. Their NVLink connector covers are off. To the
right of the bench lie the two RTX A-series NVLink bridges: the 2-slot one (40.64 mm between its sockets) and the 3-slot
one (60.96 mm).

Task: link the two cards with the bridge that fits their spacing, leave the other bridge on the mat, and finish with
both arms back where they started.

What the physics asks for: the bridge goes down onto its connectors only level. Each end's socket takes about 18 N to
push over its card's NVLink fingers (6 mm of travel); a bridge cocked 4 degrees or more while on its fingers levers
the contacts (pried: the trial fails), and at 5 degrees the low socket binds. The 2-slot bridge can't reach both
connectors. A single press lands off-centre and runs one end ahead, so the bridge has to be pushed down at both ends
together.

Run it:  inspect-robots run --embodiment yam_arena -E task=link_two_gpus_with_the_nvlink_bridge_that_fits_their_slot_spacing.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the antistatic mat ------------------------------------------------------------------------------------------------
MAT_X0, MAT_X1, MAT_Y0, MAT_Y1, MAT_T = 0.050, 0.520, -0.330, 0.240, 0.002
MT = TOP + MAT_T
# -- the test bench and the motherboard --------------------------------------------------------------------------------
BT = MT + 0.0266  # the motherboard's top: a Streacom BC1 V2 holds the board 25 mm above the desk
PITCH = 0.02032  # PCI Express slot pitch (0.8 in)
SLOT1_Y = 0.0305  # slot 1's card centre line (PCB plane); slots number toward -y
XC = 0.285  # the cards' NVLink connectors (their middle, along the card)
XB = XC + 0.046  # the cards' bracket end
CARD_L, CARD_H = 0.267, 0.112  # an RTX A6000: 267 mm long, its north edge 112 mm above the board
HC = BT + CARD_H  # the cards' north edges (TOP + 0.141)
COOL, BACK = 0.0343, 0.0027  # each card's body from its PCB plane: cooler side (-y) and back side (+y)
BOARD_X0, BOARD_X1, BOARD_Y0, BOARD_Y1 = XB - 0.242, XB + 0.002, -0.160, 0.145  # ATX, 244 x 305 mm
CARDS = {"a": SLOT1_Y, "b": SLOT1_Y - 3 * PITCH}  # slots 1 and 4: three pitches (60.96 mm) apart
# -- the NVLink connector on each card's north edge, and its cut-out in the shroud ------------------------------------
RECESS_HX, RECESS_D = 0.026, 0.0015  # the shallow recess under the bridge's plate (52 mm long, 1.5 mm deep)
WELL_HX, WELL_HY, WELL_D = 0.0235, 0.0045, 0.011  # the cut-out round the fingers, where the socket goes down
LEAD = 0.003  # 45-degree lead-in chamfers round the cut-out's mouth
FINGER_HX = 0.022  # the gold NVLink fingers (44 mm)
ENTRY = 0.0027  # a socket's bottom rests this far below the north edge when it sits on the fingers' lead-in
TRAVEL = 0.006  # and goes 6 mm further down to be seated (its plate then 2.5 mm proud of the north edge)
# -- the bridges: origin at the middle of the sockets' bottoms -----------------------------------------------------------
B_LX = 0.048  # along the cards
SOCK_W, SOCK_H = 0.007, 0.008  # each socket housing, 44 mm long
PLATE_T = 0.0032  # the PCB and its cover
B_H = SOCK_H + PLATE_T  # 11.2 mm
BRIDGES = {"bridge3": 3 * PITCH, "bridge2": 2 * PITCH}  # socket spacing (60.96 / 40.64 mm)
B_MASS = {"bridge3": 0.036, "bridge2": 0.029}
B_POS = {"bridge3": (0.300, -0.255), "bridge2": (0.190, -0.255)}  # on the mat, long side across (along y)
# -- the rules ---------------------------------------------------------------------------------------------------------
F_SEAT = float(os.environ.get("NVL_FSEAT", 18.0))  # N per end to push a socket over its fingers
SLIP = 0.001
K_SEAT, D_SEAT = F_SEAT / SLIP, 20.0  # stick-slip: the spring holds up to F_SEAT, then the socket slides
BIND_DEG = 5.0  # a cocked socket binds on its fingers: the lower end goes no further
PRY_DEG = 4.0  # cocked this far with an end on its fingers: the contacts are levered (pried): fails the trial
ON_DEPTH = 0.0005  # an end is on its fingers once pushed this far past the lead-in
SEATED_TOL = 0.0005  # seated: within 0.5 mm of the bottom
LEVEL_DEG = 1.0
MOVED2, LIFTED2 = 0.030, 0.020  # the 2-slot bridge moved this far, or lifted this high, has been taken off the mat
DROP_BELOW = 0.060  # a bridge this far below the cards' north edges over the board has fallen in among the parts
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

ALU, ALU_D = (0.78, 0.79, 0.81), (0.55, 0.56, 0.58)
MAT_C, MAT_E = (0.30, 0.42, 0.58), (0.22, 0.32, 0.45)
PCB, PCB_L = (0.10, 0.12, 0.11), (0.20, 0.24, 0.21)
SLOT_C, SLOT_M = (0.12, 0.12, 0.13), (0.70, 0.72, 0.74)
SHROUD, SHROUD_L, TRIM = (0.07, 0.07, 0.08), (0.20, 0.20, 0.22), (0.62, 0.63, 0.65)
GOLD, PCB_G = (0.86, 0.68, 0.30), (0.10, 0.22, 0.14)
SOCK_C, COVER_C, LOGO_C = (0.48, 0.49, 0.51), (0.09, 0.09, 0.10), (0.42, 0.43, 0.45)
STEEL, DARK = (0.74, 0.75, 0.77), (0.03, 0.03, 0.035)


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _mat() -> Body:
    shapes = [_box((MAT_X0, MAT_Y0, -MAT_T), (MAT_X1, MAT_Y1, 0.0), MAT_C)]
    for x in (MAT_X0 + 0.004, MAT_X1 - 0.004):  # a darker bound edge
        shapes.append(_box((x - 0.003, MAT_Y0, -0.0004), (x + 0.003, MAT_Y1, 0.0002), MAT_E, False))
    for y in (MAT_Y0 + 0.004, MAT_Y1 - 0.004):
        shapes.append(_box((MAT_X0, y - 0.003, -0.0004), (MAT_X1, y + 0.003, 0.0002), MAT_E, False))
    shapes.append(CylShape((MAT_X0 + 0.030, MAT_Y0 + 0.030, 0.002), 0.006, 0.004, STEEL, collide=False))  # ground stud
    shapes.append(CylShape((MAT_X0 + 0.030, MAT_Y0 + 0.030, 0.0045), 0.0035, 0.003, (0.1, 0.5, 0.2), collide=False))
    for k, (x, y) in enumerate(((0.120, -0.215), (0.122, -0.255))):  # the cards' two NVLink covers, taken off
        shapes.append(_box((x - 0.023, y - 0.0045, 0.0), (x + 0.023, y + 0.0045, 0.006), DARK, False))
        shapes.append(_box((x - 0.018, y - 0.0012, 0.006), (x + 0.018, y + 0.0012, 0.0064), (0.25, 0.25, 0.27), False))
    return Body.build("mat", pos=(0.0, 0.0, MT), mass=1.0, kinematic=True, shapes=shapes, color=MAT_C,
                      friction=(0.7, 0.6))


def _bench() -> Body:
    """The open benchtable (Streacom BC1 V2): an aluminium plate on four short legs, the motherboard on standoffs,
    and a bracket bar at the back that the cards' brackets are screwed to. Origin: the board's top surface, under the
    connectors' line."""
    o = np.array([XC, 0.0, BT])
    sh = []

    def add(lo, hi, color, collide=True):
        sh.append(_box(np.asarray(lo) - o, np.asarray(hi) - o, color, collide))

    tray_top = BT - 0.0091  # the BC1's 360 x 270 mm aluminium plate on its legs, the board on 7.5 mm standoffs
    add((XB + 0.018 - 0.270, -0.188, tray_top - 0.003), (XB + 0.018, 0.172, tray_top), ALU)
    for x in (XB + 0.010 - 0.270, XB + 0.010):
        for y in (-0.180, 0.164):
            add((x - 0.0075, y - 0.0075, MT), (x + 0.0075, y + 0.0075, tray_top - 0.003), ALU_D)
    for x in (BOARD_X0 + 0.006, BOARD_X1 - 0.006):  # standoffs
        for y in (BOARD_Y0 + 0.006, BOARD_Y1 - 0.006, -0.03):
            sh.append(CylShape(tuple((np.array([x, y, tray_top + 0.0037]) - o).tolist()), 0.0025, 0.0074, (0.85, 0.75,
                               0.35), collide=False))
    add((BOARD_X0, BOARD_Y0, BT - 0.0016), (BOARD_X1, BOARD_Y1, BT), PCB)
    # the bracket rail: an upright plate behind the board with a ledge on top that the brackets' tabs screw to
    add((XB + 0.0035, -0.112, tray_top), (XB + 0.0060, 0.066, HC - 0.004), ALU)
    add((XB + 0.0015, -0.112, HC - 0.004), (XB + 0.0170, 0.066, HC - 0.0015), ALU)
    # the slots: seven x16 connectors, 89 mm long, every 20.32 mm
    for k in range(7):
        y = SLOT1_Y - k * PITCH
        add((XB - 0.130, y - 0.0038, BT), (XB - 0.041, y + 0.0038, BT + 0.011), SLOT_C, collide=False)
        add((XB - 0.044, y - 0.0040, BT), (XB - 0.038, y + 0.0040, BT + 0.012), SLOT_M, collide=False)  # latch
        if k not in (0, 3):
            add((XB - 0.128, y - 0.0007, BT + 0.011), (XB - 0.043, y + 0.0007, BT + 0.0112), DARK, collide=False)
    # CPU, its top-flow cooler, the memory, VRM heat sinks, rear I/O, chipset, the 24-pin connector
    cx, cy = XB - 0.095, 0.098
    add((cx - 0.040, cy - 0.040, BT), (cx + 0.040, cy + 0.040, BT + 0.006), (0.70, 0.71, 0.72))
    add((cx - 0.050, cy - 0.046, BT + 0.006), (cx + 0.050, cy + 0.046, BT + 0.040), (0.62, 0.63, 0.66))
    for k in range(9):
        y = cy - 0.040 + k * 0.010
        add((cx - 0.050, y - 0.0006, BT + 0.040), (cx + 0.050, y + 0.0006, BT + 0.041), (0.50, 0.51, 0.53), False)
    add((cx - 0.046, cy - 0.046, BT + 0.041), (cx + 0.046, cy + 0.046, BT + 0.066), (0.13, 0.13, 0.14))  # 92 mm fan
    sh.append(CylShape(tuple((np.array([cx, cy, BT + 0.0662]) - o).tolist()), 0.043, 0.0006, (0.22, 0.22, 0.24),
                       collide=False))
    sh.append(CylShape(tuple((np.array([cx, cy, BT + 0.0668]) - o).tolist()), 0.016, 0.0008, (0.08, 0.08, 0.09),
                       collide=False))
    for k in range(4):
        x = 0.112 + k * 0.009
        add((x - 0.0035, 0.012 + 0.030, BT), (x + 0.0035, 0.142, BT + 0.007), SLOT_C, False)
        add((x - 0.003, 0.046, BT + 0.007), (x + 0.003, 0.138, BT + 0.038), (0.10, 0.10, 0.11), False)
        add((x - 0.0031, 0.050, BT + 0.036), (x + 0.0031, 0.134, BT + 0.0385), (0.42, 0.43, 0.45), False)
    add((cx - 0.060, cy + 0.048, BT), (cx + 0.035, cy + 0.060, BT + 0.024), (0.22, 0.23, 0.25), False)
    add((cx - 0.075, cy - 0.050, BT), (cx - 0.062, cy + 0.045, BT + 0.024), (0.22, 0.23, 0.25), False)
    add((XB - 0.040, 0.050, BT), (XB + 0.001, 0.140, BT + 0.035), (0.16, 0.16, 0.18), False)
    add((0.140, -0.150, BT), (0.190, -0.105, BT + 0.009), (0.25, 0.26, 0.28), False)
    add((0.092, 0.020, BT), (0.100, 0.072, BT + 0.012), (0.85, 0.85, 0.82), False)
    for k in range(3):  # M.2 covers between the slots
        y = SLOT1_Y - (k + 1.5) * PITCH
        if k != 1:
            add((XB - 0.170, y - 0.010, BT), (XB - 0.060, y + 0.010, BT + 0.004), (0.30, 0.31, 0.33), False)
    # the bench's ATX power supply, standing on the mat behind it (its cables to the board and cards not drawn)
    add((0.375, 0.080, MT), (0.515, 0.230, MT + 0.086), (0.08, 0.08, 0.09))
    sh.append(CylShape(tuple((np.array([0.445, 0.155, MT + 0.0862]) - o).tolist()), 0.062, 0.0006, (0.20, 0.20, 0.22),
                       collide=False))
    sh.append(CylShape(tuple((np.array([0.445, 0.155, MT + 0.0866]) - o).tolist()), 0.020, 0.0006, (0.12, 0.12, 0.13),
                       collide=False))
    add((0.3745, 0.090, MT + 0.010), (0.3755, 0.220, MT + 0.076), (0.16, 0.16, 0.18), False)  # modular sockets
    return Body.build("bench", pos=tuple(o.tolist()), mass=2.0, kinematic=True, shapes=sh, color=ALU,
                      friction=(0.5, 0.4))


def _card(name: str, y0: float) -> Body:
    """An RTX A6000 standing in its slot, its cooler toward -y. Origin: the middle of its NVLink connector, on the
    north edge, on the PCB's plane. The shroud is cut out round the connector so the bridge's socket can go down
    over the fingers; a 45-degree lead-in round the cut-out guides it in."""
    x0, x1 = XB - CARD_L - XC, XB - XC
    zb, cy0, cy1 = -CARD_H, -COOL, BACK
    sh = [
        _box((x0, cy0, zb), (x1, cy1, -WELL_D), SHROUD),
        _box((x0, cy0, -WELL_D), (-RECESS_HX, cy1, 0.0), SHROUD),
        _box((RECESS_HX, cy0, -WELL_D), (x1, cy1, 0.0), SHROUD),
        _box((-RECESS_HX, cy0, -WELL_D), (RECESS_HX, -WELL_HY, -RECESS_D), SHROUD_L),
        _box((-RECESS_HX, -WELL_HY, -WELL_D), (-WELL_HX, WELL_HY + 0.0015, -RECESS_D), SHROUD_L),
        _box((WELL_HX, -WELL_HY, -WELL_D), (RECESS_HX, WELL_HY + 0.0015, -RECESS_D), SHROUD_L),
        _box((-RECESS_HX, WELL_HY, -WELL_D), (RECESS_HX, WELL_HY + 0.0015, -RECESS_D), SHROUD_L),  # the cut-out's lip
    ]
    s2 = math.sqrt(0.5)
    # lead-in chamfers (their faces from the recess floor down into the cut-out, 45 degrees)
    for side, (ya, yb, n) in {"cool": (-WELL_HY - LEAD, -WELL_HY, (0.0, s2, s2)),
                              "back": (WELL_HY + 0.0015, WELL_HY, (0.0, -s2, s2))}.items():
        ym, zm = (ya + yb) / 2, -RECESS_D - abs(ya - yb) / 2
        c = np.array([0.0, ym, zm]) - 0.001 * np.array(n)
        sh.append(bodykit.plate((-WELL_HX, c[1], c[2]), (WELL_HX, c[1], c[2]), abs(ya - yb) * math.sqrt(2.0), 0.002,
                                n, SHROUD_L))
    for sx in (-1.0, 1.0):
        n = (-sx * s2, 0.0, s2)
        xm = sx * (WELL_HX + 0.0015 / 2)
        c = np.array([xm, 0.0, -RECESS_D - 0.00075]) - 0.001 * np.array(n)
        sh.append(bodykit.plate((c[0], -WELL_HY, c[2]), (c[0], WELL_HY, c[2]), 0.0015 * math.sqrt(2.0), 0.002, n,
                                SHROUD_L))
    # what shows: the gold NVLink fingers in the cut-out, the exposed PCB on the back, trim, the blower, the bracket
    sh.append(_box((-FINGER_HX, -0.0008, -WELL_D), (FINGER_HX, 0.0008, -ENTRY), PCB_G, False))
    for k in range(22):
        x = -FINGER_HX + 0.001 + k * 0.002
        sh.append(_box((x - 0.0006, -0.00085, -0.0080), (x + 0.0006, 0.00085, -ENTRY - 0.0004), GOLD, False))
    sh.append(_box((x0 + 0.004, cy1, zb + 0.004), (x1 - 0.004, cy1 + 0.0004, -0.003), PCB_G, False))
    for k in range(5):
        z = zb + 0.015 + k * 0.018
        sh.append(_box((x0 + 0.02, cy1 + 0.0004, z), (x1 - 0.06, cy1 + 0.0007, z + 0.0012), (0.55, 0.48, 0.25),
                       False))
    sh.append(_box((x0 + 0.05, cy0 - 0.0004, -0.012), (x1 - 0.004, cy0, -0.006), TRIM, False))
    sh.append(_box((x0 + 0.004, cy0 - 0.0004, zb + 0.006), (x1 - 0.004, cy0, zb + 0.010), TRIM, False))
    sh.append(CylShape((x0 + 0.050, cy0 - 0.0006, -0.056), 0.034, 0.0012, SHROUD_L, collide=False, axis="Y"))
    sh.append(CylShape((x0 + 0.050, cy0 - 0.0010, -0.056), 0.012, 0.0012, TRIM, collide=False, axis="Y"))
    sh.append(_box((x0 + 0.100, -0.006, -0.0003), (x0 + 0.150, -0.002, 0.0002), (0.36, 0.37, 0.39), False))  # name
    sh.append(_box((x0 - 0.010, -0.014, -0.022), (x0, -0.002, -0.004), DARK, False))  # 8-pin power plug at its end
    sh.append(_box((x0 - 0.024, -0.010, -0.016), (x0 - 0.010, -0.006, -0.010), DARK, False))
    sh.append(_box((x0 + 0.004, -0.0008, zb - 0.010), (x1 - 0.045, 0.0008, zb), PCB_G, False))  # the edge connector
    # the dual-slot bracket, its tab screwed to the rail, four DisplayPort sockets and a vent grille
    sh.append(_box((x1, -0.0365, zb - 0.012), (x1 + 0.0015, 0.0060, 0.003), STEEL))
    sh.append(_box((x1, -0.0365, 0.0015), (x1 + 0.0170, 0.0060, 0.003), STEEL, False))
    for y in (-0.024, -0.004):
        sh.append(CylShape((x1 + 0.012, y, 0.0035), 0.0028, 0.0012, (0.85, 0.86, 0.88), collide=False))
    for k in range(4):
        z = zb + 0.016 + k * 0.019
        sh.append(_box((x1 + 0.0015, -0.006, z), (x1 + 0.0045, 0.0015, z + 0.011), DARK, False))
    for k in range(6):
        z = zb + 0.012 + k * 0.015
        sh.append(_box((x1 + 0.0015, -0.032, z), (x1 + 0.0025, -0.012, z + 0.008), (0.25, 0.25, 0.27), False))
    return Body.build(name, pos=(XC, y0, HC), mass=1.2, kinematic=True, shapes=sh, color=SHROUD, friction=(0.5, 0.4))


def _stop(name: str, y0: float) -> Body:
    """Hidden: what a socket rests on, on the fingers' lead-in, until it is pushed down over them."""
    return Body.build(name, pos=(XC, y0, HC - ENTRY - 0.0025), mass=0.03,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (2 * FINGER_HX, SOCK_W - 0.0004, 0.005), None)],
                      friction=(0.8, 0.7), solver_iterations=32, damping=(0.0, 0.5))


def _bridge(name: str) -> Body:
    """An RTX A-series NVLink bridge lying on its sockets: two grey socket housings under a PCB with a glossy black
    cover. Origin: the middle of the sockets' bottoms."""
    p = BRIDGES[name]
    hy = p / 2 + SOCK_W / 2
    sh = [_box((-B_LX / 2, -hy, SOCK_H), (B_LX / 2, hy, B_H), COVER_C)]
    for s in (-1.0, 1.0):
        sh.append(_box((-0.022, s * p / 2 - SOCK_W / 2, 0.0), (0.022, s * p / 2 + SOCK_W / 2, SOCK_H), SOCK_C))
        for k in range(14):  # contact slots on the socket's outer face
            x = -0.019 + k * 0.0029
            sh.append(_box((x - 0.0006, s * hy - 0.0002, 0.0012), (x + 0.0006, s * hy + 0.0002, 0.0050),
                           (0.18, 0.18, 0.20), False))
    sh.append(_box((-B_LX / 2, -hy, SOCK_H), (B_LX / 2, hy, SOCK_H + 0.0016), GOLD, False))
    sh.append(_box((-B_LX / 2 + 0.0004, -hy + 0.0004, SOCK_H + 0.0016), (B_LX / 2 - 0.0004, hy - 0.0004, B_H + 0.0001),
                   COVER_C, False))
    sh.append(_box((0.008, hy - 0.016, B_H), (0.019, hy - 0.006, B_H + 0.0003), LOGO_C, False))
    sh.append(_box((-0.019, -hy + 0.006, B_H), (0.004, -hy + 0.008, B_H + 0.0003), LOGO_C, False))
    x, y = B_POS[name]
    return Body.build(name, pos=(x, y, MT + 0.0002), mass=B_MASS[name], shapes=sh, color=COVER_C,
                      friction=(1.0, 0.9), solver_iterations=32, damping=(0.0, 0.2))


for _c, _y in CARDS.items():
    slider(f"seat_{_c}", f"card_{_c}", f"stop_{_c}", anchor=(XC, _y, HC - ENTRY - 0.0025), axis="Z",
           limits=(-TRAVEL, 0.0), stiffness=K_SEAT, damping=D_SEAT, target=0.0, max_force=1.0e3)

OBJECTS = [_mat(), _bench(), _card("card_a", CARDS["a"]), _card("card_b", CARDS["b"]), _stop("stop_a", CARDS["a"]),
           _stop("stop_b", CARDS["b"]), _bridge("bridge3"), _bridge("bridge2")]
STOP0 = {c: HC - ENTRY - 0.0025 for c in CARDS}  # the stops' starting heights (their middles)
_CORNERS2 = [np.array([a, b, c]) for a in (-B_LX / 2, B_LX / 2) for b in (-BRIDGES["bridge2"] / 2 - SOCK_W / 2,
             BRIDGES["bridge2"] / 2 + SOCK_W / 2) for c in (0.0, B_H)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": {c: 0.0 for c in CARDS}, "pried": False, "dropped": False, "took2": False, "why": "", "tcps": None,
            "s": {}, "max_tilt": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("NVL_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for c in CARDS:
        bodykit.set_slider_drive(f"seat_{c}", target=0.0)


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


def _sockets(p, r, name):
    """The bottom middles of a bridge's two sockets (its -y one first) and its roll (rad, +: its +y end higher)."""
    half = BRIDGES[name] / 2
    return [p + r @ np.array([0.0, s * half, 0.0]) for s in (-1.0, 1.0)], math.asin(max(-1.0, min(1.0, r[2, 1])))


def _in_cutout(q, c) -> bool:
    return abs(q[0] - XC) <= 0.006 and abs(q[1] - CARDS[c]) <= 0.004 and q[2] <= HC - RECESS_D + 0.001


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    br = {n: _pose(base, origin, n) for n in BRIDGES}
    socks = {n: _sockets(*br[n], n) for n in BRIDGES}
    depth, occ = {}, {}
    for c in CARDS:
        depth[c] = max(0.0, STOP0[c] - float(_pose(base, origin, f"stop_{c}")[0][2]))
        occ[c] = next(((n, k) for n in BRIDGES for k in range(2) if _in_cutout(socks[n][0][k], c)), None)
    for c in CARDS:  # stick-slip: an end slides down over its fingers only when pushed harder than F_SEAT
        hold, d = _EP["hold"][c], depth[c]
        if occ[c] is None:
            if hold != 0.0:
                _EP["hold"][c] = 0.0
                bodykit.set_slider_drive(f"seat_{c}", target=0.0)
            continue
        n, k = occ[c]
        roll = socks[n][1]
        lower = (roll > 0) == (k == 0)  # this socket is the bridge's lower end
        bound = abs(math.degrees(roll)) >= BIND_DEG and lower
        if d >= TRAVEL - 0.0004:
            new = TRAVEL
        elif d - hold > SLIP and not bound:
            new = d
        else:
            new = hold
        if new != hold:
            _EP["hold"][c] = new
            bodykit.set_slider_drive(f"seat_{c}", target=-new)
    for n in BRIDGES:  # an end on its fingers with the bridge cocked: the contacts are levered
        ends = [c for c in CARDS if occ[c] is not None and occ[c][0] == n and depth[c] >= ON_DEPTH]
        tilt = abs(math.degrees(socks[n][1]))
        if ends:
            _EP["max_tilt"] = max(_EP["max_tilt"], tilt)
        if ends and tilt >= PRY_DEG and not _EP["pried"]:
            _EP["pried"], _EP["why"] = True, f"{n} cocked {tilt:.1f} deg on {ends}"
        lo = min(q[2] for q in socks[n][0])
        on_board = BOARD_X0 - 0.01 < br[n][0][0] < XB + 0.02 and BOARD_Y0 - 0.01 < br[n][0][1] < BOARD_Y1 + 0.01
        if on_board and lo < HC - DROP_BELOW and not _EP["dropped"]:
            _EP["dropped"], _EP["why"] = True, f"{n} dropped onto the board"
    p3, r3 = br["bridge3"]
    (s3a, s3b), roll3 = socks["bridge3"]
    both = all(occ[c] is not None and occ[c][0] == "bridge3" for c in CARDS)
    seat_z = HC - ENTRY - TRAVEL  # a seated socket's bottom
    seated = (both and all(depth[c] >= TRAVEL - SEATED_TOL for c in CARDS)
              and all(abs(q[2] - seat_z) <= SEATED_TOL for q in (s3a, s3b)) and abs(math.degrees(roll3)) <= LEVEL_DEG)
    p2, r2 = br["bridge2"]
    lo2 = float(min((p2 + r2 @ c)[2] for c in _CORNERS2))
    x2, y2 = B_POS["bridge2"]
    if math.hypot(p2[0] - x2, p2[1] - y2) > MOVED2 or lo2 > MT + LIFTED2:  # the other bridge has been taken up
        _EP["took2"] = True
    on_mat = (MAT_X0 + 0.01 < p2[0] < MAT_X1 - 0.01 and MAT_Y0 + 0.01 < p2[1] < MAT_Y1 - 0.01 and lo2 < MT + 0.003
              and not (0.06 < p2[0] < XB + 0.03 and -0.195 < p2[1] < 0.18))
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"seated": seated, "started": both and abs(math.degrees(roll3)) <= 3.0,
                "other_on_mat": on_mat and not _EP["took2"]}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, HC]) * 1000, 1).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            push = {c: round(K_SEAT * max(0.0, depth[c] - _EP["hold"][c]), 1) for c in CARDS}
            print(f"[nvl] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]}"
                  f" b3={mm(p3)} roll3={math.degrees(roll3):.2f} b2={mm(p2)} roll2={math.degrees(socks['bridge2'][1]):.2f}"
                  f" occ={occ} depth={ {c: round(depth[c] * 1000, 2) for c in CARDS} }"
                  f" hold={ {c: round(_EP['hold'][c] * 1000, 2) for c in CARDS} } push={push}"
                  f" s={_EP['s']} pried={_EP['pried']} dropped={_EP['dropped']} {_EP['why']}"
                  f" L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once either bridge has been cocked 4 degrees or more with an end on a card's fingers (pried) or dropped
    onto the motherboard. Else, at the end, on live states: 1.0: the 3-slot bridge seated on both cards (both sockets
    within 0.5 mm of fully down, level within 1 degree), the 2-slot bridge left on the mat all along (never moved 3 cm
    or lifted 2 cm) and both arms back where they started; 0.75: the same with an arm not back; 0.5: the 3-slot bridge
    seated on both cards; 0.25: it sits square in both connectors' cut-outs (level within 3 degrees), not yet pushed
    down; else 0.0."""
    s = _EP["s"]
    if not s or _EP["pried"] or _EP["dropped"]:
        return 0.0
    if s["seated"]:
        if s["other_on_mat"]:
            return 1.0 if _arms_home() else 0.75
        return 0.5
    return 0.25 if s["started"] else 0.0


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
    """Full score with both bridges at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("bridge3", "bridge2"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand takes the 3-slot bridge off the mat by its two ends, carries it over the cards and lets it down onto
# both connectors' lead-ins. Then both hands, leaning out to either side so their wrists stay clear of each other,
# press the bridge down at its two ends together until both sockets are seated. Both arms go home.

PICK_PITCH = math.radians(30.0)  # fingers ahead of the wrist: reaches high enough over the cards
LEAN = math.radians(45.0)  # the pressing hands lean out to either side
GRIP_ABOVE = 0.006  # the TCP over the bridge's top while it is held by its ends
GRIP_BACK = 0.004  # and this far behind its middle (the pitched pads sit ahead of the TCP)
OPEN_W = -0.0405  # 8.1 cm open round the 6.8 cm bridge
HOLD_W = -0.028  # closed to 5.6 cm: a firm grip on its ends
RELEASE_GAP = 0.003  # the sockets' bottoms this far above the north edge when the bridge is let go
PRESS_Y = 0.0300  # each hand presses over a socket
PRESS_DEPTH = {"left": 0.075, "right": 0.075}  # aimed this far below the bridge's seated top (the arms give)
TIP, TIP_SIDE = 0.013, 0.010  # a closed fingertip's lowest edge from the TCP: along the approach and sideways
LEAD_S, LEAD_STEPS = 3.0, 15


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


ROT_CARRY = _ry(-PICK_PITCH) @ grip_rotation(0.0)  # closing along y, across the bridge's ends
ROT_PRESS = {"left": _rx(-LEAN) @ grip_rotation(-math.pi / 2), "right": _rx(LEAN) @ grip_rotation(math.pi / 2)}


def _press_tcp(arm: str, top_z: float) -> np.ndarray:
    """The TCP that puts the closed fingertip's lowest edge on the bridge's top over this arm's socket."""
    rot = ROT_PRESS[arm]
    a = rot[:, 2]
    side = np.array([0.0, a[2], -a[1]])
    side = side if side[2] < 0 else -side
    y = PRESS_Y if arm == "left" else -PRESS_Y
    if arm == "right" and os.environ.get("NVL_ONEHAND") == "centre":  # checks only: one hand pressing the middle
        y = 0.0
    contact = np.array([XC, y, top_z])
    return contact - TIP * a - TIP_SIDE * side


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


def demo():
    from yam_arena.constants import READY

    from yam_arena import ik, station

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_CLOSED, r)
    moved = set()

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

    def lead(arm, pos, rot, label, finger):
        q0 = np.array(READY, float) if arm not in moved else _good_ik(arm, cur[arm][0], cur[arm][2])
        moved.add(arm)
        plan[arm].extend(_joint_path(arm, q0, _good_ik(arm, pos, rot), label, finger))
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def held(x, y, socket_bottom_z):
        """The TCP holding the bridge by its ends with its sockets' bottoms at this height."""
        return np.array([x - GRIP_BACK, y, socket_bottom_z + B_H + GRIP_ABOVE])

    # 1. the right takes the 3-slot bridge off the mat by its two ends
    bx, by = B_POS["bridge3"]
    pick = held(bx, by, MT)
    lead("right", pick + [0.0, 0.0, 0.060], ROT_CARRY, "over the 3-slot bridge", OPEN_W)
    go("right", 1.2, pick, OPEN_W, "down round its two ends")
    go("right", 0.6, pick, HOLD_W, "take it by its ends")
    go("right", 1.4, held(bx, by, HC + 0.060), HOLD_W, "lift it well above the cards' height", n=2)
    # 2. carry it over the cards and let it down onto both connectors' lead-ins
    over = held(XC, 0.0, HC + 0.060)
    go("right", 2.0, over, HOLD_W, "carry it over the cards", n=4)
    go("right", 1.2, held(XC, 0.0, HC + 0.015), HOLD_W, "lower it over both connectors")
    go("right", 1.0, held(XC, 0.0, HC + RELEASE_GAP), HOLD_W, "line both sockets up over the fingers")
    wait("right", 0.3)
    go("right", 0.4, held(XC, 0.0, HC + RELEASE_GAP), OPEN_W, "let it down onto the lead-ins")
    go("right", 0.6, held(XC, 0.0, HC + 0.045), OPEN_W, "rise clear")
    # 3. both hands lean in over the two ends and press them down together
    top_entry = HC - ENTRY + B_H
    one = os.environ.get("NVL_ONEHAND")  # checks only, not the task: "right" replays the right arm's part alone,
    if one == "straddle":  # "straddle" has the right alone press both ends with its fingers held 61 mm apart
        tip = np.array([XC, 0.0, top_entry]) - TIP * ROT_CARRY[:, 2]
        go("right", 1.0, tip + [0.0, 0.0, 0.030], -BRIDGES["bridge3"] / 2, "fingers open over both sockets")
        go("right", 0.8, tip + [0.0, 0.0, 0.002], -BRIDGES["bridge3"] / 2, "fingertips onto both ends")
        go("right", 2.0, tip - [0.0, 0.0, TRAVEL + 0.120], -BRIDGES["bridge3"] / 2, "press hard", n=4)
        wait("right", 1.5, "keep pressing")
        go("right", 0.8, tip + [0.0, 0.0, 0.050], -BRIDGES["bridge3"] / 2, "lift off")
    ready = {arm: _press_tcp(arm, top_entry + 0.030) for arm in ("left", "right")}
    if one == "straddle":
        p, f, r = cur["right"]
        plan["right"] += _joint_path("right", _good_ik("right", p, r), np.array(READY, float), "home", f)
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        plan["left"] = [Waypoint(dur("right") + 3.0, tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
        return plan
    lead("right", ready["right"], ROT_PRESS["right"], "lean in over the bridge's right-hand end", FINGER_CLOSED)
    wait("left", dur("right") - LEAD_S / 2, "wait at home")
    lead("left", ready["left"], ROT_PRESS["left"], "lean in over the bridge's left-hand end", FINGER_CLOSED)
    wait("right", dur("left") - dur("right"), "wait for the other hand")
    for arm in ("left", "right"):
        go(arm, 0.8, _press_tcp(arm, top_entry + 0.002), FINGER_CLOSED, "fingertips onto the bridge's end")
    for arm in ("left", "right"):
        seat = _press_tcp(arm, HC - ENTRY - TRAVEL + B_H - PRESS_DEPTH[arm])
        go(arm, 1.6, seat, FINGER_CLOSED, "press both ends down together", n=4)
        wait(arm, 1.0, "keep pressing: both sockets seat")
    for arm in ("left", "right"):
        go(arm, 0.8, _press_tcp(arm, top_entry + 0.040), FINGER_CLOSED, "lift off")
    # 4. both arms go home and hold still
    for arm in ("left", "right"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    if one:  # the left stays home
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        plan["left"] = [Waypoint(dur("right"), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    return plan
