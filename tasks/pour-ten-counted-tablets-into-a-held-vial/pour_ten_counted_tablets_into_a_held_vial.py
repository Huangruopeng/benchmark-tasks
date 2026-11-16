"""A pharmacy dispensing counter. At the back stands a pharmacy shelving unit of stock bottles. On the counter: a
100-count stock bottle of tablets (a 150 cc white HDPE packer with a plain 38-400 screw cap; 16 tablets left) and a
right-handed pill counting tray (a flat counting floor; down its right side a trough with a hinged lid, opening into a
spout at its far end; a return spout at its near-left corner; a spatula slot on the outside of its left wall) with its
stainless-steel counting spatula in the slot.

Task: count out a prescription of exactly ten tablets: open the stock bottle, pour tablets onto the counting tray, put
exactly ten into the tray's trough with the spatula and close the trough's lid, and screw the bottle's cap back on; no
tablet touched by a gripper or dropped.

Run it:  inspect-robots run --embodiment yam_arena -E task=pour_ten_counted_tablets_into_a_held_vial.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     plate, rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()


def _quat_from_matrix(m) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, by Shepperd's method (bodykit's own picks wrong signs near 180 deg)."""
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
    return (float(x / n), float(y / n), float(z / n), float(w / n))


bodykit._quat_from_matrix = _quat_from_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PHYSICS_DT = 1.0 / 240.0
DT = 0.05

WHITE, HDPE, CAP_WHITE, LABEL, LABEL_BLUE = (0.96, 0.96, 0.94), (0.95, 0.95, 0.93), (0.97, 0.97, 0.97), \
    (0.98, 0.97, 0.92), (0.15, 0.35, 0.70)
TRAY_C, LID_C, STEEL, WOOD, BRASS = (0.72, 0.85, 0.96), (0.22, 0.52, 0.88), (0.78, 0.79, 0.80), (0.55, 0.35, 0.18), \
    (0.78, 0.64, 0.30)
AMBER, SHELF_C, SHELF_DARK = (0.66, 0.33, 0.07), (0.93, 0.93, 0.92), (0.55, 0.56, 0.58)


def ring(r, z0, z1, t, n, color, collide=True, a0=0.0):
    """n flat wall panels round the body's z axis, mid-thickness radius r, from z0 to z1 (a cylinder's wall)."""
    w = 2.0 * (r + t / 2) * math.tan(math.pi / n) * 1.04
    out = []
    for k in range(n):
        a = a0 + 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        out.append(plate((r * c, r * s, z0), (r * c, r * s, z1), w, t, (c, s, 0.0), color, collide))
    return out


def cone(r0, z0, r1, z1, t, n, color):
    """n panels of a cone's wall from radius r0 at z0 to r1 at z1."""
    w = 2.0 * (max(r0, r1) + t / 2) * math.tan(math.pi / n) * 1.04
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        nrm = np.array([c * (z1 - z0), s * (z1 - z0), -(r1 - r0)])
        out.append(plate((r0 * c, r0 * s, z0), (r1 * c, r1 * s, z1), w, t, tuple(nrm / np.linalg.norm(nrm)), color))
    return out


def tube_x(u0, u1, v_a, z_a, r, t, a0, a1, n, color, collide=True):
    """Panels of a tube along the body's x (u0..u1) round the axis (v_a, z_a), from angle a0 to a1 (deg, measured
    from +y toward +z): a trough, or its lid."""
    out = []
    span = math.radians(a1 - a0) / n
    w = 2.0 * (r + t / 2) * math.tan(span / 2) * 1.06
    for k in range(n):
        a = math.radians(a0) + span * (k + 0.5)
        c, s = math.cos(a), math.sin(a)
        out.append(plate((u0, v_a + r * c, z_a + r * s), (u1, v_a + r * c, z_a + r * s), w, t, (0.0, c, s), color,
                         collide))
    return out


# =====================================================================================================================
# The stock bottle: a 150 cc white HDPE round packer (53.3 mm across, 93.0 mm tall to the top of its 38-400 neck),
# 17 g, with a 100-count label; 16 tablets left in it. Frame: origin at the middle of its underside.
B_R, B_T, B_SH, B_NECK0, B_H = 0.02665, 0.001, 0.068, 0.078, 0.093
NECK_RO, NECK_RI = 0.0188, 0.0155  # the 38-400 finish: 37.6 mm over the thread, a 31 mm bore
B_MASS = 0.017
B_FLOOR_X = 0.003  # hidden: the floor collides 3 mm thicker than it looks, so a falling tablet can't pass through
LINER_T = 0.003  # hidden: the body's wall collides 3 mm thicker inside, so a fast tablet can't pass through it
BX0, BY0 = 0.17, 0.05
BOTTLE_BACK = np.array([0.17, 0.09])  # where the right hand stands the bottle after pouring (clear of the spatula)
# Its cap: a white ribbed PP 38-400 screw cap, 40.5 mm across and 15 mm tall, 4 g. Frame: origin at the middle of its
# open (bottom) rim. Screwed down it rests with its inner top face on the neck's lip.
CAP_R, CAP_H, CAP_TOPT, CAP_MASS = 0.02025, 0.015, 0.0015, 0.004
CAP_SEATED_Z = B_H - (CAP_H - CAP_TOPT)  # its rim this high above the bottle's underside when screwed down
PITCH = 0.0254 / 6  # 6 threads per inch: 4.23 mm per turn
OFF_DEG = 360.0  # a 400 finish has one turn of thread: unscrewed a full turn, the cap comes off
BREAK_NM, RUN_NM = 1.0, 0.3  # torque to break the cap loose (about half its 2.0 N m application torque), then to turn it
ROT_K = 0.05  # N m per degree: the thread's friction spring (the task file drags its target along: stick-slip)

# The tablets: round, flat-faced, bevel-edged, film-coated, 10 mm across and 4 mm thick, 0.35 g.
TAB_R, TAB_H, TAB_M, TAB_MU = 0.005, 0.004, 0.00035, 0.25
N_TABS, DOSE = 16, 10


def _bottle() -> Body:
    s = [CylShape((0.0, 0.0, 0.00075), B_R, 0.0015, HDPE)]
    s += ring(B_R - B_T / 2, 0.0, B_SH, B_T, 24, HDPE)
    s += cone(B_R - B_T / 2, B_SH, NECK_RO - 0.0012, B_NECK0, B_T, 24, HDPE)
    s += ring((NECK_RO + NECK_RI) / 2, B_NECK0, B_H, NECK_RO - NECK_RI, 16, HDPE)
    # its inside surface, where the tablets slide: HDPE on film coating, slick (collision only, its own material)
    global B_SLICK
    liner = len(s)
    s += ring(B_R - B_T - 0.0001 - LINER_T / 2, 0.0015, B_SH, LINER_T, 24, None)
    r_in = B_R - B_T - 0.0001 - LINER_T  # and on through the shoulder to the neck's bore, with no ledge
    s += cone(r_in + 0.001, B_SH, NECK_RI + 0.001, B_NECK0, 0.002, 24, None)
    s.append(CylShape((0.0, 0.0, 0.0015 + B_FLOOR_X / 2), r_in, B_FLOOR_X, None))  # and a thicker floor
    B_SLICK = [0, *range(25, liner), *range(liner, len(s))]  # all but the body's outside, which the fingers grip
    # the label: white with a blue band (drawn only)
    s += [CylShape((0.0, 0.0, 0.036), B_R + 0.0003, 0.050, LABEL, collide=False),
          CylShape((0.0, 0.0, 0.050), B_R + 0.0005, 0.010, LABEL_BLUE, collide=False),
          CylShape((0.0, 0.0, 0.024), B_R + 0.0005, 0.003, LABEL_BLUE, collide=False)]
    return Body.build("bottle", pos=(BX0, BY0, TOP + 0.0005), mass=B_MASS, shapes=s, color=HDPE, friction=(1.0, 0.9),
                      com=(0.0, 0.0, 0.040), solver_iterations=32, contact_offset=0.001)


def _cap() -> Body:
    s = [CylShape((0.0, 0.0, CAP_H - CAP_TOPT / 2), CAP_R, CAP_TOPT, CAP_WHITE)]
    s += ring(CAP_R - 0.0005, 0.0, CAP_H - CAP_TOPT, 0.001, 20, CAP_WHITE)
    for k in range(30):  # knurled ribs (drawn only)
        a = 2 * math.pi * k / 30
        s.append(BoxShape(((CAP_R + 0.0002) * math.cos(a), (CAP_R + 0.0002) * math.sin(a), CAP_H / 2 - 0.0005),
                          (0.0008, 0.0012, CAP_H - 0.002), (0.88, 0.88, 0.88), yaw=a, collide=False))
    return Body.build("cap", pos=(BX0, BY0, TOP + 0.0005 + CAP_SEATED_Z), mass=CAP_MASS, shapes=s, color=CAP_WHITE,
                      friction=(1.2, 1.1), com=(0.0, 0.0, CAP_H * 0.6), ignore=("bottle",), solver_iterations=32)


def _tablets() -> list[Body]:
    out = []
    for i in range(N_TABS):
        layer, k = divmod(i, 4)
        x, y = (0.0105 if k % 2 else -0.0105), (0.0105 if k >= 2 else -0.0105)
        z = TOP + 0.0005 + 0.0015 + B_FLOOR_X + TAB_H / 2 + 0.0005 + (TAB_H + 0.0007) * layer
        out.append(Body.build(f"tab{i:02d}", pos=(BX0 + x, BY0 + y, z), mass=TAB_M,
                              shapes=[CylShape((0.0, 0.0, 0.0), TAB_R, TAB_H, WHITE)], color=WHITE,
                              friction=(TAB_MU, TAB_MU), slick=True, damping=(0.05, 0.3), solver_iterations=32,
                              contact_offset=0.0008))
    return out


# =====================================================================================================================
# The counting tray: right-handed, 212 x 149 mm and 38 mm tall, clear blue plastic, 100 g (as Apothecary Products
# 23087, 8.35 x 5.87 x 1.57 in; PharmaSystems D-50, 21 x 16 x 3.5 cm). Frame: origin at the middle of its underside,
# x (u) along its length, pointing away from the robot; y (v) to its left.
TL, TW, WT = 0.212, 0.149, 0.002
R_I, R_O = 0.016, 0.018  # the trough: a half-tube 32 mm across inside, its lid closes it into a tube
V_A, Z_A = -TW / 2 + R_O, 0.0195  # the trough's axis; the counting floor's top is level with it
Z_F = Z_A
WALL_TOP = 0.038  # the left and back walls stand 1.85 cm above the floor
RIB_H, RIB_W = 0.0025, 0.008  # the rib at the trough's edge: 2.5 mm high, its ramp 8 mm wide
FRONT_TOP = Z_F + 0.006  # the front wall only 6 mm, so the spatula reaches the whole floor
SPOUT_L, SPOUT_OUT = 0.025, 0.018  # the trough's spout: 2.5 cm past the back wall, narrowing to 1.8 cm
CHUTE = (0.038, TW / 2 - WT)  # the return spout's mouth in the front wall (v from, to)
CHUTE_L, CHUTE_OUT = 0.030, 0.022
TRAY_MASS = 0.10
TX, TY = 0.255, -0.065
# the spatula slot along the outside of the left wall: a 2 mm gap behind a rail, its floor 14 mm up
SLOT_V, SLOT_Z0, SLOT_U = TW / 2 + 0.005, 0.014, (-TL / 2 - 0.003, -TL / 2 + 0.135)  # a 1 cm slot, blade in its middle


def _rx_q(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _tray() -> Body:
    hl = TL / 2
    v0 = V_A + R_I  # the counting floor's right edge: the trough's inner rim
    s = []
    s += tube_x(-hl, hl, V_A, Z_A, (R_I + R_O) / 2, WT, 180.0, 360.0, 12, TRAY_C)  # the trough
    s.append(BoxShape((-hl - WT / 2, V_A, Z_A / 2 + 0.0005), (WT, 2 * R_O, Z_A), TRAY_C))  # its near end, closed
    # the spout out of the trough's far end: a floor narrowing from 3.2 to 1.8 cm, with low side walls
    zb = Z_A - R_I
    s.append(BoxShape((hl + SPOUT_L / 2, V_A, zb - WT / 2 + 0.0004), (SPOUT_L + 0.002, 0.030, WT), TRAY_C))
    for side in (-1, 1):
        s.append(plate((hl - 0.001, V_A + side * 0.016, zb + 0.007), (hl + SPOUT_L, V_A + side * SPOUT_OUT / 2,
                        zb + 0.007), 0.014, WT, (0.0, side, 0.0), TRAY_C))
    s.append(BoxShape((0.0, (v0 + TW / 2 - WT) / 2, Z_F - WT / 2), (TL - 2 * WT, TW / 2 - WT - v0, WT), TRAY_C))
    s.append(BoxShape((0.0, (v0 + TW / 2 - WT) / 2, Z_F - WT - 0.005), (TL - 2 * WT, TW / 2 - WT - v0, 0.010), None))
    # the rib along the floor's edge over the trough: 2.5 mm high, a ramp on the floor's side (tablets stop against it
    # unless the spatula pushes them up it and over)
    s.append(BoxShape((0.0, v0 + RIB_W / 2, Z_F + RIB_H / 2 - 0.0006),
                      (TL - 2 * WT, math.hypot(RIB_W, RIB_H), 0.0012), TRAY_C,
                      quat_xyzw=_rx_q(-math.degrees(math.atan2(RIB_H, RIB_W)))))
    s.append(BoxShape((0.0, TW / 2 - WT / 2, WALL_TOP / 2), (TL, WT, WALL_TOP), TRAY_C))  # left wall
    s.append(BoxShape((hl - WT / 2, (v0 + TW / 2) / 2, WALL_TOP / 2), (WT, TW / 2 - v0, WALL_TOP), TRAY_C))  # back
    s.append(BoxShape((-hl + WT / 2, (v0 + CHUTE[0]) / 2, FRONT_TOP / 2), (WT, CHUTE[0] - v0, FRONT_TOP), TRAY_C))
    s.append(BoxShape((-hl + WT / 2, (CHUTE[0] + TW / 2) / 2, (Z_F - WT) / 2), (WT, TW / 2 - CHUTE[0], Z_F - WT),
                      TRAY_C))  # under the return spout's mouth
    s.append(BoxShape((0.0, v0 - 0.0005, (Z_F - WT) / 2), (TL, WT, Z_F - WT), TRAY_C))  # the floor's inner skirt
    # the return spout: the floor carried 3 cm on out of the front-left corner, its sides closing to 2.2 cm
    c0, c1, cm = -hl, -hl - CHUTE_L, (CHUTE[0] + CHUTE[1]) / 2
    s.append(BoxShape(((c0 + c1) / 2, cm, Z_F - WT / 2), (CHUTE_L + 0.002, CHUTE[1] - CHUTE[0] + 0.004, WT), TRAY_C))
    for side, v in ((-1, CHUTE[0]), (1, CHUTE[1])):
        s.append(plate((c0, v, Z_F + 0.006), (c1, cm + side * CHUTE_OUT / 2, Z_F + 0.006), 0.012, WT,
                       (0.0, side, 0.0), TRAY_C))
    # the spatula slot outside the left wall: a rail 3.5 mm out and a floor
    s.append(BoxShape(((SLOT_U[0] + SLOT_U[1]) / 2, SLOT_V + 0.006, (SLOT_Z0 + 0.024) / 2),
                      (SLOT_U[1] - SLOT_U[0], 0.002, 0.024 - SLOT_Z0 + 0.002), TRAY_C))
    s.append(BoxShape(((SLOT_U[0] + SLOT_U[1]) / 2, SLOT_V + 0.0005, SLOT_Z0 - 0.001), (SLOT_U[1] - SLOT_U[0], 0.012, 0.002),
                      TRAY_C))
    return Body.build("tray", pos=(TX, TY, TOP + 0.0005), mass=TRAY_MASS, shapes=s, friction=(0.6, 0.5), color=TRAY_C,
                      com=(0.0, -0.005, 0.017), solver_iterations=32, contact_offset=0.0015)


# The trough's lid: a blue half-tube hinged along the trough's outer rim, standing open 110 degrees. Frame: origin on
# the hinge line; unturned it is closed over the trough.
LID_R, LID_OPEN, LID_MASS = (R_O + 0.0005 + R_O + 0.0025) / 2, 110.0, 0.012
HINGE_V = V_A - R_O
LID_ANCHOR = (TX, TY + HINGE_V, TOP + 0.0005 + Z_A)


def _lid() -> Body:
    hl = TL / 2
    s = tube_x(-hl, hl, R_O, 0.0, LID_R, WT, 0.0, 180.0, 10, LID_C)
    for u in (-hl - 0.001, hl + 0.001):  # end caps
        s.append(BoxShape((u, R_O, 0.0095), (WT, 2 * LID_R, 0.019), LID_C))
    return Body.build("lid", pos=LID_ANCHOR, mass=LID_MASS, shapes=s, color=LID_C, friction=(0.6, 0.5),
                      start_xyzw=_rx_q(LID_OPEN), com=(0.0, R_O, 0.012), contact_offset=0.001)


# The counting spatula: a 5 in (127 mm) stainless blade, 19 mm wide and 0.8 mm thick with a round tip, on a 100 mm
# wooden handle 19 x 11 mm (as Apothecary Products 23317), 30 g. Frame: origin where blade meets handle, x toward the
# tip, the blade's width along z (it stands on edge in the slot).
BLADE_L, BLADE_W, BLADE_T, HANDLE_L, HANDLE_T = 0.127, 0.019, 0.0008, 0.100, 0.011
SPAT_MASS = 0.030
SPAT_X0 = TX - TL / 2 - 0.004  # in the slot: the blade in it, the handle out past the tray's near end


def _spatula() -> Body:
    s = [BoxShape(((BLADE_L - 0.0095) / 2, 0.0, 0.0), (BLADE_L - 0.0095, BLADE_T, BLADE_W), STEEL),
         CylShape((BLADE_L - 0.0095, 0.0, 0.0), 0.0095, BLADE_T, STEEL, axis="Y"),
         BoxShape((-HANDLE_L / 2, 0.0, 0.0), (HANDLE_L, HANDLE_T, BLADE_W), WOOD)]
    for x in (-0.012, -0.030):
        for side in (-1, 1):
            s.append(CylShape((x, side * (HANDLE_T / 2 + 0.0002), 0.0), 0.0022, 0.0004, BRASS, collide=False,
                              axis="Y"))
    return Body.build("spatula", pos=(SPAT_X0, TY + SLOT_V, TOP + 0.0005 + SLOT_Z0 + BLADE_W / 2 + 0.0004),
                      mass=SPAT_MASS, shapes=s, color=STEEL, friction=(1.0, 0.9), com=(0.008, 0.0, 0.0),
                      solver_iterations=32, contact_offset=0.0008)


# The pharmacy shelving behind the counter: two 36 in (91.4 cm) sections of white steel shelving, 12 in (30.5 cm)
# deep, 84 in (2.13 m) tall, standing on the floor just behind the counter's back edge, full of stock bottles. Fixed,
# drawn only (far beyond the arms' reach).
SHELF_X0, SHELF_D, SHELF_W, SHELF_H = 0.93, 0.305, 0.914, 2.13
FLOOR_Z = TOP - 0.76


def _shelving() -> Body:
    s = []
    h = SHELF_H
    for k in range(3):  # uprights
        y = -SHELF_W + k * SHELF_W
        for x in (0.01, SHELF_D - 0.01):
            s.append(BoxShape((x, y, h / 2), (0.02, 0.03, h), SHELF_DARK, collide=False))
    s.append(BoxShape((SHELF_D - 0.005, 0.0, h / 2), (0.01, 2 * SHELF_W, h), SHELF_C, collide=False))  # back panel
    levels = (0.10, 0.44, 0.80, 1.14, 1.48, 1.82, 2.12)
    rng = np.random.default_rng(7)
    sizes = [(0.0267, 0.093), (0.031, 0.105), (0.0335, 0.115), (0.0385, 0.137), (0.0233, 0.083)]
    for z in levels:
        s.append(BoxShape((SHELF_D / 2, 0.0, z), (SHELF_D, 2 * SHELF_W, 0.02), SHELF_C, collide=False))
        s.append(BoxShape((0.006, 0.0, z + 0.012), (0.012, 2 * SHELF_W, 0.045), SHELF_DARK, collide=False))  # price rail
        if z > 2.0 or z < 0.7:  # bottles only on the shelves above the counter, where the cameras see them
            continue
        y = -SHELF_W + 0.04
        while y < SHELF_W - 0.04:
            r, bh = sizes[int(rng.integers(len(sizes)))]
            amber = rng.random() < 0.25
            body_c = AMBER if amber else HDPE
            label_c = [LABEL_BLUE, (0.75, 0.15, 0.15), (0.15, 0.55, 0.30), (0.95, 0.65, 0.10), (0.45, 0.25, 0.60)][
                int(rng.integers(5))]
            for row in ((0.08, 0.20) if z < 1.3 else (0.08,)):
                x = row + rng.uniform(-0.01, 0.01)
                s.append(CylShape((x, y, z + 0.01 + bh / 2), r, bh, body_c, collide=False))
                s.append(CylShape((x, y, z + 0.01 + bh + 0.008), r * 0.78, 0.016, CAP_WHITE, collide=False))
                if row < 0.1:
                    s.append(CylShape((x, y, z + 0.01 + bh * 0.45), r + 0.0006, bh * 0.45, LABEL, collide=False))
                    s.append(CylShape((x, y, z + 0.01 + bh * 0.55), r + 0.0009, bh * 0.10, label_c, collide=False))
            y += 2 * r + 0.012
        if z == levels[0]:
            s.append(BoxShape((0.012, 0.0, 0.05), (0.024, 2 * SHELF_W, 0.10), SHELF_DARK, collide=False))  # kick plate
    s.append(BoxShape((SHELF_D / 2, 0.0, h), (SHELF_D, 2 * SHELF_W, 0.02), SHELF_C))  # top (the body's one collider)
    return Body.build("shelving", pos=(SHELF_X0, 0.0, FLOOR_Z), mass=50.0, shapes=s, kinematic=True, color=SHELF_C)


SHELVING, TRAY, LID, SPATULA, BOTTLE, CAP = _shelving(), _tray(), _lid(), _spatula(), _bottle(), _cap()
TABLETS = _tablets()
TAB_NAMES = [t.name for t in TABLETS]
OBJECTS = [SHELVING, TRAY, LID, SPATULA, BOTTLE, CAP, *TABLETS]
MOVING = ["tray", "lid", "spatula", "bottle", "cap", *TAB_NAMES]

hinge("lid_hinge", "tray", "lid", anchor=LID_ANCHOR, axis="X", limits_deg=(0.0, LID_OPEN))
# The cap on the neck: a thread authored on. It keeps the cap on the neck's axis; the task file lowers or raises it
# PITCH per turn as it turns, holds it with thread friction (stick-slip, below), and lets it off after a full turn.
# Its slide coordinate runs down the axis: 0 is screwed down (the cap's inner top on the neck's lip).
thread("cap_thread", "bottle", "cap", anchor0=(0.0, 0.0, B_H), axis0=(0.0, 0.0, -1.0),
       anchor1=(0.0, 0.0, CAP_H - CAP_TOPT), axis1=(0.0, 0.0, -1.0), slide_limits=(-0.008, 0.0), turn_damping=0.01,
       slide_stiffness=4000.0, slide_damping=8.0, slide_max_force=60.0, always_on=True)

# -- authoring hooks: the thread's friction drive; contact reports on the tablets; prim paths -------------------------
_J: dict = {}
_author_threads_plain = bodykit._author_threads
_author_sliders_plain = bodykit._author_sliders


def _author_threads(stage, prim_path: str, spec) -> None:
    _author_threads_plain(stage, prim_path, spec)
    if spec.name == "cap":
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/cap_thread"), "rotX")
        drive.GetStiffnessAttr().Set(float(ROT_K))
        drive.GetMaxForceAttr().Set(float(BREAK_NM))
        drive.GetTargetPositionAttr().Set(0.0)
        _J["thread"] = f"{prim_path}/cap_thread"


HDPE_MU = 0.25  # film-coated tablets on the bottle's HDPE inside (the outside keeps 1.0 for the fingers)


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "bottle":
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material

        path = f"{prim_path}/Looks/hdpe_inside"
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=HDPE_MU, dynamic_friction=HDPE_MU,
                                             friction_combine_mode="min")
        mat.func(path, mat)
        for i in B_SLICK:
            bind_physics_material(f"{prim_path}/body/s{i}", path, stage=stage)
        path2 = f"{prim_path}/Looks/hdpe_outside"
        mat2 = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9)
        mat2.func(path2, mat2)
        for i in range(1, 25):  # the body's outside, where the fingers hold it
            bind_physics_material(f"{prim_path}/body/s{i}", path2, stage=stage)
    if spec.name.startswith("tab"):
        from pxr import PhysxSchema

        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body"))
        api.CreateThresholdAttr(0.0)
    _J[spec.name] = f"{prim_path}/body"
    _J["env"] = prim_path.rsplit("/", 1)[0]


bodykit._author_threads = _author_threads
bodykit._author_sliders = _author_sliders

PADS = [(arm, tip) for arm in ("LeftArm", "RightArm") for tip in ("tip_left", "tip_right")]
_CT = {"view": None, "failed": ""}


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _view():
    """A PhysX rigid-contact view, made once: each tablet against the four finger pads."""
    if _CT["view"] is None and not _CT["failed"] and "env" in _J and all(n in _J for n in TAB_NAMES):
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            links = asset_link_paths()
            pads = [f"{_J['env']}/{arm}{links[tip]}" for arm, tip in PADS]
            sim = PhysxManager.get_physics_sim_view()
            _CT["view"] = sim.create_rigid_contact_view([_J[n] for n in TAB_NAMES],
                                                        filter_patterns=[pads] * len(TAB_NAMES),
                                                        max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
            print(f"[tab] contact view failed: {_CT['failed']}", file=sys.stderr, flush=True)
    return _CT["view"]


def _pad_touch() -> float:
    """The largest force (N) any finger pad puts on any tablet now (0 if unknown)."""
    v = _view()
    if v is None:
        return 0.0
    try:
        m = _np(v.get_contact_force_matrix(dt=PHYSICS_DT)).reshape(len(TAB_NAMES), len(PADS), 3)
    except Exception as e:  # noqa: BLE001
        _CT["failed"] = "read: " + repr(e)[:300]
        return 0.0
    return float(np.linalg.norm(m, axis=2).max())


def _thread_target_attr():
    if "thread_tgt" not in _J:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        _J["thread_tgt"] = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(_J["thread"]), "rotX") \
            .GetTargetPositionAttr()
    return _J["thread_tgt"]


# =====================================================================================================================
# Followed after every control step.
TOUCH_N = 0.05  # a finger pad pushing a tablet this hard: touched
LID_SHUT = 8.0  # the trough's lid shut: within this many degrees of closed
DOWN_Z = 0.012  # a tablet outside the bottle and the tray with its centre this low over the counter: dropped
CATCH = (0.006, 12.0, (-0.014, 0.002), 20.0)  # recapping: radial (m), tilt (deg), slide window (m), turn to catch (deg)
SEAT_SLIDE = -0.0006  # screwed down: the cap within 0.6 mm of its seat
SUCCESS_HOLD_S = 1.0


def _fresh() -> dict:
    return {"on": True, "turn": 0.0, "last_yaw": None, "tgt": 0.0, "loose": False, "off": False, "opened": False,
            "pre": 0.0, "x0": 0.0, "recap_turn": 0.0, "caught": False, "slide": 0.0,
            "where": ["bottle"] * N_TABS, "spilt": False, "touched": False, "touch_max": 0.0, "lid": 110.0, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TAB_DEBUG")), "steps": 0, "every": int(os.environ.get("TAB_EVERY", "10"))}


@on_reset
def _new_episode() -> None:
    was_off = not _EP.get("on", True)
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    try:
        set_slide_target("cap_thread", 0.0)
        _thread_target_attr().Set(0.0)
        if was_off:
            set_thread_on("cap_thread", False)
            set_thread_on("cap_thread", True)
    except Exception:  # not authored yet (the first reset comes before the scene exists)
        pass


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _wrap(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def _where(p, bottle, tray) -> str:
    """Where a tablet's centre is: 'bottle', 'trough' (the tray's trough or its spout), 'floor' (the tray's counting
    floor or its return spout), 'down' (on the counter or the floor) or 'air'."""
    bp, br = bottle
    q = br.T @ (p - bp)
    if math.hypot(q[0], q[1]) < B_R - B_T and -0.002 < q[2] < B_H + 0.003:
        return "bottle"
    tp, tr = tray
    q = tr.T @ (p - tp)
    if -0.003 < q[2] < WALL_TOP + 0.010:
        hl = TL / 2
        if -hl <= q[0] <= hl + SPOUT_L and V_A - R_O <= q[1] <= V_A + R_I + 0.001:
            return "trough"
        if -hl - CHUTE_L <= q[0] <= hl and V_A + R_I < q[1] <= TW / 2:
            return "floor"
    if p[2] < TOP + DOWN_Z:
        return "down"
    return "air"


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    bp, br = _pose(base, origin, "bottle")
    cp, cr = _pose(base, origin, "cap")
    tray = _pose(base, origin, "tray")
    axis = br[:, 2]
    rel = br.T @ cr
    yaw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))  # the cap's turn on the neck (anticlockwise from above: +)
    d = 0.0 if _EP["last_yaw"] is None else _wrap(yaw - _EP["last_yaw"])
    _EP["last_yaw"] = yaw
    # where the cap sits on the neck's axis: its slide coordinate (0 screwed down, negative: higher), off-axis, tilt
    lip = bp + axis * B_H
    anchor1 = cp + cr @ np.array([0.0, 0.0, CAP_H - CAP_TOPT])
    slide = -float((anchor1 - lip) @ axis)
    radial = float(np.linalg.norm((anchor1 - lip) + slide * axis))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[:, 2] @ axis)))))
    _EP["slide"], _EP["radial"], _EP["ctilt"] = slide, radial, tilt
    if _EP["on"] and not _EP["caught"]:  # first unscrewing, from screwed down
        _EP["turn"] += d
        delta = (RUN_NM if _EP["loose"] else BREAK_NM) / ROT_K + 0.5  # the friction spring's stretch: stick-slip
        if _EP["turn"] - _EP["tgt"] > delta:
            _EP["tgt"] = _EP["turn"] - delta
        elif _EP["turn"] - _EP["tgt"] < -delta:
            _EP["tgt"] = _EP["turn"] + delta
        if _EP["turn"] > BREAK_NM / ROT_K:
            _EP["loose"] = True
        _thread_target_attr().Set(float(_wrap(-_EP["tgt"])))  # the spring's anchor (the joint turns about -z)
        set_slide_target("cap_thread", -max(0.0, _EP["turn"]) / 360.0 * PITCH)  # it rises as it is unscrewed
        if _EP["turn"] >= OFF_DEG:
            _EP["on"], _EP["off"], _EP["opened"] = False, True, True
            set_thread_on("cap_thread", False)
    elif _EP["caught"]:  # screwed back on: clockwise turns draw it down to its seat, against the thread's friction
        _EP["recap_turn"] -= d
        _EP["ryaw"] += d
        delta = RUN_NM / ROT_K + 0.5
        if _EP["ryaw"] - _EP["rtgt"] > delta:
            _EP["rtgt"] = _EP["ryaw"] - delta
        elif _EP["ryaw"] - _EP["rtgt"] < -delta:
            _EP["rtgt"] = _EP["ryaw"] + delta
        _thread_target_attr().Set(float(_wrap(-_EP["rtgt"])))
        target = min(0.0, _EP["x0"] + _EP["recap_turn"] / 360.0 * PITCH)
        if target < _EP["x0"] - 0.003:  # unscrewed off again
            set_thread_on("cap_thread", False)
            _EP.update(caught=False, pre=0.0, recap_turn=0.0)
        else:
            set_slide_target("cap_thread", target)
    else:  # off: it catches its thread when it sits on the neck, lined up, and is turned clockwise
        lined = radial <= CATCH[0] and tilt <= CATCH[1] and CATCH[2][0] <= slide <= CATCH[2][1]
        _EP["pre"] = _EP["pre"] - d if lined else 0.0
        if _EP["pre"] >= CATCH[3]:
            set_slide_target("cap_thread", min(0.0, slide))
            _thread_target_attr().Set(float(_wrap(-yaw)))  # the friction spring starts where the cap is
            set_thread_on("cap_thread", True)
            _EP.update(caught=True, x0=min(0.0, slide), recap_turn=0.0, ryaw=yaw, rtgt=yaw)
    # the tablets
    tabs = [_pose(base, origin, n)[0] for n in TAB_NAMES]
    where = [_where(p, (bp, br), tray) for p in tabs]
    lid_r = tray[1].T @ _pose(base, origin, "lid")[1]
    _EP["lid"] = math.degrees(math.atan2(float(lid_r[2, 1]), float(lid_r[1, 1])))  # 0 shut, 110 standing open
    for i, w in enumerate(where):
        if w == "down":
            if _DEBUG["on"] and _EP["where"][i] != "down":
                print(f"[tab] DOWN t={_DEBUG['steps'] * DT:.2f} tablet {i} at {np.round(tabs[i] * 1000, 1).tolist()} "
                      f"(was {_EP['where'][i]})", file=sys.stderr, flush=True)
            _EP["spilt"] = True
    _EP["where"] = where
    touch = _pad_touch()
    _EP["touch_max"] = max(_EP["touch_max"], touch)
    if touch > TOUCH_N:
        _EP["touched"] = True
    _EP["s"] = {"seated": bool(_EP["caught"] and slide >= SEAT_SLIDE and _EP["recap_turn"] >= 300.0)}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % _DEBUG["every"] == 0:
            tl, trc = _tcps(base, origin)
            fing = []
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                q = as_numpy(rob.data.joint_pos)[0]
                fing.append([round(float(q[i]) * 1000, 1) for i, n in enumerate(rob.joint_names)
                             if "joint7" in n or "joint8" in n])
            lid = _pose(base, origin, "lid")[1]
            spp, spr = _pose(base, origin, "spatula")
            sptip = spp + spr @ np.array([TIP_LOW, 0.0, -BLADE_W / 2])
            spat = (f"spat=[{sptip[0] * 1000:.0f},{sptip[1] * 1000:.0f},{(sptip[2] - TOP - 0.0005 - Z_F) * 1000:.1f}] "
                    f"sp={math.degrees(math.asin(max(-1.0, min(1.0, -float(spr[2, 0]))))):.1f} ")
            lid_deg = math.degrees(math.atan2(float((tray[1].T @ lid)[2, 1]), float((tray[1].T @ lid)[1, 1])))
            ttilt = math.degrees(math.acos(max(-1.0, min(1.0, float(tray[1][2, 2])))))
            cnt = {k: where.count(k) for k in ("bottle", "floor", "trough", "air", "down")}
            print(f"[tab] t={_DEBUG['steps'] * DT:6.2f} turn={_EP['turn']:6.1f} tgt={_EP['tgt']:6.1f} on={_EP['on']} "
                  f"caught={_EP['caught']} recap={_EP['recap_turn']:6.1f} slide={slide * 1000:5.1f} rad={radial * 1000:4.1f} "
                  f"ctilt={tilt:4.1f} bottle={np.round(bp * 1000).astype(int).tolist()} btilt="
                  f"{math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2]))))):5.1f} "
                  f"cap={np.round(cp * 1000).astype(int).tolist()} tray={np.round(tray[0] * 1000).astype(int).tolist()} "
                  f"ttilt={ttilt:4.1f} lid={lid_deg:5.1f} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(trc * 1000).astype(int).tolist()} "
                  f"fing={fing} {spat}cnt={cnt} touch={touch:.3f} lat=spilt:{_EP['spilt']},touch:{_EP['touched']} "
                  f"score={_points()}", file=sys.stderr, flush=True)
            if os.environ.get("TAB_POS") and _DEBUG["steps"] % (_DEBUG["every"] * 4) == 0:
                print("[tabpos] t=%.2f " % (_DEBUG["steps"] * DT) + " ".join(
                    f"{i}:{w}:{p[0] * 1000:.1f},{p[1] * 1000:.1f},{(p[2] - TOP) * 1000:.1f}"
                    for i, (w, p) in enumerate(zip(where, tabs))), file=sys.stderr, flush=True)


# =====================================================================================================================
# Scoring.

def _points() -> float:
    if _EP["spilt"] or _EP["touched"]:
        return 0.0
    where = _EP["where"]
    rest_ok = all(w in ("bottle", "floor") for w in where if w != "trough")
    if where.count("trough") != DOSE or not rest_ok or not _EP["opened"]:
        return 0.0
    shut = _EP["lid"] <= LID_SHUT
    if shut and _EP["s"].get("seated"):
        return 1.0
    return 0.5 if shut else 0.25


def score(objects) -> float:
    """0 for the whole trial if a tablet was ever dropped (on the counter or the floor) or touched by a finger pad.
    Otherwise, with the bottle opened, exactly 10 tablets in the tray's trough and every other tablet in the stock
    bottle or on the tray's counting floor: 1.0 with the trough's lid shut and the bottle's cap screwed back down; 0.5
    with the lid shut; 0.25 with the lid open; else 0."""
    return _points()


def success(objects) -> bool:
    """Full score with everything still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in MOVING)


# =====================================================================================================================
# Scripted demo. Fixed waypoints, built phase by phase; both arms are kept in step at every phase's end.

def _rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _axang(axis, deg):
    """The rotation by deg about axis (Rodrigues)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    a = math.radians(deg)
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def _good_ik(arm, pos, rot):
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4),
                                  (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.35:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {np.round(pos, 4).tolist()}")
    return best[0]


def _joint_path(arm, q0, q1, label, finger, secs=3.0, n=15):
    from yam_arena import ik, station

    out = []
    for k in range(1, n + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[arm])
        out.append(Waypoint(secs / n, tuple(float(v) for v in p), finger, label, r))
    return out


def _ready(arm):
    from yam_arena.constants import READY

    return np.array(READY, float)


UP = {"left": 0.08, "right": 0.0}  # the left hand comes in and leaves this high


def _lead_in(arm, pos, rot, label, finger, secs=3.0, rise=None):
    rise = UP[arm] if rise is None else rise
    up = np.asarray(pos, float) + [0.0, 0.0, rise]
    out = _joint_path(arm, _ready(arm), _good_ik(arm, up, rot), label, finger, secs)
    return out + ([_w(1.2, pos, finger, label, rot)] if rise else [])


def _go_home(arm, pos, rot, finger, secs=3.0, rise=None):
    rise = UP[arm] if rise is None else rise
    up = np.asarray(pos, float) + [0.0, 0.0, rise]
    out = [_w(1.0, up, finger, "rise", rot)] if rise else []
    return out + _joint_path(arm, _good_ik(arm, up, rot), _ready(arm), "back to where it started", finger, secs)


def _w(t, pos, finger, label, rot):
    return Waypoint(float(t), tuple(float(v) for v in pos), float(finger), label, rot)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _hold(ws, t, label="wait"):
    """Hold the last pose for t seconds."""
    if t > 1e-6:
        ws.append(_w(t, ws[-1].pos, ws[-1].finger, label, ws[-1].yaw))


def _sync(a, b, label="wait for the other hand"):
    ta, tb = _dur(a), _dur(b)
    _hold(a, tb - ta, label)
    _hold(b, ta - tb, label)


def _home_hold(arm, secs, finger=FINGER_OPEN):
    from yam_arena import ik, station

    p, r = ik.tcp_pose(_ready(arm), station.BASES[arm])
    return [_w(secs, p, finger, "wait at the start", r)]


# -- the left hand's side grip: pointing right (-y) and 20 deg down, closing along x
L_SIDE = _rx(20.0) @ _rx(-90.0) @ grip_rotation(-math.pi / 2)


def _side_grip(azimuth_deg, down_deg):
    """A side grip whose approach points at azimuth_deg (from +x toward +y) and down_deg below level; the fingers
    close across it, level."""
    az, dn = math.radians(azimuth_deg), math.radians(down_deg)
    a = np.array([math.cos(az) * math.cos(dn), math.sin(az) * math.cos(dn), -math.sin(dn)])
    c = np.array([-math.sin(az), math.cos(az), 0.0])
    return np.column_stack([c, np.cross(a, c), a])


# -- the left hand on the bottle: a side grip round its body from behind it on the left, pointing right and toward the
# robot (azimuth -120 deg) and 20 deg down; rolled about its approach it tips the bottle's mouth forward
L_BOTTLE = L_SIDE
B_IN = L_BOTTLE[:, 2] * 0.045  # the hand comes in this far along its approach
B_GRIP_Z = 0.030  # the left TCP this high on the bottle
B_OPEN = -0.034  # fingers open 6.8 cm round the 5.3 cm bottle
# Gentle squeezes: each finger aims this far inside the part's surface (a full close squirts round parts out of the
# V-jaw and leaves them off-centre)
SQUEEZE = 0.0045
B_HOLD = -(B_R - SQUEEZE)
# -- the right hand on the cap: straight down, turned yaw; it unscrews in four 100 deg strokes
CAP_OPEN = -0.026
CAP_HOLD = -(CAP_R - SQUEEZE)
STROKE, STROKES = 100.0, 5  # (the grip slips about 20 deg a stroke against the thread's friction)
RISE = 0.050  # between strokes the open hand rises this far, clear of the cap, to turn back
CAP_GRIP_DOWN = 0.002  # the right TCP 2 mm below the cap's top (the fingertips reach 12.8 mm past it)
BOTTLE_IN_HAND = np.array([0.0, 0.0])  # where the bottle stands once the left hand holds it, from where it was


def _r_cap(yaw_deg):
    return grip_rotation(math.radians(yaw_deg))


CAP_SPOT = np.array([0.31, 0.065, TOP + 0.0005 + CAP_H - CAP_GRIP_DOWN])  # where the right hand sets the cap down
R_SIDE = _rx(-20.0) @ _rx(90.0) @ grip_rotation(math.pi / 2)  # the right hand's side grip: pointing left, 20 deg down
R_GRIP_Z = 0.045  # the right TCP this high on the bottle
R_HOLD = B_HOLD
POUR_AT, POUR_TILT = np.array([0.23, -0.035, TOP + 0.085]), 135.0  # the right TCP pouring; the bottle rolled this far
TIP_N = 12  # tipped over in this many steps of 0.4 s
SAW = (0.010, 0.25, 0.12, 16)  # then shaken: +-10 mm along its axis, 0.25 s out and 0.12 s back, 16 times
RECAP_START = 0.003  # the cap set on the neck this high above its seat (on the thread's start)


# -- the right hand on the spatula: straight down on its handle, closing across the handle's 11 mm
SPAT_GRIP = np.array([-0.045, 0.0, BLADE_W / 2 - 0.016])  # the right TCP in the spatula frame: the fingers round the whole handle
SPAT_OPEN = -0.020
SPAT_HOLD = FINGER_CLOSED  # closed hard on the 11 mm handle
SPAT_IN_SLOT = np.array([SPAT_X0, TY + SLOT_V, TOP + 0.0005 + SLOT_Z0 + BLADE_W / 2 + 0.0004])
SPAT_HANG = 6.0  # held by its handle, the spatula hangs this many degrees tip-down (the grip gives): aim against it
SPAT_TWIST = 3.0  # measured: held by the whole handle, the blade sits about 2.3-3.6 deg flatter than the hand
TIP_LOW = BLADE_L - 0.0095  # the blade's lowest point when pitched: under its round tip's centre
TIP_H = 0.001  # the blade's lower edge this high over the floor at its tip
SWEEP_DZ = -0.002  # aim the blade's tip this much lower: it then rests on the floor
RIB_Y0, RIB_Y1 = TY + V_A + R_I + RIB_W + 0.006, TY + V_A + R_I - 0.007  # blade: start rising; end, over the trough
# The strokes, tuned to where the poured tablets lie (fixed waypoints): the blade's pitch (deg), the x of its tip and
# the y where it comes down beside the group; it then pushes the group toward the trough (-y), up the rib and in.
SWEEPS = [(6.0, 0.249, -0.0289), (5.0, 0.258, -0.0476), (6.5, 0.276, -0.028)]
SPAT_REST = np.array([0.15, -0.235, TOP + 0.0005 + BLADE_W / 2 + 0.010])  # laid down right of the tray
# -- the right hand on the lid: fingers closed, pushing with their thin face
LID_PUSH = (TX, TY + HINGE_V - 0.048, TY + HINGE_V + 0.020, TOP + 0.065)  # from beside the open lid, past its hinge
# -- the left hand steadies the tray, pinching its left wall while the right counts
TRAY_HOLD = np.array([TX + 0.07, TY + TW / 2 - WT / 2, TOP + 0.0005 + WALL_TOP - 0.006])  # the left TCP, astride the wall
L_PRESS = grip_rotation(0.0)  # straight down, the fingers closing across the left wall: pinching it
BOTTLE_SET = np.array([-0.001, -0.003])  # measured: where the bottle stands after the pour, from BOTTLE_BACK
BOTTLE_HELD = np.array([0.168, 0.0805])  # measured: where the bottle's axis is once the left hand holds it there


def _spat(origin, pitch=0.0, twist=0.0):
    """The right TCP's pose holding the spatula with its blade/handle junction at origin, its tip pitched down; the
    hand pitched twist degrees more (the spatula turns that much back in the fingers when its tip is on the floor)."""
    return np.asarray(origin, float) + _ry(pitch) @ SPAT_GRIP, _ry(pitch + twist) @ grip_rotation(0.0)


def _phase_open(left, right):
    """The left hand takes the bottle round its body; the right unscrews the cap and sets it down aside."""
    bg = np.array([BX0, BY0, TOP + B_GRIP_Z])
    left += _lead_in("left", bg - B_IN, L_BOTTLE, "beside the stock bottle", B_OPEN)
    left += [_w(1.0, bg, B_OPEN, "in round its body", L_BOTTLE),
             _w(0.6, bg, B_HOLD, "hold the bottle", L_BOTTLE)]
    cap_top = np.array([BX0 + BOTTLE_IN_HAND[0], BY0 + BOTTLE_IN_HAND[1], TOP + 0.0005 + CAP_SEATED_Z + CAP_H])
    at = cap_top - [0.0, 0.0, CAP_GRIP_DOWN]
    y0 = -STROKE / 2
    right += _lead_in("right", at + [0.0, 0.0, 0.05], _r_cap(y0), "over the cap", FINGER_OPEN)
    _sync(left, right)
    turned = 0.0
    p = at
    for k in range(STROKES):
        p = at + [0.0, 0.0, turned / 360.0 * PITCH]
        right += [_w(0.8 if k == 0 else 0.5, p, FINGER_OPEN, "down round the cap", _r_cap(y0)),
                  _w(0.5, p, CAP_HOLD, "grip the cap", _r_cap(y0))]
        for j in range(1, 5):
            right.append(_w(0.4, p + [0.0, 0.0, STROKE * j / 4 / 360.0 * PITCH], CAP_HOLD,
                            "turn it anticlockwise", _r_cap(y0 + STROKE * j / 4)))
        turned += STROKE
        p = at + [0.0, 0.0, turned / 360.0 * PITCH]
        if k < STROKES - 1:  # let go wide, rise clear of the cap, turn back
            right += [_w(0.5, p, FINGER_OPEN, "let go", _r_cap(y0 + STROKE)),
                      _w(0.3, p, FINGER_OPEN, "let go", _r_cap(y0 + STROKE)),
                      _w(0.6, p + [0.0, 0.0, RISE], FINGER_OPEN, "rise clear", _r_cap(y0 + STROKE)),
                      _w(0.6, p + [0.0, 0.0, RISE], FINGER_OPEN, "turn back", _r_cap(y0)),
                      ]
    yaw_end = _r_cap(y0 + STROKE)
    right += [_w(0.8, p + [0.0, 0.0, 0.03], CAP_HOLD, "lift the cap off", yaw_end),
              _w(1.5, CAP_SPOT + [0.0, 0.0, 0.03], CAP_HOLD, "carry it aside", yaw_end),
              _w(0.8, CAP_SPOT + [0.0, 0.0, 0.002], CAP_HOLD, "set it down", yaw_end),
              _w(0.4, CAP_SPOT + [0.0, 0.0, 0.002], CAP_OPEN, "let go", yaw_end),
              _w(0.8, CAP_SPOT + [0.0, 0.0, 0.06], CAP_OPEN, "rise", yaw_end)]
    right += _go_home("right", CAP_SPOT + [0.0, 0.0, 0.06], yaw_end, CAP_OPEN, 2.5)
    _sync(left, right)
    return bg


def _pour_rot(roll):
    """The right hand's side grip rolled roll deg about its approach axis: the bottle's mouth swings forward, right
    (toward the trough) and down."""
    return _axang(R_SIDE[:, 2], roll) @ R_SIDE


def _phase_pour(left, right, bg):
    """The left hand lets go of the open bottle; the right takes it round its upper body, pours the tablets out onto
    the counting floor (tipping it as far as the wrist turns and shaking them out) and stands it back."""
    left += [_w(0.5, bg, B_OPEN, "let go of the open bottle", L_BOTTLE),
             _w(0.8, bg - B_IN, B_OPEN, "draw back", L_BOTTLE)]
    left += _go_home("left", bg - B_IN, L_BOTTLE, B_OPEN, 2.5)
    _sync(left, right)
    rg = np.array([BX0 + BOTTLE_IN_HAND[0], BY0 + BOTTLE_IN_HAND[1], TOP + 0.0005 + R_GRIP_Z])
    rb = np.array([BOTTLE_BACK[0], BOTTLE_BACK[1], rg[2]])
    r_in = R_SIDE[:, 2] * 0.045
    right += _lead_in("right", rg - r_in, R_SIDE, "beside the open bottle", B_OPEN)
    right += [_w(1.0, rg, B_OPEN, "in round its body", R_SIDE),
              _w(0.6, rg, R_HOLD, "take the bottle", R_SIDE),
              _w(1.0, rg + [0.0, 0.0, 0.05], R_HOLD, "lift it", R_SIDE),
              _w(1.5, POUR_AT, R_HOLD, "over the counting tray", R_SIDE)]
    full = _pour_rot(POUR_TILT)
    axis = (full @ np.linalg.inv(R_SIDE))[:, 2]  # the bottle's axis, base to mouth, tipped
    for j in range(1, TIP_N + 1):
        right.append(_w(0.4, POUR_AT, R_HOLD, "tip it", _pour_rot(POUR_TILT * j / TIP_N)))
    right.append(_w(0.8, POUR_AT, R_HOLD, "let them slide to the shoulder", full))
    a, slow, fast, n = SAW
    right.append(_w(0.4, POUR_AT - axis * a, R_HOLD, "shake them out", full))
    for _ in range(int(n)):  # eased toward the mouth, jerked back: the tablets slide on over the shoulder and out
        right += [_w(slow, POUR_AT + axis * a, R_HOLD, "shake them out", full),
                  _w(fast, POUR_AT - axis * a, R_HOLD, "shake them out", full)]
    right += [_w(1.0, POUR_AT, R_HOLD, "let them settle", full),
              _w(1.5, POUR_AT + [0.0, 0.0, 0.01], R_HOLD, "turn it upright", R_SIDE),
              _w(1.5, rb + [0.0, 0.0, 0.05], R_HOLD, "back beside the tray", R_SIDE),
              _w(1.0, rb + [0.0, 0.0, 0.004], R_HOLD, "stand it down", R_SIDE),
              _w(0.5, rb + [0.0, 0.0, 0.004], B_OPEN, "let go", R_SIDE),
              _w(0.8, rb - r_in + [0.0, 0.0, 0.004], B_OPEN, "draw back", R_SIDE)]
    right += _go_home("right", rb - r_in + [0.0, 0.0, 0.004], R_SIDE, B_OPEN, 2.5)
    _sync(left, right)


def _phase_count(left, right):
    """The right hand takes the spatula and sweeps the tablets into the trough a group at a time, then lays it down."""
    left += _lead_in("left", TRAY_HOLD + [0.0, 0.0, 0.03], L_PRESS, "over the tray's left wall", SPAT_OPEN, rise=0.0)
    left += [_w(1.0, TRAY_HOLD, SPAT_OPEN, "down astride the tray's left wall", L_PRESS),
             _w(0.5, TRAY_HOLD, FINGER_CLOSED, "pinch the wall: hold the tray steady", L_PRESS)]
    g, r = _spat(SPAT_IN_SLOT)
    right += _lead_in("right", g + [0.0, 0.0, 0.05], r, "over the spatula's handle", SPAT_OPEN)
    g_up, r_up = _spat(SPAT_IN_SLOT + [0.0, 0.0, 0.065])
    right += [_w(1.0, g, SPAT_OPEN, "down round the handle", r),
              _w(0.5, g, SPAT_HOLD, "take the spatula", r),
              _w(1.5, g_up, SPAT_HOLD, "lift it up out of its slot", r_up)]
    floor_z = TOP + 0.0005 + Z_F
    for pitch, xt, ys in SWEEPS:
        sp, cp = math.sin(math.radians(pitch)), math.cos(math.radians(pitch))
        oz = floor_z + TIP_H + SWEEP_DZ + (BLADE_W / 2) * cp + TIP_LOW * sp  # the junction's centre
        ox = xt - TIP_LOW * cp

        def at(y, dz=0.0, hang=0.0):
            return _spat((ox, y, oz + dz), pitch - hang, SPAT_TWIST if not hang else 0.0)
        a, ra = at(ys)
        a2, ra2 = at(ys, 0.03, SPAT_HANG)
        b, rb = at(RIB_Y0)
        c, rc = at(RIB_Y1, 0.003)
        right += [_w(1.5, a2, SPAT_HOLD, "over the tablets", ra2),
                  _w(0.8, a, SPAT_HOLD, "blade down beside a group", ra),
                  _w(2.0, b, SPAT_HOLD, "sweep the group toward the trough", rb),
                  _w(1.0, c, SPAT_HOLD, "over the rib into the trough", rc),
                  _w(0.6, at(RIB_Y1, 0.033, SPAT_HANG)[0], SPAT_HOLD, "lift the blade", at(RIB_Y1, 0.033, SPAT_HANG)[1])]
    g2, r2 = _spat(SPAT_REST)
    right += [_w(1.5, g2 + [0.0, 0.0, 0.08], SPAT_HOLD, "carry the spatula aside, over the open lid", r2),
              _w(1.0, g2 + [0.0, 0.0, 0.04], SPAT_HOLD, "lower it", r2),
              _w(0.8, g2, SPAT_HOLD, "lay it down", r2),
              _w(0.4, g2, SPAT_OPEN, "let go", r2),
              _w(0.6, g2 + [0.0, 0.0, 0.04], SPAT_OPEN, "rise", r2)]
    _sync(left, right)


def _phase_lid(left, right):
    """The right hand pushes the trough's lid over: it falls shut."""
    x, y0, y1, z = LID_PUSH
    r = grip_rotation(math.pi / 2)  # fingers closing along x: their thin face pushes along y
    right += [_w(1.5, (x, y0, z + 0.04), FINGER_CLOSED, "beside the open lid", r),
              _w(0.8, (x, y0, z), FINGER_CLOSED, "down beside it", r),
              _w(1.5, (x, y1, z), FINGER_CLOSED, "push the lid over: it falls shut", r),
              _w(0.8, (x, y1, z + 0.05), FINGER_CLOSED, "rise", r)]
    _sync(left, right)
    left += [_w(0.5, TRAY_HOLD, SPAT_OPEN, "let the tray go", L_PRESS),
             _w(0.8, TRAY_HOLD + [0.0, 0.0, 0.04], SPAT_OPEN, "rise", L_PRESS)]
    left += _go_home("left", TRAY_HOLD + [0.0, 0.0, 0.04], L_PRESS, SPAT_OPEN, 2.5, rise=0.0)
    _sync(left, right)


def _phase_recap(left, right):
    """The left hand takes the stock bottle round its body; the right takes the cap and screws it back on, clockwise.
    """
    bg = np.array([BOTTLE_BACK[0] + BOTTLE_SET[0], BOTTLE_BACK[1] + BOTTLE_SET[1], TOP + B_GRIP_Z])
    left += _lead_in("left", bg - B_IN, L_BOTTLE, "beside the stock bottle", B_OPEN)
    left += [_w(1.0, bg, B_OPEN, "in round its body", L_BOTTLE),
             _w(0.6, bg, B_HOLD, "hold the bottle", L_BOTTLE)]
    cap_top_on = np.array([BOTTLE_HELD[0], BOTTLE_HELD[1], TOP + 0.0005 + CAP_SEATED_Z + CAP_H])
    pick = CAP_SPOT + [0.0, 0.0, 0.0]
    y0 = STROKE / 2
    right += [_w(1.5, pick + [0.0, 0.0, 0.05], CAP_OPEN, "over the cap", _r_cap(y0)),
              _w(0.8, pick, CAP_OPEN, "down round it", _r_cap(y0)),
              _w(0.5, pick, CAP_HOLD, "take the cap", _r_cap(y0)),
              _w(0.8, pick + [0.0, 0.0, 0.04], CAP_HOLD, "lift it", _r_cap(y0))]
    start = cap_top_on - [0.0, 0.0, CAP_GRIP_DOWN] + [0.0, 0.0, RECAP_START]
    right += [_w(1.5, start + [0.0, 0.0, 0.06], CAP_HOLD, "over the bottle's neck", _r_cap(y0)),
              _w(0.8, start, CAP_HOLD, "set it on the neck", _r_cap(y0))]
    turned = 0.0
    for k in range(STROKES):
        depth = max(0.0, min(RECAP_START, (turned - 20.0) / 360.0 * PITCH))
        p = start - [0.0, 0.0, depth]
        if k > 0:
            right += [_w(0.5, p, FINGER_OPEN, "down round the cap", _r_cap(y0)),
                      _w(0.5, p, CAP_HOLD, "grip the cap", _r_cap(y0))]
        for j in range(1, 5):
            dd = max(0.0, min(RECAP_START, (turned + STROKE * j / 4 - 20.0) / 360.0 * PITCH))
            right.append(_w(0.3, start - [0.0, 0.0, dd], CAP_HOLD, "screw it on, clockwise",
                            _r_cap(y0 - STROKE * j / 4)))
        turned += STROKE
        p = start - [0.0, 0.0, max(0.0, min(RECAP_START, (turned - 20.0) / 360.0 * PITCH))]
        right += [_w(0.5, p, FINGER_OPEN, "let go", _r_cap(y0 - STROKE)),
                  _w(0.5, p + [0.0, 0.0, RISE], FINGER_OPEN, "rise clear", _r_cap(y0 - STROKE))]
        if k < STROKES - 1:
            right += [_w(0.6, p + [0.0, 0.0, RISE], FINGER_OPEN, "turn back", _r_cap(y0))]
    right += _go_home("right", p + [0.0, 0.0, 0.03], _r_cap(y0 - STROKE), CAP_OPEN, 2.5)
    _sync(left, right)
    left += [_w(0.5, bg, B_OPEN, "let go of the closed bottle", L_BOTTLE),
             _w(0.8, bg - B_IN, B_OPEN, "draw back", L_BOTTLE)]
    left += _go_home("left", bg - B_IN, L_BOTTLE, B_OPEN, 2.5)
    _sync(left, right)


def demo():
    left, right = [], []
    bg = _phase_open(left, right)
    _phase_pour(left, right, bg)
    _phase_count(left, right)
    _phase_lid(left, right)
    _phase_recap(left, right)
    _hold(left, 3.0, "hold still")
    _hold(right, 3.0, "hold still")
    return {"left": left, "right": right}
