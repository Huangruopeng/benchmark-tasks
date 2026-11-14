"""A stainless steel set-up tray on a dental bracket table. On it lie a breech-loading aspirating dental syringe, resting
level on a silicone syringe rest with its loading window up and its piston part way in, and a cartridge rack holding the
1.8 ml glass anaesthetic cartridge chosen for the patient, lying across the robot's view (its dark rubber stopper at
the near end, its silver aluminium cap at the far end).

Task (the instruction gives the goal only): load the syringe with the cartridge, ready for the needle, and leave it
lying on its rest. The way it is done
at the chair: draw the piston fully back, drop a cartridge into the window with its rubber stopper toward the piston,
and push the thumb ring firmly so the piston's harpoon (a barbed tip) bites into the stopper; a gentle pull on the ring
then shows the stopper follows. Pushing much harder than the harpoon needs drives the stopper into the cartridge.

Two hands are how it is done (one holds the barrel's front end against the push on the thumb ring), but the task does
not depend on it: a syringe pushed without being held slides off its rest and can wedge, and then the harpoon sets too.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_an_anaesthetic_cartridge_into_a_dental_syringe.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the set-up tray (stainless steel, fixed on the bracket table) ------------------------------------------------------
TX0, TX1, TY0, TY1 = 0.17, 0.37, -0.14, 0.14
TRAY_T, RIM_H = 0.001, 0.015
ZF = TOP + TRAY_T  # the tray's floor
# -- the syringe rest: two silicone blocks with a slot the barrel lies in ------------------------------------------------
REST_H, SLOT_D, SLOT_W = 0.016, 0.003, 0.0130
REST_YS = (-0.020, -0.058)  # the blocks' middles, in the syringe's frame (along its axis)
# -- the syringe (frame body, steel): its origin on the barrel's axis at the window's front end; +y toward the needle hub
SX, SY = 0.31, 0.065
BAR_R, BORE_R = 0.00625, 0.0047  # barrel: outer radius (12.5 mm), bore (9.4 mm)
AXIS_Z = ZF + REST_H - SLOT_D + BAR_R
WIN0, WIN1 = -0.070, 0.0  # the loading window, along the axis (70 mm long)
BEVEL = 0.0015  # the window's edges: bevelled this far down and out
FRONT_L = 0.016  # the barrel's closed front section (where it is held)
HUB_R, HUB_L = 0.004, 0.010  # the threaded needle adapter
COLLAR0, COLLAR1 = -0.078, -0.070  # the barrel's back end, through which the piston runs
WING_Y, WING_W, WING_T, WING_H = -0.0755, 0.038, 0.005, 0.003  # the finger grip
SYR_MASS = 0.052
# -- the piston (plunger, steel): its origin at the harpoon's tip --------------------------------------------------------
TIP0, TIP_BACK, TIP_FWD = -0.035, -0.071, -0.010  # the tip's place along the axis: at the start, drawn back, pushed in
HARP_L, HARP_R, ROD_R = 0.003, 0.0008, 0.002
RING_C = 0.095  # the thumb ring's middle, behind the tip
RING_RO, RING_RI, RING_T = 0.014, 0.010, 0.004  # 28 mm across, 20 mm hole, 4 mm thick
PLUNGER_MASS = 0.016
# -- the cartridges (glass, 1.8 ml): origin on the axis at the middle; +x toward the stopper end -------------------------
CART_L, CART_R, CART_BORE = 0.065, 0.004325, 0.003425
CAP_L, CAP_R, NECK_L, NECK_R = 0.0035, 0.0042, 0.0025, 0.0035
STOP_IN, STOP_L, STOP_R = 0.003, 0.0076, 0.0034  # the stopper: inset from the glass's open end, length, radius
STOP_FACE = CART_L / 2 - STOP_IN  # the stopper's face, in the cartridge's frame
STOP_OUT = 0.002  # pulled hard, the stopper can come this far out past the glass's rim (it then still seals)
HOLLOW = 0.018  # the glass's open end is modelled hollow this deep (the piston's head goes in)
CART_MASS, STOPPER_MASS = 0.0056, 0.0004
# -- the cartridge rack (stainless, fixed): two notched rails across the cartridges' ends -------------------------------
RACK_X, RACK_YS, CART_Z = 0.215, (-0.070, -0.040, -0.010), ZF + 0.020  # the rack's three notches
CART_YS = (RACK_YS[-1],)  # the one cartridge laid out for this patient, in the notch nearest the syringe
RAIL_DX, RAIL_W, NOTCH = 0.0235, 0.008, 0.006
N_CART = len(CART_YS)

# -- how things move ----------------------------------------------------------------------------------------------------
K_P, SLIP_P, D_P, P_CAP = 1500.0, 0.001, 2.0, 12.0  # the piston in the barrel: about 1.5 N to slide; pull cap (test)
K_DET, PIERCE_D = 15000.0, 0.001  # the rubber against the harpoon's point: N/m; it gives (the barb enters) at 15 N
K_BARB, BARB_IN, BARB_HOLD, TORN_D = 20000.0, HARP_L, 40.0, 0.0012  # the barb in: held up to 40 N; out below 1.2 mm
K_FWD, ADV_BREAK, K_VAC, D_STOP = 15000.0, 35.0, 2000.0, 2.0  # stopper: stick spring, slips forward past 35 N; vacuum
SLIP_F = ADV_BREAK / K_FWD
CAPTURE_D, CAPTURE_R, CAPTURE_DEG = (0.0, 0.007), 0.0015, 10.0  # the harpoon's tip at the stopper's face
DEEP_D = 0.004  # caught already this deep (a fast, hard push in one control step): through at once
# -- scoring ------------------------------------------------------------------------------------------------------------
ADV_MAX = 0.002  # the stopper driven in further than this: the cartridge spoilt (latched)
PULL, FOLLOW_MIN, TEST_K = 0.008, 0.002, 50000.0  # the scorer's aspiration test: pull the ring 8 mm (the cartridge's
# 5 mm of play in the window, then 3 mm): the stopper must slide back at least 2 mm in its glass
SEAT_XZ, SEAT_DEG = 0.0015, 8.0
CLEAR = 0.08  # hands clear: both TCPs at least this far from the syringe's axis
REST_XZ, REST_Y, REST_DEG = 0.003, 0.015, 5.0  # on its rest: across and up, along, tilt
SUCCESS_HOLD_S = 1.0

STEEL, STEEL_D, DARKSTEEL = (0.78, 0.79, 0.81), (0.60, 0.61, 0.63), (0.42, 0.43, 0.45)
TRAY_C, TRAY_L = (0.82, 0.83, 0.85), (0.70, 0.71, 0.73)
SILICONE = (0.20, 0.45, 0.70)
GLASS, LABEL, BAND, RUBBER, ALU = (0.86, 0.90, 0.92), (0.97, 0.97, 0.95), (0.80, 0.10, 0.12), (0.10, 0.10, 0.11), \
    (0.80, 0.81, 0.84)
MARK = (0.15, 0.15, 0.15)


# -- the bodies --------------------------------------------------------------------------------------------------------

def _tray() -> Body:
    """The set-up tray: a 1 mm steel floor with a 15 mm rim. Origin: the floor's middle, on the table."""
    cx, cy, lx, ly = (TX0 + TX1) / 2, (TY0 + TY1) / 2, TX1 - TX0, TY1 - TY0
    shapes = [BoxShape((0.0, 0.0, TRAY_T / 2), (lx, ly, TRAY_T), TRAY_C)]
    t = 0.0012
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx / 2 - t / 2), 0.0, RIM_H / 2), (t, ly, RIM_H), TRAY_L))
        shapes.append(BoxShape((0.0, s * (ly / 2 - t / 2), RIM_H / 2), (lx, t, RIM_H), TRAY_L))
        shapes.append(BoxShape((s * (lx / 2 + 0.002), 0.0, RIM_H), (0.005, ly + 0.009, 0.0015), TRAY_C, collide=False))
        shapes.append(BoxShape((0.0, s * (ly / 2 + 0.002), RIM_H), (lx + 0.009, 0.005, 0.0015), TRAY_C, collide=False))
    return Body.build("tray", pos=(cx, cy, TOP), mass=0.45, kinematic=True, color=TRAY_C, shapes=shapes,
                      friction=(0.3, 0.25))


def _rest() -> Body:
    """The silicone syringe rest: two blocks, each with a slot across its top for the barrel. Origin: on the tray's
    floor under the syringe's origin."""
    shapes = []
    side = (0.030 - SLOT_W) / 2
    for y in REST_YS:
        shapes.append(BoxShape((0.0, y, (REST_H - SLOT_D) / 2), (0.030, 0.010, REST_H - SLOT_D), SILICONE))
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((s * (SLOT_W / 2 + side / 2), y, REST_H - SLOT_D / 2), (side, 0.010, SLOT_D),
                                   SILICONE))
    shapes.append(BoxShape((0.0, sum(REST_YS) / 2, 0.001), (0.030, REST_YS[0] - REST_YS[1] + 0.010, 0.002), SILICONE))
    return Body.build("rest", pos=(SX, SY, ZF), mass=0.05, kinematic=True, color=SILICONE, shapes=shapes,
                      friction=(0.8, 0.7))


def _rack() -> Body:
    """The cartridge rack: two notched steel rails across the cartridges' ends on a base plate. Origin: on the tray's
    floor under the middle cartridge's middle."""
    shapes = [BoxShape((0.0, 0.0, 0.001), (2 * RAIL_DX + RAIL_W + 0.006, 0.100, 0.002), STEEL_D)]
    top = CART_Z - ZF - math.sqrt(CART_R ** 2 - (NOTCH / 2) ** 2)  # the notches' edges, where the glass rests
    ys = [y - RACK_YS[1] for y in RACK_YS]
    for s in (-1.0, 1.0):
        x = s * RAIL_DX
        shapes.append(BoxShape((x, 0.0, (top - 0.004) / 2 + 0.001), (RAIL_W, 0.100, top - 0.004), STEEL))
        edges = [-0.050] + [e for y in ys for e in (y - NOTCH / 2, y + NOTCH / 2)] + [0.050]
        for a, b in zip(edges[::2], edges[1::2]):
            shapes.append(BoxShape((x, (a + b) / 2, top - 0.002), (RAIL_W, b - a, 0.004), STEEL))
    return Body.build("rack", pos=(RACK_X, RACK_YS[1], ZF), mass=0.12, kinematic=True, color=STEEL, shapes=shapes,
                      friction=(0.4, 0.35))


def _syringe() -> Body:
    """The syringe's frame: barrel, loading window, front section and needle adapter, back end and finger grip. The
    window is a trough open on top (collision: a U-channel); the visual barrel is an octagon round it."""
    shapes = []
    wl, wc = WIN1 - WIN0, (WIN0 + WIN1) / 2
    wall = BAR_R - BORE_R
    # the window: a trough 9.4 mm wide, its walls up to the axis (collision only), drawn as half an octagon
    shapes.append(BoxShape((0.0, wc, -BORE_R - wall / 2), (2 * BAR_R, wl, wall), None))
    for s in (-1.0, 1.0):  # the walls, their inner top edges bevelled 45 degrees (a lead-in for the cartridge)
        shapes.append(BoxShape((s * (BORE_R + wall / 2), wc, -(BORE_R + BEVEL) / 2), (wall, wl, BORE_R - BEVEL), None))
        n = np.array([-s, 0.0, 1.0]) / math.sqrt(2.0)
        mid = np.array([s * (BORE_R + BEVEL / 2), 0.0, -BEVEL / 2]) - n * 0.0005
        shapes.append(plate((mid[0], WIN0, mid[2]), (mid[0], WIN1, mid[2]), BEVEL * math.sqrt(2.0), 0.001, n, None))
    a = 2 * BAR_R * math.tan(math.pi / 8)
    for ang in (-90.0, -45.0, 0.0, 45.0, 90.0):  # half an octagon round the trough (the barrel's lower half)
        th = math.radians(ang)
        n = np.array([math.sin(th), 0.0, -math.cos(th)])  # outward normal, in the x-z plane
        c = n * (BAR_R - wall / 2)
        width = a
        if abs(ang) == 90.0:
            c, width = c + np.array([0.0, 0.0, -a / 4]), a / 2
        shapes.append(plate((c[0], WIN0, c[2]), (c[0], WIN1, c[2]), width, wall, n, STEEL, collide=False))
    for s in (-1.0, 1.0):  # the window's bright edges
        shapes.append(BoxShape((s * (BAR_R - wall / 2), wc, 0.0002), (wall + 0.0004, wl, 0.0006), (0.90, 0.91, 0.93),
                               collide=False))
    # the front section and the needle adapter (threaded)
    shapes.append(BoxShape((0.0, FRONT_L / 2, 0.0), (2 * BAR_R, FRONT_L, 2 * BAR_R), None))
    shapes.append(CylShape((0.0, FRONT_L / 2, 0.0), BAR_R, FRONT_L, STEEL, collide=False, axis="Y"))
    for k in range(5):
        shapes.append(CylShape((0.0, 0.002 + 0.003 * k, 0.0), BAR_R + 0.0002, 0.0008, STEEL_D, collide=False, axis="Y"))
    shapes.append(CylShape((0.0, FRONT_L + HUB_L / 2, 0.0), HUB_R, HUB_L, STEEL, axis="Y"))
    for k in range(6):
        shapes.append(CylShape((0.0, FRONT_L + 0.001 + 0.0016 * k, 0.0), HUB_R + 0.0003, 0.0006, DARKSTEEL,
                               collide=False, axis="Y"))
    shapes.append(CylShape((0.0, FRONT_L + HUB_L + 0.0005, 0.0), 0.0012, 0.001, DARKSTEEL, collide=False, axis="Y"))
    # the back end (the piston runs through it) and the finger grip
    cl, cc = COLLAR1 - COLLAR0, (COLLAR0 + COLLAR1) / 2
    shapes.append(BoxShape((0.0, cc, 0.0), (2 * BAR_R, cl, 2 * BAR_R), None))
    shapes.append(CylShape((0.0, cc, 0.0), BAR_R, cl, STEEL, collide=False, axis="Y"))
    shapes.append(BoxShape((0.0, WING_Y, 0.0), (WING_W, WING_T, WING_H), STEEL))
    for s in (-1.0, 1.0):
        shapes.append(CylShape((s * (WING_W / 2 - 0.004), WING_Y, 0.0), 0.0045, WING_H + 0.0004, STEEL_D,
                               collide=False))
    return Body.build("syringe", pos=(SX, SY, AXIS_Z), mass=SYR_MASS, color=STEEL, shapes=shapes,
                      friction=(0.8, 0.7), com=(0.0, -0.030, -0.001), contact_offset=0.001, solver_iterations=32)


def _plunger() -> Body:
    """The piston: the harpoon (a barbed point) at its front, a 4 mm rod, the thumb ring (soft-grip coated) at its
    back. Origin: the harpoon's tip."""
    shapes = [CylShape((0.0, -HARP_L / 2, 0.0), HARP_R, HARP_L, DARKSTEEL, axis="Y"),
              CylShape((0.0, -0.0012, 0.0), 0.0011, 0.0010, DARKSTEEL, collide=False, axis="Y"),  # the barb
              CylShape((0.0, -HARP_L - (RING_C - RING_RO - HARP_L) / 2, 0.0), ROD_R, RING_C - RING_RO - HARP_L, STEEL,
                       axis="Y")]
    n = 12
    rm, w = (RING_RO + RING_RI) / 2, RING_RO - RING_RI
    seg = 2 * RING_RO * math.tan(math.pi / n) + 0.0004
    for k in range(n):
        th = 2 * math.pi * k / n
        shapes.append(BoxShape((rm * math.cos(th), -RING_C + rm * math.sin(th), 0.0), (w, seg, RING_T),
                               (0.25, 0.27, 0.30), yaw=th))
    return Body.build("plunger", pos=(SX, SY + TIP0, AXIS_Z), mass=PLUNGER_MASS, color=STEEL, shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, -0.070, 0.0), ignore=("syringe",), contact_offset=0.001,
                      solver_iterations=32)


def _cartridge(i: int) -> Body:
    """A 1.8 ml glass cartridge lying in the rack, its stopper end toward the robot (it lies turned half round: in its
    own frame the stopper end is +x). Its open end (the stopper's end) is hollow, so the piston's head can go in; the
    collision shapes are not drawn, the drawn glass is one cylinder (the stopper, a size wider, shows through it)."""
    x_glass0 = -CART_L / 2 + CAP_L + NECK_L
    solid1 = CART_L / 2 - HOLLOW
    shapes = [CylShape(((x_glass0 + solid1) / 2, 0.0, 0.0), CART_R, solid1 - x_glass0, None, axis="X"),
              CylShape(((x_glass0 + CART_L / 2) / 2, 0.0, 0.0), CART_R, CART_L / 2 - x_glass0, GLASS, collide=False,
                       axis="X"),
              CylShape((-CART_L / 2 + CAP_L / 2, 0.0, 0.0), CAP_R, CAP_L, ALU, axis="X"),
              CylShape((-CART_L / 2 + CAP_L + NECK_L / 2, 0.0, 0.0), NECK_R, NECK_L, GLASS, axis="X"),
              CylShape((-CART_L / 2 - 0.0001, 0.0, 0.0), 0.0016, 0.0004, RUBBER, collide=False, axis="X")]  # diaphragm
    n = 8
    rm, wall = (CART_R + CART_BORE) / 2, CART_R - CART_BORE
    side = 2 * CART_R * math.tan(math.pi / n)
    for k in range(n):
        th = 2 * math.pi * k / n
        shapes.append(BoxShape((CART_L / 2 - HOLLOW / 2, rm * math.cos(th), rm * math.sin(th)), (HOLLOW, wall, side),
                               None, quat_xyzw=(math.sin(th / 2), 0.0, 0.0, math.cos(th / 2))))
    shapes.append(CylShape((-0.008, 0.0, 0.0), CART_R + 0.0002, 0.034, LABEL, collide=False, axis="X"))  # label
    shapes.append(CylShape((-0.008, 0.0, 0.0), CART_R + 0.0003, 0.006, BAND, collide=False, axis="X"))  # drug band
    for k in range(6):  # the label's scale toward the stopper end (1 mm steps), for reading the stopper's place
        shapes.append(BoxShape((0.0045 + 0.001 * k, 0.0, CART_R + 0.0003), (0.0003, 0.002 if k % 5 else 0.003, 0.0002),
                               MARK, collide=False))
    return Body.build(f"cartridge_{i + 1}", pos=(RACK_X, CART_YS[i], CART_Z), mass=CART_MASS, color=GLASS,
                      shapes=shapes, friction=(0.35, 0.30), contact_offset=0.001, solver_iterations=32, yaw=math.pi)


def _stopper(i: int) -> Body:
    """The cartridge's rubber stopper (lying turned half round, as its cartridge). Origin: the middle of its face toward
    the open end. It is drawn a size wider than the glass, so its dark rubber shows through the glass."""
    shapes = [CylShape((-STOP_L / 2, 0.0, 0.0), STOP_R, STOP_L, None, axis="X"),
              CylShape((-STOP_L / 2, 0.0, 0.0), CART_R + 0.0002, STOP_L, RUBBER, collide=False, axis="X"),
              CylShape((-0.0003, 0.0, 0.0), STOP_R - 0.0012, 0.0008, (0.20, 0.20, 0.22), collide=False, axis="X")]
    ignore = ("tray", "rest", "rack", "syringe", "plunger") + tuple(f"cartridge_{j + 1}" for j in range(N_CART))
    return Body.build(f"stopper_{i + 1}", pos=(RACK_X - STOP_FACE, CART_YS[i], CART_Z), mass=STOPPER_MASS,
                      color=RUBBER, shapes=shapes, friction=(0.9, 0.8), ignore=ignore, contact_offset=0.0005,
                      solver_iterations=32, damping=(0.5, 0.5), yaw=math.pi)


OBJECTS = [_tray(), _rest(), _rack(), _syringe(), _plunger()] + [_cartridge(i) for i in range(N_CART)] + \
    [_stopper(i) for i in range(N_CART)]
# the piston in the barrel (its friction: a stick-slip spring the task file moves; see _follow)
slider("piston", "syringe", "plunger", anchor=(SX, SY + TIP0, AXIS_Z), axis="Y",
       limits=(TIP_BACK - TIP0, TIP_FWD - TIP0), stiffness=K_P, damping=D_P, target=0.0, max_force=P_CAP)

# -- the stopper's slide in its cartridge (J2) and the harpoon in the stopper (J3): D6 joints -----------------------------
# J2, from the cartridge to its stopper, always on: the stopper slides along the glass's axis only. Its coordinate is how
# far the stopper is driven in from where it started. Forward, a stiff spring holds it (the rubber's grip and the
# sealed liquid) until 35 N, past which it slides in and stays (the task file moves the spring's hold); backward, a soft
# spring (the vacuum drawn in a sealed cartridge) pulls it back where it was.
# J3, from the piston to a stopper, switched on when the harpoon's tip meets the stopper's face square on: the harpoon's
# depth in the rubber is its one free slide. Until the barb is in, the rubber pushes the point back like a stiff spring
# (15 N at 1 mm); past that it gives, the barb goes in to the rod's shoulder and the rubber closes behind it: it then
# holds the stopper to the piston both ways, up to a 40 N pull.
_D6: dict = {}
_author_plain = bodykit._author_sliders


def _x_to(v):
    return bodykit._x_to(v)


def _d6(stage, path, body0, body1, pos0, axis0, pos1, axis1, enabled, limits, play, tilt, k, d, target, cap):
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.CreateLocalRot0Attr(Gf.Quatf(*_x_to(axis0)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in pos1]))
    joint.CreateLocalRot1Attr(Gf.Quatf(*_x_to(axis1)))
    joint.CreateJointEnabledAttr(bool(enabled))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    prim = joint.GetPrim()
    for dof, room in (("transY", play), ("transZ", play), ("rotY", tilt), ("rotZ", tilt)):
        lim = UsdPhysics.LimitAPI.Apply(prim, dof)
        lim.CreateLowAttr(-float(room) if room > 0 else 1.0)
        lim.CreateHighAttr(float(room) if room > 0 else -1.0)
    lim = UsdPhysics.LimitAPI.Apply(prim, "transX")
    lim.CreateLowAttr(float(limits[0]))
    lim.CreateHighAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(prim, "transX")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(k))
    drive.CreateDampingAttr(float(d))
    drive.CreateTargetPositionAttr(float(target))
    drive.CreateMaxForceAttr(float(cap))
    return path


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    if not spec.name.startswith("stopper_"):
        return
    i = int(spec.name.split("_")[1]) - 1
    parent = prim_path.rsplit("/", 1)[0]
    body = f"{prim_path}/body"
    # J2: the cartridge's frame, at the stopper's starting face, along -x (into the liquid)
    _D6[f"j2_{i}"] = _d6(stage, f"{prim_path}/slide", f"{parent}/cartridge_{i + 1}/body", body, (STOP_FACE, 0.0, 0.0),
                         (-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), True, (-STOP_IN - STOP_OUT, 0.045), 0.0, 0.0,
                         K_FWD, D_STOP, 0.0, 1.0e3)
    # J3: the piston's frame at the harpoon's tip, along its axis (+y); the slide is the stopper face's place ahead of
    # the tip (the harpoon's depth is minus that)
    _D6[f"j3_{i}"] = _d6(stage, f"{prim_path}/harpoon", f"{parent}/plunger/body", body, (0.0, 0.0, 0.0),
                         (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), False, (-BARB_IN, 0.010), 0.0006, 3.0,
                         K_DET, 2.0, 0.0, BARB_HOLD)
    _D6.pop("attrs", None)


bodykit._author_sliders = _author_sliders


def _attrs(name):
    """(enabled, target, stiffness) attributes of a D6 joint."""
    cache = _D6.setdefault("attrs", {})
    if name not in cache:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(_D6[name])
        drive = UsdPhysics.DriveAPI(prim, "transX")
        cache[name] = (UsdPhysics.Joint(prim).GetJointEnabledAttr(), drive.GetTargetPositionAttr(),
                       drive.GetStiffnessAttr())
    return cache[name]


def _set(name, on=None, target=None, k=None) -> None:
    if name not in _D6:
        return
    en, tg, st = _attrs(name)
    for attr, v in ((tg, target), (st, k)):
        if v is not None:
            attr.Set(float(v))
    if on is not None:
        en.Set(bool(on))


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"steps": 0, "hold_p": 0.0, "hold": [0.0] * N_CART, "kset": [K_FWD] * N_CART, "advanced": "",
            "j3": None, "caught": False, "pierced": 0, "torn": 0, "test": None, "result": None, "still": 0,
            "last": None, "tcps": None, "s": {}, "follow": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("DENTAL_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    for i in range(N_CART):
        _set(f"j2_{i}", target=0.0, k=K_FWD)
        _set(f"j3_{i}", on=False, target=0.0, k=K_DET)


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


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _deg(u, v) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v))))))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    ps, rs = _pose(base, origin, "syringe")
    pp, rp = _pose(base, origin, "plunger")
    fwd = rs[:, 1]
    # the piston in the barrel: it stays where it is left (stick-slip), unless the scorer's test is pulling it
    p = float(rs[:, 1] @ (pp - ps)) - TIP0
    test = _EP["test"]
    if test is None and abs(p - _EP["hold_p"]) > SLIP_P:
        _EP["hold_p"] = p
        set_slider_drive("piston", target=p)
    # each stopper on its slide in the glass
    carts, stops = [], []
    for i in range(N_CART):
        pc, rc = _pose(base, origin, f"cartridge_{i + 1}")
        pt, rt = _pose(base, origin, f"stopper_{i + 1}")
        carts.append((pc, rc))
        stops.append((pt, rt))
        u = STOP_FACE - float(rc[:, 0] @ (pt - pc))  # how far the stopper is driven in from where it started
        h = _EP["hold"][i]
        if u - h > SLIP_F:  # pushed past its grip: it slides in, and stays
            h = u - SLIP_F
            _EP["hold"][i] = h
            _set(f"j2_{i}", target=h)
        k = K_FWD if u >= h - 0.0001 else K_VAC
        if k != _EP["kset"][i]:
            _set(f"j2_{i}", k=k)
            _EP["kset"][i] = k
        if h > ADV_MAX and not _EP["advanced"]:
            _EP["advanced"] = f"cartridge {i + 1}: stopper driven in {h * 1000:.1f} mm"
    # the harpoon: caught by a stopper's face, the barb in once pushed past the rubber's give, torn out if pulled hard
    j3 = _EP["j3"]
    if j3 is None:
        for i, (pt, rt) in enumerate(stops):
            rel = pt - pp
            ahead = float(rel @ rp[:, 1])
            depth = -ahead
            radial = float(np.linalg.norm(rel - ahead * rp[:, 1]))
            if (CAPTURE_D[0] <= depth <= CAPTURE_D[1] and radial <= CAPTURE_R
                    and _deg(rp[:, 1], -rt[:, 0]) <= CAPTURE_DEG):
                caught = depth >= DEEP_D  # only a point already deep in counts as through: else the rubber pushes back
                _set(f"j3_{i}", target=-BARB_IN if caught else 0.0, k=K_BARB if caught else K_DET, on=True)
                _EP["j3"], _EP["caught"] = i, caught
                _EP["pierced"] += int(caught)
                break
    else:
        pt, rt = stops[j3]
        depth = -float((pt - pp) @ rp[:, 1])
        if not _EP["caught"]:
            if depth >= PIERCE_D:
                _set(f"j3_{j3}", target=-BARB_IN, k=K_BARB)
                _EP["caught"] = True
                _EP["pierced"] += 1
            elif depth < -0.0002:
                _set(f"j3_{j3}", on=False, target=0.0, k=K_DET)
                _EP["j3"] = None
        elif depth < TORN_D:
            _set(f"j3_{j3}", on=False, target=0.0, k=K_DET)
            _EP["j3"], _EP["caught"] = None, False
            _EP["torn"] += 1
    # the end state
    _EP["tcps"] = _tcps(base, origin)
    seated = None
    for i, (pc, rc) in enumerate(carts):
        rel = rs.T @ (pc - ps)
        par = abs(float(rc[:, 0] @ fwd))
        lo, hi = rel[1] - CART_L / 2 * par, rel[1] + CART_L / 2 * par
        if (abs(rel[0]) <= SEAT_XZ and -SEAT_XZ <= rel[2] + (BORE_R - CART_R) <= SEAT_XZ
                and _deg(np.array([par, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])) <= SEAT_DEG
                and WIN0 - 0.001 <= lo and hi <= WIN1 + 0.001):
            seated = (i, float(rc[:, 0] @ fwd) < 0.0)  # (which, stopper toward the piston)
    ring = pp - fwd * RING_C
    a, b = ring - fwd * RING_RO, ps + fwd * (FRONT_L + HUB_L)
    clear = all(_seg_dist(t, a, b) >= CLEAR for t in _EP["tcps"])
    # lying on its rest: the barrel down in both blocks' slots, level, window up
    on_rest = (abs(float(ps[0]) - SX) <= REST_XZ and abs(float(ps[2]) - AXIS_Z) <= REST_XZ
               and abs(float(ps[1]) - SY) <= REST_Y and abs(float(fwd[0])) <= math.sin(math.radians(REST_DEG))
               and abs(float(fwd[2])) <= math.sin(math.radians(REST_DEG)) and float(rs[2, 2]) >= math.cos(math.radians(10.0)))
    pose = np.concatenate([ps, rs[:, 1], rs[:, 2], pp])
    still = _EP["last"] is not None and float(np.abs(pose - _EP["last"]).max()) < 0.0004
    _EP["last"] = pose
    _EP["still"] = _EP["still"] + 1 if still else 0
    ready = seated is not None and seated[1] and on_rest and clear
    # the scorer's aspiration test: with the syringe loaded, on its rest, let go of and still, pull the thumb ring back
    # 8 mm (at most 12 N; the first 5 mm may only slide the cartridge back to the barrel's back end) and see whether the
    # stopper slides back at least 2 mm in its glass; then let the ring back
    if _EP["result"] is not None:
        ref = _EP["result"]["pose"]
        moved = float(np.abs(pose[:9] - ref[:9]).max()) > 0.003
        if not ready or moved or not _EP["caught"] and _EP["result"]["pass"]:
            _EP["result"] = None
    if test is not None:
        test["n"] += 1
        i = test["cart"]
        u = STOP_FACE - float(carts[i][1][:, 0] @ (stops[i][0] - carts[i][0]))
        if not ready:
            set_slider_drive("piston", target=test["p0"], stiffness=K_P)
            _EP["test"], _EP["hold_p"] = None, test["p0"]
        elif test["n"] == 16:
            test["follow"], test["pulled"] = test["u0"] - u, test["p0"] - p
            set_slider_drive("piston", target=test["p0"])
        elif test["n"] >= 28:
            set_slider_drive("piston", target=p, stiffness=K_P)
            _EP["hold_p"] = p
            ok = test["follow"] >= FOLLOW_MIN
            _EP["result"] = {"pass": ok, "pose": pose.copy()}
            _EP["follow"] = (test["pulled"], test["follow"])
            _EP["test"] = None
    elif ready and _EP["result"] is None and _EP["still"] >= 10:
        i = seated[0]
        u0 = STOP_FACE - float(carts[i][1][:, 0] @ (stops[i][0] - carts[i][0]))
        _EP["test"] = {"n": 0, "cart": i, "p0": p, "u0": u0, "follow": 0.0, "pulled": 0.0}
        set_slider_drive("piston", target=p - PULL, stiffness=TEST_K)
    _EP["s"] = {"seated": seated, "on_rest": on_rest, "clear": clear,
                "aspirates": bool(_EP["result"] and _EP["result"]["pass"] and _EP["caught"])}
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        tl, tr = _EP["tcps"]
        mm = lambda v: np.round(np.asarray(v) * 1000, 1).tolist()  # noqa: E731
        j = _EP["j3"]
        depth = -float((stops[j][0] - pp) @ rp[:, 1]) if j is not None else float("nan")
        us = [round((STOP_FACE - float(carts[i][1][:, 0] @ (stops[i][0] - carts[i][0]))) * 1000, 2)
              for i in range(N_CART)]
        c2 = rs.T @ (carts[N_CART - 1][0] - ps)
        print(f"[dental] t={_EP['steps'] / 20:5.1f}s piston={p * 1000:6.1f}mm hold={_EP['hold_p'] * 1000:6.1f} "
              f"j3={j} caught={_EP['caught']} depth={depth * 1000:5.2f}mm (F~{max(depth, 0) * K_DET:4.1f}N pre) "
              f"stopper_u={us} holds={[round(h * 1000, 2) for h in _EP['hold']]} adv='{_EP['advanced']}' "
              f"seated={seated} cart3_in_syr={mm(c2)} on_rest={on_rest} clear={clear} still={_EP['still']} "
              f"test={'run' if _EP['test'] else '-'} result={_EP['result'] and _EP['result']['pass']} "
              f"follow={_EP['follow'] and [round(v * 1000, 2) for v in _EP['follow']]} "
              f"syr={mm(ps - [SX, SY, AXIS_Z])} L={mm(tl - [0, 0, TOP])} R={mm(tr - [0, 0, TOP])} "
              f"ring={mm(ring - [0, 0, TOP])} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if any cartridge's stopper was ever driven more than 2 mm into its glass (latched). Otherwise, at the end:
    1.0 with a cartridge lying in the syringe's barrel, stopper end toward the piston, the harpoon set (the scorer's
    aspiration test: the thumb ring pulled back 8 mm, the stopper slid back at least 2 mm in its glass), the syringe
    lying on its rest (in both slots, level, window up) and both hands clear of it (each TCP at least 8 cm from its
    axis); 0.5 with a cartridge in the barrel stopper end toward the piston; 0.1 with a cartridge in the barrel the
    wrong way round (cap toward the piston); else 0.0."""
    s = _EP["s"]
    if not s or _EP["advanced"]:
        return 0.0
    seated = s["seated"]
    if seated is None:
        return 0.0
    if not seated[1]:
        return 0.1
    if s["aspirates"] and s["on_rest"] and s["clear"]:
        return 1.0
    return 0.5


def success(objects) -> bool:
    """Full score with the syringe, its piston and the cartridges at rest: the trial ends there."""
    names = ["syringe", "plunger"] + [f"cartridge_{i + 1}" for i in range(N_CART)]
    return score(objects) >= 1.0 and _EP["test"] is None and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the barrel's front section; the right takes the thumb ring and draws the piston fully back. Both
# let go and the left goes home. The right takes the nearest cartridge from the rack, turns it so its stopper end
# points at the piston, and drops it into the window. The left takes the front section again and the right the thumb
# ring; the right pushes firmly: the cartridge slides forward against the barrel's front and the harpoon bites into the
# stopper. A gentle pull on the ring shows the stopper follows; the ring goes back, both hands let go and go home.

FRONT_FINGER, RING_FINGER, CART_FINGER = -0.012, -0.020, -0.0105
FRONT_UP, RING_UP, CART_UP = 0.007, 0.006, 0.007  # TCP heights above the barrel's axis / ring / cartridge axis
LEAN_L, LEAN_R = -25.0, 25.0  # the grips at the syringe, leaned so the wrists stay apart (deg about world x)
DROP_UP = 0.008  # the cartridge's axis this far above its seat when let go
PUSH_OVER, ASP_BACK = 0.045, 0.006  # how far past the stopper's face the right aims to set the harpoon; to aspirate
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5), (-0.5, 0.0, 0.5),
                                  (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > math.radians(25.0):
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

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _f, r0 = cur[arm]
        r1 = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r1))
        cur[arm] = (p1, finger, r1)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        last = plan[arm][-1]
        plan[arm].append(Waypoint(t, last.pos, last.finger, label, last.yaw))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, finger, label, rot):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)

    up = np.array([0.0, 0.0, 0.05])
    L, R = "left", "right"
    rot_front = _rx(LEAN_L) @ grip_rotation(-math.pi / 2)
    rot_ring = _rx(LEAN_R) @ grip_rotation(math.pi / 2)
    front = np.array([SX, SY + FRONT_L / 2, AXIS_Z + FRONT_UP])

    def ring_at(tip):
        return np.array([SX, SY + tip - RING_C, AXIS_Z + RING_UP])

    # -- 1. draw the piston fully back --------------------------------------------------------------------------------
    from_home(L, front + up, FRONT_FINGER, "over the barrel's front section", rot_front)
    go(L, 1.0, front, FRONT_FINGER, "down round it")
    go(L, 0.6, front, FINGER_CLOSED, "hold the barrel")
    from_home(R, ring_at(TIP0) + up, RING_FINGER, "over the thumb ring", rot_ring)
    go(R, 1.0, ring_at(TIP0), RING_FINGER, "down round the ring")
    go(R, 0.6, ring_at(TIP0), FINGER_CLOSED, "take the thumb ring")
    sync()
    go(R, 2.0, ring_at(TIP_BACK - 0.008), FINGER_CLOSED, "draw the piston fully back", n=2)
    wait(R, 0.4, "drawn back")
    go(R, 0.5, cur[R][0], RING_FINGER, "let go of the ring")
    go(R, 0.6, cur[R][0] + up, RING_FINGER, "rise")
    sync()
    go(L, 0.5, front, FRONT_FINGER, "let go of the barrel")
    go(L, 0.6, front + up, FRONT_FINGER, "rise")
    to_home(L)
    # -- 2. a cartridge into the window, stopper end toward the piston -------------------------------------------------
    cy = CART_YS[0]
    pick = np.array([RACK_X, cy, CART_Z + CART_UP])
    seat_z = AXIS_Z - (BORE_R - CART_R)
    drop = np.array([SX, SY + (WIN0 + WIN1) / 2, seat_z + DROP_UP + CART_UP])
    rot_pick, rot_drop = grip_rotation(0.0), grip_rotation(math.pi / 2)
    go(R, 2.0, pick + up, CART_FINGER, "over the nearest cartridge", rot=rot_pick, n=2)
    go(R, 1.0, pick, CART_FINGER, "down round it")
    go(R, 0.6, pick, FINGER_CLOSED, "take the cartridge")
    go(R, 1.0, pick + up, FINGER_CLOSED, "lift it out of the rack")
    go(R, 3.0, drop + up, FINGER_CLOSED, "turn it, stopper end toward the piston, over the window", rot=rot_drop, n=4)
    go(R, 1.2, drop, FINGER_CLOSED, "lower it to the window")
    go(R, 0.5, drop, CART_FINGER, "let it drop in")
    go(R, 0.8, drop + up, CART_FINGER, "rise")
    go(R, 1.5, ring_at(TIP_BACK) + up, RING_FINGER, "over the thumb ring", rot=rot_ring, n=2)
    sync("the cartridge is in")
    # -- 3. set the harpoon -------------------------------------------------------------------------------------------
    plan[L] += _joint_path(L, np.array(READY, float), _good_ik(L, front + up, rot_front), "back over the barrel",
                           FRONT_FINGER)
    cur[L] = (front + up, FRONT_FINGER, rot_front)
    sync()
    go(L, 1.0, front, FRONT_FINGER, "down round the front section")
    go(L, 0.6, front, FINGER_CLOSED, "hold the barrel")
    back = ring_at(TIP_BACK)
    go(R, 1.0, back, RING_FINGER, "down round the ring")
    go(R, 0.6, back, FINGER_CLOSED, "take the thumb ring")
    sync()
    face = -CART_L + STOP_IN  # the tip's place when it meets the stopper of a cartridge against the barrel's front
    go(R, 2.5, ring_at(face + PUSH_OVER), FINGER_CLOSED, "push the thumb ring firmly: the harpoon bites", n=3)
    wait(R, 1.0, "hold the push")
    go(R, 1.0, ring_at(face + 0.004), FINGER_CLOSED, "ease off")
    go(R, 1.0, ring_at(face - ASP_BACK), FINGER_CLOSED, "pull the ring back gently: the stopper follows")
    wait(R, 0.8, "the stopper has followed")
    go(R, 1.0, ring_at(face + 0.006), FINGER_CLOSED, "let the ring forward again")
    sync()
    go(R, 0.5, cur[R][0], RING_FINGER, "let go of the ring")
    go(L, 0.5, front, FRONT_FINGER, "let go of the barrel")
    go(R, 0.6, cur[R][0] + up, RING_FINGER, "rise")
    go(L, 0.6, front + up, FRONT_FINGER, "rise")
    for arm in (L, R):  # both home together, before the scorer's check of the harpoon ends the trial
        p, f, r = cur[arm]
        path = _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), "back to where it started", f)
        plan[arm] += [Waypoint(w.duration * 2.0 / LEAD_S, w.pos, w.finger, w.label, w.yaw) for w in path]
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(4.0, last.pos, last.finger,
                                  "hold still: the scorer pulls the ring 3 mm to check the harpoon; the trial ends "
                                  "once the full score has held 1 s", last.yaw))
    return plan
