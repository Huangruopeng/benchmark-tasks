"""A smartphone lies face up in a repair cradle, its glass display glued to the phone's aluminium frame by an adhesive
strip along its right edge and the right halves of its top and bottom edges; along its left edge the display's flex
cables join it to the phone. On the cradle lie a suction cup and an opening tool (a thin steel blade on a blue plastic
handle), and left of the phone stands a foam block to lean the display against once it is open.

Task (the instruction gives the goal only): free the display from its adhesive, open it like a book over its left
(cable) edge until it leans on the foam block and no further, put both tools down on the cradle clear of the phone, and
take both arms back to where they started.

Why two hands: the adhesive is springy: pulled up by the suction cup the display's right edge lifts about 3 mm, and as
soon as the pull stops it snaps back flush. The blade cannot enter a closed seam (its end meets the display's edge face
square on), so one hand has to hold the gap open with the suction cup while the other slides the blade in.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_a_phones_glued_display_with_a_suction_cup_and_an_opening_tool.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the repair cradle: a slab whose top is flush with the phone's seam (the phone sits in its recess) -----------------
CX0, CX1, CY0, CY1, CR_T = 0.125, 0.375, -0.150, 0.175, 0.012
ZS = TOP + CR_T  # the seam: the frame's top, where the display's underside rests
# -- the phone: long axis forward (x), the display hinged on its cables along the left edge (+y) -----------------------
PX, PY = 0.250, 0.000
PL, PW, DT = 0.1476, 0.0716, 0.0030  # display: long, wide, thick (glass and OLED stack)
DISPLAY_MASS = 0.045
X0, X1, YF, YH = PX - PL / 2, PX + PL / 2, PY - PW / 2, PY + PW / 2  # bottom, top, free (right) and hinge (left) edges
OPEN_LIMIT = 115.0  # the hinge's stop (deg)
TEAR_DEG = 108.0  # opened further than this, the flex cables tear (latched)
OPEN_MIN, OPEN_MAX = 80.0, TEAR_DEG  # "open, leaning on the block"
# -- the adhesive: seven segments along the free edges; each acts as a spring (N/m) at its distance from the hinge ----
EDGE_IN = 0.016  # a blade point counts as in the seam within this far inside the display's outline (and >= 2 mm)
SEG_K = 257.4
SEAM_RISE = 0.0025  # in the seam: below the display's underside by at most this (the blade is 1.5 mm thick)
CABLE_DAMP_ON, CABLE_DAMP_OFF = 0.001, 0.0002  # N m s/deg while glued / once free
# -- the stand the open display leans on: a foam block to the left of the phone ----------------------------------------
STAND_Y0, STAND_Y1, STAND_H, STAND_L = 0.045, 0.067, 0.035, 0.100
# -- the tools ------------------------------------------------------------------------------------------------------
TOOL_X, TOOL_TIP_Y = 0.195, YF - 0.025  # the opening tool lies pointing at the free edge (+y), its tip 2.5 cm out
BLADE_L, BLADE_W, BLADE_T = 0.045, 0.014, 0.0015
HANDLE_L, HANDLE_W, HANDLE_H = 0.085, 0.016, 0.014
GRIP_BACK = 0.015  # the grip point: this far behind the blade's root, on the handle
TIP_FROM_GRIP = GRIP_BACK + BLADE_L
TOOL_MASS = 0.025
CUP_X, CUP_Y = 0.190, 0.105  # the suction cup's resting spot
CUP_R, CUP_H, STEM_R, STEM_H, CAP_R, CAP_H = 0.020, 0.005, 0.008, 0.035, 0.010, 0.003
CUP_MASS = 0.030
SEAL_SECONDS = 0.30  # held flat on the glass this long: sealed
SEAL_PULL = 11.0  # N: pulled off the glass harder than this, the cup pops off
SEAL_K_XY, SEAL_XY_CAP = 5000.0, 10.0  # its grip along the glass: N/m, at most N
SLIP_LOST = 0.006  # pushed this far along the glass, it peels off
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

GLASS, BEZEL, CAM, BACKFILM = (0.04, 0.045, 0.055), (0.10, 0.10, 0.11), (0.18, 0.18, 0.20), (0.55, 0.40, 0.25)
ALU, MAT, MAT_L = (0.78, 0.79, 0.80), (0.20, 0.30, 0.40), (0.26, 0.37, 0.48)
BATTERY, BOARD, SHIELD, ADHESIVE = (0.12, 0.12, 0.13), (0.10, 0.28, 0.18), (0.70, 0.71, 0.72), (0.06, 0.06, 0.06)
FLEX = (0.85, 0.55, 0.15)
BLUE, STEEL = (0.15, 0.35, 0.80), (0.80, 0.81, 0.83)
CUP_C, CUP_TOP, KNOB_C = (0.32, 0.36, 0.40), (0.40, 0.44, 0.48), (0.80, 0.80, 0.78)
FOAM = (0.85, 0.85, 0.82)

# the adhesive's segments: (name, edge, lo, hi, distance from the hinge) in the display's frame (x forward, y left);
# the free edge is at y = -PW/2, the cable (hinge) edge at y = +PW/2; the short edges are glued on their free half
_THIRD = (PL - 2 * EDGE_IN) / 3
_SHORT = ((-PW / 2 + 0.018, -PW / 2 + 0.036), (-PW / 2, -PW / 2 + 0.018))  # inner, outer
SEGMENTS = [("right_1", "free", -PL / 2 + EDGE_IN, -PL / 2 + EDGE_IN + _THIRD, PW),
            ("right_2", "free", -PL / 2 + EDGE_IN + _THIRD, PL / 2 - EDGE_IN - _THIRD, PW),
            ("right_3", "free", PL / 2 - EDGE_IN - _THIRD, PL / 2 - EDGE_IN, PW),
            ("bottom_in", "bottom", *_SHORT[0], PW / 2 - sum(_SHORT[0]) / 2),
            ("bottom_out", "bottom", *_SHORT[1], PW / 2 - sum(_SHORT[1]) / 2),
            ("top_in", "top", *_SHORT[0], PW / 2 - sum(_SHORT[0]) / 2),
            ("top_out", "top", *_SHORT[1], PW / 2 - sum(_SHORT[1]) / 2)]


def k_glued(freed) -> float:
    """The adhesive's stiffness against opening (N m per radian) with the segments not in ``freed`` still holding."""
    return sum(SEG_K * r * r for name, _e, _a, _b, r in SEGMENTS if name not in freed)


# -- the bodies --------------------------------------------------------------------------------------------------------

def _cradle() -> Body:
    """The repair cradle (an anti-static mat on a slab) with the phone's frame in its recess: what shows of the phone is
    its aluminium rim and, once the display is open, the battery, the logic board and the adhesive strip. Origin: the
    phone's centre, on the cradle's top."""
    cx, cy = (CX0 + CX1) / 2 - PX, (CY0 + CY1) / 2 - PY
    shapes = [BoxShape((cx, cy, -CR_T / 2), (CX1 - CX0, CY1 - CY0, CR_T), MAT)]
    for j in range(7):  # the mat's grid lines
        shapes.append(BoxShape((CX0 - PX + 0.02 + j * 0.035, cy, 0.0002), (0.0012, CY1 - CY0 - 0.01, 0.0003), MAT_L,
                               collide=False))
    rim = 0.0012
    for sx in (-1.0, 1.0):  # the aluminium rim round the display
        shapes.append(BoxShape((sx * (PL / 2 + rim / 2), 0.0, 0.0003), (rim, PW + 2 * rim, 0.0005), ALU, collide=False))
    for sy in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, sy * (PW / 2 + rim / 2), 0.0003), (PL + 2 * rim, rim, 0.0005), ALU, collide=False))
    # inside (seen once the display is open): the dark frame, battery, logic board, shields, adhesive strip, connectors
    shapes += [BoxShape((0.0, 0.0, 0.0002), (PL - 0.002, PW - 0.002, 0.0003), (0.16, 0.16, 0.17), collide=False),
               BoxShape((-0.018, 0.006, 0.0004), (0.080, 0.050, 0.0003), BATTERY, collide=False),
               BoxShape((-0.018, 0.006, 0.0006), (0.070, 0.040, 0.0002), (0.20, 0.20, 0.22), collide=False),
               BoxShape((0.048, -0.004, 0.0004), (0.040, 0.054, 0.0003), BOARD, collide=False),
               BoxShape((0.050, -0.006, 0.0006), (0.030, 0.040, 0.0003), SHIELD, collide=False),
               BoxShape((0.030, PW / 2 - 0.006, 0.0007), (0.012, 0.008, 0.0004), (0.25, 0.25, 0.27), collide=False),
               BoxShape((0.010, PW / 2 - 0.006, 0.0007), (0.012, 0.008, 0.0004), (0.25, 0.25, 0.27), collide=False)]
    a = 0.0022
    shapes += [BoxShape((0.0, -PW / 2 + a / 2 + 0.0005, 0.0005), (PL - 0.004, a, 0.0003), ADHESIVE, collide=False),
               BoxShape((-PL / 2 + a / 2 + 0.0005, -PW / 2 + 0.018, 0.0005), (a, 0.036, 0.0003), ADHESIVE, collide=False),
               BoxShape((PL / 2 - a / 2 - 0.0005, -PW / 2 + 0.018, 0.0005), (a, 0.036, 0.0003), ADHESIVE, collide=False)]
    return Body.build("cradle", pos=(PX, PY, ZS), mass=1.0, kinematic=True, color=MAT, shapes=shapes,
                      friction=(0.5, 0.4))


def _display() -> Body:
    """The display assembly: a black glass front over the OLED panel, with the camera cut-out near its top and its two
    flex cables along the left edge; the underside is the panel's copper-coloured back film. Origin: its middle."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (PL, PW, DT), GLASS),
              BoxShape((0.0, 0.0, DT / 2 + 0.0001), (PL - 0.004, PW - 0.004, 0.0002), (0.025, 0.028, 0.035),
                       collide=False),  # the active area, a shade darker than the border
              BoxShape((PL / 2 - 0.009, 0.0, DT / 2 + 0.0003), (0.0065, 0.020, 0.0002), CAM, collide=False),
              BoxShape((0.0, 0.0, -DT / 2 - 0.0001), (PL - 0.006, PW - 0.006, 0.0002), BACKFILM, collide=False)]
    for x in (0.030, 0.010):  # the flex cables, folded under the left edge
        shapes.append(BoxShape((x, PW / 2 - 0.004, -DT / 2 - 0.0003), (0.010, 0.008, 0.0004), FLEX, collide=False))
    return Body.build("display", pos=(PX, PY, ZS + DT / 2), mass=DISPLAY_MASS, color=GLASS, shapes=shapes,
                      friction=(0.30, 0.25), damping=(0.0, 0.0))


def _stand() -> Body:
    """A clean foam block on the cradle, left of the phone, to lean the open display against."""
    shapes = [BoxShape((0.0, 0.0, STAND_H / 2), (STAND_L, STAND_Y1 - STAND_Y0, STAND_H), FOAM)]
    for j in range(5):
        shapes.append(BoxShape((-0.04 + 0.02 * j, -(STAND_Y1 - STAND_Y0) / 2 - 0.0002, STAND_H / 2),
                               (0.0015, 0.0004, STAND_H - 0.004), (0.75, 0.75, 0.72), collide=False))
    return Body.build("stand", pos=(PX, (STAND_Y0 + STAND_Y1) / 2, ZS), mass=0.05, kinematic=True, color=FOAM,
                      shapes=shapes, friction=(0.9, 0.8))


def _tool() -> Body:
    """The opening tool: a blue plastic handle and a thin flat steel blade, lying flat. In its own frame the blade
    points -y; it lies turned half round, the blade pointing +y at the phone. Origin: the grip point, underneath."""
    hy = GRIP_BACK - BLADE_L / 2  # (unused name kept short)
    del hy
    shapes = [BoxShape((0.0, -GRIP_BACK + HANDLE_L / 2, HANDLE_H / 2), (HANDLE_W, HANDLE_L, HANDLE_H), BLUE),
              BoxShape((0.0, -GRIP_BACK - BLADE_L / 2, BLADE_T / 2), (BLADE_W, BLADE_L, BLADE_T), STEEL)]
    for j in range(6):  # grip ribs on the handle's top
        shapes.append(BoxShape((0.0, 0.010 + 0.008 * j, HANDLE_H + 0.0002), (HANDLE_W - 0.004, 0.002, 0.0004),
                               (0.10, 0.25, 0.62), collide=False))
    return Body.build("opener", pos=(TOOL_X, TOOL_TIP_Y - TIP_FROM_GRIP, ZS), mass=TOOL_MASS, color=BLUE, yaw=math.pi,
                      shapes=shapes, friction=(0.35, 0.30), com=(0.0, 0.020, 0.006))


def _cup() -> Body:
    """The suction cup: a soft grey cup 4 cm across with a ribbed handle to hold it by. Origin: the middle of the
    cup's rim, underneath."""
    shapes = [CylShape((0.0, 0.0, CUP_H / 2), CUP_R, CUP_H, CUP_C),
              CylShape((0.0, 0.0, CUP_H + 0.003), 0.013, 0.006, CUP_TOP),
              CylShape((0.0, 0.0, CUP_H + 0.006 + STEM_H / 2), STEM_R, STEM_H, KNOB_C),
              CylShape((0.0, 0.0, CUP_H + 0.006 + STEM_H + CAP_H / 2), STEM_R, CAP_H, (0.70, 0.70, 0.68))]
    for j in range(4):  # the handle's grip rings
        shapes.append(CylShape((0.0, 0.0, CUP_H + 0.012 + 0.007 * j), STEM_R + 0.0004, 0.0012, (0.62, 0.62, 0.60),
                               collide=False))
    return Body.build("suction_cup", pos=(CUP_X, CUP_Y, ZS), mass=CUP_MASS, color=CUP_C, shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, 0.0, 0.012))


OBJECTS = [_cradle(), _display(), _stand(), _tool(), _cup()]
K0_DEG = k_glued(()) * math.pi / 180.0
hinge("flex", "cradle", "display", anchor=(PX, YH, ZS), axis="X", limits_deg=(-OPEN_LIMIT, 0.0), spring=K0_DEG,
      damping=CABLE_DAMP_ON, target_deg=0.0)

# -- the suction cup's seal: a D6 joint from the display's top face to the cup's rim, authored switched off ------------
# While sealed the cup stays on the glass and square to it (its lift and tilt locked; it may spin on its axis) and holds
# where it was pressed on by drive springs along the glass, capped at 10 N (pushed further it peels off). Pulled away
# from the glass it carries the display with it; it pops off once the pull passes 11 N, which the task file reads from
# how far the pull has opened the display against the adhesive (the adhesive's torque over the cup's lever).
_SEAL: dict = {}
_author_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    if spec.name != "suction_cup":
        return
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    path = f"{prim_path}/seal"
    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([f"{parent}/display/body"])
    joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, DT / 2))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateJointEnabledAttr(False)
    joint.CreateCollisionEnabledAttr(True)
    joint.CreateExcludeFromArticulationAttr(True)
    prim = joint.GetPrim()
    for dof in ("transZ", "rotX", "rotY"):
        limit = UsdPhysics.LimitAPI.Apply(prim, dof)  # low > high: locked
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)
    for dof, k, cap in (("transX", SEAL_K_XY, SEAL_XY_CAP), ("transY", SEAL_K_XY, SEAL_XY_CAP)):
        drive = UsdPhysics.DriveAPI.Apply(prim, dof)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(k))
        drive.CreateDampingAttr(float(k) * 0.01)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(float(cap))
    _SEAL["path"] = path
    _SEAL.pop("attrs", None)


bodykit._author_sliders = _author_sliders


def _seal_attrs():
    if "attrs" not in _SEAL:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(_SEAL["path"])
        _SEAL["attrs"] = (UsdPhysics.Joint(prim).GetJointEnabledAttr(),
                          UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr(),
                          UsdPhysics.DriveAPI(prim, "transY").GetTargetPositionAttr())
    return _SEAL["attrs"]


def _set_seal(on: bool, x: float = 0.0, y: float = 0.0) -> None:
    if "path" not in _SEAL:
        return
    enabled, tx, ty = _seal_attrs()
    if on:
        tx.Set(float(x))
        ty.Set(float(y))
    enabled.Set(bool(on))


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"freed": set(), "torn": "", "in_seam": False, "sealed": False, "flat": 0, "seal_xy": (0.0, 0.0),
            "seals": 0, "pops": 0, "k": None, "pull": 0.0, "tcps": None, "s": {}, "steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PHONE_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _set_seal(False)


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


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _angle(rd) -> float:
    """How far the display is opened about its cable edge (deg): its free (right) edge lifting turns it about -x."""
    return -math.degrees(math.atan2(rd[2, 1], rd[1, 1]))


BLADE_POINTS = [np.array([x, -TIP_FROM_GRIP + d, BLADE_T / 2]) for d in (0.001, 0.010, 0.020)
                for x in (-BLADE_W / 2 + 0.001, 0.0, BLADE_W / 2 - 0.001)]


def _seam_hits(q) -> list[str]:
    """The adhesive segments a blade point at ``q`` (the display's frame) is cutting, if it is in the seam."""
    if not (-DT / 2 - SEAM_RISE <= q[2] <= -DT / 2 + 0.0005):
        return []
    if abs(q[0]) > PL / 2 - 0.002 or abs(q[1]) > PW / 2 - 0.002:
        return []
    out = []
    for name, edge, lo, hi, _r in SEGMENTS:
        if edge == "free" and q[1] < -PW / 2 + EDGE_IN and lo <= q[0] <= hi:
            out.append(name)
        elif edge == "bottom" and q[0] < -PL / 2 + EDGE_IN and lo <= q[1] <= hi:
            out.append(name)
        elif edge == "top" and q[0] > PL / 2 - EDGE_IN and lo <= q[1] <= hi:
            out.append(name)
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    pd, rd = _pose(base, origin, "display")
    phi = _angle(rd)
    hinge_pt = np.array([PX, YH, ZS])
    edge_now = pd + rd @ np.array([0.0, PW / 2, -DT / 2])  # the display's cable edge, on its underside
    off = math.hypot(edge_now[1] - hinge_pt[1], edge_now[2] - hinge_pt[2])
    if not _EP["torn"] and (phi > TEAR_DEG or off > 0.005):
        _EP["torn"] = f"cables torn: opened {phi:.0f} deg, cable edge {off * 1000:.0f} mm off its line"
    # the opening tool's blade in the seam frees the adhesive it runs along
    pt, rt = _pose(base, origin, "opener")
    for p in BLADE_POINTS:
        q = rd.T @ (pt + rt @ p - pd)
        hits = _seam_hits(q)
        if hits:
            _EP["in_seam"] = True
            _EP["freed"].update(hits)
    k = k_glued(_EP["freed"])
    if k != _EP["k"]:
        set_hinge_drive("flex", spring=k * math.pi / 180.0, damping=CABLE_DAMP_ON if k > 0 else CABLE_DAMP_OFF)
        _EP["k"] = k
    # the suction cup: seals when held flat on the glass, pops off when pulled or pushed past the seal's strength
    pc, rc = _pose(base, origin, "suction_cup")
    rel = rd.T @ (pc - pd) - np.array([0.0, 0.0, DT / 2])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((rd.T @ rc)[2, 2])))))
    if _EP["sealed"]:
        sx, sy = _EP["seal_xy"]
        slip = math.hypot(rel[0] - sx, rel[1] - sy)
        lever = max(PW / 2 - rel[1], 0.005)  # the cup's distance from the cable edge
        pull = k * math.radians(max(phi, 0.0)) / lever  # the pull the adhesive is holding against (N)
        _EP["pull"] = pull
        if pull > SEAL_PULL or slip > SLIP_LOST or tilt > 15.0:
            _set_seal(False)
            _EP["sealed"] = False
            _EP["pops"] += 1
            _EP["flat"] = 0
    else:
        on_glass = abs(rel[0]) <= PL / 2 - CUP_R + 0.002 and abs(rel[1]) <= PW / 2 - CUP_R + 0.002
        top = pc + rc @ np.array([0.0, 0.0, CUP_H + 0.006 + STEM_H / 2])
        held = _EP["tcps"] is not None and min(float(np.linalg.norm(t - top)) for t in _EP["tcps"]) < 0.03
        if on_glass and held and -0.0008 <= rel[2] <= 0.0010 and tilt < 8.0:
            _EP["flat"] += 1
        else:
            _EP["flat"] = 0
        if _EP["flat"] >= int(round(SEAL_SECONDS * 20)):
            _EP["seal_xy"] = (float(rel[0]), float(rel[1]))
            _set_seal(True, rel[0], rel[1])
            _EP["sealed"] = True
            _EP["seals"] += 1
    # the end state
    _EP["tcps"] = _tcps(base, origin)

    def down(p, r, z_max):
        out_x = p[0] < X0 - 0.010 or p[0] > X1 + 0.010 or p[1] < YF - 0.010 or p[1] > YH + 0.010
        on_cradle = CX0 <= p[0] <= CX1 and CY0 <= p[1] <= CY1 and ZS - 0.002 <= p[2] <= ZS + z_max
        return out_x and on_cradle and _tilt(r) < 20.0

    tool_down = down(pt, rt, 0.004)
    cup_down = down(pc, rc, 0.004) and not _EP["sealed"]
    _EP["s"] = {"phi": phi, "open": OPEN_MIN <= phi <= OPEN_MAX, "tools": tool_down and cup_down,
                "home": _arms_home(), "all_free": len(_EP["freed"]) == len(SEGMENTS)}
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        tl, tr = _EP["tcps"]
        mm = lambda v: np.round((np.asarray(v) - [0, 0, ZS]) * 1000).astype(int).tolist()  # noqa: E731
        tip = pt + rt @ BLADE_POINTS[0]
        print(f"[phone] t={_EP['steps'] / 20:5.1f}s phi={phi:6.2f} free={sorted(_EP['freed'])} seam={_EP['in_seam']}"
              f" sealed={_EP['sealed']} pull={_EP['pull']:.1f} seals={_EP['seals']} pops={_EP['pops']} cup_rel={np.round(rel * 1000, 1).tolist()}"
              f" tilt={tilt:.1f} tip={mm(tip)} tool={mm(pt)} cup={mm(pc)} torn='{_EP['torn']}'"
              f" tools={tool_down},{cup_down} home={_EP['s']['home']} L={mm(tl)} R={mm(tr)} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if the display was ever opened past 108 degrees or pulled off its cable edge (the flex cables torn;
    latched). Otherwise 1.0: at the end every adhesive segment freed, the display open between 80 and 108 degrees,
    both tools down on the cradle clear of the phone (the suction cup no longer stuck to the display) and both arms
    back where they started; 0.75: every segment freed and the display open between 80 and 108 degrees at the end;
    0.5: every segment freed; 0.25: the opening tool's blade in the seam under the display at some point; else 0.0."""
    s = _EP["s"]
    if not s or _EP["torn"]:
        return 0.0
    if s["all_free"]:
        if s["open"]:
            return 1.0 if (s["tools"] and s["home"]) else 0.75
        return 0.5
    return 0.25 if _EP["in_seam"] else 0.0


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
    """Full score with the display and both tools at rest: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("display", "opener", "suction_cup"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the suction cup by its stem, sets it flat on the display near the top of its right (free) edge,
# holds it there to seal, then pulls up: the right edge lifts about 3 mm. The right hand slides the opening tool along
# the cradle, blade first, into the gap low on the right edge. The left pulls harder until the cup pops off, puts it
# back and goes home. The right runs the blade down to the bottom end of the right edge, up the whole edge, pushes it in
# under the top edge and back, runs it down again and pushes it in under the bottom edge and back, then lifts the freed
# display on the blade and swings it over its cable edge until it leans on the foam block. It lays the tool down and
# goes home.

CUP_FINGER, TOOL_FINGER = -0.016, -0.016
CUP_TCP_UP = CUP_H + 0.006 + 0.020  # the TCP's height above the cup's rim when holding the handle
TOOL_TCP_UP = 0.015  # the TCP's height above the tool's underside when holding the handle
SEAL_AT = (X1 - 0.022, YF + CUP_R + 0.004)  # where the cup goes on the display
PULL_Z, POP_Z = 0.040, 0.085  # how far above the glass the left aims while pulling the gap open, and to pop the cup
IN_DEPTH = 0.014  # the blade's tip this far in from the free edge while it runs along it
DEEP_IN = 0.043  # ... and this far in when pushed in under a short edge
END_IN = 0.008  # the tip's middle this far in from the short edge at each end of its run
LIFT = 0.0005  # the tool's underside this far above the cradle while it slides
CUP_YAW, TOOL_YAW = 0.0, math.pi / 2  # the hands' grips: across the stem; across the tool's handle
LEAD_S, LEAD_STEPS = 3.0, 15


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


def tool_grip(tip_xy, lift: float = 0.0) -> np.ndarray:
    """The TCP holding the tool, lying pointing +y, whose blade tip is at ``tip_xy``, its underside ``lift`` up."""
    return np.array([tip_xy[0], tip_xy[1] - TIP_FROM_GRIP, ZS + TOOL_TCP_UP + lift])


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, n=1):
        p0, _f, yaw = cur[arm]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label,
                                      grip_rotation(yaw)))
        cur[arm] = (p1, finger, yaw)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        last = plan[arm][-1]  # stay where the last waypoint put the hand
        plan[arm].append(Waypoint(t, last.pos, last.finger, label, last.yaw))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, finger, label, yaw):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, grip_rotation(yaw)), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, yaw)

    def to_home(arm, label="back to where it started"):
        p, f, yaw = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, grip_rotation(yaw)), np.array(READY, float), label, f)

    up = np.array([0.0, 0.0, 0.05])
    C, T = "left", "right"
    # -- the left: the suction cup on, sealed, pulled up -------------------------------------------------------------
    cup0 = np.array([CUP_X, CUP_Y, ZS + CUP_TCP_UP])
    seal = np.array([SEAL_AT[0], SEAL_AT[1], ZS + DT + CUP_TCP_UP])
    from_home(C, cup0 + up, CUP_FINGER, "over the suction cup", CUP_YAW)
    go(C, 1.0, cup0, CUP_FINGER, "down round its stem")
    go(C, 0.6, cup0, FINGER_CLOSED, "take the suction cup by its stem")
    go(C, 1.0, cup0 + [0, 0, 0.06], FINGER_CLOSED, "lift it")
    go(C, 2.0, seal + [0, 0, 0.06], FINGER_CLOSED, "carry it over the display, near its right edge", n=2)
    go(C, 1.2, seal + [0, 0, 0.003], FINGER_CLOSED, "set it flat on the glass")
    go(C, 0.6, seal - [0, 0, 0.002], FINGER_CLOSED, "press it on")
    wait(C, 0.8, "hold it pressed: it seals")
    go(C, 1.5, seal + [0, 0, PULL_Z], FINGER_CLOSED, "pull straight up: the right edge lifts")
    # -- the right: the opening tool, into the gap ---------------------------------------------------------------------
    g0 = tool_grip((TOOL_X, TOOL_TIP_Y))
    from_home(T, g0 + up, TOOL_FINGER, "over the opening tool's handle", TOOL_YAW)
    go(T, 1.0, g0, TOOL_FINGER, "down round the handle")
    go(T, 0.6, g0, FINGER_CLOSED, "take the opening tool")
    go(T, 1.2, tool_grip((TOOL_X, YF - 0.004), LIFT), FINGER_CLOSED, "slide it flat along the cradle to the seam")
    sync("wait: the gap is open")
    go(T, 1.5, tool_grip((TOOL_X, YF + IN_DEPTH), LIFT), FINGER_CLOSED, "slide the blade into the gap")
    sync("the blade is in")
    # -- the left pops the cup off and puts it back ------------------------------------------------------------------
    go(C, 2.0, seal + [0, 0, POP_Z], FINGER_CLOSED, "pull harder: the cup pops off the glass", n=2)
    go(C, 1.0, seal + [0, 0, 0.06], FINGER_CLOSED, "steady")
    go(C, 3.0, cup0 + [0, 0, 0.06], FINGER_CLOSED, "carry it back", n=3)
    go(C, 1.0, cup0 + [0, 0, 0.002], FINGER_CLOSED, "set it down")
    go(C, 0.5, cup0 + [0, 0, 0.002], CUP_FINGER, "let go")
    go(C, 0.6, cup0 + up, CUP_FINGER, "rise")
    to_home(C)
    # -- the right frees the adhesive: along the whole free edge, then in under the top and the bottom edges -----------
    yb, deep = YF + IN_DEPTH, YF + DEEP_IN
    xb, xt = X0 + END_IN, X1 - END_IN
    go(T, 1.5, tool_grip((xb, yb), LIFT), FINGER_CLOSED, "run the blade down the right edge")
    go(T, 3.0, tool_grip((xt, yb), LIFT), FINGER_CLOSED, "run it up the whole right edge", n=3)
    go(T, 2.0, tool_grip((xt, deep), LIFT), FINGER_CLOSED, "push it in under the top edge", n=2)
    go(T, 1.2, tool_grip((xt, yb), LIFT), FINGER_CLOSED, "draw it back to the right edge")
    go(T, 3.0, tool_grip((xb, yb), LIFT), FINGER_CLOSED, "run it back down the right edge", n=3)
    go(T, 2.0, tool_grip((xb, deep), LIFT), FINGER_CLOSED, "push it in under the bottom edge", n=2)
    go(T, 1.2, tool_grip((xb, yb), LIFT), FINGER_CLOSED, "draw it back to the right edge")
    # -- the right swings the freed display open on the blade, over its cable edge, onto the foam block ---------------
    go(T, 1.5, tool_grip((PX, yb), LIFT), FINGER_CLOSED, "slide the blade to the middle of the right edge")
    rho = PW - IN_DEPTH  # the tip's distance from the cable edge
    for j in range(1, 13):
        phi = math.radians(4.0 + 86.0 * j / 12)
        go(T, 0.45, tool_grip((PX, YH - rho * math.cos(phi)), rho * math.sin(phi) - 0.002), FINGER_CLOSED,
           "lift the display on the blade, swinging it open")
    go(T, 1.2, tool_grip((PX, YH + 0.004), rho - 0.004), FINGER_CLOSED, "push it just past upright: it leans on the block")
    go(T, 1.0, cur[T][0] + [0, -0.04, 0.0], FINGER_CLOSED, "draw the tool back")
    go(T, 1.5, g0 + [0, 0, 0.04], FINGER_CLOSED, "carry it back to its place", n=2)
    go(T, 1.0, g0 + [0, 0, 0.002], FINGER_CLOSED, "lay it down")
    go(T, 0.5, g0 + [0, 0, 0.002], TOOL_FINGER, "let go")
    go(T, 0.6, g0 + up, TOOL_FINGER, "rise")
    to_home(T)
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
