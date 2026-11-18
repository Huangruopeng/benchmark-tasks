"""On the station table a 10 cm offcut of single-core 35 mm2 cable (H07V-K, 10.3 mm across), a square grip sleeve
clamped on its top, stands on its free end in the 12 mm hole of a beech gauge block, a steel V backstop behind it; beside
it lies a round-cable stripping knife, a JOKARI No. 27 SECURA (part 10270, for round cables 8 to 28 mm across), its blade
set 0.9 mm deep, under the insulation's 0.98 mm minimum thickness.

Task: strip 27 mm of insulation off the cable's free end with the knife, as for a 35 mm2 ferrule (25 mm sleeve, 29 mm
longest strip), without nicking the copper. The steps are the knife maker's: press the knife onto the cable, cut round
(the cable turned a full turn against the pressed knife, its bracket round the cable and the knife lying on the gauge
block, which puts the blade 27 mm above the cable's end), cut along (the cable drawn up through the pressed knife, so
the blade runs along the cut piece out past the cable's end), then peel the cut piece off by hand. With no hand
pressing the knife, the blade does not cut; the cut piece comes off only once it is cut round and along.

Run it:  inspect-robots run --embodiment yam_arena -E task=strip_a_wire_end_in_a_stripping_station.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step,  # noqa: E402
                     plate, rotation, set_glue_on, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PHYSICS_DT  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cable: H07V-K 1 x 35 mm2 (class 5 flexible copper 7.9 mm across, 1.2 mm of PVC insulation, 10.3 mm overall,
# 340 g/m), stiff here, a 10 cm offcut cut square at both ends, with a reusable grip clamped on its top 3 cm: a 16 mm
# square aluminium sleeve, 30 mm long, bored 12 mm, held by an M4 set screw. It stands on its free end in the gauge
# block's hole. The insulation over its last 34 mm, where the cut can be made, is shown as 2 mm rings and collides as part
# of the cable; a hidden sleeve over it, held to the cable, is what a hand takes hold of to peel the cut piece off ------
WIRE_L, WIRE_R, CORE_R = 0.10, 0.00515, 0.00395
INS_T = WIRE_R - CORE_R  # 1.2 mm of insulation (nominal)
INS_MIN = INS_T - (0.0001 + 0.1 * INS_T)  # 0.98 mm: IEC 60227-1 5.2.3 lets it be 0.1 mm + 10 % thinner at any place
RING_H, N_RINGS = 0.002, 17
END_L = RING_H * N_RINGS  # the insulated end, 34 mm
CABLE_KG_M = 0.340
GRIP, GRIP_H, GRIP_BORE = 0.016, 0.030, 0.012  # the grip sleeve: 16 mm square, 30 mm long, a 12 mm bore
STEEL_RHO, PVC_RHO, AL_RHO, G = 7850.0, 1400.0, 2700.0, 9.81
INS_KG_M = math.pi * (WIRE_R ** 2 - CORE_R ** 2) * PVC_RHO  # 48 g/m of the 340 is the PVC
CU_KG_M = CABLE_KG_M - INS_KG_M
CABLE_MASS = CABLE_KG_M * WIRE_L  # 34.0 g
RING_MASS = INS_KG_M * RING_H  # 0.096 g
GRIP_MASS = (GRIP * GRIP - math.pi * GRIP_BORE ** 2 / 4) * GRIP_H * AL_RHO  # 11.6 g
SLEEVE_MASS = 0.001  # the hidden sleeve: 1.0 g of the cable's copper, given to it
SLEEVE_R = WIRE_R + 0.0003  # just over the cable, so a hand that grips the end grips the sleeve
WIRE_MASS = CABLE_MASS + GRIP_MASS - N_RINGS * RING_MASS - SLEEVE_MASS  # the rest: 45.6 g in all
WIRE_COM = (CU_KG_M * WIRE_L * WIRE_L / 2 + INS_KG_M * (WIRE_L - END_L) * (WIRE_L + END_L) / 2
            + GRIP_MASS * (WIRE_L - GRIP_H / 2) - SLEEVE_MASS * END_L / 2) / WIRE_MASS  # up from the cable's end
WIRE_YAW = math.radians(45.0)  # the grip's flats at 45 degrees to the table's edges

# -- the gauge block (the strip-length stop): a beech block on which the knife lies, with a 12 mm hole (0.85 mm all round
# the 10.3 mm cable), 12.5 mm deep, for the cable's end. The hole's floor is 27 mm below the blade's plane
# when the knife lies on the block, so the cut falls 27 mm from the cable's end. It runs along the knife's axis ---------
HOLDER_AT = (0.30, 0.07)
HOLDER_R, HOLDER_BASE, HOLDER_WALL = 0.006, 0.004, 0.002  # the hole: 12 mm across, its floor 4 mm above the table
GAUGE_BACK, GAUGE_FRONT, GAUGE_HALF_W = 0.125, 0.025, 0.025  # from the hole's axis, along and across the knife's axis
GAUGE_TOP = HOLDER_BASE + 0.027 - 0.029 / 2  # 16.5 mm: the hole's floor + the 27 mm strip - half the knife's width
# two beech guide strips glued on the block's top either side of the knife's 22 mm body, 25 mm apart: they keep the knife
# square to the cable (its axis through the hole's) while it is pressed on and slid along, and while the cut along
# drags it up a few millimetres. 6 mm high, they pass under the bracket's side straps (their band's lower edge is
# 6.5 mm above the block)
GUIDE_GAP, GUIDE_W, GUIDE_H = 0.025, 0.005, 0.006
GUIDE_X = (-0.036, -0.019)  # from the hole's axis along the knife's axis: beside the knife's body, clear of its grip
HOLDER_MU = (0.4, 0.35)  # static, dynamic: beech
# the backstop: a 90-degree V standing on the block behind the hole (on the side away from the knife), its two faces
# 6 mm from the hole's axis (the hole's own clearance) and running on up past the blade's plane, so the cable is backed
# up, and kept centred, where the knife presses it. It stands inside the knife's loop (between the cable and the
# bracket's front)
BACKSTOP_T, BACKSTOP_W, BACKSTOP_H = 0.003, 0.010, 0.022  # each face 3 mm thick, 10 mm wide; 22 mm above the block
BEECH = (0.78, 0.62, 0.42)
WIRE_Z0 = TOP + HOLDER_BASE  # the cable's end, on the hole's floor

# -- the knife: JOKARI No. 27 SECURA (part 10270): 132 mm long overall, 29 mm wide (B), 35 mm high over its bracket (H),
# 72 g, for round cables 8 to 28 mm across (maker's data sheet). A polyamide body with the blade in its front face, and
# a hardened band-steel bracket in front of the face that closes a loop round the cable with it. It is used lying on its
# side here, so the cable passes up through the loop. Knife frame: origin at the middle of the front face, in the plane
# of the blade; x forward (into the loop), z across the body's 29 mm width (up when it lies on its side) --------------
KNIFE_L, KNIFE_B, KNIFE_H = 0.132, 0.029, 0.035  # the data sheet's L, B, H
BRACKET_D, BRACKET_V = 0.032, 0.006  # the bracket's front, 32 mm in front of the face, bent to a 6 mm V
BODY_L, BODY_H = KNIFE_L - BRACKET_D, 0.022  # the body: 100 mm long, 22 mm high (H 35 is over the bracket)
BRACKET_HALF, BRACKET_BAND, BRACKET_T = KNIFE_H / 2 - 0.002, 0.016, 0.002  # its sides 15.5 mm from the axis, 16 mm band
# the blade's depth, set with the red screw: 0.9 mm out of the face, under the insulation's 0.98 mm minimum thickness.
# STRIP_BLADE_MM re-runs the task with another setting (the nicking check: 1.2 sets it to the nominal thickness)
BLADE_DEPTH = float(os.environ.get("STRIP_BLADE_MM", "0.9")) * 1e-3
BLADE_W = 0.004  # the blade's edge, 4 mm across the face's middle (design value)
ON_BLADE = BLADE_W / 2 + 0.001  # the cable's axis within this of the blade's middle: the blade is in the insulation
KNIFE_MASS = 0.072
KNIFE_COM = -0.046  # behind the face: the body and the steel bracket in front
KNIFE_YAW = math.radians(60.0)  # its x axis in the world, working: from the right hand toward the cable
KNIFE_AT = (0.39, -0.02)  # where its front face lies on the bench at the start
YELLOW, BLUE, RED, STEEL, BLACK, COPPER = ((0.98, 0.80, 0.08), (0.10, 0.25, 0.65), (0.80, 0.10, 0.10),
                                           (0.66, 0.67, 0.70), (0.08, 0.08, 0.09), (0.80, 0.50, 0.25))

# -- the cuts and the peel: a blade of included edge angle BLADE_EDGE_DEG and thickness BLADE_T advancing through PVC
# whose flow stress is its tensile strength (PVC_SIGMA, the minimum for PVC/C insulation, IEC 60227-1) needs, per mm of
# edge, F' = 2 sigma h (tan(edge/2) + mu), h = t / (2 tan(edge/2)) the depth over which its flanks spread the PVC to the
# blade's thickness: F' = sigma t (1 + mu / tan(edge/2)). Over the blade's depth in the insulation that is the cutting
# force F_CUT (against the turn, as the torque T_CUT at the middle of the cut, and against the knife drawn along the
# cable); the blade only stays at its depth while the knife is pressed on with at least F_SEAT, the PVC's push on the
# blade's leading edge. The 0.3 mm of PVC left under the cuts tears as the split piece is peeled off (F_PEEL) -------
PVC_SIGMA, BLADE_EDGE_DEG, BLADE_MU, BLADE_T = 12.5, 20.0, 0.2, 0.4  # N/mm2, degrees, -, mm (blade: design values)
F_CUT = PVC_SIGMA * BLADE_T * (1 + BLADE_MU / math.tan(math.radians(BLADE_EDGE_DEG / 2))) * BLADE_DEPTH * 1e3  # 9.6 N
R_MID = WIRE_R - BLADE_DEPTH / 2  # 4.7 mm
T_CUT = F_CUT * R_MID  # 0.045 N m
F_SEAT = PVC_SIGMA * BLADE_T * BLADE_DEPTH * 1e3  # 4.5 N: the PVC's push on the blade's leading edge (0.4 x 0.9 mm)
F_PEEL = 2.0  # N: the pull that slides the cut-round, cut-along piece off the strands (design value)
RING_CUT_DEG = 360.0  # one full turn of the cable against the pressed knife cuts the insulation all round
SAME_CUT = 0.0015  # m: the knife kept within this of where it started cutting (along the cable) is the same cut
SLIT_BIN = 0.0005  # m: the cut along the cable is followed in 0.5 mm steps
SLIT_END = 0.001  # the cut along reaches the cable's end once it runs to within 1 mm of it

# -- scoring --------------------------------------------------------------------------------------------------------------
STRIP_TARGET = 0.027
STRIP = (0.025, 0.029)  # a 35 mm2 ferrule: 25 mm sleeve, 29 mm longest strip
CLEAR = 0.002  # peeled off: the cut piece's top this far past the conductor's end
PARTIAL_DEG = 90.0
SUCCESS_HOLD_S = 1.0


def _holder() -> Body:
    """The gauge block, in a frame along the knife's axis (x) with its origin on the table under the hole's axis."""
    h, f, b, w, c = GAUGE_TOP, GAUGE_FRONT, GAUGE_BACK, GAUGE_HALF_W, 0.013  # c: half the square round the hole
    shapes = [BoxShape((0.0, 0.0, HOLDER_BASE / 2), (2 * c, 2 * c, HOLDER_BASE), BEECH),  # the hole's floor
              BoxShape((-(b + c) / 2, 0.0, h / 2), (b - c, 2 * w, h), BEECH),  # behind the hole: the knife lies here
              BoxShape(((f + c) / 2, 0.0, h / 2), (f - c, 2 * w, h), BEECH),  # in front of it
              BoxShape((0.0, (w + c) / 2, h / 2), (2 * c, w - c, h), BEECH),
              BoxShape((0.0, -(w + c) / 2, h / 2), (2 * c, w - c, h), BEECH)]
    n = 16
    for k in range(n):  # the hole's wall, and the wood between it and the square round it
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + HOLDER_WALL / 2
        wd = 2 * (HOLDER_R + HOLDER_WALL) * math.tan(math.pi / n) + 0.001
        shapes.append(plate((r * u[0], r * u[1], HOLDER_BASE), (r * u[0], r * u[1], h), wd, HOLDER_WALL, u, BEECH))
        rf = HOLDER_R + HOLDER_WALL + 0.0025
        shapes.append(plate((rf * u[0], rf * u[1], HOLDER_BASE), (rf * u[0], rf * u[1], h), wd + 0.003, 0.005, u, BEECH))
    for sy in (1.0, -1.0):  # the backstop's two faces, at 45 degrees either side of the knife's axis
        n = np.array([math.sqrt(0.5), sy * math.sqrt(0.5), 0.0])
        mid = (HOLDER_R + BACKSTOP_T / 2) * n
        shapes.append(plate((mid[0], mid[1], h), (mid[0], mid[1], h + BACKSTOP_H), BACKSTOP_W, BACKSTOP_T, n, STEEL))
    for sy in (1.0, -1.0):  # the guide strips
        shapes.append(BoxShape(((GUIDE_X[0] + GUIDE_X[1]) / 2, sy * (GUIDE_GAP + GUIDE_W) / 2, h + GUIDE_H / 2),
                               (GUIDE_X[1] - GUIDE_X[0], GUIDE_W, GUIDE_H), BEECH))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.40, shapes=shapes, kinematic=True,
                      friction=HOLDER_MU, color=BEECH, yaw=KNIFE_YAW)


def _knife() -> Body:
    """The knife, lying on its side (its 29 mm width upright); origin at the middle of its front face."""
    hb, hh = KNIFE_B / 2, BODY_H / 2
    shapes = [BoxShape((-BODY_L / 2, 0.0, 0.0), (BODY_L, BODY_H, KNIFE_B), YELLOW),  # 0: the body; its front face cuts
              BoxShape((-BODY_L - 0.003, 0.0, 0.0), (0.006, 0.016, 0.016), RED, collide=False),  # the depth screw
              BoxShape((-0.008, 0.0, 0.0), (0.016, BODY_H + 0.002, KNIFE_B + 0.002), RED, collide=False),  # front cap
              BoxShape((-0.050, 0.0, hb + 0.0003), (0.050, 0.010, 0.0006), BLUE, collide=False),  # the label panel
              BoxShape((BLADE_DEPTH / 2, 0.0, 0.0), (BLADE_DEPTH, 0.004, 0.0004), STEEL, collide=False)]  # the blade
    for s in (1.0, -1.0):  # the bracket: a strap on each side of the body, its sides, and its front bent to a V
        a = np.array([-0.012, s * (hh + BRACKET_T / 2), 0.0])
        b = np.array([0.0, s * BRACKET_HALF, 0.0])
        c = np.array([BRACKET_D - BRACKET_V, s * BRACKET_HALF, 0.0])
        d = np.array([BRACKET_D, 0.0, 0.0])
        shapes.append(plate(a, b, BRACKET_BAND, BRACKET_T, (0.0, s, 0.0), STEEL))
        shapes.append(plate(b, c, BRACKET_BAND, BRACKET_T, (0.0, s, 0.0), STEEL))
        shapes.append(plate(c, d, BRACKET_BAND, BRACKET_T, (BRACKET_HALF, s * BRACKET_V, 0.0), STEEL))
    x, y = KNIFE_AT
    return Body.build("knife", pos=(x, y, TOP + hb), mass=KNIFE_MASS, shapes=shapes, yaw=KNIFE_YAW,
                      friction=(0.2, 0.15), color=YELLOW, com=(KNIFE_COM, 0.0, 0.0))  # polyamide


END_SHAPE = 3  # the wire's shape that is its end's insulation, for colliding only (hidden: the rings show it)


def _wire() -> Body:
    """Origin at the cable's end (its bottom, standing upright); the copper runs up through the rings. The insulation
    over the end collides as one hidden cylinder of the cable's full width (the rings only show it)."""
    shapes = [CylShape((0.0, 0.0, END_L / 2), CORE_R, END_L, COPPER, collide=False),
              CylShape((0.0, 0.0, END_L + (WIRE_L - GRIP_H - END_L) / 2), WIRE_R, WIRE_L - GRIP_H - END_L, BLACK),
              CylShape((0.0, 0.0, WIRE_L), CORE_R, 0.0005, COPPER, collide=False),  # the cut copper on top
              CylShape((0.0, 0.0, END_L / 2), WIRE_R, END_L, BLACK),  # END_SHAPE
              BoxShape((0.0, 0.0, WIRE_L - GRIP_H / 2), (GRIP, GRIP, GRIP_H), STEEL),  # the grip sleeve
              BoxShape((GRIP / 2 + 0.0002, 0.0, WIRE_L - GRIP_H / 2), (0.0006, 0.003, 0.003), BLACK, collide=False)]  # its set screw
    return Body.build("wire", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0), mass=WIRE_MASS, shapes=shapes, yaw=WIRE_YAW,
                      friction=(0.8, 0.7), color=BLACK, com=(0.0, 0.0, WIRE_COM))  # PVC and aluminium


def _sleeve() -> Body:
    """The cut piece, as a hand grips it to peel it off: a hidden cylinder 0.3 mm over the cable's width along its last
    34 mm, held to the cable. It never touches the gauge block, the knife or the cable (they meet the cable's own end),
    only a hand. Its 2 mm rings (shown once the piece is loose, in place of the cable's own rings below the cut) collide
    with nothing. Origin at the cable's end."""
    shapes = [CylShape((0.0, 0.0, END_L / 2), SLEEVE_R, END_L, BLACK)]
    shapes += [CylShape((0.0, 0.0, (k + 0.5) * RING_H), WIRE_R + 0.0001, RING_H * 0.98, BLACK, collide=False)
               for k in range(N_RINGS)]
    return Body.build("sleeve", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0), mass=SLEEVE_MASS, shapes=shapes,
                      friction=(0.8, 0.7), color=BLACK, yaw=WIRE_YAW, ignore=("holder", "knife", "wire"))


def _rings() -> list[Body]:
    return [Body.build(f"ring_{k:02d}", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + (k + 0.5) * RING_H), mass=RING_MASS,
                       shapes=[CylShape((0.0, 0.0, 0.0), WIRE_R, RING_H * 0.98, BLACK, collide=False)], color=BLACK,
                       yaw=WIRE_YAW)
            for k in range(N_RINGS)]


HOLDER, KNIFE, WIRE, SLEEVE = _holder(), _knife(), _wire(), _sleeve()
RINGS = _rings()
RING_NAMES = [r.name for r in RINGS]
OBJECTS = [HOLDER, KNIFE, WIRE, SLEEVE, *RINGS]
glue("sleeve_on_wire", "wire", "sleeve", anchor=WIRE.pos)  # the insulation, uncut: held fast
for _r in RINGS:
    glue(f"{_r.name}_on_wire", "wire", _r.name, anchor=_r.pos)
# the cut piece on the strands, once cut round and along: it slides off the cable's end along the cable's axis against
# at most F_PEEL (the drive's force limit); the drive's target follows it down, so it never pulls the piece back on
PEEL_STIFF = 2.0e4  # N/m: the drive's stiffness (it reaches F_PEEL 0.1 mm from its target)
slider("sleeve_peel", "wire", "sleeve", anchor=WIRE.pos, axis="Z", limits=(-0.060, 0.0), stiffness=PEEL_STIFF,
       damping=5.0, target=0.0, max_force=F_PEEL)

_PATHS: dict[str, str] = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: contact reports on the cable and the knife; what is shown of the cable and its sleeve."""
    _author_sliders_plain(stage, prim_path, spec)
    from pxr import PhysxSchema, UsdGeom

    if spec.name in ("wire", "knife"):
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateThresholdAttr(0.0)
        _PATHS[spec.name] = f"{prim_path}/body"
    if spec.name == "sleeve":
        _PATHS["sleeve"] = f"{prim_path}/body"
        for k in range(N_RINGS + 1):
            UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/body/s{k}")).MakeInvisible()
    if spec.name in RING_NAMES:
        _PATHS[spec.name] = f"{prim_path}/body"
    if spec.name == "wire":
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/body/s{END_SHAPE}")).MakeInvisible()


bodykit._author_sliders = _author_sliders


def _show(path: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    img = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(path))
    img.MakeVisible() if on else img.MakeInvisible()


def _show_cut_piece(n: int, loose: bool) -> None:
    """The cable's own rings below the cut hidden and the sleeve's shown (loose), or the other way round."""
    for k in range(N_RINGS):
        _show(_PATHS[RING_NAMES[k]], not (loose and k < n))
        _show(f"{_PATHS['sleeve']}/s{k + 1}", loose and k < n)


# -- followed after every control step: the press, the turn, the cut along, the nick check and the peel ------------------

def _fresh() -> dict:
    return {"seated": False, "force": 0.0, "depth": None, "rel": None, "arc": None, "cut_at": None,
            "coverage": 0.0, "best_coverage": 0.0, "cut": False, "cut_depth": None, "n_cut": 0,
            "slit": set(), "slit_done": False, "prev_s": None, "prev_seated": False, "pen": 0.0, "max_pen": 0.0,
            "nicked": False, "loose_s": 0.0, "armed": False, "stripped": False, "torque": 0.0, "pull": 0.0,
            "tcps": None, "view": None, "view_err": None, "peel_off": 0.0, "peel_target": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRIP_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if "sleeve" in _PATHS:
        _show_cut_piece(N_RINGS, False)


def _tcps(base, origin) -> list[np.ndarray]:
    from yam_arena.constants import TCP_OFFSET

    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _contact_force() -> np.ndarray:
    """The force on the cable from the knife (N, world), from PhysX contact reports."""
    if _EP["view"] is None and _EP["view_err"] is None:
        try:
            from isaaclab_physx.physics import PhysxManager

            sim = PhysxManager.get_physics_sim_view()
            _EP["view"] = sim.create_rigid_contact_view([_PATHS["wire"]], filter_patterns=[[_PATHS["knife"]]],
                                                        max_contact_data_count=64)
        except Exception as e:  # noqa: BLE001
            _EP["view_err"] = repr(e)[:300]
    if _EP["view"] is None:
        return np.zeros(3)
    m = _EP["view"].get_contact_force_matrix(dt=PHYSICS_DT)
    m = np.asarray(m.numpy(), float) if hasattr(m, "numpy") and not hasattr(m, "detach") else as_numpy(m)
    return m.reshape(-1, 3).sum(axis=0)


def _set_wrench(base, name: str, force, tz: float) -> None:
    """A force (N, world, at the centre of mass) and a torque about the vertical (N m) on a body, held until changed:
    the cuts' resistance."""
    import torch

    obj = base.scene[name]
    f = torch.zeros((1, 1, 3), device=obj.device)
    t = torch.zeros((1, 1, 3), device=obj.device)
    f[0, 0, :] = torch.tensor([float(v) for v in force], device=obj.device)
    t[0, 0, 2] = float(tz)
    try:
        obj.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, is_global=True)
    except Exception as e:  # noqa: BLE001
        _EP["view_err"] = "wrench: " + repr(e)[:200]


def _yaw(rot) -> float:
    return math.atan2(rot[1, 0], rot[0, 0])


def _unwrap(prev, now) -> float:
    if prev is None:
        return now
    d = (now - prev + math.pi) % (2 * math.pi) - math.pi
    return prev + d


def _slit_bins(a: float, b: float) -> range:
    lo, hi = sorted((a, b))
    return range(max(0, int(math.floor(lo / SLIT_BIN))), min(int(END_L / SLIT_BIN), int(math.floor(hi / SLIT_BIN)) + 1))


def _slit_complete() -> bool:
    """Cut along from the ring cut to the cable's end."""
    if _EP["cut_depth"] is None:
        return False
    need = _slit_bins(SLIT_END, _EP["cut_depth"] - SAME_CUT)
    return all(k in _EP["slit"] for k in need)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    w_pos, w_rot = pose("wire")
    k_pos, k_rot = pose("knife")
    s_pos, s_rot = pose("sleeve")
    _EP["tcps"] = _tcps(base, origin)
    axis = w_rot[:, 2]
    s = (k_pos[2] - w_pos[2]) / max(axis[2], 0.2)  # along the cable from its end to the blade's plane
    at_blade = w_pos + s * axis
    local = k_rot.T @ (at_blade - k_pos)  # where the cable's axis crosses the blade's plane, in the knife's frame
    force_w = _contact_force()
    force = float(force_w @ k_rot[:, 0])  # the face pushing the cable forward, away from the knife
    _EP["force"], _EP["depth"] = force, s
    in_bracket = 0.0 < local[0] < BRACKET_D and abs(local[1]) < BRACKET_HALF and abs(local[2]) < 0.004
    on_face = local[0] < WIRE_R + 0.0015 and abs(local[1]) <= ON_BLADE  # the cable against the face, on the blade
    insulated = 0.0 < s < END_L - 0.001 and not _EP["armed"]
    seated = in_bracket and on_face and force >= F_SEAT and insulated
    _EP["seated"] = seated
    # the nick check: the face rides on the insulation, so the blade's edge is its set depth in, less any gap between
    # the face and the cable, and less the cable's curve away from it where the cable sits beside the blade's middle;
    # past the insulation's thinnest (INS_MIN) it reaches the strands
    side = max(0.0, abs(local[1]) - BLADE_W / 2)
    pen = (BLADE_DEPTH - max(0.0, local[0] - WIRE_R) - (WIRE_R - math.sqrt(max(0.0, WIRE_R ** 2 - side ** 2)))
           if seated else 0.0)
    _EP["pen"] = pen
    _EP["max_pen"] = max(_EP["max_pen"], pen)
    if pen > INS_MIN + 1e-9:
        _EP["nicked"] = True
    rel = _unwrap(_EP["rel"], _yaw(w_rot) - _yaw(k_rot))
    prev_rel = _EP["rel"]
    _EP["rel"] = rel
    tq, pull = 0.0, 0.0
    if seated and not _EP["cut"]:  # the cut round
        if _EP["cut_at"] is None or abs(s - _EP["cut_at"]) > SAME_CUT:  # a new cut here
            _EP["cut_at"], _EP["arc"] = s, [rel, rel]
        lo, hi = _EP["arc"]
        if prev_rel is not None and abs(rel - prev_rel) > math.radians(0.2) and (rel > hi - math.radians(2.0) or
                                                                                rel < lo + math.radians(2.0)):
            tq = -math.copysign(T_CUT, rel - prev_rel)  # cutting fresh insulation: the blade resists the turn
        _EP["arc"] = [min(lo, rel), max(hi, rel)]
        _EP["coverage"] = math.degrees(_EP["arc"][1] - _EP["arc"][0])
        _EP["best_coverage"] = max(_EP["best_coverage"], _EP["coverage"])
        if _EP["coverage"] >= RING_CUT_DEG:  # cut all round
            _EP["cut"], _EP["cut_depth"] = True, float(_EP["cut_at"])
            _EP["n_cut"] = min(N_RINGS, int(round(_EP["cut_depth"] / RING_H)))
    prev_s = _EP["prev_s"]
    if seated and _EP["prev_seated"] and prev_s is not None:  # the cut along: the pressed knife drawn along the cable
        fresh = [k for k in _slit_bins(prev_s, s) if k not in _EP["slit"]]
        if fresh and abs(s - prev_s) > 1e-4:
            _EP["slit"].update(fresh)
            pull = math.copysign(F_CUT, s - prev_s)  # along the cable's axis: against its motion through the knife
    _EP["prev_s"], _EP["prev_seated"] = s, seated
    if not _EP["slit_done"] and _slit_complete():
        _EP["slit_done"] = True
    if abs(tq - _EP["torque"]) > 1e-9 or abs(pull - _EP["pull"]) > 1e-9:
        _set_wrench(base, "wire", pull * axis, tq)
        _set_wrench(base, "knife", -pull * axis, -tq)
        _EP["torque"], _EP["pull"] = tq, pull
    # the peel: once cut round and cut along, and with the knife off it and the cable's end clear of the knife, the cut
    # piece is loose on the strands and slides off against at most F_PEEL
    end_in_knife = k_rot.T @ (w_pos - k_pos)
    clear = (abs(end_in_knife[2]) > KNIFE_B / 2 + 0.003 or end_in_knife[0] > BRACKET_D + 0.005
             or end_in_knife[0] < -0.005 or abs(end_in_knife[1]) > BRACKET_HALF + 0.005)
    _EP["loose_s"] = _EP["loose_s"] + 1.0 / 20 if abs(force) < 1.0 else 0.0
    if _EP["cut"] and _EP["slit_done"] and not _EP["armed"] and _EP["loose_s"] >= 0.25 and clear:
        set_glue_on("sleeve_on_wire", False)  # now only the slider's drive holds it, up to F_PEEL
        _show_cut_piece(_EP["n_cut"], True)
        _EP["armed"] = True
    if _EP["armed"]:
        _EP["peel_off"] = float(np.linalg.norm(s_pos - w_pos))
        slide = float(w_rot[:, 2] @ (s_pos - w_pos))  # along the cable's axis (negative: toward its end and off)
        if slide < _EP["peel_target"]:
            _EP["peel_target"] = slide
            set_slider_drive("sleeve_peel", target=slide)
        if _EP["peel_off"] >= _EP["cut_depth"] + CLEAR:
            _EP["stripped"] = True
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        busy = seated or abs(force) > 1.0 or _EP["armed"]
        every = 1 if (_EP["cut"] and not _EP["armed"]) else 2
        if _DEBUG["steps"] % 10 == 0 or (busy and _DEBUG["steps"] % every == 0):
            tl, tr = _EP["tcps"]
            top = w_pos + w_rot @ np.array([0, 0, WIRE_L])
            print(f"[strip] t={_DEBUG['steps'] / 20:5.2f}s F={force:5.1f}N seat={int(seated)} depth={s * 1000:6.1f}mm "
                  f"inKnife=({local[0] * 1000:5.1f},{local[1] * 1000:5.1f},{local[2] * 1000:5.1f})mm "
                  f"pen={pen * 1000:4.2f}mm nick={int(_EP['nicked'])} rel={math.degrees(rel):7.1f} "
                  f"cov={_EP['coverage']:5.1f} cut={int(_EP['cut'])} slit={len(_EP['slit'])}/{int(_EP['slit_done'])} "
                  f"armed={int(_EP['armed'])} off={_EP['peel_off'] * 1000:5.1f} strip={int(_EP['stripped'])} "
                  f"tq={tq:5.3f} pull={pull:5.1f} clear={int(clear)} "
                  f"endXY={np.round((w_pos[:2] - np.array(HOLDER_AT)) * 1000, 1).tolist()} "
                  f"endZ={(w_pos[2] - TOP) * 1000:5.1f} tilt={math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))):4.1f} "
                  f"leftTCP-top={np.round((tl - top) * 1000).astype(int).tolist()} "
                  f"rightTCP-knife={np.round(k_rot.T @ (tr - k_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-end={np.round((tr - w_pos) * 1000).astype(int).tolist()} "
                  f"sleeveOff={np.round((s_pos - w_pos) * 1000, 1).tolist()} "
                  f"knifeYaw={math.degrees(_yaw(k_rot)):5.1f} knifeZ={(k_pos[2] - TOP) * 1000:5.1f} "
                  f"score={_points(_status())} err={_EP['view_err']}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    depth = _EP["cut_depth"]
    good = depth is not None and STRIP[0] - 1e-9 <= depth <= STRIP[1] + 1e-9
    return {"stripped": _EP["stripped"], "cut": _EP["cut"], "slit": _EP["slit_done"], "good": good, "depth": depth,
            "partial": _EP["best_coverage"] >= PARTIAL_DEG, "nicked": _EP["nicked"]}


def _points(st) -> float:
    if st["stripped"] and st["good"]:
        p = 1.0
    elif st["stripped"] or (st["cut"] and st["good"]):
        p = 0.5
    else:
        p = 0.25 if st["partial"] else 0.0
    return min(p, 0.25) if st["nicked"] else p


def score(objects) -> float:
    """1.0 with the insulation cut round 25 to 29 mm from the cable's end (the knife pressed on with at least 4.5 N while
    the cable turned a full turn against it), cut along from there to the end (the cable drawn through the pressed knife,
    its axis within 3 mm of the blade's middle) and
    peeled off (the cut piece pulled off the strands), the blade never deeper than the insulation's 0.98 mm minimum
    thickness; 0.5 for a strip of another length, or a cut round of the right length not peeled off; 0.25 for the
    knife pressed on and the cable turned at least a quarter turn against it at some point; a nicked conductor (the
    blade deeper than 0.98 mm) scores at most 0.25; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the cable and the knife still."""
    return score(objects) >= 1.0 and objects["wire"].resting() and objects["knife"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right picks the knife up off the bench by the back of its body, carries it high over the standing cable and lowers
# it, its bracket's loop going down over the cable onto the gauge block. The left takes the cable by its grip. The right
# presses the knife onto the cable and keeps it pressed; the left turns the cable in six quarter turns, letting go and
# taking it again between them, the press eased to a touch meanwhile (the cut round). The right eases the knife back;
# both lift together, the cable out of its hole and the knife off the block; the right presses the knife on again and
# draws it down along the held cable, past its end, back onto the block (the cut along), and lets go of it. The left
# lifts the cable clear and holds it out in front of the block. The right takes the cut piece from the side and pulls it down off the strands (the peel);
# both hold still with the bare copper showing.

LEFT_GRIP_YAW = -45.0  # degrees: the left's fingers on the grip's flats
HOLD_DOWN = 0.015  # the left TCP this far below the cable's top
SEAT = 0.003  # hold the cable this much lower than where it was taken (it stays down in its hole)
THREAD = 0.014  # the cable's axis this far in front of the knife's face as its loop goes down over it
PRESS = 0.015  # the knife aimed this far past touching the cable: it pushes 7 to 20 N
PRESS_S = 2.0  # the press comes on slowly
PRESS_LIGHT = 0.003  # while the left lets go and takes the cable again, the knife aimed only this far past touching
PRESS_Z = -0.015  # pressed, aim this much into the block: the knife stays down on it
FINGER_WIDE = -0.030  # the fingers opened wide to take the cable again
SLIT_LIFT, SLIT_S = 0.055, 6.0  # the cut along: the cable drawn up this far through the pressed knife, this slowly
SLIT_HOLD = 3.0  # then the knife kept pressed this long while the left's arm catches up with its aim
PRESS_D, PRESS_Z_D = 0.008, -0.045  # while it is drawn up, the knife aimed this far past touching, and down into the block
Y_AIM = 0.0  # pressed, aim the knife this far to its side (the guide strips keep it square)
PEEL_AT = (0.36, 0.12)  # where the left holds the cable out for the peel, in front of the block
PEEL_END_Z = 0.075  # the cable's end this high above the table there
PEEL_GRIP = 0.010  # the right takes the cut piece this far above the cable's end
PEEL_PULL, PEEL_LIFT = 0.065, 0.010  # the right pulls it this far down while the left lifts the cable this far
PEEL_S = 3.0  # the pull takes this long
PEEL_BACK = 0.07  # the right comes in from this far beside the cable
PEEL_TILT = 15.0  # degrees: the right's level approach tipped down this much (its finger roots kept off the table)
GRIP_BACK = 0.065  # the right holds the knife this far behind its face, behind the guide strips
KNIFE_TWIST = -4.5  # degrees: the pads turn the knife this much the other way in the grip (measured in the simulator)
KNIFE_LEAN = 60.0  # degrees: the right's grip leans back along the knife, its wrist camera away from the left hand
KNIFE_Z_AIM = 0.003  # aim the held knife this much above the block: it settles onto it
THREAD_AIM = -0.0043  # and stops this much short of where it is aimed, along the knife (measured in the simulator)
KNIFE_HIGH = 0.10  # the knife carried this high: its loop over the standing cable's top
STROKE_S = 0.5  # per 45 degrees of turn
TURNS = ((-45.0, 45.0),) * 6  # the left grip's yaw: six quarter turns, 540 degrees
LEAD_S, LEAD_STEPS = 3.0, 15
VARIANT = os.environ.get("STRIP_DEMO", "")  # checks: "one_hand" (right idle), "no_slit" (knife let off before the lift)


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


def _g(deg: float) -> np.ndarray:
    return grip_rotation(math.radians(deg))


def _kx() -> np.ndarray:
    return np.array([math.cos(KNIFE_YAW), math.sin(KNIFE_YAW), 0.0])


def _knife_grip(face_xy, blade_z) -> np.ndarray:
    """The right TCP holding the knife whose face's middle is at face_xy, its blade's plane at blade_z."""
    return np.array([face_xy[0], face_xy[1], blade_z + KNIFE_B / 2 - 0.010]) - GRIP_BACK * _kx()


def _side_grip(approach, tilt_deg: float = 0.0) -> np.ndarray:
    """A TCP rotation coming in along ``approach`` (world, horizontal) tipped down ``tilt_deg``, the fingers closing
    level across it."""
    h = np.array([approach[0], approach[1], 0.0], float)
    h /= np.linalg.norm(h)
    t = math.radians(tilt_deg)
    z = h * math.cos(t) - np.array([0.0, 0.0, math.sin(t)])
    x = np.array([h[1], -h[0], 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY
    from yam_arena.urdf import axis_angle_matrix

    kx = _kx()
    yk = np.array([-math.sin(KNIFE_YAW), math.cos(KNIFE_YAW), 0.0])
    gk = axis_angle_matrix(yk, -math.radians(KNIFE_LEAN)) @ _g(math.degrees(KNIFE_YAW) + KNIFE_TWIST)
    blade_z = TOP + GAUGE_TOP + KNIFE_B / 2 + KNIFE_Z_AIM  # the knife lying on the gauge block
    pick = _knife_grip(np.array(KNIFE_AT), TOP + KNIFE_B / 2)
    face_thread = np.array(HOLDER_AT) - (THREAD + THREAD_AIM) * kx[:2]
    face_press = np.array(HOLDER_AT) - (WIRE_R - PRESS) * kx[:2]
    on_block = _knife_grip(face_thread, blade_z)
    over = on_block + [0.0, 0.0, KNIFE_HIGH]  # the loop over the cable's top
    # the right: the knife off the bench, up and over the standing cable, its loop down round the cable onto the block
    right = _lead_in("right", pick + [0.0, 0.0, 0.04], gk, "over the back of the knife", FINGER_OPEN)
    right += [_w(1.0, pick, -(BODY_H / 2 + 0.006), "down round the knife's body", gk),
              _w(0.8, pick, FINGER_CLOSED, "take it", gk),
              _w(1.5, pick + [0.0, 0.0, KNIFE_HIGH - 0.01], FINGER_CLOSED, "lift it", gk),
              _w(2.0, over, FINGER_CLOSED, "its loop over the cable's top", gk),
              _w(2.5, on_block, FINGER_CLOSED, "down round the cable onto the gauge block", gk)]
    t_down = sum(w.duration for w in right)
    # the left: waits clear, then takes the cable by its grip
    top = np.array([HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + WIRE_L - HOLD_DOWN])
    ff = -(GRIP / 2 + 0.006)  # fingers open a little wider than the grip's flats
    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    left = [Waypoint(max(t_down - LEAD_S, 0.2), tuple(float(v) for v in p0), FINGER_OPEN, "wait at READY", r0)]
    left += _lead_in("left", top + [0.0, 0.0, 0.04], _g(LEFT_GRIP_YAW), "over the cable's grip", FINGER_OPEN)
    left += [_w(1.0, top, ff, "down round the grip", _g(LEFT_GRIP_YAW)),
             _w(0.8, top, FINGER_CLOSED, "take it", _g(LEFT_GRIP_YAW))]
    right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), on_block, FINGER_CLOSED,
                    "hold it on the block", gk))
    # the cut round. The knife stays pressed on. Each turn after the first: the left lets go (the pressed knife leans
    # the cable over in its hole), turns its open hand back and takes the cable again, its fingers closing nearly along
    # the knife's push so that they stand the cable up again; then it turns the cable a quarter turn.
    seat = top - [0.0, 0.0, SEAT]
    pressed = _knife_grip(face_press + Y_AIM * yk[:2], blade_z + PRESS_Z)
    pressed_d = _knife_grip(np.array(HOLDER_AT) - (WIRE_R - PRESS_D) * kx[:2] + Y_AIM * yk[:2], blade_z + PRESS_Z_D)
    right.append(_w(PRESS_S, pressed, FINGER_CLOSED, "press the knife onto the cable", gk))
    left.append(_w(max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.2), seat, FINGER_CLOSED,
                   "hold while the knife is pressed on", _g(LEFT_GRIP_YAW)))
    light = _knife_grip(np.array(HOLDER_AT) - (WIRE_R - PRESS_LIGHT) * kx[:2] + Y_AIM * yk[:2], blade_z + PRESS_Z)
    cur = LEFT_GRIP_YAW
    for start, end in TURNS:
        if abs(start - cur) > 1e-6:
            right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), pressed, FINGER_CLOSED,
                            "keep it pressed on while the cable turns", gk))
            right.append(_w(0.4, light, FINGER_CLOSED, "ease the press while the left lets go", gk))
            left.append(_w(0.4, seat, FINGER_WIDE, "let go of the cable", _g(cur)))
            steps = max(1, int(round(abs(start - cur) / 45.0)))
            for k in range(1, steps + 1):
                left.append(_w(0.35, seat, FINGER_WIDE, "turn the open hand back", _g(cur + (start - cur) * k / steps)))
            left.append(_w(0.6, seat, FINGER_CLOSED, "take the cable again", _g(start)))
            right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), light, FINGER_CLOSED,
                            "hold it lightly on while the left takes the cable again", gk))
            right.append(_w(0.4, pressed, FINGER_CLOSED, "press it on again", gk))
            left.append(_w(0.4, seat, FINGER_CLOSED, "hold while the knife is pressed on again", _g(start)))
        steps = int(round(abs(end - start) / 45.0))
        for k in range(1, steps + 1):
            left.append(_w(STROKE_S, seat, FINGER_CLOSED, "turn the cable against the pressed knife",
                           _g(start + (end - start) * k / steps)))
        cur = end
    g_end = _g(cur)
    right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), pressed, FINGER_CLOSED,
                    "keep it pressed on while the cable turns", gk))
    eased = on_block  # the knife back off the cable (its axis 10 mm in front of the face), the loop still round it
    if VARIANT == "no_slit":  # the check: the knife let off before the cable is drawn up
        right.append(_w(1.0, eased, FINGER_CLOSED, "ease the knife back before the lift", gk))
        left.append(_w(sum(w.duration for w in right) - sum(w.duration for w in left), seat, FINGER_CLOSED, "hold",
                       g_end))
    # the cut along: the left draws the cable straight up through the pressed knife, the backstop behind it; the blade
    # runs along the cut piece from the cut round to the cable's end
    left.append(_w(0.5, seat, FINGER_CLOSED, "hold", g_end))
    drawn = seat + [0.0, 0.0, SLIT_LIFT]
    left.append(_w(SLIT_S, drawn, FINGER_CLOSED, "draw the cable up through the pressed knife: the cut along", g_end))
    left.append(_w(SLIT_HOLD, drawn, FINGER_CLOSED, "hold up while the cut along runs out past the end", g_end))
    t_slit = sum(w.duration for w in left)
    if VARIANT != "no_slit":
        right.append(_w(0.5, pressed_d, FINGER_CLOSED, "keep it pressed for the cut along, down on the block", gk))
        right.append(_w(max(t_slit - sum(w.duration for w in right), 0.2), pressed_d, FINGER_CLOSED,
                        "keep it pressed while the cable is drawn up", gk))
        right.append(_w(1.0, eased, FINGER_CLOSED, "ease the knife back", gk))
    left.append(_w(max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.2), drawn, FINGER_CLOSED,
                   "hold", g_end))
    # the right lets go of the knife and backs off; the left lifts the cable clear and holds it out in front of the block
    right.append(_w(0.6, eased, -(BODY_H / 2 + 0.006), "let go of the knife", gk))
    right.append(_w(1.0, eased + [0.0, 0.0, 0.06], FINGER_OPEN, "up off the knife", gk))
    held = np.array([PEEL_AT[0], PEEL_AT[1], TOP + PEEL_END_Z + WIRE_L - HOLD_DOWN])
    left.append(_w(1.0, seat + [0.0, 0.0, 0.07], FINGER_CLOSED, "lift the cable clear of the knife", g_end))
    left.append(_w(1.5, held, FINGER_CLOSED, "hold it out in front of the block", g_end))
    # the peel: the right comes in level from its side, takes the cut piece and pulls it down off the strands
    approach = np.array([0.0, 1.0, 0.0])
    gs = _side_grip(approach, PEEL_TILT)
    piece = np.array([PEEL_AT[0], PEEL_AT[1], TOP + PEEL_END_Z + PEEL_GRIP])
    beside = piece - PEEL_BACK * gs[:, 2]
    t_l = sum(w.duration for w in left)
    right.append(_w(max(t_l - sum(w.duration for w in right), 0.2), eased + [0.0, 0.0, 0.06], FINGER_OPEN,
                    "wait while the cable is held out", gk))
    right.append(_w(2.2, beside + [0.0, 0.0, 0.06], FINGER_OPEN, "round to the cable's side", gs))
    right.append(_w(0.8, beside, FINGER_OPEN, "level with the cut piece", gs))
    right.append(_w(1.5, piece, FINGER_OPEN, "in round the cut piece", gs))
    right.append(_w(0.8, piece, FINGER_CLOSED, "take the cut piece", gs))
    left.append(_w(sum(w.duration for w in right) - sum(w.duration for w in left), held, FINGER_CLOSED, "hold it out",
                   g_end))
    right.append(_w(PEEL_S, piece - [0.0, 0.0, PEEL_PULL], FINGER_CLOSED, "pull it down off the strands: the peel", gs))
    right.append(_w(1.5, piece - [0.0, 0.0, PEEL_PULL], FINGER_CLOSED, "hold it down there", gs))
    left.append(_w(PEEL_S + 1.5, held + [0.0, 0.0, PEEL_LIFT], FINGER_CLOSED, "lift the cable against the peel", g_end))
    right.append(_w(1.0, piece - [0.0, 0.0, PEEL_PULL] - 0.03 * approach, FINGER_CLOSED, "show it", gs))
    hold = "hold still: the trial ends once the full score has held 1 s"
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    t_end = max(t_l, t_r) + 3.0
    left.append(_w(t_end - t_l, left[-1].pos, FINGER_CLOSED, hold, g_end))
    right.append(_w(t_end - t_r, right[-1].pos, FINGER_CLOSED, hold, gs))
    if VARIANT == "one_hand":  # the check: the right arm stays at its start; the left does all of its motion
        p0r, r0r = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
        right = [Waypoint(t_end, tuple(float(v) for v in p0r), FINGER_OPEN, "idle at READY", r0r)]
    return {"left": left, "right": right}
