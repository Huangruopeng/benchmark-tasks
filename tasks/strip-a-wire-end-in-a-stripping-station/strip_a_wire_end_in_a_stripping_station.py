"""On the station table a short single-core 35 mm2 cable (H07V-K, 10.3 mm across) stands on its free end in the 12 mm hole
of a beech gauge block, a plug on its other end; beside it lies a round-cable stripping knife, a JOKARI No. 27 SECURA
(part 10270, for round cables 8 to 28 mm across), its blade set 1.2 mm deep, the thickness of the cable's insulation.

Task: strip 27 mm of insulation off the cable's free end with the knife, as for a 35 mm2 ferrule (25 mm sleeve, 29 mm
longest strip). The knife's steel bracket goes over the cable and the knife lies on the gauge block, which puts its blade
27 mm above the cable's end; the knife is pressed onto the cable (its blade into the insulation, at least 6 N) while the
cable is turned against it more than a full turn (the ring cut); then the cable is pulled out of the still-pressed knife
(25 N), so that the blade holds the cut insulation back and the copper comes out bare. With no hand pressing the knife,
the blade does not cut; with no hand on the cable, nothing turns or pulls it.

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
                     plate, rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PHYSICS_DT, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cable: H07V-K 1 x 35 mm2 (class 5 flexible copper 7.9 mm across, 1.2 mm of PVC insulation, 10.3 mm overall,
# 340 g/m), stiff here, 10 cm long, its top 3 cm moulded into a square ABS plug; it stands on its free end in the gauge
# block's hole. The insulation over its last 34 mm, where the cut can be made, is shown as 2 mm rings ---------------------------
WIRE_L, WIRE_R, CORE_R = 0.10, 0.00515, 0.00395
INS_T = WIRE_R - CORE_R  # 1.2 mm of insulation
RING_H, N_RINGS = 0.002, 17
PLUG, PLUG_H = 0.016, 0.03
CABLE_KG_M = 0.340
STEEL_RHO, ABS_RHO, PVC_RHO, G = 7850.0, 1050.0, 1400.0, 9.81
INS_KG_M = math.pi * (WIRE_R ** 2 - CORE_R ** 2) * PVC_RHO  # 48 g/m of the 340 is the PVC
CABLE_MASS = CABLE_KG_M * WIRE_L  # 34.0 g
PLUG_MASS = (PLUG * PLUG - math.pi * WIRE_R ** 2) * PLUG_H * ABS_RHO  # 5.4 g
RING_MASS = INS_KG_M * RING_H  # 0.096 g
WIRE_MASS = CABLE_MASS + PLUG_MASS - N_RINGS * RING_MASS  # the rest: 39.4 g in all with the rings
WIRE_COM = ((CABLE_KG_M - INS_KG_M) * WIRE_L * WIRE_L / 2 + INS_KG_M * (WIRE_L - N_RINGS * RING_H) * (WIRE_L + N_RINGS * RING_H) / 2
            + PLUG_MASS * (WIRE_L - PLUG_H / 2)) / WIRE_MASS  # up from the cable's end
WIRE_YAW = math.radians(45.0)  # the plug's flats at 45 degrees to the table's edges

# -- the gauge block (the strip-length stop): a beech block on which the knife lies, with a 12 mm hole (0.85 mm all round
# the 10.3 mm cable), 12.5 mm deep, for the cable's end. The hole's floor is 27 mm below the blade's plane
# when the knife lies on the block, so the cut falls 27 mm from the cable's end. It runs along the knife's axis ---------
HOLDER_AT = (0.30, 0.07)
HOLDER_R, HOLDER_BASE, HOLDER_WALL = 0.006, 0.004, 0.002  # the hole: 12 mm across, its floor 4 mm above the table
GAUGE_BACK, GAUGE_FRONT, GAUGE_HALF_W = 0.125, 0.025, 0.025  # from the hole's axis, along and across the knife's axis
GAUGE_TOP = HOLDER_BASE + 0.027 - 0.029 / 2  # 16.5 mm: the hole's floor + the 27 mm strip - half the knife's width
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
BLADE_DEPTH = INS_T  # the blade set 1.2 mm out of the face (the depth screw, to the insulation's thickness)
KNIFE_MASS = 0.072
KNIFE_COM = -0.046  # behind the face: the body and the steel bracket in front
KNIFE_YAW = math.radians(60.0)  # its x axis in the world, working: from the right hand toward the cable
KNIFE_AT = (0.39, -0.02)  # where its front face lies on the bench at the start
YELLOW, BLUE, RED, STEEL, BLACK, COPPER, GREY = ((0.98, 0.80, 0.08), (0.10, 0.25, 0.65), (0.80, 0.10, 0.10),
                                                 (0.66, 0.67, 0.70), (0.08, 0.08, 0.09), (0.80, 0.50, 0.25),
                                                 (0.55, 0.56, 0.58))

# -- the cut and the pull: a blade of included edge angle BLADE_EDGE_DEG and thickness BLADE_T advancing through PVC
# whose flow stress is its tensile strength (PVC_SIGMA, the minimum for PVC/C insulation, IEC 60227-1) needs, per mm of
# edge, F' = 2 sigma h (tan(edge/2) + mu), h = t / (2 tan(edge/2)) the depth over which its flanks spread the PVC to the
# blade's thickness: F' = sigma t (1 + mu / tan(edge/2)). Over the 1.2 mm of edge in the insulation that is the cutting
# force F_CUT; at the middle of the insulation (4.55 mm from the axis) the cutting torque T_CUT. The blade only stays at
# its depth while the knife is pressed against the cable with at least F_SEAT, taken equal to F_CUT -------------------
PVC_SIGMA, BLADE_EDGE_DEG, BLADE_MU, BLADE_T = 12.5, 20.0, 0.2, 0.4  # N/mm2, degrees, -, mm (thickness assumed)
F_CUT = PVC_SIGMA * BLADE_T * (1 + BLADE_MU / math.tan(math.radians(BLADE_EDGE_DEG / 2))) * INS_T * 1e3  # 12.8 N
R_MID = (WIRE_R + CORE_R) / 2  # 4.55 mm
T_CUT = F_CUT * R_MID  # 0.058 N m
F_SEAT = PVC_SIGMA * BLADE_T * INS_T * 1e3  # 6.0 N: the PVC's push on the blade's leading edge (0.4 x 1.2 mm)
F_STRIP = 25.0  # N: pulling the conductor out of the cut 27 mm of insulation (its friction on the strands)
RING_CUT_DEG = 360.0  # one full turn of the cable against the pressed knife cuts the insulation all round
SAME_CUT = 0.0015  # m: the knife kept within this of where it started cutting (along the cable) is the same cut

# -- scoring --------------------------------------------------------------------------------------------------------------
STRIP_TARGET = 0.027
STRIP = (0.025, 0.029)  # a 35 mm2 ferrule: 25 mm sleeve, 29 mm longest strip
CLEAR = 0.002  # stripped: the conductor's end this far above the blade
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
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.40, shapes=shapes, kinematic=True,
                      friction=(0.4, 0.35), color=BEECH, yaw=KNIFE_YAW)


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


SLEEVE = 3  # the wire's shape that is its end's insulation, for colliding only (hidden: the rings show it)


def _wire() -> Body:
    """Origin at the cable's end (its bottom, standing upright); the copper runs up through the rings. The insulation
    over the end collides as one hidden cylinder of the cable's full width (the rings only show it)."""
    strip = RING_H * N_RINGS
    cable = WIRE_L - PLUG_H
    shapes = [CylShape((0.0, 0.0, strip / 2), CORE_R, strip, COPPER, collide=False),
              CylShape((0.0, 0.0, strip + (cable - strip) / 2), WIRE_R, cable - strip, BLACK),
              BoxShape((0.0, 0.0, cable + PLUG_H / 2), (PLUG, PLUG, PLUG_H), GREY),
              CylShape((0.0, 0.0, strip / 2), WIRE_R, strip, BLACK)]  # SLEEVE
    return Body.build("wire", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0), mass=WIRE_MASS, shapes=shapes, yaw=WIRE_YAW,
                      friction=(0.8, 0.7), color=BLACK, com=(0.0, 0.0, WIRE_COM))  # PVC and ABS


def _rings() -> list[Body]:
    return [Body.build(f"ring_{k:02d}", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + (k + 0.5) * RING_H), mass=RING_MASS,
                       shapes=[CylShape((0.0, 0.0, 0.0), WIRE_R, RING_H * 0.98, BLACK, collide=False)], color=BLACK,
                       yaw=WIRE_YAW)
            for k in range(N_RINGS)]


HOLDER, KNIFE, WIRE = _holder(), _knife(), _wire()
RINGS = _rings()
RING_NAMES = [r.name for r in RINGS]
OBJECTS = [HOLDER, KNIFE, WIRE, *RINGS]
for _r in RINGS:
    glue(f"{_r.name}_on_wire", "wire", _r.name, anchor=_r.pos)

# -- joints authored by hand (off at the start, never switched on by a reset): each ring held in the knife's blade or
# lying in the gauge block's hole; the conductor held in its cut insulation (only along the cable; it breaks at F_STRIP) -----------
_JOINTS: dict[str, str] = {}
_PATHS: dict[str, str] = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: contact reports on the wire and the knife; the hidden sleeve; this task's own joints."""
    _author_sliders_plain(stage, prim_path, spec)
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    if spec.name in ("wire", "knife"):
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateThresholdAttr(0.0)
        _PATHS[spec.name] = f"{prim_path}/body"

    def joint(kind, name, body0):
        j = kind.Define(stage, f"{prim_path}/{name}")
        j.CreateBody0Rel().SetTargets([f"{parent}/{body0}/body"])
        j.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        for attr in (j.CreateLocalPos0Attr, j.CreateLocalPos1Attr):
            attr(Gf.Vec3f(0.0, 0.0, 0.0))
        for attr in (j.CreateLocalRot0Attr, j.CreateLocalRot1Attr):
            attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateJointEnabledAttr(False)
        j.CreateCollisionEnabledAttr(True)
        j.CreateExcludeFromArticulationAttr(True)
        _JOINTS[name if spec.name == "wire" else f"{spec.name}_{name}"] = f"{prim_path}/{name}"
        return j

    if spec.name == "wire":
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/body/s{SLEEVE}")).MakeInvisible()
        j = joint(UsdPhysics.Joint, "slug_hold", "holder")  # a D6 joint locked along the cable only
        limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), "transZ")
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)  # low > high: locked
        j.CreateBreakForceAttr(float(F_STRIP))
        j.CreateBreakTorqueAttr(1.0e6)
    elif spec.name in RING_NAMES:
        joint(UsdPhysics.FixedJoint, "in_knife", "knife")
        joint(UsdPhysics.FixedJoint, "in_holder", "holder")


bodykit._author_sliders = _author_sliders


# -- followed after every control step: the press, the turn, the cut and the pull --------------------------------------

def _fresh() -> dict:
    return {"seated": False, "force": 0.0, "depth": None, "off": None, "rel": None, "arc": None, "cut_at": None,
            "coverage": 0.0, "best_coverage": 0.0, "cut": False, "cut_depth": None, "n_cut": 0, "holding": False,
            "unseated_s": 0.0, "stripped": False, "torque": 0.0, "tcps": None, "view": None, "view_err": None,
            "peak_force": 0.0, "end_z": None, "knife_xy": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRIP_DEBUG")), "steps": 0}


def _joint_prim(name: str):
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage().GetPrimAtPath(_JOINTS[name])


def _set_on(name: str, on: bool) -> None:
    from pxr import UsdPhysics

    UsdPhysics.Joint(_joint_prim(name)).GetJointEnabledAttr().Set(bool(on))


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


def _hold(name: str, b0_pos, b0_rot, b1_pos, b1_rot, at=None) -> None:
    """Switch one of this task's joints on with its frames on body1's origin as it is now (or ``at``, a world point,
    and body1 to go to b1_pos/b1_rot), against body0 as it is now. Off first: that also rebuilds a joint that broke."""
    from pxr import Gf, UsdPhysics

    _set_on(name, False)
    j = UsdPhysics.Joint(_joint_prim(name))
    p1 = np.zeros(3) if at is None else b1_rot.T @ (np.asarray(at) - np.asarray(b1_pos))
    p0 = b0_rot.T @ (np.asarray(b1_pos) + b1_rot @ p1 - np.asarray(b0_pos))
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*p0.tolist()))
    j.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(b0_rot.T @ b1_rot)))
    j.GetLocalPos1Attr().Set(Gf.Vec3f(*p1.tolist()))
    j.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _set_on(name, True)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for name in _JOINTS:
        _set_on(name, False)


def _tcps(base, origin) -> list[np.ndarray]:
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


def _set_torque(base, name: str, tz: float) -> None:
    """A torque about the vertical on a body (N m), held until changed: the cut's resistance."""
    import torch

    obj = base.scene[name]
    t = torch.zeros((1, 1, 3), device=obj.device)
    t[0, 0, 2] = float(tz)
    try:
        obj.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(t), torques=t, is_global=True)
    except Exception as e:  # noqa: BLE001
        _EP["view_err"] = "torque: " + repr(e)[:200]


def _yaw(rot) -> float:
    return math.atan2(rot[1, 0], rot[0, 0])


def _unwrap(prev, now) -> float:
    if prev is None:
        return now
    d = (now - prev + math.pi) % (2 * math.pi) - math.pi
    return prev + d


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    w_pos, w_rot = pose("wire")
    k_pos, k_rot = pose("knife")
    h_pos, h_rot = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP]), rotation((0.0, 0.0, math.sin(KNIFE_YAW / 2),
                                                                         math.cos(KNIFE_YAW / 2)))
    _EP["tcps"] = _tcps(base, origin)
    axis = w_rot[:, 2]
    s = (k_pos[2] - w_pos[2]) / max(axis[2], 0.2)  # along the cable from its end to the blade's plane
    at_blade = w_pos + s * axis
    local = k_rot.T @ (at_blade - k_pos)  # where the cable's axis crosses the blade's plane, in the knife's frame
    force_w = _contact_force()
    force = float(force_w @ k_rot[:, 0])  # the face pushing the cable forward, away from the knife
    _EP["force"], _EP["depth"], _EP["off"] = force, s, local
    _EP["end_z"], _EP["knife_xy"] = float(w_pos[2]), local
    _EP["peak_force"] = max(_EP["peak_force"], force)
    in_bracket = 0.0 < local[0] < BRACKET_D and abs(local[1]) < BRACKET_HALF and abs(local[2]) < 0.004
    on_face = local[0] < WIRE_R + 0.0015
    insulated = 0.0 < s < N_RINGS * RING_H - 0.001 and not _EP["stripped"]
    seated = in_bracket and on_face and force >= F_SEAT and insulated
    _EP["seated"] = seated
    rel = _unwrap(_EP["rel"], _yaw(w_rot) - _yaw(k_rot))
    prev_rel = _EP["rel"]
    _EP["rel"] = rel
    tq = 0.0
    if seated and not _EP["cut"]:
        if _EP["cut_at"] is None or abs(s - _EP["cut_at"]) > SAME_CUT:  # a new cut here
            _EP["cut_at"], _EP["arc"] = s, [rel, rel]
        lo, hi = _EP["arc"]
        if prev_rel is not None and abs(rel - prev_rel) > math.radians(0.2) and (rel > hi - math.radians(2.0) or
                                                                                rel < lo + math.radians(2.0)):
            tq = -math.copysign(T_CUT, rel - prev_rel)  # cutting fresh insulation: the blade resists the turn
        _EP["arc"] = [min(lo, rel), max(hi, rel)]
        _EP["coverage"] = math.degrees(_EP["arc"][1] - _EP["arc"][0])
        _EP["best_coverage"] = max(_EP["best_coverage"], _EP["coverage"])
        if _EP["coverage"] >= RING_CUT_DEG:  # cut all round: the insulation below the blade is a loose slug
            _EP["cut"], _EP["cut_depth"] = True, float(_EP["cut_at"])
            _EP["n_cut"] = min(N_RINGS, int(round(_EP["cut_depth"] / RING_H)))
    if abs(tq - _EP["torque"]) > 1e-9:
        _set_torque(base, "wire", tq)
        _set_torque(base, "knife", -tq)
        _EP["torque"] = tq
    if _EP["cut"] and not _EP["stripped"]:
        pressed = in_bracket and local[0] < WIRE_R + 0.003 and force >= 3.0
        _EP["unseated_s"] = 0.0 if pressed else _EP["unseated_s"] + 1.0 / 20
        if not _EP["holding"] and seated and abs(s - _EP["cut_depth"]) < SAME_CUT:
            for r in RINGS[:_EP["n_cut"]]:  # the slug, held back by the blade in the cut
                r_pos, r_rot = pose(r.name)
                set_glue_on(f"{r.name}_on_wire", False)
                _hold(f"{r.name}_in_knife", k_pos, k_rot, r_pos, r_rot)
            _hold("slug_hold", h_pos, h_rot, w_pos, w_rot, at=at_blade)
            _EP["holding"] = True
        if _EP["holding"]:
            if s < -CLEAR:  # the conductor pulled out of its insulation: stripped; the slug drops into the hole
                _EP["stripped"], _EP["holding"] = True, False
                _set_on("slug_hold", False)
                for k, r in enumerate(RINGS[:_EP["n_cut"]]):
                    _set_on(f"{r.name}_in_knife", False)
                    _hold(f"{r.name}_in_holder", h_pos, h_rot, np.array([HOLDER_AT[0], HOLDER_AT[1],
                                                                         WIRE_Z0 + (k + 0.5) * RING_H]), w_rot)
            elif _EP["unseated_s"] > 0.25:  # the knife let off: the cut insulation stays on the conductor
                _EP["holding"] = False
                _set_on("slug_hold", False)
                for r in RINGS[:_EP["n_cut"]]:
                    r_pos, r_rot = pose(r.name)
                    _set_on(f"{r.name}_in_knife", False)
                    bodykit.set_glue_on(f"{r.name}_on_wire", False)
                    _hold_glue(f"{r.name}_on_wire", w_pos, w_rot, r_pos, r_rot)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        busy = seated or _EP["holding"] or abs(force) > 1.0
        if _DEBUG["steps"] % 10 == 0 or (busy and _DEBUG["steps"] % 2 == 0):
            tl, tr = _EP["tcps"]
            print(f"[strip] t={_DEBUG['steps'] / 20:5.2f}s F={force:5.1f}N seat={int(seated)} depth={s * 1000:6.1f}mm "
                  f"inKnife=({local[0] * 1000:5.1f},{local[1] * 1000:5.1f},{local[2] * 1000:5.1f})mm "
                  f"rel={math.degrees(rel):7.1f} cov={_EP['coverage']:5.1f} cut={int(_EP['cut'])} "
                  f"hold={int(_EP['holding'])} strip={int(_EP['stripped'])} tq={tq:5.3f} "
                  f"endOffHolder={np.round((w_pos[:2] - np.array(HOLDER_AT)) * 1000, 1).tolist()} "
                  f"endZ={(w_pos[2] - TOP) * 1000:5.1f} leftTCP-plug="
                  f"{np.round((tl - (w_pos + w_rot @ np.array([0, 0, WIRE_L]))) * 1000).astype(int).tolist()} "
                  f"rightTCP-knife={np.round(k_rot.T @ (tr - k_pos) * 1000).astype(int).tolist()} "
                  f"holderInKnife={np.round(k_rot.T @ (h_pos - k_pos) * 1000, 1)[:2].tolist()} "
                  f"knifeYaw={math.degrees(_yaw(k_rot)):5.1f} knifeZ={(k_pos[2] - TOP) * 1000:5.1f} "
                  f"pitch={math.degrees(math.asin(max(-1.0, min(1.0, k_rot[2, 0])))):5.1f} "
                  f"roll={math.degrees(math.asin(max(-1.0, min(1.0, k_rot[2, 1])))):5.1f} rTCPz={(tr[2] - TOP) * 1000:5.1f} "
                  f"score={_points(_status())} err={_EP['view_err']}", file=sys.stderr, flush=True)


def _hold_glue(name: str, b0_pos, b0_rot, r_pos, r_rot) -> None:
    """A bodykit glue switched back on with body1 where it is now against body0."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    set_glue_on(name, False)
    j = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*(b0_rot.T @ (np.asarray(r_pos) - np.asarray(b0_pos))).tolist()))
    j.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(b0_rot.T @ r_rot)))
    j.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    set_glue_on(name, True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    depth = _EP["cut_depth"]
    good = depth is not None and STRIP[0] - 1e-9 <= depth <= STRIP[1] + 1e-9
    return {"stripped": _EP["stripped"], "cut": _EP["cut"], "good": good, "depth": depth,
            "partial": _EP["best_coverage"] >= PARTIAL_DEG}


def _points(st) -> float:
    if st["stripped"] and st["good"]:
        return 1.0
    if st["stripped"] or (st["cut"] and st["good"]):
        return 0.5
    return 0.25 if st["partial"] else 0.0


def score(objects) -> float:
    """1.0 with the insulation cut all round 25 to 29 mm from the cable's end (the knife pressed on with at least 6 N
    while the cable turned a full turn against it) and pulled off (the conductor drawn out of it while the knife stayed
    pressed); 0.5 for a strip of another length, or a full ring cut of the right length not pulled off; 0.25 for the
    knife pressed on and the cable turned at least a quarter turn against it at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the cable and the knife still."""
    return score(objects) >= 1.0 and objects["wire"].resting() and objects["knife"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right picks the knife up off the bench by the back of its body, carries it high over the standing cable and lowers
# it, its bracket's loop going down over the plug and the cable, onto the gauge block. The left takes the cable's plug.
# The right presses the knife onto the cable and keeps it pressed; the left turns the cable by its plug in six quarter
# turns, letting go and taking it again between them. Then it pulls the cable straight up out of the still-pressed
# knife, which holds the cut insulation back: it drops into the hole. The right eases the knife back; both hold still.

LEFT_GRIP_YAW = -45.0  # degrees: the left's fingers on the plug's flats
HOLD_DOWN = 0.015  # the left TCP this far below the plug's top
SEAT = 0.006  # hold the plug this much lower than where it was taken (the cable stays down in its hole)
THREAD = 0.014  # the cable's axis this far in front of the knife's face as its loop goes down over it
PRESS = 0.012  # the knife aimed this far past touching the cable: it pushes 7 to 20 N
PRESS_S = 2.0  # the press comes on slowly
PRESS_Z = -0.015  # pressed, aim this much into the block: the knife stays down on it
FINGER_WIDE = -0.030  # the fingers opened wide to take the plug again
PULL = 0.07  # aim this far above the seated cable: the arm pulls past 25 N before the conductor comes out
GRIP_BACK = 0.090  # the right holds the knife this far behind its face
KNIFE_TWIST = -4.5  # degrees: the pads turn the knife this much the other way in the grip (measured)
KNIFE_LEAN = 60.0  # degrees: the right's grip leans back along the knife, its wrist camera away from the left hand
KNIFE_Z_AIM = 0.003  # aim the held knife this much above the block: it settles onto it
THREAD_AIM = -0.0043  # and stops this much short of where it is aimed, along the knife (measured)
KNIFE_HIGH = 0.10  # the knife carried this high: its loop over the standing cable's plug
STROKE_S = 0.5  # per 45 degrees of turn
TURNS = ((-45.0, 45.0),) * 6  # the left grip's yaw: six quarter turns, 540 degrees
LEAD_S, LEAD_STEPS = 3.0, 15


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
    over = on_block + [0.0, 0.0, KNIFE_HIGH]  # the loop over the cable's plug
    # the right: the knife off the bench, up and over the standing cable, its loop down over the plug and the cable onto
    # the gauge block
    right = _lead_in("right", pick + [0.0, 0.0, 0.04], gk, "over the back of the knife", FINGER_OPEN)
    right += [_w(1.0, pick, -(BODY_H / 2 + 0.006), "down round the knife's body", gk),
              _w(0.8, pick, FINGER_CLOSED, "take it", gk),
              _w(1.5, pick + [0.0, 0.0, KNIFE_HIGH - 0.01], FINGER_CLOSED, "lift it", gk),
              _w(2.0, over, FINGER_CLOSED, "its loop over the cable's plug", gk),
              _w(2.5, on_block, FINGER_CLOSED, "down round the cable onto the gauge block", gk)]
    t_down = sum(w.duration for w in right)
    # the left: waits clear, then takes the plug
    plug = np.array([HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + WIRE_L - HOLD_DOWN])
    ff = -(PLUG / 2 + 0.006)  # fingers open a little wider than the plug's flats
    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    left = [Waypoint(max(t_down - LEAD_S, 0.2), tuple(float(v) for v in p0), FINGER_OPEN, "wait at READY", r0)]
    left += _lead_in("left", plug + [0.0, 0.0, 0.04], _g(LEFT_GRIP_YAW), "over the plug", FINGER_OPEN)
    left += [_w(1.0, plug, ff, "down round the plug", _g(LEFT_GRIP_YAW)),
             _w(0.8, plug, FINGER_CLOSED, "take it", _g(LEFT_GRIP_YAW))]
    right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), on_block, FINGER_CLOSED,
                    "hold it on the block", gk))
    L = {"seat": plug - [0.0, 0.0, SEAT], "ff": ff}
    # the turns. The knife stays pressed on. Each turn after the first: the left lets go (the pressed knife leans the
    # cable over in its hole), turns its open hand back and takes the plug again, its fingers closing nearly along the
    # knife's push so that they stand the cable up again; then it turns the cable a quarter turn. The plug's flats come
    # round every 90 degrees.
    seat = L["seat"]
    pressed = _knife_grip(face_press, blade_z + PRESS_Z)
    right.append(_w(PRESS_S, pressed, FINGER_CLOSED, "press the knife onto the cable", gk))
    left.append(_w(max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.2), seat, FINGER_CLOSED,
                   "hold while the knife is pressed on", _g(LEFT_GRIP_YAW)))
    cur = LEFT_GRIP_YAW
    for start, end in TURNS:
        if abs(start - cur) > 1e-6:
            left.append(_w(0.4, seat, FINGER_WIDE, "let go of the plug", _g(cur)))
            steps = max(1, int(round(abs(start - cur) / 45.0)))
            for k in range(1, steps + 1):
                left.append(_w(0.35, seat, FINGER_WIDE, "turn the open hand back", _g(cur + (start - cur) * k / steps)))
            left.append(_w(0.6, seat, FINGER_CLOSED, "take the plug again", _g(start)))
        steps = int(round(abs(end - start) / 45.0))
        for k in range(1, steps + 1):
            left.append(_w(STROKE_S, seat, FINGER_CLOSED, "turn the cable against the pressed knife",
                           _g(start + (end - start) * k / steps)))
        cur = end
    right.append(_w(sum(w.duration for w in left) - sum(w.duration for w in right), pressed, FINGER_CLOSED,
                    "keep it pressed on while the cable turns", gk))
    end_yaw = cur
    left.append(_w(0.5, seat, FINGER_CLOSED, "hold", _g(end_yaw)))
    left.append(_w(2.0, seat + [0.0, 0.0, PULL], FINGER_CLOSED, "pull the cable straight up out of the pressed knife",
                   _g(end_yaw)))
    t_pulled = sum(w.duration for w in left)
    right.append(_w(max(t_pulled - sum(w.duration for w in right), 0.2) + 0.5, pressed,
                    FINGER_CLOSED, "keep it pressed while the cable is pulled out", gk))
    right.append(_w(1.0, _knife_grip(face_thread - 0.03 * kx[:2], blade_z), FINGER_CLOSED, "ease the knife back", gk))
    hold = "hold still: the trial ends once the full score has held 1 s"
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    t_end = max(t_l, t_r) + 3.0
    left.append(_w(t_end - t_l, left[-1].pos, FINGER_CLOSED, hold, _g(end_yaw)))
    right.append(_w(t_end - t_r, right[-1].pos, FINGER_CLOSED, hold, gk))
    return {"left": left, "right": right}
