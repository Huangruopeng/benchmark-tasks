"""On the station table stands a benchtop wire-stripping station: a block with a funnel down to two blades and a bore
under them; the blades close round whatever is in them while its lever is held down. A short cable with a plug on one
end stands upright in a holder beside it.

Task: strip 10 mm of insulation off the cable's end. One hand puts the cable's end down through the funnel, 10 mm past
the blades (as deep as it goes in, so much is stripped); the other presses the lever down (the cut takes about 20 N at
the grip), closing the blades, and holds it down while the first pulls the cable straight back up (about 20 N): the cut
insulation stays behind in the station, and the copper is bare. Let the lever up before the cable is out and the blades
let go: the cut insulation comes back out on the cable.

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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, plate, rotation, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stripping station (fixed): a steel block with a funnel and a bore down into it; origin on the table under the
# bore. Real build (all of it in the README): two hardened-steel full-radius V-blades (90-degree V entry ending in a
# semicircle 3.95 mm in radius, 1.0 mm thick, edges ground to 20 degrees) slide toward each other along y in the
# horizontal plane of the funnel's throat, symmetric about the bore's axis, on a rack and pinion that keeps them
# centred. An eccentric (circular cam) on the lever's hub, ECC off the hinge axis, runs in a vertical slot in the
# right-hand blade carrier (an eccentric-and-yoke drive) and swings from 10 degrees before to 10 degrees past straight
# down over the lever's 20 degrees: each blade moves 4.54 mm, from a 12.0 mm opening to a 7.9 mm circle ------------
SX, SY = 0.24, 0.12
BLOCK = (0.05, 0.05, 0.04)  # x, y, and the funnel's rim height (the block itself is 3 cm tall, the funnel on it)
HOLE_RI = 0.008  # the bore under the blades: 16 mm across, down to its floor 5 mm above the table (the slug drops in)
FLOOR_Z = 0.005
FUNNEL_R, FUNNEL_L = 0.022, 0.010  # a lead-in funnel: 44 mm across at its rim, 10 mm deep, down to the blades
BLADE_Z = BLOCK[2] - FUNNEL_L  # the blades' cutting plane, at the funnel's throat: 30 mm above the table
BLADE_OPEN_R = 0.006  # the open blades leave a 12.0 mm opening (0.85 mm all round the 10.3 mm cable)
BLADE_T = 0.001  # each blade 1.0 mm thick, one on the other
PIVOT = np.array([0.0, -BLOCK[1] / 2 - 0.005, BLOCK[2] + 0.012])  # the lever's hinge, along x, at the block's right side
ARM_X, ARM_Z, ARM_WALL = 0.024, 0.016, 0.001  # the lever: a steel rectangular tube 24 x 16 mm, 1 mm walls ...
ARM_BACK, ARM_END = 0.020, -0.225  # ... from 2 cm behind the hinge to 22.5 cm in front of it (to the right): 24.5 cm
HANDLE_Y, HANDLE_L = -0.20, 0.05  # the middle of its 5 cm rubber grip sleeve (3 mm thick), 20 cm from the hinge
OPEN_DEG = 20.0  # the lever's travel: up 20 degrees (open, on its stop) to 0 (closed, on its stop)
SPRING = 0.04  # N m per degree: the torsion spring on the hinge pin ...
SPRING_PRELOAD_DEG = 10.0  # ... wound 10 degrees at the open stop (0.40 N m), 30 degrees closed (1.20 N m)
SPRING_TO = -OPEN_DEG - SPRING_PRELOAD_DEG  # its free position, 10 degrees past the open stop
CLOSED_WITHIN = 0.5  # degrees: the blades are closed (on the stop)
OPENED = 4.0  # degrees: let up this far, the blades let go of the cut insulation
STEEL_RHO, RUBBER_RHO, ABS_RHO, PVC_RHO, G = 7850.0, 1200.0, 1050.0, 1400.0, 9.81  # kg/m3, m/s2

# -- the cable: an H07V-K 1 x 35 mm2 single core (class 5 flexible copper, 7.9 mm across, 1.2 mm PVC insulation, 10.3 mm
# overall, 340 g/m), stiff here, 12 cm long, its top 3 cm moulded into a square ABS plug; the insulation round its last
# 24 mm is 12 rings, each held to it until cut ------------------------------------------------------------------------
WIRE_L, WIRE_R, CORE_R = 0.12, 0.00515, 0.00395
RING_H, N_RINGS = 0.002, 12
PLUG, PLUG_H = 0.016, 0.03  # the plug, 16 mm square and 3 cm tall, moulded round the cable's top 3 cm
CABLE_KG_M = 0.340  # the datasheet's 340 g per metre
INS_KG_M = math.pi * (WIRE_R ** 2 - CORE_R ** 2) * PVC_RHO  # 48 g/m of it is the PVC insulation (34.3 mm2 at 1.4 g/cm3)
CABLE_MASS = CABLE_KG_M * WIRE_L  # 40.8 g
PLUG_MASS = (PLUG * PLUG - math.pi * WIRE_R ** 2) * PLUG_H * ABS_RHO  # the ABS round the cable's top: 5.4 g
RING_MASS = INS_KG_M * RING_H  # 0.096 g of PVC in each 2 mm ring
WIRE_MASS = CABLE_MASS + PLUG_MASS - N_RINGS * RING_MASS  # the rest (46.2 g in all, with the rings)
WIRE_COM = ((CABLE_KG_M - INS_KG_M) * WIRE_L * WIRE_L / 2 + INS_KG_M * (WIRE_L - N_RINGS * RING_H) * (WIRE_L + N_RINGS * RING_H) / 2
            + PLUG_MASS * (WIRE_L - PLUG_H / 2)) / WIRE_MASS  # up from the cable's end
HOLDER_AT = (0.30, 0.21)
HOLDER_R, HOLDER_H = 0.006, 0.018  # a 12 mm bore round the insulated end: 0.85 mm clearance all round the 10.3 mm cable
HOLDER_MASS = 0.0026  # ABS: its 20 mm x 4 mm base and 2 mm walls, 2.5 cm3
STATION_MASS = 0.65  # the steel block (0.55 kg), funnel, hinge bracket, blades, carriers and eccentric; fixed anyway
BLACK, COPPER, STEEL, DARK, RED = (0.08, 0.08, 0.09), (0.80, 0.50, 0.25), (0.62, 0.63, 0.66), (0.20, 0.21, 0.23), \
    (0.75, 0.10, 0.10)

# -- the lever's mass: the steel tube and the rubber grip sleeve ----------------------------------------------------------
_TUBE_KG = (ARM_X * ARM_Z - (ARM_X - 2 * ARM_WALL) * (ARM_Z - 2 * ARM_WALL)) * (ARM_BACK - ARM_END) * STEEL_RHO
_GRIP_KG = ((ARM_X + 0.006) * (ARM_Z + 0.006) - ARM_X * ARM_Z) * HANDLE_L * RUBBER_RHO
LEVER_MASS = _TUBE_KG + _GRIP_KG  # 146 g + 17 g = 163 g
LEVER_COM_Y = (_TUBE_KG * (ARM_BACK + ARM_END) / 2 + _GRIP_KG * HANDLE_Y) / LEVER_MASS  # 11.2 cm from the hinge
LEVER_WEIGHT_NM = LEVER_MASS * G * -LEVER_COM_Y  # 0.18 N m with the lever level (closed)

# -- the cut: the force the blades need, from a wedge model: a sharp edge of included angle BLADE_EDGE_DEG pushed h deep
# into PVC whose flow stress is its tensile strength (PVC_SIGMA, the standard's minimum for PVC/C insulation) takes
# F' = 2 sigma h (tan(edge/2) + mu) per mm of edge, mu the PVC-on-steel friction. Summed over each blade's edge (the
# semicircle and the V flanks inside the insulation), times 2 blades, through the eccentric and the mechanism's
# efficiency, it is a torque against closing the lever: up to CUT_NM (about 14.5 N more at the grip) at the end -------
PVC_SIGMA, BLADE_EDGE_DEG, BLADE_MU, MECH_EFF = 12.5, 20.0, 0.2, 0.9  # N/mm2, degrees, -, -
BLADE_R = CORE_R  # the blades' semicircles: closed, a 7.9 mm circle, the conductor's diameter
E_OPEN = (2 * BLADE_OPEN_R / math.sqrt(2) - BLADE_R)  # each blade's travel from open (inscribed radius (r + e)/sqrt 2)
E_TOUCH = math.sqrt(WIRE_R ** 2 - BLADE_R ** 2)  # the semicircles' ends meet the insulation: 3.30 mm from closed
ECC = E_OPEN / (2 * math.sin(math.radians(OPEN_DEG / 2)))  # the eccentric's offset: 13.06 mm


def _blade_e(lever_deg: float) -> float:
    """How far each blade is from closed (m) at a lever angle (0 closed, -OPEN_DEG open)."""
    phi = math.radians(lever_deg + OPEN_DEG / 2)
    return E_OPEN - ECC * (math.sin(phi) + math.sin(math.radians(OPEN_DEG / 2)))


def _blade_force(e: float, n: int = 600) -> float:
    """The insulation's push (N) on one blade along its travel, the blade e (m) from closed, the cable centred."""
    kf = 2 * PVC_SIGMA * (math.tan(math.radians(BLADE_EDGE_DEG / 2)) + BLADE_MU)  # N/mm per mm of depth
    r, ro, t_ins = BLADE_R * 1e3, WIRE_R * 1e3, (WIRE_R - CORE_R) * 1e3
    e = e * 1e3
    t = np.linspace(0.0, math.pi, n)  # the semicircle: (r cos t, e + r sin t), pushed along +y by sin t of its push
    h = np.clip(ro - np.hypot(r * np.cos(t), e + r * np.sin(t)), 0.0, t_ins)
    force = float(np.sum(kf * h * np.sin(t)) * r * (t[1] - t[0]))
    s = np.linspace(0.0, 3.0, n)  # each V flank: (+-(r + s), e - s), its push 45 degrees off the travel
    h = np.clip(ro - np.hypot(r + s, e - s), 0.0, t_ins)
    force += 2 * float(np.sum(kf * h) * (s[1] - s[0]))  # each step is sqrt(2) ds of edge, pushed 1/sqrt(2) along y
    return force


def _cut_nm(lever_deg: float) -> float:
    """The torque (N m) the cut takes at the lever's hinge, both blades, through the eccentric and its efficiency."""
    lever_deg = min(0.0, max(-OPEN_DEG, lever_deg))
    phi = math.radians(lever_deg + OPEN_DEG / 2)
    return 2 * _blade_force(_blade_e(lever_deg)) * ECC * math.cos(phi) / MECH_EFF


_CUT_DEG = np.linspace(-OPEN_DEG, 0.0, 81)
_CUT_TABLE = np.array([_cut_nm(d) for d in _CUT_DEG])
CUT_NM = float(_CUT_TABLE[-1])
F_PULL = 20.0  # N: pulling the conductor out of the 10 mm slug held in the closed blades (friction of the slug on it)

# -- scoring ----------------------------------------------------------------------------------------------------------
STRIP = (0.008, 0.012)  # a good strip: this much insulation off
INSERTED = 0.005
CLEAR = 0.002  # pulled off: the wire's end this far above the blades
RELEASED_DEG = 15.0
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _station() -> Body:
    bx, by, bz = BLOCK
    s = HOLE_RI + 0.002  # half the square shaft the bore's tube sits in
    zw = BLADE_Z  # the block's walls stop under the funnel
    shapes = [BoxShape((0.0, 0.0, 0.0025), (bx, by, 0.005), DARK),  # base, under the hole
              BoxShape(((bx / 2 + s) / 2, 0.0, zw / 2), (bx / 2 - s, by, zw), STEEL),
              BoxShape((-(bx / 2 + s) / 2, 0.0, zw / 2), (bx / 2 - s, by, zw), STEEL),
              BoxShape((0.0, (by / 2 + s) / 2, zw / 2), (2 * s, by / 2 - s, zw), STEEL),
              BoxShape((0.0, -(by / 2 + s) / 2, zw / 2), (2 * s, by / 2 - s, zw), STEEL),
              BoxShape((PIVOT[0], PIVOT[1], (bz + PIVOT[2] + 0.008) / 2), (0.02, 0.01, PIVOT[2] + 0.008 - bz), DARK)]
    n = 10
    zb = BLADE_Z - 2 * BLADE_T  # the open blades' underside
    for k in range(n):  # the bore's tube, the open blades round the funnel's throat, and the funnel
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLE_RI + 0.0008
        shapes.append(plate((r * u[0], r * u[1], FLOOR_Z), (r * u[0], r * u[1], zb),
                            2 * r * math.tan(math.pi / n) + 0.001, 0.0016, u, STEEL))
        shapes.append(plate((BLADE_OPEN_R * u[0], BLADE_OPEN_R * u[1], BLADE_Z - BLADE_T),
                            ((HOLE_RI + 0.0016) * u[0], (HOLE_RI + 0.0016) * u[1], BLADE_Z - BLADE_T),
                            2 * (HOLE_RI + 0.0016) * math.tan(math.pi / n) + 0.001, 2 * BLADE_T, (0.0, 0.0, 1.0), STEEL))
        dr, dz = FUNNEL_R - BLADE_OPEN_R, FUNNEL_L  # the funnel's inner face runs from the blades' opening to its rim
        ln = math.hypot(dr, dz)
        r0, z0 = BLADE_OPEN_R + 0.0008 * dz / ln, BLADE_Z - 0.0008 * dr / ln
        r1, z1 = FUNNEL_R + 0.0008 * dz / ln, bz - 0.0008 * dr / ln
        shapes.append(plate((r0 * u[0], r0 * u[1], z0), (r1 * u[0], r1 * u[1], z1), 2 * r1 * math.tan(math.pi / n) + 0.001,
                            0.0016, u, STEEL))
    return Body.build("stripper", pos=(SX, SY, TOP), mass=STATION_MASS, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=STEEL, slick=True)  # polished steel on PVC: friction 0.3


def _lever() -> Body:
    shapes = [BoxShape((0.0, (ARM_BACK + ARM_END) / 2, 0.0), (ARM_X, ARM_BACK - ARM_END, ARM_Z), DARK),
              BoxShape((0.0, HANDLE_Y, 0.0), (ARM_X + 0.006, HANDLE_L, ARM_Z + 0.006), RED)]
    a = math.radians(-OPEN_DEG) / 2
    return Body.build("lever", pos=(SX + PIVOT[0], SY + PIVOT[1], TOP + PIVOT[2]), mass=LEVER_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=DARK, start_xyzw=(math.sin(a), 0.0, 0.0, math.cos(a)),
                      com=(0.0, LEVER_COM_Y, 0.0))


def _holder() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), HOLDER_R + 0.004, 0.004, DARK)]
    n = 10
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + 0.001
        shapes.append(plate((r * u[0], r * u[1], 0.004), (r * u[0], r * u[1], HOLDER_H), 2 * r * math.tan(math.pi / n) + 0.001,
                            0.002, u, DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=HOLDER_MASS, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


WIRE_Z0 = TOP + 0.0045  # the wire's end, standing in its holder


SLEEVE = 3  # the wire's shape that is its end's insulation, for colliding only (hidden: the rings show it)


def _wire() -> Body:
    """Origin at the wire's end (its bottom, standing upright); the copper runs up through the rings. The insulation
    over the end collides as one hidden cylinder of the cable's full width, so the end stands in its holder's bore and
    passes the open blades' 12 mm opening as the insulated cable does (the rings only show it)."""
    strip = RING_H * N_RINGS
    cable = WIRE_L - PLUG_H  # 9 cm of the cable shows below the plug, which is moulded round its top 3 cm
    shapes = [CylShape((0.0, 0.0, strip / 2), CORE_R, strip, COPPER, collide=False),
              CylShape((0.0, 0.0, strip + (cable - strip) / 2), WIRE_R, cable - strip, BLACK),
              BoxShape((0.0, 0.0, cable + PLUG_H / 2), (PLUG, PLUG, PLUG_H), (0.55, 0.56, 0.58)),
              CylShape((0.0, 0.0, strip / 2), WIRE_R, strip, BLACK)]  # SLEEVE
    return Body.build("wire", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0), mass=WIRE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLACK, com=(0.0, 0.0, WIRE_COM))


def _rings() -> list[Body]:
    """Each ring is just to be seen (it does not collide): held to the wire, or once cut and the wire pulled out, held
    in the station's blades."""
    return [Body.build(f"ring_{k:02d}", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + (k + 0.5) * RING_H), mass=RING_MASS,
                       shapes=[CylShape((0.0, 0.0, 0.0), WIRE_R, RING_H * 0.98, BLACK, collide=False)], color=BLACK)
            for k in range(N_RINGS)]


_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: hide the wire's collision-only insulation sleeve, and make the joint by which the closed
    blades hold the cut cable break once the pull on it passes F_PULL."""
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "wire":
        from pxr import UsdGeom, UsdPhysics

        UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/body/s{SLEEVE}")).MakeInvisible()
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"{prim_path}/wire_in_blades"))
        joint.CreateBreakForceAttr(float(F_PULL))
        joint.CreateBreakTorqueAttr(float(BREAK_NM))


bodykit._author_sliders = _author_sliders


def _blade_hold() -> Body:
    """A hidden fixed anchor under the table: once the blades have cut the cable, it holds the cable (a joint that breaks
    at F_PULL) for the closed blades, without switching off the cable's contact with the station."""
    return Body.build("blade_hold", pos=(SX, SY, TOP - 0.05), mass=0.01, kinematic=True,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.002, 0.002, 0.002), DARK, collide=False)])


BREAK_NM = 2.0  # N m: the closed blades also hold the cable against bending (the arm's sideways pushes are far below)
STATION, LEVER, HOLDER, BLADE_HOLD, WIRE = _station(), _lever(), _holder(), _blade_hold(), _wire()
RINGS = _rings()
RING_NAMES = [r.name for r in RINGS]
OBJECTS = [STATION, LEVER, HOLDER, BLADE_HOLD, WIRE, *RINGS]
hinge("lever_pin", "stripper", "lever", anchor=(SX + PIVOT[0], SY + PIVOT[1], TOP + PIVOT[2]), axis="X",
      limits_deg=(-OPEN_DEG, 0.0), spring=SPRING, damping=0.002, target_deg=SPRING_TO)
glue("wire_in_blades", "blade_hold", "wire", anchor=WIRE.pos)  # off until cut
for _r in RINGS:
    glue(f"{_r.name}_on_wire", "wire", _r.name, anchor=_r.pos)
    glue(f"{_r.name}_in_blades", "stripper", _r.name, anchor=_r.pos)  # off until cut


# -- followed after every control step: the cut and the pull ------------------------------------------------------------

def _fresh() -> dict:
    return {"cut": 0, "depth": None, "stripped": False, "best_depth": -1.0, "tcps": None, "lever_deg": -OPEN_DEG,
            "handle": None, "lever_max": -OPEN_DEG, "drive": (SPRING, SPRING_TO), "cut_nm": 0.0, "held_z": None,
            "peak_cut_nm": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRIP_DEBUG")), "steps": 0}
_CUT_SLOPE = np.gradient(_CUT_TABLE, _CUT_DEG)  # N m per degree
TOUCH_DEG = float(_CUT_DEG[np.argmax(_CUT_TABLE > 0.0)]) - (_CUT_DEG[1] - _CUT_DEG[0])  # the blades meet the cable


def _joint(name: str):
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    return UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))


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


def _hold(name: str, b0_pos, b0_rot, r_pos, r_rot) -> None:
    """Switch a glue on with its body1 at r_pos, r_rot (where it is now, or where it is to go), against its body0 as it
    is now. Off first: that also rebuilds a joint that broke."""
    from pxr import Gf

    set_glue_on(name, False)
    j = _joint(name)
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*(b0_rot.T @ (np.asarray(r_pos) - np.asarray(b0_pos))).tolist()))
    j.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(b0_rot.T @ r_rot)))
    j.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    set_glue_on(name, True)


_START_FRAMES: dict = {}


@on_reset
def _new_episode() -> None:
    from pxr import Gf

    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("wire_in_blades", False)
    for k, r in enumerate(RINGS):  # every ring back on the wire as it started, none in the blades
        name = f"{r.name}_on_wire"
        j = _joint(name)
        if name not in _START_FRAMES:
            _START_FRAMES[name] = (j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get(), j.GetLocalPos1Attr().Get(),
                                   j.GetLocalRot1Attr().Get())
        p0, q0, p1, q1 = _START_FRAMES[name]
        j.GetLocalPos0Attr().Set(Gf.Vec3f(p0))
        j.GetLocalRot0Attr().Set(Gf.Quatf(q0))
        j.GetLocalPos1Attr().Set(Gf.Vec3f(p1))
        j.GetLocalRot1Attr().Set(Gf.Quatf(q1))
        set_glue_on(name, False)
        set_glue_on(name, True)
        set_glue_on(f"{r.name}_in_blades", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _lever_deg(l_rot) -> float:
    return math.degrees(math.atan2(l_rot[2, 1], l_rot[1, 1]))


def _depth(w_pos, w_rot) -> tuple[float, float]:
    """(how far the wire's end is below the blades' cutting plane; how far it is off the bore's axis)."""
    end = np.asarray(w_pos, float) - np.array([SX, SY, TOP])
    return float(BLADE_Z - end[2]), math.hypot(end[0], end[1])


def _lever_drive(lever: float, in_blades: bool) -> None:
    """The lever's spring, plus, while the closing blades advance into uncut insulation, the torque the cut takes
    (_CUT_TABLE): a drive linearised about the lever's angle now (USD angular drives: N m per degree)."""
    advancing = in_blades and _EP["cut"] == 0 and lever >= _EP["lever_max"] - 0.3 and lever > TOUCH_DEG
    _EP["lever_max"] = max(_EP["lever_max"], lever) if in_blades and _EP["cut"] == 0 and lever > TOUCH_DEG \
        else -OPEN_DEG
    tq = float(np.interp(lever, _CUT_DEG, _CUT_TABLE)) if advancing else 0.0
    slope = float(np.interp(lever, _CUT_DEG, _CUT_SLOPE)) if advancing else 0.0
    _EP["cut_nm"] = tq
    _EP["peak_cut_nm"] = max(_EP["peak_cut_nm"], tq)
    k = SPRING + max(slope, 0.0)
    target = lever - (SPRING * (lever - SPRING_TO) + tq) / k
    if abs(k - _EP["drive"][0]) > 1e-6 or abs(target - _EP["drive"][1]) > 1e-3:
        set_hinge_drive("lever_pin", target_deg=target, spring=k)
        _EP["drive"] = (k, target)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    w_pos, w_rot = pose("wire")
    l_pos, l_rot = pose("lever")
    s_pos, s_rot = np.array([SX, SY, TOP]), np.eye(3)
    depth, off = _depth(w_pos, w_rot)
    if off < BLADE_OPEN_R:
        _EP["best_depth"] = max(_EP["best_depth"], depth)
    _EP["lever_deg"] = _lever_deg(l_rot)
    _EP["handle"] = l_pos + l_rot @ np.array([0.0, HANDLE_Y, 0.0])
    _EP["tcps"] = _tcps(base, origin)
    in_blades = off < BLADE_OPEN_R and depth > 0.0 and not _EP["stripped"]  # insulation across the blades
    _lever_drive(_EP["lever_deg"], in_blades)
    closed = _EP["lever_deg"] > -CLOSED_WITHIN
    if closed and _EP["cut"] == 0 and not _EP["stripped"] and off < BLADE_OPEN_R and depth > RING_H / 2:
        n = min(N_RINGS, int(round(depth / RING_H)))  # the rings below the blades are cut off: the blades hold them
        _EP["cut"], _EP["depth"] = n, depth
        for r in RINGS[:n]:
            r_pos, r_rot = pose(r.name)
            set_glue_on(f"{r.name}_on_wire", False)
            _hold(f"{r.name}_in_blades", s_pos, s_rot, r_pos, r_rot)
        h_pos, h_rot = pose("blade_hold")  # and the closed blades hold the cable until it is pulled with F_PULL
        _hold("wire_in_blades", h_pos, h_rot, w_pos, w_rot)
        _EP["held_z"] = float(w_pos[2])
    if _EP["cut"] and not _EP["stripped"]:
        if depth < -CLEAR:  # pulled clear while the blades held the cut insulation: stripped
            _EP["stripped"] = True
            set_glue_on("wire_in_blades", False)
            for k, r in enumerate(RINGS[:_EP["cut"]]):  # the slug, off the conductor, drops onto the bore's floor
                _hold(f"{r.name}_in_blades", s_pos, s_rot, np.array([SX, SY, TOP + FLOOR_Z + (k + 0.5) * RING_H]),
                      np.eye(3))
        elif _EP["lever_deg"] < -OPENED:  # let up too soon: the cut insulation stays on the wire, the cable is free
            set_glue_on("wire_in_blades", False)
            for r in RINGS[:_EP["cut"]]:
                r_pos, r_rot = pose(r.name)
                set_glue_on(f"{r.name}_in_blades", False)
                _hold(f"{r.name}_on_wire", w_pos, w_rot, r_pos, r_rot)
            _EP["cut"] = 0
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0 or (_DEBUG["steps"] % 2 == 0 and (_EP["cut_nm"] > 0 or (_EP["cut"] and not _EP["stripped"]))):
            st = _status()
            tl, tr = _EP["tcps"]
            lift = "" if _EP["held_z"] is None else f" lift={(w_pos[2] - _EP['held_z']) * 1000:5.1f}mm"
            print(f"[strip] t={_DEBUG['steps'] / 20:5.2f}s lever={_EP['lever_deg']:6.2f} cutNm={_EP['cut_nm']:4.2f} "
                  f"depth={depth * 1000:6.1f}mm off={off * 1000:4.1f}mm cut={_EP['cut']} stripped={_EP['stripped']}{lift} "
                  f"leftTCP-end={np.round((tl - w_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-grip={np.round((tr - _EP['handle']) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or _EP["handle"] is None or all(float(np.linalg.norm(t - _EP["handle"])) >= LET_GO
                                                           for t in tcps)
    strip = _EP["cut"] * RING_H
    return {"stripped": _EP["stripped"], "good": STRIP[0] - 1e-9 <= strip <= STRIP[1] + 1e-9, "strip": strip,
            "released": _EP["lever_deg"] <= -RELEASED_DEG and let_go, "inserted": _EP["best_depth"] >= INSERTED}


def _points(st) -> float:
    if st["stripped"] and st["good"] and st["released"]:
        return 1.0
    if st["stripped"] or (st["good"] and _EP["cut"]):
        return 0.5
    return 0.25 if st["inserted"] else 0.0


def score(objects) -> float:
    """1.0 with 8 to 12 mm of insulation stripped off (cut by the closed blades and left in the station as the cable was
    pulled out) and the lever let go (no TCP within 4 cm of its grip) and back up at least 15 degrees; 0.5 for any
    strip, or for a cut of the right length with the blades still closed on it; 0.25 if the cable's end went at least
    5 mm below the blades at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the lever and the wire still."""
    return score(objects) >= 1.0 and objects["lever"].resting() and objects["wire"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable's plug, fingers closing across it, lifts it out of its holder,
# carries it over the station and lowers its end through the funnel, 10 mm past the blades. The right hand comes
# straight down on the lever's rubber grip, fingers closed, and presses the lever down hard (the cut takes about 20 N at
# the grip), closing the blades, and holds it there while the left pulls the wire straight up out of the station
# (against the 20 N the closed blades hold it with). Then the right rises and the lever springs back up.

GRIP = grip_rotation(0.0)  # straight down: the left's fingers close along y, across the wire; the right's are closed
HOLD_DOWN = 0.015  # the left TCP this far below the plug's top (the plug's flat sides hold)
DEPTH = 0.0075  # aim 7.5 mm: the end goes in about 2.5 mm further (the cable slips 1 mm in the fingers, the arm sags)
PRESS = 0.06  # aim this far below the closed grip's top: the soft arm presses about 20 N, the lever on its stop
PULL = 0.06  # aim this far above where the end was: the arm pulls past 20 N before it comes free
APPROACH = 0.03  # wait this far above the open lever's grip
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


def _grip_top(deg: float) -> np.ndarray:
    a = math.radians(deg)
    p = np.array([0.0, HANDLE_Y * math.cos(a), HANDLE_Y * math.sin(a)])
    return np.array([SX, SY, TOP]) + PIVOT + p + [0.0, 0.0, (ARM_Z + 0.006) / 2]


HAND_UP = WIRE_L - HOLD_DOWN  # the left TCP above the wire's end
HANG = np.array([-0.011, 0.0, 0.0])  # held by its plug, the cable's end hangs 11 mm in -x from the left TCP (measured)


def _left() -> tuple[list[Waypoint], float, np.ndarray, float]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + HAND_UP])
    over = np.array([SX, SY, TOP + BLOCK[2] + 0.03 + HAND_UP]) - HANG  # its end 3 cm above the funnel's rim
    down = np.array([SX, SY, TOP + BLADE_Z - DEPTH + HAND_UP]) - HANG
    out = _lead_in("left", pick + [0.0, 0.0, 0.03], GRIP, "over the wire", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP),
            _w(2.5, np.array([pick[0], pick[1], over[2]]), FINGER_CLOSED, "lift it straight out of its holder, high", GRIP),
            _w(2.5, over, FINGER_CLOSED, "over the station's hole (slowly: it hangs in the hand)", GRIP),
            _w(0.8, over, FINGER_CLOSED, "let it settle", GRIP),
            _w(2.0, down, FINGER_CLOSED, "its end through the funnel, 10 mm past the blades", GRIP)]
    return out, sum(w.duration for w in out), down, 0.0


def demo():
    left, t_in, down, _ = _left()
    above = _grip_top(-OPEN_DEG) + [0.0, 0.0, APPROACH]
    right = _lead_in("right", above, GRIP, "over the lever's grip", FINGER_CLOSED)
    t = sum(w.duration for w in right)
    press = _grip_top(0.0) - [0.0, 0.0, PRESS]
    right += [_w(max(t_in - t, 0.2) + 0.3, above, FINGER_CLOSED, "wait for the wire to be in the hole", GRIP),
              _w(0.8, _grip_top(-OPEN_DEG) + [0.0, 0.0, 0.005], FINGER_CLOSED, "onto the grip", GRIP),
              _w(1.5, press, FINGER_CLOSED, "press the lever down hard: the blades close and cut", GRIP)]
    t_closed = sum(w.duration for w in right)
    right.append(_w(3.0, press, FINGER_CLOSED, "hold it down while the wire is pulled out", GRIP))
    t_pull = t_closed + 0.6
    right += [_w(1.5, above, FINGER_CLOSED, "rise: the lever springs back up", GRIP)]
    left += [_w(max(t_pull - t_in, 0.2), down, FINGER_CLOSED, "hold it while the blades close", GRIP),
             _w(1.5, down + [0.0, 0.0, PULL], FINGER_CLOSED, "pull it straight up out of the station", GRIP)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    hold = "hold still: the trial ends once the full score has held 1 s"
    left.append(_w(max(t_r - t_l, 0.2) + 3.0, left[-1].pos, FINGER_CLOSED, hold, GRIP))
    t_l = sum(w.duration for w in left)
    right.append(_w(max(t_l - sum(w.duration for w in right), 0.5), above, FINGER_CLOSED, "wait", GRIP))
    return {"left": left, "right": right}
