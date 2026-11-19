"""On the station table stands a terracotta pot with a leafy house plant. Its leaves spread low over the soil, each on
a springy stalk: three in the crown over the back and sides of the pot, and one big leaf in front, lying over the
front of the soil and reaching out past the rim. Beside the pot, on a small wooden stool, stands a blue watering jug
with water in it.

Task: water the plant at its roots, onto the soil, without wetting its leaves; then put the jug back on its stool,
leave the leaves as they were and take both arms home. Water poured from above lands on the leaves, so the scripted
demo pushes the big front leaf aside on its stalk with one hand and holds it there (let go, it springs back over the
soil) while the other pours the jug onto the soil underneath. The 12 water beads stand for 120 ml of water (10 ml
each). Scored 0: water on a leaf, or a leaf disturbed (out of place at the end, or pressed harder than a leaf takes
undamaged). A dry touch that leaves a leaf undamaged and in place is allowed. A real trial weighs the pot and its saucer
separately and reads leaf wetting from two synchronized 60 fps cameras and grazing-light photos (see task.json).

Run it:  inspect-robots run --embodiment yam_arena -E task=water_a_plant_holding_its_big_leaf_aside.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _u(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


# -- the pot (fixed) and the plant: a terracotta pot 17 cm across, the soil 1.3 cm below its rim -----------------------
P = np.array([0.30, -0.02])  # the pot's middle
POT_R, POT_WALL, POT_H = 0.085, 0.009, 0.075
SOIL_Z = TOP + 0.062
SOIL_R = POT_R - POT_WALL  # 7.6 cm
STEM_R, STEM_TOP = 0.008, TOP + 0.140
LEAF_T = 0.003
# a leaf's outline, out from where it joins the stem: (from, to, width) in m
OUTLINE = ((0.015, 0.030, 0.070), (0.030, 0.050, 0.104), (0.050, 0.070, 0.127), (0.070, 0.090, 0.129),
           (0.090, 0.110, 0.123), (0.110, 0.130, 0.104), (0.130, 0.150, 0.065))
# the crown's leaves over the back and sides of the pot: (pointing, height above the table); each turns on its stalk
# about the vertical against a spring, and springs back when let go
CROWN = ((0.0, 0.118), (60.0, 0.112), (-78.0, 0.124))
CROWN_SPRING = 0.004  # N m per degree (design value: shorter, stiffer stalks than the big leaf's)
CROWN_LIMIT = 30.0  # degrees either way
LEAF_LEN = OUTLINE[-1][1]  # 15 cm from the stem to the tip

# -- the big front leaf on its springy stalk: it joins the stem 10 cm above the table and lies pointing at the arms ---
HINGE_Z = TOP + 0.100
LEAF_REST = 180.0  # pointing straight back toward the arms (-x)
LEAF_SPRING = 0.0016  # N m per degree: pushed 85 degrees aside, it pushes back with 0.14 N m
LEAF_MASS = 0.02

# -- the watering jug and its water, on a stool beside the pot ---------------------------------------------------------
JUG_AT = np.array([0.262, 0.130])  # 1.9 cm clear of the left crown leaf's edge
STOOL_H = 0.060
CUP_R_I, CUP_WALL, CUP_H, CUP_FLOOR = 0.024, 0.003, 0.085, 0.006
JUG_MASS = 0.07
BEAD_R, BEAD_MASS, N_BEADS = 0.0035, 0.0008, 12

TERRACOTTA, RIM_C, SOIL, STEM, LEAF_C, LEAF_DARK, VEIN, JUG_C, WATER, STOOL_C = \
    (0.72, 0.38, 0.24), (0.66, 0.33, 0.21), (0.24, 0.16, 0.10), (0.30, 0.50, 0.22), (0.22, 0.55, 0.20), \
    (0.15, 0.42, 0.16), (0.55, 0.75, 0.40), (0.20, 0.50, 0.85), (0.55, 0.80, 1.00), (0.72, 0.55, 0.36)

# -- scoring ----------------------------------------------------------------------------------------------------------
WET = 0.004  # a bead, or the jug's wet lip and rim, within this of a leaf has wetted it
WET_RIM = 0.010  # once water has left the jug, its lip and the top 1 cm of its rim are wet
SOAKED_FULL, SOAKED_SOME = 9, 4  # beads on the soil (10 ml each): 90 ml for the higher scores, 40 ml for 0.25
TIP_WITHIN = 0.03  # a leaf is in place when its tip is within 3 cm of where it lay (the big leaf: an 11.5 degree turn)
DAMAGE_N = 5.0  # a contact force above this on a leaf (from the jug or any part of either arm), averaged over a 50 ms
# control step (12 physics steps, so a single-step solver spike does not count), damages it: design value
BACK_WITHIN, UPRIGHT = 0.03, 15.0
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _chord(r: float, n: int) -> float:
    return 2 * r * math.tan(math.pi / n) + 0.001


def _leaf_shapes(origin, deg: float, z: float, colour, dark) -> list:
    """A leaf's blade (boxes along ``deg`` from ``origin``, at height ``z`` in the body frame) and its midrib."""
    out = []
    u = _u(deg)[:2]
    for k, (r0, r1, w) in enumerate(OUTLINE):
        c = np.asarray(origin, float) + u * (r0 + r1) / 2
        out.append(BoxShape((float(c[0]), float(c[1]), z), (r1 - r0 + 0.001, w, LEAF_T), colour,
                            yaw=math.radians(deg)))
    c = np.asarray(origin, float) + u * (OUTLINE[0][0] + OUTLINE[-1][1]) / 2
    out.append(BoxShape((float(c[0]), float(c[1]), z + LEAF_T / 2 + 0.0003),
                        (OUTLINE[-1][1] - OUTLINE[0][0] - 0.01, 0.004, 0.0005), VEIN, yaw=math.radians(deg),
                        collide=False))
    return out


def _pot() -> Body:
    """The pot, its soil and the stem: one fixed body, origin at the pot's middle, on the table."""
    r = POT_R - POT_WALL / 2
    shapes = [CylShape((0.0, 0.0, (SOIL_Z - TOP) / 2), SOIL_R + 0.001, SOIL_Z - TOP, SOIL),  # soil, up to SOIL_Z
              CylShape((0.0, 0.0, 0.004), POT_R + 0.012, 0.008, RIM_C, collide=False)]  # its saucer (seen only)
    for k in range(20):
        a = 2 * math.pi * k / 20
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r * u[0], r * u[1], 0.0), (r * u[0], r * u[1], POT_H), _chord(r, 20), POT_WALL, u,
                            TERRACOTTA))
    shapes.append(CylShape((0.0, 0.0, POT_H - 0.006), POT_R + 0.004, 0.012, RIM_C, collide=False))  # the rim band
    shapes.append(CylShape((0.0, 0.0, (STEM_TOP + SOIL_Z) / 2 - TOP), STEM_R, STEM_TOP - SOIL_Z, STEM))
    for deg, h, tall in ((25.0, 0.13, 0.07), (-35.0, 0.13, 0.06), (110.0, 0.12, 0.05)):  # upright young leaves (seen)
        at = _u(deg) * 0.02
        shapes.append(BoxShape((float(at[0]), float(at[1]), h - 0.0 + tall / 2 - (TOP - TOP)),
                               (0.004, 0.035, tall), LEAF_DARK, yaw=math.radians(deg), collide=False))
    for k in range(9):  # bits of bark on the soil (seen only)
        a = 2.4 * k
        rr = 0.02 + 0.05 * ((k * 37) % 9) / 9
        shapes.append(BoxShape((rr * math.cos(a), rr * math.sin(a), SOIL_Z - TOP + 0.0006), (0.012, 0.008, 0.0012),
                               (0.40, 0.27, 0.16), yaw=a, collide=False))
    return Body.build("plant_pot", pos=(float(P[0]), float(P[1]), TOP), mass=3.0, shapes=shapes, kinematic=True,
                      color=TERRACOTTA, friction=(1.2, 1.1))


def _crown_leaf(k: int) -> Body:
    """One of the crown's leaves: its body origin on the stem where its stalk joins; it lies pointing CROWN[k][0]."""
    deg, h = CROWN[k]
    c = _u(deg) * 0.08
    return Body.build(f"crown_leaf_{k}", pos=(float(P[0]), float(P[1]), TOP + h), mass=LEAF_MASS,
                      shapes=_leaf_shapes((0.0, 0.0), deg, 0.0, LEAF_C, LEAF_DARK), color=LEAF_C, friction=(0.8, 0.7),
                      damping=(0.5, 3.0), com=(float(c[0]), float(c[1]), 0.0), report_contacts=True)


def _leaf() -> Body:
    """The big front leaf: its body origin on the stem where its stalk joins, at HINGE_Z; it lies pointing LEAF_REST."""
    shapes = _leaf_shapes((0.0, 0.0), LEAF_REST, 0.0, LEAF_C, LEAF_DARK)
    shapes.append(CylShape((float(_u(LEAF_REST)[0]) * 0.008, 0.0, 0.0), 0.004, 0.016, STEM, axis="X"))  # its stalk
    c = _u(LEAF_REST) * 0.08
    return Body.build("big_leaf", pos=(float(P[0]), float(P[1]), HINGE_Z), mass=LEAF_MASS, shapes=shapes,
                      color=LEAF_C, friction=(0.8, 0.7), damping=(0.5, 3.0), com=(float(c[0]), float(c[1]), 0.0),
                      report_contacts=True)


def _jug() -> Body:
    """The watering jug: a cup with a pouring lip toward +x and a handle on its back (seen only); origin at the middle
    of its floor's underside."""
    r = CUP_R_I + CUP_WALL / 2
    shapes = [CylShape((0.0, 0.0, CUP_FLOOR / 2), CUP_R_I + CUP_WALL, CUP_FLOOR, JUG_C)]
    for k in range(16):
        a = 2 * math.pi * k / 16
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r * u[0], r * u[1], CUP_FLOOR), (r * u[0], r * u[1], CUP_H), _chord(r, 16), CUP_WALL, u,
                            JUG_C))
    shapes.append(BoxShape((CUP_R_I + 0.008, 0.0, CUP_H - 0.004), (0.018, 0.020, 0.003), JUG_C, collide=False))  # lip
    shapes.append(BoxShape((-(CUP_R_I + 0.016), 0.0, CUP_H * 0.55), (0.006, 0.012, 0.050), JUG_C, collide=False))
    for zz in (CUP_H * 0.85, CUP_H * 0.25):  # the handle's arms
        shapes.append(BoxShape((-(CUP_R_I + 0.010), 0.0, zz), (0.014, 0.012, 0.006), JUG_C, collide=False))
    return Body.build("jug", pos=(float(JUG_AT[0]), float(JUG_AT[1]), TOP + STOOL_H + 0.0005), mass=JUG_MASS,
                      shapes=shapes, friction=(1.5, 1.4), color=JUG_C, com=(0.0, 0.0, 0.022), report_contacts=True)


def _bead_spots() -> list[tuple[float, float, float]]:
    d = 2 * BEAD_R + 0.0005
    rmax = CUP_R_I - BEAD_R - 0.0015
    spots = []
    for layer in range(6):
        z = CUP_FLOOR + BEAD_R + 0.0005 + layer * d * math.sqrt(2 / 3)
        ox, oy = (d / 2, d / (2 * math.sqrt(3))) if layer % 2 else (0.0, 0.0)
        pts = [(ox + (i + 0.5 * (j % 2)) * d, oy + j * d * math.sqrt(3) / 2, z) for j in range(-6, 7) for i in range(-6, 7)]
        pts = sorted([p for p in pts if math.hypot(p[0], p[1]) <= rmax],
                     key=lambda p: (round(math.hypot(p[0], p[1]), 6), p))
        spots += pts
        if len(spots) >= N_BEADS:
            break
    return spots[:N_BEADS]


def _beads() -> list[Body]:
    cx, cy = JUG_AT
    return [Body.build(f"water_{i:02d}", pos=(float(cx) + x, float(cy) + y, TOP + STOOL_H + 0.0005 + z), mass=BEAD_MASS,
                       color=WATER, friction=(0.6, 0.5), shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, WATER)],
                       damping=(0.5, 6.0)) for i, (x, y, z) in enumerate(_bead_spots())]


def _stool() -> Body:
    shapes = [BoxShape((0.0, 0.0, STOOL_H - 0.006), (0.085, 0.085, 0.012), STOOL_C)]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shapes.append(BoxShape((sx * 0.033, sy * 0.033, (STOOL_H - 0.012) / 2), (0.012, 0.012, STOOL_H - 0.012),
                                   STOOL_C))
    return Body.build("stool", pos=(float(JUG_AT[0]), float(JUG_AT[1]), TOP), mass=0.3, kinematic=True, color=STOOL_C,
                      friction=(0.7, 0.6), shapes=shapes)


BEADS = _beads()
BEAD_NAMES = [b.name for b in BEADS]
JUG_START = np.array([float(JUG_AT[0]), float(JUG_AT[1]), TOP + STOOL_H])
CROWN_NAMES = [f"crown_leaf_{k}" for k in range(len(CROWN))]
LEAF_NAMES = (*CROWN_NAMES, "big_leaf")  # every leaf: the crown's (back, left, right), then the big one
LEAF_DEG = (*(d for d, _ in CROWN), LEAF_REST)  # where each points as it lies
OBJECTS = [_pot(), _stool(), *(_crown_leaf(k) for k in range(len(CROWN))), _leaf(), _jug(), *BEADS]
# the big leaf's stalk: turns round the stem (about the vertical), sprung back to where it lies; pushed aside it goes
# round toward the right (counter-clockwise seen from above)
hinge(name="leaf_stalk", body0="plant_pot", body1="big_leaf", anchor=(float(P[0]), float(P[1]), HINGE_Z), axis="Z",
      limits_deg=(-8.0, 100.0), spring=LEAF_SPRING, damping=LEAF_SPRING * 0.05, target_deg=0.0)
for _k, (_, _h) in enumerate(CROWN):  # the crown's stalks: the same kind of sprung turn, stiffer
    hinge(name=f"crown_stalk_{_k}", body0="plant_pot", body1=f"crown_leaf_{_k}",
          anchor=(float(P[0]), float(P[1]), TOP + _h), axis="Z", limits_deg=(-CROWN_LIMIT, CROWN_LIMIT),
          spring=CROWN_SPRING, damping=CROWN_SPRING * 0.05, target_deg=0.0)


# -- followed after every physics step and every control step ---------------------------------------------------------

def _fresh() -> dict:
    return {"wet": set(), "soil": set(), "tcps": None, "s": {}, "substeps": 0, "poured": False, "lip_wet": False,
            "force": [0.0] * len(LEAF_NAMES), "force_by": [""] * len(LEAF_NAMES), "f_sum": None, "f_n": 0,
            "peak": 0.0, "jug_gap": 1.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PLANT_DEBUG")), "steps": 0}
_WATCH: dict = {}
_CT: dict = {"view": None, "failed": "", "names": []}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return t[:3] - origin, rotation(t[3:7])


def _on_leaf(local) -> bool:
    """Whether a bead's centre (in a leaf's own frame: out along +x from where it joins the stem, its blade at z 0) is
    within WET of the blade."""
    x, y, z = (float(v) for v in local)
    if abs(z) > LEAF_T / 2 + BEAD_R + WET:
        return False
    return any(r0 - BEAD_R - WET <= x <= r1 + BEAD_R + WET and abs(y) <= w / 2 + BEAD_R + WET for r0, r1, w in OUTLINE)


def _box_grid(centre, size, step: float = 0.004) -> list[tuple[float, float, float]]:
    axes = [np.linspace(c - d / 2, c + d / 2, max(2, int(math.ceil(d / step)) + 1)) for c, d in zip(centre, size)]
    return [(float(x), float(y), float(z)) for x in axes[0] for y in axes[1] for z in axes[2]]


def _jug_points() -> np.ndarray:
    """Points over the whole jug as drawn (its body, rim, floor, pouring lip and handle), in its own frame, at most 4 mm
    apart (for the debug read-out of how close the jug comes to a leaf)."""
    ro = CUP_R_I + CUP_WALL
    pts = [(0.0, 0.0, 0.0)]
    for k in range(48):
        a = 2 * math.pi * k / 48
        c, s = math.cos(a), math.sin(a)
        pts += [(ro * c, ro * s, float(z)) for z in np.linspace(0.0, CUP_H, 23)]
        pts += [(CUP_R_I * c, CUP_R_I * s, CUP_H), (0.009 * c, 0.009 * s, 0.0), (0.018 * c, 0.018 * s, 0.0)]
    pts += _box_grid((CUP_R_I + 0.008, 0.0, CUP_H - 0.004), (0.018, 0.020, 0.003))  # the pouring lip
    pts += _box_grid((-(CUP_R_I + 0.016), 0.0, CUP_H * 0.55), (0.006, 0.012, 0.050))  # the handle
    for zz in (CUP_H * 0.85, CUP_H * 0.25):
        pts += _box_grid((-(CUP_R_I + 0.010), 0.0, zz), (0.014, 0.012, 0.006))
    return np.array(pts)


def _wet_points() -> np.ndarray:
    """The jug's wet part once water has left it: its pouring lip and the top WET_RIM of its rim, inside and out."""
    pts = []
    for k in range(48):
        a = 2 * math.pi * k / 48
        c, s = math.cos(a), math.sin(a)
        for r in (CUP_R_I, CUP_R_I + CUP_WALL):
            pts += [(r * c, r * s, float(z)) for z in np.linspace(CUP_H - WET_RIM, CUP_H, 4)]
    pts += _box_grid((CUP_R_I + 0.008, 0.0, CUP_H - 0.004), (0.018, 0.020, 0.003))
    return np.array(pts)


JUG_POINTS = _jug_points()
WET_POINTS = _wet_points()
_OUTLINE = np.array(OUTLINE)


def _leaf_gap(local: np.ndarray) -> float:
    """The smallest distance from any of these points (rows, in a leaf's own frame) to that leaf's blade."""
    x, y, z = local[:, 0:1], local[:, 1:2], local[:, 2]
    dx = np.maximum(np.maximum(_OUTLINE[None, :, 0] - x, x - _OUTLINE[None, :, 1]), 0.0)
    dy = np.maximum(np.abs(y) - _OUTLINE[None, :, 2] / 2, 0.0)
    dxy = np.sqrt(dx ** 2 + dy ** 2).min(axis=1)
    dz = np.maximum(np.abs(z) - LEAF_T / 2, 0.0)
    return float(np.sqrt(dxy ** 2 + dz ** 2).min())


def _points_gap(points, j_pos, j_rot, frames) -> float:
    """How close any of these points of the jug (its own frame) come to any leaf's blade (m)."""
    world = j_pos + points @ np.asarray(j_rot).T
    return min(_leaf_gap((world - o) @ r) for o, r in frames)


def _leaf_frames(leaf_poses) -> list[tuple[np.ndarray, np.ndarray]]:
    """Every leaf's frame (origin, rotation whose x points out along the leaf), from each leaf body's pose, in
    LEAF_NAMES order."""
    return [(p, r @ np.array([_u(d), _u(d + 90.0), [0.0, 0.0, 1.0]]).T) for (p, r), d in zip(leaf_poses, LEAF_DEG)]


def _leaf_turn(l_rot) -> float:
    """How far round a leaf is from where it lies (degrees, counter-clockwise seen from above)."""
    return math.degrees(math.atan2(float(l_rot[1, 0]), float(l_rot[0, 0])))


def _rest_tips() -> list[np.ndarray]:
    out = [np.array([float(P[0]), float(P[1]), TOP + h]) + _u(d) * LEAF_LEN for d, h in CROWN]
    out.append(np.array([float(P[0]), float(P[1]), HINGE_Z]) + _u(LEAF_REST) * LEAF_LEN)
    return out


REST_TIPS = _rest_tips()


def _tip_moves(frames) -> list[float]:
    """How far each leaf's tip is from where it lay (m)."""
    return [float(np.linalg.norm(o + r @ np.array([LEAF_LEN, 0.0, 0.0]) - t)) for (o, r), t in zip(frames, REST_TIPS)]


def _on_soil(p) -> bool:
    """Whether a bead lies on the soil inside the pot: its centre within the wall's inner face (a bead against the wall
    counts; one outside the pot is at least 8.85 cm out) and between 3 mm below and 15 mm above the soil's surface (a
    bead on the rim is 16.5 mm up)."""
    return float(np.linalg.norm(np.asarray(p[:2]) - P)) <= SOIL_R and SOIL_Z - 0.003 <= float(p[2]) <= SOIL_Z + 0.015


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _contact_view():
    """A PhysX rigid-contact view, made once: each leaf against the jug and against every link of both arms."""
    if _CT["view"] is None and not _CT["failed"]:
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            env = "/World/envs/env_*"
            links = asset_link_paths()
            arms = [f"{env}/{arm}{p}" for arm in ("LeftArm", "RightArm") for p in links.values()]
            others = [f"{env}/jug/body"] + arms
            _CT["names"] = ["jug"] + [f"{arm[:-3].lower()} {n}" for arm in ("LeftArm", "RightArm") for n in links]
            _CT["view"] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                [f"{env}/{n}/body" for n in LEAF_NAMES], filter_patterns=[others] * len(LEAF_NAMES),
                max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
            print(f"[plant] leaf contact-force view failed: {_CT['failed']}", file=sys.stderr, flush=True)
    return _CT["view"]


def _leaf_forces() -> np.ndarray | None:
    """(leaves, partners): the size of the contact force (N) each leaf takes from the jug and from each link of either
    arm over the last physics step (each finger separately, so a pinch is not cancelled out)."""
    view = _contact_view()
    if view is None:
        return None
    from yam_arena.constants import PHYSICS_DT

    m = _np(view.get_contact_force_matrix(dt=PHYSICS_DT)).reshape(len(LEAF_NAMES), len(_CT["names"]), 3)
    return np.linalg.norm(m, axis=2)


def _watch_leaves(base) -> None:
    """After every physics step (240 a second, not only after each 50 ms control step): every bead that comes within
    WET of any leaf's blade has wetted a leaf, even if it runs off straight away; once water has left the jug, its lip
    and rim within WET of a leaf's blade have wetted it; and the largest contact force each leaf has taken from the jug
    or either arm is added up, to be averaged over the control step."""
    import torch

    names = (*LEAF_NAMES, "jug", *BEAD_NAMES)
    poses = torch.cat([getattr(base.scene[n].data.root_link_pose_w, "torch", base.scene[n].data.root_link_pose_w)[:1]
                       for n in names]).detach().cpu().numpy().astype(np.float64)
    origin = _WATCH["origin"]
    nl = len(LEAF_NAMES)
    frames = _leaf_frames([(poses[i, :3] - origin, rotation(poses[i, 3:7])) for i in range(nl)])
    _EP["substeps"] += 1
    jp, jr = poses[nl, :3] - origin, rotation(poses[nl, 3:7])
    for name, row in zip(BEAD_NAMES, poses[nl + 1:]):
        p = row[:3] - origin
        if not _EP["poured"]:
            q = jr.T @ (p - jp)  # in the jug's frame: still inside it?
            if math.hypot(float(q[0]), float(q[1])) > CUP_R_I + 0.002 or not -0.002 <= float(q[2]) <= CUP_H + 0.002:
                _EP["poured"] = True
        if name in _EP["wet"] or name in _EP["soil"]:
            continue
        if any(_on_leaf(r.T @ (p - o)) for o, r in frames):
            _EP["wet"].add(name)
    if _EP["poured"] and not _EP["lip_wet"] and _points_gap(WET_POINTS, jp, jr, frames) <= WET:
        _EP["lip_wet"] = True
        if _DEBUG["on"]:
            print(f"[plant] wet lip touches a leaf at {_EP['substeps'] / 240:.2f} s", file=sys.stderr, flush=True)
    if _DEBUG["on"]:
        _EP["jug_gap"] = min(_EP["jug_gap"], _points_gap(JUG_POINTS, jp, jr, frames))
        if _EP["poured"]:
            _EP["lip_gap"] = min(_EP.get("lip_gap", 1.0), _points_gap(WET_POINTS, jp, jr, frames))
    f = _leaf_forces()
    if f is not None:
        _EP["f_sum"] = f if _EP["f_sum"] is None else _EP["f_sum"] + f
        _EP["f_n"] += 1
        _EP["peak"] = max(_EP["peak"], float(f.max()))  # a single physics step's largest (debug only)


def _press() -> None:
    """After each control step: each leaf's contact force from each partner, averaged over the step's physics steps;
    the largest so far per leaf is kept."""
    if not _EP["f_n"]:
        return
    mean = _EP["f_sum"] / _EP["f_n"]
    _EP["f_sum"], _EP["f_n"] = None, 0
    for i in range(len(LEAF_NAMES)):
        j = int(np.argmax(mean[i]))
        if mean[i, j] > _EP["force"][i]:
            _EP["force"][i], _EP["force_by"][i] = float(mean[i, j]), _CT["names"][j]


def _install_watch(base) -> None:
    """Run _watch_leaves after every physics step: Isaac Lab calls scene.update(dt=physics_dt) after each one."""
    scene = base.scene
    if getattr(scene, "_plant_watch", False):
        return
    plain = scene.update
    _WATCH["origin"] = as_numpy(scene.env_origins[0])

    def update(*args, **kwargs):
        plain(*args, **kwargs)
        _watch_leaves(base)

    scene.update = update
    scene._plant_watch = True


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    _install_watch(base)
    _press()
    origin = as_numpy(base.scene.env_origins[0])
    leaf_poses = [_pose(base, origin, n) for n in LEAF_NAMES]
    frames = _leaf_frames(leaf_poses)
    beads = []
    for name in BEAD_NAMES:
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0][:3] - origin
        beads.append(p)
        if name not in _EP["soil"] and any(_on_leaf(r.T @ (p - o)) for o, r in frames):
            _EP["wet"].add(name)
        if name not in _EP["wet"] and _on_soil(p):
            _EP["soil"].add(name)
    soaked = sum(1 for n, p in zip(BEAD_NAMES, beads) if n in _EP["soil"] and _on_soil(p))
    tips = _tip_moves(frames)
    in_place = all(t <= TIP_WITHIN for t in tips)
    pressed = max(_EP["force"]) > DAMAGE_N
    l_pos, l_rot = leaf_poses[-1]
    j_pos, j_rot = _pose(base, origin, "jug")
    jug_back = (float(np.linalg.norm(j_pos[:2] - JUG_START[:2])) <= BACK_WITHIN
                and math.degrees(math.acos(max(-1.0, min(1.0, float(j_rot[2, 2]))))) <= UPRIGHT
                and float(j_pos[2]) < JUG_START[2] + 0.01)
    _EP["tcps"] = _tcps(base, origin)
    jug_mid = j_pos + j_rot @ np.array([0.0, 0.0, CUP_H / 2])
    leaf_mid = l_pos + l_rot @ (_u(LEAF_REST) * 0.10)
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (jug_mid, leaf_mid))
    _EP["s"] = {"soaked": soaked, "wet": len(_EP["wet"]), "lip_wet": _EP["lip_wet"], "in_place": in_place,
                "pressed": pressed, "jug_back": jug_back, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            in_jug = sum(1 for p in beads if float(np.linalg.norm(p[:2] - j_pos[:2])) < CUP_R_I + 0.012
                         and p[2] > j_pos[2] - 0.01)
            turns = [round(_leaf_turn(r), 1) for _, r in leaf_poses]
            off = [np.round((p - [P[0], P[1], SOIL_Z]) * 1000).astype(int).tolist()
                   for n, p in zip(BEAD_NAMES, beads) if _EP["poured"] and not (n in _EP["soil"] and _on_soil(p))]
            force = " ".join(f"{v:.2f}({b})" for v, b in zip(_EP["force"], _EP["force_by"]))
            print(f"[plant] t={_DEBUG['steps'] / 20:5.1f}s turns={turns} tips_mm={[round(t * 1000) for t in tips]} "
                  f"in_place={in_place} force_max_N=[{force}] step_peak_N={_EP['peak']:.2f} pressed={pressed} "
                  f"soaked={soaked} "
                  f"wet={len(_EP['wet'])} poured={_EP['poured']} lip_wet={_EP['lip_wet']} substeps={_EP['substeps']} "
                  f"jug_gap_min_mm={_EP['jug_gap'] * 1000:.1f} lip_gap_min_mm={_EP.get('lip_gap', 1.0) * 1000:.1f} "
                  f"in_jug={in_jug} off_soil={off} "
                  f"jug={np.round(j_pos * 1000).astype(int).tolist()} jug_back={jug_back} "
                  f"jug_tilt={math.degrees(math.acos(max(-1.0, min(1.0, float(j_rot[2, 2]))))):5.1f} "
                  f"clear={clear} L={np.round(tl * 1000).astype(int).tolist()} "
                  f"R={np.round(tr * 1000).astype(int).tolist()} view={_CT['failed'] or 'ok'} "
                  f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

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


def _status() -> dict:
    s = _EP["s"]
    return {"soaked": int(s.get("soaked", 0)), "wet": int(s.get("wet", 0)), "lip_wet": bool(s.get("lip_wet")),
            "in_place": bool(s.get("in_place")), "pressed": bool(s.get("pressed")),
            "jug_back": bool(s.get("jug_back")), "clear": bool(s.get("clear")), "home": _arms_home()}


def _points(s) -> float:
    if s["wet"] or s["lip_wet"]:  # water on a leaf: the instruction's constraint is broken
        return 0.0
    if not s["in_place"] or s["pressed"]:  # a leaf out of place at the end, or pressed harder than it takes undamaged
        return 0.0
    if s["soaked"] >= SOAKED_FULL and s["jug_back"] and s["clear"] and s["home"]:
        return 1.0
    if s["soaked"] >= SOAKED_FULL:
        return 0.5
    return 0.25 if s["soaked"] >= SOAKED_SOME else 0.0


def score(objects) -> float:
    """Water on a leaf: a bead ever within 4 mm of a leaf's blade, or, once water has left the jug, the jug's lip or the
    top 1 cm of its rim ever within 4 mm of one (both checked after every physics step). A leaf disturbed: at the
    end, any leaf's tip more than 3 cm from where it lay, or, at any time, a contact force above 5 N (averaged over
    a 50 ms control step) on a leaf from the jug or any part of either arm. A dry touch of the jug or a hand on a
    leaf is allowed. A bead waters the soil if it ends on the soil inside the pot's wall without having touched a
    leaf (the saucer is drawn only, so a bead outside the pot never counts). Judged at the end: 0.0 for water on a
    leaf or a leaf disturbed. Otherwise 1.0 with at least 9 of the 12 beads (90 of 120 ml) on the soil, the jug back
    on its stool (within 3 cm, upright within 15 degrees), both hands let go (each TCP at least 4 cm from the jug
    and the big leaf) and both arms back where they started (each TCP within 5 cm); 0.5 with at least 9 beads on the
    soil; 0.25 with at least 4 beads (40 ml) on the soil; else 0.0. (Each bead stands for 10 ml of water.)"""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the jug, every leaf and every bead still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("jug", *LEAF_NAMES, *BEAD_NAMES))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down beside the big leaf's left edge, fingers closed, and pushes the leaf round on its stalk,
# 85 degrees, until it points to the right, clear of the front of the soil, and holds it there. The left comes in from
# the side on the jug, fingers closing across it low down, lifts it, carries it over the front of the pot and tips it
# out onto the soil the leaf covered. It sets the jug back on its stool, lets go and goes back to where it started. The
# right lets the leaf go back round onto the soil, rises and goes back to where it started.

PUSH_R = 0.100  # where along the leaf the right hand pushes it, from the stem
PUSH_OFF = 0.0615 + 0.013  # the leaf's half width there plus half the closed fingers
PUSH_Z = HINGE_Z - 0.002
PUSH_TO = 85.0
PUSH_STEP, PUSH_S = 5.0, 0.30
JUG_GRIP = _rx(-90.0) @ grip_rotation(-math.pi / 2)  # left: from the side (pointing -y), fingers closing across (x)
PUSH_ROT = grip_rotation(0.0)  # right: straight down, fingers closed
LEAD_S, LEAD_STEPS = 3.0, 15
GRIP_FROM_AXIS = np.array([0.0, 0.0, 0.044])  # low on the jug's body, below the rim
HELD_AT = np.array([0.023, 0.0, 0.042])  # where the TCP ends up in the jug's frame once it has taken the jug
MOUTH_AT = np.array([P[0] - 0.066, P[1] + 0.022, SOIL_Z + 0.075])  # the lip, over the front left of the soil
POUR_DEG, SWEEP = 115.0, 0.006
START_OFF = np.array([-0.06, 0.0, 0.04])  # the jug comes in upright from behind and above where it will pour


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(secs / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _push_at(turn: float, z: float = PUSH_Z) -> np.ndarray:
    """Where the right TCP pushes the leaf with it turned ``turn`` degrees round from where it lies."""
    d = LEAF_REST + turn
    u = _u(d)
    n = np.array([math.sin(math.radians(d)), -math.cos(math.radians(d)), 0.0])  # the leaf's trailing side
    p = np.array([float(P[0]), float(P[1]), 0.0]) + u * PUSH_R + n * PUSH_OFF
    return np.array([p[0], p[1], z])


def _jug_tcp(deg: float, mouth) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP (and its rotation) with the jug tipped ``deg`` about y (lip toward +x) and its lip at ``mouth``."""
    r = _ry(deg)
    return np.asarray(mouth, float) - r @ (np.array([0.0, 0.0, CUP_H]) - HELD_AT), r @ JUG_GRIP


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    grip = JUG_START + GRIP_FROM_AXIS
    back = grip + [0.0, 0.0, 0.11]
    out = _lead_in("left", back, JUG_GRIP, "above the watering jug, hand sideways", FINGER_OPEN)
    out += [_w(0.6, back, FINGER_OPEN, "above the watering jug, hand sideways", JUG_GRIP),
            _w(1.0, grip + [0.0, 0.0, 0.07], FINGER_OPEN, "down over it", JUG_GRIP),
            _w(1.2, grip, FINGER_OPEN, "down, fingers either side of its body, low down", JUG_GRIP),
            _w(0.5, grip, FINGER_OPEN, "down, fingers either side of its body, low down", JUG_GRIP),
            _w(0.5, grip, -0.024, "close on it", JUG_GRIP),
            _w(0.6, grip, FINGER_CLOSED, "take it", JUG_GRIP),
            _w(1.0, grip + [0.0, 0.0, 0.10], FINGER_CLOSED, "lift it", JUG_GRIP)]
    upright, rot0 = _jug_tcp(0.0, MOUTH_AT + START_OFF)
    t = sum(w.duration for w in out)
    out += [_w(max(start - t, 2.0), upright, FINGER_CLOSED, "over beside the front of the pot, upright", rot0)]
    for k in range(1, 15):  # tip it in small steps, the lip moving to its place over the soil
        deg = POUR_DEG * k / 14
        p, r = _jug_tcp(deg, MOUTH_AT + START_OFF * max(0.0, 1.0 - deg / 90.0))
        out.append(_w(0.3, p, FINGER_CLOSED, "tip it out onto the soil the leaf covered", r))
    for dx, secs in ((SWEEP, 0.8), (-SWEEP, 1.2), (0.0, 1.5)):
        p, r = _jug_tcp(POUR_DEG, MOUTH_AT + [dx, 0.0, 0.0])
        out.append(_w(secs, p, FINGER_CLOSED, "let it all run out", r))
    poured = sum(w.duration for w in out)
    for k in range(13, -1, -1):  # tip it back up the same way
        deg = POUR_DEG * k / 14
        p, r = _jug_tcp(deg, MOUTH_AT + START_OFF * max(0.0, 1.0 - deg / 90.0))
        out.append(_w(0.2, p, FINGER_CLOSED, "tip it back up, away from the plant", r))
    out += [_w(1.5, grip + [0.0, 0.0, 0.10], FINGER_CLOSED, "back over its stool", JUG_GRIP),
            _w(1.5, grip + [0.0, 0.0, 0.004], FINGER_CLOSED, "lower it onto the stool", JUG_GRIP),
            _w(0.8, grip - [0.0, 0.0, 0.004], FINGER_CLOSED, "set it down", JUG_GRIP),
            _w(0.6, grip - [0.0, 0.0, 0.004], FINGER_OPEN, "let go", JUG_GRIP),
            _w(1.4, back, FINGER_OPEN, "rise, fingers wide open", JUG_GRIP)]
    out += _go_home("left", back, JUG_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, poured


def _right(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    start = _push_at(0.0)
    out = _lead_in("right", start + [0.0, 0.0, 0.06], PUSH_ROT, "over the big leaf's left edge", FINGER_CLOSED)
    out += [_w(1.2, start, FINGER_CLOSED, "down beside the leaf's edge, fingers closed", PUSH_ROT)]
    turn = 0.0
    while turn < PUSH_TO - 1e-6:
        turn = min(turn + PUSH_STEP, PUSH_TO)
        out.append(_w(PUSH_S, _push_at(turn), FINGER_CLOSED, "push the leaf round on its stalk, aside", PUSH_ROT))
    aside = sum(w.duration for w in out)
    out.append(_w(max(release_at - aside, 0.5), _push_at(PUSH_TO), FINGER_CLOSED,
                  "hold the leaf aside while the water goes on", PUSH_ROT))
    while turn > 10.0 + 1e-6:  # let it go back round slowly
        turn = max(turn - 2 * PUSH_STEP, 10.0)
        out.append(_w(PUSH_S, _push_at(turn), FINGER_CLOSED, "let the leaf go back round over the soil", PUSH_ROT))
    out += [_w(0.8, _push_at(turn, PUSH_Z + 0.06), FINGER_CLOSED, "rise", PUSH_ROT)]
    out += _go_home("right", out[-1].pos, PUSH_ROT, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, aside


def demo():
    _, aside = _right(0.0, 0.0)
    left, poured = _left(aside + 0.5, 0.0)
    right, _ = _right(poured + 4.0, 0.0)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    left, _ = _left(aside + 0.5, total)
    right, _ = _right(poured + 4.0, total)
    return {"left": left, "right": right}
