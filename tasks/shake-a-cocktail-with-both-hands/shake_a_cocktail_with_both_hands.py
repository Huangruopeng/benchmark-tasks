"""On the station table, a two-piece Boston shaker stands upright in front of the arms on the centre line: a large
28 oz stainless steel tin holding the drink (24 beads, each standing for 5 ml of a 120 ml drink) and four 2.5 cm ice
cubes, with the small 18 oz tin resting mouth down in its mouth, not yet sealed. The two tins seal by a friction press
fit: pressed home, the small tin's rim jams inside the large tin's tapered mouth and holds until pulled apart.

Task: seal the shaker, shake the drink with its ice, then open the shaker, leaving the drink in the large tin standing
on the table, without spilling any. A sealed shaker is 29.9 cm long and weighs 0.53 kg, and its tins hold together
until pulled apart with 12 N: opening it needs the large tin held down while the small one is pulled off.

Run it:  inspect-robots run --embodiment yam_arena -E task=shake_a_cocktail_with_both_hands.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     plate, rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tins (metres; outside radii; both are 32-sided cones of 2 mm steel) ------------------------------------------
# The large tin (28 oz, 830 ml): origin at the centre of its underside, axis up.
TIN_H, TIN_R0, TIN_R1, TIN_FLOOR, TIN_MASS = 0.175, 0.035, 0.0475, 0.008, 0.21  # 17.5 cm; 7.0 cm base, 9.5 cm mouth
# The small tin (18 oz, 530 ml), built mouth down: origin at the centre of its rim (its mouth), +z toward its base.
SM_H, SM_R0, SM_R1, SM_FLOOR, SM_MASS = 0.145, 0.0432, 0.0305, 0.006, 0.14  # 14.5 cm; 8.64 cm rim, 6.1 cm base
WALL, SIDES = 0.002, 32  # weighted tins: thick floors (8 mm, 6 mm)
# The press fit: the small tin's rim rests REST deep in the large tin's mouth; pressed home it sits SEAT deep, where
# the large tin's taper has narrowed to the rim (the simulated tins keep a 0.8 mm gap there; a joint is the fit).
REST, SEAT = 0.016, 0.021

TIN_XY = (0.22, 0.0)

# -- the drink and the ice ------------------------------------------------------------------------------------------
BEAD_R, BEAD_MASS, N_BEADS, ML_PER_BEAD = 0.004, 0.0053, 24, 5  # 24 x 5 ml = 120 ml (127 g) of drink
ICE, ICE_MASS, N_ICE = 0.025, 0.014, 4  # 2.5 cm cubes of ice, 14 g each

STEEL, STEEL_S = (0.74, 0.75, 0.77), (0.66, 0.67, 0.70)
DRINK_C, ICE_C = (0.30, 0.45, 0.85), (0.86, 0.93, 0.97)

# -- the press fit (a slide joint along the tins' axis; its coordinate is the small tin's rim depth in the mouth) ----
K_SEAL, D_SEAL, F_MAX = 2400.0, 8.0, 60.0
SLIP_IN = 0.003  # unsealed, the rim slides further in once pushed 3 mm past where it sits (2400 N/m x 3 mm = 7.2 N)
SLIP_OUT = 0.0005  # ... and out once pulled 0.5 mm (1.2 N): resting, the small tin lifts off freely
SEATED_WITHIN = 0.0005  # within this of SEAT it is pressed home: sealed
GIVE, POP_STEPS = 0.005, 3  # sealed, it lets go once drawn 5 mm out of its seat (12 N) for 3 control steps (0.15 s)
OFF_DEPTH = -0.005  # the rim this far above the mouth: the tins are apart
CATCH_R, CATCH_DEG = 0.01, 15.0  # a small tin set back in the mouth this near square is caught again

# -- scoring --------------------------------------------------------------------------------------------------------
TIP_SEALED = math.radians(30)  # the large tin tipped past this while not sealed: the seal was open while tipped
SHAKE_TIP = math.radians(45)  # strokes count while sealed and tipped past this ...
TURNS, LEG = 6, 0.025  # ... at least TURNS changes of direction along the axis, each after a leg of LEG or more
MID = 0.15  # the sealed shaker's middle, this far up the large tin's axis
END_ZONE = 0.09  # an ice cube is at an end once its centre is within this of that end's floor and it touches that tin
PASSES = 8  # the ice must reach alternate ends at least this many times in all, while sealed and tipped
LOST_FULL, LOST_HALF = 1, 3  # beads (5 ml each) allowed out: 1.0 and 0.25 / 0.5
UPRIGHT = math.radians(10)
APART = 0.03  # opened: the small tin's rim centre at least this far from the large tin's mouth centre
TOUCH_N = 0.01
SUCCESS_HOLD_S = 1.0


def _r_tin(z: float) -> float:
    """The large tin's outside radius (apothem) at height z above its underside."""
    return TIN_R0 + (TIN_R1 - TIN_R0) * z / TIN_H


def _r_small(z: float) -> float:
    """The small tin's outside radius at z from its rim toward its base."""
    return SM_R0 + (SM_R1 - SM_R0) * z / SM_H


SEAL_GAP = _r_tin(TIN_H - SEAT) - WALL - SM_R0  # 0.8 mm: narrower than a drink bead, so nothing gets out there
assert SEAL_GAP > 0.0007


def _cone(n: int, z0: float, z1: float, r0: float, r1: float, color, collide: bool = True, t: float = WALL) -> list:
    """A cone wall of n flat panels, t thick, its outside at r0 (at z0) and r1 (at z1). Each panel is as wide as the
    widest chord, so neighbours overlap and leave no slot."""
    m0, m1 = r0 - t / 2, r1 - t / 2
    width = 2 * max(m0, m1) * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate((m0 * u[0], m0 * u[1], z0), (m1 * u[0], m1 * u[1], z1), width, t, u, color, collide))
    return out


def _lip(n: int, z: float, r: float, color) -> list:
    """The large tin's rolled lip: a 3 mm bead round the outside of its mouth (looks only)."""
    width = 2 * (r + 0.0015) * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate(((r + 0.0015) * u[0], (r + 0.0015) * u[1], z - 0.003),
                         ((r + 0.0015) * u[0], (r + 0.0015) * u[1], z), width, 0.003, u, color, collide=False))
    return out


def _tin() -> Body:
    shapes = [CylShape((0.0, 0.0, TIN_FLOOR / 2), TIN_R0 - 0.0005, TIN_FLOOR, STEEL)]
    shapes += _cone(SIDES, 0.0, TIN_H, TIN_R0, TIN_R1, STEEL)
    shapes += _lip(SIDES, TIN_H, TIN_R1, STEEL)
    return Body.build("tin", pos=(*TIN_XY, TOP + 0.0005), mass=TIN_MASS, shapes=shapes, friction=(0.35, 0.3), color=STEEL,
                      solver_iterations=64)


def _small_tin() -> Body:
    shapes = [CylShape((0.0, 0.0, SM_H - SM_FLOOR / 2), SM_R1 - 0.0005, SM_FLOOR, STEEL_S)]
    shapes += _cone(SIDES, 0.0, SM_H, SM_R0, SM_R1, STEEL_S)
    return Body.build("small_tin", pos=(*TIN_XY, TOP + 0.0005 + TIN_H - REST), mass=SM_MASS, shapes=shapes,
                      friction=(0.35, 0.3), color=STEEL_S, solver_iterations=64)


def _bead_spots() -> list[tuple[float, float, float]]:
    d = 2 * BEAD_R + 0.0008
    z = TIN_FLOOR + BEAD_R + 0.0005
    rmax = _r_tin(z) - WALL - BEAD_R - 0.0015
    pts = [((i + 0.5 * (j % 2)) * d, j * d * math.sqrt(3) / 2, z) for j in range(-6, 7) for i in range(-6, 7)]
    pts = sorted([p for p in pts if math.hypot(p[0], p[1]) <= rmax], key=lambda p: (round(math.hypot(p[0], p[1]), 6), p))
    return pts[:N_BEADS]


def _beads() -> list[Body]:
    tx, ty = TIN_XY
    return [Body.build(f"drink_{i:02d}", pos=(tx + x, ty + y, TOP + 0.0005 + z), mass=BEAD_MASS, color=DRINK_C,
                       friction=(0.3, 0.25), shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, DRINK_C)], damping=(0.05, 0.3), solver_iterations=32)
            for i, (x, y, z) in enumerate(_bead_spots())]


def _ice() -> list[Body]:
    """Four cubes on top of the drink, in two layers of two side by side, the upper pair turned square to the lower."""
    tx, ty = TIN_XY
    z0 = TIN_FLOOR + 2 * BEAD_R + 0.001 + ICE / 2
    h = (ICE + 0.0015) / 2
    spots = [(h, 0.0, z0), (-h, 0.0, z0), (0.0, h, z0 + ICE + 0.0015), (0.0, -h, z0 + ICE + 0.0015)]
    return [Body.build(f"ice_{i}", pos=(tx + x, ty + y, TOP + 0.0005 + z), mass=ICE_MASS, color=ICE_C,
                       friction=(0.05, 0.04), solver_iterations=32, shapes=[BoxShape((0.0, 0.0, 0.0), (ICE, ICE, ICE), ICE_C)])
            for i, (x, y, z) in enumerate(spots)]


TIN = _tin()
SMALL = _small_tin()
BEADS = _beads()
ICES = _ice()
OBJECTS = [TIN, SMALL, *ICES, *BEADS]
BEAD_NAMES = [b.name for b in BEADS]
ICE_NAMES = [b.name for b in ICES]
CONTENTS = set(BEAD_NAMES + ICE_NAMES)

# the press fit: the small tin's rim slides along the large tin's axis, its depth below the mouth the coordinate
thread("seal", "tin", "small_tin", anchor0=(0.0, 0.0, TIN_H), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(-0.04, SEAT), turn_damping=0.2, slide_stiffness=K_SEAL,
       slide_damping=D_SEAL, slide_max_force=F_MAX, always_on=True)

_J: dict = {}
_author_threads_plain = bodykit._author_threads


def _author_threads(stage, prim_path: str, spec) -> None:
    """As bodykit's, but the fit starts at REST, and each body's prim path is noted for the contact view. The two tins
    never collide with each other (the joint is the fit; contacts between the jointed tins fight it in PhysX), and the
    drink and the ice never collide with the arms: the fingers' collision hulls reach through a gripped tin's wall,
    and would push what is inside out through it."""
    _author_threads_plain(stage, prim_path, spec)
    _J[spec.name] = f"{prim_path}/body"
    if spec.name in ICE_NAMES:  # report the ice's contacts, so the contact view below can read them
        from pxr import PhysxSchema

        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateThresholdAttr(0.0)
    if spec.name in CONTENTS:
        from pxr import UsdPhysics
        from yam_arena.robot import asset_link_paths

        env = prim_path.rsplit("/", 1)[0]
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateFilteredPairsRel()
        for arm in ("LeftArm", "RightArm"):
            for link in asset_link_paths().values():
                pairs.AddTarget(f"{env}/{arm}{link}")
    if spec.name == "small_tin":
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/seal")
        UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr().Set(float(REST))


bodykit._author_threads = _author_threads


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


_CT = {"view": None, "failed": ""}


def _ice_touches() -> np.ndarray | None:
    """(N_ICE, 2): whether each ice cube touches the large tin, the small tin, now (PhysX contact forces)."""
    if _CT["view"] is None and not _CT["failed"] and "tin" in _J and "small_tin" in _J:
        try:
            from isaaclab_physx.physics import PhysxManager

            sim = PhysxManager.get_physics_sim_view()
            _CT["view"] = sim.create_rigid_contact_view([_J[n] for n in ICE_NAMES],
                                                        filter_patterns=[[_J["tin"], _J["small_tin"]]] * N_ICE,
                                                        max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
    if _CT["view"] is None:
        return None
    try:
        m = _np(_CT["view"].get_contact_force_matrix(dt=1.0 / 240.0)).reshape(N_ICE, 2, 3)
    except Exception as e:  # noqa: BLE001
        _CT["failed"], _CT["view"] = repr(e)[:300], None
        return None
    _CT["reads"] = _CT.get("reads", 0) + 1
    return np.linalg.norm(m, axis=2) > TOUCH_N


# -- followed after every control step ------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": True, "hold": REST, "sealed": False, "pull": 0, "pops": 0, "ever_sealed": False,
            "open_tipped": False, "turns": 0, "last": None, "s": 0.0, "ext": 0.0, "lo": 0.0, "hi": 0.0, "dir": 0,
            "ice_end": [None] * N_ICE, "passes": 0, "max_tip": 0.0, "depth": REST, "max_give": 0.0, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SHAKE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    try:
        set_slide_target("seal", REST)
        set_thread_on("seal", True)
    except Exception:  # noqa: BLE001
        pass


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _restart_stroke() -> None:
    _EP.update(s=0.0, ext=0.0, lo=0.0, hi=0.0, dir=0)


def _track_stroke(centre, axis, counting: bool) -> None:
    """Count the turns of the shaker's middle along its own axis while it is sealed and tipped past SHAKE_TIP: s
    integrates the middle's motion along the axis (turning the shaker about its middle hardly moves s), and a turn
    counts once s has come back LEG or more from its last extreme."""
    e = _EP
    last, e["last"] = e["last"], centre
    if not counting or last is None:
        _restart_stroke()
        return
    e["s"] += float(np.dot(centre - last, axis))
    s = e["s"]
    if e["dir"] == 0:
        e["lo"], e["hi"] = min(e["lo"], s), max(e["hi"], s)
        if e["hi"] - e["lo"] >= LEG:
            e["dir"], e["ext"] = (1, s) if s >= e["hi"] else (-1, s)
    elif (s - e["ext"]) * e["dir"] > 0:
        e["ext"] = s
    elif abs(s - e["ext"]) >= LEG:
        e["turns"] += 1
        e["dir"], e["ext"] = -e["dir"], s


def _seal_step(depth: float, off: float, angle: float) -> None:
    """The press fit, from the small tin's rim depth in the mouth (m), its distance off the axis (m) and its tilt
    against the large tin (deg)."""
    e = _EP
    if not e["on"]:
        if 0.0 <= depth <= SEAT + 0.01 and off <= CATCH_R and angle <= CATCH_DEG:  # set back in the mouth: caught
            e.update(on=True, hold=min(depth, SEAT), sealed=False, pull=0)
            set_slide_target("seal", e["hold"])
            set_thread_on("seal", True)
        return
    if depth < OFF_DEPTH:  # drawn out of the mouth: apart
        e.update(on=False, sealed=False, pull=0)
        set_thread_on("seal", False)
        return
    if e["sealed"]:
        e["max_give"] = max(e["max_give"], SEAT - depth)
        if depth < SEAT - GIVE:  # drawn out of its seat: held 0.15 s, the seal lets go
            e["pull"] += 1
            if e["pull"] >= POP_STEPS:
                e.update(sealed=False, pull=0, hold=depth, pops=e["pops"] + 1)
                set_slide_target("seal", depth)
        else:
            e["pull"] = 0
        return
    if depth >= SEAT - SEATED_WITHIN:  # pressed home: sealed
        e.update(sealed=True, ever_sealed=True, hold=SEAT, pull=0)
        set_slide_target("seal", SEAT)
    elif depth > e["hold"] + SLIP_IN or depth < e["hold"] - SLIP_OUT:  # pushed in (7.2 N) or drawn out (1.2 N)
        e["hold"] = depth
        set_slide_target("seal", depth)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    e = _EP
    _DEBUG["steps"] += 1
    t_pos, t_rot = _pose(base, origin, "tin")
    s_pos, s_rot = _pose(base, origin, "small_tin")
    q = t_rot.T @ (s_pos - t_pos)
    depth, off = TIN_H - float(q[2]), math.hypot(float(q[0]), float(q[1]))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(t_rot[:, 2] @ s_rot[:, 2])))))
    e["depth"] = depth
    _seal_step(depth, off, angle)
    tip = math.acos(max(-1.0, min(1.0, float(t_rot[2, 2]))))
    sealed = e["sealed"]
    if tip > TIP_SEALED and not sealed:
        e["open_tipped"] = True  # in a real trial the drink pours out: no credit
    if sealed:
        e["max_tip"] = max(e["max_tip"], tip)
    _track_stroke(t_pos + t_rot[:, 2] * MID, t_rot[:, 2], sealed and tip >= SHAKE_TIP)
    touches = _ice_touches()
    for i, name in enumerate(ICE_NAMES):
        if not sealed:
            e["ice_end"][i] = None
            continue
        if tip < SHAKE_TIP:  # passes count while sealed and tipped past SHAKE_TIP, as the strokes do
            continue
        p = _pose(base, origin, name)[0]
        zb = float((t_rot.T @ (p - t_pos))[2])
        zs = float((s_rot.T @ (p - s_pos))[2])
        end = None
        if zb <= TIN_FLOOR + END_ZONE and (touches is None or touches[i, 0]):
            end = "large"
        elif zs >= SM_H - SM_FLOOR - END_ZONE and (touches is None or touches[i, 1]):
            end = "small"
        if end is not None:
            if e["ice_end"][i] is not None and e["ice_end"][i] != end:
                e["passes"] += 1
            e["ice_end"][i] = end
    tcps = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        k = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][k]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    e["tcps"] = tcps
    if _DEBUG["on"] and _DEBUG["steps"] % 10 == 0:
        lt = t_rot.T @ (tcps[0] - t_pos)
        rt = s_rot.T @ (tcps[1] - s_pos)
        print(f"[shake] t={_DEBUG['steps'] / 20:5.1f}s depth={depth * 1000:5.1f}mm off={off * 1000:4.1f}mm "
              f"ang={angle:4.1f} on={e['on']} sealed={sealed} pops={e['pops']} give={e['max_give'] * 1000:4.1f}mm "
              f"tip={math.degrees(tip):5.1f} turns={e['turns']} passes={e['passes']} ends={e['ice_end']} "
              f"tin_at={np.round((t_pos - [TIN_XY[0], TIN_XY[1], TOP]) * 1000, 1).tolist()} "
              f"left_in_tin={np.round(lt * 1000, 1).tolist()} right_in_small={np.round(rt * 1000, 1).tolist()} "
              f"contacts={'ok %d reads' % _CT.get('reads', 0) if _CT['view'] is not None else 'FAILED ' + _CT['failed'][:80]} "
              f"touch={None if touches is None else touches.astype(int).tolist()}", file=sys.stderr, flush=True)
        for name in ICE_NAMES + BEAD_NAMES:
            p = _pose(base, origin, name)[0]
            if not (_inside_tin(t_pos, t_rot, p) or _inside_small(s_pos, s_rot, p)):
                qb, qs = t_rot.T @ (p - t_pos), s_rot.T @ (p - s_pos)
                print(f"[shake-out] {name} in_large(z,r)=({qb[2] * 1000:.1f},{math.hypot(qb[0], qb[1]) * 1000:.1f}) "
                      f"in_small(z,r)=({qs[2] * 1000:.1f},{math.hypot(qs[0], qs[1]) * 1000:.1f}) "
                      f"tcpL_in_large={np.round(lt * 1000, 1).tolist()}", file=sys.stderr, flush=True)
        if _DEBUG["steps"] % 20 == 0:
            zs = [round(float((t_rot.T @ (_pose(base, origin, n)[0] - t_pos))[2]) * 1000) for n in ICE_NAMES]
            print(f"[shake-ice] z_in_large_mm={zs}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _inside_tin(t_pos, t_rot, w) -> bool:
    q = t_rot.T @ (np.asarray(w, float) - t_pos)
    return TIN_FLOOR < q[2] < TIN_H and math.hypot(q[0], q[1]) < _r_tin(float(q[2])) - WALL


def _inside_small(s_pos, s_rot, w) -> bool:
    q = s_rot.T @ (np.asarray(w, float) - s_pos)
    return 0.0 < q[2] < SM_H - SM_FLOOR and math.hypot(q[0], q[1]) < _r_small(float(q[2])) - WALL


def _tally(objects) -> dict:
    tin, small = objects["tin"], objects["small_tin"]
    t_pos, t_rot = np.asarray(tin.pos, float), rotation(tin.quat_xyzw)
    s_pos, s_rot = np.asarray(small.pos, float), rotation(small.quat_xyzw)

    def kept(name):
        w = objects[name].pos
        return _inside_tin(t_pos, t_rot, w) or _inside_small(s_pos, s_rot, w)

    lost = sum(not kept(n) for n in BEAD_NAMES)
    ice_lost = sum(not kept(n) for n in ICE_NAMES)
    tip = math.acos(max(-1.0, min(1.0, float(t_rot[2, 2]))))
    mouth = t_pos + t_rot[:, 2] * TIN_H
    opened = not _EP["on"] and float(np.linalg.norm(s_pos - mouth)) >= APART
    return {"lost": lost, "ice_lost": ice_lost, "opened": opened, "open_tipped": _EP["open_tipped"],
            "shaken": _EP["turns"] >= TURNS and _EP["passes"] >= PASSES,
            "upright_on_table": tip < UPRIGHT and abs(float(t_pos[2]) - TOP) < 0.01}


def _points(t) -> float:
    if t["open_tipped"] or not t["upright_on_table"] or t["ice_lost"] > 0 or not t["shaken"]:
        return 0.0
    if t["opened"]:
        if t["lost"] <= LOST_FULL:
            return 1.0
        return 0.5 if t["lost"] <= LOST_HALF else 0.0
    return 0.25 if t["lost"] <= LOST_FULL else 0.0


def score(objects) -> float:
    """0 if the large tin was ever tipped past 30 degrees while the shaker was not sealed (pressed home), if any ice
    cube is out of the tins, unless the large tin ends standing upright on the table, or unless the shaker was shaken:
    while sealed and tipped past 45 degrees, at least 6 changes of direction along its axis, each after a leg of 2.5 cm
    or more, and the ice cubes reaching alternate ends (within 9 cm of that end's floor, touching that tin) at least 8
    times in all. Otherwise 1.0 if the shaker is open (the small tin off: its rim at least 3 cm from the large tin's
    mouth) with at most 1 of the 24 drink beads (5 ml each) out of the tins; 0.5 if open with at most 3 out; 0.25 if not
    opened, with at most 1 out; else 0."""
    t = _tally(objects)
    if _DEBUG["on"] and _DEBUG["steps"] % 20 == 0:
        print(f"[shake-score] lost={t['lost']} ice_lost={t['ice_lost']} opened={t['opened']} "
              f"open_tipped={t['open_tipped']} shaken={t['shaken']} upright={t['upright_on_table']} "
              f"score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the tins, the ice and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("tin", "small_tin", *ICE_NAMES, *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# The right hand grips the small tin from the right side, the left grips the large tin low from the left, each with
# its fingers closing front and back. The right presses the small tin home; then the shaker follows one shared path,
# given as where the large tin's underside centre is and how far the shaker is tipped forward (about world y), with
# each hand fixed on its tin. Finally the left holds the large tin down while the right pulls the small tin off.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_GRIP = {
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2),  # pointing +y
}
LEFT_DOWN = math.radians(20)  # the left grip points 20 deg down: a level side grip can't reach this low
LEFT_LEAN, RIGHT_LEAN = math.radians(-6.0), math.radians(4.5)  # the pads aren't parallel: counter-turn each grip
GRIP = {"left": _ry(LEFT_LEAN) @ _rx(LEFT_DOWN) @ SIDE_GRIP["left"], "right": _ry(RIGHT_LEAN) @ SIDE_GRIP["right"]}
H_LEFT = 0.055  # the left grip on the large tin, above its underside
SM_GRIP = 0.066  # the right grip on the small tin, above its rim
H_RIGHT = TIN_H - SEAT + SM_GRIP  # ... above the large tin's underside once sealed
OUT = 0.07
PRESS = 0.035  # the right aims this far below the seated grip to press the small tin home
SQUEEZE = 0.005  # while the shaker is off the table each hand aims this far toward the other along its axis
LIFT, PIVOT = 0.03, 0.15  # lift the shaker 3 cm, tip it about its middle
SHAKE_AT, ROCK = math.radians(86), math.radians(21)  # shake between 65 and 107 deg (the wrist roll allows ~115)
STROKE, LEGS, LEG_S, DWELL = 0.03, 12, 0.7, 0.3  # ... 3 cm along the axis each way, 12 legs, a pause at each end
TIP_STEP = math.radians(15)
HOLD_DOWN, PULL = 0.05, 0.10  # opening: the left presses 5 cm into the table, the right pulls 10 cm straight up
LEAD_S, LEAD_STEPS = 3.0, 15
B0 = np.array([TIN_XY[0], TIN_XY[1], TOP + 0.0005])
PIVOT_AT = B0 + [0.0, 0.0, LIFT + PIVOT]


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
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _hand(arm: str, base_at, tip: float, squeeze: float = 0.0):
    """The TCP pose of one hand with the large tin's underside centre at base_at and the shaker tipped forward by tip."""
    rot = _ry(tip)
    h = H_LEFT + squeeze if arm == "left" else H_RIGHT - squeeze
    return np.asarray(base_at, float) + rot @ np.array([0.0, 0.0, h]), rot @ GRIP[arm]


def _about_pivot(tip: float, axial: float = 0.0):
    rot = _ry(tip)
    return PIVOT_AT - rot @ np.array([0.0, 0.0, PIVOT - axial])


def _shake_path() -> list:
    """(seconds, large tin underside centre, tip, squeeze, label) after both grips hold and the shaker is sealed."""
    up = B0 + [0.0, 0.0, LIFT]
    path = [(1.5, up, 0.0, SQUEEZE, "lift the shaker")]
    n = round(SHAKE_AT / TIP_STEP)
    for k in range(1, n + 1):
        a = k * SHAKE_AT / n
        path.append((0.4, _about_pivot(a), a, SQUEEZE, "tip it on its side"))
    tip, axial = SHAKE_AT, 0.0
    for k in range(LEGS):
        to_tip = SHAKE_AT + (ROCK if k % 2 == 0 else -ROCK)  # small tin end down, then up
        to_ax = STROKE if k % 2 == 0 else -STROKE  # ... and the shaker moved with the sliding ice, which then hits the end
        mid_tip, mid_ax = (tip + to_tip) / 2, (axial + to_ax) / 2
        path.append((LEG_S / 2, _about_pivot(mid_tip, mid_ax), mid_tip, SQUEEZE, "shake"))
        path.append((LEG_S / 2, _about_pivot(to_tip, to_ax), to_tip, SQUEEZE, "shake"))
        path.append((DWELL, _about_pivot(to_tip, to_ax), to_tip, SQUEEZE, "shake: the ice slides to the end"))
        tip, axial = to_tip, to_ax
    path.append((0.5, _about_pivot(SHAKE_AT), SHAKE_AT, SQUEEZE, "shake: back to the middle"))
    for k in range(n - 1, -1, -1):
        a = k * SHAKE_AT / n
        path.append((0.4, _about_pivot(a), a, SQUEEZE, "stand it up"))
    path += [(1.5, B0 + [0.0, 0.0, 0.003], 0.0, 0.0, "set it down")]
    return path


SHAKE = _shake_path()
RIGHT_IN = [1.5, 0.8]  # the right hand: slide in round the small tin, grip it
LEFT_IN = [1.5, 0.8]  # then the left: slide in round the large tin, grip it
PRESSING = [1.0, 0.6, 0.8]  # then the right presses the small tin home, holds, eases off
OPENING = [0.8, 1.5, 0.6, 1.5, 0.8, 1.2, 2.0]  # hold down, pull off, pause, up, left lets go, left out, hold still


def _right() -> list[Waypoint]:
    rot = GRIP["right"]
    at_rest = B0 + [0.0, 0.0, TIN_H - REST + SM_GRIP]
    seated = B0 + [0.0, 0.0, H_RIGHT]
    side = np.array([0.0, -OUT, 0.0])
    out = _lead_in("right", at_rest + side, rot, "level, beside the small tin")
    out += [
        _w(RIGHT_IN[0], at_rest, FINGER_OPEN, "slide in round the small tin", rot),
        _w(RIGHT_IN[1], at_rest, FINGER_CLOSED, "grip the small tin", rot),
        _w(sum(LEFT_IN), at_rest, FINGER_CLOSED, "wait while the left grips the large tin", rot),
        _w(PRESSING[0], seated - [0.0, 0.0, PRESS], FINGER_CLOSED, "press the small tin home: sealed", rot),
        _w(PRESSING[1], seated - [0.0, 0.0, PRESS], FINGER_CLOSED, "hold the press", rot),
        _w(PRESSING[2], seated, FINGER_CLOSED, "ease off", rot),
    ]
    for dt, b, tip, sq, label in SHAKE:
        pos, r = _hand("right", b, tip, sq)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    pos, r = _hand("right", B0, 0.0)
    pulled = pos + [0.0, 0.0, PULL]
    up = pulled + [0.0, 0.0, 0.02]  # held over the large tin: anything wedged in the small tin can only fall back in
    out += [_w(OPENING[0], pos, FINGER_CLOSED, "hold while the left holds the large tin down", r),
            _w(OPENING[1], pulled, FINGER_CLOSED, "pull the small tin off: the seal lets go", r),
            _w(OPENING[2], pulled, FINGER_CLOSED, "pause over the large tin: anything inside drops back", r),
            _w(OPENING[3], up, FINGER_CLOSED, "hold the small tin up, off the open large tin", r),
            _w(sum(OPENING[4:]), up, FINGER_CLOSED, "hold it still", r)]
    return out


def _left() -> list[Waypoint]:
    grip, rot = _hand("left", B0, 0.0)
    side = np.array([0.0, OUT, 0.0])
    out = _lead_in("left", grip + side, rot, "beside the large tin, pointing down a little")
    out += [
        _w(sum(RIGHT_IN), grip + side, FINGER_OPEN, "wait while the right grips the small tin", rot),
        _w(LEFT_IN[0], grip, FINGER_OPEN, "slide in round the large tin", rot),
        _w(LEFT_IN[1], grip, FINGER_CLOSED, "grip the large tin", rot),
        _w(sum(PRESSING), grip, FINGER_CLOSED, "hold it while the right presses the small tin home", rot),
    ]
    for dt, b, tip, sq, label in SHAKE:
        pos, r = _hand("left", b, tip, sq)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    pos, r = _hand("left", B0, 0.0)
    down = pos - [0.0, 0.0, HOLD_DOWN]
    out += [_w(OPENING[0], down, FINGER_CLOSED, "hold the large tin down", r),
            _w(sum(OPENING[1:4]), down, FINGER_CLOSED, "keep holding it down", r),
            _w(OPENING[4], pos, FINGER_OPEN, "let go of the large tin", r),
            _w(OPENING[5], pos + side, FINGER_OPEN, "slide out", r),
            _w(OPENING[6], pos + side + [0.0, 0.0, 0.05], FINGER_OPEN, "up and away", r)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
