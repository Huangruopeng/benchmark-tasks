"""A kitchen mandoline slicer stands on the worktop: a stainless-steel deck sloping up to the left between two black rails,
with a diagonal blade across its lower part, a folding wire leg under its upper end and a loop handle beyond it. A red
plastic hand guard sits on the deck above the blade, a chunk of cucumber standing in it with its cut end on the deck.
A white tray stands under the lower half of the mandoline, under the blade, its pull tab sticking out past the
mandoline's lower end.

Task: slice the cucumber safely. Hold the mandoline steady by its handle with one hand while the other slides the
guard down the deck across the blade and back up, four times: each pass cuts a slice, which falls into the tray. Never
bring a gripper near the blade itself. Then leave the guard at the top of the deck, slide the tray of slices out from
under the mandoline by its tab and take both arms back to where they started. The guard slides stiffly in the rails
(it moves only when pushed with more than about 3 N, the blade's cut included) while the mandoline stands on smooth
feet that let it slide at about 0.5 N: an unheld mandoline slides off with the guard and nothing is cut.

Run it:  inspect-robots run --embodiment yam_arena -E task=slice_a_cucumber_on_a_mandoline_with_its_hand_guard.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation, segment)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the mandoline: its origin is on the table under the middle of the deck's lower end ------------------------------
INC = math.radians(15.0)  # the deck rises this steeply toward +y (the left)
C, S = math.cos(INC), math.sin(INC)
U = np.array([0.0, C, S])  # up the deck
N = np.array([0.0, -S, C])  # out of the deck
MX, Y0 = 0.320, -0.190  # the deck's lower end, on the table
Z0 = 0.036  # the deck's surface at its lower end, above the table
L_DECK, W_DECK = 0.300, 0.110
S_BLADE = 0.095  # the blade crosses the deck here (its middle), slanting 1.2 cm either way
BLADE_SLANT = 0.012
FOOT_X = 0.074  # the rubber feet under the lower end, this far out either side (the tray slides out between)
LEG_S = 0.240  # the folding leg, under the deck this far up
HANDLE = (0.305, 0.400, 0.040)  # the loop handle's side bars, from/to (along the deck) and out from the middle
MAND_MASS = 0.70
# -- the hand guard: its origin is on the deck's surface under its middle ----------------------------------------------
S_START = 0.200  # where it starts (its middle, along the deck)
S_LOW, S_HIGH = 0.045, 0.245  # how far it can slide
GUARD_L, GUARD_W, GUARD_H = 0.075, 0.096, 0.036
KNOB_H = 0.026
GRIP_OFF = 0.054  # the hand holds the knob this far out of the deck
CUKE_R = 0.020
K_GUARD, SLIP_G, D_GUARD = 5000.0, 0.001, 1.0  # the guard's stiff slide: held where it is left ...
F_GUARD = 3.0  # ... with at most 3 N (its drive's force cap): it slides when pushed harder, like friction
S_ARM, S_CUT = 0.150, 0.078  # a pass: from above S_ARM down to below S_CUT (the cucumber pushed across the blade)
S_PARK = 0.160  # left at the top of the deck: at least this far up
# -- the tray: its origin is under the middle of its floor -------------------------------------------------------------
TRAY_Y = -0.110
TRAY_L, TRAY_W, TRAY_H, TRAY_T = 0.130, 0.094, 0.018, 0.004
FIN_L, FIN_H, FIN_T = 0.065, 0.028, 0.006  # the pull tab: a low bar out to a taller grip
BAR_H = 0.010
FIN_Y = -0.2275  # where the hand takes the tab
PULLED = 0.100  # the tray counts as slid out once moved this far toward -y
# -- the slices: kept out of sight on a shelf under the table until cut ------------------------------------------------
N_SLICES = 6
SLICE_R, SLICE_T = 0.019, 0.003
SHELF = np.array([0.45, 0.0, TOP - 0.20])
SPREAD = ((0.000, 0.000), (0.016, 0.010), (-0.015, 0.006), (0.006, -0.016), (-0.008, -0.010), (0.010, 0.016))  # dx, ds
# -- safety: a gripper's TCP this close to the blade's edge, at any time, scores 0 -----------------------------------------
BLADE_NEAR = 0.030
NEED = 4  # slices

STEEL, STEEL_D, BRIGHT = (0.78, 0.79, 0.81), (0.55, 0.56, 0.58), (0.93, 0.94, 0.96)
BLACK, RUBBER, DARK = (0.09, 0.09, 0.10), (0.16, 0.16, 0.17), (0.04, 0.04, 0.05)
RED, RED_D = (0.84, 0.12, 0.10), (0.62, 0.07, 0.06)
SKIN, SKIN_L, FLESH = (0.12, 0.33, 0.12), (0.30, 0.52, 0.20), (0.84, 0.91, 0.70)
WHITE, WHITE_D = (0.94, 0.94, 0.92), (0.84, 0.84, 0.82)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _p(s: float, off: float = 0.0, dx: float = 0.0) -> tuple[float, float, float]:
    """A point in the mandoline's frame: ``s`` up the deck from its lower end, ``off`` out of the deck's surface, ``dx``
    across it."""
    return (dx, s * C - off * S, Z0 + s * S + off * C)


def _q(t: float, off: float = 0.0, dx: float = 0.0) -> tuple[float, float, float]:
    """A point in the guard's frame: ``t`` up the deck from its middle, ``off`` out of the deck, ``dx`` across."""
    return (dx, t * C - off * S, t * S + off * C)


M0 = np.array([MX, Y0, TOP])
G0 = M0 + np.array(_p(S_START))


def _mandoline() -> Body:
    shapes = [plate(_p(0.0, -0.002), _p(L_DECK, -0.002), W_DECK, 0.004, N, STEEL)]  # the deck
    for k in range(1, 12):  # fine ridges on the deck, as on a real one (visual)
        s = 0.012 + 0.024 * k
        if abs(s - S_BLADE) > 0.02:
            shapes.append(plate(_p(s, 0.0003, -0.04), _p(s, 0.0003, 0.04), 0.002, 0.0006, N, STEEL_D, collide=False))
    # the blade: a dark slot across the deck and its bright edge just above it, slanting
    shapes.append(segment(_p(S_BLADE + BLADE_SLANT - 0.006, 0.0004, -0.050), _p(S_BLADE - BLADE_SLANT - 0.006, 0.0004,
                                                                                    0.050), 0.011, DARK, collide=False))
    shapes.append(segment(_p(S_BLADE + BLADE_SLANT, 0.0012, -0.050), _p(S_BLADE - BLADE_SLANT, 0.0012, 0.050), 0.0040,
                          BRIGHT, collide=False))
    for side in (-1.0, 1.0):  # the black rails along both edges
        shapes.append(plate(_p(-0.006, 0.004, side * 0.060), _p(L_DECK + 0.006, 0.004, side * 0.060), 0.010, 0.020, N,
                            BLACK))
    shapes.append(plate(_p(-0.006, 0.004, -FOOT_X - 0.008), _p(-0.006, 0.004, FOOT_X + 0.008), 0.010, 0.020, N,
                        BLACK))  # the end caps (the lower one wide, over the feet)
    shapes.append(plate(_p(L_DECK + 0.006, 0.004, -0.065), _p(L_DECK + 0.006, 0.004, 0.065), 0.010, 0.020, N, BLACK))
    for side in (-1.0, 1.0):
        # the rubber feet under the lower end
        shapes.append(BoxShape((side * FOOT_X, -0.004, (Z0 - 0.006) / 2), (0.012, 0.016, Z0 - 0.006), RUBBER))
        # the folding wire leg under the upper end, its foot bar on the table
        top = _p(LEG_S, -0.006, side * 0.050)
        foot = (side * 0.056, top[1] + 0.025, 0.005)
        shapes.append(segment(top, foot, 0.008, STEEL_D))
        # the loop handle's side bars beyond the top end
        shapes.append(segment(_p(HANDLE[0], 0.006, side * HANDLE[2]), _p(HANDLE[1], 0.006, side * HANDLE[2]), 0.016,
                              BLACK))
    yf = _p(LEG_S, -0.006)[1] + 0.025
    shapes.append(segment((-0.060, yf, 0.005), (0.060, yf, 0.005), 0.009, STEEL_D))
    shapes.append(segment(_p(HANDLE[1], 0.006, -HANDLE[2] - 0.008), _p(HANDLE[1], 0.006, HANDLE[2] + 0.008), 0.016,
                          BLACK))
    return Body.build("mandoline", pos=tuple(M0.tolist()), mass=MAND_MASS, shapes=shapes, color=STEEL,
                      friction=(0.08, 0.06), slick=True, com=(0.0, 0.150, 0.070), damping=(0.0, 0.05))


def _guard() -> Body:
    t2 = GUARD_L / 2
    shapes = []
    for side in (-1.0, 1.0):  # the runners on the deck and the side walls
        shapes.append(plate(_q(-t2, 0.004, side * 0.044), _q(t2, 0.004, side * 0.044), 0.010, 0.008, N, RED_D))
        shapes.append(plate(_q(-t2, GUARD_H / 2, side * 0.045), _q(t2, GUARD_H / 2, side * 0.045), GUARD_H, 0.006,
                            (1.0, 0.0, 0.0), RED))
    shapes.append(plate(_q(-t2, GUARD_H, 0.0), _q(t2, GUARD_H, 0.0), GUARD_W, 0.005, N, RED))  # the top
    for t in (-t2 + 0.004, t2 - 0.004):  # short lips at the front and back, over the cucumber
        shapes.append(plate(_q(t, GUARD_H - 0.006, -0.040), _q(t, GUARD_H - 0.006, 0.040), 0.008, 0.012, N, RED,
                            collide=False))
    # a round window in the top shows the cucumber's end
    shapes.append(CylShape(_q(0.0, GUARD_H + 0.0027, 0.0), CUKE_R, 0.0006, SKIN, collide=False))
    shapes.append(CylShape(_q(0.0, GUARD_H + 0.0030, 0.0), CUKE_R - 0.003, 0.0006, FLESH, collide=False))
    shapes.append(segment(_q(0.0, GUARD_H, 0.0), _q(0.0, GUARD_H + KNOB_H, 0.0), 0.022, RED_D))  # the knob
    shapes.append(SphereShape(_q(0.0, GUARD_H + KNOB_H, 0.0), 0.012, RED_D, collide=False))
    # the cucumber, standing in the guard on its cut end (a stack of thin discs along the deck's normal; visual only)
    for k in range(7):
        off = 0.0025 + 0.0047 * k
        shapes.append(CylShape(_q(0.0, off, 0.0), CUKE_R, 0.0050, SKIN if k % 3 else SKIN_L, collide=False))
    shapes.append(CylShape(_q(0.0, 0.0012, 0.0), CUKE_R - 0.0015, 0.0012, FLESH, collide=False))
    return Body.build("guard", pos=tuple(G0.tolist()), mass=0.10, shapes=shapes, color=RED, friction=(1.0, 0.9),
                      com=_q(0.0, 0.022, 0.0))


def _tray() -> Body:
    hx, hy = TRAY_W / 2, TRAY_L / 2
    shapes = [BoxShape((0.0, 0.0, 0.0015), (TRAY_W, TRAY_L, 0.003), WHITE)]
    for sx in (-1.0, 1.0):
        shapes.append(BoxShape((sx * (hx - TRAY_T / 2), 0.0, TRAY_H / 2), (TRAY_T, TRAY_L, TRAY_H), WHITE_D))
    for sy in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, sy * (hy - TRAY_T / 2), TRAY_H / 2), (TRAY_W, TRAY_T, TRAY_H), WHITE_D))
    shapes.append(BoxShape((0.0, -hy - FIN_L / 2, BAR_H / 2), (FIN_T, FIN_L, BAR_H), WHITE_D))  # the pull tab: a low bar
    shapes.append(BoxShape((0.0, -hy - FIN_L + 0.0125, FIN_H / 2), (FIN_T, 0.025, FIN_H), WHITE_D))  # and its grip
    return Body.build("tray", pos=(MX, TRAY_Y, TOP), mass=0.12, shapes=shapes, color=WHITE, friction=(0.45, 0.40),
                      ignore=("mandoline",))  # it stands clear of the mandoline; it never stops it sliding


def _shelf() -> Body:
    """Out of sight under the table: where the slices wait until they are cut."""
    return Body.build("shelf", pos=tuple(SHELF.tolist()), mass=1.0, kinematic=True, color=DARK,
                      shapes=[BoxShape((0.0, 0.0, -0.005), (0.10, 0.40, 0.010), None)])


def _slice(k: int) -> Body:
    shapes = [CylShape((0.0, 0.0, SLICE_T / 2), SLICE_R, SLICE_T, SKIN),
              CylShape((0.0, 0.0, SLICE_T / 2 + 0.0002), SLICE_R - 0.0018, SLICE_T, FLESH, collide=False)]
    for i in range(6):  # the seeds
        a = 2 * math.pi * i / 6
        shapes.append(BoxShape((0.007 * math.cos(a), 0.007 * math.sin(a), SLICE_T + 0.0004), (0.004, 0.0022, 0.0004),
                               (0.70, 0.80, 0.55), yaw=a, collide=False))
    return Body.build(f"slice{k}", pos=(SHELF[0], SHELF[1] - 0.15 + 0.05 * k, SHELF[2] + 0.0005), mass=0.006,
                      shapes=shapes, color=FLESH, friction=(0.9, 0.8), damping=(0.3, 0.3))


OBJECTS = [_mandoline(), _guard(), _tray(), _shelf()] + [_slice(k) for k in range(N_SLICES)]


# -- the guard's stiff slide down the deck: a prismatic joint along the deck, authored with the guard -------------------

_SLIDE = {"path": None, "attrs": None}
_plain_author_sliders = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _plain_author_sliders(stage, prim_path, spec)
    if spec.name != "guard":
        return
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/guard_slide")
    joint.CreateBody0Rel().SetTargets([f"{parent}/mandoline/body"])
    joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*(G0 - M0).tolist()))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    q = Gf.Quatf(*bodykit._x_to(U))  # the joint's x axis up the deck
    joint.CreateLocalRot0Attr(q)
    joint.CreateLocalRot1Attr(q)
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(float(S_LOW - S_START))
    joint.CreateUpperLimitAttr(float(S_HIGH - S_START))
    joint.CreateExcludeFromArticulationAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(K_GUARD)
    drive.CreateDampingAttr(D_GUARD)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateMaxForceAttr(F_GUARD)
    _SLIDE["path"] = f"{prim_path}/guard_slide"
    _SLIDE["attrs"] = None


bodykit._author_sliders = _author_sliders


def _set_guard_hold(rel: float) -> None:
    """Move the guard's hold (m along the deck from where it started)."""
    if _SLIDE["attrs"] is None:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(_SLIDE["path"]), "linear")
        _SLIDE["attrs"] = drive.GetTargetPositionAttr()
    _SLIDE["attrs"].Set(float(rel))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": 0.0, "s": S_START, "armed": True, "cut": 0, "placed": [], "unsafe": False, "tcps": None,
            "near": 1.0, "s_state": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MANDOLINE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if _SLIDE["path"] is not None:
        _set_guard_hold(0.0)


def _tcps(base, origin) -> list[np.ndarray]:
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


def _place(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pm, rm = _pose(base, origin, "mandoline")
    pg, _ = _pose(base, origin, "guard")
    pt, _ = _pose(base, origin, "tray")
    s = float(np.dot(rm.T @ (pg - pm) - np.array([0.0, 0.0, Z0]), U))  # the guard's middle, up the deck
    _EP["s"] = s
    rel = s - S_START
    if abs(rel - _EP["hold"]) > SLIP_G:  # pushed past its hold: it slides, and stays where it is left
        _EP["hold"] = rel
        _set_guard_hold(rel)
    # a pass across the blade cuts a slice, which drops into whatever is under the blade
    if _EP["armed"] and s <= S_CUT:
        _EP["armed"] = False
        k = _EP["cut"]
        _EP["cut"] += 1
        if k < N_SLICES:
            dx, ds = SPREAD[k]  # a loose pile, as slices fall
            _place(base, origin, f"slice{k}", pm + rm @ np.array(_p(S_BLADE + ds, -0.018, dx)))
    elif not _EP["armed"] and s >= S_ARM:
        _EP["armed"] = True
    # safety: no gripper at the blade's edge
    a = pm + rm @ np.array(_p(S_BLADE + BLADE_SLANT, 0.001, -0.050))
    b = pm + rm @ np.array(_p(S_BLADE - BLADE_SLANT, 0.001, 0.050))
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    near = min(_seg_dist(t, a, b) for t in tcps)
    _EP["near"] = min(_EP["near"], near)
    if near < BLADE_NEAR:
        _EP["unsafe"] = True
    # what the score is built from
    on_tray = 0
    for k in range(N_SLICES):
        ps, _ = _pose(base, origin, f"slice{k}")
        d = ps - pt
        if abs(d[0]) < TRAY_W / 2 and abs(d[1]) < TRAY_L / 2 and -0.002 < d[2] < TRAY_H + 0.02:
            on_tray += 1
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rm[2, 2])))))
    _EP["s_state"] = {"slices": on_tray, "pulled": TRAY_Y - pt[1] >= PULLED, "parked": s >= S_PARK,
                      "steady": float(np.linalg.norm(pm[:2] - M0[:2])) <= 0.03 and tilt <= 10.0}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = tcps
            print(f"[mandoline] t={_DEBUG['steps'] / 20:5.1f}s s={s * 1000:6.1f} hold={_EP['hold'] * 1000:6.1f}"
                  f" cut={_EP['cut']} on_tray={on_tray} mand={np.round((pm - M0) * 1000).astype(int).tolist()}"
                  f" tilt={tilt:.1f} tray={np.round((pt - [MX, TRAY_Y, TOP]) * 1000).astype(int).tolist()}"
                  f" near={near * 1000:.0f} unsafe={_EP['unsafe']}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((tr - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a gripper has come within 3 cm of the blade's edge. Otherwise, at the end: 1.0 with at least four
    slices in the tray, the tray slid out from under the mandoline, the guard left at the top of the deck, the
    mandoline upright where it started and both arms back where they started; 0.75 with four slices in the tray and the
    tray slid out; 0.5 with four slices in the tray; 0.25 with two or three; else 0.0."""
    if _EP["unsafe"]:
        return 0.0
    st = _EP["s_state"]
    n = st.get("slices", 0)
    if n < 2:
        return 0.0
    if n < NEED:
        return 0.25
    if not st["pulled"]:
        return 0.5
    if st["parked"] and st["steady"] and _arms_home():
        return 1.0
    return 0.75


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
    """Full score with the mandoline, its guard and the tray still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["mandoline"].resting() and objects["guard"].resting()
            and objects["tray"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the handle's near side bar and holds the mandoline. The right comes down on the guard's
# knob, takes it and slides the guard down the deck past the blade and back up, four times (a slice drops into the tray
# each time), then leaves it at the top of the deck, lets go and rises. The left lets go and goes home. The right takes
# the tray's pull tab, slides the tray out from under the mandoline toward the right, lets go and goes home.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.060
HANDLE_FINGER = -0.016
KNOB_FINGER = -0.018
FIN_FINGER = -0.012
PASS_LOW, PASS_HIGH = 0.050, 0.225  # where the hand aims the guard (against the arm's lag)
PARK_AT = 0.235
PASSES = 4
DOWN_S, UP_S = 1.4, 1.2
FIN_DZ = -0.002  # the TCP this far above the tab's top (the arm stops short of low targets)
PULL_TO = -0.150  # the tab pulled this far toward -y (past PULLED, against the arm's lag)
TILT = rotation((math.sin(INC / 2), 0.0, 0.0, math.cos(INC / 2)))  # about x by INC: square to the deck
LEAN = rotation((-math.sin(INC / 2), 0.0, 0.0, math.cos(INC / 2)))  # about x by -INC: the left wrist leaning out
HANDLE_S = 0.355
APPROACH = 0.045  # come down onto a grip from this far straight above


def _handle_grip():
    p = M0 + np.array(_p(HANDLE_S, 0.006, -HANDLE[2]))
    return p, LEAN @ grip_rotation(-math.pi / 2)


def _knob_grip(s: float):
    p = M0 + np.array(_p(s, GRIP_OFF))
    return p, TILT @ grip_rotation(math.pi / 2)


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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    L, R = [], []
    up = np.array([0.0, 0.0, OVER])
    n_up = np.array([0.0, 0.0, APPROACH])

    # the left hand takes the handle's near side bar
    hp, hr = _handle_grip()
    L += _joint_path("left", np.array(READY, float), _good_ik("left", hp + n_up, hr), "over the mandoline's handle",
                     HANDLE_FINGER)
    L.append(_w(1.2, hp, HANDLE_FINGER, "down round the handle's side bar", hr))
    L.append(_w(0.6, hp, FINGER_CLOSED, "hold the mandoline steady by its handle", hr))
    # meanwhile the right comes down on the guard's knob
    kp, kr = _knob_grip(S_START)
    R += _joint_path("right", np.array(READY, float), _good_ik("right", kp + n_up, kr), "over the hand guard's knob",
                     KNOB_FINGER)
    R.append(_w(1.2, kp, KNOB_FINGER, "down round the guard's knob", kr))
    R.append(_w(0.6, kp, FINGER_CLOSED, "take the guard by its knob", kr))
    R.append(_w(0.4, kp, FINGER_CLOSED, "take the guard by its knob", kr))
    # four passes down the deck across the blade and back up
    for k in range(PASSES):
        p, r = _knob_grip(PASS_LOW)
        R.append(_w(DOWN_S, p, FINGER_CLOSED, f"slide the guard down across the blade (pass {k + 1})", r))
        R.append(_w(0.3, p, FINGER_CLOSED, f"slide the guard down across the blade (pass {k + 1})", r))
        p, r = _knob_grip(PASS_HIGH if k < PASSES - 1 else PARK_AT)
        R.append(_w(UP_S, p, FINGER_CLOSED, "slide it back up the deck", r))
        R.append(_w(0.3, p, FINGER_CLOSED, "slide it back up the deck", r))
    R.append(_w(0.5, p, KNOB_FINGER, "leave the guard at the top of the deck", r))
    R.append(_w(0.8, p + n_up, KNOB_FINGER, "rise clear of the guard", r))
    t_done = sum(w.duration for w in R)
    # the left holds on until then, lets go and goes home
    L.append(_w(max(0.2, t_done - sum(w.duration for w in L)), hp, FINGER_CLOSED,
                "hold the mandoline steady by its handle", hr))
    L.append(_w(0.5, hp, HANDLE_FINGER, "let go of the handle", hr))
    L.append(_w(0.8, hp + n_up, HANDLE_FINGER, "rise clear of the handle", hr))
    L += _joint_path("left", _good_ik("left", hp + n_up, hr), np.array(READY, float), "back to where it started",
                     HANDLE_FINGER)
    # the right takes the tray's pull tab and slides the tray out from under the mandoline
    g = grip_rotation(math.pi / 2)
    tab = np.array([MX, FIN_Y, TOP + FIN_H + FIN_DZ])
    R.append(_w(1.6, tab + up + [0.0, 0.0, 0.04], FIN_FINGER, "over the tray's pull tab", g))
    R.append(_w(0.8, tab + up, FIN_FINGER, "over the tray's pull tab", g))
    R.append(_w(1.4, tab, FIN_FINGER, "down round the tab", g))
    R.append(_w(0.6, tab, FINGER_CLOSED, "take the tray's pull tab", g))
    out = tab + np.array([0.0, PULL_TO, 0.0])
    R.append(_w(2.5, out, FINGER_CLOSED, "slide the tray of slices out from under the mandoline", g))
    R.append(_w(0.4, out, FINGER_CLOSED, "slide the tray of slices out from under the mandoline", g))
    R.append(_w(0.5, out, FIN_FINGER, "let go of the tab", g))
    R.append(_w(0.8, out + up, FIN_FINGER, "rise clear of the tray", g))
    R += _joint_path("right", _good_ik("right", out + up, g), np.array(READY, float), "back to where it started",
                     FIN_FINGER)
    R.append(_w(3.0, R[-1].pos, FIN_FINGER,
                "hold still: the trial ends once the full score has held 1 s with both arms home", R[-1].yaw))
    t_end = sum(w.duration for w in R)
    L.append(_w(max(0.5, t_end - sum(w.duration for w in L)), L[-1].pos, HANDLE_FINGER, "wait at home", L[-1].yaw))
    return {"left": L, "right": R}
