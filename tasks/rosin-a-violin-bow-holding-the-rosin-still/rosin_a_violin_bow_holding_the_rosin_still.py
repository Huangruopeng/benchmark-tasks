"""A full-size violin lies on a green felt cloth at the back of a black-lacquered music-room table, with its full-size
bow (75 cm, 60 g) laid beside it, hair down, the black ebony frog to the right. In front, a cake of amber rosin sits in
its little beech holder, which has a raised grip across its front end, on the smooth lacquer. The bow's hair is clean
and pale: it has not been rosined, so it would slide over the strings without a sound.

Task: rosin the bow. Pick it up by its frog and draw the hair back and forth across the rosin cake while the other hand
holds the holder still by its grip, until the hair is coated white along most of its length (the rosin may be slid to
a new spot between strokes); then lay the bow back down on the cloth beside the violin and take both arms back to where
they started.

Why two hands: the holder stands on smooth lacquer (it slides at about 0.03 N), and the bow's hair grips the rosin;
drawn across it with nothing holding it, the rosin just travels along with the hair and nothing is coated. One hand
holds the holder by its raised grip while the other draws the bow.

Run it:  inspect-robots run --embodiment yam_arena -E task=rosin_a_violin_bow_holding_the_rosin_still.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the table's lacquered top (front) and the felt cloth (back), fixed -------------------------------------------------
LX0, LX1, LY0, LY1, LAC_H = 0.100, 0.300, -0.380, 0.400, 0.006
CX0, CX1, CY0, CY1, CLOTH_H = 0.290, 0.570, -0.370, 0.450, 0.008
LT = TOP + LAC_H  # the lacquer's top
CT = TOP + CLOTH_H  # the cloth's top
# -- the rosin: a round amber cake set in a wooden holder; origin at the holder's underside middle ----------------------
HOLD_L, HOLD_W, HOLD_H = 0.096, 0.052, 0.010  # the holder, front to back (x), across (y), tall
CAKE_R, CAKE_UP, CAKE_DX = 0.025, 0.005, 0.020  # the cake: radius, how far it stands above the wood, offset back (x)
CAKE_TOP = HOLD_H + CAKE_UP
ROSIN_START = (0.224, 0.110)
ROSIN_B_Y = 0.340  # the demo slides the rosin here for the half of the hair toward the head
GRIP_DX, GRIP_W, GRIP_L, GRIP_H = -0.038, 0.012, 0.044, 0.014  # the raised grip at the holder's front end
# -- the bow: origin at the middle of the frog; it points along +y, hair down --------------------------------------------
FROG_W, FROG_L, FROG_H = 0.014, 0.046, 0.026  # the ebony frog: across (x), along (y), tall
STICK_R0, STICK_R1 = 0.0042, 0.0029  # the stick tapers from 8.4 mm at the frog to 5.8 mm at the head
STICK_R = STICK_R0
HAIR_Y0, HAIR_Y1 = 0.025, 0.685  # the hair runs 66 cm from the frog's front to the head
HAIR_Z0, HAIR_Z1 = -FROG_H / 2 + 0.001, -0.012  # the hair's middle line height at the frog and at the head
HAIR_W, HAIR_T = 0.011, 0.002
STICK_Z0, STICK_Z1 = FROG_H / 2 + STICK_R0 - 0.002, 0.010  # the stick's middle at the frog and at the head
CAMBER = 0.008  # the stick bows down toward the hair this much at its middle
HEAD_Y = 0.700  # the head's middle; the bow is 75.5 cm from the screw button to the tip
NSEG = 12  # the hair, drawn in 12 lengths that whiten one by one
SEG_L = (HAIR_Y1 - HAIR_Y0) / NSEG
BOW_START = (0.320, -0.300)  # the frog's middle, lying on the cloth
BOW_MASS = 0.060
BOW_COM_Y = 0.198  # it balances 19.8 cm in front of the frog's middle (23.5 cm from the button's end), like a real bow
# -- a full-size violin, lying on the cloth along y, fixed ---------------------------------------------------------------
VX, VY = 0.452, 0.000  # the middle of its body
# -- the rules ---------------------------------------------------------------------------------------------------------
ROSIN_RUB = 0.10  # a length of hair is coated once the rosin has passed 10 cm under it
SEGS_NEEDED = 8  # of the 12 lengths of hair
STILL_MM, STILL_DEG = 0.0015, 0.75  # rubbing counts only in steps the rosin stayed this still
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

LACQUER, LACQUER_D = (0.06, 0.06, 0.07), (0.16, 0.16, 0.18)
FELT, FELT_D = (0.10, 0.32, 0.18), (0.07, 0.24, 0.13)
WOOD, WOOD_D = (0.70, 0.52, 0.30), (0.50, 0.34, 0.18)
AMBER, AMBER_D = (0.78, 0.45, 0.08), (0.62, 0.30, 0.04)
EBONY, SILVER, PEARL, STICK = (0.05, 0.04, 0.04), (0.78, 0.79, 0.82), (0.92, 0.92, 0.88), (0.38, 0.15, 0.07)
HAIR_CLEAN, HAIR_ROSIN, IVORY = (0.70, 0.62, 0.46), (1.0, 1.0, 0.98), (0.94, 0.91, 0.82)
VARNISH, VARNISH_D, SPRUCE_D, STRINGS = (0.62, 0.26, 0.08), (0.42, 0.15, 0.04), (0.20, 0.08, 0.03), (0.80, 0.80, 0.78)


def _lacquer() -> Body:
    shapes = [BoxShape((0.0, 0.0, LAC_H / 2), (LX1 - LX0, LY1 - LY0, LAC_H), LACQUER)]
    rng = np.random.default_rng(4)
    for _ in range(8):  # grain
        shapes.append(BoxShape((rng.uniform(-0.09, 0.09), rng.uniform(-0.23, 0.23), LAC_H + 0.0002),
                               (rng.uniform(0.04, 0.12), rng.uniform(0.001, 0.003), 0.0004), LACQUER_D,
                               collide=False, yaw=rng.uniform(-0.2, 0.2)))
    return Body.build("lacquer", pos=((LX0 + LX1) / 2, (LY0 + LY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=LACQUER, shapes=shapes, friction=(0.07, 0.06), slick=True)


def _cloth() -> Body:
    shapes = [BoxShape((0.0, 0.0, CLOTH_H / 2), (CX1 - CX0, CY1 - CY0, CLOTH_H), FELT)]
    for s in (-1.0, 1.0):  # a stitched hem
        shapes.append(BoxShape(((s * (CX1 - CX0) / 2 - s * 0.008), 0.0, CLOTH_H + 0.0002),
                               (0.002, CY1 - CY0 - 0.016, 0.0004), FELT_D, collide=False))
    return Body.build("cloth", pos=((CX0 + CX1) / 2, (CY0 + CY1) / 2, TOP), mass=1.0, kinematic=True, color=FELT,
                      shapes=shapes, friction=(0.9, 0.8))


def _rosin() -> Body:
    shapes = [BoxShape((0.0, 0.0, HOLD_H / 2), (HOLD_L, HOLD_W, HOLD_H), WOOD),
              BoxShape((0.0, 0.0, HOLD_H + 0.0003), (HOLD_L - 0.006, HOLD_W - 0.006, 0.0006), WOOD_D, collide=False),
              BoxShape((GRIP_DX, 0.0, HOLD_H + GRIP_H / 2), (GRIP_W, GRIP_L, GRIP_H), WOOD),  # the raised grip
              BoxShape((GRIP_DX, 0.0, HOLD_H + GRIP_H + 0.0003), (GRIP_W - 0.004, GRIP_L - 0.006, 0.0006), WOOD_D,
                       collide=False),
              CylShape((CAKE_DX, 0.0, (HOLD_H - 0.003 + CAKE_TOP) / 2), CAKE_R, CAKE_TOP - HOLD_H + 0.003, AMBER),
              CylShape((CAKE_DX, 0.0, CAKE_TOP + 0.0002), CAKE_R - 0.006, 0.0004, AMBER_D, collide=False),
              CylShape((CAKE_DX - 0.005, 0.004, CAKE_TOP + 0.0004), 0.004, 0.0004, (0.90, 0.68, 0.30),
                       collide=False)]  # a worn, paler groove where bows have rubbed
    return Body.build("rosin", pos=(ROSIN_START[0], ROSIN_START[1], LT), mass=0.05, color=WOOD, shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, 0.0, 0.006))


SEG_CLEAN: list[int] = []
SEG_ROSIN: list[int] = []


def _hair_z(y: float) -> float:
    return HAIR_Z0 + (HAIR_Z1 - HAIR_Z0) * (y - HAIR_Y0) / (HAIR_Y1 - HAIR_Y0)


def _bow() -> Body:
    """The bow: an ebony frog with a pearl eye and silver ferrule, the screw button behind it, a round pernambuco stick
    and an ivory-faced head, and the pale horsehair between frog and head."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (FROG_W, FROG_L, FROG_H), EBONY),
              CylShape((FROG_W / 2 + 0.0003, 0.0, 0.002), 0.004, 0.0006, PEARL, collide=False, axis="X"),
              CylShape((-FROG_W / 2 - 0.0003, 0.0, 0.002), 0.004, 0.0006, PEARL, collide=False, axis="X"),
              BoxShape((0.0, FROG_L / 2 - 0.002, -FROG_H / 2 + 0.003), (FROG_W + 0.001, 0.004, 0.006), SILVER,
                       collide=False),  # the ferrule
              CylShape((0.0, -FROG_L / 2 - 0.007, STICK_Z0), 0.0045, 0.014, EBONY, axis="Y"),  # the screw button
              CylShape((0.0, -FROG_L / 2 - 0.0005, STICK_Z0), 0.0047, 0.002, SILVER, collide=False, axis="Y")]
    n = 12  # the stick, in straight lengths, tapering and cambered
    for k in range(n):
        y0 = -FROG_L / 2 + (HEAD_Y + FROG_L / 2) * k / n
        y1 = -FROG_L / 2 + (HEAD_Y + FROG_L / 2) * (k + 1) / n
        z0 = STICK_Z0 + (STICK_Z1 - STICK_Z0) * k / n - CAMBER * math.sin(math.pi * k / n)
        z1 = STICK_Z0 + (STICK_Z1 - STICK_Z0) * (k + 1) / n - CAMBER * math.sin(math.pi * (k + 1) / n)
        r = STICK_R0 + (STICK_R1 - STICK_R0) * (k + 0.5) / n
        shapes.append(plate((0.0, y0, z0), (0.0, y1, z1), 2 * r, 2 * r, (0.0, 0.0, 1.0), STICK))
    head_top = STICK_Z1 + STICK_R1
    shapes.append(BoxShape((0.0, HEAD_Y, (head_top + HAIR_Z1 - 0.002) / 2),
                           (0.009, 0.020, head_top - HAIR_Z1 + 0.002), STICK))  # the head
    shapes.append(BoxShape((0.0, HEAD_Y + 0.0105, (head_top + HAIR_Z1 - 0.002) / 2),
                           (0.009, 0.001, head_top - HAIR_Z1 + 0.002), IVORY, collide=False))
    # the hair: one collision strip, and its look in 12 lengths, each clean (shown) and coated (hidden at first)
    shapes.append(plate((0.0, HAIR_Y0, HAIR_Z0), (0.0, HAIR_Y1, HAIR_Z1), HAIR_W, HAIR_T, (0.0, 0.0, 1.0), None))
    SEG_CLEAN.clear()
    SEG_ROSIN.clear()
    for k in range(NSEG):
        y0, y1 = HAIR_Y0 + SEG_L * k, HAIR_Y0 + SEG_L * (k + 1)
        SEG_CLEAN.append(len(shapes))
        shapes.append(plate((0.0, y0, _hair_z(y0)), (0.0, y1, _hair_z(y1)), HAIR_W - 0.0004, HAIR_T - 0.0002,
                            (0.0, 0.0, 1.0), HAIR_CLEAN, collide=False))
        SEG_ROSIN.append(len(shapes))
        shapes.append(plate((0.0, y0, _hair_z(y0)), (0.0, y1, _hair_z(y1)), HAIR_W + 0.0006, HAIR_T + 0.0006,
                            (0.0, 0.0, 1.0), HAIR_ROSIN, collide=False))
    z = CT + FROG_H / 2
    return Body.build("bow", pos=(BOW_START[0], BOW_START[1], z), mass=BOW_MASS, color=STICK, shapes=shapes,
                      friction=(0.9, 0.8), com=(0.0, BOW_COM_Y, 0.006), solver_iterations=32)


def _violin() -> Body:
    """A full-size violin lying face up: its body (two bouts and a waist, 35.5 cm long), the fingerboard, pegbox and
    scroll, the bridge, the tailpiece, f-holes and strings, 59 cm long in all."""
    z0 = CT
    rib = 0.032
    shapes = [CylShape((0.0, -0.075, z0 + rib / 2), 0.1025, rib, VARNISH),  # the lower bout
              CylShape((0.0, 0.093, z0 + rib / 2), 0.083, rib, VARNISH),  # the upper bout
              BoxShape((0.0, 0.008, z0 + rib / 2), (0.110, 0.100, rib), VARNISH),  # the waist
              CylShape((0.0, -0.075, z0 + rib + 0.0003), 0.098, 0.0006, VARNISH_D, collide=False),
              CylShape((0.0, 0.093, z0 + rib + 0.0003), 0.079, 0.0006, VARNISH_D, collide=False),
              BoxShape((0.0, 0.165, z0 + rib + 0.012), (0.036, 0.270, 0.008), EBONY),  # the fingerboard
              BoxShape((0.0, 0.245, z0 + 0.016), (0.026, 0.130, 0.030), VARNISH),  # the neck
              BoxShape((0.0, 0.335, z0 + 0.016), (0.024, 0.060, 0.030), VARNISH),  # the pegbox
              CylShape((0.0, 0.378, z0 + 0.017), 0.018, 0.026, VARNISH_D, axis="X"),  # the scroll
              BoxShape((0.0, -0.118, z0 + rib + 0.004), (0.040, 0.110, 0.006), EBONY),  # the tailpiece
              BoxShape((0.0, -0.020, z0 + rib + 0.016), (0.042, 0.004, 0.032), IVORY),  # the bridge
              CylShape((0.0, -0.181, z0 + rib / 2), 0.006, 0.010, EBONY, axis="Y")]  # the end button
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * 0.034, -0.020, z0 + rib + 0.0005), (0.005, 0.075, 0.0008), SPRUCE_D,
                               collide=False))  # the f-holes
        for k in range(2):  # the pegs
            shapes.append(CylShape((s * 0.024, 0.322 + 0.026 * k, z0 + 0.020), 0.005, 0.026, EBONY, axis="X",
                                   collide=False))
    for k in range(4):  # the strings
        x = -0.012 + 0.008 * k
        shapes.append(plate((x, -0.065, z0 + rib + 0.010), (x, 0.300, z0 + rib + 0.018), 0.0009, 0.0009,
                            (0.0, 0.0, 1.0), STRINGS, collide=False))
    return Body.build("violin", pos=(VX, VY, 0.0), mass=0.45, kinematic=True, color=VARNISH, shapes=shapes,
                      friction=(0.8, 0.7))


OBJECTS = [_lacquer(), _cloth(), _violin(), _rosin(), _bow()]

# the bow's prim path, noted as it is spawned (to show the hair's coat as it builds up)
_PATHS: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "bow":
        _PATHS["bow"] = prim_path
        for k in range(NSEG):
            _show_seg(k, False, stage)


bodykit._author_sliders = _author_sliders


def _show_seg(k: int, coated: bool, stage=None) -> None:
    from pxr import UsdGeom

    if "bow" not in _PATHS:
        return
    if stage is None:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
    for i, on in ((SEG_CLEAN[k], not coated), (SEG_ROSIN[k], coated)):
        prim = stage.GetPrimAtPath(f"{_PATHS['bow']}/body/s{i}")
        if prim:
            img = UsdGeom.Imageable(prim)
            img.MakeVisible() if on else img.MakeInvisible()


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"rub": [0.0] * NSEG, "coated": [False] * NSEG, "prev_rosin": None, "prev_yc": None, "tcps": None,
            "s": {}, "shown_any": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ROSIN_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    shown = _EP.get("shown_any")
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if shown:
        for k in range(NSEG):
            _show_seg(k, False)


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


def _on_cloth(p) -> bool:
    return CX0 + 0.005 < p[0] < CX1 - 0.005 and CY0 + 0.005 < p[1] < CY1 - 0.005


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "bow")
    pr, rr = _pose(base, origin, "rosin")
    cake = pr + rr @ np.array([CAKE_DX, 0.0, CAKE_TOP])
    c = rb.T @ (cake - pb)  # the cake's top middle, in the bow's frame
    prev = _EP["prev_rosin"]
    still = False
    if prev is not None:
        dp = float(np.linalg.norm(pr - prev[0]))
        dr = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(prev[1].T @ rr) - 1) / 2))))
        still = dp < STILL_MM and dr < STILL_DEG
    yc = float(c[1])
    on_hair = HAIR_Y0 - 0.005 < yc < HAIR_Y1 + 0.005 and abs(c[0]) < CAKE_R + HAIR_W / 2
    gap = float(c[2] - (_hair_z(min(max(yc, HAIR_Y0), HAIR_Y1)) - HAIR_T / 2))
    touching = on_hair and -0.0025 < gap < 0.003 and float(rb[2, 2]) > math.cos(math.radians(25.0))
    if still and touching and _EP["prev_yc"] is not None:
        step = abs(yc - _EP["prev_yc"])
        if 0.0003 < step < 0.03:
            for k in range(NSEG):
                y0, y1 = HAIR_Y0 + SEG_L * k, HAIR_Y0 + SEG_L * (k + 1)
                if y0 - CAKE_R * 0.6 < yc < y1 + CAKE_R * 0.6 and not _EP["coated"][k]:
                    _EP["rub"][k] += step
                    if _EP["rub"][k] >= ROSIN_RUB:
                        _EP["coated"][k] = True
                        _EP["shown_any"] = True
                        _show_seg(k, True)
    _EP["prev_rosin"] = (pr.copy(), rr.copy())
    _EP["prev_yc"] = yc if touching else None
    _EP["tcps"] = _tcps(base, origin)
    frog = pb
    head = pb + rb @ np.array([0.0, HEAD_Y, 0.0])
    laid = (_on_cloth(frog) and _on_cloth(head) and frog[2] < CT + FROG_H / 2 + 0.006 and head[2] < CT + 0.03
            and abs(float(rb[2, 2])) > math.cos(math.radians(20.0)))
    _EP["s"] = {"coated": sum(_EP["coated"]), "laid": laid, "home": _arms_home()}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[x].data.joint_pos)[0][base.scene[x].joint_names.index("joint7")]) * 1000
                  for x in ("left_robot", "right_robot")]
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(rb[2, 1])))))
            yaw = math.degrees(math.atan2(-rb[0, 1], rb[1, 1]))
            print(f"[rosin] t={_DEBUG['steps'] / 20:5.1f}s rosin={mm(pr - [ROSIN_START[0], ROSIN_START[1], LT])}"
                  f" still={still} bow={mm(pb)} pitch={pitch:.1f} yaw={yaw:.1f} c={mm(c)} gap={gap * 1000:.1f}"
                  f" touch={touching} rub={[round(r * 100) for r in _EP['rub']]} coated={sum(_EP['coated'])}"
                  f" laid={laid} home={_EP['s']['home']} fingers={[round(v, 1) for v in fj]} L={mm(tl)} R={mm(tr)}"
                  f" score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: at least 8 of the hair's 12 lengths coated (each once the rosin, held still, has passed 10 cm under it), and
    at the end the bow lying on the cloth and both arms back where they started; 0.75: 8 coated and the bow lying on
    the cloth at the end; 0.5: 8 coated; 0.25: at least 3 coated; else 0.0."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["coated"] >= SEGS_NEEDED:
        if s["laid"]:
            return 1.0 if s["home"] else 0.75
        return 0.5
    return 0.25 if s["coated"] >= 3 else 0.0


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
    """Full score with the bow and the rosin at rest: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("bow", "rosin"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left holds the rosin holder by its grip. The right picks the bow up by its frog, carries it over the rosin and
# lowers the hair onto the cake, then draws it back and forth three times (the frog 16 to 45 cm right of the cake). It
# holds the bow back while the left slides the rosin 23 cm to the left, then draws four more times each way (the frog 45
# to 64 cm right of the cake), lifts it and lays it back on the cloth. The left lets go; both arms go home.

FROG_FINGER = -0.012
FROG_GRIP = 0.006  # the TCP this far below the frog's top
TIP = 0.013  # closed fingertips reach this far below the TCP
GRIP_FINGER = -0.013  # pre-open round the holder's grip
GRIP_TCP = 0.004  # the TCP this far above the grip's top (pinched about 9 mm deep)
BOW_PRESS = 0.002  # the hair is lowered this far into the cake
STROKE = (-0.050, -0.340)  # the frog's y at the two ends of a stroke, the rosin where it starts (y 0.110)
STROKE_B = (-0.110, -0.300)  # and with the rosin slid to y 0.340 (the half of the hair toward the head)
CARRY = 0.075  # the frog's middle this high above the lacquer while carrying
LEAD_S, LEAD_STEPS = 3.0, 15
OFF = np.zeros(3)  # measured grip offset of the frog (TCP minus frog), aimed against
STROKE_YAW = -2.5  # degrees: the bow turns about 2.5 degrees in the grip on strokes toward the frog; turn against it


def _rot(arm: str) -> np.ndarray:
    return grip_rotation(math.pi / 2) if arm == "right" else grip_rotation(-math.pi / 2)  # both close along x


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5), (0.0,), (-0.4, 0.4)):
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
    return _demo_plan()


def _demo_plan():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, n=1, rot=None):
        rot = _rot(arm) if rot is None else rot
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        p, f, r = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, finger, label):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, _rot(arm)), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, _rot(arm))

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        last = plan[arm][-1]
        cur[arm] = (np.asarray(last.pos, float), f, last.yaw)

    grip_dz = FROG_H / 2 - FROG_GRIP  # the TCP above the frog's middle
    frog0 = np.array([BOW_START[0], BOW_START[1], CT + FROG_H / 2 + grip_dz]) + OFF
    cake_x = ROSIN_START[0] + CAKE_DX
    # the frog's middle is -HAIR_Z0 + HAIR_T/2 above the hair's underside near the frog
    rub_z = LT + CAKE_TOP - HAIR_Z0 + HAIR_T / 2 + grip_dz - BOW_PRESS
    near = np.array([cake_x, STROKE[0], rub_z]) + OFF
    far = np.array([cake_x, STROKE[1], rub_z]) + OFF
    near_b = np.array([cake_x, STROKE_B[0], rub_z]) + OFF
    far_b = np.array([cake_x, STROKE_B[1], rub_z]) + OFF
    carry = LT + CARRY + grip_dz
    clear = np.array([0.310, STROKE_B[0], carry])  # the bow held back over the cloth's edge while the rosin moves
    grip = np.array([ROSIN_START[0] + GRIP_DX, ROSIN_START[1], LT + HOLD_H + GRIP_H + GRIP_TCP])
    rl = grip_rotation(-math.pi / 2)  # the left's fingers close along x, across the grip

    # 1. the left takes hold of the rosin holder's grip; the right picks up the bow by its frog
    from_home("left", grip + [0.0, 0.0, 0.05], GRIP_FINGER, "over the rosin holder's grip")
    wait("left", 0.4, "settle over it")
    go("left", 1.0, grip, GRIP_FINGER, "down round the grip")
    go("left", 0.6, grip, FINGER_CLOSED, "hold the rosin still")
    from_home("right", frog0 + [0.0, 0.0, 0.05], FROG_FINGER, "over the bow's frog")
    go("right", 1.0, frog0, FROG_FINGER, "down round the frog")
    go("right", 0.6, frog0, FINGER_CLOSED, "grip the frog")
    go("right", 1.0, [frog0[0], frog0[1], carry], FINGER_CLOSED, "lift the bow")
    go("right", 2.0, [near[0], near[1], carry], FINGER_CLOSED, "carry it over the rosin", n=4)
    sync("wait: the rosin held")
    # 2. hair down on the cake; three strokes each way along the hair toward the frog
    c, sn = math.cos(math.radians(STROKE_YAW)), math.sin(math.radians(STROKE_YAW))
    rs = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]]) @ _rot("right")  # the grip while rubbing
    go("right", 1.2, near, FINGER_CLOSED, "lower the hair onto the rosin")
    for k in range(3):
        go("right", 3.0, far, FINGER_CLOSED, f"draw the bow toward its tip across the rosin ({k + 1})", n=6)
        go("right", 3.0, near, FINGER_CLOSED, f"and back toward the frog ({k + 1})", n=6, rot=rs)
    go("right", 0.8, [near[0], near[1], carry], FINGER_CLOSED, "lift the bow off the rosin")
    go("right", 1.2, clear, FINGER_CLOSED, "hold it back, clear of the other hand", n=2)
    # 3. the left slides the rosin along the table to the left, under the half of the hair toward the head
    wait("left", dur("right") - dur("left"), "hold the rosin still")
    grip_b = grip + [0.0, ROSIN_B_Y - ROSIN_START[1], 0.0]
    go("left", 3.0, grip_b, FINGER_CLOSED, "slide the rosin to the left", n=6)
    wait("left", 0.4, "hold it there")
    sync("wait: the rosin moved")
    go("right", 1.2, [near_b[0], near_b[1], carry], FINGER_CLOSED, "the bow back over the rosin", n=2)
    go("right", 1.2, near_b, FINGER_CLOSED, "lower the hair onto the rosin")
    for k in range(4):
        go("right", 2.4, far_b, FINGER_CLOSED, f"draw the bow toward its tip across the rosin ({k + 1})", n=5)
        go("right", 2.4, near_b, FINGER_CLOSED, f"and back toward the frog ({k + 1})", n=5, rot=rs)
    go("right", 0.8, [near_b[0], near_b[1], carry], FINGER_CLOSED, "lift the bow off the rosin")
    wait("left", dur("right") - dur("left"), "hold the rosin still")
    go("left", 0.5, grip_b, GRIP_FINGER, "let go of the holder")
    go("left", 0.8, grip_b + [0.0, 0.0, 0.05], GRIP_FINGER, "rise")
    # 4. the bow back on the cloth; both arms home
    go("right", 2.0, [frog0[0], frog0[1], carry], FINGER_CLOSED, "carry the bow back over the cloth", n=4)
    go("right", 1.2, frog0 + [0.0, 0.0, 0.004], FINGER_CLOSED, "lay it down on the cloth")
    go("right", 0.5, frog0 + [0.0, 0.0, 0.004], FROG_FINGER, "let go")
    go("right", 0.6, frog0 + [0.0, 0.0, 0.05], FROG_FINGER, "rise")
    to_home("left")
    to_home("right")
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
