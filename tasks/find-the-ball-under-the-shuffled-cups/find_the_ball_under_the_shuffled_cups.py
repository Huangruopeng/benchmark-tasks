"""On a green felt mat stand three identical copper cups mouth down on three white rings in a row, left, middle and
right; a little felt ball seat sits in the middle of each ring. A red ball lies in a small white dish at the front of
the mat, in the middle; a start button with a lamp stands at its front left.

Task: play the cups-and-ball game. First hide the ball under the middle cup: lift the middle cup with one hand and hold
it up while the other hand takes the ball from the dish and puts it on the middle ring, then lower the cup over it. Press
the start button and keep both arms back where they started while the cups are shuffled: the lamp blinks amber, then the
cups glide around and swap places, and the lamp turns green when they stop. Then lift the cup the ball is under (the
first cup lifted after the shuffle must be that one), hold it up while the other hand takes the ball out, set the cup
back down on its ring, put the ball back in the dish and take both arms back to where they started.

What it tests: memory. Once the cups have stopped they look exactly alike; only a policy that kept track of the ball's
cup through the swaps knows which one to lift (one that sees only the present frame is right one time in three).
The swaps depend on the trial's seed.

Why two hands: a cup held up over its ring covers the ring from above, so the hand holding it cannot also reach the
ball; the instruction makes one hand hold the cup up while the other puts the ball in or takes it out (checked every
step).

Run it:  inspect-robots run --embodiment yam_arena -E task=find_the_ball_under_the_shuffled_cups.py \\
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
                     on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the felt mat -----------------------------------------------------------------------------------------------------
MX0, MX1, MY0, MY1, MAT_H = 0.100, 0.400, -0.225, 0.225, 0.006
MT = TOP + MAT_H
# -- the cups: origin at the middle of the mouth (on the mat) ---------------------------------------------------------
SPOT_X = 0.260
SPOT_Y = (0.100, 0.0, -0.100)  # left, middle, right
R_MOUTH, R_FOOT, CONE_H, CUP_H, WALL = 0.040, 0.027, 0.040, 0.060, 0.003
N_WALL = 28
# -- the ball and its dish ---------------------------------------------------------------------------------------------
R_BALL = 0.0095
SEAT_R, SEAT_H = 0.0105, 0.003  # the felt nubs of each ring's ball seat: this far from its middle, this tall
DX, DY = 0.155, 0.0
R_DISH, DISH_FLOOR, DISH_H = 0.040, 0.004, 0.014
# -- the start button and its lamp ------------------------------------------------------------------------------------
BX, BY = 0.158, 0.170
BASE_W, BASE_H, BTN_R, BTN_H = 0.070, 0.030, 0.017, 0.012
BTN_DX, BTN_GAP, BTN_TRAVEL = 0.012, 0.009, 0.008  # the cap rides 9 mm above the base and goes down 8 mm
BTN_TOP = BASE_H + BTN_GAP + BTN_H
PRESSED_BY = 0.004  # the cap pushed down this far: pressed
LAMP_DX = -0.022  # the lamp, on the base's front half
# -- the shuffle (a rule: the mat's hidden shuffler glides the cups) -------------------------------------------------
COUNTDOWN_S = 5.0  # after the button is pressed: the lamp blinks this long, then the cups start moving
N_SWAPS, SWAP_S, GAP_S = 5, 1.4, 0.35  # each swap: two neighbouring cups trade places
SIDE_BACK, SIDE_FRONT = 0.060, 0.035  # one passes this far behind the row, the other this far in front of it
DEMO_SEED = 1415158438  # the reset seed inspect-robots uses for --seed 0 (its default): the demo's trial
SWAP_SALT = 0
# -- the rules ---------------------------------------------------------------------------------------------------------
LIFTED_Z = 0.010  # a cup is lifted once its mouth is this far above the mat (or it tips over 15 degrees)
HOME_WITHIN = 0.05
AWAY_DURING_SHUFFLE = 0.08  # a gripper farther than this from where it started while the cups move: latched 0
ON_SPOT = 0.025
SUCCESS_HOLD_S = 1.0

FELT, FELT_D = (0.07, 0.34, 0.15), (0.05, 0.27, 0.12)
GOLD = (0.83, 0.68, 0.30)
RING = (0.93, 0.93, 0.90)
COPPER, COPPER_D, BRASS = (0.74, 0.40, 0.22), (0.55, 0.28, 0.15), (0.86, 0.70, 0.36)
RED_BALL = (0.80, 0.08, 0.10)
PORCELAIN = (0.95, 0.95, 0.93)
BLACK, RED_BTN = (0.08, 0.08, 0.09), (0.82, 0.10, 0.10)
LAMP_OFF, AMBER, GREEN = (0.22, 0.20, 0.16), (1.00, 0.62, 0.05), (0.15, 0.85, 0.25)


def _mat() -> Body:
    w, d = MX1 - MX0, MY1 - MY0
    shapes = [BoxShape((0.0, 0.0, MAT_H / 2), (w, d, MAT_H), FELT)]
    for s in (-1.0, 1.0):  # a gold border
        shapes.append(BoxShape((s * (w / 2 - 0.012), 0.0, MAT_H + 0.0002), (0.004, d - 0.020, 0.0004), GOLD,
                               collide=False))
        shapes.append(BoxShape((0.0, s * (d / 2 - 0.012), MAT_H + 0.0002), (w - 0.020, 0.004, 0.0004), GOLD,
                               collide=False))
    cx = (MX0 + MX1) / 2
    for y in SPOT_Y:  # the three white rings, each with a little felt ball seat in its middle (eight nubs)
        shapes.append(CylShape((SPOT_X - cx, y, MAT_H + 0.0002), R_MOUTH + 0.006, 0.0004, RING, collide=False))
        shapes.append(CylShape((SPOT_X - cx, y, MAT_H + 0.0004), R_MOUTH + 0.001, 0.0004, FELT_D, collide=False))
        for k in range(8):
            a = 2 * math.pi * k / 8
            shapes.append(CylShape((SPOT_X - cx + SEAT_R * math.cos(a), y + SEAT_R * math.sin(a), MAT_H + SEAT_H / 2),
                                   0.0025, SEAT_H, FELT_D))
    return Body.build("mat", pos=(cx, (MY0 + MY1) / 2, TOP), mass=1.0, kinematic=True, color=FELT, shapes=shapes,
                      friction=(0.9, 0.8))


def _dish() -> Body:
    """A small white porcelain dish: a floor and a flared wall of 16 panels."""
    shapes = [CylShape((0.0, 0.0, DISH_FLOOR / 2), R_DISH - 0.006, DISH_FLOOR, PORCELAIN)]
    n = 16
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        shapes.append(plate((c * (R_DISH - 0.012), s * (R_DISH - 0.012), DISH_FLOOR * 0.5),
                            (c * R_DISH, s * R_DISH, DISH_H), 2 * math.pi * R_DISH / n * 1.15, 0.004, (c, s, 0.0),
                            PORCELAIN))
    shapes.append(CylShape((0.0, 0.0, DISH_FLOOR + 0.0002), R_DISH - 0.012, 0.0004, (0.30, 0.42, 0.70),
                           collide=False))  # a blue band on its floor
    shapes.append(CylShape((0.0, 0.0, DISH_FLOOR + 0.0004), R_DISH - 0.016, 0.0004, PORCELAIN, collide=False))
    return Body.build("dish", pos=(DX, DY, MT), mass=0.2, kinematic=True, color=PORCELAIN, shapes=shapes,
                      friction=(0.8, 0.7))


_SHAPE: dict = {}


def _console() -> Body:
    """A black base with a lamp in front of a chrome collar, in which the red start button (its own body) rides."""
    shapes = [BoxShape((0.0, 0.0, BASE_H / 2), (BASE_W, BASE_W, BASE_H), BLACK),
              CylShape((BTN_DX, 0.0, BASE_H + 0.002), BTN_R + 0.005, 0.004, (0.55, 0.56, 0.58), collide=False)]
    for name, color in (("lamp_off", LAMP_OFF), ("lamp_amber", AMBER), ("lamp_green", GREEN)):
        _SHAPE[name] = len(shapes)
        shapes.append(CylShape((LAMP_DX, 0.0, BASE_H + 0.004), 0.0085, 0.008, color, collide=False))
    shapes.append(CylShape((LAMP_DX, 0.0, BASE_H + 0.001), 0.011, 0.002, (0.55, 0.56, 0.58), collide=False))
    return Body.build("console", pos=(BX, BY, MT), mass=0.5, kinematic=True, color=BLACK, shapes=shapes,
                      friction=(0.8, 0.7))


def _button() -> Body:
    """The red start button: a cap on a spring, 8 mm of travel (a slider to the console)."""
    shapes = [CylShape((0.0, 0.0, BTN_H / 2), BTN_R, BTN_H, RED_BTN),
              CylShape((0.0, 0.0, BTN_H + 0.0002), BTN_R - 0.004, 0.0004, (0.92, 0.25, 0.22), collide=False)]
    return Body.build("button", pos=(BX + BTN_DX, BY, MT + BASE_H + BTN_GAP), mass=0.02, color=RED_BTN,
                      shapes=shapes, friction=(0.8, 0.7), ignore=("console",))


bodykit.slider("button_spring", "console", "button", (BX + BTN_DX, BY, MT + BASE_H + BTN_GAP), axis="Z",
               limits=(-BTN_TRAVEL, 0.0), stiffness=250.0, damping=3.0, target=0.0)


def _cup(i: int) -> Body:
    """A copper cup of a cups-and-balls set, mouth down: a cone of 24 wall panels, a straight foot band on top with
    brass beads, and its closed bottom (now the top)."""
    shapes = []
    for k in range(N_WALL):
        a = 2 * math.pi * k / N_WALL
        c, s = math.cos(a), math.sin(a)
        r0, r1 = R_MOUTH - WALL / 2, R_FOOT - WALL / 2
        shapes.append(plate((c * r0, s * r0, 0.0015), (c * r1, s * r1, CONE_H), 2 * math.pi * R_MOUTH / N_WALL * 1.12,
                            WALL, (c, s, 0.0), COPPER))
        shapes.append(plate((c * r1, s * r1, CONE_H), (c * r1, s * r1, CUP_H - 0.004),
                            2 * math.pi * R_FOOT / N_WALL * 1.12, WALL, (c, s, 0.0), COPPER))
    shapes.append(CylShape((0.0, 0.0, CUP_H - 0.002), R_FOOT, 0.004, COPPER))  # the closed end
    shapes.append(CylShape((0.0, 0.0, CUP_H + 0.0002), R_FOOT - 0.006, 0.0004, COPPER_D, collide=False))
    for z, r in ((0.0035, R_MOUTH + 0.0005), (CONE_H, R_FOOT + 0.0012), (CUP_H - 0.006, R_FOOT + 0.0012)):
        shapes.append(CylShape((0.0, 0.0, z), r, 0.003, BRASS, collide=False))  # brass beads
    return Body.build(f"cup{i}", pos=(SPOT_X, SPOT_Y[i], MT), mass=0.09, color=COPPER, shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, 0.0, 0.040), damping=(0.0, 0.3))


def _ball() -> Body:
    return Body.build("ball", pos=(DX, DY, MT + DISH_FLOOR + R_BALL), mass=0.012, color=RED_BALL,
                      shapes=[SphereShape((0.0, 0.0, 0.0), R_BALL, RED_BALL)], friction=(1.0, 0.9),
                      damping=(0.3, 2.0))


_PATHS: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: note where the lamp's colours are drawn, and show it off."""
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "console":
        from pxr import UsdGeom

        for name in ("lamp_off", "lamp_amber", "lamp_green"):
            _PATHS[name] = f"{prim_path}/body/s{_SHAPE[name]}"
            if name != "lamp_off":
                UsdGeom.Imageable(stage.GetPrimAtPath(_PATHS[name])).MakeInvisible()


bodykit._author_sliders = _author_sliders

OBJECTS = [_mat(), _dish(), _console(), _button(), _cup(0), _cup(1), _cup(2), _ball()]
CUPS = ("cup0", "cup1", "cup2")


# -- the trial's seed (the swaps depend on it) --------------------------------------------------------------------------

def _catch_seed() -> None:
    """Wrap the embodiment's reset so the task file sees each trial's seed (stored on bodykit, shared by reloads)."""
    mod = sys.modules.get("yam_arena.inspect_embodiment")
    if mod is None:
        plain = bodykit._TARGETS["yam_arena.inspect_embodiment"]

        def patch(module):
            plain(module)
            _wrap(module)

        bodykit._TARGETS["yam_arena.inspect_embodiment"] = patch
    else:
        _wrap(mod)


def _wrap(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_shell_seed", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        bodykit._TRIAL_SEED = kwargs.get("seed", args[1] if len(args) > 1 else None)
        return plain(self, *args, **kwargs)

    cls.reset = reset
    cls._shell_seed = True


_catch_seed()


def swaps_for(seed) -> list[tuple[int, int, int]]:
    """The trial's swaps: N_SWAPS trades between neighbouring places (0 left, 1 middle, 2 right), as (a, b, back):
    the cup from place ``a`` passes behind the row if ``back`` is 1, in front of it if 0. The place the ball's cup ends
    on is drawn first, evenly from the three, and the ball's cup takes part in at least two swaps."""
    rng = np.random.default_rng([SWAP_SALT, DEMO_SEED if seed is None else int(seed)])
    target = int(rng.integers(3))
    while True:
        out = [((0, 1), (1, 2))[int(rng.integers(2))] + (int(rng.integers(2)),) for _ in range(N_SWAPS)]
        place, moves = 1, 0
        for a, b, _back in out:
            if place in (a, b):
                moves += 1
                place = b if place == a else a
        if place == target and moves >= 2:
            return out


def final_place(seed, start: int = 1) -> int:
    """Where a cup starting at place ``start`` ends after the trial's swaps."""
    p = start
    for a, b, _back in swaps_for(seed):
        p = b if p == a else a if p == b else p
    return p


def _spot(i: int) -> np.ndarray:
    return np.array([SPOT_X, SPOT_Y[i], MT])


def _shuffle_pose(t: float, swaps, order) -> dict:
    """Where each cup is ``t`` s into the shuffle: {cup name: (x, y)}; ``order`` maps places to cups at the start."""
    places = list(order)
    for k, (a, b, back) in enumerate(swaps):
        t0 = k * (SWAP_S + GAP_S)
        if t < t0:
            break
        if t >= t0 + SWAP_S:
            places[a], places[b] = places[b], places[a]
            continue
        s = (t - t0) / SWAP_S
        s = (1 - math.cos(math.pi * s)) / 2
        out = {places[j]: (SPOT_X, SPOT_Y[j]) for j in range(3)}
        ya, yb = SPOT_Y[a], SPOT_Y[b]
        cy = (ya + yb) / 2
        ang = math.pi * s
        da, db = (SIDE_BACK, -SIDE_FRONT) if back else (-SIDE_FRONT, SIDE_BACK)
        out[places[a]] = (SPOT_X + da * math.sin(ang), cy + (ya - cy) * math.cos(ang))
        out[places[b]] = (SPOT_X + db * math.sin(ang), cy + (yb - cy) * math.cos(ang))
        return out
    return {places[j]: (SPOT_X, SPOT_Y[j]) for j in range(3)}


SHUFFLE_S = N_SWAPS * SWAP_S + (N_SWAPS - 1) * GAP_S


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"t": 0.0, "pressed_at": None, "phase": "hide", "hidden": False, "hide_bad": False, "placed": False,
            "away": False, "ball_cup": None, "order": None, "offset": None, "first_lift": None, "took_ok": False,
            "take_bad": False, "lifted": {c: False for c in CUPS}, "tcps": None, "s": {}, "lamp": "lamp_off"}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SHELL_DEBUG")), "steps": 0}


def _show(path: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(path)
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


def _lamp(name: str) -> None:
    if name == _EP["lamp"] or "lamp_off" not in _PATHS:
        _EP["lamp"] = name
        return
    _show(_PATHS[_EP["lamp"]], False)
    _show(_PATHS[name], True)
    _EP["lamp"] = name


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _EP["swaps"] = swaps_for(getattr(bodykit, "_TRIAL_SEED", None))
    _DEBUG["steps"] = 0
    for name in ("lamp_amber", "lamp_green"):
        if name in _PATHS:
            _show(_PATHS[name], False)
    if "lamp_off" in _PATHS:
        _show(_PATHS["lamp_off"], True)


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


def _put(base, origin, name, pos) -> None:
    """Move a body to ``pos``, upright and at rest."""
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _is_lifted(p, r) -> bool:
    return p[2] - MT > LIFTED_Z or _tilt(r) > 15.0


def _under(pb, pc) -> bool:
    """The ball is under this cup (inside its mouth, low on the mat)."""
    return math.hypot(pb[0] - pc[0], pb[1] - pc[1]) < R_MOUTH - 0.004 and pb[2] < MT + 2.5 * R_BALL


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    dt = base.step_dt
    _EP["t"] += dt
    t = _EP["t"]
    cups = {c: _pose(base, origin, c) for c in CUPS}
    pb, _rb = _pose(base, origin, "ball")
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    home = _home_tcps()
    lifted_now = {c: _is_lifted(*cups[c]) for c in CUPS}
    phase = _EP["phase"]

    if phase == "hide":
        # the ball arriving on the middle ring: the middle cup must be held up at that moment
        on_ring = (math.hypot(pb[0] - SPOT_X, pb[1] - SPOT_Y[1]) < R_MOUTH - 0.008 and pb[2] < MT + 1.6 * R_BALL)
        if on_ring and not _EP["placed"]:
            _EP["placed"] = True
            if not lifted_now["cup1"]:
                _EP["hide_bad"] = True
        if not on_ring:
            _EP["placed"] = False
        pbtn, _r = _pose(base, origin, "button")
        if _EP["pressed_at"] is None and (MT + BASE_H + BTN_GAP) - pbtn[2] > PRESSED_BY:
            _EP["pressed_at"] = t
            _EP["phase"] = phase = "countdown"
    if phase == "countdown":
        _lamp("lamp_amber" if int((t - _EP["pressed_at"]) * 4) % 2 == 0 else "lamp_off")
        if t - _EP["pressed_at"] >= COUNTDOWN_S:
            # the shuffle starts: which cup stands on which ring, and is the ball under the middle one?
            order = []
            for j in range(3):
                near = min(CUPS, key=lambda c: math.hypot(cups[c][0][0] - SPOT_X, cups[c][0][1] - SPOT_Y[j]))
                order.append(near)
            ok_layout = len(set(order)) == 3 and all(
                math.hypot(cups[order[j]][0][0] - SPOT_X, cups[order[j]][0][1] - SPOT_Y[j]) < 0.03
                and not lifted_now[order[j]] for j in range(3))
            _EP["order"] = order if ok_layout else list(CUPS)
            mid = _EP["order"][1]
            _EP["hidden"] = ok_layout and _under(pb, cups[mid][0]) and not _EP["hide_bad"]
            _EP["ball_cup"] = next((c for c in CUPS if _under(pb, cups[c][0])), None)
            if _EP["ball_cup"] is not None:  # the shuffler centres the ball under its cup (out of sight)
                _EP["offset"] = np.zeros(3)
                pb = cups[_EP["ball_cup"]][0] + [0.0, 0.0, R_BALL]
            _EP["start_z"] = {c: float(cups[c][0][2]) for c in CUPS}
            _EP["ball_z"] = MT + R_BALL + 0.0005
            _EP["phase"] = phase = "shuffle"
            _EP["shuffle_t0"] = t
            _lamp("lamp_amber")
    if phase == "shuffle":
        ts = t - _EP["shuffle_t0"]
        if any(float(np.linalg.norm(p - h)) > AWAY_DURING_SHUFFLE for p, h in zip(tcps, home)):
            _EP["away"] = True
        where = _shuffle_pose(min(ts + dt, SHUFFLE_S), _EP["swaps"], _EP["order"])
        for c in CUPS:
            x, y = where[c]
            _put(base, origin, c, (x, y, _EP["start_z"][c]))
        if _EP["ball_cup"] is not None:
            x, y = where[_EP["ball_cup"]]
            _put(base, origin, "ball", (x + _EP["offset"][0], y + _EP["offset"][1], _EP["ball_z"]))
        if ts >= SHUFFLE_S:
            _EP["phase"] = phase = "find"
            _lamp("lamp_green")
    elif phase == "find":
        bc = _EP["ball_cup"]
        if "ball_spot" not in _EP:
            _EP["ball_spot"] = pb.copy()
        for c in CUPS:
            if lifted_now[c] and not _EP["lifted"][c]:
                _EP["lifted"][c] = True
                if _EP["first_lift"] is None:
                    _EP["first_lift"] = c
        if bc is not None and not _EP["took_ok"] and not _EP["take_bad"]:
            sp = _EP["ball_spot"]
            if math.hypot(pb[0] - sp[0], pb[1] - sp[1]) > R_MOUTH - 0.004 or pb[2] > MT + 2.5 * R_BALL:
                if lifted_now[bc]:
                    _EP["took_ok"] = True
                else:
                    _EP["take_bad"] = True

    # the end state
    in_dish = math.hypot(pb[0] - DX, pb[1] - DY) < R_DISH - 0.008 and pb[2] < MT + 0.035
    taken = []
    for c in CUPS:
        p, r = cups[c]
        j = min(range(3), key=lambda k: math.hypot(p[0] - SPOT_X, p[1] - SPOT_Y[k]))
        if math.hypot(p[0] - SPOT_X, p[1] - SPOT_Y[j]) < ON_SPOT and p[2] - MT < 0.004 and _tilt(r) < 8.0:
            taken.append(j)
    _EP["s"] = {"in_dish": in_dish, "cups_down": sorted(taken) == [0, 1, 2]}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, MT]) * 1000).astype(int).tolist()  # noqa: E731
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            print(f"[shell] t={t:5.1f}s seed={getattr(bodykit, '_TRIAL_SEED', None)} swaps={_EP['swaps']} "
                  f"phase={_EP['phase']} fingers={[round(v, 1) for v in fj]} ball={mm(pb)} "
                  f"cups={[mm(cups[c][0]) for c in CUPS]} lifted={[int(lifted_now[c]) for c in CUPS]} "
                  f"hidden={_EP['hidden']} hide_bad={_EP['hide_bad']} away={_EP['away']} "
                  f"ball_cup={_EP['ball_cup']} first={_EP['first_lift']} took={_EP['took_ok']} "
                  f"take_bad={_EP['take_bad']} dish={in_dish} cups_down={_EP['s']['cups_down']} "
                  f"L={mm(tcps[0])} R={mm(tcps[1])} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the ball in the dish, all three cups mouth down on the three rings, both arms back where they started;
    0.75: the ball in the dish (taken out while its cup was held up); 0.5: at some point the first cup lifted after
    the shuffle was the ball's cup; 0.25: at some point the shuffle ran with the ball hidden under the middle cup and
    both arms at home; else 0.0. 0.0 whatever else if the ball was put on the middle ring while the middle cup was not
    held up, if a gripper left home while the cups moved, if the first cup lifted after the shuffle was not the ball's
    cup, or if the ball came out from under its cup while that cup was not held up."""
    if (_EP["hide_bad"] or _EP["away"] or _EP["take_bad"]
            or (_EP["first_lift"] is not None and _EP["first_lift"] != _EP["ball_cup"])):
        return 0.0
    if not _EP["hidden"] or _EP["phase"] != "find":
        return 0.0
    s = _EP["s"]
    if s.get("in_dish") and _EP["took_ok"] and _EP["first_lift"] == _EP["ball_cup"]:
        return 1.0 if s["cups_down"] and _arms_home() else 0.75
    if _EP["first_lift"] == _EP["ball_cup"]:
        return 0.5
    return 0.25


_HOME: dict = {}


def _home_tcps():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return _HOME["tcps"]


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _home_tcps()))


def success(objects) -> bool:
    """Full score with the cups and the ball at rest: the trial ends there."""
    return (score(objects) >= 1.0 and objects["ball"].resting()
            and all(objects[c].resting() for c in CUPS))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Written for the demo's trial (DEMO_SEED, the reset seed of inspect-robots' default --seed 0), whose swaps end with the
# ball's cup on the ring given by final_place(DEMO_SEED). The left lifts the middle cup and holds it up in front of its
# ring; the right takes the ball from the dish and sets it on the ring; the left lowers the cup over it, presses the
# start button and both arms go home and wait through the shuffle. Then the arm on the ball's side (the left for the
# left or middle ring, the right for the right ring) lifts that cup and holds it up in front of its ring, the other hand
# takes the ball out, the cup goes back down and the ball goes into the dish; both arms go home.

CUP_FINGER = -0.036
GRIP_Z = CUP_H - 0.011  # the TCP this far above the mouth: pinching the foot band
# Closing on a cup's foot band shoves the cup about 1.9 cm along the closing axis (toward the robot), and opening the
# fingers shoves it back as far, every time (measured): so a cup is taken and put back with the TCP over its ring, and
# its wide mouth keeps the shoved wall clear of the ball under it.
PUT_DX = {"left": 0.0, "right": 0.0}
PARK = {0: (0.265, 0.280), 1: (0.265, 0.280), 2: (0.265, -0.280)}  # a held-up cup waits out past the mat, clear of the other arm
PARK_UP = {0: 0.030, 1: 0.072, 2: 0.030}  # ...its mouth this high (the middle one passes over the left cup)
BALL_FINGER = -0.022
LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.050


def _rot(what: str, arm: str = "left") -> np.ndarray:
    """Straight down, closing front to back (each arm turns its wrist its own way) for a cup or the ball, whose
    neighbours are to the sides; side to side for the start button."""
    if what in ("cup", "ball"):
        return np.asarray(grip_rotation(-math.pi / 2 if arm == "left" else math.pi / 2))
    return np.asarray(grip_rotation(0.0))


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


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = np.array([0.0, 0.0, OVER])
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, np.asarray(r, float))

    def go(arm, t, pos, finger, label, n=1):
        p0, _f0, r = cur[arm]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r))
        cur[arm] = (p1, finger, r)

    def wait(arm, t, label="wait", finger=None):
        p, f, r = cur[arm]
        f = f if finger is None else finger
        if t > 1e-6:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))
        cur[arm] = (p, f, r)

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def leave_home(arm, pos, finger, label, what):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, _rot(what, arm)), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, _rot(what, arm))

    def go_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        last = plan[arm][-1]
        cur[arm] = (np.asarray(last.pos, float), f, np.asarray(last.yaw, float))

    def sync():
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm))

    def cup_grip(j, arm, put=False):
        return _spot(j) + [PUT_DX[arm] if put else 0.0, 0.0, GRIP_Z]

    def park(j):
        return np.array([PARK[j][0], PARK[j][1], MT + GRIP_Z + PARK_UP[j]])

    def lift_and_park(arm, j, what):
        g = cup_grip(j, arm)
        leave_home(arm, g + up, CUP_FINGER, f"over the {what} cup", "cup")
        go(arm, 1.2, g, CUP_FINGER, "down round its foot", n=3)
        wait(arm, 0.5, "settle")
        go(arm, 0.6, g, FINGER_CLOSED, "take the cup by its foot")
        go(arm, 1.4, g + [0.0, 0.0, PARK_UP[j]], FINGER_CLOSED, "lift it straight up", n=2)
        go(arm, 1.6, park(j), FINGER_CLOSED, "hold it up out to the side", n=4)

    def put_back(arm, j):
        g = cup_grip(j, arm, put=True)
        go(arm, 1.6, g + [0.0, 0.0, PARK_UP[j]], FINGER_CLOSED, "carry it back over its ring", n=4)
        go(arm, 1.0, g + [0.0, 0.0, 0.002], FINGER_CLOSED, "lower it onto the ring", n=2)
        go(arm, 0.5, g + [0.0, 0.0, 0.002], CUP_FINGER, "let go")
        go(arm, 0.8, g + up, CUP_FINGER, "rise clear")

    # -- hide the ball under the middle cup: the right takes the ball first and waits with it, the left lifts the
    # middle cup and holds it up out to its side, the right sets the ball on the ring and goes home, the left lowers
    # the cup over it and presses the start button
    ball0 = np.array([DX, DY, MT + DISH_FLOOR + R_BALL])
    on_ring = _spot(1) + [0.0, 0.0, R_BALL + 0.001]
    ready_ball = np.array([0.280, -0.210, MT + 0.120])  # out of the left arm's way (their elbows meet over the middle)
    leave_home("right", ball0 + up, BALL_FINGER, "over the ball in the dish", "ball")
    go("right", 1.0, ball0, BALL_FINGER, "down round the ball", n=2)
    go("right", 0.6, ball0, FINGER_CLOSED, "take the ball")
    go("right", 0.8, ball0 + up, FINGER_CLOSED, "lift it")
    go("right", 1.6, ready_ball, FINGER_CLOSED, "wait with it out to the right", n=3)
    wait("left", dur("right"), "wait at home")
    lift_and_park("left", 1, "middle")
    wait("right", dur("left") - dur("right"), "wait for the cup to be held up")
    go("right", 1.8, on_ring + up, FINGER_CLOSED, "carry it over the middle ring", n=4)
    go("right", 1.0, on_ring, FINGER_CLOSED, "lower it onto the ring", n=2)
    go("right", 0.6, on_ring, BALL_FINGER, "let go")
    wait("right", 0.6, "let it settle")
    go("right", 0.8, on_ring + up, BALL_FINGER, "rise clear")
    go_home("right")
    wait("left", dur("right") - 2.0 - dur("left"), "hold the cup up")
    put_back("left", 1)
    press = np.array([BX + BTN_DX, BY, MT + BTN_TOP + 0.013 - 0.007])  # closed fingertips reach ~13 mm below the TCP
    cur["left"] = (cur["left"][0], cur["left"][1], _rot("button"))
    go("left", 1.6, press + up, FINGER_CLOSED, "turn over the start button and close the hand", n=3)
    go("left", 0.8, press, FINGER_CLOSED, "press the start button")
    pressed_at = dur("left")
    go("left", 0.6, press + up, FINGER_CLOSED, "let it go")
    go_home("left")
    sync()
    # -- keep still through the countdown and the shuffle
    t_free = pressed_at + COUNTDOWN_S + SHUFFLE_S + 1.0
    for arm in ("left", "right"):
        wait(arm, t_free - dur(arm), "keep still at home and watch the cups being shuffled")
    # -- find the ball: the arm on the ball's side lifts its cup and holds it up out to its side, the other takes the
    # ball out; the cup goes back down and the ball into the dish
    j = final_place(DEMO_SEED, 1)
    holder, taker = ("right", "left") if j == 2 else ("left", "right")
    lift_and_park(holder, j, "ball's")
    ball = _spot(j) + [0.0, 0.0, R_BALL]
    wait(taker, dur(holder) - LEAD_S - dur(taker), "wait")
    leave_home(taker, ball + up, BALL_FINGER, "over the ball, its cup held up", "ball")
    go(taker, 1.0, ball, BALL_FINGER, "down round the ball", n=2)
    go(taker, 0.6, ball, FINGER_CLOSED, "take the ball")
    go(taker, 0.8, ball + up, FINGER_CLOSED, "lift it out")
    dish = np.array([DX, DY, MT + DISH_FLOOR + R_BALL + 0.012])
    go(taker, 1.6, dish + up, FINGER_CLOSED, "carry it over the dish", n=3)
    wait(holder, dur(taker) - dur(holder), "hold the cup up")
    put_back(holder, j)
    go_home(holder)
    go(taker, 0.8, dish, FINGER_CLOSED, "lower it into the dish", n=2)
    go(taker, 0.6, dish, BALL_FINGER, "let go")
    go(taker, 0.8, dish + up, BALL_FINGER, "rise clear")
    go_home(taker)
    sync()
    for arm in ("left", "right"):
        wait(arm, 3.0, "hold still: the trial ends once the full score has held 1 s with both arms home")
    return plan
