"""On the station table stands a lever guillotine paper cutter: a white cutting board with a back fence, its blade arm
hinged at the back right corner and raised, its black handle at the front. A stack of card lies on the board against
the back fence, overhanging the cutting edge by 2.5 cm. A blue recycling bin stands to the right of the cutting edge,
below it. A red stripe on the board marks the guard zone along the blade.

Task: trim the stack. One hand holds the stack down, away from the blade (the blade's shear drags loose card away
from the fence, so an unheld stack is cut crooked), while the other brings the blade down by its handle until the
strip is cut off and drops into the bin; then raise the blade back up to its rest. Neither gripper may be in the
guard zone along the blade while the blade is down.

Run it:  inspect-robots run --embodiment yam_arena -E task=trim_paper_on_a_guillotine_keeping_the_holding_hand_clear_of_the_blade.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, rotation, segment, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cutter's board: left of the cutting edge (the line y = YC), from X0 to X1 -------------------------------------
YC = -0.06
X0, X1 = 0.24, 0.46
BOARD_W, BOARD_T = 0.20, 0.020
BOARD_TOP = TOP + BOARD_T
FENCE_X = 0.432  # the back fence's front face
# -- the blade arm: hinged at the back right corner, its edge through the pivot ----------------------------------------
PIVOT = np.array([0.448, YC, BOARD_TOP + 0.003])
ARM_L = 0.26
REST_DEG = 32.0  # the blade's angle above level when raised
HANDLE_OUT, HANDLE_R = 0.07, 0.012  # the handle sticks out to the right of the arm's end
HOLD_GIVE = 2.0  # degrees: pushed further than this past where it rests, the arm moves (its pivot's friction)
LATCH_DEG = 3.0  # lifted to within this of its rest, the catch takes it the rest of the way and holds it
CUT_DEG, BAND_DEG = 2.5, 16.0  # the strip is cut off at 2.5 degrees; the blade shears the card below 16 degrees
SHEAR_N = 0.8  # the blade's shear drags the card forward, away from the fence
DRAG_MAX = 0.015  # m: it drags a loose stack at most this far
SQUARE_MM, SQUARE_DEG = 3.0, 1.5  # a square cut: the stack moved less than this while the blade went through
RAISED_DEG = 30.0  # back at its rest
DOWN_DEG = 25.0  # the guard zone counts while the blade is below this
ZONE_HALF, ZONE_UP = 0.030, 0.06  # the guard zone: within 3 cm of the cutting edge, up to 6 cm above the board

# -- the stack of card (main part on the board, the strip over the edge, glued until cut) ----------------------------
SX0, SX1 = 0.30, FENCE_X
STACK_T = 0.008
MAIN_W, STRIP_W = 0.115, 0.025

# -- the recycling bin, right of the cutting edge ----------------------------------------------------------------------
BIN = (0.275, 0.420, -0.165, YC - 0.003)  # inside: x from, x to, y from, y to
BIN_H, BIN_T = 0.015, 0.004

WHITE, GRID, RED, ALU, STEEL, DARK, BLACK, CARD, BLUE = (0.92, 0.92, 0.90), (0.55, 0.62, 0.70), (0.80, 0.12, 0.10), \
    (0.78, 0.80, 0.82), (0.70, 0.72, 0.75), (0.18, 0.18, 0.20), (0.06, 0.06, 0.06), (0.95, 0.62, 0.28), \
    (0.15, 0.35, 0.70)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _base() -> Body:
    """The cutting board (white, a printed grid, red guard stripes along the edge), the steel cutting edge, the
    aluminium back fence and the blade arm's pivot block. Origin on the table at the board's front right corner."""
    shapes = [BoxShape(((X1 - X0) / 2, BOARD_W / 2, BOARD_T / 2), (X1 - X0, BOARD_W, BOARD_T), WHITE),
              BoxShape(((X1 - X0) / 2, 0.003, BOARD_T + 0.0005), (X1 - X0, 0.006, 0.001), STEEL, collide=False)]
    for k in range(1, 8):  # the grid
        shapes.append(BoxShape(((X1 - X0) / 2, 0.025 * k, BOARD_T + 0.0004), (X1 - X0, 0.0012, 0.0008), GRID,
                               collide=False))
    for k in range(1, 8):
        shapes.append(BoxShape((0.025 * k, BOARD_W / 2 + 0.004, BOARD_T + 0.0004), (0.0012, BOARD_W - 0.008, 0.0008),
                               GRID, collide=False))
    for x0, x1 in ((0.0, SX0 - X0 - 0.004), (FENCE_X - X0 + 0.0, X1 - X0)):  # the guard zone's red stripes
        if x1 > x0:
            shapes.append(BoxShape(((x0 + x1) / 2, ZONE_HALF / 2 + 0.003, BOARD_T + 0.0008), (x1 - x0, ZONE_HALF, 0.0012),
                                   RED, collide=False))
    shapes += [BoxShape((FENCE_X - X0 + 0.004, BOARD_W / 2 + 0.008, BOARD_T + 0.0075), (0.008, BOARD_W - 0.016, 0.015),
                        ALU),
               BoxShape((PIVOT[0] - X0 + 0.001, -0.012, BOARD_T / 2 + 0.006), (0.014, 0.024, BOARD_T + 0.012), DARK),
               CylShape((PIVOT[0] - X0, -0.012, PIVOT[2] - TOP), 0.009, 0.030, STEEL, axis="Y", collide=False)]
    return Body.build("cutter", pos=(X0, YC, TOP), mass=3.0, shapes=shapes, kinematic=True, color=WHITE)


def _bin() -> Body:
    x0, x1, y0, y1 = BIN
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    shapes = [BoxShape((0.0, 0.0, BIN_T / 2), (x1 - x0 + 2 * BIN_T, y1 - y0 + 2 * BIN_T, BIN_T), BLUE)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * ((x1 - x0) / 2 + BIN_T / 2), 0.0, BIN_H / 2), (BIN_T, y1 - y0 + 2 * BIN_T, BIN_H),
                               BLUE))
        shapes.append(BoxShape((0.0, s * ((y1 - y0) / 2 + BIN_T / 2), BIN_H / 2), (x1 - x0, BIN_T, BIN_H), BLUE))
    return Body.build("bin", pos=(cx, cy, TOP), mass=0.5, shapes=shapes, kinematic=True, color=BLUE)


def _arm() -> Body:
    """The blade arm, raised REST_DEG: a steel blade whose edge runs from the pivot along the arm, a black arm on top
    of it and the black rubber handle sticking out to the right at its end. Origin at the pivot."""
    a = math.radians(REST_DEG)
    d = np.array([-math.cos(a), 0.0, math.sin(a)])
    up = np.array([math.sin(a), 0.0, math.cos(a)])
    end = d * ARM_L
    shapes = [segment(tuple((up * 0.012).tolist()), tuple((end + up * 0.012).tolist()), 0.004, STEEL, collide=False),
              segment(tuple((up * 0.024).tolist()), tuple((end + up * 0.024).tolist()), 0.016, DARK),
              CylShape(tuple((end + up * 0.024 + [0.0, -HANDLE_OUT / 2, 0.0]).tolist()), HANDLE_R, HANDLE_OUT, BLACK,
                       axis="Y"),
              CylShape(tuple((end + up * 0.024 + [0.0, -HANDLE_OUT - 0.004, 0.0]).tolist()), HANDLE_R + 0.002, 0.008,
                       RED, axis="Y", collide=False)]
    return Body.build("arm", pos=tuple(PIVOT.tolist()), mass=0.25, shapes=shapes, color=DARK,
                      com=tuple((end * 0.5 + up * 0.02).tolist()), friction=(1.0, 0.9),
                      ignore=("cutter", "stack", "strip", "bin"))


def _stack() -> Body:
    w = MAIN_W
    shapes = [BoxShape(((SX1 - SX0) / 2, w / 2, STACK_T / 2), (SX1 - SX0, w, STACK_T), CARD)]
    for k in range(1, 4):  # the sheets' edges (drawn)
        shapes.append(BoxShape(((SX1 - SX0) / 2, w / 2, STACK_T * k / 4), (SX1 - SX0 + 0.0006, w + 0.0006, 0.0005),
                               (0.80, 0.50, 0.22), collide=False))
    return Body.build("stack", pos=(SX0, YC, BOARD_TOP), mass=0.07, shapes=shapes, color=CARD, friction=(0.5, 0.45))


def _strip() -> Body:
    shapes = [BoxShape(((SX1 - SX0) / 2, -STRIP_W / 2, STACK_T / 2), (SX1 - SX0, STRIP_W, STACK_T), CARD)]
    return Body.build("strip", pos=(SX0, YC, BOARD_TOP), mass=0.015, shapes=shapes, color=CARD, friction=(0.5, 0.45),
                      ignore=("stack",))


OBJECTS = [_base(), _bin(), _arm(), _stack(), _strip()]
hinge("blade", "cutter", "arm", anchor=tuple(PIVOT.tolist()), axis="Y", limits_deg=(-REST_DEG - 0.5, 1.0),
      spring=0.2, damping=0.005, target_deg=0.0)
glue("uncut", "stack", "strip", anchor=(SX0, YC, BOARD_TOP + STACK_T / 2))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": 0.0, "last": None, "cut": False, "square": False, "entered": None, "inside": False,
            "violation": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GUILLOTINE_DEBUG")), "steps": 0}


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
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _set_force(asset, f_world, rot) -> None:
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)


def _in_zone(p) -> bool:
    return abs(p[1] - YC) < ZONE_HALF and X0 <= p[0] <= X1 and p[2] < BOARD_TOP + ZONE_UP


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _, ar = _pose(base, origin, "arm")
    sp, sr = _pose(base, origin, "stack")
    tp, _ = _pose(base, origin, "strip")
    hinge_deg = math.degrees(math.atan2(-ar[2, 0], ar[0, 0]))  # the arm's turn from its rest (+ raises it)
    blade = REST_DEG + hinge_deg  # the blade's angle above level
    # the pivot's friction: the arm stays where it is left; pushed past HOLD_GIVE it moves, and stays there
    if blade >= REST_DEG - LATCH_DEG:
        if _EP["hold"] != 0.0:  # back up at its rest: the catch takes it there
            _EP["hold"] = 0.0
            set_hinge_drive("blade", target_deg=0.0)
    elif abs(hinge_deg - _EP["hold"]) > HOLD_GIVE:
        _EP["hold"] = hinge_deg - math.copysign(HOLD_GIVE * 0.5, hinge_deg - _EP["hold"])
        set_hinge_drive("blade", target_deg=_EP["hold"])
    falling = _EP["last"] is not None and blade < _EP["last"] - 0.01
    _EP["last"] = blade
    shearing = not _EP["cut"] and CUT_DEG < blade < BAND_DEG
    if shearing and _EP["entered"] is None:
        _EP["entered"] = (sp.copy(), _yaw(sr))
    if shearing and float(np.linalg.norm((sp - _EP["entered"][0])[:2])) > DRAG_MAX:
        shearing = False  # dragged clear of the blade's edge: it drags no further
    _set_force(base.scene["stack"], (-SHEAR_N, 0.0, 0.0) if shearing and falling else (0.0, 0.0, 0.0), sr)
    if not _EP["cut"] and blade <= CUT_DEG:
        _EP["cut"] = True
        set_glue_on("uncut", False)
        p0, y0 = _EP["entered"] if _EP["entered"] is not None else (sp, _yaw(sr))
        _EP["square"] = float(np.linalg.norm((sp - p0)[:2])) * 1000 <= SQUARE_MM and abs(_yaw(sr) - y0) <= SQUARE_DEG
    if blade >= BAND_DEG + 2.0 and not _EP["cut"]:
        _EP["entered"] = None  # raised again before cutting through: the next descent is judged afresh
    _EP["tcps"] = _tcps(base, origin)
    zone = [_in_zone(t) for t in _EP["tcps"]]
    if blade < DOWN_DEG and any(zone):
        _EP["violation"] = True
    moved = float(np.linalg.norm((sp - np.array([SX0, YC, BOARD_TOP]))[:2])) * 1000
    _EP["s"] = {"blade": blade, "moved_mm": moved, "yaw": _yaw(sr), "strip_z": tp[2], "zone": zone}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, trc = _EP["tcps"]
            print(f"[guillotine] t={_DEBUG['steps'] / 20:5.1f}s blade={blade:5.1f}deg hold={_EP['hold']:6.1f} "
                  f"stack_moved={moved:5.1f}mm yaw={_yaw(sr):5.2f} cut={_EP['cut']} square={_EP['square']} "
                  f"strip_z={(tp[2] - TOP) * 1000:5.1f}mm zone={zone} violation={_EP['violation']} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000).astype(int).tolist()} score={score(None)}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if a gripper was ever in the guard zone (within 3 cm of the cutting edge over the board, less than 6 cm
    above it) while the blade was down below 25 degrees. Otherwise 1.0 with the strip cut off square (the stack moved
    less than 3 mm and turned less than 1.5 degrees while the blade went through it), the blade back up at its rest
    (30 degrees or more) and both arms back where they started; 0.5 with the strip cut off square at some point; 0.25
    with the strip cut off at all; else 0.0."""
    if _EP["violation"]:
        return 0.0
    if _EP["cut"] and _EP["square"]:
        if _EP["s"].get("blade", 0.0) >= RAISED_DEG and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["cut"] else 0.0


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
    """Full score with the stack, the strip and the blade still: the trial ends there."""
    return score(objects) >= 1.0 and objects["stack"].resting() and objects["strip"].resting() \
        and objects["arm"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the stack well left of the guard zone, fingers closed, and presses it down.
# The right hand comes down round the blade's handle, fingers closing across it, and brings the blade down along its
# swing until the strip is cut off and drops into the bin, then lifts it back up to its rest, lets go and goes back
# to where it started. The left lets go of the stack and goes back to where it started.

PRESS_GRIP = grip_rotation(0.0)
HANDLE_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x, across the handle
PRESS_AT = np.array([0.365, YC + 0.085])
PRESS_IN = 0.006  # the left hand aims this far into the stack's top, pressing it down
HANDLE_FINGER = -0.010  # a light pinch: the handle turns in the fingers as the arm swings
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _handle(blade_deg: float) -> np.ndarray:
    """The middle of the handle's grip with the blade at ``blade_deg`` above level."""
    a = math.radians(blade_deg)
    d = np.array([-math.cos(a), 0.0, math.sin(a)])
    up = np.array([math.sin(a), 0.0, math.cos(a)])
    return PIVOT + d * ARM_L + up * 0.024 + np.array([0.0, -HANDLE_OUT / 2 - 0.005, 0.0])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    top = np.array([PRESS_AT[0], PRESS_AT[1], BOARD_TOP + STACK_T])
    out = _lead_in("left", top + [0.0, 0.0, 0.04], PRESS_GRIP, "over the stack, well left of the guard zone",
                   FINGER_CLOSED)
    out += [_w(1.0, top - [0.0, 0.0, PRESS_IN], FINGER_CLOSED, "press the stack down", PRESS_GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), top - [0.0, 0.0, PRESS_IN], FINGER_CLOSED,
               "hold it down while the blade comes down", PRESS_GRIP),
            _w(0.8, top + [0.0, 0.0, 0.04], FINGER_CLOSED, "rise", PRESS_GRIP)]
    out += _go_home("left", top + [0.0, 0.0, 0.04], PRESS_GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    h0 = _handle(REST_DEG)
    out = _lead_in("right", h0 + [0.0, -0.07, 0.0], HANDLE_GRIP, "beside the blade's handle, at its end", FINGER_OPEN)
    out += [_w(1.2, h0, FINGER_OPEN, "slide the open fingers onto the handle from its end", HANDLE_GRIP),
            _w(0.5, h0, HANDLE_FINGER, "take it", HANDLE_GRIP),
            _w(max(start - LEAD_S - 1.5, 0.3), h0, HANDLE_FINGER, "wait for the stack to be held", HANDLE_GRIP)]
    for k in range(1, 9):  # down along its swing, a little past closed
        out.append(_w(0.35, _handle(REST_DEG - (REST_DEG + 1.0) * k / 8) - [0.0, 0.0, 0.003], HANDLE_FINGER,
                      "bring the blade down", HANDLE_GRIP))
    cut_at = sum(w.duration for w in out)
    out.append(_w(0.5, _handle(-1.0), HANDLE_FINGER, "the strip is cut off", HANDLE_GRIP))
    for k in range(1, 9):
        out.append(_w(0.3, _handle(-1.0 + (REST_DEG + 2.0) * k / 8) + [0.0, 0.0, 0.002], HANDLE_FINGER,
                      "lift the blade back up to its rest", HANDLE_GRIP))
    up = _handle(REST_DEG + 1.0)
    out += [_w(0.5, up, FINGER_OPEN, "let go", HANDLE_GRIP),
            _w(1.0, up + [0.0, -0.07, 0.0], FINGER_OPEN, "slide off the handle's end", HANDLE_GRIP)]
    out += _go_home("right", up + [0.0, -0.07, 0.0], HANDLE_GRIP, FINGER_OPEN)
    return out, cut_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, cut_at = _right(held_at + 0.5)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(cut_at + 0.6, total)
    return {"left": left, "right": right}
