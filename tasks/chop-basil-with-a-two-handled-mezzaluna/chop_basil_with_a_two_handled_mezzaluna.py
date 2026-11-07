"""On the station table lies a beech chopping board with eight basil leaves on it in a row. In front of the board, a
double-handled mezzaluna (a curved stainless steel blade with an upright beech handle at each end) stands edge down in
a wooden rest.

Task: chop the basil with the mezzaluna. Take it out of its rest by both handles, one in each hand, set its curved
edge down across the basil and rock it from end to end, back and forth, until every leaf is chopped, then stand it
back in its rest and take both arms back to where they started. A mezzaluna is worked with both hands: held by one
handle, it swings down round that hand instead of rocking.

Run it:  inspect-robots run --embodiment yam_arena -E task=chop_basil_with_a_two_handled_mezzaluna.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the board and the basil ----------------------------------------------------------------------------------------
BX0, BX1, BY0, BY1, BOARD_T = 0.225, 0.345, -0.150, 0.150, 0.020
BT = TOP + BOARD_T  # the board's top
X0 = 0.285  # the line the basil lies along (and the blade rocks along)
LEAVES = 8
LEAF_Y = [-0.049 + 0.014 * k for k in range(LEAVES)]
LEAF_DX = [0.004, -0.003, 0.002, -0.004, 0.003, -0.002, 0.004, -0.003]
LEAF_YAW = [0.5, -0.3, 0.9, 0.1, -0.6, 0.4, -0.2, 0.7]
# -- the mezzaluna: body origin at its edge's lowest point, the blade in the body's y-z plane -----------------------
R, PHI, BAND, BLADE_T = 0.150, 0.60, 0.035, 0.003  # edge radius, half the arc's angle, blade depth, thickness
SEGS = 10
HANDLE_Y, HANDLE_Z0, HANDLE_Z1, HANDLE_R = 0.088, 0.040, 0.095, 0.011
GRIP_Z = 0.072  # the hands hold the handles this high above the edge's lowest point
# -- the rest: a V trough along y in front of the board, its bottom slot holding the blade upright -----------------
RX = 0.180
REST_BOTTOM = TOP + 0.012  # where the edge's lowest point sits in the rest
CHOP_Z, CHOP_ROLL, CHOP_DX, CHOP_DY = 0.006, 30.0, 0.012, 0.012

STEEL, BEECH, DARK_BEECH, BASIL, BASIL_DARK = (0.80, 0.82, 0.84), (0.80, 0.62, 0.40), (0.55, 0.38, 0.22), \
    (0.20, 0.55, 0.18), (0.12, 0.38, 0.10)
PARK = [(0.70, -0.30 + 0.04 * k, 0.30) for k in range(2 * LEAVES)]

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _arc(phi: float, r: float) -> np.ndarray:
    """A point on a circle of radius r about the blade's centre, at angle phi from straight down (body frame)."""
    return np.array([0.0, r * math.sin(phi), R - r * math.cos(phi)])


def _board() -> Body:
    shapes = [BoxShape((0.0, 0.0, BOARD_T / 2), (BX1 - BX0, BY1 - BY0, BOARD_T), BEECH),
              BoxShape((0.0, 0.0, BOARD_T + 0.0002), (BX1 - BX0 - 0.012, BY1 - BY0 - 0.012, 0.0004),
                       (0.83, 0.66, 0.45), collide=False)]
    return Body.build("board", pos=((BX0 + BX1) / 2, (BY0 + BY1) / 2, TOP), mass=1.0, kinematic=True, color=BEECH,
                      shapes=shapes, friction=(0.8, 0.7))


def _rest() -> Body:
    """A beech rest: a 5 mm slot 2.5 cm deep along y, boards leaning out above it in a V to guide the blade in."""
    z_slot = REST_BOTTOM - TOP
    shapes = [BoxShape((0.0, 0.0, z_slot / 2), (0.06, 0.20, z_slot), DARK_BEECH)]
    wall_top = z_slot + 0.025
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * 0.0040, 0.0, z_slot + 0.0125), (0.003, 0.20, 0.025), DARK_BEECH))  # slot walls
        shapes.append(BoxShape((s * 0.019, 0.0, z_slot + 0.009), (0.024, 0.20, 0.018), DARK_BEECH))  # shoulders
        # the V's sides lean out from the slot's lips, so a blade let down anywhere over the V slides into the slot
        d = np.array([s * 0.026, 0.0, 0.020]) / math.hypot(0.026, 0.020)
        n = np.array([s * 0.020, 0.0, -0.026]) / math.hypot(0.026, 0.020)  # pointing out and down, away from the slot
        p1 = np.array([s * 0.0025, 0.0, wall_top]) + 0.0015 * n
        shapes.append(plate(p1, p1 + 0.033 * d, 0.20, 0.003, n, DARK_BEECH))
    return Body.build("rest", pos=(RX, 0.0, TOP), mass=1.0, kinematic=True, color=DARK_BEECH, shapes=shapes,
                      friction=(0.5, 0.4))


def _mezzaluna() -> Body:
    """A double-handled mezzaluna: a curved stainless blade, an upright beech handle on a steel collar at each end."""
    shapes = []
    d = 2 * PHI / SEGS
    for k in range(SEGS):
        a, b = -PHI + d * k, -PHI + d * (k + 1)
        mid = R - BAND / 2
        shapes.append(plate(_arc(a, mid) - [0.0, 0.0, 0.0], _arc(b, mid), BAND, BLADE_T, (1.0, 0.0, 0.0), STEEL))
    for k in range(SEGS):  # the bevel along the edge, a little brighter
        a, b = -PHI + d * k, -PHI + d * (k + 1)
        shapes.append(plate(_arc(a, R - 0.003), _arc(b, R - 0.003), 0.006, BLADE_T + 0.0006, (1.0, 0.0, 0.0),
                            (0.92, 0.93, 0.95), collide=False))
    for s in (-1.0, 1.0):
        end_top = _arc(s * PHI, R - BAND)
        shapes.append(BoxShape((0.0, (s * HANDLE_Y + end_top[1]) / 2, HANDLE_Z0 - 0.004),
                               (0.008, abs(s * HANDLE_Y - end_top[1]) + 0.012, 0.010), STEEL))  # the collar
        shapes.append(CylShape((0.0, s * HANDLE_Y, (HANDLE_Z0 + HANDLE_Z1) / 2), HANDLE_R, HANDLE_Z1 - HANDLE_Z0,
                               BEECH))
        shapes.append(CylShape((0.0, s * HANDLE_Y, HANDLE_Z1 + 0.003), HANDLE_R + 0.002, 0.006, DARK_BEECH))
    return Body.build("mezzaluna", pos=(RX, 0.0, REST_BOTTOM), mass=0.25, shapes=shapes, friction=(1.2, 1.1),
                      color=STEEL, com=(0.0, 0.0, 0.045), damping=(0.0, 0.5))


def _leaf(k: int) -> Body:
    """A basil leaf: two overlapping ovals of green with a darker midrib."""
    c, s = math.cos(LEAF_YAW[k]), math.sin(LEAF_YAW[k])
    shapes = [BoxShape((0.0, 0.0, 0.0012), (0.026, 0.014, 0.0016), BASIL, yaw=LEAF_YAW[k], collide=False),
              BoxShape((0.0, 0.0, 0.0012), (0.018, 0.019, 0.0014), BASIL, yaw=LEAF_YAW[k], collide=False),
              BoxShape((0.002 * c, 0.002 * s, 0.0022), (0.024, 0.0018, 0.0006), BASIL_DARK, yaw=LEAF_YAW[k],
                       collide=False)]
    return Body.build(f"leaf{k}", pos=(X0 + LEAF_DX[k], LEAF_Y[k], BT), mass=0.002, kinematic=True, color=BASIL,
                      shapes=shapes)


def _bits(k: int) -> Body:
    """The same leaf chopped: a little heap of green bits."""
    rng = np.random.default_rng(7 + k)
    shapes = []
    for j in range(9):
        x, y = rng.uniform(-0.009, 0.009), rng.uniform(-0.009, 0.009)
        shapes.append(BoxShape((float(x), float(y), 0.001 + 0.0006 * (j % 3)), (0.0045, 0.0035, 0.0014),
                               BASIL if j % 3 else BASIL_DARK, yaw=float(rng.uniform(0, math.pi)), collide=False))
    return Body.build(f"bits{k}", pos=PARK[LEAVES + k], mass=0.002, kinematic=True, color=BASIL, shapes=shapes)


OBJECTS = [_board(), _rest(), _mezzaluna()] + [_leaf(k) for k in range(LEAVES)] + [_bits(k) for k in range(LEAVES)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"chopped": [False] * LEAVES, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MEZZALUNA_DEBUG")), "steps": 0}
_EDGE = [_arc(-PHI + 2 * PHI * k / 40, R) for k in range(41)]  # points along the edge (body frame)


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


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _roll(r) -> float:
    """How far the blade is rocked from level (about the axis across its plane), degrees."""
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def _upright(r) -> float:
    """How far the blade's plane leans from vertical, degrees."""
    return math.degrees(math.asin(min(1.0, abs(float(r[2, 0])))))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    mp, mr = _pose(base, origin, "mezzaluna")
    edge = [mp + mr @ e for e in _EDGE]
    low = min(edge, key=lambda e: e[2])
    roll, lean = _roll(mr), _upright(mr)
    cutting = BT - 0.004 <= low[2] <= BT + CHOP_Z and BX0 <= low[0] <= BX1 and abs(roll) <= CHOP_ROLL and lean <= 15.0
    if cutting:
        for k in range(LEAVES):
            if not _EP["chopped"][k] and abs(low[0] - (X0 + LEAF_DX[k])) <= CHOP_DX \
                    and abs(low[1] - LEAF_Y[k]) <= CHOP_DY:
                _EP["chopped"][k] = True
                _place(base, origin, f"bits{k}", (X0 + LEAF_DX[k], LEAF_Y[k], BT))
                _place(base, origin, f"leaf{k}", PARK[k])
    in_rest = abs(mp[0] - RX) <= 0.006 and abs(mp[1]) <= 0.02 and abs(mp[2] - REST_BOTTOM) <= 0.008 and lean <= 10.0
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"in_rest": in_rest}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            print(f"[mezz] t={_DEBUG['steps'] / 20:5.1f}s pos={np.round(mp * 1000).astype(int).tolist()} "
                  f"roll={roll:6.1f} lean={lean:5.1f} low={np.round(low * 1000).astype(int).tolist()} "
                  f"above_board={(low[2] - BT) * 1000:6.1f}mm cutting={cutting} chopped={sum(_EP['chopped'])} "
                  f"in_rest={in_rest} leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000).astype(int).tolist()} score={score(None)}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with at least seven of the eight basil leaves chopped, the mezzaluna standing back in its rest and both
    arms back where they started, at the end; 0.5 with at least seven chopped; 0.25 with at least one chopped;
    else 0.0. (A chopped leaf stays chopped.)"""
    n = sum(_EP["chopped"])
    if n >= LEAVES - 1:
        if _EP["s"].get("in_rest") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if n >= 1 else 0.0


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
    """Full score with the mezzaluna still in its rest: the trial ends there."""
    return score(objects) >= 1.0 and objects["mezzaluna"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come down on the handles and take them, lift the mezzaluna out of its rest together and carry it over the
# basil, set its edge just over the board and rock it from end to end, back and forth (rolling it along the row, as a
# rocked curved blade rolls). Then they lift it, carry it back over the rest and lower it in, let go and go back to
# where they started.

G_HANDLE = {"left": grip_rotation(-math.pi / 2), "right": grip_rotation(math.pi / 2)}  # down, fingers closing along x
HANDLE_FINGER = -0.026
LEAD_S, LEAD_STEPS = 3.0, 15
ROCK = math.radians(16.0)
OVER = 0.050  # the hands come down onto the handles from this far above the grip (clear of the handles' tops)
AIM_DX = 0.003
EDGE_UP = 0.002  # the edge's lowest point this far over the board while rocking


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _tool(origin, theta: float) -> tuple[np.ndarray, np.ndarray]:
    """The mezzaluna's origin and rotation with its blade's centre over ``origin``'s y and rocked by ``theta`` so that
    the edge rests on the line below that centre (``origin`` is where the edge's lowest point would be, unrocked)."""
    centre = np.asarray(origin, float) + [0.0, 0.0, R]
    rot = _rx(theta)
    return centre - rot @ np.array([0.0, 0.0, R]), rot


def _grips(origin, theta: float) -> dict:
    pos, rot = _tool(origin, theta)
    out = {}
    for arm, s in (("left", 1.0), ("right", -1.0)):
        out[arm] = (pos + rot @ np.array([0.0, s * HANDLE_Y, GRIP_Z]), rot @ G_HANDLE[arm])
    return out


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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena.constants import READY

    in_rest = np.array([RX + AIM_DX, 0.0, REST_BOTTOM])  # aimed a little long: the loaded arms fall ~3 mm short
    over_rest = in_rest + [0.0, 0.0, 0.05]
    on_board = np.array([X0 + AIM_DX, 0.0, BT + EDGE_UP])
    over_board = on_board + [0.0, 0.0, 0.03]
    plan = {"left": [], "right": []}
    # lead in: each hand over its handle, in joint space
    for arm in ("left", "right"):
        p, r = _grips(in_rest, 0.0)[arm]
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, p + [0.0, 0.0, OVER], r),
                                 "over the mezzaluna's handle", HANDLE_FINGER)

    def both(t, origin, theta, finger, label):
        for arm, (p, r) in _grips(origin, theta).items():
            plan[arm].append(_w(t, p, finger, label, r))

    def both_offset(t, origin, theta, finger, label, dz):
        for arm, (p, r) in _grips(origin, theta).items():
            plan[arm].append(_w(t, p + [0.0, 0.0, dz], finger, label, r))

    both_offset(0.5, in_rest, 0.0, HANDLE_FINGER, "over the mezzaluna's handle", OVER)
    both(1.2, in_rest, 0.0, HANDLE_FINGER, "down round the handle")
    both(0.6, in_rest, 0.0, FINGER_CLOSED, "take it by both handles")
    both(1.2, over_rest, 0.0, FINGER_CLOSED, "lift it out of its rest together")
    both(1.8, over_board, 0.0, FINGER_CLOSED, "carry it over the basil")
    both(1.0, on_board, 0.0, FINGER_CLOSED, "set its edge down across the basil")
    rocked = [0.0]
    for sweep in range(2):  # rock: tip it one way, then right over to the other, and back to level
        for target in (ROCK, -ROCK, 0.0) if sweep == 0 else (-ROCK, ROCK, 0.0):
            n = 8
            th0 = rocked[0]
            for j in range(1, n + 1):
                th = th0 + (target - th0) * j / n
                both(0.22, on_board + [0.0, -R * th, 0.0], th, FINGER_CLOSED,
                     "rock it from end to end over the basil")
            rocked[0] = target
    both(1.0, over_board, 0.0, FINGER_CLOSED, "lift it off the board")
    both(1.8, over_rest, 0.0, FINGER_CLOSED, "carry it back over its rest")
    both(1.4, in_rest - [0.0, 0.0, 0.006], 0.0, FINGER_CLOSED, "stand it back in its rest, right down in the slot")
    both(0.6, in_rest - [0.0, 0.0, 0.006], 0.0, HANDLE_FINGER, "let go")
    both_offset(1.0, in_rest, 0.0, HANDLE_FINGER, "rise clear of the handle", OVER)
    for arm in ("left", "right"):
        p, r = _grips(in_rest, 0.0)[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p + [0.0, 0.0, OVER], r), np.array(READY, float),
                                 "back to where it started", HANDLE_FINGER)
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, HANDLE_FINGER,
                            "hold still: the trial ends once the full score has held 1 s with both arms home",
                            last.yaw))
    return plan
