"""On the station table a short copper riser stands in a plumber's bench stand, whose collar lets it turn freely. A tee
low on the riser carries a branch to the left, ending in a brass ball valve; at the top the riser ends in a brass
adapter. A tube cutter is already clamped on the riser at the pencil mark, its feed knob pointing to the right. A
scrap bucket stands to the right.

Task: cut the riser at the mark. The cutter's wheel drags the riser round with it, so one hand holds the riser still
by its valve while the other swings the cutter back and forth round the pipe until the top piece comes free; then
lift the cut-off piece off and drop it in the bucket, let go of the valve, and take both arms back to where they
started.

Run it:  inspect-robots run --embodiment yam_arena -E task=cut_a_copper_riser_with_a_tube_cutter_holding_it_by_its_valve.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the riser: upright, turning freely in the stand's collar ---------------------------------------------------------
RX, RY = 0.27, 0.0
R_OD = 0.011  # 22 mm copper pipe
Z0 = TOP + 0.012  # its bottom, on the stand's base
ZB = TOP + 0.050  # the tee's middle
ZC = TOP + 0.095  # the pencil mark, where it is cut
ZT = ZC + 0.030  # the pipe's top, under the adapter
HEX = (0.020, 0.020, 0.016)  # the brass adapter's hex: across the flats, x and y; tall
BRANCH_END = 0.122  # the branch pipe runs left to here
VALVE = (0.030, 0.020, 0.030)  # the ball valve's body: x, y (along the branch), z
VALVE_Y = BRANCH_END + VALVE[1] / 2
COLLAR_DAMP = 0.0002  # N m per degree/s: the riser turns almost freely in the collar
FEED_DAMP = 0.3 * math.pi / 180  # N m per degree/s: the cutter's wheel and rollers drag on the pipe (0.3 N m s/rad)

# -- the cutter: hinged round the pipe at the mark, its feed knob pointing to the right (-y) at the start -----------
KNOB_R = 0.110  # from the pipe's axis to the middle of the knob's grip block
GRIP_BLOCK = (0.016, 0.020, 0.020)  # tangential, radial, tall
CUT_DEG = 720.0  # degrees the cutter must turn on the pipe (either way, added up) to cut it through
HALF_DEG = 360.0

# -- the scrap bucket ----------------------------------------------------------------------------------------------
BUCKET_C = np.array([0.20, -0.25])
BUCKET_IN, BUCKET_H, BUCKET_T = 0.12, 0.060, 0.004
LIFTED = 0.02  # the top piece is off once lifted this far

COPPER, BRASS, IRON, RED, STEEL, DARK, BUCKET_BLUE, PENCIL = (0.78, 0.45, 0.28), (0.80, 0.66, 0.30), \
    (0.25, 0.26, 0.28), (0.75, 0.15, 0.10), (0.75, 0.77, 0.80), (0.12, 0.12, 0.12), (0.20, 0.35, 0.60), \
    (0.10, 0.10, 0.10)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _stand() -> Body:
    """A cast-iron bench stand: a base plate and a collar round the riser's foot (drawn only; a hinge holds it)."""
    o = np.array([RX, RY, TOP])
    shapes = [BoxShape((0.0, 0.0, 0.006), (0.12, 0.12, 0.012), IRON)]
    for k in range(8):
        a = k * math.pi / 4
        shapes.append(BoxShape((0.018 * math.cos(a), 0.018 * math.sin(a), 0.012 + 0.02), (0.006, 0.016, 0.04), IRON,
                               yaw=a, collide=False))
    shapes.append(CylShape((0.045, 0.045, 0.014), 0.006, 0.004, STEEL, collide=False))  # bolt heads
    shapes.append(CylShape((-0.045, -0.045, 0.014), 0.006, 0.004, STEEL, collide=False))
    return Body.build("stand", pos=tuple(o.tolist()), mass=3.0, shapes=shapes, kinematic=True, color=IRON)


def _riser() -> Body:
    """The riser below the mark, with the tee, the branch and the valve; origin on its axis at the mark."""
    zb = ZB - ZC
    shapes = [CylShape((0.0, 0.0, (Z0 - ZC) / 2), R_OD, ZC - Z0, COPPER),
              CylShape((0.0, 0.0, zb), 0.0135, 0.032, COPPER),  # the tee
              CylShape((0.0, (0.012 + BRANCH_END) / 2, zb), R_OD, BRANCH_END - 0.012, COPPER, axis="Y"),
              BoxShape((0.0, VALVE_Y, zb), VALVE, BRASS),
              CylShape((0.0, VALVE_Y + VALVE[1] / 2 + 0.007, zb), 0.009, 0.014, BRASS, collide=False, axis="Y"),
              BoxShape((VALVE[0] / 2 + 0.003, VALVE_Y, zb + 0.008), (0.006, 0.010, 0.010), BRASS, collide=False),
              BoxShape((VALVE[0] / 2 + 0.006, VALVE_Y - 0.03, zb + 0.008), (0.004, 0.06, 0.008), RED, collide=False),
              CylShape((0.0, 0.0, 0.0), R_OD + 0.0003, 0.0015, PENCIL, collide=False)]  # the pencil mark
    return Body.build("riser", pos=(RX, RY, ZC), mass=0.30, shapes=shapes, friction=(1.2, 1.1), color=COPPER,
                      com=(0.0, 0.03, zb))


def _top_piece() -> Body:
    """The riser above the mark with its brass adapter; origin on the axis at the mark."""
    h = ZT - ZC
    shapes = [CylShape((0.0, 0.0, h / 2), R_OD, h, COPPER),
              BoxShape((0.0, 0.0, h + HEX[2] / 2), HEX, BRASS)]
    for yaw in (math.pi / 6, -math.pi / 6):  # the hex's other flats (drawn)
        shapes.append(BoxShape((0.0, 0.0, h + HEX[2] / 2), (HEX[0] * 1.03, HEX[1] * 0.58, HEX[2]), BRASS, yaw=yaw,
                               collide=False))
    shapes.append(CylShape((0.0, 0.0, h + HEX[2] + 0.005), 0.0085, 0.010, BRASS, collide=False))  # its thread
    return Body.build("top_piece", pos=(RX, RY, ZC), mass=0.06, shapes=shapes, friction=(1.2, 1.1), color=BRASS,
                      com=(0.0, 0.0, h / 2 + 0.008))


def _cutter() -> Body:
    """A tube cutter round the pipe at the mark; origin on the axis. Its frame, wheel and rollers sit round the pipe,
    its feed screw and knob point to the right (-y), ending in a flat grip block."""
    shapes = [BoxShape((0.0, 0.021, 0.0), (0.036, 0.008, 0.018), RED),  # the frame behind the pipe (wheel side)
              BoxShape((0.0155, 0.0, 0.0), (0.005, 0.050, 0.018), RED),
              BoxShape((-0.0155, 0.0, 0.0), (0.005, 0.050, 0.018), RED),
              BoxShape((0.0, -0.022, 0.0), (0.036, 0.012, 0.022), RED),  # the roller carriage
              CylShape((0.0, 0.0175, 0.0), 0.0075, 0.002, STEEL, collide=False),  # the cutting wheel
              CylShape((0.006, -0.0145, 0.0), 0.0045, 0.012, STEEL, collide=False),  # rollers
              CylShape((-0.006, -0.0145, 0.0), 0.0045, 0.012, STEEL, collide=False),
              CylShape((0.0, -0.050, 0.0), 0.004, 0.044, STEEL, axis="Y"),  # the feed screw
              CylShape((0.0, -0.088, 0.0), 0.012, 0.030, RED, axis="Y"),  # the knob
              BoxShape((0.0, -KNOB_R, 0.0), (GRIP_BLOCK[0], GRIP_BLOCK[1], GRIP_BLOCK[2]), DARK)]  # its grip block
    for k in range(6):  # knurling on the knob (drawn)
        shapes.append(BoxShape((0.0, -0.077 - k * 0.0045, 0.0), (0.0255, 0.0012, 0.0255), (0.55, 0.10, 0.08),
                               collide=False))
    return Body.build("cutter", pos=(RX, RY, ZC), mass=0.15, shapes=shapes, friction=(1.2, 1.1), color=RED,
                      com=(0.0, -0.03, 0.0), ignore=("top_piece",))


def _bucket() -> Body:
    shapes = [BoxShape((0.0, 0.0, BUCKET_T / 2), (BUCKET_IN + 2 * BUCKET_T, BUCKET_IN + 2 * BUCKET_T, BUCKET_T),
                       BUCKET_BLUE)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (BUCKET_IN + BUCKET_T) / 2, 0.0, BUCKET_H / 2),
                               (BUCKET_T, BUCKET_IN + 2 * BUCKET_T, BUCKET_H), BUCKET_BLUE))
        shapes.append(BoxShape((0.0, s * (BUCKET_IN + BUCKET_T) / 2, BUCKET_H / 2), (BUCKET_IN, BUCKET_T, BUCKET_H),
                               BUCKET_BLUE))
    return Body.build("bucket", pos=(BUCKET_C[0], BUCKET_C[1], TOP), mass=0.5, shapes=shapes, kinematic=True,
                      color=BUCKET_BLUE)


OBJECTS = [_stand(), _bucket(), _riser(), _top_piece(), _cutter()]
hinge("collar", "stand", "riser", anchor=(RX, RY, Z0), axis="Z", damping=COLLAR_DAMP)
hinge("feed", "riser", "cutter", anchor=(RX, RY, ZC), axis="Z", damping=FEED_DAMP)
# The pipe is whole until cut: the top piece is held on the riser by a stiff slide along the axis; once cut through,
# the slide goes slack (the top piece rests on the cut, free to lift), and once lifted off, the task file frees it.
slider("cut", "riser", "top_piece", anchor=(RX, RY, ZC), axis="Z", limits=(0.0, 0.06), stiffness=1.0e5,
       damping=50.0, target=0.0)


def _set_cut_joint(on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    joint = UsdPhysics.Joint(get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS["cut"]))
    attr = joint.GetJointEnabledAttr() or joint.CreateJointEnabledAttr(True)
    attr.Set(bool(on))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"rel": None, "turned": 0.0, "cut": False, "off": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CUTTER_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _set_cut_joint(False)  # rebuilt whole, from the reset poses
    _set_cut_joint(True)


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
    return math.atan2(r[1, 0], r[0, 0])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    rp, rr = _pose(base, origin, "riser")
    cp, cr = _pose(base, origin, "cutter")
    tp, tr = _pose(base, origin, "top_piece")
    rel = _yaw(rr.T @ cr)  # the cutter's turn on the pipe
    if _EP["rel"] is not None:
        d = (rel - _EP["rel"] + math.pi) % (2 * math.pi) - math.pi
        _EP["turned"] += abs(math.degrees(d))
    _EP["rel"] = rel
    if not _EP["cut"] and _EP["turned"] >= CUT_DEG:
        _EP["cut"] = True
        set_slider_drive("cut", stiffness=0.0, damping=0.5)  # cut through: the top piece rests on the cut, loose
    lift = float((tp - rp) @ rr[:, 2])
    if _EP["cut"] and not _EP["off"] and lift >= LIFTED:
        _EP["off"] = True
        _set_cut_joint(False)
    _EP["tcps"] = _tcps(base, origin)
    mid = tp + tr @ np.array([0.0, 0.0, (ZT - ZC) / 2])
    in_bucket = abs(mid[0] - BUCKET_C[0]) <= BUCKET_IN / 2 and abs(mid[1] - BUCKET_C[1]) <= BUCKET_IN / 2 \
        and mid[2] < TOP + BUCKET_H
    _EP["s"] = {"in_bucket": in_bucket, "lift": lift, "riser_yaw": math.degrees(_yaw(rr))}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, trc = _EP["tcps"]
            print(f"[cutter] t={_DEBUG['steps'] / 20:5.1f}s turned={_EP['turned']:6.1f}deg rel={math.degrees(rel):6.1f} "
                  f"riser_yaw={_EP['s']['riser_yaw']:6.1f} cut={_EP['cut']} lift={lift * 1000:5.1f}mm off={_EP['off']} "
                  f"top={np.round(mid * 1000).astype(int).tolist()} in_bucket={in_bucket} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} rightTCP={np.round(trc * 1000).astype(int).tolist()} "
                  f"score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the riser cut through (the cutter turned at least 720 degrees on the pipe, back and forth added up),
    the cut-off piece in the scrap bucket and both arms back where they started; 0.5 with the riser cut through at
    some point; 0.25 with the cutter turned at least 360 degrees on the pipe at some point; else 0.0."""
    if _EP["cut"]:
        if _EP["s"].get("in_bucket") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["turned"] >= HALF_DEG else 0.0


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
    """Full score with the cut-off piece still: the trial ends there."""
    return score(objects) >= 1.0 and objects["top_piece"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the valve, fingers closing along the branch, and holds the riser still. The
# right hand comes down with its fingers part open on either side of the knob's grip block and swings the cutter back
# and forth round the pipe, about a quarter turn each way, the wrist turning with it, until the top piece is cut free.
# It rises off the knob; the left lets go of the valve and goes back to where it started. The right takes the cut-off piece by its
# brass hex, lifts it off, carries it over the bucket and drops it in, and goes back to where it started.

VALVE_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (along the branch)
TAB_IN = -0.004  # TCPs this far below a part's top (4 mm above it: the fingertips pinch ~9 mm deep)
SWING = (-40.0, -140.0)  # the knob's direction (degrees from +x) at each end of a swing
SWINGS = 10  # strokes, each about a quarter turn
STROKE_S, STROKE_STEPS = 2.0, 8
FORK = -0.014  # the fingers part open ~2.2 cm round the 1.6 cm grip block
FORK_IN = -0.005  # the TCP this far below the block's top
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


def _knob(deg: float) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP at the knob's grip block with the knob pointing ``deg`` from +x, and the grip: the fingers part
    open on either side of the block, across the way it swings (a fork that pushes it round, free to slide along the
    knob)."""
    a = math.radians(deg)
    pos = np.array([RX + KNOB_R * math.cos(a), RY + KNOB_R * math.sin(a), ZC + GRIP_BLOCK[2] / 2 + FORK_IN])
    return pos, grip_rotation(a + math.pi)


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    valve = np.array([RX, RY + VALVE_Y, ZB + VALVE[2] / 2 - TAB_IN])
    out = _lead_in("left", valve + [0.0, 0.0, 0.04], VALVE_GRIP, "over the valve", -0.017)
    out += [_w(1.0, valve, -0.017, "down round it", VALVE_GRIP),
            _w(0.6, valve, FINGER_CLOSED, "hold the riser still by its valve", VALVE_GRIP)]
    held_at = sum(w.duration for w in out)
    up = valve + [0.0, 0.0, 0.05]
    out += [_w(max(release_at - held_at, 0.3), valve, FINGER_CLOSED, "hold it while the cutter turns", VALVE_GRIP),
            _w(0.5, valve, FINGER_OPEN, "let go", VALVE_GRIP),
            _w(0.8, up, FINGER_OPEN, "rise", VALVE_GRIP)]
    out += _go_home("left", up, VALVE_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    p0, g0 = _knob(-90.0)
    out = _lead_in("right", p0 + [0.0, 0.0, 0.04], g0, "over the cutter's knob", FORK)
    out.append(_w(max(start - LEAD_S, 0.3), p0 + [0.0, 0.0, 0.04], FORK, "wait for the riser to be held", g0))
    out += [_w(1.2, p0, FORK, "down, the fingers on either side of the knob's grip block", g0)]
    deg, ends = -90.0, [SWING[k % 2] for k in range(SWINGS)]
    for end in ends:
        for k in range(1, STROKE_STEPS + 1):
            d = deg + (end - deg) * k / STROKE_STEPS
            p, g = _knob(d)
            out.append(_w((STROKE_S if abs(end - deg) > 60 else STROKE_S / 2) / STROKE_STEPS, p, FORK,
                          "swing the cutter round the pipe", g))
        deg = end
    p, g = _knob(deg)
    out += [_w(0.8, p + [0.0, 0.0, 0.05], FORK, "rise off the knob", g)]
    cut_at = sum(w.duration for w in out)
    hexg = grip_rotation(0.0)
    hex_top = np.array([RX, RY, ZT + HEX[2] - TAB_IN])
    put = np.array([BUCKET_C[0], BUCKET_C[1], hex_top[2] + 0.02])
    side = hex_top - [0.06, 0.0, 0.0]  # in front of the hex, at its height: the arm cannot reach far above it
    out += [_w(2.5, side, FINGER_OPEN, "beside the cut-off piece's brass hex, fingers open (once the left is away)",
               hexg),
            _w(1.2, hex_top, FINGER_OPEN, "in round it, the hex between the fingers", hexg),
            _w(0.6, hex_top, FINGER_CLOSED, "take it", hexg),
            _w(1.2, hex_top + [0.0, 0.0, 0.025], FINGER_CLOSED, "lift the cut-off piece off the riser", hexg),
            _w(2.0, put, FINGER_CLOSED, "carry it over the scrap bucket", hexg),
            _w(0.5, put, FINGER_OPEN, "drop it in", hexg),
            _w(0.6, put + [0.0, 0.0, 0.03], FINGER_OPEN, "rise", hexg)]
    out += _go_home("right", put + [0.0, 0.0, 0.03], hexg, FINGER_OPEN)
    return out, cut_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, cut_at = _right(held_at)
    # the right waits over the hex until the left has let go and gone home
    left, _ = _left(cut_at, 0.0)
    left_home = sum(w.duration for w in left[:-1])
    for i, w in enumerate(right):
        if w.label.startswith("beside the cut-off piece"):
            right.insert(i, _w(max(left_home - cut_at - 1.0, 0.3), right[i - 1].pos, FINGER_OPEN,
                               "wait for the left to be away", right[i - 1].yaw))
            break
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(cut_at, total)
    return {"left": left, "right": right}
