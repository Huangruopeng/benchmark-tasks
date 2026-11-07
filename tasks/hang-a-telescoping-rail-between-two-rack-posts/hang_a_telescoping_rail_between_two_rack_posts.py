"""On the station table stand the two posts of a small equipment rack, 38 cm apart along the table: square steel
tubes with square mounting holes, a pair of them at the height where the rail goes. In front of them lies a
telescoping rack rail, pushed short: an outer channel with a front bracket, and an inner channel, sliding in it, with
a rear bracket. Each bracket has two square pegs.

Task: hang the rail between the posts. One hand holds the rail's outer channel and puts the front bracket's pegs into
the left post's holes; it keeps the rail there while the other hand pulls the inner channel out (it slides stiffly)
until the rear bracket's pegs go into the right post's holes, where they latch and lock the rail in place. The front
pegs only rest in their holes until then: let go of the front, and the rail drops out; pull the inner channel without
holding the outer one, and the whole rail comes off the front post. Then both hands let go, and both arms go back to
where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=hang_a_telescoping_rail_between_two_rack_posts.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step, rotation,  # noqa: E402
                     set_glue_on, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the posts: at y = +-POST_Y, their inner faces at +-FACE ----------------------------------------------------------
RX = 0.32  # the posts' and the hung rail's x
POST_Y, FACE, POST_W, POST_H = 0.20, 0.19, 0.02, 0.12  # low posts: the arms reach over them
ZR = TOP + 0.08  # the rail's axis when hung
PEG_DZ = 0.009  # each bracket's two pegs: this far above and below the rail's axis
HOLE = 0.014  # the posts' square holes
PEG, PEG_L = 0.006, 0.010

# -- the rail, in its hung place (y along the rail); it starts pushed short and lying in front of the posts ---------
OUT_Y = (-0.040, FACE - 0.004)  # the outer channel, up to its front bracket's plate
IN_Y = (-FACE + 0.004, 0.074)  # the inner channel (pulled out), from its rear bracket's plate
OUT_C = np.array([RX, (OUT_Y[0] + OUT_Y[1]) / 2, ZR])
IN_C = np.array([RX, (IN_Y[0] + IN_Y[1]) / 2, ZR])
EXT = 0.07  # pushed short by this much at the start
START = np.array([0.24 - RX, -0.02, TOP + 0.022 - ZR])  # where the rail lies at the start, from its hung place
K_T, SLIP_T, D_T = 1500.0, 0.001, 4.0  # the inner channel's slide: about 1.5 N to move it
ANCHOR_AT = np.array([RX, 0.0, TOP - 0.05])  # a hidden fixed body under the table that the latched rail is held to

# -- scoring ------------------------------------------------------------------------------------------------------------
IN_DEPTH = 0.005  # a peg is in its hole once its tip is this far past the post's face
LEVEL_DEG = 5.0
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

STEEL, ZINC, DARK, ORANGE, WHITE = (0.30, 0.31, 0.33), (0.70, 0.72, 0.74), (0.08, 0.08, 0.09), (0.95, 0.50, 0.10), \
    (0.95, 0.95, 0.95)


def _post(name: str, side: float) -> Body:
    """A rack post: a square steel tube with a pair of square holes (along y) at the rail's height; origin on the table
    under its middle."""
    yc = side * POST_Y
    zr = ZR - TOP
    lo, hi = zr - PEG_DZ - HOLE / 2, zr + PEG_DZ + HOLE / 2  # the holes' span
    mid = (zr - PEG_DZ + HOLE / 2, zr + PEG_DZ - HOLE / 2)  # the bar between them
    side_w = (POST_W - HOLE) / 2
    shapes = [BoxShape((0.0, 0.0, 0.003), (0.06, 0.05, 0.006), STEEL),  # its foot
              BoxShape((0.0, 0.0, lo / 2), (POST_W, POST_W, lo), STEEL),
              BoxShape((0.0, 0.0, (mid[0] + mid[1]) / 2), (POST_W, POST_W, mid[1] - mid[0]), STEEL),
              BoxShape((0.0, 0.0, (hi + POST_H) / 2), (POST_W, POST_W, POST_H - hi), STEEL)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (HOLE / 2 + side_w / 2), 0.0, (lo + hi) / 2), (side_w, POST_W, hi - lo), STEEL))
    face = -side * (POST_W / 2 + 0.0002)  # decoration: more square holes up and down its inner face
    for k in range(-3, 5):
        z = zr + k * 0.025
        if lo - 0.01 < z < hi + 0.01 or z < 0.012 or z > POST_H - 0.01:
            continue
        shapes.append(BoxShape((0.0, face, z), (0.009, 0.0004, 0.009), DARK, collide=False))
    return Body.build(name, pos=(RX, yc, TOP), mass=2.0, kinematic=True, color=STEEL, shapes=shapes)


def _outer() -> Body:
    """The outer channel with its front bracket; origin in the channel's middle."""
    yc = OUT_C[1]
    plate_y = FACE - 0.002 - yc
    return Body.build("outer", pos=tuple((OUT_C + START).tolist()), mass=0.15, friction=(1.0, 0.9), color=ZINC,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.014, OUT_Y[1] - OUT_Y[0], 0.030), ZINC),
                              BoxShape((0.0, plate_y, 0.0), (0.030, 0.004, 0.044), ZINC),
                              BoxShape((0.0, plate_y + 0.002 + PEG_L / 2, PEG_DZ), (PEG, PEG_L, PEG), ZINC),
                              BoxShape((0.0, plate_y + 0.002 + PEG_L / 2, -PEG_DZ), (PEG, PEG_L, PEG), ZINC),
                              BoxShape((0.0, plate_y - 0.03, 0.0151), (0.010, 0.03, 0.0004), WHITE, collide=False)])


def _inner() -> Body:
    """The inner channel with its rear bracket and latch; origin in the channel's middle."""
    yc = IN_C[1]
    plate_y = -FACE + 0.002 - yc
    return Body.build("inner", pos=tuple((IN_C + START + [0.0, EXT, 0.0]).tolist()), mass=0.10, friction=(1.0, 0.9),
                      color=ZINC, shapes=[
                          BoxShape((0.0, 0.0, 0.0), (0.010, IN_Y[1] - IN_Y[0], 0.022), ZINC),
                          BoxShape((0.0, plate_y, 0.0), (0.030, 0.004, 0.044), ZINC),
                          BoxShape((0.0, plate_y - 0.002 - PEG_L / 2, PEG_DZ), (PEG, PEG_L, PEG), ZINC),
                          BoxShape((0.0, plate_y - 0.002 - PEG_L / 2, -PEG_DZ), (PEG, PEG_L, PEG), ZINC),
                          BoxShape((0.0, plate_y + 0.006, 0.016), (0.008, 0.008, 0.010), ORANGE)])  # the latch


def _anchor() -> Body:
    return Body.build("lock_anchor", pos=tuple(ANCHOR_AT.tolist()), mass=1.0, kinematic=True, color=DARK,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.01, 0.01, 0.01), DARK, collide=False)])


OBJECTS = [_post("left_post", 1.0), _post("right_post", -1.0), _anchor(), _outer(), _inner()]
slider("telescope", "outer", "inner", anchor=tuple((IN_C + START + [0.0, EXT, 0.0]).tolist()), axis="Y",
       limits=(-EXT - 0.012, 0.0), stiffness=K_T, damping=D_T, target=0.0)
glue("rail_lock", "lock_anchor", "outer", anchor=tuple(ANCHOR_AT.tolist()))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": 0.0, "front_ever": False, "locked": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("RAIL_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("rail_lock", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


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


def _lock_here(o_pos, o_rot) -> None:
    """The latch: hold the outer channel where it is (to the hidden fixed body), and the inner one in it."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS["rail_lock"]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in np.asarray(o_pos) - ANCHOR_AT]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(o_rot)))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    set_glue_on("rail_lock", True)


def _pegs_in(pos, rot, tip_y: float, side: float) -> tuple[float, bool]:
    """(how deep the shallower peg is past its post's face, whether both pegs are inside their holes' outline)."""
    depth, inside = 1.0, True
    for dz in (PEG_DZ, -PEG_DZ):
        tip = pos + rot @ np.array([0.0, tip_y, dz])
        depth = min(depth, side * float(tip[1]) - FACE)
        inside = inside and abs(float(tip[0]) - RX) <= (HOLE - PEG) / 2 + 0.001 and \
            abs(float(tip[2]) - (ZR + dz)) <= (HOLE - PEG) / 2 + 0.001
    return depth, inside


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    o = as_numpy(base.scene["outer"].data.root_link_pose_w)[0]
    i = as_numpy(base.scene["inner"].data.root_link_pose_w)[0]
    o_pos, o_rot, i_pos, i_rot = o[:3] - origin, rotation(o[3:7]), i[:3] - origin, rotation(i[3:7])
    slid = float(o_rot[:, 1] @ (i_pos - o_pos)) - (IN_C[1] + EXT - OUT_C[1])  # the inner pulled out (negative)
    if not _EP["locked"] and abs(slid - _EP["hold"]) > SLIP_T:  # pulled or pushed: it slides, and stays where left
        _EP["hold"] = slid
        set_slider_drive("telescope", target=slid)
    f_depth, f_inside = _pegs_in(o_pos, o_rot, FACE + PEG_L - OUT_C[1], 1.0)  # each peg's tip, in its channel's frame
    r_depth, r_inside = _pegs_in(i_pos, i_rot, -FACE - PEG_L - IN_C[1], -1.0)
    front = f_depth >= IN_DEPTH and f_inside
    rear = r_depth >= IN_DEPTH and r_inside
    _EP["front_ever"] = _EP["front_ever"] or front
    if not _EP["locked"] and front and rear:  # the rear latch clicks: the rail is locked between the posts
        _EP["locked"] = True
        _lock_here(o_pos, o_rot)
        set_slider_drive("telescope", target=slid, stiffness=30000.0)
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    a = o_pos + o_rot @ np.array([0.0, FACE - OUT_C[1], 0.0])  # the rail, from its front bracket ...
    b = i_pos + i_rot @ np.array([0.0, -FACE - IN_C[1], 0.0])  # ... to its rear one

    def to_rail(t):
        u = np.clip(float((t - a) @ (b - a)) / float((b - a) @ (b - a)), 0.0, 1.0)
        return float(np.linalg.norm(t - (a + u * (b - a))))

    clear = all(to_rail(t) >= LET_GO for t in tcps)
    level = math.degrees(math.acos(min(1.0, abs(float(o_rot[1, 1]))))) <= LEVEL_DEG
    _EP["s"] = {"clear": clear, "level": level, "front": front, "rear": rear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[rail] t={_DEBUG['steps'] / 20:5.1f}s slid={slid * 1000:6.1f}mm front={f_depth * 1000:6.1f}mm "
                  f"({f_inside}) rear={r_depth * 1000:6.1f}mm ({r_inside}) locked={_EP['locked']} level={level} "
                  f"clear={clear} outer={np.round(o_pos * 1000).astype(int).tolist()} "
                  f"inner={np.round(i_pos * 1000).astype(int).tolist()} "
                  f"leftTCP-outer={np.round((tl - o_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-inner={np.round((tr - i_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"locked": _EP["locked"], "front_ever": _EP["front_ever"], "clear": bool(st.get("clear")),
            "level": bool(st.get("level")), "home": _arms_home()}


def _points(s) -> float:
    if s["locked"] and s["level"] and s["clear"] and s["home"]:
        return 1.0
    if s["locked"]:
        return 0.5
    return 0.25 if s["front_ever"] else 0.0


def score(objects) -> float:
    """1.0 with the rail latched between the posts (both brackets' pegs at least 5 mm into their holes at once, which locks
    it), level within 5 degrees and let go of (each TCP at least 4 cm from the rail) and both arms back where they
    started (each TCP within 5 cm of its start); 0.5 with the rail latched; 0.25 if the front pegs were in the left
    post's holes at some point; else 0.0. Once latched, it stays latched."""
    return _points(_status())


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
    """Full score with both channels still."""
    return (score(objects) >= 1.0 and objects["outer"].resting() and objects["inner"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come straight down on the rail where it lies, fingers closing across it: the left on the outer channel,
# the right on the end of the inner one. Together they lift it to the holes' height, carry it in between the posts and
# push it toward the left post, so the front pegs go into its holes. The left keeps it there while the right pulls the
# inner channel out until the rear pegs go into the right post's holes and latch. Both let go, rise and go back to
# where they started.

L_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the channel)
R_GRIP = grip_rotation(math.pi / 2)
L_AT, R_AT = 0.13, -0.15  # where each hand holds the rail, along y in its hung place (wrists 21 cm or more apart)
L_IN, R_IN = 0.012, 0.010  # each TCP this far below its channel's top
L_OPEN, R_OPEN = -0.02, -0.02  # fingers part open (4 cm) round the channels
SHIFT_IN = -START[1]  # pushed this far toward the left post once between the posts
PRESS = 0.004  # the left aims this much past the front bracket's stop
PULL_PAST = 0.020  # the right aims this much past the rear bracket's stop
LEAD_S, LEAD_STEPS = 3.0, 15
OFF = {"left": np.array([-0.004, 0.0, 0.0]), "right": np.array([0.0, 0.0, 0.0])}  # where the carried rail sits off its planned place (measured: the arms lag; aimed against)


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


def _plan(arm: str) -> list[Waypoint]:
    from yam_arena.constants import READY

    left = arm == "left"
    grip = L_GRIP if left else R_GRIP
    hung = np.array([RX, L_AT if left else R_AT, ZR + (0.015 - L_IN if left else 0.011 - R_IN)])
    lying = hung + START + ([0.0, 0.0, 0.0] if left else [0.0, EXT, 0.0])  # the inner is pushed short at the start
    carried = hung + ([0.0, -SHIFT_IN, 0.0] if left else [0.0, EXT - SHIFT_IN, 0.0]) - OFF[arm]
    between = carried + [0.0, SHIFT_IN + (PRESS if left else 0.0), 0.0]
    part_open = L_OPEN if left else R_OPEN
    out = _joint_path(arm, np.array(READY, float), _good_ik(arm, lying + [0.0, 0.0, 0.05], grip),
                      "over the rail", part_open)
    out += [_w(1.0, lying, part_open, "down round the rail", grip),
            _w(0.5, lying, FINGER_CLOSED, "take it", grip),
            _w(1.5, [lying[0], lying[1], carried[2]], FINGER_CLOSED, "lift it to the holes' height", grip),
            _w(2.0, carried, FINGER_CLOSED, "carry it in between the posts", grip),
            _w(1.5, between, FINGER_CLOSED, "the front pegs into the left post's holes", grip)]
    pulled = between - [0.0, EXT + PULL_PAST, 0.0]
    if left:
        out += [_w(3.6, between, FINGER_CLOSED, "hold the rail on the left post", grip)]
        end = between
    else:
        out += [_w(3.0, pulled, FINGER_CLOSED, "pull the inner channel out: the rear pegs latch in the right post",
                   grip),
                _w(0.6, pulled, FINGER_CLOSED, "hold", grip)]
        end = pulled
    up = end + [0.0, 0.0, 0.05]
    out += [_w(0.5, end, FINGER_OPEN, "let go", grip),
            _w(0.8, up, FINGER_OPEN, "rise", grip)]
    out += _joint_path(arm, _good_ik(arm, up, grip), np.array(READY, float), "back to where it started", FINGER_OPEN)
    out.append(_w(3.0, out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out


def demo():
    return {"left": _plan("left"), "right": _plan("right")}
