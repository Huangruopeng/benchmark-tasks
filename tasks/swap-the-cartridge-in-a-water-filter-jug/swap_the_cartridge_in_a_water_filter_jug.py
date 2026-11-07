"""On the station table stands an empty water filter jug, its lid off, its handle toward the left arm. In the seat at the
bottom of its top reservoir is an old, grey filter cartridge, wedged in tight. Beside the jug stand a new white
cartridge with a blue cap and a small waste bin.

Task: change the cartridge. The jug weighs 200 g and the old cartridge takes about 2.5 N to pull out of its seat, so
pulling it alone lifts the whole jug: one hand holds the jug down by its handle while the other pulls the old cartridge
out and drops it in the bin. Then the new cartridge goes into the seat and is pushed down until it is seated. Let go of
everything and take both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_the_cartridge_in_a_water_filter_jug.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the jug: a hollow box, its handle toward the left arm (+y) ------------------------------------------------------------
JX, JY = 0.25, 0.0  # the jug's middle
JUG_X, JUG_Y = (-0.06, 0.06), (-0.08, 0.08)  # its outside, from its middle
JUG_H, WALL = 0.090, 0.004
PLATE_Z = (0.052, 0.056)  # the reservoir's floor, with the cartridge's seat in it
SEAT = np.array([0.0, 0.0, 0.0])  # the seat's axis (jug frame, x and y): in the middle, over the jug's centre of mass
HOLE = 0.021  # half the seat hole's width
HANDLE_GRIP_Y = 0.150  # the handle's top bar, where it is held (jug frame)
HANDLE_END = 0.165  # the handle's far end
JUG_MASS = 0.20

# -- the cartridges: origin on the axis at the bottom ----------------------------------------------------------------
CART_R, BODY_H = 0.019, 0.043  # the body (below the cap)
CAP_H, CAP_R = 0.020, 0.0195
COLLAR_Z, COLLAR_R = PLATE_Z[1] - 0.045, 0.024  # the collar rests on the reservoir's floor when seated
SEATED_Z = 0.045  # a seated cartridge's bottom, above the table
CART_MASS = 0.08
K_OLD, K_NEW, SLIP, D_SEAT = 2500.0, 2500.0, 0.001, 6.0  # the seat's grip (about 2.5 N: more than the jug weighs)
FREE_AT = 0.020  # pulled up this far, a cartridge is out of the seat
SEATED_WITHIN = 0.004
CAPTURE_R, CAPTURE_UP, CAPTURE_DEG = 0.008, 0.022, 12.0

NEW_AT = (0.17, -0.17)  # where the new cartridge stands
BIN_AT, BIN_IN, BIN_H = (0.34, -0.25), 0.046, 0.07  # the bin: middle, inside half-width, height

LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

JUG_C, WHITE, OLD_C, OLD_CAP, BLUE, BIN_C, DARK = (0.72, 0.84, 0.92), (0.95, 0.95, 0.94), (0.56, 0.52, 0.45), \
    (0.36, 0.34, 0.31), (0.18, 0.42, 0.78), (0.30, 0.31, 0.33), (0.12, 0.12, 0.12)


def _jug() -> Body:
    """The jug: walls, floor, the reservoir's floor with a square seat hole, and a D-handle; origin on the table."""
    (x0, x1), (y0, y1), h, w = JUG_X, JUG_Y, JUG_H, WALL
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    shapes = [BoxShape((cx, cy, 0.0025), (x1 - x0, y1 - y0, 0.005), JUG_C),  # the floor
              BoxShape((x0 + w / 2, cy, h / 2), (w, y1 - y0, h), JUG_C),
              BoxShape((x1 - w / 2, cy, h / 2), (w, y1 - y0, h), JUG_C),
              BoxShape((cx, y0 + w / 2, h / 2), (x1 - x0 - 2 * w, w, h), JUG_C),
              BoxShape((cx, y1 - w / 2, h / 2), (x1 - x0 - 2 * w, w, h), JUG_C),
              BoxShape((cx, y0 + 0.004, h - 0.002), (0.02, 0.012, 0.004), JUG_C)]  # the spout's lip
    pz, pt = (PLATE_Z[0] + PLATE_Z[1]) / 2, PLATE_Z[1] - PLATE_Z[0]
    ix0, ix1, iy0, iy1 = x0 + w, x1 - w, y0 + w, y1 - w
    sx, sy = SEAT[0], SEAT[1]
    shapes += [BoxShape(((ix0 + sx - HOLE) / 2, (iy0 + iy1) / 2, pz), (sx - HOLE - ix0, iy1 - iy0, pt), WHITE),
               BoxShape(((sx + HOLE + ix1) / 2, (iy0 + iy1) / 2, pz), (ix1 - sx - HOLE, iy1 - iy0, pt), WHITE),
               BoxShape((sx, (iy0 + sy - HOLE) / 2, pz), (2 * HOLE, sy - HOLE - iy0, pt), WHITE),
               BoxShape((sx, (sy + HOLE + iy1) / 2, pz), (2 * HOLE, iy1 - sy - HOLE, pt), WHITE)]
    hx = 0.016
    e = HANDLE_END
    shapes += [BoxShape((0.0, (y1 + e) / 2, h - 0.006), (hx, e - y1, 0.012), JUG_C),  # the handle's top bar
               BoxShape((0.0, e - 0.005, 0.052), (hx, 0.010, 0.064), JUG_C),  # its side
               BoxShape((0.0, (y1 + e) / 2, 0.025), (hx, e - y1, 0.010), JUG_C)]  # its foot
    return Body.build("jug", pos=(JX, JY, TOP), mass=JUG_MASS, shapes=shapes, friction=(0.6, 0.5), color=JUG_C,
                      com=(0.0, 0.0, 0.03))


def _cartridge(name: str, at, body_c, cap_c) -> Body:
    shapes = [CylShape((0.0, 0.0, BODY_H / 2), CART_R, BODY_H, body_c, axis="Z"),
              CylShape((0.0, 0.0, COLLAR_Z + 0.002), COLLAR_R, 0.004, body_c, axis="Z"),  # the collar
              CylShape((0.0, 0.0, BODY_H + CAP_H / 2), CAP_R, CAP_H, cap_c, axis="Z")]  # the cap it is held by
    for k in range(4):  # the inlet slots round the body
        a = k * math.pi / 2
        shapes.append(BoxShape((CART_R * math.cos(a), CART_R * math.sin(a), 0.012), (0.004, 0.004, 0.014), DARK,
                               collide=False))
    return Body.build(name, pos=at, mass=CART_MASS, shapes=shapes, friction=(1.0, 0.9), color=body_c,
                      com=(0.0, 0.0, 0.025))


def _bin() -> Body:
    bx, by = BIN_AT
    o, w, h = BIN_IN + 0.004, 0.004, BIN_H
    shapes = [BoxShape((0.0, 0.0, 0.002), (2 * o, 2 * o, 0.004), BIN_C),
              BoxShape((-o + w / 2, 0.0, h / 2), (w, 2 * o, h), BIN_C), BoxShape((o - w / 2, 0.0, h / 2), (w, 2 * o, h), BIN_C),
              BoxShape((0.0, -o + w / 2, h / 2), (2 * o - 2 * w, w, h), BIN_C),
              BoxShape((0.0, o - w / 2, h / 2), (2 * o - 2 * w, w, h), BIN_C)]
    return Body.build("bin", pos=(bx, by, TOP), mass=0.5, kinematic=True, color=BIN_C, shapes=shapes)


SEAT_W = np.array([JX + SEAT[0], JY + SEAT[1], TOP + SEATED_Z])  # a seated cartridge's origin, at the start
OBJECTS = [_bin(), _jug(), _cartridge("old_cartridge", tuple(SEAT_W.tolist()), OLD_C, OLD_CAP),
           _cartridge("new_cartridge", (NEW_AT[0], NEW_AT[1], TOP), WHITE, BLUE)]
for _name, _k in (("old_seat", K_OLD), ("new_seat", K_NEW)):
    # the slide runs DOWN into the seat (bodykit authors the drive's target at the slide's upper limit: 2 mm in, here)
    thread(_name, "jug", _name.split("_")[0] + "_cartridge", anchor0=(SEAT[0], SEAT[1], SEATED_Z),
           axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, -1.0), slide_limits=(-0.04, 0.002),
           turn_damping=0.5, slide_stiffness=_k, slide_damping=D_SEAT, slide_max_force=60.0, play=0.001,
           tilt_deg=3.0, always_on=(_name == "old_seat"))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"t": 0, "old_on": True, "old_hold": -0.002, "old_out": False, "new_on": False, "new_hold": 0.0,
            "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("JUG_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    set_slide_target("old_seat", 0.002)
    set_thread_on("old_seat", True)
    set_slide_target("new_seat", 0.002)
    set_thread_on("new_seat", False)


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


def _seat_coords(j_pos, j_rot, c_pos, c_rot) -> tuple[float, float, float]:
    """A cartridge against the seat: how far it is up from seated, off the seat's axis, and tilted (deg)."""
    q = j_rot.T @ (c_pos - j_pos) - np.array([SEAT[0], SEAT[1], SEATED_Z])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(j_rot[:, 2] @ c_rot[:, 2])))))
    return float(q[2]), math.hypot(float(q[0]), float(q[1])), tilt


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["t"] += 1
    j_pos, j_rot = _pose(base, origin, "jug")
    o_pos, o_rot = _pose(base, origin, "old_cartridge")
    n_pos, n_rot = _pose(base, origin, "new_cartridge")
    o_up, o_off, _ = _seat_coords(j_pos, j_rot, o_pos, o_rot)
    n_up, n_off, n_tilt = _seat_coords(j_pos, j_rot, n_pos, n_rot)
    if _EP["old_on"]:
        if o_up >= FREE_AT:  # pulled clear of the seat
            set_thread_on("old_seat", False)
            _EP["old_on"], _EP["old_out"] = False, True
        elif abs(o_up - _EP["old_hold"]) > SLIP:  # pulled (or pushed) past its hold: it slides, and stays
            _EP["old_hold"] = o_up
            set_slide_target("old_seat", -o_up)  # (the slide runs down: its position is minus the rise)
    if not _EP["new_on"]:
        if not _EP["old_on"] and n_off <= CAPTURE_R and 0.0 <= n_up <= CAPTURE_UP and n_tilt <= CAPTURE_DEG:
            set_slide_target("new_seat", -n_up)
            set_thread_on("new_seat", True)
            _EP["new_on"], _EP["new_hold"] = True, n_up
    elif n_up >= FREE_AT + 0.005:  # pulled back out
        set_thread_on("new_seat", False)
        _EP["new_on"] = False
    elif abs(n_up - _EP["new_hold"]) > SLIP:
        _EP["new_hold"] = n_up
        set_slide_target("new_seat", -n_up)
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    handle = j_pos + j_rot @ np.array([0.0, HANDLE_GRIP_Y, JUG_H - 0.006])
    caps = [p + r @ np.array([0.0, 0.0, BODY_H + CAP_H / 2]) for p, r in ((o_pos, o_rot), (n_pos, n_rot))]
    clear = all(min(float(np.linalg.norm(t - x)) for x in [handle] + caps) >= LET_GO for t in tcps)
    in_bin = (abs(float(o_pos[0]) - BIN_AT[0]) <= BIN_IN and abs(float(o_pos[1]) - BIN_AT[1]) <= BIN_IN
              and float(o_pos[2]) - TOP <= BIN_H)
    jug_ok = float(j_rot[2, 2]) >= math.cos(math.radians(15.0)) and float(j_pos[2]) - TOP <= 0.01
    seated = _EP["new_on"] and n_up <= SEATED_WITHIN
    old_away = not _EP["old_on"] and not (JUG_X[0] <= float((j_rot.T @ (o_pos - j_pos))[0]) <= JUG_X[1]
                                         and JUG_Y[0] <= float((j_rot.T @ (o_pos - j_pos))[1]) <= JUG_Y[1])
    _EP["s"] = {"in_bin": in_bin, "seated": seated, "old_away": old_away, "jug_ok": jug_ok, "clear": clear}
    if _DEBUG["on"] and _EP["t"] % 10 == 0:
        tl, tr = tcps
        print(f"[jug] t={_EP['t'] / 20:5.1f}s old_up={o_up * 1000:5.1f}mm old_on={_EP['old_on']} out={_EP['old_out']} "
              f"in_bin={in_bin} new_up={n_up * 1000:6.1f} off={n_off * 1000:4.1f} tilt={n_tilt:4.1f} new_on={_EP['new_on']} "
              f"seated={seated} jug={np.round((j_pos - [JX, JY, TOP]) * 1000).astype(int).tolist()} jug_ok={jug_ok} "
              f"L-handle={np.round((tl - handle) * 1000).astype(int).tolist()} "
              f"R-oldcap={np.round((tr - caps[0]) * 1000).astype(int).tolist()} "
              f"R-newcap={np.round((tr - caps[1]) * 1000).astype(int).tolist()} clear={clear} home={_arms_home()} "
              f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"out": _EP["old_out"], "in_bin": bool(st.get("in_bin")), "seated": bool(st.get("seated")),
            "old_away": bool(st.get("old_away")), "jug_ok": bool(st.get("jug_ok")), "clear": bool(st.get("clear")),
            "home": _arms_home()}


def _points(s) -> float:
    if s["in_bin"] and s["seated"] and s["jug_ok"] and s["clear"] and s["home"]:
        return 1.0
    if s["seated"] and s["old_away"]:
        return 0.5
    return 0.25 if s["out"] else 0.0


def score(objects) -> float:
    """1.0 with the old cartridge in the bin, the new one seated in the jug (in the seat, within 4 mm of fully down), the
    jug upright on the table, let go (each TCP at least 4 cm from the jug's handle and both cartridges' caps) and both
    arms back where they started (each TCP within 5 cm of its start); 0.5 with the new cartridge seated and the old one
    out of the jug; 0.25 with the old cartridge pulled out of its seat (2 cm up) at some point; else 0.0. All but the
    pull-out are judged at the end."""
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
    """Full score with the jug and both cartridges still."""
    return (score(objects) >= 1.0 and objects["jug"].resting() and objects["old_cartridge"].resting()
            and objects["new_cartridge"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the jug's handle, fingers closing across its top bar, and holds the jug down.
# The right comes down on the old cartridge's cap, fingers closing across it, pulls the cartridge up out of its seat and
# clear of the jug, carries it over the bin and drops it in. It takes the new cartridge by its cap, carries it over the
# seat, lowers it in and pushes it down until it is seated, lets go and goes back to where it started; the left lets
# go of the handle and goes back to where it started.

L_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the handle's bar)
R_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (across a cap)
HANDLE_OPEN = -0.014  # about 2.8 cm round the 16 mm bar
CAP_OPEN = -0.0255  # about 5.1 cm round the 39 mm cap
CAP_IN = 0.013  # the right TCP this far below a cap's top
LIFT_TO = 0.160  # the right TCP's height (above the table) carrying a cartridge
PUSH_PAST = 0.012  # pushing a cartridge down, aimed this far past seated (the arm gives)
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


def _left(release_at: float) -> tuple[list[Waypoint], float]:
    grip = np.array([JX, JY + HANDLE_GRIP_Y, TOP + JUG_H - 0.004])
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], L_GRIP, "over the jug's handle", HANDLE_OPEN)
    out += [_w(1.0, grip, HANDLE_OPEN, "down round the handle", L_GRIP),
            _w(0.5, grip, FINGER_CLOSED, "hold the jug down by its handle", L_GRIP)]
    held_at = sum(w.duration for w in out)
    out.append(_w(max(release_at - held_at, 0.3), grip, FINGER_CLOSED, "keep holding the jug", L_GRIP))
    out += [_w(0.5, grip, HANDLE_OPEN, "let go", L_GRIP),
            _w(0.8, grip + [0.0, 0.0, 0.05], HANDLE_OPEN, "rise", L_GRIP)]
    out += _go_home("left", grip + [0.0, 0.0, 0.05], L_GRIP, FINGER_OPEN)
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    cap_top = SEATED_Z + BODY_H + CAP_H
    old = np.array([JX + SEAT[0], JY + SEAT[1], TOP + cap_top - CAP_IN])
    out = _lead_in("right", old + [0.0, 0.0, 0.04], R_GRIP, "over the old cartridge", CAP_OPEN)
    t = sum(w.duration for w in out)
    out += [_w(max(start - t, 0.3), old + [0.0, 0.0, 0.04], CAP_OPEN, "wait for the jug to be held", R_GRIP),
            _w(1.0, old, CAP_OPEN, "down round its cap", R_GRIP),
            _w(0.5, old, FINGER_CLOSED, "take it by its cap", R_GRIP),
            _w(2.0, old + [0.0, 0.0, 0.030], FINGER_CLOSED, "pull it up out of its seat", R_GRIP),
            _w(1.2, [old[0], old[1], TOP + LIFT_TO], FINGER_CLOSED, "lift it clear of the jug", R_GRIP)]
    over_bin = np.array([BIN_AT[0], BIN_AT[1], TOP + LIFT_TO])
    out += [_w(1.8, over_bin, FINGER_CLOSED, "carry it over the bin", R_GRIP),
            _w(0.5, over_bin, CAP_OPEN, "drop it in the bin", R_GRIP)]
    new = np.array([NEW_AT[0], NEW_AT[1], TOP + BODY_H + CAP_H - CAP_IN])
    out += [_w(1.6, new + [0.0, 0.0, 0.05], CAP_OPEN, "over the new cartridge", R_GRIP),
            _w(1.0, new, CAP_OPEN, "down round its cap", R_GRIP),
            _w(0.5, new, FINGER_CLOSED, "take it by its cap", R_GRIP),
            _w(1.2, [new[0], new[1], TOP + LIFT_TO], FINGER_CLOSED, "lift it", R_GRIP)]
    seat_top = np.array([old[0], old[1], TOP + LIFT_TO])
    into = old + [0.0, 0.0, 0.012]
    pushed = old + [0.0, 0.0, -PUSH_PAST]
    out += [_w(1.8, seat_top, FINGER_CLOSED, "carry it over the seat", R_GRIP),
            _w(1.5, into, FINGER_CLOSED, "lower it into the seat", R_GRIP),
            _w(1.5, pushed, FINGER_CLOSED, "push it down until it is seated", R_GRIP),
            _w(0.5, pushed, CAP_OPEN, "let go", R_GRIP),
            _w(0.8, pushed + [0.0, 0.0, 0.05], CAP_OPEN, "rise", R_GRIP)]
    done = sum(w.duration for w in out)
    out += _go_home("right", pushed + [0.0, 0.0, 0.05], R_GRIP, FINGER_OPEN)
    return out, done


def demo():
    _, held_at = _left(0.0)
    right, done = _right(held_at)
    left, _ = _left(done)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        t = sum(w.duration for w in arm)
        arm.append(_w(total - t, arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s with both arms home", arm[-1].yaw))
    return {"left": left, "right": right}
