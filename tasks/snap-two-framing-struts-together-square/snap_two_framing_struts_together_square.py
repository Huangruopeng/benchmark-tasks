"""On the station table lie two lengths of 20 mm square aluminium framing strut. The first (A) runs away from the
arms; its near end carries a die-cast corner connector whose square peg points to the right, with a spring catch on
the peg's top. The second (B) lies to the right, square to A, its hollow end facing the peg, 1.5 cm short of it. Both
are light and slide easily on the table.

Task: join them into a square corner. One hand holds strut A still while the other pushes strut B's hollow end straight
onto the peg until the catch snaps up into the hole in B's wall (the catch has to be pressed down on the way, and a
strut nobody holds just slides away from the push). Then strut A is let go and strut B is lifted at its free end, so the
whole corner rises as one piece, showing the joint holds; the frame is set back down, let go, and both arms go back to
where they started. The corner must end square, within 3 degrees.

Run it:  inspect-robots run --embodiment yam_arena -E task=snap_two_framing_struts_together_square.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

S = 0.020  # the struts' section
ZC = TOP + S / 2  # their axes' height, lying on the table

# -- strut A: 26 cm along x; its near end carries the connector, the peg pointing -y ---------------------------------
A_X, A_Y = (0.20, 0.46), 0.07
PEG = (0.011, 0.040)  # its square section and length
PEG_X = A_X[0] + S / 2  # the peg's axis
PEG_ROOT = A_Y - S / 2  # the peg starts at A's right side ...
PEG_TIP = PEG_ROOT - PEG[1]  # ... and ends here
CATCH_Y = PEG_ROOT - 0.015  # the catch, on the peg's top
CATCH = (0.006, 0.006, 0.0045)  # across, along, tall (2.5 mm stands above the peg)
CATCH_UP = 0.003
K_CATCH = 1500.0  # N/m

# -- strut B: 24 cm along y, its hollow end (the socket) facing +y -------------------------------------------------------
B_LEN = 0.24
B_GAP = 0.015  # its end this far short of the peg's tip at the start
B_END0 = PEG_TIP - B_GAP  # its open end's y at the start
SOCKET = 0.046  # the socket's depth
BORE = 0.014  # its inside (the peg is 11 mm: 1.5 mm all round)
HOLE = (0.011, 0.019)  # the hole in its top wall, as distances in from its open end

# -- the jig: two low steel guide strips either side of strut B's end, in line with the peg ---------------------------
JIG_LEN, JIG_W, JIG_H, JIG_PLAY = 0.034, 0.004, 0.016, 0.0015

ALU, DARK, ZINC, RED = (0.78, 0.79, 0.81), (0.35, 0.36, 0.38), (0.55, 0.56, 0.58), (0.80, 0.12, 0.10)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_ON = 0.002  # B's end within this of A's side: fully on
SQUARE_DEG = 3.0
RAISED = 0.025  # A's near end this far off the table, lifted through the joint
LET_GO = 0.05
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _strut_a() -> Body:
    """Strut A with its connector; origin at its middle."""
    cx = (A_X[0] + A_X[1]) / 2
    o = np.array([cx, A_Y, ZC])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    shapes = [BoxShape(at(cx + 0.010, A_Y, ZC), (A_X[1] - A_X[0] - 0.020, S, S), ALU),  # the strut
              BoxShape(at(A_X[0] + 0.010, A_Y, ZC), (0.020, S, S), ZINC),  # the connector's body, on its end
              BoxShape(at(PEG_X, (PEG_ROOT + PEG_TIP + PEG[0] / 2) / 2, ZC), (PEG[0], PEG[1] - PEG[0] / 2, PEG[0]),
                       ZINC),
              BoxShape(at(PEG_X, PEG_TIP + PEG[0] / 2, ZC), (PEG[0] / math.sqrt(2), PEG[0] / math.sqrt(2), PEG[0]), ZINC,
                       yaw=math.pi / 4)]  # its tip, pointed (wedge-shaped seen from above)
    for z in (ZC - S / 2 + 0.0002, ZC + S / 2 - 0.0002):  # the slots' dark lines along the top and bottom
        shapes.append(BoxShape(at(cx + 0.010, A_Y, z), (A_X[1] - A_X[0] - 0.024, 0.006, 0.0006), DARK, collide=False))
    return Body.build("strut_a", pos=tuple(o.tolist()), mass=0.15, shapes=shapes, friction=(0.6, 0.5), color=ALU)


def _catch() -> Body:
    """The spring catch on the peg's top: origin on the peg's top face; its face toward the tip bevelled."""
    q = (math.sin(-math.pi / 8), 0.0, 0.0, math.cos(-math.pi / 8))  # turned 45 degrees about x
    shapes = [BoxShape((0.0, 0.0010, CATCH_UP - CATCH[2] / 2), (CATCH[0], CATCH[1] - 0.002, CATCH[2]), RED),
              BoxShape((0.0, -CATCH[1] / 2 + 0.0015, CATCH_UP - 0.0028), (CATCH[0], 0.0042, 0.0042), RED,
                       quat_xyzw=q)]
    return Body.build("catch", pos=(PEG_X, CATCH_Y, ZC + PEG[0] / 2), mass=0.003, shapes=shapes,
                      friction=(0.15, 0.10), slick=True, color=RED)


def _strut_b() -> Body:
    """Strut B; origin at its open end's middle (the socket's mouth)."""
    w = (S - BORE) / 2
    shapes = [BoxShape((0.0, -(SOCKET + B_LEN) / 2, 0.0), (S, B_LEN - SOCKET, S), ALU),  # solid beyond the socket
              BoxShape((0.0, -SOCKET / 2, -(S - w) / 2), (S, SOCKET, w), ALU),  # the socket's floor
              *[BoxShape((s * (S - w) / 2, -SOCKET / 2, 0.0), (w, SOCKET, S), ALU) for s in (-1.0, 1.0)],
              BoxShape((0.0, -HOLE[0] / 2, (S - w) / 2), (S, HOLE[0], w), ALU),  # its roof, either side of the hole
              BoxShape((0.0, -(HOLE[1] + SOCKET) / 2, (S - w) / 2), (S, SOCKET - HOLE[1], w), ALU)]
    for z in (-S / 2 + 0.0002, S / 2 - 0.0002):
        shapes.append(BoxShape((0.0, -(SOCKET + B_LEN) / 2, z), (0.006, B_LEN - SOCKET - 0.004, 0.0006), DARK,
                               collide=False))
    return Body.build("strut_b", pos=(PEG_X, B_END0, ZC), mass=0.13, shapes=shapes, friction=(0.6, 0.5), color=ALU,
                      com=(0.0, -B_LEN / 2, 0.0))


def _jig() -> Body:
    """The framing jig's guide: two low steel strips either side of strut B's end, in line with the peg (1.5 mm play
    each side), so B's end runs straight onto the peg."""
    y0, y1 = PEG_TIP - JIG_LEN, PEG_TIP - 0.004
    o = np.array([PEG_X, (y0 + y1) / 2, TOP])
    shapes = []
    for sx in (-1.0, 1.0):
        xc = sx * (S / 2 + JIG_PLAY + JIG_W / 2)
        shapes.append(BoxShape((xc, 0.0, JIG_H / 2), (JIG_W, y1 - y0, JIG_H), DARK))
    shapes.append(BoxShape((0.0, 0.0, 0.0008), (S + 2 * JIG_PLAY + 2 * JIG_W, y1 - y0, 0.0016), DARK, collide=False))
    return Body.build("jig", pos=tuple(o.tolist()), mass=1.0, kinematic=True, color=DARK, shapes=shapes)


OBJECTS = [_strut_a(), _catch(), _strut_b(), _jig()]
slider("catch_spring", "strut_a", "catch", anchor=(PEG_X, CATCH_Y, ZC + PEG[0] / 2), axis="Z",
       limits=(-CATCH[2], 0.0), stiffness=K_CATCH, damping=1.0, target=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"clicked": False, "raised": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRUT_DEBUG")), "steps": 0}


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


A0 = np.array([(A_X[0] + A_X[1]) / 2, A_Y, ZC])


def _to_segment(p, a, b) -> float:
    """Distance from point p to the segment a-b."""
    ab = b - a
    t = max(0.0, min(1.0, float((p - a) @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(p - (a + t * ab)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    a_p, a_r = _pose(base, origin, "strut_a")
    b_p, b_r = _pose(base, origin, "strut_b")
    c_p, _ = _pose(base, origin, "catch")
    mouth = a_r.T @ (b_p - a_p)  # B's open end in A's frame
    depth = (PEG_ROOT - A0[1]) - float(mouth[1])  # how far B's end is still short of A's side (m); 0 when fully on
    catch_up = float((a_r.T @ (c_p - a_p))[2]) - (ZC + PEG[0] / 2 - A0[2])  # the catch's travel (0 = fully up)
    on = abs(depth) < HOME_ON + 0.001 and catch_up > -0.0008
    if on:
        _EP["clicked"] = True
    angle = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(a_r[:, 0] @ b_r[:, 1]))))))  # A's axis to B's
    _EP["tcps"] = tcps = _tcps(base, origin)
    near_end = a_p + a_r @ np.array([A_X[0] - A0[0], 0.0, 0.0])
    far_end = a_p + a_r @ np.array([A_X[1] - A0[0], 0.0, 0.0])
    b_far = b_p + b_r @ np.array([0.0, -B_LEN, 0.0])
    off_a = min(_to_segment(t, near_end, far_end) for t in tcps)
    off_b = min(_to_segment(t, b_p, b_far) for t in tcps)
    if on and near_end[2] > ZC + RAISED and off_a >= LET_GO:
        _EP["raised"] = True  # A lifted through the joint alone
    flat = abs(near_end[2] - ZC) < 0.004 and abs(far_end[2] - ZC) < 0.004 and abs(b_far[2] - ZC) < 0.004
    _EP["s"] = {"on": on, "square": abs(90.0 - angle) <= SQUARE_DEG, "flat": flat,
                "clear": off_a >= LET_GO and off_b >= LET_GO, "depth": depth}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[strut] t={_DEBUG['steps'] / 20:5.1f}s depth={depth * 1000:5.1f}mm catch={catch_up * 1000:5.1f}mm "
                  f"on={on} clicked={_EP['clicked']} angle={angle:5.1f} near_end_z={(near_end[2] - ZC) * 1000:5.1f}mm "
                  f"raised={_EP['raised']} flat={flat} ends_z(mm)={round((near_end[2] - ZC) * 1000, 1)},{round((far_end[2] - ZC) * 1000, 1)},{round((b_far[2] - ZC) * 1000, 1)} offA={off_a * 1000:.0f} offB={off_b * 1000:.0f} A={np.round(a_p * 1000).astype(int).tolist()} "
                  f"B={np.round(b_p * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} home={_arms_home()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"on": bool(st.get("on")), "square": bool(st.get("square")), "raised": _EP["raised"],
            "flat": bool(st.get("flat")), "clear": bool(st.get("clear")), "depth": st.get("depth", 1.0),
            "home": _arms_home()}


def _points(s) -> float:
    if s["on"] and s["square"] and s["raised"] and s["flat"] and s["clear"] and s["home"]:
        return 1.0
    if s["on"] and s["square"]:
        return 0.5
    return 0.25 if s["depth"] < PEG[1] - 0.010 else 0.0


def score(objects) -> float:
    """Judged at the end. Fully on: strut B's open end within 3 mm of strut A's side with the catch up in B's hole.
    Square: the two struts' axes 90 degrees apart, within 3 degrees. 1.0: fully on and square, the corner lifted at some
    point after it snapped on, by strut B alone (strut A's near end at least 2.5 cm off the table with each gripper's
    TCP at least 5 cm from strut A), the frame lying flat on the table again, each TCP at least 5 cm from both struts,
    and both arms back where they started (each TCP within 5 cm of its start); 0.5: fully on and square; 0.25: strut B
    at least 1 cm onto the peg; else 0.0."""
    return _points(_status())


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
    """Full score with both struts still."""
    return score(objects) >= 1.0 and objects["strut_a"].resting() and objects["strut_b"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on strut A near its far end, fingers closing across it, and holds it. The right
# hand takes strut B 13 cm from its open end, fingers closing across it, and slides it straight along the table onto
# the peg until the catch snaps up into its hole. The left lets go of strut A and goes home; the right lifts strut B
# 7 cm, so the corner rises with strut A hanging on the joint, holds it there, sets it back down, lets go and goes home.

A_GRIP = grip_rotation(0.0)  # left: straight down, fingers closing along y, across strut A
B_GRIP = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x, across strut B
PRE_OPEN = -(S / 2 + 0.006)
IN = 0.003  # the TCP this far below a strut's top
A_HOLD_X = 0.41
B_HOLD = 0.115  # strut B taken this far from its open end (the fingers clear of the jig)
OVERSHOOT = 0.030
LIFT = 0.07
SET_ASIDE = 0.045  # set the frame down this far along +x (away from the arms), strut B clear of the jig
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


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    hold = np.array([A_HOLD_X, A_Y, TOP + S - IN])
    out = _lead_in("left", hold + [0.0, 0.0, 0.05], A_GRIP, "over strut A's far end", PRE_OPEN)
    out += [_w(1.0, hold, PRE_OPEN, "down round it", A_GRIP),
            _w(0.6, hold, FINGER_CLOSED, "hold strut A", A_GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), hold, FINGER_CLOSED, "hold it still while strut B is pushed on", A_GRIP),
            _w(0.5, hold, FINGER_OPEN, "let go", A_GRIP),
            _w(0.8, hold + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", A_GRIP)]
    out += _go_home("left", hold + [0.0, 0.0, 0.06], A_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    take = np.array([PEG_X, B_END0 - B_HOLD, TOP + S - IN])
    home_on = take + [0.0, B_GAP + PEG[1], 0.0]
    g = B_GRIP
    out = _lead_in("right", take + [0.0, 0.0, 0.05], g, "over strut B", PRE_OPEN)
    out += [_w(1.0, take, PRE_OPEN, "down round it", g),
            _w(0.6, take, FINGER_CLOSED, "take strut B", g)]
    t = sum(w.duration for w in out)
    if start > t:
        out.append(_w(start - t, take, FINGER_CLOSED, "wait for strut A to be held", g))
    out += [_w(1.0, take + [0.0, B_GAP - 0.003, 0.0], FINGER_CLOSED, "line its end up with the peg", g),
            _w(2.0, home_on + [0.0, OVERSHOOT, 0.0], FINGER_CLOSED, "push it onto the peg: the catch snaps up", g),
            _w(0.6, home_on + [0.0, OVERSHOOT, 0.0], FINGER_CLOSED, "hold", g)]
    pushed_at = sum(w.duration for w in out)
    up = home_on + [0.0, 0.0, LIFT]
    out += [_w(1.6, home_on, FINGER_CLOSED, "wait for strut A to be let go", g),
            _w(1.5, up, FINGER_CLOSED, "lift strut B: the corner rises in one piece", g),
            _w(1.2, up, FINGER_CLOSED, "hold it up", g),
            _w(1.0, up + [SET_ASIDE, 0.0, 0.0], FINGER_CLOSED, "move it clear of the jig", g),
            _w(1.5, home_on + [SET_ASIDE, 0.0, 0.0], FINGER_CLOSED, "set it back down", g),
            _w(0.5, home_on + [SET_ASIDE, 0.0, 0.0], FINGER_OPEN, "let go", g),
            _w(0.8, home_on + [SET_ASIDE, 0.0, 0.06], FINGER_OPEN, "rise", g)]
    out += _go_home("right", home_on + [SET_ASIDE, 0.0, 0.06], g, FINGER_OPEN)
    return out, pushed_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, pushed_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(pushed_at + 0.2, total)
    return {"left": left, "right": right}
