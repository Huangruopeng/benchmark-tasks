"""The scripted demo of clamp_a_small_part_before_drilling_it.py, as each arm's TCP waypoints.

The right hand forks the feed handle's front spoke (its fingers part-closed either side of it) and, with the motor off,
brings the bit's point down to just above the plate. The left hand comes down on the plate's front edge, slides the
plate 20 mm back under the point and nudges it until the punch mark is under the point; the right then lowers the
point, which drops into the punch mark, centres the plate on the bit's axis and holds it; the right presses it in at
15 N. Two parts of the plan follow the simulation as it runs, the way a person watches and feels the work (a scripted
demo may read the simulator's state): steer() turns the right hand's hold on the handle to keep the point hovering,
then to keep its push on the plate at its target, and moves the left hand's nudge toward the mark until the point is
in the dimple. Everything else is fixed in advance. The left lifts off and turns the clamp's star knob clockwise in
strokes (grip, turn, let go, rise, turn back in the air, come down, grip again) until the rubber tip presses past
85 N, while the point in the punch mark holds the plate against the tip's drag. The right lets the handle back up,
still forking it; the left presses the green start button and goes home. The right feeds the bit through the plate
into the spoil board, lets the handle back up, lets go, presses the red stop button and goes back the way it came,
then home.
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

from drill_press import (BTN_START_OUT, BTN_START_Y, BTN_STOP_OUT, BTN_STOP_Y, BTN_Z, CLEAR, GAP0, HUB, KNOB_H,
                         KNOB_HUB_H, KNOB_Z0, PINION_R, PITCH, PLATE_C, PLATE_START, PLATE_TOP, PLATE_X, SCREW0, SPOKE_L,
                         SPOKES0, SW_X, spoke_dir)

L_TILT = math.radians(45.0)  # the left hand on the clamp's knob leans out to the left and toward the arms
P_TILT = math.radians(55.0)  # the left hand pressing the plate leans toward the arms
P_WRIST = 180.0  # degrees (seen from above): where the left wrist sits from its fingers while it slides the plate
P_TOUCH_XY = (PLATE_X[0] - PLATE_START + 0.015, 0.0)  # where the left comes down on the plate: its front
# edge region, near the centre line through the punch mark and the bit
P_DEPTH = 0.025  # it aims this far below where its fingers meet the plate: the arm gives, pressing the plate down
P_LEAN = 0.008  # and this far back toward the arms: a leaning hand slides forward as it presses
P_SLIDE = (PLATE_START + 0.001, 0.004)  # the slide: 20 mm back to the bit, aimed 1 mm on and 4 mm left against the
# plate's lag and drift under the pressing hand (seen in the simulation)
P_SLIDE_S = 3.0
NUDGE_S, NUDGE_N = 6.0, 24  # the left then nudges the plate for 6 s, in 24 steps, until the point drops in
NUDGE_K, NUDGE_STEP, NUDGE_MAX, NUDGE_OK = 2.5, 0.0008, 0.025, 0.0003  # 1/s on the mark's offset; at most 0.8 mm a
# control step and 25 mm in all (the pressing hand gives sideways and slips a little on the plate); no nudge while the
# mark is within 0.3 mm of the axis
LOWER_AT = 0.0006  # the right lowers the point once the mark is this close to the axis (or the nudge is nearly over)
SEAT_N = (6.0, 15.0)  # N: the point's push on the plate while the plate is nudged, and once it is in the dimple
SEAT_GAIN, SEAT_STEP, SEAT_DEG = 0.5, 0.15, (-6.0, 30.0)  # deg/(N s) on the handle; at most 0.15 deg a step (the point
# comes down at under 1 mm/s, so it does not bounce off the plate); its range
HOVER_DEG = 11.0  # the hold first turns this far past the touch angle: the arm gives, so the point hovers about 1 mm up
HOLD_STEP = 0.25  # s: the right's hold, in steps this long (each one steered)
KNOB_IN_HAND = KNOB_Z0 + (KNOB_HUB_H + KNOB_H) / 2  # the left TCP this far above the screw's origin: level with the
# middle of the knob's lobes
STROKE = math.radians(90.0)
STROKES = 6
LAST_STROKE = math.radians(45.0)  # the last stroke is shorter: a turn brings the tip down, the rest presses it
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 1.0, 0.5
LIFT, LIFT_S = 0.025, 0.4  # between strokes the open hand rises off the knob and turns back in the air
CAP_GAMMA = math.radians(25.0)  # the right hand on the knob: pointing left, leaning this far toward the spoke's normal
FORK, FORK_R = -0.010, 0.092  # the right hand works the handle as a fork: its fingers part-closed to a 14 mm gap,
# straddling the 10 mm spoke 92 mm from the hub, just inside the ball (the fingers are 68 mm wide along the spoke), so
# it pushes the spoke either way without squeezing it (the jaw's V pushes a squeezed ball out to the fingertips)
TOUCH_DEG = math.degrees(CLEAR / PINION_R)  # the handle's turn that brings the point down onto the plate (19.1)
REST_BETA = SPOKES0[0] + TOUCH_DEG  # the front spoke where the point meets the plate; steer() turns the hand on from
# there as far as the point's push on the plate needs (the arm gives)
FEED_BETA = (SPOKES0[0], SPOKES0[0] + 19.0, 15.0)  # the front spoke: at rest; the tip at the plate; the end of the pull
FEED_S = (1.5, 13.0)  # s for the approach and for the cut
PRESS_X = 0.019  # the closed fingertips aimed this far past a button's face (it gives under the spring)
SW_APPROACH = 0.035  # the TCP this far out from a button before and after pressing it


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _frame(approach, closing) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float) - np.dot(closing, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _dir(deg: float) -> np.ndarray:
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])


# the left on the knob, mid-stroke: fingers closing along 225 degrees (seen from above), leaned about the closing axis
# so the wrist sits at 135 degrees, out to the left and toward the arms, clear of the head. A stroke turns the hand from
# +45 degrees to -45 degrees about the vertical.
KNOB_GRIP = _frame(-math.sin(L_TILT) * _dir(135.0) + [0.0, 0.0, -math.cos(L_TILT)], _dir(225.0))
# the left pressing the plate: fingers closed, leaned about the closing axis so the wrist sits toward the arms
PRESS_L = _frame(-math.sin(P_TILT) * _dir(P_WRIST) + [0.0, 0.0, -math.cos(P_TILT)], _dir(P_WRIST + 90.0))
# a hand on a button: pointing forward, the fingers closing vertically (thin across, so it presses one button)
BUTTON_GRIP = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def _cap_grip(beta_deg: float) -> np.ndarray:
    """The right hand on the front knob with the spoke at ``beta_deg``: pointing left and leaned CAP_GAMMA toward the
    spoke's normal, the fingers closing across the spoke."""
    d = -spoke_dir(beta_deg)  # knob to hub
    b = math.radians(beta_deg)
    n = np.array([math.cos(b), 0.0, math.sin(b)])
    a = math.cos(CAP_GAMMA) * np.array([0.0, 1.0, 0.0]) + math.sin(CAP_GAMMA) * n
    return _frame(a, np.cross(a, d))


def _knob_pos(beta_deg: float) -> np.ndarray:
    """The right TCP forking the front spoke when the spoke is at ``beta_deg``."""
    return HUB + FORK_R * spoke_dir(beta_deg)


_IK_CACHE: dict = {}


def _good_ik(arm: str, pos, rot, seed=None) -> np.ndarray:
    """Joints for the TCP pose: from ``seed`` if that lands well inside the joint limits, else the solution with the
    most room over a grid of starts."""
    import itertools

    from yam_arena import ik, station

    key = (arm, tuple(np.round(pos, 5)), tuple(np.round(np.asarray(rot), 5).ravel()),
           None if seed is None else tuple(np.round(seed, 4)))
    if key in _IK_CACHE:
        return _IK_CACHE[key]

    def attempt(s):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(s, float),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            return q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        return None

    best = attempt(seed) if seed is not None else None
    if best is None or best[1] < 0.02:
        for s in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3, 2.0), (0.4, 1.0, 1.8), (-0.8, 0.0, 0.8),
                                   (-1.4, 0.0, 1.4), (0.0,)):
            got = attempt(s)
            if got is not None and (best is None or got[1] > best[1]):
                best = got
                if best[1] > 0.15:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {np.round(pos, 3)}")
    _IK_CACHE[key] = best[0]
    return best[0]


def _joint_move(arm: str, q0, pos, rot, label: str, finger, duration: float, steps: int) -> tuple[list, np.ndarray]:
    """Waypoints moving the arm from joints ``q0`` to the pose in a straight line in joint space."""
    from yam_arena import ik, station

    q0 = np.asarray(q0, float)
    q1 = _good_ik(arm, pos, rot, q0)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(duration / steps, tuple(float(v) for v in p), finger, label, r))
    return out, q1


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _t(plan) -> float:
    return sum(w.duration for w in plan)


def _knob_at(turned: float) -> np.ndarray:
    """The left TCP on the clamp's knob after the screw has turned ``turned`` (rad); it stops on the plate."""
    return SCREW0 + [0.0, 0.0, KNOB_IN_HAND - min(PITCH * turned / (2 * math.pi), GAP0)]


def _home(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _button(y: float, out_len: float):
    return (np.array([SW_X[0] - out_len - SW_APPROACH, y, BTN_Z]), np.array([SW_X[0] - out_len + PRESS_X - 0.0128, y,
                                                                              BTN_Z]))


def _left(times: dict) -> list[Waypoint]:
    """Slide the plate under the resting point; tighten the clamp; press the start button; go home."""
    from yam_arena.constants import READY

    touch = np.array([*P_TOUCH_XY, PLATE_TOP + 0.013])
    pressed = touch + [-P_LEAN, 0.0, -P_DEPTH]
    slid = pressed + [*P_SLIDE, 0.0]
    above = touch + [0.0, 0.0, 0.04]
    high = above + [0.0, -0.02, 0.08]  # high over the plate first, clear of the clamp's knob
    out, q = _joint_move("left", READY, high, PRESS_L, "up, clear of the clamp", FINGER_CLOSED, 2.0, 10)
    out.append(_w(1.5, above, FINGER_CLOSED, "over the plate's front edge", PRESS_L))
    out.append(_w(max(times.get("rest", 0.0) - 1.6 - _t(out), 0.2), above, FINGER_CLOSED,
                  "wait: the right hand brings the bit's point down onto the plate", PRESS_L))
    out += [_w(1.0, touch, FINGER_CLOSED, "down onto the plate", PRESS_L),
            _w(0.6, pressed, FINGER_CLOSED, "press it onto the board", PRESS_L),
            _w(P_SLIDE_S, slid, FINGER_CLOSED, "slide it 20 mm back under the bit's point", PRESS_L)]
    times["slid"] = _t(out)
    nudge = [_w(NUDGE_S / NUDGE_N, slid, FINGER_CLOSED, "nudge it until the point drops into the punch mark", PRESS_L)
             for _ in range(NUDGE_N)]
    lift = _w(0.6, slid + [0.0, 0.0, P_DEPTH + 0.04], FINGER_CLOSED,
              "lift off: the point in the punch mark holds the plate", PRESS_L)
    out += nudge + [lift]
    LIVE["left"] = {"wps": [(w, np.array(w.pos)) for w in nudge + [lift]], "t0": times["slid"] + 0.5,
                    "t1": times["slid"] + NUDGE_S, "off": np.zeros(2), "done": False}
    times["placed"] = _t(out)
    q = _good_ik("left", out[-1].pos, PRESS_L, q)
    g_start = _rz(STROKE / 2) @ KNOB_GRIP
    via = _knob_at(0.0) + [-0.06, 0.08, 0.08]  # out in front and to the left of the head
    moves, q = _joint_move("left", q, via, g_start, "toward the clamp, clear of the head", FINGER_OPEN, 2.5, 12)
    out += moves
    out += [_w(1.2, _knob_at(0.0) + [0.0, 0.0, 0.04], FINGER_OPEN, "over the clamp's knob", g_start),
            _w(1.0, _knob_at(0.0), FINGER_OPEN, "down round the knob", g_start),
            _w(0.6, _knob_at(0.0), FINGER_CLOSED, "grip it", g_start)]
    turned = 0.0
    g_end = g_start
    for k in range(STROKES):
        stroke = LAST_STROKE if k == STROKES - 1 else STROKE
        for j in range(1, 5):  # a quarter of the stroke at a time
            out.append(_w(TURN_S / 4 * stroke / STROKE, _knob_at(turned + stroke * j / 4), FINGER_CLOSED,
                          f"turn it clockwise ({k + 1})", _rz(STROKE / 2 - stroke * j / 4) @ KNOB_GRIP))
        turned += stroke
        at = _knob_at(turned)
        g_end = _rz(STROKE / 2 - stroke) @ KNOB_GRIP
        out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", g_end))
        if k < STROKES - 1:
            up = at + [0.0, 0.0, LIFT]
            out.append(_w(LIFT_S, up, FINGER_OPEN, "rise off the knob", g_end))
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, up, FINGER_OPEN, "turn back in the air",
                              _rz(-STROKE / 2 + STROKE * j / 4) @ KNOB_GRIP))
            out.append(_w(LIFT_S, at, FINGER_OPEN, "down round the knob", g_start))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the knob again", g_start))
    times["clamped"] = _t(out)
    out.append(_w(1.0, _knob_at(turned) + [-0.03, 0.0, 0.03], FINGER_OPEN, "rise, toward the arms", g_end))
    q = _good_ik("left", out[-1].pos, g_end, _good_ik("left", _knob_at(turned), g_end, q))
    away, pushed = _button(BTN_START_Y, BTN_START_OUT)
    moves, q = _joint_move("left", q, away, BUTTON_GRIP, "to the switch", FINGER_CLOSED, 2.5, 12)
    out += moves
    out.append(_w(max(times.get("handle_up", 0.0) - _t(out), 0.2), away, FINGER_CLOSED,
                  "wait: the right hand has let the bit up", BUTTON_GRIP))
    out += [_w(1.0, pushed, FINGER_CLOSED, "press the green start button: the chuck turns", BUTTON_GRIP),
            _w(0.4, pushed, FINGER_CLOSED, "pressed", BUTTON_GRIP),
            _w(0.6, away, FINGER_CLOSED, "back off", BUTTON_GRIP)]
    times["started"] = _t(out)
    home = _home("left")
    moves, q = _joint_move("left", _good_ik("left", away, BUTTON_GRIP, q), home[0], home[1], "home, well away",
                           FINGER_OPEN, 2.5, 12)
    out += moves
    times["left_home"] = _t(out)
    return out


def _right(times: dict) -> list[Waypoint]:
    """Bring the point down onto the plate and hold it in the punch mark while the plate is clamped; feed the bit through
    once the motor runs; let the handle up; stop the motor; go home."""
    from yam_arena.constants import READY

    g0 = _cap_grip(FEED_BETA[0])
    knob0 = _knob_pos(FEED_BETA[0])
    beside = knob0 - 0.03 * g0[:, 2]
    out, q = _joint_move("right", READY, beside, g0, "beside the feed handle's front spoke", FINGER_OPEN, 3.0, 12)
    q_beside = q
    out += [_w(1.0, knob0, FINGER_OPEN, "its fingers either side of the spoke", g0),
            _w(0.6, knob0, FORK, "close to a fork round the spoke", g0)]

    def turn(b0, b1, dur, label):
        n = max(2, int(abs(b1 - b0) / 5.0))
        return [_w(dur / n, _knob_pos(b0 + (b1 - b0) * k / n), FORK, label, _cap_grip(b0 + (b1 - b0) * k / n))
                for k in range(1, n + 1)]

    near = REST_BETA  # the point just above the plate (the arm gives); steer() lowers it from there
    hold_beta = near
    out += turn(FEED_BETA[0], near, 1.5, "motor off: bring the bit's point down to just above the plate")
    times["rest"] = _t(out)
    out.append(_w(max(times.get("slid", 0.0) - 2.5 - _t(out), 0.3), _knob_pos(near), FORK,
                  "wait: the left slides the plate under it", _cap_grip(near)))
    t0 = _t(out)
    n = max(8, round((times.get("clamped", 0.0) + 0.5 - t0) / HOLD_STEP))
    hold = [_w(HOLD_STEP, _knob_pos(hold_beta), FORK,
               "point down on the plate; in the punch mark it holds the plate on the bit's axis", _cap_grip(hold_beta))
            for _ in range(n)]
    out += hold
    LIVE["right"] = {"wps": [(w, hold_beta) for w in hold], "t0": t0, "t1": t0 + n * HOLD_STEP, "d": 0.0,
                     "seated": False, "go": False}
    out += turn(hold_beta, FEED_BETA[0], 2.0, "let the handle back up, keeping hold of it")
    times["handle_up"] = _t(out)
    out.append(_w(max(times.get("started", 0.0) + 0.5 - _t(out), 0.3), knob0, FORK,
                  "wait: the chuck turns", g0))
    out += turn(FEED_BETA[0], FEED_BETA[1], FEED_S[0], "pull the handle: the bit down to the plate")
    out += turn(FEED_BETA[1], FEED_BETA[2], FEED_S[1], "feed it through the plate")
    out.append(_w(0.8, _knob_pos(FEED_BETA[2]), FORK, "through, into the spoil board", _cap_grip(FEED_BETA[2])))
    out += turn(FEED_BETA[2], FEED_BETA[0], 3.0, "let the handle back up")
    out += [_w(0.5, knob0, FINGER_OPEN, "let go: the quill is up", g0),
            _w(0.8, beside, FINGER_OPEN, "back off", g0)]
    away, pushed = _button(BTN_STOP_Y, BTN_STOP_OUT)
    out += [_w(2.5, away, FINGER_CLOSED, "to the switch", BUTTON_GRIP),
            _w(1.0, pushed, FINGER_CLOSED, "press the red stop button: the chuck runs down", BUTTON_GRIP),
            _w(0.4, pushed, FINGER_CLOSED, "pressed", BUTTON_GRIP),
            _w(0.6, away, FINGER_CLOSED, "back off", BUTTON_GRIP)]
    out.append(_w(2.5, beside, FINGER_OPEN, "back the way it came, clear of the feed handle", g0))
    home = _home("right")
    moves, q = _joint_move("right", q_beside, home[0], home[1], "home", FINGER_OPEN, 3.0, 12)
    return out + moves


LIVE: dict = {}  # the steered parts of the plan (waypoints and where they were planned), filled by demo()


def steer(t: float, err, seat_n: float, in_dimple: bool, dt: float = 0.05) -> None:
    """Run after every control step (from the task file) at time ``t``: ``err`` is the punch mark's offset from the
    bit's axis (m, world x-y), ``seat_n`` the point's push on the plate (N), ``in_dimple`` whether the point is in
    the punch mark. Holds the point just above the plate, and moves the left's nudge toward the mark, until the mark is
    within LOWER_AT of the axis; then lowers the point and moves the right's hold to keep its push at SEAT_N, and
    stops nudging once the point is in the dimple."""
    r, lf = LIVE.get("right"), LIVE.get("left")
    if r is not None:
        if t < r["t0"]:
            r["d"], r["seated"], r["go"] = 0.0, False, False
        elif t <= r["t1"]:
            r["seated"] = r["seated"] or in_dimple
            r["go"] = r["go"] or float(np.linalg.norm(err)) < LOWER_AT or (lf is not None and t > lf["t1"] - 1.5)
            if r["go"]:  # press the point onto the plate: lightly until it is in the dimple, then at 15 N
                step = SEAT_GAIN * (SEAT_N[1 if r["seated"] else 0] - seat_n) * dt
            else:  # hover just above the plate while the left brings the mark under the point
                step = HOVER_DEG - r["d"]
            r["d"] = float(np.clip(r["d"] + np.clip(step, -SEAT_STEP, SEAT_STEP), *SEAT_DEG))
            for w, b in r["wps"]:
                w.pos, w.yaw = tuple(float(v) for v in _knob_pos(b + r["d"])), _cap_grip(b + r["d"])
    if lf is not None:
        if t < lf["t0"]:
            lf["off"], lf["done"] = np.zeros(2), False
        elif t <= lf["t1"] and not lf["done"]:
            if in_dimple:
                lf["done"] = True
            elif float(np.linalg.norm(err)) > NUDGE_OK:
                step = NUDGE_K * np.asarray(err, float) * dt
                step *= min(1.0, NUDGE_STEP / max(float(np.linalg.norm(step)), 1e-9))
                off = lf["off"] + step
                lf["off"] = off * min(1.0, NUDGE_MAX / max(float(np.linalg.norm(off)), 1e-9))
            for w, p in lf["wps"]:
                w.pos = (float(p[0] + lf["off"][0]), float(p[1] + lf["off"][1]), float(p[2]))


def demo():
    times: dict = {}
    for _ in range(3):  # each arm's timing depends on the other's
        right = _right(times)
        left = _left(times)
    total = max(_t(left), _t(right)) + 5.0
    for plan in (left, right):
        plan.append(_w(total - _t(plan), plan[-1].pos, plan[-1].finger,
                       "hold still: the trial ends once the full score has held 1 s", plan[-1].yaw))
    return {"left": left, "right": right}
