"""On the station table stands a clockwork tin locomotive, run down, with its winding key on top near the front. A
finish line is painted on the table ahead of it.

Task: wind the toy and let it run past the finish line. The key is stiff (the mainspring and its ratchet resist it), so
turned on its own it just turns the light toy on the table: one hand holds the toy while the other turns the key, half a
turn at a time, at least a turn and a half. Then both hands let go, and the spring drives the toy forward.

Run it:  inspect-robots run --embodiment yam_arena -E task=wind_a_clockwork_toy_and_let_it_run.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the toy: a tin locomotive, heading +x; its frame: origin at the middle of its underside --------------------------
TOY = (0.18, 0.045, 0.032)  # long, wide, tall: the tin body, which rides on two axles
BODY_Z0 = 0.008  # its underside above the table
TOY_TOP = BODY_Z0 + TOY[2]
TOY_MASS = 0.10
WHEEL_R, WHEEL_W, WHEEL_Y = 0.009, 0.006, 0.016  # the wheels, under the body
AXLES = {"front_axle": 0.05, "rear_axle": -0.045}  # x from the toy's middle
WHEEL_FRICTION = 0.2  # tin wheels: they skid sideways easily
BRAKE = 0.01  # N m s per degree on the axles unless it is running: the clockwork holds the wheels
TX, TY = 0.225, 0.0
KEY_X = 0.07  # the key's shaft, on top, this far ahead of the middle
SHAFT_R, SHAFT_H = 0.003, 0.012
WING = (0.005, 0.026, 0.018)  # the key's wing: thick, wide, tall
WING_YAW = math.radians(62.0)  # the wing's face turned this far from the toy's x axis (where the demo grips it)
RED, YELLOW, BLACK, BRASS = (0.78, 0.12, 0.10), (0.92, 0.75, 0.12), (0.08, 0.08, 0.09), (0.78, 0.62, 0.25)

# -- the spring and its ratchet ---------------------------------------------------------------------------------------
# The key's hinge has a stiff spring (STIFF) whose target the task file keeps a little (the slack) behind the most the
# key has been wound: so winding it on takes STIFF * slack (0.6 N m, and 0.05 N m more for each turn wound), and let go
# it springs back only by the slack, where the ratchet holds it.
STIFF = 0.1  # N m per degree
SLACK0, SLACK_PER_TURN = 6.0, 0.5  # degrees
FULL = 3 * 360.0  # fully wound at three turns: the key stops there
TRAVEL_PER_TURN = 0.12  # m the toy runs on each turn of winding
SPEED = 0.10  # m/s it runs at: the task file sets that speed along its heading every step and pushes it with a force
PUSH0, PUSH_MAX = 0.6, 3.0  # that cancels the table's friction (N; learned as it goes, from how much it slowed)
STEP_S = 0.05  # the control step
FREE = 0.03  # it runs once both TCPs are this far from it (and from the space 3 cm above it, round the key)

# -- scoring ----------------------------------------------------------------------------------------------------------
FINISH_X = TX + 0.15  # the finish line: the toy's middle past it
SUCCESS_HOLD_S = 1.0


def _toy() -> Body:
    lx, wy, h = TOY
    shapes = [BoxShape((0.0, 0.0, BODY_Z0 + h / 2), (lx, wy, h), RED),
              BoxShape((-lx / 2 + 0.03, 0.0, TOY_TOP + 0.0005), (0.05, wy - 0.006, 0.001), YELLOW, collide=False)]  # cab
    return Body.build("toy", pos=(TX, TY, TOP + 0.0005), mass=TOY_MASS, shapes=shapes, friction=(1.0, 0.9), color=RED,
                      com=(0.0, 0.0, BODY_Z0 + h / 2))


def _axle(name: str, x: float) -> Body:
    """Origin on the axle's line; turns on a hinge in the body."""
    shapes = [CylShape((0.0, y, 0.0), WHEEL_R, WHEEL_W, BLACK, axis="Y") for y in (-WHEEL_Y, WHEEL_Y)]
    shapes.append(CylShape((0.0, 0.0, 0.0), 0.002, 2 * WHEEL_Y, (0.5, 0.5, 0.5), collide=False, axis="Y"))
    return Body.build(name, pos=(TX + x, TY, TOP + 0.0005 + WHEEL_R), mass=0.008, shapes=shapes, color=BLACK,
                      friction=(WHEEL_FRICTION, WHEEL_FRICTION), slick=True)


def _key() -> Body:
    """Origin at the foot of the shaft, on the toy's top."""
    shapes = [CylShape((0.0, 0.0, SHAFT_H / 2), SHAFT_R, SHAFT_H, BRASS),
              BoxShape((0.0, 0.0, SHAFT_H + WING[2] / 2), WING, BRASS, yaw=WING_YAW)]
    return Body.build("key", pos=(TX + KEY_X, TY, TOP + 0.0005 + TOY_TOP), mass=0.006, shapes=shapes,
                      friction=(1.0, 0.9), color=BRASS)


def _finish() -> Body:
    return Body.build("finish_line", pos=(FINISH_X, TY, TOP), mass=1.0, kinematic=True, color=(0.95, 0.95, 0.95),
                      shapes=[BoxShape((0.0, 0.0, 0.0004), (0.012, 0.30, 0.0008), (0.95, 0.95, 0.95), collide=False)])


TOY_B, KEY, FINISH = _toy(), _key(), _finish()
AXLE_BODIES = [_axle(name, x) for name, x in AXLES.items()]
OBJECTS = [TOY_B, *AXLE_BODIES, KEY, FINISH]
for _a in AXLE_BODIES:
    hinge(f"{_a.name}_bearing", "toy", _a.name, anchor=_a.pos, axis="Y", damping=BRAKE)
hinge("key_shaft", "toy", "key", anchor=KEY.pos, axis="Z", spring=STIFF, damping=0.0005, target_deg=-SLACK0)


# -- followed after every control step: the spring, the ratchet and the run -------------------------------------------

def _fresh() -> dict:
    return {"turn": 0.0, "raw": 0.0, "wound": 0.0, "most": 0.0, "running": False, "crossed": False, "x_last": None,
            "tcps": None, "push": PUSH0, "run_from": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TOY_DEBUG")), "steps": 0}


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


def _gap_to_toy(p, t_pos, t_rot) -> float:
    v = t_rot.T @ (np.asarray(p, float) - t_pos)
    half = np.array([TOY[0] / 2, TOY[1] / 2, TOY_TOP / 2 + 0.015])  # the toy and the key above it
    centre = np.array([0.0, 0.0, TOY_TOP / 2 + 0.015])
    return float(np.linalg.norm(np.maximum(np.abs(v - centre) - half, 0.0)))


def _set_force(asset, f_world, rot) -> None:
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), positions=None,
                                                           is_global=False)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    toy = base.scene["toy"]
    t = as_numpy(toy.data.root_link_pose_w)[0]
    k = as_numpy(base.scene["key"].data.root_link_pose_w)[0]
    t_pos, t_rot, k_rot = t[:3] - origin, rotation(t[3:7]), rotation(k[3:7])
    rel = t_rot.T @ k_rot
    raw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    _EP["turn"] += (raw - _EP["raw"] + 180.0) % 360.0 - 180.0  # the key's turn against the toy, counted on (degrees)
    _EP["raw"] = raw
    _EP["tcps"] = _tcps(base, origin)
    free = all(_gap_to_toy(p, t_pos, t_rot) >= FREE for p in _EP["tcps"])
    upright = t_rot[2, 2] > math.cos(math.radians(15.0)) and t_pos[2] < TOP + 0.01
    heading = t_rot[:, 0] * np.array([1.0, 1.0, 0.0])
    heading /= max(float(np.linalg.norm(heading)), 1e-6)
    was_running = _EP["running"]
    _EP["running"] = free and upright and _EP["wound"] > 1.0
    if _EP["running"] != was_running:  # the wheels turn only while it runs
        for name in AXLES:
            set_hinge_drive(f"{name}_bearing", damping=0.0 if _EP["running"] else BRAKE)
        _EP["run_from"] = float(t_pos[0]) if _EP["running"] else None  # where this run started
    if _EP["running"]:
        import torch

        v = as_numpy(toy.data.root_lin_vel_w)[0]
        w = as_numpy(toy.data.root_ang_vel_w)[0]
        if was_running:  # it was set going at SPEED last step: push harder by what friction took off since
            short = SPEED - float(np.dot(v, heading))
            _EP["push"] = float(np.clip(_EP["push"] + 0.5 * TOY_MASS * short / STEP_S, 0.0, PUSH_MAX))
        _set_force(toy, _EP["push"] * heading, t_rot)
        side = v - float(np.dot(v, heading)) * heading
        vel = np.concatenate([SPEED * heading + np.array([0.0, 0.0, side[2]]), w])
        toy.write_root_velocity_to_sim(torch.tensor(vel, dtype=torch.float32, device=toy.device).view(1, 6))
        if _EP["x_last"] is not None:
            ran = max(0.0, float(np.dot(t_pos - _EP["x_last"], heading)))
            _EP["wound"] = max(0.0, _EP["wound"] - ran / TRAVEL_PER_TURN * 360.0)
        if t_pos[0] >= FINISH_X and _EP["run_from"] is not None and _EP["run_from"] < FINISH_X:
            _EP["crossed"] = True  # it ran over the line by itself
    else:
        _set_force(toy, np.zeros(3), t_rot)
        if _EP["turn"] > _EP["wound"]:  # wound on by hand; the ratchet holds what it has
            _EP["wound"] = min(_EP["turn"], FULL)
    slack = SLACK0 + SLACK_PER_TURN * _EP["wound"] / 360.0  # (running, the key turns back as the spring runs down)
    set_hinge_drive("key_shaft", target_deg=_EP["wound"] - slack if _EP["wound"] < FULL else FULL)
    _EP["x_last"] = t_pos.copy()
    _EP["most"] = max(_EP["most"], _EP["wound"])
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            key_at = t_pos + t_rot @ np.array([KEY_X, 0.0, TOY_TOP + SHAFT_H + WING[2] / 2])
            print(f"[toy] t={_DEBUG['steps'] / 20:5.1f}s toy={np.round(t_pos * 1000).astype(int).tolist()} "
                  f"yaw={math.degrees(math.atan2(t_rot[1, 0], t_rot[0, 0])):6.1f} key_turn={_EP['turn']:7.1f} "
                  f"wound={_EP['wound']:6.1f} most={_EP['most']:6.1f} running={_EP['running']} crossed={_EP['crossed']} "
                  f"leftTCP-toy={np.round((tl - t_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-wing={np.round((tr - key_at) * 1000).astype(int).tolist()} score={_points()}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points() -> float:
    if _EP["crossed"]:
        return 1.0
    if _EP["most"] >= 360.0:
        return 0.5
    return 0.25 if _EP["most"] >= 90.0 else 0.0


def score(objects) -> float:
    """1.0 once the toy has run with its middle over the finish line under its own power (let go: both TCPs 3 cm from
    it, and the run started behind the line); 0.5 if it was wound at least a turn; 0.25 if at least a quarter turn;
    else 0.0."""
    return _points()


def success(objects) -> bool:
    """Full score with the toy run down and still."""
    return score(objects) >= 1.0 and not _EP["running"] and objects["toy"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the toy's back end, fingers closed, and presses it onto the table. The
# right hand comes straight down on the key's wing and turns it half a turn, lets go, turns back in the air and takes
# it again: four times, two turns. Then both hands let go and rise, and the toy runs.

YAW_FROM, YAW_TO = math.radians(-28.0), math.radians(152.0)  # the right hand's half turn, about the key
HALF_TURNS = 4
WING_IN_HAND = 0.005  # the right TCP this far above the wing's top: the fingertips 8 mm down its faces
LEAD_S, LEAD_STEPS = 3.0, 15


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


GRIP_L = grip_rotation(0.0)  # straight down, fingers closed: it presses the toy's back end down onto the table
PRESS = 0.012  # aim the fingertips this far into the toy's top (the arm gives: about 10 N)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4),
                                  (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


HOLD = np.array([TX - 0.075, TY, TOP + 0.0005 + TOY_TOP + 0.013 - PRESS])  # fingertips on its back end
HELD_AT = LEAD_S + 1.0 + 0.8


def _left(release_at: float, total: float) -> list[Waypoint]:
    out = _lead_in("left", HOLD + [0.0, 0.0, 0.04 + PRESS], GRIP_L, "over the toy's back end", FINGER_CLOSED)
    out += [_w(1.0, HOLD + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP_L),
            _w(0.8, HOLD, FINGER_CLOSED, "press it down onto the table: it holds the toy still", GRIP_L)]
    t = sum(w.duration for w in out)
    out += [_w(max(release_at - t, 0.3), HOLD, FINGER_CLOSED, "hold it while the key is wound", GRIP_L),
            _w(1.0, HOLD + [-0.06, 0.10, 0.08], FINGER_CLOSED, "rise and move aside: it runs", GRIP_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), HOLD + [-0.06, 0.10, 0.08], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", GRIP_L))
    return out


def _right() -> list[Waypoint]:
    wing = np.array([TX + KEY_X, TY, TOP + 0.0005 + TOY_TOP + SHAFT_H + WING[2] + WING_IN_HAND])
    up = wing + [0.0, 0.0, 0.025]
    g0, g1 = grip_rotation(YAW_FROM), grip_rotation(YAW_TO)
    out = _lead_in("right", up, g0, "over the key", FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(HELD_AT - t, 0.2) + 0.3, up, FINGER_OPEN, "wait for the toy to be held", g0))
    for n in range(HALF_TURNS):
        out += [_w(0.6, wing, FINGER_OPEN, "down round the key's wing", g0),
                _w(0.6, wing, FINGER_CLOSED, "take it", g0)]
        for k in range(1, 5):
            out.append(_w(0.5, wing, FINGER_CLOSED, f"turn it half a turn ({n + 1} of {HALF_TURNS})",
                          grip_rotation(YAW_FROM + (YAW_TO - YAW_FROM) * k / 4)))
        out += [_w(0.4, wing, FINGER_OPEN, "let go of it", g1),
                _w(0.5, up, FINGER_OPEN, "up", g1)]
        for k in range(1, 4):
            out.append(_w(0.4, up, FINGER_OPEN, "turn back in the air", grip_rotation(YAW_TO + (YAW_FROM - YAW_TO) * k / 3)))
    out.append(_w(1.2, up + [-0.05, -0.12, 0.04], FINGER_OPEN, "rise and move aside: it runs", g0))
    return out


def demo():
    right = _right()
    t_r = sum(w.duration for w in right)
    total = t_r + 5.0
    right.append(_w(5.0, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": _left(t_r - 1.0, total), "right": right}
