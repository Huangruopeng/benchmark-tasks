"""On the station table stands a flat glass hob with a light frying pan on it, its long handle out to the left. A pancake,
golden side up, lies in the pan, a little to the right of its middle. A steel turner (a spatula with a cranked neck)
lies to the right, its blade on the worktop and its wooden handle propped on a small rest.

Task: flip the pancake. One hand holds the pan still by its handle while the other slides the turner's blade under the
pancake's right edge and lifts that edge up and over, so the pancake turns over onto its other side in the pan (pushed
with nothing holding it, the light pan just slides across the smooth glass ahead of the blade). Then the turner goes
back on its rest, the pan is let go, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=flip_a_pancake_holding_the_pan_by_its_handle.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the hob and the pan ------------------------------------------------------------------------------------------------
HOB = (0.34, 0.40, 0.012)  # black glass hob: along x, across, tall
HOB_AT = np.array([0.31, 0.02])
PAN_AT = np.array([0.31, 0.0])
PAN_R, PAN_WALL_H, PAN_FLOOR = 0.140, 0.025, 0.004
PAN_HANDLE = (0.024, 0.170, 0.020)  # across, out (+y), tall
PAN_MASS = 0.60
PAN_Z = TOP + HOB[2]  # the pan's underside
FLOOR = PAN_Z + PAN_FLOOR  # the pan's cooking surface

# -- the pancake: 9 cm across, 1.3 cm thick; it rests on a smaller hidden base, so a thin blade gets under its edge ----
CAKE_AT = np.array([0.31, -0.020])
CAKE_R, CAKE_H = 0.040, 0.009
BASE_R, BASE_H = 0.024, 0.004

# -- the turner: a wooden handle along +x as it lies, a steel neck dropping 4.5 cm, a thin blade ----------------------
HANDLE_L, HANDLE_W, HANDLE_H = 0.140, 0.020, 0.016
NECK_DROP, NECK_DEG = 0.045, 70.0
BLADE_L, BLADE_W, BLADE_T = 0.060, 0.070, 0.002
NECK_RUN = NECK_DROP / math.tan(math.radians(NECK_DEG))
BLADE_X0 = HANDLE_L / 2 + NECK_RUN  # the blade's back edge, from the handle's middle
BLADE_Z = -(HANDLE_H / 2 + NECK_DROP)  # the blade's middle plane, below the handle's middle
GRIP_X = 0.010  # where the handle is held, from its middle toward the blade
TURNER_AT = np.array([0.185, -0.235])  # the handle's middle as it lies on its rest
REST = (0.040, 0.050, -BLADE_Z - HANDLE_H / 2 + BLADE_T / 2)  # under the handle's back end: along x, across, tall
REST_X = TURNER_AT[0] - HANDLE_L / 2 + 0.022

BLACK, STEEL, DARK, GOLD, BROWN, WOOD, WHITE = (0.08, 0.08, 0.09), (0.70, 0.71, 0.73), (0.20, 0.20, 0.22), \
    (0.93, 0.74, 0.38), (0.55, 0.33, 0.15), (0.62, 0.43, 0.25), (0.93, 0.93, 0.91)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _hob() -> Body:
    shapes = [BoxShape((0.0, 0.0, HOB[2] / 2), HOB, BLACK)]
    for r in (0.11, 0.075):  # the cooking zone's rings (seen only)
        n = 28
        for i in range(n):
            a = 2 * math.pi * i / n
            shapes.append(BoxShape((r * math.cos(a) + PAN_AT[0] - HOB_AT[0], r * math.sin(a) + PAN_AT[1] - HOB_AT[1],
                                    HOB[2] + 0.0002), (0.004, 2 * math.pi * r / n * 0.7, 0.0004), (0.35, 0.35, 0.38),
                                   yaw=a, collide=False))
    return Body.build("hob", pos=(HOB_AT[0], HOB_AT[1], TOP), mass=3.0, shapes=shapes, kinematic=True, color=BLACK,
                      friction=(0.07, 0.06), slick=True)


def _pan() -> Body:
    shapes = [CylShape((0.0, 0.0, PAN_FLOOR / 2), PAN_R, PAN_FLOOR, DARK)]
    n = 32
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((PAN_R - 0.002) * math.cos(a), (PAN_R - 0.002) * math.sin(a), PAN_WALL_H / 2),
                               (0.004, 2 * math.pi * PAN_R / n * 1.12, PAN_WALL_H), DARK, yaw=a))
    shapes.append(BoxShape((0.0, PAN_R + PAN_HANDLE[1] / 2 - 0.004, PAN_WALL_H - 0.006), PAN_HANDLE, BLACK))
    shapes.append(CylShape((0.0, PAN_R + PAN_HANDLE[1] - 0.012, PAN_WALL_H - 0.006 + PAN_HANDLE[2] / 2 + 0.0002),
                           0.005, 0.0004, (0.5, 0.5, 0.5), collide=False))  # the hanging hole (seen only)
    return Body.build("pan", pos=(PAN_AT[0], PAN_AT[1], PAN_Z), mass=PAN_MASS, shapes=shapes, color=DARK,
                      friction=(0.5, 0.4), com=(0.0, 0.04, 0.008))


def _pancake() -> Body:
    shapes = [CylShape((0.0, 0.0, BASE_H / 2), BASE_R, BASE_H, BROWN),
              CylShape((0.0, 0.0, BASE_H + CAKE_H / 2), CAKE_R, CAKE_H, GOLD),
              CylShape((0.0, 0.0, BASE_H + CAKE_H + 0.0002), CAKE_R * 0.9, 0.0004, (0.96, 0.80, 0.45), collide=False),
              CylShape((0.0, 0.0, BASE_H - 0.0002), CAKE_R * 0.97, 0.0004, BROWN, collide=False)]
    for i in range(7):  # bubbles on the golden side (seen only)
        a = 2 * math.pi * i / 7 + 0.4
        r = 0.022 if i % 2 else 0.031
        shapes.append(CylShape((r * math.cos(a), r * math.sin(a), BASE_H + CAKE_H + 0.0005), 0.0025, 0.0004,
                               (0.85, 0.62, 0.30), collide=False))
    return Body.build("pancake", pos=(CAKE_AT[0], CAKE_AT[1], FLOOR), mass=0.05, shapes=shapes, color=GOLD,
                      friction=(0.6, 0.5), damping=(0.1, 0.3))


def _turner() -> Body:
    """Its frame: origin at the handle's middle, handle along +x toward the blade."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (HANDLE_L, HANDLE_W, HANDLE_H), WOOD)]
    for k in (-1.0, 1.0):  # rivets (seen only)
        shapes.append(CylShape((k * 0.03, 0.0, HANDLE_H / 2 + 0.0002), 0.003, 0.0004, (0.8, 0.8, 0.8), collide=False))
    a = math.radians(NECK_DEG)
    neck_len = NECK_DROP / math.sin(a)
    mid = np.array([HANDLE_L / 2 - 0.004 + NECK_RUN / 2, 0.0, -HANDLE_H / 2 - NECK_DROP / 2 + 0.002])
    shapes.append(BoxShape(tuple(mid.tolist()), (neck_len + 0.006, 0.014, 0.003), STEEL,
                           quat_xyzw=(0.0, math.sin(a / 2), 0.0, math.cos(a / 2))))
    shapes.append(BoxShape((BLADE_X0 + BLADE_L / 2, 0.0, BLADE_Z), (BLADE_L, BLADE_W, BLADE_T), STEEL))
    for k in range(4):  # slots in the blade (seen only)
        shapes.append(BoxShape((BLADE_X0 + 0.012 + k * 0.014, 0.0, BLADE_Z + BLADE_T / 2 + 0.0002),
                               (0.004, BLADE_W * 0.6, 0.0004), DARK, collide=False))
    return Body.build("turner", pos=(TURNER_AT[0], TURNER_AT[1], TOP + REST[2] + HANDLE_H / 2), mass=0.09,
                      shapes=shapes, color=STEEL, friction=(0.5, 0.4), com=(0.0, 0.0, -0.006))


def _rest() -> Body:
    shapes = [BoxShape((0.0, 0.0, REST[2] / 2), REST, WHITE),
              BoxShape((0.0, 0.0, REST[2] + 0.002), (REST[0], 0.010, 0.004), WHITE)]
    return Body.build("turner_rest", pos=(REST_X, TURNER_AT[1], TOP), mass=0.3, shapes=shapes, kinematic=True,
                      color=WHITE)


OBJECTS = [_hob(), _pan(), _pancake(), _rest(), _turner()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CAKE_DEBUG")), "steps": 0}


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


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c_pos, c_rot = _pose(base, origin, "pancake")
    p_pos, p_rot = _pose(base, origin, "pan")
    t_pos, t_rot = _pose(base, origin, "turner")
    up = float(c_rot[2, 2])
    mid = c_pos + c_rot @ np.array([0.0, 0.0, (BASE_H + CAKE_H) / 2])  # the pancake's middle
    v = p_rot.T @ (mid - p_pos)
    in_pan = math.hypot(float(v[0]), float(v[1])) < PAN_R - 0.02 and float(v[2]) < PAN_FLOOR + 0.012  # flat on it
    on_hob = (abs(p_pos[0] - HOB_AT[0]) < HOB[0] / 2 and abs(p_pos[1] - HOB_AT[1]) < HOB[1] / 2
              and float(p_rot[2, 2]) > math.cos(math.radians(10.0)))
    t_off = float(np.linalg.norm(t_pos[:2] - TURNER_AT)) if t_pos is not None else 1.0
    t_yaw = abs(math.degrees(math.atan2(float(t_rot[1, 0]), float(t_rot[0, 0]))))
    on_rest = t_off < 0.03 and t_yaw < 15.0 and t_pos[2] < TOP + REST[2] + HANDLE_H / 2 + 0.01
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = {"flipped": up < -0.8, "in_pan": in_pan and on_hob, "on_rest": on_rest}
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and _DEBUG["steps"] % 10 == 0:
        tl, tr = tcps
        blade = t_pos + t_rot @ np.array([BLADE_X0 + BLADE_L, 0.0, BLADE_Z - BLADE_T / 2])
        print(f"[cake] t={_DEBUG['steps'] / 20:5.1f}s up={up:5.2f} cake={np.round(mid * 1000).astype(int).tolist()} "
              f"in_pan={in_pan} pan={np.round(p_pos * 1000).astype(int).tolist()} on_hob={on_hob} "
              f"tip={np.round(blade * 1000).astype(int).tolist()} turner_off={t_off * 1000:5.1f} yaw={t_yaw:5.1f} "
              f"on_rest={on_rest} home={_arms_home()} L={np.round(tl * 1000).astype(int).tolist()} "
              f"R={np.round(tr * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"flipped": bool(st.get("flipped")), "in_pan": bool(st.get("in_pan")), "on_rest": bool(st.get("on_rest")),
            "home": _arms_home()}


def _points(s) -> float:
    if s["flipped"] and s["in_pan"] and s["on_rest"] and s["home"]:
        return 1.0
    if s["flipped"] and s["in_pan"]:
        return 0.5
    return 0.25 if s["flipped"] else 0.0


def score(objects) -> float:
    """Judged at the end. Flipped: the pancake's golden side faces down (its axis within 37 degrees of straight down).
    In the pan: the pancake's middle within 12 cm of the pan's middle and within 1.2 cm of its floor (lying on it, not
    leaning on the wall), with the pan still on the hob
    and upright. On its rest: the turner's handle middle within 3 cm of where it started, turned less than 15 degrees,
    down on its rest. 1.0 when the pancake is flipped and in the pan, the turner is on its rest and both arms are back
    where they started (each TCP within 5 cm of its start); 0.5 when the pancake is flipped and in the pan; 0.25 when
    it is flipped anywhere; else 0.0."""
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
    """Full score with the pancake, the pan and the turner still."""
    return (score(objects) >= 1.0 and objects["pancake"].resting() and objects["pan"].resting()
            and objects["turner"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the pan's handle near its end, fingers closing across it, and holds the pan.
# The right takes the turner by its handle, lifts it off its rest, turns it to point at the pan and lowers the blade
# flat onto the pan's floor right of the pancake; it slides the blade in under the pancake's right edge, then lifts
# that edge up and over toward the left, until the pancake stands on edge and topples onto its other side. The right
# draws the turner back out, lays it on its rest and goes back to where it started; the left lets go of the pan and goes
# back to where it started too.

L_PAN = grip_rotation(-math.pi / 2)  # left: top-down, fingers closing along x, across the pan's handle
R_LIE = grip_rotation(0.0)  # right: the turner as it lies (handle along x), fingers closing along y
R_USE = grip_rotation(math.pi / 2)  # right: the turner pointing +y at the pan, fingers closing along x
PAN_GRIP_Y = PAN_R + PAN_HANDLE[1] - 0.045
TURNER_OPEN = -(HANDLE_W / 2 + 0.006)
PRESS = 0.0015  # the blade aimed this far below the pan's floor as it slides in
TUCK = 0.014  # how far the blade's tip goes in under the pancake's edge
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


def _grip_for_tip(tip_y: float, tip_z: float) -> np.ndarray:
    """The TCP that puts the turner's blade tip (its front edge's underside) at (PAN_AT x, tip_y, tip_z), the turner
    pointing +y, held at GRIP_X."""
    back = BLADE_X0 + BLADE_L - GRIP_X
    return np.array([PAN_AT[0], tip_y - back, tip_z - (BLADE_Z - BLADE_T / 2)])


def demo():
    grip_lie = np.array([TURNER_AT[0] + GRIP_X, TURNER_AT[1], TOP + REST[2] + HANDLE_H / 2 + 0.002])
    left = _lead_in("left", [PAN_AT[0], PAN_GRIP_Y, PAN_Z + PAN_WALL_H + 0.05], L_PAN, "over the pan's handle",
                    -(PAN_HANDLE[0] / 2 + 0.003))
    pan_grip = np.array([PAN_AT[0], PAN_GRIP_Y, PAN_Z + PAN_WALL_H - 0.006 + 0.004])
    left += [_w(1.0, pan_grip, -(PAN_HANDLE[0] / 2 + 0.003), "down round it", L_PAN),
             _w(0.6, pan_grip, FINGER_CLOSED, "hold the pan by its handle", L_PAN)]
    held_at = sum(w.duration for w in left)

    right = _lead_in("right", grip_lie + [0.0, 0.0, 0.05], R_LIE, "over the turner's handle", TURNER_OPEN)
    right += [_w(1.0, grip_lie, TURNER_OPEN, "down round it", R_LIE),
              _w(0.6, grip_lie, FINGER_CLOSED, "take the turner", R_LIE),
              _w(1.0, grip_lie + [0.0, 0.0, 0.08], FINGER_CLOSED, "lift it off its rest", R_LIE)]
    edge = CAKE_AT[1] - CAKE_R  # the pancake's right edge
    start = _grip_for_tip(edge - 0.006, FLOOR + 0.004)  # the whole blade inside the pan's wall
    right += [_w(2.0, start + [0.0, 0.0, 0.06], FINGER_CLOSED, "turn it to point at the pan, over the floor", R_USE)]
    t = sum(w.duration for w in right)
    if held_at > t:
        right.append(_w(held_at - t, right[-1].pos, FINGER_CLOSED, "wait for the pan to be held", R_USE))
    right += [_w(1.2, start + [0.0, 0.0, -PRESS], FINGER_CLOSED, "lay the blade flat on the pan's floor", R_USE),
              _w(1.5, _grip_for_tip(edge + TUCK, FLOOR - PRESS), FINGER_CLOSED,
                 "slide the blade in under the pancake's edge", R_USE)]
    # lift the edge up and over: the blade's tip goes round the pancake's far edge, which stays on the floor
    pivot_y = CAKE_AT[1] + CAKE_R
    d = 2 * CAKE_R - TUCK
    for k in range(1, 11):
        phi = math.radians(130.0 * k / 10)  # on past standing on edge, so it topples over
        right.append(_w(0.4, _grip_for_tip(pivot_y - d * math.cos(phi) - 0.004, FLOOR + d * math.sin(phi) + 0.004),
                        FINGER_CLOSED, "lift its edge up and over: it turns over", R_USE))
    top = right[-1].pos
    right += [_w(0.6, [top[0], top[1], top[2] + 0.04], FINGER_CLOSED, "lift the turner clear", R_USE),
              _w(0.8, [top[0], top[1] - 0.06, top[2] + 0.05], FINGER_CLOSED, "draw it back", R_USE),
              _w(2.0, grip_lie + [0.0, 0.0, 0.08], FINGER_CLOSED, "carry it back over its rest", R_LIE),
              _w(1.2, grip_lie + [0.0, 0.0, 0.003], FINGER_CLOSED, "lay it on its rest", R_LIE),
              _w(0.5, grip_lie + [0.0, 0.0, 0.003], TURNER_OPEN, "let go", R_LIE),
              _w(0.7, grip_lie + [0.0, 0.0, 0.06], TURNER_OPEN, "rise", R_LIE)]
    right_done = sum(w.duration for w in right)
    right += _go_home("right", right[-1].pos, R_LIE, FINGER_OPEN)
    left.append(_w(max(right_done - 3.0 - sum(w.duration for w in left), 0.3), pan_grip, FINGER_CLOSED,
                   "hold the pan still while the pancake is turned", L_PAN))
    left += [_w(0.5, pan_grip, -(PAN_HANDLE[0] / 2 + 0.003), "let go", L_PAN),
             _w(0.8, pan_grip + [0.0, 0.0, 0.05], -(PAN_HANDLE[0] / 2 + 0.003), "rise", L_PAN)]
    left += _go_home("left", left[-1].pos, L_PAN, FINGER_OPEN)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
