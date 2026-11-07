"""On the station table stands the pivot of a loader arm: a yellow steel lug on a base plate with a steel pin through it,
and on the lug's top a grease nipple. To the left stands a grease pail with a lever pump on its lid; its black hose
runs over the table to a push-on coupler standing in a steel holder cup.

Task: grease the pivot. One hand takes the coupler from its holder, presses it onto the grease nipple and holds it
there (a push-on coupler is pushed off by the grease pressure unless it is held on), while the other hand pumps the
pail's lever down at least four full strokes; grease squeezes out of the pin's ends. Then pull the coupler off, put it
back in its holder and take both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, ball, hinge,  # noqa: E402
                     on_reset, on_step, rotation, segment)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the loader pivot: a lug with a pin through it and a grease nipple on top ------------------------------------------
MX, MY = 0.30, -0.08
LUG = (0.05, 0.04, 0.05)
BASE_T = 0.010
PIN_Z = 0.030  # the pin's axis above the table
Z_SEAT = TOP + BASE_T + LUG[2] + 0.006  # the boss's top, where the coupler's mouth seats
NIPPLE_TOP = Z_SEAT + 0.019

# -- the grease pail with its lever pump ---------------------------------------------------------------------------
PX, PY = 0.37, 0.20
PAIL_R, PAIL_H = 0.065, 0.085
PIVOT = np.array([PX - 0.015, PY, TOP + 0.120])  # the lever's pivot on the pump head
LEVER_L = 0.125  # from the pivot to the middle of the lever's grip pad
LEVER_UP = 15.0  # degrees the lever stands up from level at rest
PAD = (0.030, 0.028, 0.016)
STROKE_DOWN, STROKE_UP = 18.0, 10.0  # degrees pressed down: a stroke is down past 18, then back up past 10
STROKES = 4
OUTLET = (PX, PY - PAIL_R - 0.006, TOP + 0.070)

# -- the coupler (origin: the middle of its mouth) and its holder cup ------------------------------------------------
HX, HY = 0.24, 0.0
CUP_IN, CUP_H, CUP_FLOOR = 0.036, 0.030, 0.004
BORE = 0.013  # half the nozzle's bore
SLEEVE_R, SLEEVE_Z0, SLEEVE_Z1 = 0.012, 0.025, 0.065
STUB_Z, STUB_L = 0.057, 0.035
SEATED_OFF, SEATED_UP, SEATED_TILT = 0.009, 0.003, 20.0  # mouth within 9 mm of the nipple's axis, on the boss
PUSH_OFF, PUSH_UP = 0.013, 0.014  # while the mouth is this close round the nipple, the grease pressure pushes it off
PUSH_N, PUSH_SIDE_N = 2.5, 0.2

# -- the hose: five segments on ball joints from the pump's outlet to the coupler's elbow ---------------------------
HOSE = [OUTLET, (PX, 0.120, TOP + 0.007), (0.40, 0.060, TOP + 0.007), (0.38, 0.0, TOP + 0.007),
        (0.31, 0.0, TOP + 0.007), (HX + STUB_L, HY, TOP + CUP_FLOOR + STUB_Z)]

YELLOW, STEEL, DARK, RED, BRASS, BLACK, GREASE, ZINC = (0.93, 0.68, 0.10), (0.72, 0.74, 0.77), (0.20, 0.21, 0.23), \
    (0.72, 0.12, 0.10), (0.80, 0.66, 0.30), (0.06, 0.06, 0.06), (0.62, 0.45, 0.12), (0.62, 0.64, 0.66)
GREASE_SPOTS = [(MX + 0.006, MY + 0.0235, TOP + PIN_Z + 0.017), (MX + 0.006, MY - 0.0235, TOP + PIN_Z + 0.017),
                (MX - 0.010, MY + 0.0235, TOP + PIN_Z + 0.015), (MX - 0.010, MY - 0.0235, TOP + PIN_Z + 0.015)]
PARKED = [(0.55, -0.40 + 0.03 * k, TOP - 0.12) for k in range(4)]  # under the table until squeezed out

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _pivot() -> Body:
    """The loader arm's pivot: a base plate, a yellow lug, a steel pin through it (washers, cotter pins) with the
    arm's two side plates on it, a boss on the lug's top and the grease nipple. Origin on the table under the lug."""
    shapes = [BoxShape((0.0, 0.0, BASE_T / 2), (0.12, 0.10, BASE_T), DARK),
              BoxShape((0.0, 0.0, BASE_T + LUG[2] / 2), LUG, YELLOW),
              CylShape((0.0, 0.0, PIN_Z), 0.010, 0.100, STEEL, axis="Y")]
    for s in (-1.0, 1.0):
        shapes += [CylShape((0.0, s * 0.024, PIN_Z), 0.016, 0.004, ZINC, axis="Y", collide=False),  # washers
                   BoxShape((0.0, s * 0.031, PIN_Z), (0.042, 0.008, 0.034), YELLOW, collide=False),  # side plates
                   BoxShape((-0.035, s * 0.031, PIN_Z + 0.004), (0.035, 0.008, 0.026), YELLOW, collide=False),
                   BoxShape((0.0, s * 0.047, PIN_Z), (0.003, 0.003, 0.022), STEEL, collide=False)]  # cotter pins
    shapes += [CylShape((0.0, 0.0, Z_SEAT - TOP - 0.003), 0.012, 0.006, STEEL),  # the boss
               BoxShape((0.0, 0.0, Z_SEAT - TOP + 0.003), (0.010, 0.010, 0.006), BRASS),  # the nipple's hex
               BoxShape((0.0, 0.0, Z_SEAT - TOP + 0.003), (0.010, 0.010, 0.006), BRASS, yaw=math.pi / 4,
                        collide=False),
               CylShape((0.0, 0.0, Z_SEAT - TOP + 0.008), 0.0025, 0.004, BRASS),  # its neck
               SphereShape((0.0, 0.0, NIPPLE_TOP - TOP - 0.0045), 0.0045, BRASS)]  # its ball head
    return Body.build("pivot", pos=(MX, MY, TOP), mass=5.0, shapes=shapes, kinematic=True, color=YELLOW)


def _pump() -> Body:
    """The grease pail (red, with a yellow band) and its pump on the lid: a column, the head with the lever's
    bracket and the hose outlet on the pail's right side. Origin on the table under the pail's middle."""
    hx = PIVOT[0] - PX
    shapes = [CylShape((0.0, 0.0, PAIL_H / 2), PAIL_R, PAIL_H, RED),
              CylShape((0.0, 0.0, PAIL_H * 0.55), PAIL_R + 0.001, 0.022, YELLOW, collide=False),
              CylShape((0.0, 0.0, PAIL_H + 0.003), PAIL_R + 0.002, 0.006, DARK),
              CylShape((0.0, 0.0, (PAIL_H + 0.006 + 0.108) / 2), 0.011, 0.108 - PAIL_H - 0.006, STEEL),
              BoxShape((hx + 0.008, 0.0, 0.112), (0.040, 0.030, 0.022), DARK),  # the pump head
              BoxShape((hx, 0.0, 0.120), (0.014, 0.034, 0.012), DARK, collide=False),  # the lever's bracket
              CylShape((hx, 0.0, 0.120), 0.004, 0.040, STEEL, axis="Y", collide=False),  # its pivot pin
              CylShape((0.0, -PAIL_R - 0.003, OUTLET[2] - TOP), 0.008, 0.012, BRASS, axis="Y", collide=False)]
    return Body.build("pump", pos=(PX, PY, TOP), mass=6.0, shapes=shapes, kinematic=True, color=RED)


def _lever() -> Body:
    """The pump's lever: a steel bar from the pivot toward the robot, up LEVER_UP, with a red rubber grip pad at its
    end. Origin at the pivot."""
    a = math.radians(LEVER_UP)
    end = np.array([-LEVER_L * math.cos(a), 0.0, LEVER_L * math.sin(a)])
    shapes = [segment((0.0, 0.0, 0.0), tuple((end * 1.06).tolist()), 0.010, STEEL),
              BoxShape(tuple(end.tolist()), PAD, RED)]
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=0.12, shapes=shapes, color=STEEL,
                      com=tuple((end * 0.55).tolist()), friction=(1.0, 0.9), ignore=("pump",))


def _holder() -> Body:
    """A steel holder cup for the coupler, on a small foot plate."""
    shapes = [CylShape((0.0, 0.0, CUP_FLOOR / 2), CUP_IN + 0.004, CUP_FLOOR, STEEL),
              BoxShape((-0.03, 0.0, 0.0015), (0.03, 0.05, 0.003), DARK, collide=False)]
    for k in range(10):
        a = k * 2 * math.pi / 10
        shapes.append(BoxShape(((CUP_IN + 0.0015) * math.cos(a), (CUP_IN + 0.0015) * math.sin(a), CUP_H / 2),
                               (0.003, 0.025, CUP_H), STEEL, yaw=a))
    return Body.build("holder", pos=(HX, HY, TOP), mass=1.0, shapes=shapes, kinematic=True, color=STEEL)


def _hose(i: int) -> Body:
    a, b = np.asarray(HOSE[i], float), np.asarray(HOSE[i + 1], float)
    shapes = [segment((0.0, 0.0, 0.0), tuple((b - a).tolist()), 0.012, BLACK)]
    ign = ("pump", "lever") if i == 0 else ("pump", "lever", f"hose{i - 1}")
    return Body.build(f"hose{i}", pos=tuple(a.tolist()), mass=0.012, shapes=shapes, color=BLACK,
                      damping=(0.6, 0.6), friction=(0.8, 0.7), ignore=ign)


def _coupler() -> Body:
    """A push-on grease coupler: a square steel nozzle with a 26 mm bore (drawn round), a cap inside it, a knurled
    black grip sleeve above, and a brass elbow at the top where the hose comes in. Origin: the middle of its mouth."""
    w, t, h = BORE, 0.003, 0.018
    shapes = [BoxShape((w + t / 2, 0.0, h / 2), (t, 2 * (w + t), h), None),
              BoxShape((-(w + t / 2), 0.0, h / 2), (t, 2 * (w + t), h), None),
              BoxShape((0.0, w + t / 2, h / 2), (2 * w, t, h), None),
              BoxShape((0.0, -(w + t / 2), h / 2), (2 * w, t, h), None),
              BoxShape((0.0, 0.0, 0.023), (2 * (w + t), 2 * (w + t), 0.004), ZINC),  # the cap
              CylShape((0.0, 0.0, h / 2), w + t + 0.0005, h, ZINC, collide=False),  # the nozzle, drawn round
              CylShape((0.0, 0.0, (SLEEVE_Z0 + SLEEVE_Z1) / 2), SLEEVE_R, SLEEVE_Z1 - SLEEVE_Z0, BLACK),
              CylShape((STUB_L / 2, 0.0, STUB_Z), 0.006, STUB_L, BRASS, axis="X"),
              CylShape((0.0, 0.0, SLEEVE_Z1 + 0.003), 0.008, 0.006, BRASS, collide=False)]
    for k in range(5):  # knurling (drawn)
        shapes.append(CylShape((0.0, 0.0, SLEEVE_Z0 + 0.006 + k * 0.006), SLEEVE_R + 0.0006, 0.0015, DARK,
                               collide=False))
    return Body.build("coupler", pos=(HX, HY, TOP + CUP_FLOOR), mass=0.09, shapes=shapes, color=ZINC,
                      com=(0.0, 0.0, 0.022), friction=(1.2, 1.1), ignore=("hose4",))


def _grease(k: int) -> Body:
    """A dab of grease squeezed out of the pin's bushing (drawn only), parked under the table until it appears."""
    shapes = [SphereShape((0.0, 0.0, 0.0), 0.0060, GREASE, collide=False),
              SphereShape((0.006, 0.0, -0.002), 0.0050, GREASE, collide=False),
              SphereShape((-0.005, 0.0, -0.003), 0.0045, GREASE, collide=False)]
    return Body.build(f"grease{k}", pos=PARKED[k], mass=0.01, shapes=shapes, kinematic=True, color=GREASE)


OBJECTS = [_pivot(), _pump(), _lever(), _holder()] + [_hose(i) for i in range(5)] + [_coupler()] + \
    [_grease(k) for k in range(4)]
hinge("pump_lever", "pump", "lever", anchor=tuple(PIVOT.tolist()), axis="Y", limits_deg=(-26.0, 8.0), spring=0.02,
      damping=0.001, target_deg=4.0)
ball("hose_out", "pump", "hose0", anchor=HOSE[0], swing_damping=0.002, twist_damping=0.002)
for _i in range(1, 5):
    ball(f"hose_j{_i}", f"hose{_i - 1}", f"hose{_i}", anchor=HOSE[_i], swing_damping=0.002, twist_damping=0.002)
ball("hose_in", "hose4", "coupler", anchor=HOSE[5], swing_damping=0.002, twist_damping=0.002)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"down": False, "strokes": 0, "shots": 0, "seated_ever": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GREASE_DEBUG")), "steps": 0}


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


def _set_force(asset, f_world, rot) -> None:
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _coupler_state(cp, cr) -> dict:
    off = math.hypot(cp[0] - MX, cp[1] - MY)
    up = cp[2] - Z_SEAT
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[2, 2])))))
    seated = off <= SEATED_OFF and up <= SEATED_UP and tilt <= SEATED_TILT
    pushed = off <= PUSH_OFF and up <= PUSH_UP and tilt <= 45.0
    in_holder = math.hypot(cp[0] - HX, cp[1] - HY) <= CUP_IN - 0.004 and cp[2] < TOP + CUP_H and tilt <= 40.0
    return {"off": off, "up": up, "tilt": tilt, "seated": seated, "pushed": pushed, "in_holder": in_holder}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    cp, cr = _pose(base, origin, "coupler")
    _, lr = _pose(base, origin, "lever")
    c = _coupler_state(cp, cr)
    # the grease pressure pushes a coupler off the nipple unless it is held on
    _set_force(base.scene["coupler"], (PUSH_SIDE_N, 0.0, PUSH_N) if c["pushed"] else (0.0, 0.0, 0.0), cr)
    pressed = -math.degrees(math.atan2(-lr[2, 0], lr[0, 0]))  # degrees the lever is pressed down from rest
    if not _EP["down"] and pressed >= STROKE_DOWN:
        _EP["down"] = True
        if c["seated"]:  # a full stroke with the coupler held on squeezes grease in
            _EP["shots"] += 1
            if _EP["shots"] <= len(GREASE_SPOTS):
                _place(base, origin, f"grease{_EP['shots'] - 1}", GREASE_SPOTS[_EP["shots"] - 1])
    elif _EP["down"] and pressed <= STROKE_UP:
        _EP["down"] = False
        _EP["strokes"] += 1
    _EP["seated_ever"] = _EP["seated_ever"] or c["seated"]
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = dict(c, pressed=pressed)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, trc = _EP["tcps"]
            print(f"[grease] t={_DEBUG['steps'] / 20:5.1f}s lever={pressed:5.1f}deg strokes={_EP['strokes']} "
                  f"shots={_EP['shots']} coupler={np.round(cp * 1000).astype(int).tolist()} off={c['off'] * 1000:5.1f}mm "
                  f"up={c['up'] * 1000:6.1f}mm tilt={c['tilt']:5.1f} seated={c['seated']} pushed={c['pushed']} "
                  f"in_holder={c['in_holder']} leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000).astype(int).tolist()} score={score(None)}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the pivot greased (at least four full strokes of the pump's lever, each pressed down while the coupler
    was seated on the nipple), the coupler back upright in its holder cup and both arms back where they started; 0.5
    with the pivot greased at some point; 0.25 with the coupler seated on the nipple at some point; else 0.0."""
    if _EP["shots"] >= STROKES:
        if _EP["s"].get("in_holder") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["seated_ever"] else 0.0


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
    """Full score with the coupler still in its holder: the trial ends there."""
    return score(objects) >= 1.0 and objects["coupler"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down round the coupler's grip sleeve in its holder, takes it, lifts it, carries it over the
# nipple and presses it down onto it, and holds it there. The left hand comes down on the lever's grip pad, fingers
# closed, and pumps the lever five full strokes, following its swing, then rises and goes back to where it started.
# The right pulls the coupler off, carries it back over the holder, lowers it into the cup, lets go and goes back to
# where it started.

GRIP = grip_rotation(0.0)  # straight down, fingers closing along y
SLEEVE_TCP = 0.050  # the TCP this far above the coupler's mouth when holding it by its sleeve
PART_OPEN = -0.026
PRESS = 0.004  # the right hand aims this far below seated, pressing the coupler on
PUMPS = 5
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


def _pad_top(pressed: float) -> np.ndarray:
    """The top of the lever's grip pad with the lever pressed ``pressed`` degrees down from rest."""
    a = math.radians(LEVER_UP - pressed)
    return PIVOT + np.array([-LEVER_L * math.cos(a), 0.0, LEVER_L * math.sin(a) + PAD[2] / 2])


def _right(release_at: float) -> tuple[list[Waypoint], float]:
    """The right hand; returns its waypoints and when the coupler is seated."""
    take = np.array([HX, HY, TOP + CUP_FLOOR + SLEEVE_TCP])
    out = _lead_in("right", take + [0.0, 0.0, 0.05], GRIP, "over the coupler in its holder", PART_OPEN)
    carry_z = Z_SEAT + SLEEVE_TCP + 0.035
    seat = np.array([MX - 0.003, MY, Z_SEAT + SLEEVE_TCP - PRESS])
    out += [_w(1.0, take, PART_OPEN, "down round its grip sleeve", GRIP),
            _w(0.6, take, FINGER_CLOSED, "take it", GRIP),
            _w(1.0, [HX, HY, carry_z], FINGER_CLOSED, "lift it out of the holder", GRIP),
            _w(1.6, [MX, MY, carry_z], FINGER_CLOSED, "carry it over the grease nipple", GRIP),
            _w(1.4, seat, FINGER_CLOSED, "press it down onto the nipple", GRIP)]
    seated_at = sum(w.duration for w in out)
    out += [_w(max(release_at - seated_at, 0.5), seat, FINGER_CLOSED, "hold it on while the other hand pumps", GRIP),
            _w(1.0, [MX, MY, carry_z], FINGER_CLOSED, "pull it off the nipple", GRIP),
            _w(1.6, [HX, HY, carry_z], FINGER_CLOSED, "carry it back over the holder", GRIP),
            _w(1.2, take + [0.0, 0.0, 0.004], FINGER_CLOSED, "lower it into the cup", GRIP),
            _w(0.5, take + [0.0, 0.0, 0.004], PART_OPEN, "let go", GRIP),
            _w(0.8, take + [0.0, 0.0, 0.05], PART_OPEN, "rise", GRIP)]
    out += _go_home("right", take + [0.0, 0.0, 0.05], GRIP, PART_OPEN)
    return out, seated_at


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    """The left hand; returns its waypoints and when it is done pumping."""
    over = _pad_top(0.0) + [0.0, 0.0, 0.03]
    out = _lead_in("left", over, GRIP, "over the pump lever's grip pad", FINGER_CLOSED)
    out.append(_w(max(start - LEAD_S, 0.3), over, FINGER_CLOSED, "wait for the coupler to be held on", GRIP))
    out.append(_w(0.8, _pad_top(0.0) - [0.0, 0.0, 0.002], FINGER_CLOSED, "down onto the pad", GRIP))
    for _ in range(PUMPS):
        for k in range(1, 5):
            out.append(_w(0.25, _pad_top(25.0 * k / 4) - [0.0, 0.0, 0.004], FINGER_CLOSED,
                          "press the lever down a full stroke", GRIP))
        for k in range(3, -2, -1):
            out.append(_w(0.3, _pad_top(25.0 * k / 4) + [0.0, 0.0, 0.003], FINGER_CLOSED,
                          "let it come back up", GRIP))
    out.append(_w(0.8, over, FINGER_CLOSED, "rise off the pad", GRIP))
    done = sum(w.duration for w in out)
    out += _go_home("left", over, GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, done


def demo():
    _, seated_at = _right(0.0)
    _, pumped_at = _left(seated_at + 0.5, 0.0)
    right, _ = _right(pumped_at + 0.3)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, PART_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(seated_at + 0.5, total)
    return {"left": left, "right": right}
