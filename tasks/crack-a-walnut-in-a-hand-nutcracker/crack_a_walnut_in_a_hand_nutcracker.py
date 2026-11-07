"""On the station table lies a steel hand nutcracker, its two long handles open in a V toward the arms, on a smooth
worktop, with a walnut between its jaws near the pivot. A small white bowl waits to the right and a saucer to the left.

Task: crack the walnut. The two handles are pushed toward each other from both sides, one hand on each (pushed from
one side only, the light nutcracker just slides across the smooth worktop with the walnut and nothing cracks), until
the jaws crush the shell. Then the kernel goes into the bowl, both shell halves go onto the saucer, and both arms go
back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=crack_a_walnut_in_a_hand_nutcracker.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the nutcracker: two steel levers on a pivot, open in a V toward the arms -------------------------------------------
PX, PY = 0.44, 0.0  # the pivot
OPEN_DEG = 30.0  # each lever's angle from the V's middle line when open (the spring holds them there)
SHUT_LIMIT = 20.0  # the levers can close no further than this (each lever's angle)
BEND = 0.20  # each lever runs straight out from the pivot to here, then bends into its handle ...
PARALLEL_DEG = 25.0  # ... which runs parallel to the V's middle line when the lever is this far from it
GRIP_LEN, GRIP_W, GRIP_H = 0.090, 0.022, 0.022  # the red handle grip
BAR_W, BAR_H = 0.014, 0.012  # the steel bar
JAW = (0.040, 0.098, 0.018, 0.034)  # the jaw: from, to (along the lever), width, height
LEVER_MASS = 0.16
SPRING = 0.04  # N m per degree, opening the levers ...
PRELOAD = 8.0  # ... toward 8 degrees past fully open, so they rest open against the stop

# -- the walnut: 3.5 cm, on the worktop between the jaws, 6 cm out from the pivot ---------------------------------------
NUT_S = 0.060
NUT_R, NUT_CORE = 0.0175, 0.012  # what is seen; what the jaws touch (they crush the shell in by up to 5.5 mm)
NUT_Z = 0.0125  # its middle above the worktop (it rests on a hidden flat-ended core 2.5 cm tall)
CRACK_GAP = 0.029  # cracked once the jaws close to this gap where the walnut sits (3.5 cm across)
JAW_IN = JAW[2] - BAR_W / 2  # a jaw's inner face, this far in from its lever's middle line
PARK = (0.30, 0.0, TOP + 1.5)  # pieces wait out of sight in a closed box up here

# -- after it cracks: the kernel and two shell halves fall out along the V ----------------------------------------------
KERNEL_S, SHELL_S = 0.100, (0.150, 0.200)
KERNEL = (0.026, 0.020, 0.020)  # its size: along x, across, tall
SHELL_R, SHELL_H = 0.016, 0.0205

BOWL_AT, BOWL_R, BOWL_H = np.array([0.27, -0.25]), 0.050, 0.040
DISH_AT, DISH_R = np.array([0.27, 0.25]), 0.062

STEEL, RED, BROWN, DARK, KERNEL_C, WHITE, BLUE, HOLE = (0.72, 0.73, 0.75), (0.80, 0.12, 0.10), (0.55, 0.40, 0.24), \
    (0.42, 0.30, 0.18), (0.86, 0.72, 0.50), (0.95, 0.95, 0.93), (0.30, 0.50, 0.85), (0.10, 0.10, 0.10)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _u(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


def _lever(name: str, side: float) -> Body:
    """A lever in its own frame: origin on the pivot, unrotated; its shapes laid out along the open angle."""
    phi = 180.0 - side * OPEN_DEG  # the left lever (+1) points back and left, the right (-1) back and right
    u, yaw = _u(phi), math.radians(phi)
    shapes = [BoxShape(tuple((u * (BEND + 0.006) / 2 + [0, 0, BAR_H / 2]).tolist()), (BEND + 0.006, BAR_W, BAR_H),
                       STEEL, yaw=yaw)]
    g_phi = 180.0 - side * (OPEN_DEG - PARALLEL_DEG)  # the handle's direction
    g = _u(g_phi)
    mid = u * BEND + g * GRIP_LEN / 2
    shapes.append(BoxShape(tuple((mid + [0, 0, GRIP_H / 2]).tolist()), (GRIP_LEN, GRIP_W, GRIP_H), RED,
                           yaw=math.radians(g_phi)))
    shapes.append(CylShape(tuple((u * BEND + [0, 0, BAR_H / 2]).tolist()), BAR_W / 2, BAR_H, STEEL))  # the bend
    a, b, w, h = JAW
    n = np.array([-u[1], u[0], 0.0]) * side  # toward the V's middle
    shapes.append(BoxShape(tuple((u * (a + b) / 2 + n * (w - BAR_W) / 2 + [0, 0, h / 2]).tolist()), (b - a, w, h),
                           STEEL, yaw=yaw))
    for k in range(5):  # the jaw's teeth (seen only)
        r = a + 0.006 + k * (b - a - 0.012) / 4
        shapes.append(BoxShape(tuple((u * r + n * (w - BAR_W / 2 + 0.0005) + [0, 0, h / 2]).tolist()),
                               (0.003, 0.002, h * 0.8), (0.55, 0.56, 0.58), yaw=yaw, collide=False))
    if side > 0:  # the pivot pin (seen only)
        shapes.append(CylShape((0.0, 0.0, 0.016), 0.009, 0.032, (0.60, 0.61, 0.63), collide=False))
    return Body.build(name, pos=(PX, PY, TOP), mass=LEVER_MASS, shapes=shapes, friction=(0.30, 0.25), color=STEEL,
                      damping=(0.0, 0.05))


def _nut_at(s: float) -> np.ndarray:
    return np.array([PX - s, PY, TOP])


def _walnut() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), NUT_CORE, 2 * NUT_Z, BROWN)]
    shapes.append(SphereShape((0.0, 0.0, 0.0), NUT_R, BROWN, collide=False))
    for i in range(10):  # wrinkles (seen only)
        a = 2 * math.pi * i / 10
        z = 0.006 * (1 if i % 2 else -1)
        shapes.append(SphereShape((NUT_R * 0.82 * math.cos(a), NUT_R * 0.82 * math.sin(a), z), 0.0045, DARK,
                                  collide=False))
    shapes.append(BoxShape((0.0, 0.0, 0.0), (2 * NUT_R + 0.001, 0.0025, 2 * NUT_R + 0.001), DARK, collide=False))
    p = _nut_at(NUT_S)
    return Body.build("walnut", pos=(p[0], p[1], TOP + NUT_Z), mass=0.012, shapes=shapes, friction=(0.8, 0.7),
                      color=BROWN, damping=(0.2, 2.0))


def _kernel() -> Body:
    lx, ly, lz = KERNEL
    shapes = [BoxShape((0.0, 0.0, lz * 0.475), (lx * 0.85, ly * 0.9, lz * 0.95), KERNEL_C)]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shapes.append(SphereShape((sx * lx * 0.28, sy * ly * 0.22, lz * 0.5), lz * 0.42, KERNEL_C, collide=False))
    shapes.append(BoxShape((0.0, 0.0, lz * 0.9), (lx * 0.7, 0.002, 0.002), (0.70, 0.55, 0.35), collide=False))
    return Body.build("kernel", pos=(PARK[0], PARK[1], PARK[2] + 0.005), mass=0.008, shapes=shapes,
                      friction=(1.0, 0.9), color=KERNEL_C, damping=(0.2, 1.0))


def _shell(name: str, dy: float) -> Body:
    # what is touched: a flat-sided block hidden inside the round shell (fingers hold flat faces, not a curve)
    shapes, z = [BoxShape((0.0, 0.0, 0.0085), (0.019, 0.019, 0.017), BROWN)], 0.0
    for r, h in ((SHELL_R, 0.015), (SHELL_R * 0.81, 0.003), (SHELL_R * 0.5, 0.0025)):
        shapes.append(CylShape((0.0, 0.0, z + h / 2), r, h, BROWN, collide=False))
        z += h
    for i in range(6):
        a = 2 * math.pi * i / 6
        shapes.append(SphereShape((SHELL_R * 0.92 * math.cos(a), SHELL_R * 0.92 * math.sin(a), 0.010), 0.0035, DARK,
                                  collide=False))
    shapes.append(CylShape((0.0, 0.0, 0.0006), SHELL_R * 0.8, 0.0012, (0.80, 0.68, 0.48), collide=False))
    return Body.build(name, pos=(PARK[0] + 0.02, PARK[1] + dy, PARK[2] + 0.005), mass=0.008, shapes=shapes,
                      friction=(1.0, 0.9), color=BROWN, damping=(0.2, 1.0))


def _park() -> Body:
    w, t = 0.10, 0.004
    shapes = [BoxShape((0.0, 0.0, t / 2), (w, w, t), HOLE), BoxShape((0.0, 0.0, w - t / 2), (w, w, t), HOLE)]
    shapes += [BoxShape((s * (w / 2 - t / 2), 0.0, w / 2), (t, w, w), HOLE) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (w / 2 - t / 2), w / 2), (w, t, w), HOLE) for s in (-1.0, 1.0)]
    return Body.build("piece_store", pos=PARK, mass=1.0, shapes=shapes, kinematic=True, color=HOLE)


def _bowl() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.003), BOWL_R * 0.6, 0.006, WHITE)]
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((BOWL_R - 0.003) * math.cos(a), (BOWL_R - 0.003) * math.sin(a), BOWL_H / 2),
                               (0.006, 2 * math.pi * BOWL_R / n * 1.12, BOWL_H), WHITE, yaw=a))
    shapes.append(CylShape((0.0, 0.0, 0.0065), BOWL_R - 0.004, 0.001, (0.88, 0.88, 0.86), collide=False))
    return Body.build("bowl", pos=(BOWL_AT[0], BOWL_AT[1], TOP), mass=0.30, shapes=shapes, color=WHITE,
                      friction=(0.6, 0.5))


def _dish() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), DISH_R, 0.004, BLUE)]
    n = 22
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((DISH_R - 0.003) * math.cos(a), (DISH_R - 0.003) * math.sin(a), 0.007),
                               (0.006, 2 * math.pi * DISH_R / n * 1.12, 0.010), BLUE, yaw=a))
    return Body.build("saucer", pos=(DISH_AT[0], DISH_AT[1], TOP), mass=0.30, shapes=shapes, color=BLUE,
                      friction=(0.6, 0.5))


PIECES = ("kernel", "shell_a", "shell_b")
OBJECTS = [_lever("lever_left", 1.0), _lever("lever_right", -1.0), _walnut(), _park(), _kernel(),
           _shell("shell_a", 0.0), _shell("shell_b", -0.04), _bowl(), _dish()]
hinge("pivot", "lever_left", "lever_right", anchor=(PX, PY, TOP + 0.01), axis="Z",
      limits_deg=(-2.0 * (OPEN_DEG - SHUT_LIMIT), 0.0), spring=SPRING, damping=0.0005, target_deg=PRELOAD)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cracked": False, "crack_t": None, "tcps": None, "s": {}, "min_half": OPEN_DEG}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("NUT_DEBUG")), "steps": 0}


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
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))


def _yaw(rot) -> float:
    return math.degrees(math.atan2(float(rot[1, 0]), float(rot[0, 0])))


def _crack(base, origin, l_pos, l_rot) -> None:
    """The shell gives: the walnut goes to the store; the kernel and the two shell halves fall out along the V."""
    _place(base, origin, "walnut", (PARK[0] - 0.02, PARK[1] + 0.03, PARK[2] + 0.02))
    # the V's middle line: the pivot and the direction halfway between the two levers
    r_pos, r_rot = _pose(base, origin, "lever_right")
    a_l = _yaw(l_rot) + 180.0 - OPEN_DEG
    a_r = _yaw(r_rot) + 180.0 + OPEN_DEG
    d = _u((a_l + a_r) / 2.0)
    pivot = l_pos
    _place(base, origin, "kernel", pivot + d * KERNEL_S + [0.0, 0.0, 0.002])
    for name, s in zip(("shell_a", "shell_b"), SHELL_S):
        _place(base, origin, name, pivot + d * s + [0.0, 0.0, 0.002])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    l_pos, l_rot = _pose(base, origin, "lever_left")
    r_pos, r_rot = _pose(base, origin, "lever_right")
    rel = (_yaw(r_rot) - _yaw(l_rot) + 180.0) % 360.0 - 180.0  # 0 open; negative as they close
    half = OPEN_DEG + rel / 2.0  # each lever's angle from the V's middle line
    _EP["min_half"] = min(_EP["min_half"], half)
    n_pos, _ = _pose(base, origin, "walnut")
    a_mid = (_yaw(l_rot) + 180.0 - OPEN_DEG + _yaw(r_rot) + 180.0 + OPEN_DEG) / 2.0
    d = _u(a_mid)
    v = n_pos - l_pos
    along, across = float(v @ d), float(abs(v[0] * d[1] - v[1] * d[0]))
    between = 0.035 < along < 0.090 and across < 0.012 and n_pos[2] < TOP + 0.03
    gap = 2.0 * (along * math.sin(math.radians(half)) - JAW_IN)  # between the jaws' faces, where the walnut is
    _EP["min_gap"] = min(_EP.get("min_gap", 1.0), gap) if between else _EP.get("min_gap", 1.0)
    if not _EP["cracked"] and between and gap <= CRACK_GAP:
        _EP["cracked"] = True
        _EP["crack_t"] = _DEBUG["steps"] / 20.0
        _crack(base, origin, l_pos, l_rot)
    k_pos, _ = _pose(base, origin, "kernel")
    b_pos, _ = _pose(base, origin, "bowl")
    s_pos, _ = _pose(base, origin, "saucer")
    in_bowl = (_EP["cracked"] and float(np.linalg.norm(k_pos[:2] - b_pos[:2])) < BOWL_R - 0.008
               and b_pos[2] - 0.005 < k_pos[2] < b_pos[2] + BOWL_H)
    on_dish = 0
    for name in ("shell_a", "shell_b"):
        p, _ = _pose(base, origin, name)
        if (_EP["cracked"] and float(np.linalg.norm(p[:2] - s_pos[:2])) < DISH_R - 0.006
                and s_pos[2] - 0.005 < p[2] < s_pos[2] + 0.03):
            on_dish += 1
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = {"cracked": _EP["cracked"], "in_bowl": in_bowl, "on_dish": on_dish}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[nut] t={_DEBUG['steps'] / 20:5.1f}s half={half:5.1f} min={_EP['min_half']:5.1f} "
                  f"gap={gap * 1000:5.1f} between={between} along={along * 1000:5.1f} across={across * 1000:5.1f} cracked={_EP['cracked']} "
                  f"pivot={np.round(l_pos * 1000).astype(int).tolist()} "
                  f"kernel={np.round(k_pos * 1000).astype(int).tolist()} shells={[np.round(_pose(base, origin, n)[0] * 1000).astype(int).tolist() for n in ('shell_a', 'shell_b')]} in_bowl={in_bowl} on_dish={on_dish} "
                  f"home={_arms_home()} L={np.round(tl * 1000).astype(int).tolist()} "
                  f"R={np.round(tr * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr,
                  flush=True)
    else:
        _DEBUG["steps"] += 1


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"cracked": bool(st.get("cracked")), "in_bowl": bool(st.get("in_bowl")), "on_dish": st.get("on_dish", 0),
            "home": _arms_home()}


def _points(s) -> float:
    if not s["cracked"]:
        return 0.0
    if s["in_bowl"] and s["on_dish"] == 2 and s["home"]:
        return 1.0
    if s["in_bowl"]:
        return 0.5
    return 0.25


def score(objects) -> float:
    """Cracked: at some point the jaws closed to 2.9 cm apart where the walnut sits (it is 3.5 cm across), with the
    walnut between them (3.5-9 cm out from the pivot, within 1.2 cm of the V's middle line, on the worktop); the
    shell then breaks into a kernel and two shell halves. Judged at the end: 1.0 when it cracked, the kernel is in the
    bowl (its middle within 4.2 cm of the bowl's middle, inside it), both shell halves are on the saucer (each within
    5.6 cm of its middle, on it) and both arms are back where they started (each TCP within 5 cm of its start); 0.5
    when it cracked and the kernel is in the bowl; 0.25 when it cracked; else 0.0."""
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
    """Full score with the kernel, the shell halves, the bowl and the saucer still."""
    return (score(objects) >= 1.0 and all(objects[n].resting() for n in PIECES)
            and objects["bowl"].resting() and objects["saucer"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each hand comes down beside the outside of one handle's red grip, fingers closed, and the two push the handles toward
# each other along their swing until the jaws crush the walnut and the levers meet their stop. They draw back and the
# spring opens the levers. The right hand takes the kernel to the bowl and goes back to where it started; then the left
# takes each shell half to the saucer and goes back to where it started too.

TIP = 0.013
FACE = 0.0125  # a closed gripper's side, this far from the TCP across its closing axis
PUSH_Z = TOP + 0.024  # the TCP: the fingertips 1.1 cm above the worktop, on the grip's side
SQUEEZE_TO = SHUT_LIMIT - 3.5  # aim each lever past its stop: the arms stop about 1.7 cm short against the spring
STEPS, STEP_S = 10, 0.45
LEAD_S, LEAD_STEPS = 3.0, 15
PICK_Z = TOP + 0.016
KERNEL_PICK_Z = TOP + 0.018
LIFT = 0.07


def _push_rot(side: float, half: float = 0.0) -> np.ndarray:
    """Top-down, fingers closing along x (along the handle): the gripper's thin side faces the handle."""
    return grip_rotation(-math.pi / 2 if side > 0 else math.pi / 2)


def _push_at(side: float, half: float, gap: float = 0.0) -> np.ndarray:
    """The TCP beside the outside of the middle of a handle's grip (side +1 left, -1 right) with the lever ``half``
    degrees from the V's middle line, ``gap`` m off it."""
    u = _u(180.0 - side * half)
    g = _u(180.0 - side * (half - PARALLEL_DEG))
    out = np.array([g[1], -g[0], 0.0]) * side  # away from the V's middle
    c = np.array([PX, PY, 0.0]) + u * BEND + g * GRIP_LEN / 2 + out * (GRIP_W / 2 + FACE + gap)
    return np.array([c[0], c[1], PUSH_Z])


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


def _squeeze(side: float, arm: str) -> tuple[list[Waypoint], np.ndarray, np.ndarray]:
    name = "left" if side > 0 else "right"
    start = _push_at(side, OPEN_DEG, 0.02)
    out = _lead_in(arm, start + [0.0, 0.0, 0.04], _push_rot(side, OPEN_DEG),
                   f"over the outside of the {name} handle's grip", FINGER_CLOSED)
    out += [_w(1.0, start, FINGER_CLOSED, "down beside it, fingers closed", _push_rot(side, OPEN_DEG)),
            _w(0.6, _push_at(side, OPEN_DEG), FINGER_CLOSED, "against the grip", _push_rot(side, OPEN_DEG))]
    for k in range(1, STEPS + 1):
        h = OPEN_DEG + (SQUEEZE_TO - OPEN_DEG) * k / STEPS
        out.append(_w(STEP_S, _push_at(side, h), FINGER_CLOSED, "push the handles together: the jaws crush the shell",
                      _push_rot(side, h)))
    shut = _push_at(side, SQUEEZE_TO)
    back = _push_at(side, SQUEEZE_TO, 0.06) + [0.0, 0.0, 0.08]
    out += [_w(1.0, shut, FINGER_CLOSED, "hold them shut", _push_rot(side, SQUEEZE_TO)),
            _w(0.8, back, FINGER_CLOSED, "draw back and up: the spring opens the levers", _push_rot(side, SQUEEZE_TO))]
    return out, back, _push_rot(side, SQUEEZE_TO)


def _pick_place(src, dst, rot, open_w, name, dst_name, drop_z) -> list[Waypoint]:
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    return [_w(2.0, src + [0.0, 0.0, LIFT], open_w, f"over the {name}", rot),
            _w(1.2, src, open_w, "down round it", rot),
            _w(0.4, src, open_w, "settle", rot),
            _w(0.6, src, FINGER_CLOSED, f"take the {name}", rot),
            _w(0.9, src + [0.0, 0.0, LIFT], FINGER_CLOSED, "lift it", rot),
            _w(1.6, [dst[0], dst[1], drop_z], FINGER_CLOSED, f"over the {dst_name}", rot),
            _w(0.5, [dst[0], dst[1], drop_z], open_w, "let it drop in", rot),
            _w(0.6, [dst[0], dst[1], drop_z + 0.03], open_w, "rise", rot)]


def _piece_at(s: float, z: float = PICK_Z) -> np.ndarray:
    return np.array([PX - s, PY, z])


def demo():
    left, l_back, l_rot = _squeeze(1.0, "left")
    right, r_back, r_rot = _squeeze(-1.0, "right")
    pick = grip_rotation(0.0)  # top-down, fingers closing across the V (y)
    k_open = -(KERNEL[1] / 2 + 0.008)
    right += _pick_place(_piece_at(KERNEL_S, KERNEL_PICK_Z), BOWL_AT, pick, k_open, "kernel", "bowl", TOP + BOWL_H + 0.04)
    right += _go_home("right", right[-1].pos, pick, FINGER_OPEN)
    right_done = sum(w.duration for w in right)
    s_open = -(0.011 + 0.013)
    t_left = sum(w.duration for w in left)
    left.append(_w(max(right_done - 6.0 - t_left, 0.5), l_back, FINGER_CLOSED,
                   "wait for the right hand to take the kernel away", l_rot))
    left += _pick_place(_piece_at(SHELL_S[0], KERNEL_PICK_Z), DISH_AT, pick, s_open, "first shell half", "saucer", TOP + 0.05)
    left += _pick_place(_piece_at(SHELL_S[1], KERNEL_PICK_Z), DISH_AT + [0.0, -0.01], pick, s_open, "second shell half", "saucer",
                        TOP + 0.05)
    left += _go_home("left", left[-1].pos, pick, FINGER_OPEN)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
