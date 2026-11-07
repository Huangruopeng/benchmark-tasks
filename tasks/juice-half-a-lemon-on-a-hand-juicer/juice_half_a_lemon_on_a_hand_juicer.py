"""On the station table stands a light plastic citrus juicer, a bowl with a ribbed cone in the middle and a long flat
handle out to the left, on a smooth worktop. Half a lemon sits cut side down on the cone. A small saucer waits to the
right.

Task: juice the lemon. One hand holds the juicer still by its handle while the other takes the lemon half and twists
it to and fro on the cone, so its juice runs into the bowl (twisted with nothing holding the juicer, the light juicer
just turns with the lemon on the smooth worktop and no juice comes). Once it is squeezed dry, the lemon half is lifted
off and put on the saucer, the handle is let go, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=juice_half_a_lemon_on_a_hand_juicer.py \\
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
                     on_step, rotation, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the juicer: a bowl 13 cm across, 3.5 cm deep, a cone in the middle, a flat handle out along +y -------------------
JX, JY = 0.30, -0.10
BOWL_R, BOWL_H, FLOOR = 0.065, 0.035, 0.004
CONE = ((0.024, 0.008), (0.017, 0.010), (0.010, 0.010), (0.004, 0.006))  # (radius, height), stacked from the floor
HANDLE = (0.018, 0.115, 0.012)  # across, out, thick
HANDLE_Z = BOWL_H - 0.008  # its middle, above the juicer's base
HANDLE_GRIP = BOWL_R + 0.090  # where the handle is held: this far from the juicer's axis (15.5 cm)
JUICER_MASS = 0.15

# -- the lemon half: cut side down on the cone; origin on the axis, at its cut face --------------------------------------
L_Z = FLOOR + 0.010  # its cut face, above the juicer's base (the cone runs up inside it)
L_R, L_RIM = 0.030, 0.010  # its outer radius, the rind-and-pulp wall around the cone
L_WALL_H = 0.026
HOLD = 0.05  # N m per degree: the lemon grips the cone where it last stopped ...
SLIP_DEG = 2.0  # ... until turned this far past it (0.1 N m; then the task file moves its hold there)
PER_DROP = 25.0  # degrees of twisting on the cone (against the juicer) per drop of juice

# -- the juice: 10 drops, parked out of sight, each let into the bowl as the lemon is twisted --------------------------
N_DROPS, DROP_R = 10, 0.0045
PARK = (0.30, 0.0, TOP + 1.5)

SAUCER_AT, SAUCER_R = np.array([0.22, -0.26]), 0.055

WHITE, YELLOW, PULP, JUICE, BLUE, HOLE = (0.94, 0.94, 0.92), (0.95, 0.82, 0.15), (0.98, 0.92, 0.55), \
    (0.98, 0.90, 0.35), (0.30, 0.50, 0.85), (0.10, 0.10, 0.10)

# -- scoring ----------------------------------------------------------------------------------------------------------
FULL = 8  # drops in the bowl for full score
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _juicer() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR / 2), BOWL_R, FLOOR, WHITE)]
    n = 24
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((BOWL_R - 0.002) * math.cos(a), (BOWL_R - 0.002) * math.sin(a), BOWL_H / 2),
                               (0.004, 2 * math.pi * BOWL_R / n * 1.12, BOWL_H), WHITE, yaw=a))
    z = FLOOR
    for r, h in CONE:
        shapes.append(CylShape((0.0, 0.0, z + h / 2), r, h, WHITE))
        z += h
    for i in range(8):  # the cone's ribs (seen only)
        a = 2 * math.pi * i / 8
        shapes.append(BoxShape((0.014 * math.cos(a), 0.014 * math.sin(a), FLOOR + 0.016), (0.022, 0.002, 0.020),
                               (0.88, 0.88, 0.86), yaw=a, collide=False))
    shapes.append(BoxShape((0.0, BOWL_R - 0.004 + HANDLE[1] / 2, HANDLE_Z), HANDLE, WHITE))  # the handle
    shapes.append(BoxShape((-BOWL_R - 0.006, 0.0, BOWL_H - 0.004), (0.016, 0.024, 0.004), WHITE))  # the spout
    return Body.build("juicer", pos=(JX, JY, TOP), mass=JUICER_MASS, shapes=shapes, friction=(0.12, 0.10), slick=True,
                      color=WHITE, com=(0.0, 0.02, 0.010))


def _lemon() -> Body:
    """The lemon half: a wall of rind and pulp round the cone, and a domed rind on top."""
    shapes = []
    n = 14
    rc = L_R - L_RIM / 2
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape((rc * math.cos(a), rc * math.sin(a), L_WALL_H / 2), (L_RIM, 2 * math.pi * L_R / n * 1.1,
                                                                                    L_WALL_H), YELLOW, yaw=a))
        shapes.append(BoxShape(((L_R - L_RIM) * math.cos(a), (L_R - L_RIM) * math.sin(a), 0.0008),
                               (0.004, 2 * math.pi * (L_R - L_RIM) / n * 1.1, 0.0016), PULP, yaw=a, collide=False))
    z = L_WALL_H
    for r, h in ((L_R - 0.001, 0.008), (0.023, 0.006), (0.013, 0.005), (0.005, 0.004)):
        shapes.append(CylShape((0.0, 0.0, z + h / 2), r, h, YELLOW))
        z += h
    return Body.build("lemon", pos=(JX, JY, TOP + L_Z), mass=0.07, shapes=shapes, friction=(1.0, 0.9), color=YELLOW)


def _park() -> Body:
    w, t = 0.06, 0.004
    shapes = [BoxShape((0.0, 0.0, t / 2), (w, w, t), HOLE), BoxShape((0.0, 0.0, w - t / 2), (w, w, t), HOLE)]
    shapes += [BoxShape((s * (w / 2 - t / 2), 0.0, w / 2), (t, w, w), HOLE) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (w / 2 - t / 2), w / 2), (w, t, w), HOLE) for s in (-1.0, 1.0)]
    return Body.build("juice_store", pos=PARK, mass=1.0, shapes=shapes, kinematic=True, color=HOLE)


def _drops() -> list[Body]:
    out = []
    for i in range(N_DROPS):
        dx, dy = (i % 4 - 1.5) * 0.011, (i // 4 - 1.0) * 0.012
        out.append(Body.build(f"drop_{i:02d}", pos=(PARK[0] + dx, PARK[1] + dy, PARK[2] + 0.004 + DROP_R), mass=0.001,
                              shapes=[SphereShape((0.0, 0.0, 0.0), DROP_R, JUICE)], friction=(0.3, 0.25),
                              damping=(0.3, 0.5), color=JUICE))
    return out


def _saucer() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), SAUCER_R, 0.004, BLUE)]
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((SAUCER_R - 0.003) * math.cos(a), (SAUCER_R - 0.003) * math.sin(a), 0.006),
                               (0.006, 2 * math.pi * SAUCER_R / n * 1.12, 0.008), BLUE, yaw=a))
    return Body.build("saucer", pos=(SAUCER_AT[0], SAUCER_AT[1], TOP), mass=0.3, kinematic=True, shapes=shapes,
                      color=BLUE)


DROPS = _drops()
DROP_NAMES = [d.name for d in DROPS]
OBJECTS = [_juicer(), _lemon(), _park(), *DROPS, _saucer()]
hinge("on_the_cone", "juicer", "lemon", anchor=(JX, JY, TOP + L_Z), axis="Z", spring=HOLD, damping=0.002,
      target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": 0.0, "twisted": 0.0, "dropped": 0, "free": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("LEMON_DEBUG")), "steps": 0}


def _joint():
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    return UsdPhysics.Joint(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS["on_the_cone"]))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    j = _joint()
    j.CreateCollisionEnabledAttr().Set(True)  # the lemon sits round the cone, touching it, like any two things
    j.CreateJointEnabledAttr().Set(True)


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


def _let_drop(base, origin, j_pos, j_rot) -> None:
    """Teleport the next parked drop into the bowl, beside the cone's foot."""
    import torch

    k = _EP["dropped"]
    asset = base.scene[DROP_NAMES[k]]
    a = 2 * math.pi * k / N_DROPS
    p = j_pos + j_rot @ np.array([0.038 * math.cos(a), 0.038 * math.sin(a), FLOOR + DROP_R + 0.004])
    pose = torch.tensor([[*(p + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim(pose)
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))
    _EP["dropped"] += 1


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    j_pos, j_rot = _pose(base, origin, "juicer")
    l_pos, l_rot = _pose(base, origin, "lemon")
    rel = j_rot.T @ l_rot
    turn = math.degrees(math.atan2(float(rel[1, 0]), float(rel[0, 0])))  # the lemon turned on the cone
    if not _EP["free"]:
        moved = (turn - _EP["hold"] + 180.0) % 360.0 - 180.0
        if abs(moved) > SLIP_DEG:  # turned past its hold: it gives (squeezing it), and stays
            _EP["twisted"] += abs(moved)
            _EP["hold"] = turn
            set_hinge_drive("on_the_cone", target_deg=turn)
        while _EP["dropped"] < N_DROPS and _EP["twisted"] >= PER_DROP * (_EP["dropped"] + 1):
            _let_drop(base, origin, j_pos, j_rot)
        if _EP["dropped"] >= N_DROPS:  # squeezed dry: it comes off the cone
            _EP["free"] = True
            _joint().GetJointEnabledAttr().Set(False)
    drops = [_pose(base, origin, n)[0] for n in DROP_NAMES]
    in_bowl = 0
    for d in drops:
        v = j_rot.T @ (d - j_pos)
        if math.hypot(float(v[0]), float(v[1])) < BOWL_R - 0.004 and -0.002 < float(v[2]) < BOWL_H:
            in_bowl += 1
    upright = float(j_rot[2, 2]) > math.cos(math.radians(15.0))
    on_saucer = (float(np.linalg.norm(l_pos[:2] - SAUCER_AT)) < SAUCER_R - 0.012 and l_pos[2] < TOP + 0.02)
    _EP["tcps"] = tcps = _tcps(base, origin)
    clear = all(float(np.linalg.norm(t - (l_pos + l_rot @ np.array([0.0, 0.0, 0.035])))) >= LET_GO for t in tcps)
    _EP["s"] = {"in_bowl": in_bowl if upright else 0, "on_saucer": on_saucer and _EP["free"], "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            jyaw = math.degrees(math.atan2(float(j_rot[1, 0]), float(j_rot[0, 0])))
            print(f"[lemon] t={_DEBUG['steps'] / 20:5.1f}s turn={turn:6.1f} twisted={_EP['twisted']:6.1f} "
                  f"dropped={_EP['dropped']} in_bowl={in_bowl} free={_EP['free']} juicer_yaw={jyaw:6.1f} "
                  f"juicer={np.round(j_pos * 1000).astype(int).tolist()} lemon={np.round(l_pos * 1000).astype(int).tolist()}"
                  f" on_saucer={on_saucer} clear={clear} home={_arms_home()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"in_bowl": st.get("in_bowl", 0), "on_saucer": bool(st.get("on_saucer")), "clear": bool(st.get("clear")),
            "home": _arms_home()}


def _points(s) -> float:
    if s["in_bowl"] >= FULL and s["on_saucer"] and s["clear"] and s["home"]:
        return 1.0
    if s["in_bowl"] >= FULL:
        return 0.5
    return 0.25 if s["in_bowl"] >= 3 else 0.0


def score(objects) -> float:
    """Judged at the end. Juice: drops in the juicer's bowl (inside its wall, below its rim), counted only with the
    juicer upright within 15 degrees. 1.0 with at least 8 drops in the bowl, the lemon half off the cone and on the
    saucer (its middle within 4.3 cm of the saucer's middle, down on it), each gripper's TCP at least 4 cm from the
    lemon, and both arms back where they started (each TCP within 5 cm of its start); 0.5 with at least 8 drops in the
    bowl; 0.25 with at least 3; else 0.0."""
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
    """Full score with the juicer, the lemon and the juice still."""
    return (score(objects) >= 1.0 and objects["juicer"].resting() and objects["lemon"].resting()
            and all(objects[n].resting() for n in DROP_NAMES))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the end of the juicer's handle, fingers closing across it, and holds it. The
# right hand comes down round the lemon half, fingers closing across it, and twists it to and fro on the cone, 90
# degrees each way, until it is squeezed dry; then it lifts it off and puts it on the saucer. The left lets go of the
# handle, and both arms go back to where they started.

HOLD_GRIP = grip_rotation(-math.pi / 2)  # left: straight down, fingers closing along x, across the handle
LEMON_OPEN = -(L_R + 0.006)
LEMON_IN = 0.010  # the TCP this far below the lemon's top
STROKES = 3
TWIST = math.radians(45.0)  # each way from the middle
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


LEMON_TOP = L_WALL_H + 0.008 + 0.006 + 0.005 + 0.004  # above its cut face


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    grip = np.array([JX, JY + HANDLE_GRIP, TOP + HANDLE_Z + HANDLE[2] / 2 - 0.004])
    pre = -(HANDLE[0] / 2 + 0.006)
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], HOLD_GRIP, "over the end of the juicer's handle", pre)
    out += [_w(1.0, grip, pre, "down round it", HOLD_GRIP),
            _w(0.6, grip, FINGER_CLOSED, "hold the juicer by its handle", HOLD_GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), grip, FINGER_CLOSED, "hold it still while the lemon is twisted",
               HOLD_GRIP),
            _w(0.5, grip, pre, "let go", HOLD_GRIP),
            _w(0.8, grip + [0.0, 0.0, 0.05], pre, "rise", HOLD_GRIP)]
    out += _go_home("left", grip + [0.0, 0.0, 0.05], HOLD_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    grip_z = TOP + L_Z + LEMON_TOP - LEMON_IN
    at = np.array([JX, JY, grip_z])

    def g(a):
        return grip_rotation(math.pi / 2 + a)

    out = _lead_in("right", at + [0.0, 0.0, 0.05], g(0.0), "over the lemon half", LEMON_OPEN)
    out += [_w(1.0, at, LEMON_OPEN, "down round it", g(0.0)),
            _w(0.6, at, FINGER_CLOSED, "take it", g(0.0))]
    t = sum(w.duration for w in out)
    if start > t:
        out.append(_w(start - t, at, FINGER_CLOSED, "wait for the juicer to be held", g(0.0)))
    out.append(_w(0.6, at, FINGER_CLOSED, "twist it on the cone", g(TWIST)))
    for k in range(STROKES):
        out += [_w(1.1, at, FINGER_CLOSED, "twist it back: juice runs into the bowl", g(-TWIST)),
                _w(1.1, at, FINGER_CLOSED, "and forth", g(TWIST))]
    out.append(_w(0.6, at, FINGER_CLOSED, "back to the middle", g(0.0)))
    off_at = sum(w.duration for w in out)
    put = np.array([SAUCER_AT[0], SAUCER_AT[1], TOP + 0.004 + LEMON_TOP - LEMON_IN + 0.003])
    out += [_w(1.0, at + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift the squeezed lemon off the cone", g(0.0)),
            _w(1.5, put + [0.0, 0.0, 0.05], FINGER_CLOSED, "over the saucer", g(0.0)),
            _w(0.8, put, FINGER_CLOSED, "set it on the saucer", g(0.0)),
            _w(0.4, put, LEMON_OPEN, "let go", g(0.0)),
            _w(0.6, put + [0.0, 0.0, 0.05], LEMON_OPEN, "rise", g(0.0))]
    out += _go_home("right", put + [0.0, 0.0, 0.05], g(0.0), FINGER_OPEN)
    return out, off_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, off_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(off_at + 1.0, total)
    return {"left": left, "right": right}
