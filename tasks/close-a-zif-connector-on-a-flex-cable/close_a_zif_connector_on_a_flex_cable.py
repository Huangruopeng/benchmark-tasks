"""On the station table lies a circuit board with a ZIF (zero insertion force) connector on its right part: a socket
with a flip latch standing open behind it. A long flat flex cable lies on the board, its end at the socket's mouth.

Task: connect the cable. One hand slides the cable straight into the socket, fully in, and holds it there: the socket's
contacts push it back out as soon as it is let go. The other flips the latch down shut over it, which locks it in.
Then both hands let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=close_a_zif_connector_on_a_flex_cable.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the circuit board (fixed): slick, so the cable slides on it --------------------------------------------------------
PCB_AT, PCB = (0.30, -0.02), (0.10, 0.30, 0.002)
PT = TOP + PCB[2]  # its top

# -- the socket (fixed): two side guides and a back stop; the cable goes in toward -y -----------------------------------
SX = PCB_AT[0]
YM = -0.095  # its mouth
DEPTH = 0.006  # the back stop's face, this far in
YS = YM - DEPTH
CW, CL, CT = 0.024, 0.18, 0.0015  # the cable: wide (x), long (y), thick
GUIDE_H, GUIDE_T, PLAY = 0.0025, 0.003, 0.003
STOP_H, STOP_T = 0.004, 0.004

# -- the latch: a flap hinged along x above the back stop; shut, it lies flat over the cable's end; it starts open,
# leaning just past upright against its stop. Tipped over, it falls; within SNAP_DEG of shut, it snaps shut
FLAP = (0.028, 0.018, 0.0015)  # wide (x), long (from the hinge, shut: toward +y), thick
HINGE_AT = np.array([SX, YS - 0.002, PT + 0.0045])
OPEN_DEG = 100.0
SHUT_K = 0.003  # N m per degree, holding it shut once snapped
SMALL = 0.0005  # the contact offset of these small parts (PhysX's default 2 mm would stop the cable short)
SNAP_DEG = 12.0

# -- the cable's springback: in the socket and not locked, its contacts push it back out ---------------------------------
PUSH0, PUSH_K = 0.05, 40.0  # N, N/m (of depth): 0.29 N fully in
CABLE_MASS = 0.006
END0 = YM + 0.001  # the cable's end at the start: 1 mm outside the mouth
ANCHOR_AT = np.array([SX, 0.0, TOP - 0.05])  # the lock's hidden fixed block, under the table

PCB_C, SOCKET_C, FLAP_C, CABLE_C, BLUE, METAL = (0.10, 0.42, 0.20), (0.92, 0.92, 0.88), (0.15, 0.15, 0.16), \
    (0.88, 0.80, 0.55), (0.15, 0.30, 0.75), (0.80, 0.78, 0.70)

# -- scoring ----------------------------------------------------------------------------------------------------------
FULLY_IN = 0.0045  # the cable's end at least this deep in the socket: fully in
STRAIGHT_DEG = 5.0
LET_GO = 0.03
SUCCESS_HOLD_S = 1.0


def _pcb() -> Body:
    return Body.build("pcb", pos=(*PCB_AT, TOP), mass=1.0, kinematic=True, friction=(0.15, 0.15), slick=True,
                      color=PCB_C, contact_offset=SMALL, shapes=[BoxShape((0.0, 0.0, PCB[2] / 2), PCB, PCB_C)])


def _socket() -> Body:
    """Origin at the middle of the mouth, on the board's top."""
    gx = CW / 2 + PLAY + GUIDE_T / 2
    shapes = [BoxShape((s * gx, -DEPTH / 2, GUIDE_H / 2), (GUIDE_T, DEPTH, GUIDE_H), SOCKET_C) for s in (-1.0, 1.0)]
    shapes.append(BoxShape((0.0, -DEPTH - STOP_T / 2, STOP_H / 2), (CW + 2 * (PLAY + GUIDE_T), STOP_T, STOP_H),
                           SOCKET_C))
    shapes.append(BoxShape((0.0, -DEPTH / 2, 0.0002), (CW, DEPTH, 0.0004), METAL, collide=False))  # its contacts
    return Body.build("socket", pos=(SX, YM, PT), mass=0.1, kinematic=True, friction=(0.3, 0.3), color=SOCKET_C,
                      contact_offset=SMALL, shapes=shapes)


def _flap() -> Body:
    """Origin on the hinge line; shapes as when shut (lying toward +y); it starts open."""
    a = math.radians(OPEN_DEG) / 2
    shapes = [BoxShape((0.0, FLAP[1] / 2, 0.0), FLAP, FLAP_C)]
    return Body.build("latch", pos=tuple(HINGE_AT.tolist()), mass=0.004, shapes=shapes, friction=(0.8, 0.7),
                      color=FLAP_C, start_xyzw=(math.sin(a), 0.0, 0.0, math.cos(a)), contact_offset=SMALL)


def _cable() -> Body:
    """Origin at the middle of its end (the one that goes in), on the board's top."""
    shapes = [BoxShape((0.0, CL / 2, CT / 2), (CW, CL, CT), CABLE_C),
              BoxShape((0.0, 0.012, CT + 0.0002), (CW, 0.016, 0.0004), BLUE, collide=False),  # the stiffener
              BoxShape((0.0, 0.003, -0.0001), (CW - 0.002, 0.006, 0.0002), METAL, collide=False)]  # the contacts
    return Body.build("cable", pos=(SX, END0, PT), mass=CABLE_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=CABLE_C, com=(0.0, CL / 2, CT / 2), contact_offset=SMALL, ignore=("latch",),
                      damping=(20.0, 2.0))  # air and board drag: it pops out a few cm, no further


def _anchor() -> Body:
    """A small fixed block hidden under the table: the lock fixes the cable to it (a joint to the socket itself would
    stop the two colliding)."""
    return Body.build("anchor", pos=tuple(ANCHOR_AT.tolist()), mass=0.1, kinematic=True, color=FLAP_C,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.005, 0.005, 0.005), FLAP_C, collide=False)])


OBJECTS = [_anchor(), _pcb(), _socket(), _flap(), _cable()]
hinge("latch_pin", "socket", "latch", anchor=tuple(HINGE_AT.tolist()), axis="X", limits_deg=(0.0, OPEN_DEG + 4.0),
      damping=2e-6)
CABLE_AT0 = np.array([SX, END0, PT])
glue("lock", "anchor", "cable", anchor=tuple(CABLE_AT0.tolist()))


# -- followed after every control step: the springback, the latch, the lock ----------------------------------------------

def _fresh() -> dict:
    return {"latched": False, "locked": False, "ever_in": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ZIF_DEBUG")), "steps": 0}


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


def _lock_frames(pos0, rot0) -> None:
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS["lock"]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(np.asarray(rot0, float))))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _lock_frames(CABLE_AT0 - ANCHOR_AT, np.eye(3))
    set_glue_on("lock", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _set_force(asset, f_world, rot, at) -> None:
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    pos = torch.tensor(np.asarray(at, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), positions=pos,
                                                           is_global=False)


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = min(max(float((p - a) @ ab / (ab @ ab)), 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def _state(c_pos, c_rot, l_rot, tcps) -> dict:
    depth = float(YM - c_pos[1])
    yaw = math.degrees(math.atan2(float(c_rot[0, 1]), float(c_rot[1, 1])))
    angle = math.degrees(math.atan2(float(l_rot[2, 1]), float(l_rot[1, 1])))  # the latch's turn up from shut
    far = c_pos + c_rot @ np.array([0.0, CL, 0.0])
    clear = all(_seg_dist(p, c_pos, far) >= LET_GO and float(np.linalg.norm(p - HINGE_AT)) >= LET_GO for p in tcps)
    return {"depth": depth, "yaw": yaw, "angle": angle, "in": depth >= FULLY_IN and abs(yaw) <= STRAIGHT_DEG,
            "clear": clear}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c_pos, c_rot = _pose(base, origin, "cable")
    _, l_rot = _pose(base, origin, "latch")
    _EP["tcps"] = _tcps(base, origin)
    st = _state(c_pos, c_rot, l_rot, _EP["tcps"])
    _EP["s"] = st
    _EP["ever_in"] = _EP["ever_in"] or st["in"]
    if not _EP["latched"] and st["angle"] < SNAP_DEG:  # pushed down far enough: it snaps shut
        _EP["latched"] = True
        set_hinge_drive("latch_pin", target_deg=0.0, spring=SHUT_K)
        if st["in"]:
            _lock_frames(c_pos - ANCHOR_AT, c_rot)
            set_glue_on("lock", True)
            _EP["locked"] = True
    push = 0.0 if _EP["locked"] or st["depth"] <= 0.0 else PUSH0 + PUSH_K * st["depth"]
    _set_force(base.scene["cable"], [0.0, push, 0.0], c_rot, at=(0.0, 0.0, CT / 2))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[zif] t={_DEBUG['steps'] / 20:5.1f}s depth={st['depth'] * 1000:5.1f}mm yaw={st['yaw']:5.1f} "
                  f"cable={np.round(c_pos * 1000).astype(int).tolist()} latch={st['angle']:6.1f} in={st['in']} "
                  f"ever_in={_EP['ever_in']} latched={_EP['latched']} locked={_EP['locked']} clear={st['clear']} "
                  f"push={push:4.2f}N leftTCP-cable_end={np.round((tl - c_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-hinge={np.round((tr - HINGE_AT) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"locked": _EP["locked"], "in": bool(st.get("in")), "clear": bool(st.get("clear")),
            "latched": _EP["latched"], "ever_in": _EP["ever_in"]}


def _points(s) -> float:
    if s["locked"] and s["in"] and s["clear"]:
        return 1.0
    if s["locked"]:
        return 0.5
    return 0.25 if s["latched"] or s["ever_in"] else 0.0


def score(objects) -> float:
    """1.0 with the cable locked in the socket (the latch snapped shut while the cable was fully in and straight) and
    both hands clear of the cable and the latch; 0.5 locked but a hand still on them; 0.25 with the latch shut without
    the cable fully in, or the cable pushed fully in at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable near its far end, fingers closed, presses it onto the board and slides
# it straight into the socket, fully in, and holds it there. The right hand comes down behind the open latch, fingers
# closed, and tips it over forward: it falls shut and snaps. Then both hands rise.

GRIP = grip_rotation(0.0)  # straight down, fingers closed (closing along y: a narrow hand along the cable)
PRESS_Y = 0.155  # the left presses the cable this far from its end
PRESS = 0.008  # aimed this far into the cable (the arm gives: it presses firmly)
PUSH_IN = 0.014  # and slid this far toward the socket (it stops at the back stop; the arm comes short)
TIP = 0.013  # the closed fingertips this far below the TCP
SWEEP_R = 0.014  # the right's fingertips tip the latch over at this radius from the hinge, on its back
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    press = np.array([SX, END0 + PRESS_Y, PT + CT + TIP - PRESS])
    pushed = press - [0.0, PUSH_IN + DEPTH, 0.0]
    out = _lead_in("left", press + [0.0, 0.0, 0.04], GRIP, "over the cable near its far end", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP),
            _w(0.6, press, FINGER_CLOSED, "press it onto the board", GRIP),
            _w(1.5, pushed, FINGER_CLOSED, "slide it straight into the socket, fully in", GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), pushed, FINGER_CLOSED, "hold it in while the latch is shut", GRIP),
            _w(1.0, pushed + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), pushed + [0.0, 0.0, 0.05], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", GRIP))
    return out, held_at


def _on_latch(deg: float, off: float = 0.0) -> np.ndarray:
    """The right TCP with its fingertips on the latch's back at SWEEP_R from the hinge, the latch turned ``deg`` up
    from shut (``off``: this far off its back face, behind it)."""
    a = math.radians(deg)
    along = np.array([0.0, math.cos(a), math.sin(a)])
    back = np.array([0.0, -math.sin(a), math.cos(a)])  # the latch's back face's outward normal
    return HINGE_AT + SWEEP_R * along + (FLAP[2] / 2 + 0.002 + off) * back + [0.0, 0.0, TIP]


def _right(start: float) -> list[Waypoint]:
    begin = _on_latch(OPEN_DEG + 5.0, 0.006)
    out = _lead_in("right", begin + [0.0, 0.0, 0.03], GRIP, "over and behind the open latch", FINGER_CLOSED)
    out += [_w(max(start - LEAD_S, 0.3) + 1.0, begin, FINGER_CLOSED, "down behind it", GRIP),
            _w(0.8, _on_latch(85.0, -0.001), FINGER_CLOSED, "tip it over forward: it falls", GRIP),
            _w(0.6, _on_latch(65.0, -0.001), FINGER_CLOSED, "tip it over forward: it falls", GRIP),
            _w(0.4, _on_latch(50.0, 0.003), FINGER_CLOSED, "tip it over forward: it falls", GRIP)]
    out += [_w(1.0, _on_latch(50.0, 0.003) + [0.0, -0.01, 0.05], FINGER_CLOSED, "rise: it falls shut and snaps",
               GRIP)]
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 5.0
    right.append(_w(5.0, right[-1].pos, FINGER_CLOSED, "hold still", GRIP))
    left, _ = _left(t_r + 0.5, total + 1.0)
    return {"left": left, "right": right}
