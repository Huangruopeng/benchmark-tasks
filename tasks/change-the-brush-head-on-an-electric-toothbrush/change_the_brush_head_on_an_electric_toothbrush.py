"""On a dark granite bathroom counter lies an electric toothbrush on its back, its worn brush head still on its metal
shaft. A new brush head lies in its open blister pack to the right, and a small pedal bin stands near the front right.

Task: change the brush head. Hold the toothbrush's handle with one hand; with the other pull the worn head straight off
the shaft and drop it in the bin, then take the new head from its pack, push it straight onto the shaft until it is all
the way on, and let go. Then take both arms back to where they started.

Why two hands: the head sits on its shaft with a press fit that takes about 3 N to pull off or push on, and the
toothbrush lying on the counter slides at about 0.6 N. Pulled or pushed with nothing holding the handle, the whole
toothbrush slides along the counter with the head.

Run it:  inspect-robots run --embodiment yam_arena -E task=change_the_brush_head_on_an_electric_toothbrush.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy,  # noqa: E402
                     on_reset, on_step, plate, rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the counter ------------------------------------------------------------------------------------------------------
CX0, CX1, CY0, CY1, CT_H = 0.170, 0.480, -0.230, 0.200, 0.010
CT = TOP + CT_H
# -- the toothbrush, lying on its back along x, its shaft forward (+x) -------------------------------------------------
B_LEN, B_W, B_H = 0.170, 0.026, 0.026
AXIS_Z = CT + B_H / 2  # the toothbrush's axis (and its shaft's)
BX, BY = 0.280, -0.005  # the handle's middle
SHAFT_BASE = np.array([BX + B_LEN / 2, BY, AXIS_Z])  # where a head's base sits when it is all the way on
SHAFT_L, SHAFT_R = 0.024, 0.0022
# -- the brush heads: origin at the base of the head (on the shaft's base when on), neck along +x ----------------------
NECK_R, NECK_L = 0.0055, 0.052
COLLAR_R, COLLAR_L = 0.0080, 0.012
HEAD_R = 0.0095
NEW0 = np.array([0.255, -0.140, AXIS_Z])  # the new head's base as it lies in its pack (neck along +x)
# -- the bin ----------------------------------------------------------------------------------------------------------
BINX, BINY, BIN_R, BIN_H = 0.405, -0.175, 0.045, 0.085  # clear of the finger housings at the shaft
# -- the rules -------------------------------------------------------------------------------------------------------
K_FIT, SLIP, D_FIT = 3000.0, 0.001, 3.0  # the press fit: about 3 N to slide a head along the shaft
OFF = SHAFT_L + 0.004  # a head pulled this far out is off the shaft
CAPTURE_R, CAPTURE_DEG = 0.010, 15.0
ON_DEPTH = 0.003  # a head within this of the shaft's base is all the way on

QUARTZ, QUARTZ_V = (0.20, 0.21, 0.23), (0.36, 0.37, 0.39)  # dark granite: the white toothbrush shows
WHITE_PL, BLUE, BLUE_D, GREY = (0.96, 0.96, 0.96), (0.15, 0.45, 0.85), (0.08, 0.25, 0.55), (0.62, 0.64, 0.67)
STEEL = (0.78, 0.80, 0.82)
BRISTLE_NEW, BRISTLE_TIP, BRISTLE_OLD = (0.97, 0.97, 0.97), (0.20, 0.55, 0.92), (0.86, 0.82, 0.66)
CARD, BLISTER = (0.30, 0.55, 0.85), (0.86, 0.90, 0.94)
BIN_C, BIN_D = (0.95, 0.95, 0.94), (0.55, 0.56, 0.58)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _counter() -> Body:
    shapes = [BoxShape((0.0, 0.0, CT_H / 2), (CX1 - CX0, CY1 - CY0, CT_H), QUARTZ)]
    rng = np.random.default_rng(5)
    for _ in range(14):  # paler veins in the granite
        x0, y0 = rng.uniform(-0.14, 0.14), rng.uniform(-0.2, 0.2)
        a = rng.uniform(0, math.pi)
        shapes.append(BoxShape((x0, y0, CT_H + 0.0002), (rng.uniform(0.04, 0.10), 0.0015, 0.0004), QUARTZ_V,
                               collide=False, yaw=a))
    return Body.build("counter", pos=((CX0 + CX1) / 2, (CY0 + CY1) / 2, TOP), mass=1.0, kinematic=True, color=QUARTZ,
                      shapes=shapes, friction=(0.5, 0.4))


def _brush() -> Body:
    """The handle: a white body with a flat back (it lies still), a blue band and power button, and the steel shaft."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (B_LEN - 0.010, B_W - 0.006, B_H), WHITE_PL),
              BoxShape((0.0, 0.0, 0.0), (B_LEN - 0.010, B_W, B_H - 0.006), WHITE_PL)]
    for s in (-1.0, 1.0):  # rounded long edges
        for t in (-1.0, 1.0):
            shapes.append(CylShape((0.0, s * (B_W / 2 - 0.003), t * (B_H / 2 - 0.003)), 0.003, B_LEN - 0.010,
                                   WHITE_PL, axis="X"))
    shapes.append(CylShape((-B_LEN / 2 + 0.005, 0.0, 0.0), B_W / 2 - 0.001, 0.010, GREY, axis="X"))  # the base cap
    shapes.append(CylShape((B_LEN / 2 - 0.004, 0.0, 0.0), 0.0095, 0.008, WHITE_PL, axis="X"))  # the tapered top
    shapes.append(CylShape((B_LEN / 2 - 0.016, 0.0, 0.0), B_W / 2 + 0.0004, 0.006, BLUE, axis="X", collide=False))
    shapes.append(BoxShape((0.030, 0.0, B_H / 2 + 0.0006), (0.016, 0.010, 0.0016), BLUE, collide=False))  # button
    shapes.append(CylShape((-0.002, 0.0, B_H / 2 + 0.0004), 0.0018, 0.0008, (0.25, 0.85, 0.35), collide=False))
    shapes.append(CylShape((B_LEN / 2 + SHAFT_L / 2, 0.0, 0.0), SHAFT_R, SHAFT_L, STEEL, axis="X", collide=False))
    return Body.build("brush", pos=(BX, BY, AXIS_Z), mass=0.14, shapes=shapes, color=WHITE_PL, friction=(0.5, 0.4))


def _head(name: str, pos, worn: bool) -> Body:
    """A round oscillating brush head: a collar over the shaft, a neck, and a round head with bristles up."""
    shapes = [CylShape((COLLAR_L / 2, 0.0, 0.0), COLLAR_R, COLLAR_L, BLUE if not worn else BLUE_D, axis="X"),
              CylShape((COLLAR_L + NECK_L / 2, 0.0, 0.0), NECK_R, NECK_L, WHITE_PL, axis="X")]
    hx = COLLAR_L + NECK_L + HEAD_R - 0.002
    shapes.append(CylShape((hx, 0.0, 0.0), HEAD_R, 0.008, WHITE_PL))  # the round head, its bristles up
    bristle = BRISTLE_OLD if worn else BRISTLE_NEW
    shapes.append(CylShape((hx, 0.0, 0.0085), HEAD_R - 0.0015, 0.009, bristle))
    if worn:  # splayed, worn tufts
        for k in range(8):
            a = k * math.pi / 4
            shapes.append(SphereShape((hx + 0.008 * math.cos(a), 0.008 * math.sin(a), 0.010), 0.0028, bristle,
                                      collide=False))
    else:
        shapes.append(CylShape((hx, 0.0, 0.0133), HEAD_R - 0.0030, 0.0008, BRISTLE_TIP, collide=False))
        for k in range(8):
            a = k * math.pi / 4
            shapes.append(CylShape((hx + 0.0068 * math.cos(a), 0.0068 * math.sin(a), 0.0132), 0.0012, 0.001,
                                   BRISTLE_TIP, collide=False))
    ring = BLUE if not worn else (0.5, 0.5, 0.5)
    shapes.append(CylShape((COLLAR_L + 0.001, 0.0, 0.0), COLLAR_R + 0.0003, 0.002, ring, axis="X", collide=False))
    return Body.build(name, pos=tuple(np.asarray(pos, float).tolist()), mass=0.010, shapes=shapes, color=WHITE_PL,
                      friction=(1.2, 1.1), ignore=("brush",), damping=(0.0, 0.1))


def _pack() -> Body:
    """The new head's open blister pack: a blue card with a clear tray that holds the head up level with the shaft."""
    cx, cy = NEW0[0] + 0.035, NEW0[1]
    shapes = [BoxShape((0.0, 0.0, 0.0015), (0.100, 0.050, 0.003), CARD)]
    hx = COLLAR_L + NECK_L + HEAD_R - 0.002
    for x, top in ((COLLAR_L / 2, AXIS_Z - COLLAR_R), (hx, AXIS_Z - 0.004)):  # clear cradles under the collar and head
        x -= cx - NEW0[0]
        top -= CT
        shapes.append(BoxShape((x, 0.0, (0.003 + top) / 2), (0.010, 0.020, top - 0.003), BLISTER))
    shapes.append(BoxShape((0.0, 0.0, 0.0032), (0.090, 0.036, 0.0004), (0.95, 0.95, 0.95), collide=False))
    return Body.build("pack", pos=(cx, cy, CT), mass=0.2, kinematic=True, color=CARD, shapes=shapes,
                      friction=(0.8, 0.7))


def _bin() -> Body:
    shapes = []
    n = 14
    for k in range(n):
        a = 2 * math.pi * k / n
        nrm = np.array([math.cos(a), math.sin(a), 0.0])
        m = (BIN_R - 0.0015) * nrm
        w = 2 * BIN_R * math.tan(math.pi / n) + 0.0004
        shapes.append(plate(m, m + [0.0, 0.0, BIN_H], w, 0.003, nrm, BIN_C))
    shapes.append(CylShape((0.0, 0.0, 0.002), BIN_R - 0.002, 0.004, BIN_D))
    shapes.append(CylShape((0.0, 0.0, BIN_H - 0.002), BIN_R + 0.002, 0.004, BIN_D, collide=False))
    return Body.build("bin", pos=(BINX, BINY, CT), mass=0.5, kinematic=True, color=BIN_C, shapes=shapes)


SHAFT_LOCAL = (B_LEN / 2, 0.0, 0.0)  # the shaft's base in the handle's frame
thread("old_on", "brush", "old_head", anchor0=SHAFT_LOCAL, axis0=(1.0, 0.0, 0.0),
       anchor1=(0.0, 0.0, 0.0), axis1=(1.0, 0.0, 0.0), slide_limits=(-0.001, 0.10), turn_damping=0.5,
       slide_stiffness=K_FIT, slide_damping=D_FIT, slide_max_force=40.0, always_on=True)
thread("new_on", "brush", "new_head", anchor0=SHAFT_LOCAL, axis0=(1.0, 0.0, 0.0),
       anchor1=(0.0, 0.0, 0.0), axis1=(1.0, 0.0, 0.0), slide_limits=(-0.001, 0.10), turn_damping=0.5,
       slide_stiffness=K_FIT, slide_damping=D_FIT, slide_max_force=40.0, play=0.001, tilt_deg=3.0)

_author_threads_plain = bodykit._author_threads


def _author_threads(stage, prim_path: str, spec) -> None:
    """bodykit's threads, then: the worn head's press fit (on from the start) holds it where it sits, not 10 cm out."""
    _author_threads_plain(stage, prim_path, spec)
    if spec.name == "old_head":
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/old_on")
        UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr().Set(0.0)


bodykit._author_threads = _author_threads

OBJECTS = [_counter(), _pack(), _bin(), _brush(), _head("old_head", SHAFT_BASE, True), _head("new_head", NEW0, False)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"old_on": True, "hold_old": 0.0, "new_on": False, "hold_new": 0.0, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BRUSH_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_slide_target("old_on", 0.0)
    set_thread_on("old_on", True)
    set_thread_on("new_on", False)


def _tcps(base, origin):
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


def _on_shaft(pb, rb, ph, rh):
    """(distance from the shaft's axis, how far out from its base along it, angle to it in degrees) for a head."""
    base_w = pb + rb @ (SHAFT_BASE - np.array([BX, BY, AXIS_Z]))
    q = rb.T @ (ph - base_w)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[:, 0] @ rh[:, 0])))))
    return math.hypot(q[1], q[2]), float(q[0]), angle


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "brush")
    po, ro = _pose(base, origin, "old_head")
    pn, rn = _pose(base, origin, "new_head")
    off_o, d_o, a_o = _on_shaft(pb, rb, po, ro)
    off_n, d_n, a_n = _on_shaft(pb, rb, pn, rn)
    if _EP["old_on"]:
        if d_o > OFF:  # pulled off the shaft's end
            set_thread_on("old_on", False)
            _EP["old_on"] = False
        elif abs(d_o - _EP["hold_old"]) > SLIP:  # pulled (or pushed) along it: it slides, and stays where left
            _EP["hold_old"] = d_o
            set_slide_target("old_on", d_o)
    if not _EP["new_on"]:
        if (not _EP["old_on"] and off_n <= CAPTURE_R and 0.0 <= d_n <= SHAFT_L + 0.002
                and a_n <= CAPTURE_DEG):  # its collar over the shaft's end
            set_slide_target("new_on", d_n)
            set_thread_on("new_on", True)
            _EP["new_on"], _EP["hold_new"] = True, d_n
    else:
        if d_n > OFF:
            set_thread_on("new_on", False)
            _EP["new_on"] = False
        elif abs(d_n - _EP["hold_new"]) > SLIP:
            _EP["hold_new"] = d_n
            set_slide_target("new_on", d_n)
    old_in_bin = math.hypot(po[0] - BINX, po[1] - BINY) <= BIN_R - 0.005 and CT < po[2] < CT + BIN_H
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"old_off": not _EP["old_on"] and d_o > OFF, "old_in_bin": old_in_bin,
                "new_seated": _EP["new_on"] and d_n <= ON_DEPTH}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, CT]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            print(f"[brush] t={_DEBUG['steps'] / 20:5.1f}s fingers={[round(v, 1) for v in fj]} brush={mm(pb)}"
                  f" old={mm(po)} d_old={d_o * 1000:.1f} on={_EP['old_on']} new={mm(pn)} off_new={off_n * 1000:.1f}"
                  f" d_new={d_n * 1000:.1f} a_new={a_n:.1f} on_new={_EP['new_on']} bin={old_in_bin}"
                  f" L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the new head all the way on the shaft, the worn head in the bin and both arms back where they started;
    0.5: the new head all the way on the shaft; 0.25: the worn head off the shaft; else 0.0. All judged at the end."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["new_seated"]:
        return 1.0 if s["old_in_bin"] and _arms_home() else 0.5
    return 0.25 if s["old_off"] else 0.0


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
    """Full score with the toothbrush and the new head at rest: the trial ends there."""
    return score(objects) >= 1.0 and objects["brush"].resting() and objects["new_head"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the toothbrush's handle near its base and holds it. The right takes the worn head by its neck,
# pulls it straight off the shaft and drops it in the bin; it takes the new head from its pack by the neck, lines it up
# in front of the shaft and pushes it straight on, all the way. Both let go and go back where they started.

LEFT_GRIP_X = BX - 0.030  # the left holds the handle here, near its middle (a shove moves it, not turns it)
HANDLE_FINGER = -0.020  # pre-open only a little wider than the handle
NECK_FINGER = -0.012
HANDLE_DEPTH = 0.004
NECK_DEPTH = 0.004  # the neck's axis this far past the TCP: held deep in the jaw (the tips just clear the counter)
NECK_GRIP = COLLAR_L + 0.026  # where the right takes a head's neck, from its base
OVER = 0.060
PULL_PAST = 0.020  # the arm lags under the press fit's 3 N: aim this far past
LEAD_S, LEAD_STEPS = 3.0, 15


PITCH = math.radians(25.0)  # the right's grip pitched forward (its wrist further back) to reach the shaft's end
LEFT_PITCH = math.radians(35.0)  # the left's more, so its wrist stays well clear of the right's


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot(arm: str) -> np.ndarray:
    """Grips closing along y, pitched forward (fingers ahead of the wrist)."""
    return _ry(-(LEFT_PITCH if arm == "left" else PITCH)) @ grip_rotation(0.0)


def _tcp_for(point, depth: float, arm: str = "right") -> np.ndarray:
    return np.asarray(point, float) - depth * _rot(arm)[:, 2]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
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


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = np.array([0.0, 0.0, OVER])

    def go(arm, t, pos, finger, label, n=1):
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label,
                                      _rot(arm)))
        cur[arm] = (p1, finger)

    def wait(arm, t, label="wait"):
        p, f = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, _rot(arm)))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def neck(head_base):
        return _tcp_for(np.asarray(head_base, float) + [NECK_GRIP, 0.0, 0.0], NECK_DEPTH)

    hold = _tcp_for((LEFT_GRIP_X, BY, AXIS_Z), HANDLE_DEPTH, "left")
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", hold + up, _rot("left")),
                                "over the toothbrush's handle", HANDLE_FINGER)
    cur["left"] = (hold + up, HANDLE_FINGER)
    go("left", 1.2, hold, HANDLE_FINGER, "down round the handle")
    go("left", 0.6, hold, FINGER_CLOSED, "hold the handle")
    old_grip = neck(SHAFT_BASE)
    plan["right"] += _joint_path("right", np.array(READY, float), _good_ik("right", old_grip + up, _rot("right")),
                                 "over the worn head", NECK_FINGER)
    cur["right"] = (old_grip + up, NECK_FINGER)
    wait("right", dur("left") - dur("right"))
    go("right", 1.2, old_grip, NECK_FINGER, "down round its neck")
    go("right", 0.6, old_grip, FINGER_CLOSED, "take the worn head")
    go("right", 1.6, neck(SHAFT_BASE + [OFF + PULL_PAST, 0.0, 0.0]), FINGER_CLOSED, "pull it straight off the shaft",
       n=4)
    go("right", 0.8, neck(SHAFT_BASE + [OFF + PULL_PAST, 0.0, 0.040]), FINGER_CLOSED, "lift it clear")
    over_bin = np.array([BINX - NECK_GRIP, BINY, CT + BIN_H + 0.040])
    go("right", 1.6, neck(over_bin), FINGER_CLOSED, "carry it over the bin", n=3)
    go("right", 0.5, neck(over_bin), NECK_FINGER, "drop it in")
    new_grip = neck(NEW0)
    go("right", 1.6, new_grip + up, NECK_FINGER, "over the new head in its pack", n=3)
    go("right", 1.0, new_grip, NECK_FINGER, "down round its neck")
    go("right", 0.6, new_grip, FINGER_CLOSED, "take the new head")
    go("right", 0.8, new_grip + [0.0, 0.0, 0.030], FINGER_CLOSED, "lift it out of the pack")
    front = SHAFT_BASE + [SHAFT_L + 0.012, 0.0, 0.0]
    go("right", 1.8, neck(front) + [0.0, 0.0, 0.030], FINGER_CLOSED, "carry it in front of the shaft", n=4)
    go("right", 0.8, neck(front), FINGER_CLOSED, "line it up with the shaft")
    go("right", 2.0, neck(SHAFT_BASE - [PULL_PAST, 0.0, 0.0]), FINGER_CLOSED, "push it straight on, all the way",
       n=5)
    wait("right", 0.4)
    go("right", 0.5, neck(SHAFT_BASE - [PULL_PAST, 0.0, 0.0]), NECK_FINGER, "let go")
    go("right", 0.8, neck(SHAFT_BASE) + up, NECK_FINGER, "rise clear")
    wait("left", dur("right") - dur("left"), "hold the handle")
    go("left", 0.5, hold, HANDLE_FINGER, "let go of the handle")
    go("left", 0.8, hold + up, HANDLE_FINGER, "rise clear")
    wait("right", 1.3)
    for arm in ("left", "right"):
        p, f = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, _rot(arm)), np.array(READY, float), "back to where it started",
                                 f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
