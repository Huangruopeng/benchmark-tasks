"""On the station table stands a server's power supply, its back face toward the arms, with a power inlet in the middle
of the face. A thick, stiff power cord's plug sits in the inlet but not home: the cord's stiffness has pushed it 1.5 cm
back out, and it springs back out whenever it is let go. Beside the inlet, a retention latch lies folded flat against
the face: a short arm hinged at the inlet's side, with a hook at its end and a round knob on top. The power light above
the inlet is red.

Task: power the supply and secure the cord. One hand pushes the plug fully home by its cord and keeps it there (the
light turns green); the other swings the latch round by its knob until its hook closes behind the plug, where it
clicks shut. A plug that is not home stands in the hook's way. Then both hands let go, the latch holding the plug home,
and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=plug_in_a_server_power_cord_and_latch_its_retention_clip.py \\
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
                     rotation, segment, set_hinge_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the power supply: its back face (with the inlet) toward the arms, at x = FACE ------------------------------------
FACE = 0.42
PSU_X, PSU_Y, PSU_H = (FACE, FACE + 0.15), (-0.07, 0.11), 0.07
ZC = TOP + 0.035  # the inlet's middle height
BEZEL = (0.010, 0.044, 0.032)  # the inlet's black bezel, standing proud of the face (x), wide (y), tall (z)
MOUTH = FACE - BEZEL[0]  # the bezel's front: where the plug's front face sits when home
LAMP = np.array([FACE - 0.0015, -0.045, ZC + 0.022])  # the power light, on the face
HIDDEN = np.array([FACE + 0.07, -0.045, TOP - 0.05])  # the green light's place while the supply is off (under the table)

# -- the plug on its cord: on a slider along x; its spring (the stiff cord) pushes it back out -------------------------
OUT = 0.015  # where it starts (and springs back to): this far short of home
PLUG = (0.040, 0.026, 0.020)  # its body: long (x), wide, tall
BOOT = (0.024, 0.014, 0.008)
CORD_W = 0.010
CORD_END = 0.210  # the stiff cord's straight end, this far behind the plug's front face
GRIP_BACK = 0.180  # where the cord is held: this far behind the plug's front face
K_PLUG, PLUG_DRAG = 250.0, 3.0  # N/m (3.75 N at home), N s/m
HOME_TOL = 0.005  # the plug counts as home within this of fully in

# -- the latch: an arm hinged about z on a bracket beside the bezel; it starts swung open (OPEN_AT from along +y) and
# shuts at 90 degrees, its arm along -x beside the plug and its hook across behind the plug's back -------------------
PIVOT = np.array([FACE - 0.025, BEZEL[1] / 2 + 0.004, ZC])
ARM_L = 0.031
ARM_Z = 0.0065  # the arm's middle height above ZC: the hook passes over the boot, across the plug's back
OPEN_AT = 25.0  # degrees: where the latch starts (and is left open)
HOOK = 0.034  # the hook, from the arm's end across behind the plug
KNOB_R, KNOB_H = 0.0065, 0.030
LATCH_HOLD = 0.004  # N m per degree: the latch stays where it is left ...
LATCH_SLIP = 2.0  # ... once pushed this many degrees past its hold (the task file moves the hold there)
LATCH_DRAG = 0.0004
SHUT_DEG = 86.0  # at or past this with the plug home, the latch clicks shut ...
CLICK = 0.06  # ... and is held there (N m per degree, toward 90)
OPEN_DEG = 70.0  # pulled back under this, it is open again

DARK, BLACK, GREY, SILVER, RED, GREEN, ORANGE = (0.20, 0.21, 0.23), (0.05, 0.05, 0.06), (0.45, 0.46, 0.48), \
    (0.72, 0.73, 0.75), (0.85, 0.08, 0.06), (0.10, 0.85, 0.20), (0.95, 0.55, 0.10)

LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _psu() -> Body:
    o = np.array([(PSU_X[0] + PSU_X[1]) / 2, (PSU_Y[0] + PSU_Y[1]) / 2, TOP])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    shapes = [BoxShape(at(o[0], o[1], TOP + PSU_H / 2), (PSU_X[1] - PSU_X[0], PSU_Y[1] - PSU_Y[0], PSU_H), DARK),
              BoxShape(at(FACE - BEZEL[0] / 2, 0.0, ZC), BEZEL, BLACK),
              BoxShape(at(MOUTH - 0.0003, 0.0, ZC), (0.0006, 0.028, 0.020), GREY, collide=False),  # the inlet's mouth
              BoxShape(at(FACE - 0.001, LAMP[1], LAMP[2]), (0.002, 0.012, 0.012), RED, collide=False),  # the power light
              BoxShape(at(FACE - 0.0005, -0.045, ZC - 0.012), (0.001, 0.034, 0.010), SILVER, collide=False),  # a label
              BoxShape(at((PIVOT[0] + FACE) / 2, PIVOT[1] + 0.002, ZC - 0.003), (FACE - PIVOT[0] + 0.006, 0.012, 0.006),
                       GREY)]  # the latch's bracket, under its hinge
    for k in range(5):  # a fan grille on the face, left of the inlet
        shapes.append(BoxShape(at(FACE - 0.0005, 0.050 + 0.0, TOP + 0.012 + k * 0.011), (0.001, 0.05, 0.003), BLACK,
                               collide=False))
    return Body.build("psu", pos=tuple(o.tolist()), mass=3.0, shapes=shapes, kinematic=True, color=DARK)


def _plug() -> Body:
    """The plug and its stiff cord; origin at the plug's front face, on the inlet's axis."""
    shapes = [BoxShape((-PLUG[0] / 2, 0.0, 0.0), PLUG, BLACK),
              BoxShape((-PLUG[0] - BOOT[0] / 2, 0.0, 0.0), BOOT, BLACK),
              BoxShape((-(PLUG[0] + BOOT[0] + CORD_END) / 2, 0.0, 0.0),
                       (CORD_END - PLUG[0] - BOOT[0], CORD_W, CORD_W), BLACK),
              BoxShape((-PLUG[0] / 2 - 0.004, 0.0, PLUG[2] / 2 + 0.0002), (0.016, 0.014, 0.0004), GREY,
                       collide=False)]  # a grip patch
    for k in range(4):  # ridges on its sides
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((-0.012 - k * 0.006, s * (PLUG[1] / 2 + 0.0003), 0.0), (0.002, 0.0006, 0.014), GREY,
                                   collide=False))
    drop = ZC - TOP - CORD_W / 2  # the rest of the cord falls to the table and runs off toward the arms (looks only)
    shapes += [segment((-CORD_END, 0.0, 0.0), (-CORD_END - 0.03, 0.0, -drop), CORD_W, BLACK, collide=False),
               segment((-CORD_END - 0.03, 0.0, -drop), (-CORD_END - 0.03, -0.12, -drop), CORD_W, BLACK, collide=False)]
    return Body.build("plug", pos=(MOUTH - OUT, 0.0, ZC), mass=0.12, shapes=shapes, friction=(1.0, 0.9), color=BLACK,
                      com=(-0.06, 0.0, 0.0))


def _latch() -> Body:
    """The latch, open (its arm along +y); origin on its hinge."""
    shapes = [BoxShape((0.0, ARM_L / 2, ARM_Z), (0.006, ARM_L, 0.008), SILVER),  # the arm
              BoxShape((-HOOK / 2 + 0.003, ARM_L, ARM_Z), (HOOK, 0.006, 0.003), SILVER),  # the hook (toward -x, open)
              CylShape((0.0, ARM_L, ARM_Z + 0.004 + KNOB_H / 2), KNOB_R, KNOB_H, ORANGE),  # the knob
              CylShape((0.0, 0.0, ARM_Z), 0.004, 0.012, GREY)]  # the hinge pin
    a = math.radians(OPEN_AT) / 2
    return Body.build("latch", pos=tuple(PIVOT.tolist()), mass=0.01, shapes=shapes, friction=(1.0, 0.9),
                      color=SILVER, start_xyzw=(0.0, 0.0, math.sin(a), math.cos(a)))


def _green() -> Body:
    return Body.build("power_light", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=GREEN,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.003, 0.0125, 0.0125), GREEN, collide=False)])


OBJECTS = [_psu(), _plug(), _latch(), _green()]
slider("plug_slide", "psu", "plug", anchor=(MOUTH - OUT, 0.0, ZC), axis="X", limits=(-0.004, OUT),
       stiffness=K_PLUG, damping=PLUG_DRAG, target=0.0)
hinge("latch_pivot", "psu", "latch", anchor=tuple(PIVOT.tolist()), axis="Z", limits_deg=(0.0, 91.0),
      spring=LATCH_HOLD, damping=LATCH_DRAG, target_deg=OPEN_AT)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": OPEN_AT, "shut": False, "lit": None, "ever_home": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PLUG_DEBUG")), "steps": 0}


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


def _place(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["plug"].data.root_link_pose_w)[0][:3] - origin
    q = as_numpy(base.scene["latch"].data.root_link_pose_w)[0]
    r = rotation(q[3:7])
    angle = math.degrees(math.atan2(r[1, 0], r[0, 0]))  # the latch's turn from open (+: toward shut)
    short = MOUTH - float(p[0])  # how far the plug is from fully home
    home = short <= HOME_TOL
    _EP["ever_home"] = _EP["ever_home"] or home
    if not _EP["shut"]:
        if angle >= SHUT_DEG and home:  # swung shut behind a plug that is home: it clicks
            _EP["shut"] = True
            set_hinge_drive("latch_pivot", target_deg=90.0, spring=CLICK)
        elif abs(angle - _EP["hold"]) > LATCH_SLIP:  # pushed past its hold: it turns, and stays
            _EP["hold"] = angle
            set_hinge_drive("latch_pivot", target_deg=angle)
    elif angle < OPEN_DEG:  # pulled open again
        _EP["shut"] = False
        _EP["hold"] = angle
        set_hinge_drive("latch_pivot", target_deg=angle, spring=LATCH_HOLD)
    if _EP["lit"] != home:  # the power light: green while the plug is home
        _place(base, origin, "power_light", LAMP if home else HIDDEN)
        _EP["lit"] = home
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    grip = p + [-GRIP_BACK, 0.0, 0.0]
    knob = PIVOT + [-ARM_L * math.sin(math.radians(angle)), ARM_L * math.cos(math.radians(angle)), 0.0]
    clear = all(float(np.linalg.norm(t - c)) >= LET_GO for t in tcps for c in (p, grip, knob))
    _EP["s"] = {"home": home, "ever_home": _EP["ever_home"], "shut": _EP["shut"], "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[plug] t={_DEBUG['steps'] / 20:5.1f}s short={short * 1000:5.1f}mm home={home} "
                  f"latch={angle:6.1f}deg shut={_EP['shut']} clear={clear} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} knob={np.round(knob * 1000).astype(int).tolist()} "
                  f"R={np.round(tr * 1000).astype(int).tolist()} grip={np.round(grip * 1000).astype(int).tolist()} "
                  f"arms_home={_arms_home()} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the plug home (within 5 mm of fully in) and the latch shut behind it, both hands let go (each TCP at
    least 4 cm from the plug, the cord where it is held in the demo and the latch's knob) and both arms back where
    they started; 0.5 with the plug home and the latch shut; 0.25 with the plug pushed home at some point; else 0.0."""
    s = _EP["s"]
    if s.get("home") and s.get("shut"):
        return 1.0 if s["clear"] and _arms_home() else 0.5
    return 0.25 if s.get("ever_home") else 0.0


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
    """Full score with the plug and the latch still: the trial ends there."""
    return score(objects) >= 1.0 and objects["plug"].resting() and objects["latch"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the cord 18 cm behind the plug, fingers closing across it, and pushes the plug
# home (the power light turns green) and keeps it there. The left comes straight down behind the latch's knob, fingers
# closed, and pushes the knob round its hinge until the hook is shut behind the plug (it clicks). The left rises; the right gives the cord a tug (the latch holds the plug in), lets
# go, and both arms go back to where they started.

CORD_GRIP = grip_rotation(0.0)  # right, straight down, fingers closing along y (across the cord)
CORD_OPEN = -0.011
PUSH_PAST = 0.025  # aim this far past home (the arm gives under the spring)
TUG = 0.010
KNOB_IN = 0.010  # the left TCP this far below the knob's top
BEHIND = 0.011  # the left TCP this far behind the knob's middle (beyond its radius), pushing it round
PUSH_GRIP = grip_rotation(0.0)  # left, straight down, fingers closed (their thin side pushes the knob)
SWING_STEPS = 10
LEAD_S, LEAD_STEPS = 3.0, 15


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
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


def _knob_at(angle_deg: float) -> np.ndarray:
    a = math.radians(angle_deg)
    return PIVOT + [-ARM_L * math.sin(a), ARM_L * math.cos(a), ARM_Z + 0.004 + KNOB_H - KNOB_IN]


def _pusher(angle_deg: float) -> np.ndarray:
    """Where the left TCP pushes the knob round from, with the latch at ``angle_deg``: behind the knob's way."""
    a = math.radians(angle_deg)
    return _knob_at(angle_deg) + (KNOB_R + BEHIND) * np.array([math.cos(a), math.sin(a), 0.0])


def _right(pushed_by: float, release_at: float, total: float) -> tuple[list[Waypoint], float]:
    start = np.array([MOUTH - OUT - GRIP_BACK, 0.0, ZC])
    home = start + [OUT + PUSH_PAST, 0.0, 0.0]
    out = _lead_in("right", start + [0.0, 0.0, 0.05], CORD_GRIP, "over the cord", CORD_OPEN)
    out += [_w(1.0, start, CORD_OPEN, "down round the cord", CORD_GRIP),
            _w(0.5, start, FINGER_CLOSED, "take it", CORD_GRIP),
            _w(1.2, home, FINGER_CLOSED, "push the plug home: the light turns green", CORD_GRIP)]
    held_at = sum(w.duration for w in out)
    tug = home - [TUG + PUSH_PAST, 0.0, 0.0]
    up = tug + [0.0, 0.0, 0.06]
    out += [_w(max(release_at - held_at, 0.3), home, FINGER_CLOSED, "keep it home while the latch is swung shut",
               CORD_GRIP),
            _w(0.8, tug, FINGER_CLOSED, "tug the cord: the latch holds the plug in", CORD_GRIP),
            _w(0.4, tug, CORD_OPEN, "let go", CORD_GRIP),
            _w(0.8, up, CORD_OPEN, "rise", CORD_GRIP)]
    out += _go_home("right", up, CORD_GRIP, CORD_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, CORD_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _left(start: float) -> tuple[list[Waypoint], float]:
    p0 = _pusher(OPEN_AT)
    out = _lead_in("left", p0 + [0.0, 0.0, 0.03], PUSH_GRIP, "over the latch, behind its knob", FINGER_CLOSED)
    out += [_w(max(start - LEAD_S, 0.3), p0 + [0.0, 0.0, 0.03], FINGER_CLOSED, "wait for the plug to be home",
               PUSH_GRIP),
            _w(1.0, p0, FINGER_CLOSED, "down behind the knob", PUSH_GRIP)]
    for k in range(1, SWING_STEPS + 1):
        a = OPEN_AT + (94.0 - OPEN_AT) * k / SWING_STEPS
        out.append(_w(0.35, _pusher(a), FINGER_CLOSED, "push the latch round behind the plug", PUSH_GRIP))
    shut = _pusher(94.0)
    out += [_w(0.6, shut, FINGER_CLOSED, "press it shut: it clicks", PUSH_GRIP),
            _w(0.8, shut + [0.0, 0.0, 0.03], FINGER_CLOSED, "rise", PUSH_GRIP)]
    done = sum(w.duration for w in out)
    out += _go_home("left", shut + [0.0, 0.0, 0.03], PUSH_GRIP, FINGER_CLOSED)
    return out, done


def demo():
    _, held_at = _right(0.0, 0.0, 0.0)
    left, done = _left(held_at)
    right, _ = _right(0.0, done + 0.3, 0.0)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    right, _ = _right(0.0, done + 0.3, total)
    left.append(_w(max(total - sum(w.duration for w in left), 0.5), left[-1].pos, FINGER_CLOSED, "wait at the start",
                   left[-1].yaw))
    return {"left": left, "right": right}
