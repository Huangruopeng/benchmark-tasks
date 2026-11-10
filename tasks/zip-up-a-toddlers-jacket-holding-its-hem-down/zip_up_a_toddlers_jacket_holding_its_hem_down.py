"""A toddler's red fleece jacket lies open on the station table, front up, its collar away from the arms and its hem
toward them. It closes with a separating zip, like most jackets: the zip's slider waits at the bottom of the left front,
on the box, and the pin at the bottom of the right front's zip tape sticks out just below the hem, its moulded black
tab standing up.

Task: zip the jacket up. Hold the jacket still by its hem (the drawcord toggle there) with one hand; with the other push the pin up into the box, then
take the slider's pull and pull the zip all the way up to the collar. Then let go and take both arms back to where they
started. The jacket is light and slides on the table: pushing the pin home and pulling the zip drag it along unless
the other hand holds it down, and the zip won't run until the pin is in the box.

Run it:  inspect-robots run --embodiment yam_arena -E task=zip_up_a_toddlers_jacket_holding_its_hem_down.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the jacket: one loose body lying flat, front up; x runs from the hem (near) to the collar (far) ---------------------
HEM_X, NECK_X, HALF_W = 0.105, 0.325, 0.150
JT = 0.016  # lying flat (two layers of fleece)
JX = (HEM_X + NECK_X) / 2
J0 = np.array([JX, 0.0, TOP])  # the jacket's frame: on the table under the middle of its body
J_MASS = 0.25
J_FRICTION = (0.5, 0.45)
# -- the zip, along the centre line ------------------------------------------------------------------------------------
GAP = 0.0045  # the open zip: its two tapes' inner edges this far either side of the centre line
TAPE_W = 0.006
BOX_X0, BOX_X1 = HEM_X + 0.004, HEM_X + 0.022  # the box at the bottom of the left tape
SL_LEN, SL_W, SL_H = 0.014, 0.024, 0.006  # the slider
SL_X = BOX_X1 + SL_LEN / 2  # the slider's middle at the start, resting on the box
TRACK = 0.167  # how far the slider can go: its top end against the top stop under the collar
DONE = 0.152  # pulled this far (within 1.5 cm of the top stop): zipped all the way up
PULL_H, PULL_T, PULL_L = 0.022, 0.006, 0.012  # the slider's pull: a stiff moulded tab standing up
# -- the pin at the bottom of the right tape: it slides up (+x) into place beside the box -------------------------------
PIN_LEN = 0.020
PIN_X1 = BOX_X0 - 0.016  # its top end at the start (16 mm short of the box's bottom, below the hem)
PIN_TRAVEL = 0.022
PIN_IN = 0.017  # pushed this far: in the box
TAB_L, TAB_T, TAB_H = 0.012, 0.008, 0.020  # the pin's moulded tab, standing up at its bottom end
# -- resistance (a stick-slip hold: the part stays where it is left; pushed past its hold it gives) --------------------
PIN_K, PIN_GIVE = 1700.0, 0.0015  # about 2.5 N to push the pin
ZIP_K, ZIP_GIVE = 1750.0, 0.0015  # about 2.6 N to pull the zip
JAM_K = 2.0e4  # the slider before the pin is in: jammed
TOG_X, TOG_Y, TOG_R, TOG_H = HEM_X + 0.014, HALF_W + 0.025, 0.008, 0.024  # the hem's drawcord toggle (a cord lock)
N_SEG = 9  # the closed zip, drawn in short lengths behind the slider
SEG_L = TRACK / N_SEG
PARK = (0.40, 0.0, 0.30)  # the closed lengths wait here, under the table

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

RED, RED_DARK, RED_SEAM = (0.80, 0.12, 0.13), (0.62, 0.08, 0.10), (0.70, 0.10, 0.11)
NAVY, LINING, SILVER, BLACK = (0.10, 0.12, 0.25), (0.55, 0.56, 0.58), (0.78, 0.79, 0.81), (0.06, 0.06, 0.07)
LABEL = (0.95, 0.95, 0.93)


def _jacket() -> Body:
    """The jacket, lying flat: its body, collar, hem band, sleeves with cuffs, pockets, the two zip tapes with their
    teeth, the lining showing between them, and the box at the bottom of the left tape (in the jacket's frame)."""
    x0, x1 = HEM_X - JX, NECK_X - JX
    shapes = [BoxShape(((x0 + x1) / 2, 0.0, JT / 2), (x1 - x0, 2 * HALF_W, JT), RED),
              BoxShape((x1 + 0.013, 0.0, (JT + 0.004) / 2), (0.026, 0.13, JT + 0.004), RED_DARK),  # the collar
              BoxShape((x0 + 0.010, 0.0, JT / 2 + 0.0005), (0.020, 2 * HALF_W, JT + 0.001), RED_DARK)]  # hem band
    for s in (-1.0, 1.0):
        # the sleeves, lying out from the shoulders toward the hem, and their cuffs
        a = math.radians(-35.0) * s  # each sleeve's direction off the +y / -y axis, toward the hem
        d = np.array([-math.sin(abs(a)), s * math.cos(a), 0.0])
        sh = np.array([x1 - 0.035, s * (HALF_W - 0.025), 0.0])
        mid = sh + d * 0.085
        yaw = math.atan2(d[1], d[0])
        shapes.append(BoxShape((float(mid[0]), float(mid[1]), JT / 2), (0.17, 0.075, JT * 0.9), RED, yaw=yaw))
        cuff = sh + d * 0.162
        shapes.append(BoxShape((float(cuff[0]), float(cuff[1]), JT / 2 + 0.0005), (0.018, 0.068, JT * 0.9 + 0.001),
                               RED_DARK, yaw=yaw, collide=False))
        # side seams and pockets (drawn only)
        shapes.append(BoxShape(((x0 + x1) / 2, s * (HALF_W - 0.002), JT + 0.0002), (x1 - x0, 0.004, 0.0004), RED_SEAM,
                               collide=False))
        shapes.append(BoxShape((x0 + 0.060, s * 0.085, JT + 0.0003), (0.050, 0.004, 0.0006), RED_SEAM, collide=False))
        shapes.append(BoxShape((x0 + 0.060, s * 0.085, JT + 0.0004), (0.045, 0.0015, 0.0008), BLACK, collide=False))
        # the zip tapes and their teeth
        yt = s * (GAP + TAPE_W / 2)
        shapes.append(BoxShape(((x0 + x1) / 2 + 0.004, yt, JT + 0.0006), (x1 - x0 - 0.008, TAPE_W, 0.0012), NAVY,
                               collide=False))
        n = int((x1 - x0 - 0.03) / 0.007)
        for k in range(n):
            x = x0 + 0.022 + 0.007 * k + (0.0035 if s > 0 else 0.0)
            shapes.append(BoxShape((x, s * (GAP + 0.001), JT + 0.0013), (0.0035, 0.004, 0.0014), SILVER,
                                   collide=False))
    shapes.append(BoxShape(((x0 + x1) / 2 + 0.004, 0.0, JT + 0.0003), (x1 - x0 - 0.008, 2 * GAP, 0.0006), LINING,
                           collide=False))
    shapes.append(BoxShape((x1 - 0.012, 0.0, JT + 0.0012), (0.010, 0.020, 0.0016), BLACK, collide=False))  # top stop
    shapes.append(BoxShape((x1 - 0.004, 0.0, JT + 0.0010), (0.012, 0.030, 0.0012), LABEL, collide=False))
    # the hem's drawcord: it comes out of the hem band at the left side seam, through a cord lock standing on the table
    shapes.append(BoxShape((TOG_X - JX, HALF_W + 0.010, 0.004), (0.004, 0.024, 0.003), BLACK, collide=False))
    shapes.append(CylShape((TOG_X - JX, TOG_Y, TOG_H / 2), TOG_R, TOG_H, BLACK))
    shapes.append(CylShape((TOG_X - JX, TOG_Y, TOG_H + 0.002), 0.004, 0.004, (0.25, 0.25, 0.27), collide=False))
    # the box, at the bottom of the left tape (the slider rests on it)
    shapes.append(BoxShape(((BOX_X0 + BOX_X1) / 2 - JX, 0.006, JT + 0.003), (BOX_X1 - BOX_X0, 0.012, 0.006), SILVER))
    return Body.build("jacket", pos=tuple(J0.tolist()), mass=J_MASS, shapes=shapes, color=RED, friction=J_FRICTION,
                      damping=(0.5, 0.5))


def _zip() -> Body:
    """The zip's slider with its stiff black pull standing up (its frame at the slider's middle, on the tapes)."""
    shapes = [BoxShape((0.0, 0.0, SL_H / 2), (SL_LEN, SL_W, SL_H), SILVER),
              BoxShape((0.0, 0.0, SL_H + 0.002), (0.008, 0.008, 0.004), SILVER),
              BoxShape((0.0, 0.0, SL_H + 0.004 + PULL_H / 2), (PULL_L, PULL_T, PULL_H), BLACK)]
    return Body.build("zip", pos=(SL_X, 0.0, TOP + JT), mass=0.010, shapes=shapes, color=SILVER, friction=(1.3, 1.2))


def _pin() -> Body:
    """The pin at the bottom of the right tape, its moulded black tab standing up at its bottom end (frame at the pin's
    top end, on the tape)."""
    shapes = [BoxShape((-PIN_LEN / 2, 0.0, 0.0015), (PIN_LEN, 0.004, 0.003), SILVER),
              BoxShape((-PIN_LEN / 2 - 0.004, 0.0, 0.0006), (PIN_LEN + 0.008, TAPE_W, 0.0012), NAVY, collide=False),
              BoxShape((-PIN_LEN - TAB_L / 2 + 0.004, 0.0, TAB_H / 2), (TAB_L, TAB_T, TAB_H), BLACK)]
    return Body.build("pin", pos=(PIN_X1, -0.0045, TOP + JT), mass=0.006, shapes=shapes, color=SILVER,
                      friction=(1.3, 1.2), ignore=("zip",))


def _seg(k: int) -> Body:
    """A length of closed zip (the two tapes' teeth locked together), drawn only, in the jacket's frame."""
    x = SL_X - SL_LEN / 2 + SEG_L * (k + 0.5) - JX
    w = 2 * (GAP + TAPE_W) + 0.002  # the closed zip covers both tapes: a dark band with a thin chain of locked teeth
    shapes = [BoxShape((x, 0.0, JT + 0.0016), (SEG_L + 0.001, w, 0.0016), NAVY, collide=False),
              BoxShape((x, 0.0, JT + 0.0026), (SEG_L + 0.001, 0.0025, 0.0006), SILVER, collide=False)]
    return Body.build(f"zc{k}", pos=PARK, mass=0.01, kinematic=True, color=NAVY, shapes=shapes)


slider("zip_track", "jacket", "zip", anchor=(SL_X, 0.0, TOP + JT), axis="X", limits=(0.0, TRACK), stiffness=JAM_K,
       damping=2.0, target=0.0, max_force=50.0)
slider("pin_track", "jacket", "pin", anchor=(PIN_X1, -0.0045, TOP + JT), axis="X", limits=(0.0, PIN_TRAVEL),
       stiffness=PIN_K, damping=1.0, target=0.0, max_force=50.0)

OBJECTS = [_jacket(), _zip(), _pin()] + [_seg(k) for k in range(N_SEG)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"pin_in": False, "pin_hold": 0.0, "zip_hold": 0.0, "max_zip": 0.0, "shown": 0, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ZIP_DEBUG")), "steps": 0}


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


def _write_pose(base, name, pose) -> None:
    import torch

    asset = base.scene[name]
    asset.write_root_pose_to_sim(torch.tensor([list(map(float, pose))], dtype=torch.float32, device=asset.device))


def _hold(s: float, hold: float, give: float) -> float:
    """A stick-slip hold: the hold moves only when the part is pushed more than ``give`` past it."""
    if s - hold > give:
        return s - give
    if hold - s > give:
        return s + give
    return hold


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    jraw = as_numpy(base.scene["jacket"].data.root_link_pose_w)[0]
    jp, jr = jraw[:3] - origin, rotation(jraw[3:7])
    zp = as_numpy(base.scene["zip"].data.root_link_pose_w)[0][:3] - origin
    pp = as_numpy(base.scene["pin"].data.root_link_pose_w)[0][:3] - origin
    s_zip = float(jr[:, 0] @ (zp - jp)) - (SL_X - JX)  # how far the slider has been pulled up the jacket
    s_pin = float(jr[:, 0] @ (pp - jp)) - (PIN_X1 - JX)  # how far the pin has been pushed up
    if not _EP["pin_in"]:
        _EP["pin_hold"] = _hold(s_pin, _EP["pin_hold"], PIN_GIVE)
        set_slider_drive("pin_track", target=_EP["pin_hold"])
        if s_pin >= PIN_IN:
            _EP["pin_in"] = True  # the pin is in the box: it stays there, and the zip can run
            set_slider_drive("pin_track", target=PIN_TRAVEL, stiffness=JAM_K)
            _EP["zip_hold"] = s_zip
            set_slider_drive("zip_track", target=s_zip, stiffness=ZIP_K)
    else:
        _EP["zip_hold"] = _hold(s_zip, _EP["zip_hold"], ZIP_GIVE)
        set_slider_drive("zip_track", target=_EP["zip_hold"])
    _EP["max_zip"] = max(_EP["max_zip"], s_zip if _EP["pin_in"] else 0.0)
    closed = min(N_SEG, int(max(0.0, s_zip) / SEG_L)) if _EP["pin_in"] else 0
    for k in range(N_SEG):
        if k < closed:
            _write_pose(base, f"zc{k}", jraw)
        elif k < _EP["shown"]:
            _write_pose(base, f"zc{k}", [*(np.asarray(PARK) + origin).tolist(), 0.0, 0.0, 0.0, 1.0])
    _EP["shown"] = closed
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"zip": s_zip if _EP["pin_in"] else 0.0, "pin": s_pin}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            print(f"[zip] t={_DEBUG['steps'] / 20:5.1f}s jacket={np.round((jp - J0) * 1000).astype(int).tolist()}"
                  f" yaw={math.degrees(math.atan2(jr[1, 0], jr[0, 0])):5.1f} pin={s_pin * 1000:5.1f}mm in={_EP['pin_in']}"
                  f" zip={s_zip * 1000:6.1f}mm ({s_zip / DONE * 100:4.0f}%) closed={closed}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the pin in the box, the zip pulled all the way up to the collar and both arms back where they started,
    at the end; 0.5 with the pin in and the zip pulled at least half way up; 0.25 with the pin in the box; else 0.0.
    (A pin in the box stays in.)"""
    if not _EP["pin_in"]:
        return 0.0
    z = _EP["s"].get("zip", 0.0)
    if z >= DONE and _arms_home():
        return 1.0
    if z >= DONE / 2:
        return 0.5
    return 0.25


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
    """Full score with the jacket and the slider still: the trial ends there."""
    return score(objects) >= 1.0 and objects["jacket"].resting() and objects["zip"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the drawcord toggle at the left end of the jacket's hem, fingers closing across it, and
# holds the jacket still by it.
# The right hand comes straight down on the pin's tab, fingers closing across it, and pushes the pin up into the box. It
# lets go, takes the slider's pull and pulls the zip all the way up to the collar. Then both hands rise and go back.

LEAN = math.radians(15.0)  # the holding hand leans in, its wrist out to the side (clear of the other hand)
TOG_PRE = -(TOG_R + 0.004)  # only 4 mm clear each side: the first finger to touch shoves the light jacket
TAB_PRE = -(TAB_T / 2 + 0.012)
PULL_PRE = -(PULL_T / 2 + 0.012)
PIN_PUSH = PIN_TRAVEL + 0.012  # aimed past the stop: the arm lags under the pin's resistance
ZIP_PULL = DONE + 0.040  # aimed past the top: the arm trails the slider by about 2 cm under its 3 N
OVER = 0.050
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena.constants import READY

    surface = TOP + JT
    # -- the left hand: holds the jacket still by the drawcord toggle at its hem
    # fingers closing along y, across the pull of the zip (a grip closing along x lets the jacket slip along it),
    # leaning in a little: its wrist out to the left
    hold_rot = _rx(-LEAN) @ grip_rotation(0.0)
    approach = hold_rot @ np.array([0.0, 0.0, 1.0])  # the gripper's pointing direction
    press = np.array([TOG_X, TOG_Y, TOP + TOG_H - 0.006])
    above = press - approach * OVER
    left = _joint_path("left", np.array(READY, float), _good_ik("left", above, hold_rot), "over the hem's drawcord toggle",
                       TOG_PRE)
    left.append(_w(1.0, press, TOG_PRE, "down round the toggle", hold_rot))
    left.append(_w(0.5, press, FINGER_CLOSED, "hold the jacket still by its hem's toggle", hold_rot))
    # -- the right hand: pushes the pin home, then pulls the zip up
    rot = grip_rotation(0.0)  # fingers closing across the tabs (along y), pushing and pulling along x
    tab_x = PIN_X1 - PIN_LEN - TAB_L / 2 + 0.004
    # a thin tab is pinched about 9 mm deep: the TCP 4 mm above its top (the fingertips reach 13 mm past the TCP)
    tab = np.array([tab_x, -0.0045, surface + TAB_H + 0.004])
    pull = np.array([SL_X, 0.0, surface + SL_H + 0.004 + PULL_H + 0.004])
    right = _joint_path("right", np.array(READY, float), _good_ik("right", tab + [0.0, 0.0, OVER], rot),
                        "over the pin's tab", TAB_PRE)
    t_left = sum(w.duration for w in left)
    t_right = sum(w.duration for w in right)
    if t_right < t_left + 0.6:  # the right waits over the tab until the hem is held
        right.append(_w(t_left + 0.6 - t_right, tab + [0.0, 0.0, OVER], TAB_PRE, "over the pin's tab", rot))
    right.append(_w(1.0, tab, TAB_PRE, "down round the pin's tab", rot))
    right.append(_w(0.5, tab, FINGER_CLOSED, "take the pin's tab", rot))
    for k in range(1, 5):
        right.append(_w(0.5, tab + [PIN_PUSH * k / 4, 0.0, 0.0], FINGER_CLOSED, "push the pin up into the box", rot))
    right.append(_w(0.5, tab + [PIN_PUSH, 0.0, 0.0], TAB_PRE, "let go of the pin's tab", rot))
    right.append(_w(0.8, tab + [PIN_PUSH, 0.0, OVER], TAB_PRE, "rise clear of it", rot))
    right.append(_w(1.0, pull + [0.0, 0.0, OVER], PULL_PRE, "over the slider's pull", rot))
    right.append(_w(1.0, pull, PULL_PRE, "down round the pull", rot))
    right.append(_w(0.5, pull, FINGER_CLOSED, "take the pull", rot))
    n = 10
    for k in range(1, n + 1):
        right.append(_w(0.45, pull + [ZIP_PULL * k / n, 0.0, 0.0], FINGER_CLOSED, "pull the zip up to the collar", rot))
    right.append(_w(0.6, pull + [ZIP_PULL, 0.0, 0.0], PULL_PRE, "let go of the pull", rot))
    right.append(_w(0.8, pull + [ZIP_PULL, 0.0, OVER], PULL_PRE, "rise clear of the jacket", rot))
    right += _joint_path("right", _good_ik("right", pull + [ZIP_PULL, 0.0, OVER], rot), np.array(READY, float),
                         "back to where it started", PULL_PRE)
    # the left holds until the right has let go of the zip, then rises and goes back
    t_release = sum(w.duration for w in right[:-LEAD_STEPS - 1])
    t_now = sum(w.duration for w in left)
    left.append(_w(max(0.3, t_release - t_now), press, FINGER_CLOSED, "hold the jacket still by its hem's toggle",
                   hold_rot))
    left.append(_w(0.5, press, TOG_PRE, "let go of the toggle", hold_rot))
    left.append(_w(0.8, above, TOG_PRE, "rise clear of it", hold_rot))
    left += _joint_path("left", _good_ik("left", above, hold_rot), np.array(READY, float), "back to where it started",
                        TOG_PRE)
    for plan in (left, right):
        last = plan[-1]
        plan.append(_w(3.0, last.pos, last.finger,
                       "hold still: the trial ends once the full score has held 1 s with both arms home", last.yaw))
    # line the two plans up in time at the end
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    short = left if tl < tr else right
    last = short[-1]
    short[-1] = _w(last.duration + abs(tl - tr), last.pos, last.finger, last.label, last.yaw)
    return {"left": left, "right": right}
