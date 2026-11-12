"""A desk pendulum stands on a smooth worktop: a walnut base with a brass knob on each end, a post at the back and an
arm reaching forward over the base, from whose end a brass pendulum hangs on a pivot, swinging left and right. Its bob
is a brass cylinder with a long knurled peg sticking out of its front, and under the peg a brass scale on the base is
marked every 5 degrees, with a red start mark 20 degrees out to the right. A STOP button (a red mushroom on a yellow box)
sits on the base's front right corner; it works the pendulum's magnetic brake.

Task: start the pendulum and stop it on its third swing. One hand holds the stand still by the knob on its left end all
the while (the stand is light and the worktop smooth, so it slides when the bob is pushed). The other pushes the bob by
its peg out to the red start mark and lets go, then keeps count: when the bob comes back to the right for the third
time, it presses STOP once, and the brake stops the pendulum hanging straight down. Don't touch the bob while it swings
and don't let the stand slide. Both arms end where they started.

Every swing looks the same (the swing hardly dies away), and nothing in view keeps the count: the policy has to count
the swings itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=stop_a_desk_pendulum_on_its_third_swing.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stand: its origin is on the table under the pivot ----------------------------------------------------------
PX, PY = 0.300, 0.000
BASE_T = 0.022
BX0, BX1, BY = -0.115, 0.110, 0.220  # the base, from the stand's origin: front and back edges, half width
POST_X, POST_W = 0.085, 0.028
L = 0.300  # the pivot to the bob's centre
PIVOT_H = 0.360  # the pivot above the table
PIVOT_Z = TOP + PIVOT_H
STAND_MASS = 0.30
KNOB = np.array([0.000, 0.200])  # the knob on the base's left end, in line with the pivot (the left hand holds it)
KNOB_R, KNOB_H = 0.010, 0.025
BUTTON = np.array([-0.080, -0.170])  # the STOP button, from the stand's origin
BOX_W, BOX_H = 0.040, 0.022
CAP_R, CAP_H = 0.013, 0.012
CAP_TOP = BASE_T + BOX_H + CAP_H  # above the table
LAMP = np.array([-0.093, -0.184])  # the brake lamp on the button's box
# -- the pendulum: its origin is the pivot ---------------------------------------------------------------------------
BOB_R, BOB_H = 0.020, 0.035
PEG_R, PEG_X0, PEG_X1 = 0.006, -0.020, -0.095  # the peg on the bob's front, along -x
BOB_MASS = 0.18
FREE_DRAG = 1.0e-6  # N m s per degree: the pivot's own friction, next to nothing
BRAKE_DRAG = 0.006  # N m s per degree once STOP is pressed (about three times critical damping)
# -- the scale and the rules --------------------------------------------------------------------------------------------
START_DEG = -20.0  # the red start mark: 20 degrees out to the right (-y)
PULLED_DEG = 15.0  # let go from at least this far out ...
PULLED_MAX = 35.0  # ... and no further
PEAK_DEG = 5.0  # a turn of the swing counts as a swing's far end only this far out
SWINGS = 3
NEAR_PEG, NEAR_BOB = 0.028, 0.045  # a TCP this close to the peg's axis or the bob's centre is at the bob
BTN_R, BTN_DZ, BTN_STEPS = 0.020, 0.020, 2  # a TCP this close to the cap's axis, this low over its top, this long
SLID = 0.030  # the stand moved this far (m) or tilted ...
TIPPED = 8.0  # ... this far (degrees) at any time: it slid
HANGING = 3.0  # the bob within this of straight down: stopped
# -- hidden lights, moved into view ----------------------------------------------------------------------------------
HIDDEN = np.array([0.40, 0.0, 0.30])

WALNUT, WALNUT_D, FELT = (0.36, 0.22, 0.13), (0.27, 0.16, 0.09), (0.12, 0.30, 0.16)
BRASS, BRASS_D, BRASS_L = (0.80, 0.63, 0.30), (0.62, 0.47, 0.20), (0.90, 0.78, 0.48)
STEEL, INK, RED, YELLOW = (0.72, 0.73, 0.75), (0.08, 0.07, 0.06), (0.86, 0.10, 0.08), (0.95, 0.78, 0.12)
DARK, AMBER = (0.15, 0.13, 0.12), (1.0, 0.55, 0.05)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _stand() -> Body:
    cx = (BX0 + BX1) / 2
    shapes = [BoxShape((cx, 0.0, BASE_T / 2), (BX1 - BX0, 2 * BY, BASE_T), WALNUT),
              BoxShape((cx, 0.0, BASE_T + 0.0004), (BX1 - BX0 - 0.012, 2 * BY - 0.012, 0.0008), WALNUT_D,
                       collide=False)]  # a moulded edge
    top = PIVOT_H + 0.035
    shapes.append(BoxShape((POST_X, 0.0, (BASE_T + top) / 2), (POST_W, POST_W, top - BASE_T), WALNUT))
    shapes.append(BoxShape((POST_X, 0.0, BASE_T + 0.006), (POST_W + 0.014, POST_W + 0.014, 0.012), WALNUT_D))
    arm_z = PIVOT_H + 0.022
    shapes.append(BoxShape(((POST_X - 0.014) / 2, 0.0, arm_z), (POST_X + 0.014 + 0.016, 0.024, 0.024), WALNUT))
    shapes.append(BoxShape((0.0, 0.0, PIVOT_H + 0.006), (0.026, 0.020, 0.012), BRASS_D))  # the pivot's brass bearing
    shapes.append(CylShape((POST_X, 0.0, top + 0.004), 0.010, 0.008, BRASS, collide=False))  # a finial
    # a brass knob on each end of the base, in line with the pivot, to hold the stand by
    for ky in (KNOB[1], -KNOB[1]):
        shapes.append(CylShape((KNOB[0], ky, BASE_T + 0.003), KNOB_R + 0.006, 0.006, BRASS_D))  # its rose
        shapes.append(CylShape((KNOB[0], ky, BASE_T + KNOB_H / 2), KNOB_R, KNOB_H, BRASS))
        shapes.append(CylShape((KNOB[0], ky, BASE_T + KNOB_H + 0.0005), KNOB_R - 0.002, 0.001, BRASS_L, collide=False))
    # the scale: a brass plate under the peg's path, marked every 5 degrees, the start mark red
    sx = (PEG_X0 + PEG_X1) / 2 - 0.004
    half = L * math.sin(math.radians(32))
    shapes.append(BoxShape((sx, 0.0, BASE_T + 0.0006), (0.036, 2 * half, 0.0012), BRASS_L, collide=False))
    for d in range(-30, 31, 5):
        y = L * math.sin(math.radians(d))
        long = d % 10 == 0
        shapes.append(BoxShape((sx - (0.0 if long else 0.006), y, BASE_T + 0.0014),
                               (0.026 if long else 0.014, 0.0016 if d else 0.0024, 0.0006), INK, collide=False))
    ys = L * math.sin(math.radians(START_DEG))
    shapes.append(BoxShape((sx, ys, BASE_T + 0.0018), (0.034, 0.0045, 0.0006), RED, collide=False))
    shapes.append(BoxShape((sx - 0.022, ys, BASE_T + 0.0018), (0.010, 0.010, 0.0006), RED, collide=False))
    # the STOP button: a yellow box with a red mushroom cap, and the brake lamp (dark until braked)
    bx, by = BUTTON
    shapes.append(BoxShape((bx, by, BASE_T + BOX_H / 2), (BOX_W, BOX_W, BOX_H), YELLOW))
    shapes.append(CylShape((bx, by, BASE_T + BOX_H + 0.002), CAP_R - 0.003, 0.004, DARK))
    shapes.append(CylShape((bx, by, BASE_T + BOX_H + 0.004 + (CAP_H - 0.004) / 2), CAP_R, CAP_H - 0.004, RED))
    shapes.append(CylShape((LAMP[0], LAMP[1], BASE_T + BOX_H + 0.0008), 0.0045, 0.0016, DARK, collide=False))
    # felt pads under the base (visual)
    for fx in (BX0 + 0.015, BX1 - 0.015):
        for fy in (-BY + 0.015, BY - 0.015):
            shapes.append(BoxShape((fx, fy, 0.0006), (0.018, 0.018, 0.0012), FELT, collide=False))
    return Body.build("stand", pos=(PX, PY, TOP), mass=STAND_MASS, shapes=shapes, color=WALNUT,
                      friction=(0.07, 0.06), slick=True, com=(0.010, 0.0, 0.030), damping=(0.0, 0.05))


def _pendulum() -> Body:
    """The pendulum in its own frame (origin at the pivot), hanging straight down."""
    rod = L - BOB_H / 2
    shapes = [CylShape((0.0, 0.0, 0.0), 0.004, 0.030, BRASS_L, axis="X", collide=False),  # the pivot's axle
              CylShape((0.0, 0.0, -0.006), 0.006, 0.014, BRASS_D),  # the suspension block
              CylShape((0.0, 0.0, -rod / 2), 0.003, rod, STEEL),
              CylShape((0.0, 0.0, -L), BOB_R, BOB_H, BRASS),
              CylShape((0.0, 0.0, -L + BOB_H / 2 + 0.0005), BOB_R - 0.003, 0.001, BRASS_L, collide=False),
              CylShape((0.0, 0.0, -L - BOB_H / 2 - 0.0005), BOB_R - 0.003, 0.001, BRASS_D, collide=False),
              CylShape((0.0, 0.0, -L + BOB_H / 2 + 0.004), 0.006, 0.008, BRASS_D)]  # the rod's nut on the bob
    for z in (-0.009, 0.0, 0.009):  # bands round the bob
        shapes.append(CylShape((0.0, 0.0, -L + z), BOB_R + 0.0005, 0.0012, BRASS_D, collide=False))
    shapes.append(CylShape(((PEG_X0 + PEG_X1) / 2, 0.0, -L), PEG_R, PEG_X0 - PEG_X1, BRASS_L, axis="X"))
    for k in range(8):  # knurling
        x = PEG_X1 + 0.006 + 0.008 * k
        shapes.append(CylShape((x, 0.0, -L), PEG_R + 0.0005, 0.002, BRASS_D, axis="X", collide=False))
    shapes.append(CylShape((PEG_X1 - 0.003, 0.0, -L), PEG_R + 0.002, 0.006, BRASS_D, axis="X"))  # the peg's end knob
    return Body.build("pendulum", pos=(PX, PY, PIVOT_Z), mass=BOB_MASS, shapes=shapes, color=BRASS,
                      friction=(0.6, 0.5), com=(-0.004, 0.0, -L + 0.002), damping=(0.0, 0.0), solver_iterations=32)


def _lamp() -> Body:
    """The brake lamp lit, hidden until STOP is pressed."""
    return Body.build("brake_lamp", pos=tuple(HIDDEN.tolist()), mass=0.01, kinematic=True, color=AMBER,
                      shapes=[CylShape((0.0, 0.0, 0.0), 0.0050, 0.0018, AMBER, collide=False)])


hinge("swing", "stand", "pendulum", anchor=(PX, PY, PIVOT_Z), axis="X", limits_deg=(-75.0, 75.0), spring=0.0,
      damping=FREE_DRAG, target_deg=0.0)

OBJECTS = [_stand(), _pendulum(), _lamp()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"theta": 0.0, "dtheta": 0.0, "pulled": False, "out_to": 0.0, "released": False, "release_deg": None,
            "far_peaks": 0,
            "press_steps": 0, "pressed_at": None, "braked": False, "touched": False, "slid": False, "lamp": False,
            "tcps": None, "stand_off": 0.0, "stand_tilt": 0.0, "steps": 0, "peak_times": []}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PENDULUM_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


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


def _place(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _at_bob(q) -> bool:
    """Is a point (in the pendulum's frame) at the bob or its peg?"""
    x = min(max(q[0], PEG_X1 - 0.006), PEG_X0)
    near_peg = math.hypot(q[0] - x, math.hypot(q[1], q[2] + L)) < NEAR_PEG
    near_bob = float(np.linalg.norm(q - np.array([0.0, 0.0, -L]))) < NEAR_BOB
    return near_peg or near_bob


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    ps, rs = _pose(base, origin, "stand")
    pp, rp = _pose(base, origin, "pendulum")
    rel = rs.T @ rp
    theta = math.degrees(math.atan2(rel[2, 1], rel[1, 1]))  # + : the bob out to the left (+y)
    dtheta = theta - _EP["theta"]
    _EP["steps"] += 1
    # the stand on its spot
    _EP["stand_off"] = math.hypot(ps[0] - PX, ps[1] - PY)
    _EP["stand_tilt"] = math.degrees(math.acos(max(-1.0, min(1.0, float(rs[2, 2])))))
    if _EP["stand_off"] > SLID or _EP["stand_tilt"] > TIPPED:
        _EP["slid"] = True  # latched
    # the hands at the bob
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    at_bob = any(_at_bob(rp.T @ (t - pp)) for t in tcps)
    # pushed out to the start mark (right, -y), then let go: the swing starts
    if not _EP["released"]:
        if at_bob:  # how far out the hand has the bob
            _EP["out_to"] = min(_EP["out_to"], theta)
        elif theta > -PULLED_DEG + 3.0:
            _EP["out_to"] = 0.0
        _EP["pulled"] = -PULLED_MAX <= _EP["out_to"] <= -PULLED_DEG
        if _EP["pulled"] and not at_bob and not _EP["braked"]:
            _EP["released"] = True
            _EP["release_deg"] = _EP["out_to"]
            _EP["release_step"] = _EP["steps"]
    elif not _EP["braked"] and at_bob:
        _EP["touched"] = True  # a hand at the swinging bob: latched
    # each swing's far end (out to the left), counted once the bob has been let go
    if _EP["released"] and not _EP["braked"] and theta > PEAK_DEG and _EP["dtheta"] > 0.0 >= dtheta:
        _EP["far_peaks"] += 1
        _EP["peak_times"].append(_EP["steps"])
    _EP["theta"], _EP["dtheta"] = theta, dtheta
    # the STOP button, pressed by a fingertip: the brake goes on at the first press
    cap = ps + rs @ np.array([BUTTON[0], BUTTON[1], CAP_TOP])
    pressing = any(math.hypot(t[0] - cap[0], t[1] - cap[1]) <= BTN_R and t[2] <= cap[2] + BTN_DZ for t in tcps)
    _EP["press_steps"] = _EP["press_steps"] + 1 if pressing else 0
    if _EP["press_steps"] >= BTN_STEPS and not _EP["braked"]:
        _EP["braked"] = True
        _EP["pressed_at"] = _EP["far_peaks"] if _EP["released"] else -1
        _EP["press_step"] = _EP["steps"]
        set_hinge_drive("swing", target_deg=0.0, spring=0.0, damping=BRAKE_DRAG)
    if _EP["braked"]:
        _place(base, origin, "brake_lamp", ps + rs @ np.array([LAMP[0], LAMP[1], BASE_T + BOX_H + 0.0020]))
        _EP["lamp"] = True
    if _DEBUG["on"] and (_EP["steps"] % 2 == 0 or pressing):
        tl, tr = tcps
        print(f"[pend] t={_EP['steps'] / 20:5.2f}s th={theta:6.1f} pulled={_EP['pulled']} rel={_EP['released']}"
              f" rdeg={_EP['release_deg']} peaks={_EP['far_peaks']} at_bob={at_bob} braked={_EP['braked']}"
              f" pressed_at={_EP['pressed_at']} touched={_EP['touched']} slid={_EP['slid']}"
              f" stand={np.round((ps[:2] - [PX, PY]) * 1000).astype(int).tolist()}mm/{_EP['stand_tilt']:.1f}"
              f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
              f" R={np.round((tr - [0, 0, TOP]) * 1000).astype(int).tolist()}"
              f" pegdy={(tr[1] - (pp + rp @ np.array([PEG_X1 + 0.015, 0.0, -L]))[1]) * 1000:.0f}"
              f" pressing={pressing} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once the stand has slid (moved more than 2.5 cm or tilted more than 8 degrees) or a hand has touched the bob
    while it swings. Otherwise, at the end: 1.0 with the bob let go from the start mark (15 to 35 degrees out to the
    right), STOP first pressed on the third swing (after the bob's third turn at the far left end of its swing and
    before its fourth), the pendulum now hanging still within 3 degrees of straight down and both arms back where they
    started; 0.75 with all of that but the arms not home or the bob not yet hanging still; 0.5 with the bob let go from
    the start mark and STOP pressed, but not on the third swing; 0.25 with the bob let go from the start mark; else
    0.0."""
    if _EP["slid"] or _EP["touched"]:
        return 0.0
    if not _EP["released"]:
        return 0.0
    if not _EP["braked"]:
        return 0.25
    if _EP["pressed_at"] != SWINGS:
        return 0.5
    if abs(_EP["theta"]) <= HANGING and _arms_home():
        return 1.0
    return 0.75


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
    """Full score with the stand and the pendulum still: the trial ends there."""
    return score(objects) >= 1.0 and objects["stand"].resting() and objects["pendulum"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the knob on the stand's left end and holds it. The right comes down in front of the
# pendulum, leaning back, its closed fingers beside the bob's peg, pushes the bob out along its arc to the red start mark
# and holds it there, then lifts straight up off the peg: the pendulum swings. The right goes over the STOP button and waits, counting; as the bob comes
# back to the right for the third time it presses STOP, and the brake stops the pendulum hanging straight down. Both
# hands let go and go home.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.060
KNOB_FINGER = -0.022
PEG_FINGER = FINGER_CLOSED
PUSH_DY = 0.014  # the TCP this far to the left (+y) of the peg's axis: the closed fingers' side against the peg
PUSH_DZ = 0.004
PEG_PUSH = 0.015  # the hand pushes the peg this far in from its end
PUSH_PITCH = -30.0  # degrees: the pushing hand leans back toward the arms
PRE = np.array([PX - 0.095, -0.080, TOP + 0.130])  # in front of the pendulum, clear of its rod
PUSH_TO = -22.0  # degrees: aimed past the start mark (the soft arm lags against the bob's weight)
PUSH_STEPS = 10
LIFT = 0.032
PERIOD = 1.20  # s: the pendulum's period (measured)
PRESS_AFTER = 3.0  # periods after the lift starts: the press lands mid-window
PRESS_DZ = 0.006
BRACE = np.array([0.0, 0.006, -0.002])  # the left hand's aim shifted this far against the push at full push


def _peg(theta_deg: float) -> np.ndarray:
    """The peg's middle with the pendulum at ``theta_deg`` (the stand unmoved)."""
    t = math.radians(theta_deg)
    return np.array([PX + PEG_X1 + PEG_PUSH, PY + L * math.sin(t), PIVOT_Z - L * math.cos(t)])


def _pitch(deg: float) -> np.ndarray:
    b = math.radians(deg)
    return np.array([[math.cos(b), 0.0, math.sin(b)], [0.0, 1.0, 0.0], [-math.sin(b), 0.0, math.cos(b)]])


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

    L_, R_ = [], []
    up = np.array([0.0, 0.0, OVER])

    # the left hand takes the knob on the stand's left end and holds the stand
    knob = np.array([PX + KNOB[0], PY + KNOB[1], TOP + BASE_T + KNOB_H - 0.002])
    kr = grip_rotation(-math.pi / 2)
    L_ += _joint_path("left", np.array(READY, float), _good_ik("left", knob + up, kr), "over the knob on the stand's left end",
                      KNOB_FINGER)
    L_.append(_w(1.2, knob, KNOB_FINGER, "down round the knob", kr))
    L_.append(_w(0.6, knob, FINGER_CLOSED, "hold the stand still by its knob", kr))
    # the right comes down with its fingers either side of the bob's peg
    g0 = _pitch(PUSH_PITCH) @ grip_rotation(0.0)  # leaning back: the hand's body clear of the pendulum's rod
    p0 = _peg(0.0) + [0.0, 0.0, PUSH_DZ]
    R_ += _joint_path("right", np.array(READY, float), _good_ik("right", PRE, g0), "in front of the pendulum", PEG_FINGER)
    R_.append(_w(0.8, p0 + up + [0.0, PUSH_DY + 0.008, 0.0], PEG_FINGER, "over the bob's peg", g0))
    R_.append(_w(1.2, p0 + [0.0, PUSH_DY + 0.008, 0.0], PEG_FINGER, "down beside the peg, on its left", g0))
    R_.append(_w(max(0.3, sum(w.duration for w in L_) - sum(w.duration for w in R_)), p0, PEG_FINGER,
                 "wait for the stand to be held", g0))
    R_.append(_w(0.8, p0 + [0.0, PUSH_DY, 0.0], PEG_FINGER, "the fingers' side against the peg", g0))
    t_push = sum(w.duration for w in R_)
    # push the bob out along its arc to the red start mark, and hold it there
    for k in range(1, PUSH_STEPS + 1):
        th = PUSH_TO * k / PUSH_STEPS
        R_.append(_w(0.22, _peg(th) + [0.0, PUSH_DY, PUSH_DZ], PEG_FINGER, "push the bob out to the start mark", g0))
    held = _peg(PUSH_TO) + [0.0, PUSH_DY, PUSH_DZ]
    R_.append(_w(0.6, held, PEG_FINGER, "hold it at the start mark", g0))
    # let go: straight up off the peg
    t_lift = sum(w.duration for w in R_)
    # the left braces against the push as it builds up (the soft arm gives way otherwise), and eases off at the let-go
    L_.append(_w(max(0.2, t_push - sum(w.duration for w in L_)), knob, FINGER_CLOSED, "hold the stand still by its knob",
                 kr))
    for k in range(1, PUSH_STEPS + 1):
        L_.append(_w(0.22, knob + BRACE * k / PUSH_STEPS, FINGER_CLOSED, "hold the stand against the push", kr))
    L_.append(_w(max(0.1, t_lift - sum(w.duration for w in L_)), knob + BRACE, FINGER_CLOSED,
                 "hold the stand against the push", kr))
    L_.append(_w(0.3, knob, FINGER_CLOSED, "hold the stand still by its knob", kr))
    R_.append(_w(0.15, held + [0.0, 0.0, LIFT], PEG_FINGER, "let go: up off the peg", g0))
    R_.append(_w(0.35, held + [-0.020, 0.0, LIFT + 0.020], FINGER_CLOSED, "clear of the swing", g0))
    # over the STOP button, waiting for the third swing
    cap = np.array([PX + BUTTON[0], PY + BUTTON[1], TOP + CAP_TOP])
    gb = grip_rotation(math.pi / 2)
    hover = cap + [0.0, 0.0, 0.013 + 0.045]
    R_.append(_w(1.2, hover, FINGER_CLOSED, "over the STOP button, counting the swings", gb))
    t_press = t_lift + PRESS_AFTER * PERIOD
    R_.append(_w(max(0.2, t_press - 0.25 - sum(w.duration for w in R_)), hover, FINGER_CLOSED,
                 "over the STOP button, counting the swings", gb))
    press = cap + [0.0, 0.0, PRESS_DZ]
    R_.append(_w(0.25, press, FINGER_CLOSED, "press STOP on the third swing", gb))
    R_.append(_w(0.35, press, FINGER_CLOSED, "press STOP on the third swing", gb))
    R_.append(_w(0.4, hover, FINGER_CLOSED, "lift the finger off", gb))
    R_.append(_w(1.6, hover, FINGER_CLOSED, "the brake stops the pendulum", gb))
    t_done = sum(w.duration for w in R_)
    R_ += _joint_path("right", _good_ik("right", hover, gb), np.array(READY, float), "back to where it started",
                      FINGER_CLOSED)
    # the left holds the stand until the pendulum has stopped, then lets go and goes home
    L_.append(_w(max(0.2, t_done - sum(w.duration for w in L_)), knob, FINGER_CLOSED, "hold the stand still by its knob",
                 kr))
    L_.append(_w(0.5, knob, KNOB_FINGER, "let go of the knob", kr))
    L_.append(_w(0.8, knob + up, KNOB_FINGER, "rise clear of the knob", kr))
    L_ += _joint_path("left", _good_ik("left", knob + up, kr), np.array(READY, float), "back to where it started",
                      KNOB_FINGER)
    t_end = max(sum(w.duration for w in R_), sum(w.duration for w in L_)) + 3.0
    R_.append(_w(t_end - sum(w.duration for w in R_), R_[-1].pos, FINGER_CLOSED,
                 "hold still: the trial ends once the full score has held 1 s with both arms home", R_[-1].yaw))
    L_.append(_w(t_end - sum(w.duration for w in L_), L_[-1].pos, KNOB_FINGER, "wait at home", L_[-1].yaw))
    return {"left": L_, "right": R_}
