"""On the station table, a Victor M326 Professional rat trap (a wooden snap trap with a yellow plastic trigger pedal)
is screwed to an oak plank that lies loose on the table. Its kill bar lies sprung forward over the trap's far half. Its
arm bar stands up at the near end, leaning back against its stop.

Task: set the trap, its arm bar holding the kill bar back, without ever letting the kill bar snap and without a gripper
in its strike path while nothing holds it back (a hand on the kill bar, or the arm bar hooked in the catch). Both
grippers end clear of the set trap and both arms back where they started.

Everything that holds the set trap is contact physics. The kill bar's torsion spring pushes it up against the arm bar;
the arm bar's tip pushes up under the ledge of the catch on the trigger pedal; the pedal, on its own hinge and return
spring, rests against its stop. The tip gets under the ledge only by being pressed down the catch's 45-degree chamfer,
which tips the pedal back until the ledge clears the tip; the pedal then springs back over it. A touch on the pedal
tips it the same way, the ledge slides off the tip and the kill bar snaps. Nothing is latched by a rule.

One way (the scripted demo): the left hand holds the plank down while the right pulls the kill bar back over the top,
against its spring, to the near end and holds it there; the left swings the arm bar forward over the kill bar and
presses its tip down the chamfer into the catch; the right lets go and the kill bar settles against the arm bar.

Run it:  inspect-robots run --embodiment yam_arena -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py \\
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
                     plate, rotation, segment)
from yam_arena.constants import FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plank and the trap: the trap along x, its near end toward the arms ------------------------------------------
# Victor M326: board 7 x 3-1/4 x 1/2 in (the maker's listing); the parts on it scaled from the maker's photos.
XC = 0.27  # the kill bar's axle (the coil springs' axis)
PLANK = (0.32, 0.22, 0.025)  # oak plank: length, width, thickness
BASE_MASS = 1.47  # the plank (1.32 kg) and the trap screwed to it (0.15 kg), lying loose on the table
BOARD = (XC - 0.102, XC + 0.076, 0.083, 0.0127)  # the trap's board: near end, far end (x), width, thickness
BASE_TOP = TOP + PLANK[2] + BOARD[3]
AXLE_Z = BASE_TOP + 0.007  # the axle, through the two coils (14 mm across)
KILL_R = 0.074  # the striking bar, from the axle: sprung, it lies on the board's far end
KILL_W = 0.070  # between the kill bar's side arms
BAR_R = 0.006  # the striking bar's radius in the simulation (a 12 mm rod; the real bar is about 3 mm wire)
KILL_MASS = 0.015
SLEEVE_MASS = 0.005  # of it, the striking bar's sleeve
COCKED = 175.0  # the kill bar's travel, from sprung forward (0) over the top to the near end
# The spring, calibrated to the published tests of this trap (Warburton et al. 2002, DOC Science Internal Series 83,
# Table 1): 8.2 N clamping force with the striking bar 10 mm off closed, and 3.16 J at impact. A linear torsion spring,
# torque = PRELOAD + SPRING_K * travel, through both (the energy released from COCKED to 10 mm off closed):
CLAMP_N, CLAMP_AT, STRIKE_J = 8.2, 0.010, 3.16
_A, _C = CLAMP_AT / KILL_R, math.radians(COCKED)
SPRING_K = (STRIKE_J - CLAMP_N * KILL_R * (_C - _A)) / ((_C ** 2 - _A ** 2) / 2 - _A * (_C - _A))  # N m per radian
PRELOAD = CLAMP_N * KILL_R - SPRING_K * _A  # N m with the bar closed (a torque pair on the kill bar and the board)
SET_TORQUE = PRELOAD + SPRING_K * _C  # N m with the kill bar pulled right back
SPRUNG = math.degrees(math.asin((AXLE_Z - BASE_TOP - BAR_R) / KILL_R))  # sprung, the striking bar lies on the board

BASE0 = np.array([XC, 0.0, TOP])  # the plank's origin where it starts (under the axle, on the table)

# -- the arm bar: on a staple at the near end, along the middle line ---------------------------------------------------
ARM_T = 0.003  # a 3 mm steel wire (square in the simulation)
XS = BOARD[0] + 0.008  # its staple
LATCHED = COCKED - 1.5  # the hooked arm bar holds the kill bar this far back, 1.5 degrees short of its stop
ZS = AXLE_Z + KILL_R * math.sin(math.radians(180.0 - LATCHED)) + BAR_R + ARM_T / 2  # the staple's pin: the arm bar
# lies level on the striking bar when it holds the kill bar
ARM_MASS = 0.007
ARM_REST = 112.0  # it starts standing up, leaning back against its stop (degrees up from lying forward)

# -- the trigger pedal and its catch (pedal frame: origin on the pedal's pivot, x toward the far end) ------------------
XP, ZP = XC + 0.010, BASE_TOP + 0.007  # the pedal's pivot: a wire staple through its near edge, beside the coils
PEDAL = (0.045, 0.046, 0.0025)  # the plate: length (from the pivot), width, thickness
PEDAL_MASS = 0.008
PEDAL_TRIP = 6.0  # the pedal tips at most this far (far end down) before its far edge meets the board
PEDAL_REST = -0.3  # its stop (degrees, far end up)
PEDAL_RETURN = (0.004, -1.5)  # its return spring: N m per degree, toward this target (degrees): 0.005 N m at the stop
H_L = ZS + ARM_T / 2 - ZP  # the ledge's underside, above the pivot: at the top of the level arm bar's tip
X_E = 0.0005  # the ledge's near edge, past the pivot
LEDGE = 0.0012  # how far the ledge overhangs the arm bar's tip
X_W = 0.0045  # the notch's back wall
LEDGE_T = 0.003  # the ledge's thickness under the chamfer
CHAMFER = 0.003  # the 45-degree chamfer above the ledge, this high
CATCH_W = 0.012  # the catch across the trap
ARM_LEN = XP + X_E + LEDGE - XS  # staple to the arm bar's tip: the tip reaches under the ledge

# -- scoring --------------------------------------------------------------------------------------------------------
PULLED_BACK = 165.0  # the kill bar this far back: pulled back
SET_AT = 160.0  # ... and held at least this far back by the hooked arm bar: set
SNAP_SPEED = 200.0  # deg/s: the kill bar swinging forward this fast (pulled back past 30 degrees): it snapped
CONTROL_HZ = 20.0
ZONE = (0.025, 0.095, 0.055)  # the strike path: this far from the axle (min, max) and across the trap (half-width)
ZONE_MARGIN = 15.0  # ... ahead of the kill bar by more than this many degrees
HOLDING = 0.035  # a TCP within this of the striking bar holds the kill bar back
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

PINE, STEEL, COPPER, YELLOW, DARK = (0.86, 0.74, 0.52), (0.62, 0.64, 0.67), (0.72, 0.45, 0.30), \
    (0.95, 0.80, 0.15), (0.20, 0.21, 0.23)


def _quat_y(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg)
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _base() -> Body:
    """The plank, the trap's board, the coils on their brackets and the arm bar's staple; origin on the table under
    the axle."""
    bx0, bx1, bw, bt = BOARD
    zb = BASE_TOP - TOP
    bxc = (bx0 + bx1) / 2 - XC
    shapes = [BoxShape((bxc, 0.0, PLANK[2] / 2), PLANK, (0.70, 0.55, 0.35)),
              BoxShape((bxc, 0.0, PLANK[2] + bt / 2), (bx1 - bx0, bw, bt), PINE)]
    for s in (1.0, -1.0):  # the coils and their brackets
        shapes += [CylShape((0.0, s * 0.026, AXLE_Z - TOP), 0.007, 0.016, DARK, collide=False, axis="Y"),
                   BoxShape((0.0, s * 0.037, zb + 0.004), (0.010, 0.002, 0.008), COPPER, collide=False)]
    shapes += [CylShape((0.0, 0.0, AXLE_Z - TOP), 0.0015, KILL_W + 0.004, COPPER, collide=False, axis="Y"),
               BoxShape((XS - XC, 0.0, (zb + ZS - TOP) / 2), (0.003, 0.010, ZS - BASE_TOP + 0.003), COPPER,
                        collide=False),  # the arm bar's staple
               BoxShape((XP - XC, 0.0, (zb + ZP - TOP) / 2), (0.003, 0.020, ZP - BASE_TOP), COPPER,
                        collide=False)]  # the pedal's staple
    return Body.build("trap_base", pos=(XC, 0.0, TOP), mass=BASE_MASS, shapes=shapes, color=PINE, friction=(0.6, 0.5),
                      solver_iterations=32, com=(bxc, 0.0, PLANK[2] / 2))


def _kill_bar() -> Body:
    """The U-shaped kill bar, lying sprung forward; origin on the axle."""
    shapes = [segment((0.0, s * KILL_W / 2, 0.0), (KILL_R, s * KILL_W / 2, 0.0), 0.004, COPPER) for s in (1.0, -1.0)]
    shapes += [CylShape((KILL_R, 0.0, 0.0), 0.0015, KILL_W + 0.004, COPPER, collide=False, axis="Y")]
    return Body.build("kill_bar", pos=(XC, 0.0, AXLE_Z), mass=KILL_MASS - SLEEVE_MASS, shapes=shapes, color=COPPER,
                      friction=(1.2, 1.0), com=(KILL_R * 0.6, 0.0, 0.0), solver_iterations=32,
                      start_xyzw=_quat_y(SPRUNG))


_BAR0 = (XC + KILL_R * math.cos(math.radians(SPRUNG)), 0.0, AXLE_Z - KILL_R * math.sin(math.radians(SPRUNG)))


def _sleeve() -> Body:
    """The striking bar's outside: a 12 mm sleeve free to turn on it, so that it turns in the fingers as easily as the
    real 3 mm wire does."""
    return Body.build("bar_sleeve", pos=_BAR0, mass=SLEEVE_MASS,
                      shapes=[CylShape((0.0, 0.0, 0.0), BAR_R, KILL_W + 0.004, COPPER, axis="Y")], color=COPPER,
                      friction=(1.2, 1.0), solver_iterations=32, start_xyzw=_quat_y(SPRUNG))


def _arm_bar() -> Body:
    """The arm bar, standing up leaning back against its stop; origin on its pin."""
    shapes = [BoxShape((ARM_LEN / 2 + 0.002, 0.0, 0.0), (ARM_LEN - 0.004, ARM_T, ARM_T), COPPER),
              CylShape((0.0, 0.0, 0.0), 0.003, 0.010, COPPER, collide=False, axis="Y")]  # its eye round the staple
    return Body.build("arm_bar", pos=(XS, 0.0, ZS), mass=ARM_MASS, shapes=shapes, color=COPPER, friction=(0.3, 0.25),
                      start_xyzw=_quat_y(-ARM_REST), com=(ARM_LEN / 2, 0.0, 0.0), solver_iterations=64,
                      contact_offset=0.002)


def _pedal() -> Body:
    """The yellow trigger pedal with its catch; origin on its pivot."""
    c, t = CHAMFER, 0.0014
    zl = H_L + LEDGE_T  # the chamfer starts on top of the ledge's near edge and rises at 45 degrees toward the far end
    inward = np.array([1.0, 0.0, -1.0]) / math.sqrt(2.0) * (t / 2)  # its middle plane, behind its face
    shapes = [BoxShape((PEDAL[0] / 2, 0.0, 0.0), PEDAL, YELLOW),
              BoxShape(((X_E + X_W) / 2, 0.0, H_L + LEDGE_T / 2), (X_W - X_E, CATCH_W, LEDGE_T), YELLOW),  # the ledge
              plate(np.array([X_E, 0.0, zl]) + inward, np.array([X_E + c, 0.0, zl + c]) + inward, CATCH_W, t,
                    (1.0, 0.0, -1.0), YELLOW),  # the chamfer
              BoxShape((X_W + 0.002, 0.0, (PEDAL[2] / 2 + zl + c) / 2), (0.004, CATCH_W, zl + c - PEDAL[2] / 2),
                       YELLOW)]  # the notch's back wall
    return Body.build("pedal", pos=(XP, 0.0, ZP), mass=PEDAL_MASS, shapes=shapes, color=YELLOW, friction=(0.2, 0.15),
                      com=(0.019, 0.0, 0.0), solver_iterations=64, contact_offset=0.002,
                      start_xyzw=_quat_y(PEDAL_REST))


OBJECTS = [_base(), _kill_bar(), _sleeve(), _arm_bar(), _pedal()]
# the kill bar turns about +y the negative way as it is pulled back (its striking bar going up and over toward -x)
hinge("kill_bar_axle", "trap_base", "kill_bar", anchor=(XC, 0.0, AXLE_Z), axis="Y", limits_deg=(-COCKED, SPRUNG),
      spring=math.radians(1.0) * SPRING_K, damping=2e-6, target_deg=0.0)
hinge("bar_sleeve_turn", "kill_bar", "bar_sleeve", anchor=_BAR0, axis="Y", spring=0.0, damping=1e-7)
hinge("arm_bar_pin", "trap_base", "arm_bar", anchor=(XS, 0.0, ZS), axis="Y", limits_deg=(-ARM_REST, 8.0),
      spring=0.0, damping=1e-5, target_deg=0.0)
# the pedal tips the positive way about +y (its far end down) when pressed; its return spring holds it on its stop
hinge("pedal_pin", "trap_base", "pedal", anchor=(XP, 0.0, ZP), axis="Y", limits_deg=(PEDAL_REST, PEDAL_TRIP),
      spring=PEDAL_RETURN[0], damping=2e-5, target_deg=PEDAL_RETURN[1])


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"snapped": False, "unsafe": False, "pulled": False, "tcps": None, "tcps_w": None, "s": {}, "steps": 0,
            "moved": 0.0, "preload": False, "back": None, "still": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TRAP_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _preload(base) -> None:
    """The coil springs' preload: a torque pair about the axle, on the kill bar (toward sprung) and on the board."""
    import torch

    for name, sign in (("kill_bar", 1.0), ("trap_base", -1.0)):
        obj = base.scene[name]
        t = torch.zeros((obj.num_instances, 1, 3), device=obj.device)
        t[..., 1] = sign * PRELOAD
        obj.permanent_wrench_composer.set_forces_and_torques_index(torques=t, is_global=False)
    _EP["preload"] = True


def _grippers(base, origin) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each arm's (TCP, fingertips' middle)."""
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        r = rotation(g[3:7])
        tcp = g[:3] - origin + r @ np.array(TCP_OFFSET)
        out.append((tcp, tcp + r[:, 2] * 0.013))
    return out


def _turn_about_y(r: np.ndarray) -> float:
    return math.degrees(math.atan2(float(r[0, 2]), float(r[0, 0])))


def _engaged(tip_top_end: np.ndarray, pedal_pose) -> tuple[bool, np.ndarray]:
    """Whether the arm bar's tip is in the catch's notch, under the ledge (in the pedal's frame)."""
    rp, pp = pedal_pose
    q = rp.T @ (tip_top_end - pp)  # the top corner of the tip's end, from the pedal's pivot
    inside = X_E + 0.0003 <= float(q[0]) <= X_W and H_L - 0.0025 <= float(q[2]) <= H_L + 0.0008 \
        and abs(float(q[1])) <= CATCH_W / 2
    return inside, q


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    if not _EP["preload"]:
        _preload(base)
    origin = as_numpy(base.scene.env_origins[0])
    # everything is measured in the trap's own frame (the plank lies loose and may be moved): a point p in the world
    # is taken to where it would be had the plank stayed where it starts
    b = base.scene["trap_base"].data
    bpose = as_numpy(b.root_link_pose_w)[0]
    rb, pb = rotation(bpose[3:7]), bpose[:3] - origin
    to_trap = lambda p: rb.T @ (np.asarray(p, float) - pb) + BASE0  # noqa: E731
    h = base.scene["kill_bar"].data
    hp = as_numpy(h.root_link_pose_w)[0]
    back = -_turn_about_y(rb.T @ rotation(hp[3:7]))  # 0: sprung forward; 180: pulled right back
    if back < -20.0:
        back += 360.0
    w_rel = rb.T @ (as_numpy(h.root_ang_vel_w)[0] - as_numpy(b.root_ang_vel_w)[0])
    speed = math.degrees(float(w_rel[1]))  # deg/s toward sprung is positive about the trap's y
    lp = as_numpy(base.scene["arm_bar"].data.root_link_pose_w)[0]
    lr = rotation(lp[3:7])
    up = -_turn_about_y(rb.T @ lr)  # the arm bar: 0 lying level forward, 112 leaning back
    tip = to_trap(lp[:3] - origin + lr @ np.array([ARM_LEN, 0.0, ARM_T / 2]))  # the top corner of its tip's end
    pp = as_numpy(base.scene["pedal"].data.root_link_pose_w)[0]
    rped = rb.T @ rotation(pp[3:7])
    pedal = _turn_about_y(rped)  # degrees, far end down
    engaged, q = _engaged(tip, (rped, to_trap(pp[:3] - origin)))
    bar = to_trap(hp[:3] - origin + rotation(hp[3:7]) @ np.array([KILL_R, 0.0, 0.0]))  # the striking bar's middle
    if back >= PULLED_BACK:
        _EP["pulled"] = True
    prev = _EP.get("back")
    if prev is not None and prev >= 30.0 and (prev - back) * CONTROL_HZ > SNAP_SPEED:
        _EP["snapped"] = True  # swung forward more than 10 degrees within one 50 ms control step
    still = prev is not None and abs(prev - back) < 0.2 and abs(_EP.get("up", up) - up) < 0.2
    _EP["back"], _EP["up"], _EP["still"] = back, up, still
    grips_w = _grippers(base, origin)
    _EP["tcps_w"] = [g[0] for g in grips_w]  # in the world (for "back where they started")
    grips = [(to_trap(t), to_trap(f)) for t, f in grips_w]
    _EP["tcps"] = [g[0] for g in grips]
    _EP["moved"] = float(np.linalg.norm(pb - BASE0))
    by_hand = any(abs(float(t[1] - bar[1])) <= KILL_W / 2 + 0.01 and math.hypot(
        float(t[0] - bar[0]), float(t[2] - bar[2])) <= HOLDING for t, _ in grips)
    held = engaged or by_hand
    if back >= 30.0 and not held:  # nothing holds the kill bar back: no gripper may be in its strike path
        for pts in grips:
            for p in pts:
                dx, dy, dz = float(p[0] - XC), float(p[1]), float(p[2] - AXLE_Z)
                rho = math.hypot(dx, dz)
                ahead = math.degrees(math.atan2(dz, dx))
                if ZONE[0] <= rho <= ZONE[1] and abs(dy) <= ZONE[2] and -5.0 <= ahead <= back - ZONE_MARGIN:
                    _EP["unsafe"] = True
    clear = all(float(np.linalg.norm(t - p)) >= LET_GO for t in _EP["tcps"] for p in (bar, tip))
    _EP["s"] = {"back": back, "up": up, "clear": clear, "engaged": engaged, "pedal": pedal}
    _EP["steps"] += 1
    if _DEBUG["on"] and _EP["steps"] % 10 == 0:
        (tl, _), (tr, _) = grips
        print(f"[trap] t={_EP['steps'] / 20:5.1f}s moved={_EP['moved'] * 1000:5.1f}mm kill={back:6.1f}deg "
              f"speed={speed:7.1f} arm={up:6.1f}deg pedal={pedal:5.2f}deg tip-in-pedal(mm)="
              f"{np.round(q * 1000, 1).tolist()} engaged={engaged} byhand={by_hand} snapped={_EP['snapped']} "
              f"unsafe={_EP['unsafe']} clear={clear} rTCP-bar={np.round((tr - bar) * 1000).astype(int).tolist()} "
              f"lTCP-tip={np.round((tl - tip) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if the kill bar ever snapped forward (swinging forward faster than 200 degrees a second once pulled back
    past 30 degrees), or if a gripper's TCP or fingertips were ever in its strike path (2.5 to 9.5 cm from the axle,
    within 5.5 cm of the trap's middle line, more than 15 degrees ahead of the kill bar) while nothing held it back
    (no TCP within 3.5 cm of the striking bar, and the arm bar's tip not in the catch). Else 1.0 with the trap set
    (the arm bar's tip in the catch's notch under its ledge, the kill bar held at least 160 degrees back), both hands
    let go (each TCP at least 4 cm from the striking bar and the arm bar's tip) and both arms back where they started;
    0.5 with the trap set; 0.25 if the kill bar was pulled back 165 degrees at some point; else 0.0."""
    s = _EP["s"]
    if _EP["snapped"] or _EP["unsafe"] or not s:
        return 0.0
    if s["engaged"] and s["back"] >= SET_AT:
        if s["clear"] and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["pulled"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps_w")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the kill bar and the arm bar still (each turned less than 0.2 degrees over the last control
    step): the trial ends there."""
    return score(objects) >= 1.0 and _EP["still"] and VARIANT != "pencil"


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand, leaning out to the left, holds the plank down beside the trap. The right, leaning out to the right
# with its grip pitched back 25 degrees, pinches the striking bar where it lies on the board's far end, its far finger
# past the board's end. It carries the bar up and over the top to the near end, leading it along its arc against the
# spring, the bar turning in its fingers, and presses it down there. The left lets go of the plank, pushes the arm
# bar's upper end forward so that it falls over the kill bar, and presses the arm bar down near its tip until the tip
# drops past the chamfer under the ledge; it rises and withdraws to the left. The right eases up, the kill bar settling
# against the arm bar, opens its fingers and rises. Both arms go back to where they started.
# Checks (TRAP_DEMO=...): near: the left holds the arm bar's tip just above the catch, within 8 mm of its hooked place
# but not under the ledge, while the right lets go and leaves, then lets go itself; ontop: the left sets the tip on
# top of the catch's ledge without pressing it in and leaves; onehand: the left never moves; pencil: the full demo,
# then a closed fingertip touches the pedal's far half (the trial does not end at full score).

VARIANT = os.environ.get("TRAP_DEMO", "")
BAR_Y = -0.022  # the right hand holds the striking bar here
LEAN = 30.0
PINCH = -0.003  # a firm pinch; the sleeve lets the bar turn in it
FORK = -(BAR_R + 0.005)
PINCH_DEPTH = 0.012  # the pinched striking bar's middle, this far past the TCP toward the fingertips
PITCH0 = 25.0  # the right grip is pitched back this far (degrees): the bar's pull goes into a finger or the palm
PRESS = 0.07  # the right presses the pulled-back kill bar down this far below where it lies (about 30 N)
ARM_STIFFNESS = 170.0  # N/m: the arm gives way this much under load, so it leads the bar round its arc
PLANK_AT = (XC + 0.030, 0.090)  # the left holds the plank down here, beside the trap
PLANK_PRESS = 0.03
ARM_PUSH_Z = 0.090  # the left pushes the arm bar over at this height above its staple, near its tip
ARM_PRESS_AT = ARM_LEN - 0.030  # ... and presses it into the catch here
ARM_PRESS = 0.012  # ... this far below the arm bar's level
TIP = 0.013 * math.cos(math.radians(LEAN))  # closed fingertips, this far below the leaning TCP


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


RIGHT_GRIP = _rx(LEAN) @ grip_rotation(math.pi / 2)  # fingers closing along x, across the bar; leaning out to the right
LEFT_HAND = _rx(-LEAN) @ grip_rotation(-math.pi / 2)  # closed fingers, thin across y; leaning out to the left
RIGHT_TOUCH = RIGHT_GRIP  # closed fingertips, for the pencil check
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


def _joint_path(arm: str, q0, q1, label: str, finger, s: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(s / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _on_bar(back: float, r: float = KILL_R) -> np.ndarray:
    a = math.radians(back)
    return np.array([XC + r * math.cos(a), BAR_Y, AXLE_Z + r * math.sin(a)])


def _on_arm(up: float, along: float, press: float = 0.0) -> np.ndarray:
    a = math.radians(up)
    return np.array([XS + along * math.cos(a), 0.0, ZS + along * math.sin(a) + ARM_T / 2 + TIP - press])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _pinch_pose(back: float, pitch: float, r: float = KILL_R) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP pinching the striking bar (at ``back`` degrees), the grip pitched back ``pitch`` degrees about
    the bar's own axis: the far finger under the bar's far side, so a lift presses the bar into it."""
    rot = _ry(pitch) @ RIGHT_GRIP
    return _on_bar(back, r) - rot[:, 2] * PINCH_DEPTH, rot


def _pitch(back: float) -> float:
    """The right grip's pitch: back PITCH0 while the bar's pull could draw it out of the fingers, upright at the near
    end, where the pull only presses the bar into the palm."""
    return PITCH0 * min(1.0, max(0.0, (COCKED - back) / (COCKED - 120.0)))


def _right(left_done: float) -> tuple[list[Waypoint], float]:
    hold_back = COCKED
    start, rot0 = _pinch_pose(-SPRUNG, PITCH0)
    out = _lead_in("right", start + [0.0, 0.0, 0.04], rot0, "over the striking bar", FORK)
    out += [_w(1.0, start, FORK, "down round the striking bar, the far finger past the board's end", rot0),
            _w(0.5, start, PINCH, "pinch it", rot0)]
    n = 14
    for k in range(1, n + 1):
        back = hold_back * k / n
        a = math.radians(back)
        lead = min((PRELOAD + SPRING_K * a) / KILL_R / ARM_STIFFNESS, 0.08)  # ahead of the bar along its arc
        p, r = _pinch_pose(back, _pitch(back))
        p = p + np.array([-math.sin(a), 0.0, math.cos(a)]) * lead
        out.append(_w(0.6 if k <= 4 else 0.5, p, PINCH, "pull the kill bar up and over to the near end, turning the "
                      "grip with it", r))
    lying, rot = _pinch_pose(hold_back, _pitch(hold_back))
    held = lying + [0.0, 0.0, -PRESS]
    out.append(_w(0.8, held, PINCH, "press it down at the near end", rot))
    cocked_at = sum(w.duration for w in out)
    out.append(_w(max(left_done + 0.5 - cocked_at, 0.5), held, PINCH, "hold the kill bar back", rot))
    out += [_w(1.5, lying + [0.0, 0.0, 0.005], PINCH, "ease up: the kill bar settles against the arm bar", rot),
            _w(0.6, lying + [0.0, 0.0, 0.005], FORK, "open the fingers", rot)]
    away = lying + [0.0, 0.0, 0.08]
    out += [_w(1.0, away, FORK, "rise, away from the trap", rot)]
    out += _go_home("right", away, rot, FORK)
    if VARIANT == "pencil":  # check: once set and both arms home, a closed fingertip touches the pedal's far half
        from yam_arena import ik, station
        from yam_arena.constants import READY

        p0, _ = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
        touch = np.array([XP + 0.032, 0.0, ZP + PEDAL[2] / 2 + TIP])
        out += [_w(3.0, p0, FORK, "wait: the trap is set", out[-1].yaw)]
        out += _lead_in("right", touch + [0.0, 0.0, 0.05], RIGHT_TOUCH, "over the trigger pedal", 0.0)
        out += [_w(1.5, touch + [0.0, 0.0, -0.004], 0.0, "touch the pedal's far half lightly", RIGHT_TOUCH),
                _w(1.0, touch + [0.0, 0.0, 0.05], 0.0, "rise", RIGHT_TOUCH)]
    return out, cocked_at


def _left(cocked_at: float) -> tuple[list[Waypoint], float]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    if VARIANT == "onehand":
        return [_w(cocked_at + 4.0, p0, 0.0, "the left never moves", r0)], cocked_at + 4.0
    plank = np.array([PLANK_AT[0], PLANK_AT[1], TOP + PLANK[2] + TIP])
    out = _lead_in("left", plank + [0.0, 0.0, 0.04], LEFT_HAND, "over the plank, beside the trap", 0.0)
    out += [_w(1.0, plank + [0.0, 0.0, -PLANK_PRESS], 0.0, "hold the plank down", LEFT_HAND),
            _w(max(cocked_at - 4.0, 0.5), plank + [0.0, 0.0, -PLANK_PRESS], 0.0, "hold the plank down", LEFT_HAND),
            _w(0.8, plank + [0.0, 0.0, 0.05], 0.0, "let go of the plank", LEFT_HAND)]
    z = ZS + ARM_PUSH_Z + TIP
    high = np.array([XS - 0.065, 0.0, ZS + 0.14])
    out += _joint_path("left", _good_ik("left", plank + [0.0, 0.0, 0.05], LEFT_HAND),
                       _good_ik("left", high, LEFT_HAND), "up and over, behind the arm bar", 0.0, 2.0)
    out += [_w(1.0, (XS - 0.065, 0.0, z), 0.0, "down behind the arm bar", LEFT_HAND)]
    for x in (XS - 0.035, XS - 0.012, XS + 0.010):
        out.append(_w(0.6, (x, 0.0, z), 0.0, "push the arm bar forward: it falls over the kill bar", LEFT_HAND))
    out += [_w(0.8, (XS + 0.010, 0.0, z + 0.04), 0.0, "rise", LEFT_HAND)]
    over = _on_arm(0.0, ARM_PRESS_AT) + [0.0, 0.0, 0.03]
    out += _joint_path("left", _good_ik("left", (XS + 0.010, 0.0, z + 0.04), LEFT_HAND),
                       _good_ik("left", over, LEFT_HAND), "over the arm bar, near its tip", 0.0, 1.5)
    if VARIANT == "ontop":  # check: the tip set on top of the catch's ledge, not pressed in; nothing holds it
        out += [_w(0.8, _on_arm(0.0, ARM_PRESS_AT, press=-0.003), 0.0, "set its tip on top of the catch's ledge",
                   LEFT_HAND)]
    elif VARIANT == "near":  # check: the tip held within 8 mm of its hooked place, not under the ledge
        near = _on_arm(0.0, ARM_PRESS_AT, press=-0.001)
        out += [_w(0.8, near, 0.0, "hold its tip just above the catch, not under the ledge", LEFT_HAND)]
        left_done = sum(w.duration for w in out)
        out += [_w(5.0, near, 0.0, "keep holding it there while the right lets go and leaves", LEFT_HAND)]
    else:
        out += [_w(0.8, _on_arm(0.0, ARM_PRESS_AT, press=ARM_PRESS), 0.0, "press its tip down the chamfer into the "
                   "catch", LEFT_HAND),
                _w(0.5, _on_arm(0.0, ARM_PRESS_AT, press=ARM_PRESS), 0.0, "until the pedal springs back over it",
                   LEFT_HAND)]
    out_at = _on_arm(0.0, ARM_PRESS_AT) + [-0.01, 0.07, 0.06]
    out += [_w(0.6, _on_arm(0.0, ARM_PRESS_AT) + [0.0, 0.0, 0.03], 0.0, "rise", LEFT_HAND),
            _w(1.0, out_at, 0.0, "withdraw to the left, out of the trap", LEFT_HAND)]
    if VARIANT != "near":
        left_done = sum(w.duration for w in out)
    out += _go_home("left", out_at, LEFT_HAND, 0.0)
    return out, left_done


def demo():
    _, cocked_at = _right(0.0)
    left, left_done = _left(cocked_at)
    right, _ = _right(left_done)
    total = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    for arm in (left, right):
        arm.append(_w(total - sum(w.duration for w in arm), arm[-1].pos, arm[-1].finger,
                      "hold still: the trial ends once the full score has held 1 s with both arms home", arm[-1].yaw))
    return {"left": left, "right": right}
