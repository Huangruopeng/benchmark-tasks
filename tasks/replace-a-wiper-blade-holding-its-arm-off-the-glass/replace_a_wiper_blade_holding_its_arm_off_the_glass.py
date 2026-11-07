"""The front of a car stands on the table: its black cowl, its windscreen rising away from the arms between two red
pillars, and the driver's wiper parked along the bottom of the glass, its arm pivoting at the left and pressed onto the
glass by its spring. The wiper's old blade, its rubber worn grey, hangs on the hook at the arm's tip. In front of the car
lie a new blade in its cardboard tray and a long grey scrap tray.

Task: replace the blade. One hand lifts the wiper arm off the glass and holds it up (let go, and its spring snaps it
down onto the glass); the other slides the old blade along the arm toward the pivot, off its hook, and drops it in the
scrap tray, then takes the new blade from its tray, hooks it onto the arm and slides it toward the tip until it clicks.
Then the arm is lowered gently back onto the glass (letting it snap down could crack the windscreen) and both arms go
back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=replace_a_wiper_blade_holding_its_arm_off_the_glass.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the car (fixed): cowl, windscreen rising away from the arms at 20 degrees, pillars, roof edge --------------------
SLOPE = math.radians(20.0)
GLASS_X0 = 0.310  # the glass's bottom edge
GLASS_LEN = 0.300  # along its slope
GLASS_Y = (-0.25, 0.29)
GLASS_T = 0.006
GLASS_Z0 = TOP + 0.040  # the glass's top surface at its bottom edge


def glass_z(x: float) -> float:
    """Height of the windscreen's top surface at ``x``."""
    return GLASS_Z0 + (x - GLASS_X0) * math.tan(SLOPE)


# -- the wiper: arm parked along y on the glass, pivot at the left; the blade hangs under it on a hook -----------------
BLADE_X = 0.345  # the blade's rubber runs along the glass here
BLADE_LEN = 0.240
FRAME = (0.012, BLADE_LEN, 0.014)  # the blade's frame: x, y, z
RUBBER_H = 0.004
BLADE_Z = glass_z(BLADE_X) + RUBBER_H + FRAME[2] / 2  # the frame's middle, blade on the glass
ARM_GAP = 0.015  # from the blade's top up to the arm's underside
ARM = (0.012, 0.270, 0.009)  # the arm: x, its length along y, z
ARM_Z = BLADE_Z + FRAME[2] / 2 + ARM_GAP + ARM[2] / 2
PIVOT = np.array([BLADE_X, 0.170, ARM_Z])
HOOK_AT = 0.250  # the hook (the blade's middle, its adapter) this far from the pivot along the arm
HOOK_LOCAL = np.array([0.0, -HOOK_AT, BLADE_Z - ARM_Z])  # in the arm's frame (origin at the pivot)
BLADE_AT = PIVOT + HOOK_LOCAL  # the old blade's middle at the start
ARM_MASS, BLADE_MASS = 0.05, 0.03
SPRING = 0.004  # N m per degree, pressing the arm down toward the glass ...
SPRING_TARGET = 5.0  # ... toward 5 degrees below where the blade rests on it
LIFT_MAX = 35.0
DOWN_MAX = 6.0  # without a blade the arm comes down this far (its tip on the glass)

# -- the hook: a slide along the arm (a bodykit Thread) the task file switches on and off ------------------------------
UNHOOK = 0.020  # a blade slid this far toward the pivot comes off the hook
CLICK = 0.004  # within this of the hook's end it clicks in
LIFT_FREE = 8.0  # the blade slides on the hook only with the arm lifted at least this far (deg); on the glass it is held
HOLD_LAG = 0.003  # stick-slip: the hook's hold follows a blade pushed more than 3 mm past it
SOFT_K = 400.0  # N/m: the hook's hold (stick-slip while the arm is lifted; on the glass the hold stays put)
CATCH_OFF, CATCH_S, CATCH_DEG = 0.015, (0.005, 0.016), 30.0  # a free blade catches on the hook within these

# -- the new blade's cardboard tray and the scrap tray ----------------------------------------------------------------
CARD = np.array([0.225, -0.080])  # the new blade's middle
CARD_T = 0.004
NEW_AT = np.array([CARD[0], CARD[1], TOP + CARD_T + RUBBER_H + FRAME[2] / 2])
SCRAP = np.array([0.140, -0.150])  # the scrap tray's middle
SCRAP_IN = (0.060, 0.290, 0.035)  # inside: x, y, height

GLASS_C, FRAME_C, BODY_C, COWL_C, ARM_C, RUBBER_C, WORN_C, STEEL_C, CARD_C, TRAY_C = \
    (0.24, 0.35, 0.40), (0.12, 0.12, 0.13), (0.62, 0.06, 0.07), (0.07, 0.07, 0.08), (0.10, 0.10, 0.11), \
    (0.04, 0.04, 0.04), (0.42, 0.42, 0.40), (0.72, 0.73, 0.75), (0.70, 0.56, 0.38), (0.45, 0.46, 0.48)

# -- scoring ----------------------------------------------------------------------------------------------------------
DOWN = 3.0  # the arm is down on the glass within this of its resting angle
SLAM_DEG = 4.0  # an arm reaching the glass ...
SLAM_SPEED = 1.5  # ... faster than this (rad/s, ~86 deg/s) has snapped down onto it
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _q_rx(rad: float) -> tuple[float, float, float, float]:
    return (math.sin(rad / 2), 0.0, 0.0, math.cos(rad / 2))


def _q_ry(rad: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(rad / 2), 0.0, math.cos(rad / 2))


def _car() -> Body:
    """The front of the car, one fixed body; origin on the table under the glass's bottom edge, at y = 0."""
    o = np.array([GLASS_X0, 0.0, TOP])
    cy, wy = (GLASS_Y[0] + GLASS_Y[1]) / 2, GLASS_Y[1] - GLASS_Y[0]
    ca, sa = math.cos(SLOPE), math.sin(SLOPE)
    q = _q_ry(-SLOPE)  # turned so that +x rises along the slope

    def along(d: float, h: float) -> np.ndarray:
        """A point ``d`` up the slope from the glass's bottom edge and ``h`` above its surface (body frame, y = 0)."""
        return np.array([d * ca - h * sa, 0.0, (GLASS_Z0 - TOP) + d * sa + h * ca])

    shapes = []
    g = along(GLASS_LEN / 2, -GLASS_T / 2)
    shapes.append(BoxShape((g[0], cy, g[2]), (GLASS_LEN, wy, GLASS_T), GLASS_C, quat_xyzw=q))  # the windscreen
    f = along(0.012, 0.0004)
    shapes.append(BoxShape((f[0], cy, f[2]), (0.024, wy, 0.0008), COWL_C, quat_xyzw=q, collide=False))  # black frit
    t = along(GLASS_LEN - 0.010, 0.0004)
    shapes.append(BoxShape((t[0], cy, t[2]), (0.020, wy, 0.0008), COWL_C, quat_xyzw=q, collide=False))
    u = along(GLASS_LEN / 2, -0.030)  # the body under the glass (red), so the glass sits on the car
    shapes.append(BoxShape((u[0], cy, u[2]), (GLASS_LEN, wy - 0.02, 0.048), BODY_C, quat_xyzw=q))
    for side, y in ((-1.0, GLASS_Y[0] - 0.015), (1.0, GLASS_Y[1] + 0.015)):  # the pillars (red)
        p = along(GLASS_LEN / 2, 0.004)
        shapes.append(BoxShape((p[0], y, p[2]), (GLASS_LEN + 0.02, 0.030, 0.034), BODY_C, quat_xyzw=q))
    r = along(GLASS_LEN + 0.020, 0.004)  # the roof's front edge
    shapes.append(BoxShape((r[0], cy, r[2]), (0.045, wy + 0.06, 0.030), BODY_C, quat_xyzw=q))
    # the cowl: a black plastic panel in front of the glass, with grille slots; the body below it red
    shapes.append(BoxShape((-0.022, cy, (GLASS_Z0 - TOP) / 2), (0.044, wy + 0.06, GLASS_Z0 - TOP), COWL_C))
    for k in range(6):
        shapes.append(BoxShape((-0.022, GLASS_Y[0] + 0.04 + 0.085 * k, GLASS_Z0 - TOP + 0.0003),
                               (0.030, 0.060, 0.0006), (0.02, 0.02, 0.02), collide=False))
    shapes.append(BoxShape((-0.046, cy, (GLASS_Z0 - TOP - 0.01) / 2), (0.004, wy + 0.06, GLASS_Z0 - TOP - 0.01),
                           BODY_C))
    # a band of reflected sky across the upper glass, and the passenger's wiper parked higher up (shown only)
    b = along(GLASS_LEN * 0.72, 0.0004)
    shapes.append(BoxShape((b[0], cy, b[2]), (0.05, wy, 0.0008), (0.36, 0.47, 0.52), quat_xyzw=q, collide=False))
    px = 0.405
    pz = glass_z(px) - TOP
    shapes += [BoxShape((px - o[0], -0.140, pz + 0.032), (0.011, 0.230, 0.008), ARM_C, collide=False),
               BoxShape((px - o[0], -0.140, pz + 0.011), (0.011, 0.220, 0.013), FRAME_C, collide=False),
               BoxShape((px - o[0], -0.140, pz + 0.002), (0.003, 0.216, 0.004), RUBBER_C, collide=False),
               CylShape((px - o[0], -0.022, pz + 0.018), 0.016, 0.036, ARM_C, collide=False)]
    # the wiper's pivot: a black cap on the glass at the arm's root
    pv = PIVOT - o
    shapes.append(CylShape((pv[0], pv[1] + 0.004, (glass_z(BLADE_X) - TOP + pv[2]) / 2), 0.017,
                           pv[2] - (glass_z(BLADE_X) - TOP) + 0.010, ARM_C))
    return Body.build("car", pos=tuple(o.tolist()), mass=20.0, shapes=shapes, kinematic=True, color=BODY_C,
                      friction=(0.6, 0.5))


def _arm() -> Body:
    """The wiper arm: a black steel bar from the pivot (its origin) toward -y, its tip bent down into the hook."""
    L = ARM[1]
    shapes = [BoxShape((0.0, -L / 2, 0.0), ARM, ARM_C),
              BoxShape((0.0, -0.010, 0.006), (0.020, 0.030, 0.012), ARM_C),  # the root, under the pivot cap
              # the hook: the tip bent down to the blade's adapter (shown only)
              BoxShape((0.0, -L + 0.004, -0.008), (0.010, 0.008, 0.007), ARM_C, collide=False),
              BoxShape((0.0, -HOOK_AT - 0.005, HOOK_LOCAL[2] + FRAME[2] / 2 + 0.010), (0.010, 0.032, 0.004), ARM_C,
                       collide=False)]
    return Body.build("wiper_arm", pos=tuple(PIVOT.tolist()), mass=ARM_MASS, shapes=shapes, color=ARM_C,
                      friction=(1.2, 1.0), com=(0.0, -L / 2, 0.0), ignore=("car",))


def _blade(name: str, at, rubber, adapter) -> Body:
    """A wiper blade along y: a black steel frame on its rubber, its adapter (where it hooks on) on top at its middle,
    which is its origin."""
    shapes = [BoxShape((0.0, 0.0, 0.0), FRAME, FRAME_C),
              BoxShape((0.0, 0.0, -FRAME[2] / 2 - RUBBER_H / 2), (0.003, BLADE_LEN - 0.004, RUBBER_H), rubber),
              BoxShape((0.0, 0.0, FRAME[2] / 2 + 0.004), (0.014, 0.030, 0.008), adapter),
              BoxShape((0.0, BLADE_LEN / 2 - 0.006, FRAME[2] / 2 + 0.0015), (0.013, 0.012, 0.003), FRAME_C),
              BoxShape((0.0, -BLADE_LEN / 2 + 0.006, FRAME[2] / 2 + 0.0015), (0.013, 0.012, 0.003), FRAME_C)]
    return Body.build(name, pos=tuple(float(v) for v in at), mass=BLADE_MASS, shapes=shapes, color=FRAME_C,
                      friction=(1.3, 1.1), com=(0.0, 0.0, 0.0), ignore=("wiper_arm",), contact_offset=0.001,
                      damping=(0.2, 1.0))


def _card() -> Body:
    """The new blade's cardboard tray, with two cradles holding it upright."""
    shapes = [BoxShape((0.0, 0.0, CARD_T / 2), (0.050, 0.280, CARD_T), CARD_C)]
    for y in (0.065, -0.020):
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((s * (FRAME[0] / 2 + 0.0035), y, CARD_T + 0.007), (0.006, 0.020, 0.014), CARD_C))
    return Body.build("blade_tray", pos=(float(CARD[0]), float(CARD[1]), TOP), mass=0.1, shapes=shapes,
                      kinematic=True, color=CARD_C, friction=(0.6, 0.5))


def _scrap() -> Body:
    lx, ly, h = SCRAP_IN
    w = 0.004
    shapes = [BoxShape((0.0, 0.0, w / 2), (lx + 2 * w, ly + 2 * w, w), TRAY_C)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx / 2 + w / 2), 0.0, h / 2), (w, ly + 2 * w, h), TRAY_C))
        shapes.append(BoxShape((0.0, s * (ly / 2 + w / 2), h / 2), (lx, w, h), TRAY_C))
    return Body.build("scrap_tray", pos=(float(SCRAP[0]), float(SCRAP[1]), TOP), mass=0.3, shapes=shapes,
                      kinematic=True, color=TRAY_C, friction=(0.6, 0.5))


OBJECTS = [_car(), _arm(), _blade("old_blade", BLADE_AT, WORN_C, (0.30, 0.30, 0.32)),
           _blade("new_blade", NEW_AT, RUBBER_C, STEEL_C), _card(), _scrap()]
hinge("wiper_lift", "car", "wiper_arm", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-LIFT_MAX, DOWN_MAX),
      spring=SPRING, damping=0.00002, target_deg=SPRING_TARGET)
for _name, _on in (("old_blade", True), ("new_blade", False)):
    thread(f"{_name}_hook", "wiper_arm", _name, anchor0=tuple(HOOK_LOCAL.tolist()), axis0=(0.0, 1.0, 0.0),
           anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.002, 0.050), turn_damping=20.0,
           slide_stiffness=SOFT_K, slide_damping=2.0, slide_max_force=40.0, always_on=_on)

_orig_author_threads = bodykit._author_threads


def _author_threads(stage, prim_path, spec):
    """bodykit points a thread's slide drive at its far limit; start each hook's at 0 (the blade clicked in)."""
    _orig_author_threads(stage, prim_path, spec)
    from pxr import UsdPhysics
    for t in bodykit._THREADS:
        if t.body1 == spec.name:
            UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/{t.name}"), "transX").GetTargetPositionAttr().Set(0.0)


bodykit._author_threads = _author_threads

# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "hook": {"old_blade": {"on": True, "hold": 0.0, "clicked": True, "armed": False},
                                           "new_blade": {"on": False, "hold": 0.0, "clicked": False, "armed": True}},
            "slammed": False, "max_lift": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("WIPER_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_thread_on("old_blade_hook", True)
    set_slide_target("old_blade_hook", 0.0)
    set_thread_on("new_blade_hook", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _lift(rot) -> float:
    """How far the arm is lifted off its resting angle (deg): its tip (toward -y) rising."""
    d = rot @ np.array([0.0, -1.0, 0.0])
    return math.degrees(math.atan2(float(d[2]), -float(d[1])))


def _on_hook(arm_pos, arm_rot, b_pos, b_rot) -> tuple[float, float, float]:
    """(slide toward the pivot from the hook's end, distance off the hook's line, angle between blade and arm in deg)
    for a blade at ``b_pos``/``b_rot``."""
    p = arm_rot.T @ (np.asarray(b_pos) - arm_pos) - HOOK_LOCAL
    off = math.hypot(float(p[0]), float(p[2]))
    c = float(np.dot(arm_rot[:, 1], b_rot[:, 1]))
    return float(p[1]), off, math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _in_scrap(pos) -> bool:
    return (abs(float(pos[0]) - SCRAP[0]) <= SCRAP_IN[0] / 2 and abs(float(pos[1]) - SCRAP[1]) <= SCRAP_IN[1] / 2
            and float(pos[2]) < TOP + SCRAP_IN[2])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    st = {}
    for n in ("wiper_arm", "old_blade", "new_blade"):
        a = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        st[n] = (a[:3] - origin, rotation(a[3:7]))
    a_pos, a_rot = st["wiper_arm"]
    lift = _lift(a_rot)
    w = as_numpy(base.scene["wiper_arm"].data.root_com_vel_w)[0][3:6]
    speed = abs(float(np.dot(w, a_rot[:, 0])))
    if lift < SLAM_DEG and speed > SLAM_SPEED:
        _EP["slammed"] = True
    _EP["max_lift"] = max(_EP["max_lift"], lift)
    _EP["tcps"] = _tcps(base, origin)
    free = lift >= LIFT_FREE
    info = {}
    for n in ("old_blade", "new_blade"):
        h = _EP["hook"][n]
        other = _EP["hook"]["new_blade" if n == "old_blade" else "old_blade"]
        s, off, ang = _on_hook(a_pos, a_rot, *st[n])
        info[n] = (s, off, ang)
        joint = f"{n}_hook"
        if h["on"]:
            if s >= UNHOOK and free:  # slid off the hook
                set_thread_on(joint, False)
                h.update(on=False, clicked=False, armed=False)
                continue
            if free:
                if s > h["hold"] + HOLD_LAG:
                    h["hold"] = s - HOLD_LAG
                elif s < h["hold"] - HOLD_LAG:
                    h["hold"] = s + HOLD_LAG
                if s <= CLICK:
                    h["hold"] = 0.0
            set_slide_target(joint, max(h["hold"], 0.0))
            h["clicked"] = s <= CLICK
        elif not h["armed"]:
            h["armed"] = off > 2 * CATCH_OFF + 0.01 or not CATCH_S[0] - 0.02 <= s <= CATCH_S[1] + 0.02
        elif (free and not other["on"] and off <= CATCH_OFF and CATCH_S[0] <= s <= CATCH_S[1]
              and ang <= CATCH_DEG):  # hooked on
            h.update(on=True, hold=s, clicked=False)
            set_slide_target(joint, s)
            set_thread_on(joint, True)
    _EP["s"] = {"lift": lift, "speed": speed, "old_on": _EP["hook"]["old_blade"]["on"],
                "new_clicked": _EP["hook"]["new_blade"]["on"] and _EP["hook"]["new_blade"]["clicked"],
                "old_in_scrap": _in_scrap(st["old_blade"][0]) and not _EP["hook"]["old_blade"]["on"]}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            o, nb = st["old_blade"][0], st["new_blade"][0]
            vel = []
            for n in ("wiper_arm", "old_blade", "new_blade"):
                v = as_numpy(base.scene[n].data.root_com_vel_w)[0]
                vel.append((round(float(np.linalg.norm(v[:3])), 3), round(float(np.linalg.norm(v[3:])), 2)))
            ho, hn = _EP["hook"]["old_blade"], _EP["hook"]["new_blade"]
            print(f"[wiper] t={_DEBUG['steps'] / 20:5.1f}s lift={lift:5.1f}deg w={speed:4.2f} slam={_EP['slammed']} "
                  f"old={np.round(o * 1000).astype(int).tolist()} on={ho['on']} clk={ho['clicked']} "
                  f"s/off/ang={info['old_blade'][0] * 1000:.0f}/{info['old_blade'][1] * 1000:.0f}/{info['old_blade'][2]:.0f} "
                  f"new={np.round(nb * 1000).astype(int).tolist()} on={hn['on']} clk={hn['clicked']} "
                  f"s/off/ang={info['new_blade'][0] * 1000:.0f}/{info['new_blade'][1] * 1000:.0f}/{info['new_blade'][2]:.0f} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"score={_points(_status())} v={vel}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    s = _EP["s"]
    return {"old_in_scrap": bool(s.get("old_in_scrap")), "new_clicked": bool(s.get("new_clicked")),
            "down": s.get("lift", 90.0) <= DOWN, "slammed": bool(_EP.get("slammed")), "home": _arms_home()}


def _points(s) -> float:
    if s["slammed"]:
        return 0.0
    if s["old_in_scrap"] and s["new_clicked"] and s["down"] and s["home"]:
        return 1.0
    if s["old_in_scrap"] and s["new_clicked"]:
        return 0.5
    return 0.25 if s["old_in_scrap"] else 0.0


def score(objects) -> float:
    """0.0 if the wiper arm ever snapped down onto the glass (reaching within 4 degrees of resting on it faster than
    1.5 rad/s). Otherwise 1.0 with the old blade off the arm and in the scrap tray, the new blade hooked on the arm and
    clicked in (within 4 mm of the hook's end), the arm down on the glass (within 3 degrees of resting on it) and both
    arms back where they started (each TCP within 5 cm of its start); 0.5 with the old blade in the scrap tray and the
    new one clicked in; 0.25 with the old blade in the scrap tray; else 0.0."""
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
    """Full score with the wiper arm and both blades still (under 1 cm/s; a blade pressed onto the glass by the arm
    jitters about its own axis, so up to 1 rad/s counts as still)."""
    still = all(objects[n].resting(max_spin=1.0) for n in ("wiper_arm", "old_blade", "new_blade"))
    return score(objects) >= 1.0 and still


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the wiper arm 9 cm from its pivot, fingers closing across it, and lifts it 15
# degrees off the glass along its swing, then holds it there. The right hand comes down on the old blade 8 cm beyond
# its middle (past the arm's tip), fingers closing across it, slides it 3 cm along the arm toward the pivot, off the
# hook, draws it back toward the arms, carries it over the scrap tray and lets it drop in. It takes the new blade out
# of its cardboard tray the same way, carries it to the arm, hooks it on 1.6 cm short of the hook's end and slides it
# along the arm toward the tip until it clicks, lets go and goes back to where it started. The left lowers the arm
# gently onto the glass, lets go and goes back to where it started.

LGRIP = grip_rotation(-math.pi / 2)  # left: straight down, fingers closing along x, across the arm
RGRIP = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x, across a blade
ARM_HOLD = 0.070  # the left holds the arm this far from its pivot
GRIP_ON_BLADE = -0.080  # the right holds a blade this far from its middle (toward its far end, past the arm's tip)
ABOVE_ARM = ARM[2] / 2 + 0.004  # TCP above the arm's middle line
ABOVE_BLADE = FRAME[2] / 2 + 0.002  # TCP above a blade's middle line (its frame's top + 4 mm)
LIFT_CMD = 15.0  # the left aims for this lift (deg) ...
LIFT_SAG = 0.0155  # ... aiming this much higher (the arm sags under the spring)
LIFT_DEMO = 15.0  # the lift the right hand plans for
OPEN_ARM = -0.013
OPEN_BLADE = -0.013
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


def _arm_point(dist: float, lift: float, up: float = 0.0) -> np.ndarray:
    """A point on the arm's middle line ``dist`` from the pivot, the arm lifted ``lift`` degrees, ``up`` above it."""
    r = _rx(-lift)
    return PIVOT + r @ np.array([0.0, -dist, 0.0]) + np.array([0.0, 0.0, up])


def _blade_point(slide: float, along: float, lift: float, up: float = ABOVE_BLADE) -> np.ndarray:
    """A point on a hooked blade's middle line ``along`` from its middle (toward the pivot), the blade slid ``slide``
    toward the pivot from the hook's end and the arm lifted ``lift`` degrees; ``up`` straight above it."""
    r = _rx(-lift)
    return PIVOT + r @ (HOOK_LOCAL + np.array([0.0, slide + along, 0.0])) + np.array([0.0, 0.0, up])


def _left(hold_until: float, total: float) -> tuple[list[Waypoint], float]:
    grip = _arm_point(ARM_HOLD, 0.0, ABOVE_ARM)
    out = _lead_in("left", grip + [0.0, 0.0, 0.04], LGRIP, "over the wiper arm", OPEN_ARM)
    out += [_w(0.8, grip, OPEN_ARM, "down round the arm", LGRIP),
            _w(0.5, grip, FINGER_CLOSED, "take hold of it", LGRIP)]
    for k in range(1, 7):
        a = LIFT_CMD * k / 6
        out.append(_w(0.35, _arm_point(ARM_HOLD, a, ABOVE_ARM + LIFT_SAG * k / 6), FINGER_CLOSED,
                      "lift it off the glass along its swing", LGRIP))
    lifted = sum(w.duration for w in out)
    held = _arm_point(ARM_HOLD, LIFT_CMD, ABOVE_ARM + LIFT_SAG)
    out.append(_w(max(hold_until - lifted, 0.5), held, FINGER_CLOSED, "hold it up while the blade is changed", LGRIP))
    for k in range(1, 9):
        a = LIFT_CMD * (1 - k / 8) - 1.0 * k / 8
        out.append(_w(0.35, _arm_point(ARM_HOLD, a, ABOVE_ARM + LIFT_SAG * (1 - k / 8)), FINGER_CLOSED,
                      "lower it gently onto the glass", LGRIP))
    down = _arm_point(ARM_HOLD, -1.0, ABOVE_ARM)
    out += [_w(0.5, down, OPEN_ARM, "let go", LGRIP),
            _w(0.6, down + [0.0, 0.0, 0.05], OPEN_ARM, "up", LGRIP)]
    out += _go_home("left", down + [0.0, 0.0, 0.05], LGRIP, OPEN_ARM)
    t = sum(w.duration for w in out)
    out.append(Waypoint(max(total - t, 0.5), out[-1].pos, OPEN_ARM,
                        "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, lifted


def _right(lifted_at: float) -> tuple[list[Waypoint], float]:
    L = LIFT_DEMO
    old = _blade_point(0.0, GRIP_ON_BLADE, L)
    back = np.array([-0.08, 0.0, 0.05])  # drawn back toward the arms and up, clear of the arm
    out = _lead_in("right", old + back, RGRIP, "near the old blade", OPEN_BLADE)
    t = sum(w.duration for w in out)
    out += [_w(max(lifted_at + 0.5 - t, 0.3), old + back, OPEN_BLADE, "wait for the arm to come up", RGRIP),
            _w(1.0, old + [0.0, 0.0, 0.03], OPEN_BLADE, "over the old blade, past the arm's tip", RGRIP),
            _w(0.8, old, OPEN_BLADE, "down round it", RGRIP),
            _w(0.5, old, FINGER_CLOSED, "take it", RGRIP)]
    for k in range(1, 5):
        out.append(_w(0.35, _blade_point(0.036 * k / 4, GRIP_ON_BLADE, L), FINGER_CLOSED,
                      "slide it along the arm toward the pivot, off the hook", RGRIP))
    off = _blade_point(0.036, GRIP_ON_BLADE, L)
    # the blade's middle over the tray's middle: the grip is 8 cm toward its far end (-y)
    over = np.array([SCRAP[0], SCRAP[1] + GRIP_ON_BLADE, TOP + 0.090])
    out += [_w(0.9, off + back, FINGER_CLOSED, "draw it back off the arm", RGRIP),
            _w(1.8, over, FINGER_CLOSED, "carry it over the scrap tray", RGRIP),
            _w(0.8, over - [0.0, 0.0, 0.030], FINGER_CLOSED, "lower it", RGRIP),
            _w(0.5, over - [0.0, 0.0, 0.030], OPEN_BLADE, "let it drop in", RGRIP)]
    new = NEW_AT + np.array([0.0, GRIP_ON_BLADE, ABOVE_BLADE])
    out += [_w(0.8, over + [0.0, 0.0, 0.02], OPEN_BLADE, "up", RGRIP),
            _w(1.4, new + [0.0, 0.0, 0.05], OPEN_BLADE, "over the new blade", RGRIP),
            _w(0.9, new, OPEN_BLADE, "down round it", RGRIP),
            _w(0.5, new, FINGER_CLOSED, "take it", RGRIP),
            _w(1.0, new + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it out of its tray", RGRIP)]
    start = _blade_point(0.011, GRIP_ON_BLADE, L)
    out += [_w(1.8, start + back, FINGER_CLOSED, "carry it to the arm", RGRIP),
            _w(1.0, start, FINGER_CLOSED, "onto the hook", RGRIP)]
    for k in range(1, 5):
        out.append(_w(0.35, _blade_point(0.011 - 0.028 * k / 4, GRIP_ON_BLADE, L), FINGER_CLOSED,
                      "slide it along the arm toward the tip until it clicks", RGRIP))
    end = _blade_point(-0.017, GRIP_ON_BLADE, L)
    out += [_w(0.5, end, OPEN_BLADE, "let go", RGRIP),
            _w(0.8, end + back, OPEN_BLADE, "draw back", RGRIP)]
    done = sum(w.duration for w in out)
    out += _go_home("right", end + back, RGRIP, OPEN_BLADE)
    return out, done


def demo():
    _, lifted = _left(0.0, 0.0)
    right, done = _right(lifted)
    total = max(sum(w.duration for w in right), done + 0.5 + 8 * 0.35 + 1.1 + LEAD_S) + 3.0
    left, _ = _left(done + 0.5, total)
    right.append(Waypoint(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, OPEN_BLADE,
                          "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
