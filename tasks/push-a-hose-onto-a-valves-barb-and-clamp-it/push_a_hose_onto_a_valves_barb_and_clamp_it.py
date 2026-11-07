"""On the station table lies a light grey PVC ball valve, loose, its red lever pointing toward the left arm and its
barbed hose tail toward the right arm. In line with the barb, in two notched wooden rests, lies a length of green garden
hose, a steel worm-drive hose clamp loose on it about 9 cm from its end.

Task: fit the hose to the valve. The valve weighs 80 g and slides on the table at a push, so one hand holds it by its
lever while the other pushes the hose's end over the barb (the barb grips the hose harder than the table holds the
valve: it takes about 3 N to push the hose further on) at least 2 cm, then slides the clamp along the hose until it
sits over the barb, which locks the hose on. Then the hand holding the valve lifts it by its lever to show the hose
comes with it, sets it back down and lets go, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=push_a_hose_onto_a_valves_barb_and_clamp_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_slider_drive, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the valve: lying loose, its pipe axis along y ----------------------------------------------------------------------
VALVE_XY = (0.32, 0.03)
AXIS_Z = 0.015  # its pipe axis, above the table
LEVER_Z = 2 * AXIS_Z + 0.013  # the lever's middle, above the table
BARB = (-0.043, -0.071)  # the barb: from its shoulder to its tip, along y (valve frame)
BARB_R = 0.0062
TIP = np.array([0.0, BARB[1], AXIS_Z])  # the barb's tip, in the valve's frame
GRIP_Y = 0.0875  # the lever's grip, from the valve's middle
VALVE_MASS = 0.08

# -- the hose and its clamp -----------------------------------------------------------------------------------------
HOSE_R, HOSE_L = 0.008, 0.30
HOSE_Z = AXIS_Z - 0.0045  # its axis at the start: lying in two notched wooden rests, a little below the barb
RESTS = (0.115, 0.25)  # the rests, this far from the hose's end (behind the clamp, which moves away from them)
END0 = np.array([VALVE_XY[0], VALVE_XY[1] + BARB[1] - 0.035, TOP + HOSE_Z])  # the hose's end at the start
CLAMP_BACK = 0.09  # the clamp starts this far from the hose's end
BAND_R, BAND_W = 0.0100, 0.009
ON_DEPTH = 0.020  # the hose is on once pushed this far over the barb (of 28 mm)
ZONE = (0.004, 0.024)  # the clamp is over the barb with its middle this far from the hose's end
K_HOSE, SLIP_H, D_HOSE = 3000.0, 0.001, 4.0  # the barb's grip on the hose (about 3 N)
K_CLAMP, SLIP_C, D_CLAMP = 1500.0, 0.001, 3.0  # the clamp's grip on the hose (about 1.5 N)
K_LOCKED = 30000.0  # clamped: the hose stays on
CAPTURE_R, CAPTURE_DEG = 0.008, 12.0

# -- scoring ------------------------------------------------------------------------------------------------------------
LIFTED = 0.03  # the valve shown lifted: its bottom this high off the table, the hose on it
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

WOOD = (0.70, 0.52, 0.32)
PVC, PVC_DARK, RED, RED_DARK, GREEN, YELLOW, STEEL, DARK = (0.62, 0.64, 0.66), (0.50, 0.52, 0.54), \
    (0.82, 0.12, 0.10), (0.62, 0.08, 0.07), (0.16, 0.52, 0.22), (0.92, 0.80, 0.20), (0.76, 0.77, 0.79), \
    (0.10, 0.10, 0.10)


def _valve() -> Body:
    """The valve; origin on the table under its middle."""
    z = AXIS_Z
    shapes = [BoxShape((0.0, 0.0, z), (0.036, 0.046, 2 * z), PVC),  # the body
              CylShape((0.0, -0.029, z), z - 0.0005, 0.012, PVC_DARK, axis="Y", collide=False),  # union nuts
              CylShape((0.0, 0.029, z), z - 0.0005, 0.012, PVC_DARK, axis="Y", collide=False),
              CylShape((0.0, (-0.035 + BARB[0]) / 2, z), 0.011, 0.008, PVC, axis="Y"),  # the barb's shoulder
              CylShape((0.0, (BARB[0] + BARB[1]) / 2, z), BARB_R, BARB[0] - BARB[1], PVC, axis="Y"),  # the barb
              CylShape((0.0, 0.040, z), 0.011, 0.010, PVC, axis="Y"),  # the threaded end
              BoxShape((0.0, 0.0, 2 * z + 0.005), (0.008, 0.008, 0.010), PVC_DARK),  # the stem
              BoxShape((0.0, 0.050, LEVER_Z), (0.014, 0.11, 0.006), RED),  # the lever
              BoxShape((0.0, GRIP_Y, LEVER_Z), (0.018, 0.035, 0.012), RED_DARK)]  # its grip
    for k in range(3):  # the barb's ridges
        shapes.append(CylShape((0.0, BARB[1] + 0.004 + k * 0.009, z), BARB_R + 0.0009, 0.003, PVC, axis="Y",
                               collide=False))
    return Body.build("valve", pos=(VALVE_XY[0], VALVE_XY[1], TOP), mass=VALVE_MASS, shapes=shapes,
                      friction=(0.7, 0.6), color=PVC, com=(0.0, 0.01, z))


def _hose() -> Body:
    """The hose; origin on its axis at the end that goes on the barb (it runs toward -y from there)."""
    return Body.build("hose", pos=tuple(END0.tolist()), mass=0.03, friction=(1.0, 0.9), color=GREEN,
                      ignore=("valve",), com=(0.0, -HOSE_L / 2, 0.0), shapes=[
                          CylShape((0.0, -HOSE_L / 2, 0.0), HOSE_R, HOSE_L, GREEN, axis="Y"),
                          CylShape((0.0, 0.0003, 0.0), 0.0052, 0.0006, DARK, axis="Y", collide=False),  # its bore
                          BoxShape((0.0, -HOSE_L / 2 - 0.01, HOSE_R), (0.0016, HOSE_L - 0.04, 0.0006), YELLOW,
                                   collide=False)])  # a stripe


def _rests() -> Body:
    """Two small wooden rests with a notch the hose lies in (so it cannot roll); origin on the table."""
    shapes = []
    base_h = HOSE_Z - HOSE_R  # up to the hose's underside
    for d in RESTS:
        y = END0[1] - d
        shapes += [BoxShape((END0[0], y, base_h / 2), (0.04, 0.012, base_h), WOOD),
                   BoxShape((END0[0] - 0.0125, y, base_h + 0.0015), (0.008, 0.012, 0.003), WOOD),
                   BoxShape((END0[0] + 0.0125, y, base_h + 0.0015), (0.008, 0.012, 0.003), WOOD)]
    return Body.build("rests", pos=(0.0, 0.0, TOP), mass=0.2, kinematic=True, color=WOOD, shapes=shapes)


def _clamp() -> Body:
    """The worm-drive clamp, round the hose; origin on the hose's axis in its middle."""
    return Body.build("clamp", pos=(END0[0], END0[1] - CLAMP_BACK, END0[2]), mass=0.008, friction=(0.8, 0.7),
                      color=STEEL, ignore=("valve",), shapes=[  # (its band is drawn solid: it slides over the barb)
                          CylShape((0.0, 0.0, 0.0), BAND_R, BAND_W, STEEL, axis="Y"),
                          BoxShape((0.0, 0.0, BAND_R + 0.006), (0.014, 0.012, 0.012), STEEL),  # the screw housing
                          CylShape((0.0, 0.0, BAND_R + 0.0125), 0.003, 0.018, (0.55, 0.56, 0.58), axis="X",
                                   collide=False)])


OBJECTS = [_rests(), _valve(), _hose(), _clamp()]
slider("clamp_slide", "hose", "clamp", anchor=(END0[0], END0[1] - CLAMP_BACK, END0[2]), axis="Y",
       limits=(-0.03, CLAMP_BACK), stiffness=K_CLAMP, damping=D_CLAMP, target=0.0)
thread("hose_on", "valve", "hose", anchor0=tuple(TIP.tolist()), axis0=(0.0, 1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 1.0, 0.0), slide_limits=(-0.05, BARB[0] - BARB[1]), turn_damping=0.5, slide_stiffness=K_HOSE,
       slide_damping=D_HOSE, slide_max_force=60.0, play=0.002, tilt_deg=4.0)


def _set_hose_stiffness(k: float) -> None:
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(bodykit._THREAD_PATHS["hose_on"])
    UsdPhysics.DriveAPI(prim, "transX").GetStiffnessAttr().Set(float(k))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold_h": 0.0, "hold_c": 0.0, "clamped": False, "shown": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("HOSE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _set_hose_stiffness(K_HOSE)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    v = as_numpy(base.scene["valve"].data.root_link_pose_w)[0]
    h = as_numpy(base.scene["hose"].data.root_link_pose_w)[0]
    c = as_numpy(base.scene["clamp"].data.root_link_pose_w)[0]
    v_pos, v_rot, h_pos, h_rot, c_pos = v[:3] - origin, rotation(v[3:7]), h[:3] - origin, rotation(h[3:7]), c[:3] - origin
    q = v_rot.T @ (h_pos - v_pos) - TIP  # the hose's end, from the barb's tip
    off, depth = math.hypot(q[0], q[2]), float(q[1])
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(v_rot[:, 1] @ h_rot[:, 1])))))
    back = float(h_rot[:, 1] @ (h_pos - c_pos))  # the clamp's middle, behind the hose's end
    slid = CLAMP_BACK - back
    if abs(slid - _EP["hold_c"]) > SLIP_C:  # the clamp pushed along the hose: it slides, and stays where left
        _EP["hold_c"] = slid
        set_slider_drive("clamp_slide", target=slid)
    if not _EP["on"]:
        if off <= CAPTURE_R and 0.0 <= depth <= 0.008 and angle <= CAPTURE_DEG:  # the hose's end over the barb
            set_slide_target("hose_on", depth)
            set_thread_on("hose_on", True)
            _EP["on"], _EP["hold_h"] = True, depth
    elif not _EP["clamped"]:
        if depth < -0.003:  # pulled back off
            set_thread_on("hose_on", False)
            _EP["on"] = False
        elif abs(depth - _EP["hold_h"]) > SLIP_H:  # pushed further on (or pulled): it slides, and stays
            _EP["hold_h"] = depth
            set_slide_target("hose_on", depth)
    seated = _EP["on"] and depth >= ON_DEPTH
    if not _EP["clamped"] and seated and ZONE[0] <= back <= ZONE[1]:  # the clamp over the barb: the hose is held
        _EP["clamped"] = True
        set_slide_target("hose_on", depth)
        _set_hose_stiffness(K_LOCKED)
    lift = float(v_pos[2] - TOP)
    if _EP["clamped"] and lift >= LIFTED:
        _EP["shown"] = True
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    lever = v_pos + v_rot @ np.array([0.0, GRIP_Y, LEVER_Z])
    clear = all(min(float(np.linalg.norm(t - lever)), float(np.linalg.norm(t - h_pos)),
                    float(np.linalg.norm(t - c_pos))) >= LET_GO for t in tcps)
    down = abs(lift) < 0.004 and float(v_rot[2, 2]) > math.cos(math.radians(10.0))
    _EP["s"] = {"on": seated, "clamped": _EP["clamped"] and seated and ZONE[0] <= back <= ZONE[1], "down": down,
                "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[hose] t={_DEBUG['steps'] / 20:5.1f}s depth={depth * 1000:6.1f}mm off={off * 1000:4.1f}mm "
                  f"q={np.round(q * 1000, 1).tolist()} angle={angle:4.1f} on={_EP['on']} clamp_back={back * 1000:5.1f}mm "
                  f"clamped={_EP['clamped']} lift={lift * 1000:5.1f}mm shown={_EP['shown']} down={down} clear={clear} "
                  f"valve={np.round(v_pos * 1000).astype(int).tolist()} leftTCP-lever={np.round((tl - lever) * 1000).astype(int).tolist()} "
                  f"rightTCP-hoseEnd={np.round((tr - h_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-clamp={np.round((tr - c_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"on": bool(st.get("on")), "clamped": bool(st.get("clamped")), "shown": _EP["shown"],
            "down": bool(st.get("down")), "clear": bool(st.get("clear")), "home": _arms_home()}


def _points(s) -> float:
    if s["clamped"] and s["shown"] and s["down"] and s["clear"] and s["home"]:
        return 1.0
    if s["clamped"]:
        return 0.5
    return 0.25 if s["on"] else 0.0


def score(objects) -> float:
    """1.0 with the hose pushed at least 2 cm onto the barb and the clamp over the barb (its middle 4 to 24 mm from the
    hose's end), the valve lifted at least 3 cm with the hose on it at some point after that, then back flat on the
    table and let go (each TCP at least 4 cm from the valve's lever, the hose's end and the clamp) and both arms
    back where they started (each TCP within 5 cm of its start); 0.5 with the hose on and the clamp over the barb;
    0.25 with the hose at least 2 cm onto the barb; else 0.0. All but the lift are judged at the end."""
    return _points(_status())


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
    """Full score with the valve, the hose and the clamp still."""
    return (score(objects) >= 1.0 and objects["valve"].resting() and objects["hose"].resting()
            and objects["clamp"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the valve's lever grip, fingers closing across it, and holds the valve. The
# right takes the hose near its end, lifts it out of its rests and pushes it over the barb, 2.6 cm, lets go and
# rises. It comes down round the hose behind the clamp, closes its fingers round the hose and pushes the clamp along it
# until it sits over the barb, then
# rises and goes back to where it started. The left lifts the valve 6 cm by its lever, the hose coming with it, holds
# it up, sets it back down, lets go and goes back to where it started.

L_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the lever)
R_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (across the hose)
LEVER_IN = 0.006  # the left TCP this far below the lever grip's top
LEVER_OPEN = -0.015  # the left's fingers opened only ~5 mm wider than the grip each side: closing, they barely shove it
HOSE_OPEN = -0.011
HOSE_UP = 0.006  # the right TCP this far above the hose's axis
HOSE_HELD = 0.06  # the right takes the hose this far from its end (its fingers clear of the valve as it pushes)
DEPTH = 0.026  # pushed this far over the barb
PUSH_PAST = 0.025  # the arm gives against the barb's grip and the valve gives in the other hand
PUSH_Z = 0.004  # the right pushes the clamp with its fingers round the hose behind it: TCP this far above the axis
FORK = -0.009  # ... closed to 18 mm: round the 16 mm hose, but not past the clamp's 20 mm band
CONTACT = 0.012  # the right TCP is about this far behind the clamp's middle when its fingers push it
CLAMP_TO = -0.004  # the clamp aimed this far from the hose's end (it stops about 2 cm short: the arm gives)
LIFT = 0.06
LEAD_S, LEAD_STEPS = 3.0, 15
HOSE_OFF = np.array([-0.010, 0.0, 0.0])  # where the held hose's end sits off its planned place (measured: the
# fingers closing on it swing it a little about its rests; aimed against)


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


def _left(lift_at: float, total: float) -> tuple[list[Waypoint], float]:
    grip = np.array([VALVE_XY[0], VALVE_XY[1] + GRIP_Y, TOP + LEVER_Z + 0.006 - LEVER_IN])
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], L_GRIP, "over the valve's lever", LEVER_OPEN)
    out += [_w(1.0, grip, LEVER_OPEN, "down round the lever's grip", L_GRIP),
            _w(0.5, grip, FINGER_CLOSED, "hold the valve by its lever", L_GRIP)]
    held_at = sum(w.duration for w in out)
    up = grip + [0.0, 0.0, LIFT]
    out += [_w(max(lift_at - held_at, 0.3), grip, FINGER_CLOSED, "hold the valve still", L_GRIP),
            _w(1.8, up, FINGER_CLOSED, "lift the valve: the hose comes with it", L_GRIP),
            _w(1.2, up, FINGER_CLOSED, "hold it up", L_GRIP),
            _w(1.8, grip + [0.0, 0.0, 0.002], FINGER_CLOSED, "set it back down", L_GRIP),
            _w(0.5, grip + [0.0, 0.0, 0.002], FINGER_OPEN, "let go", L_GRIP),
            _w(0.8, grip + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", L_GRIP)]
    out += _go_home("left", grip + [0.0, 0.0, 0.05], L_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    take = END0 + [0.0, -HOSE_HELD, HOSE_UP]
    tip = np.array([VALVE_XY[0], VALVE_XY[1], TOP]) + TIP
    line = np.array([tip[0], tip[1] - 0.008 - HOSE_HELD, tip[2] + HOSE_UP]) - HOSE_OFF  # the end just short of the tip
    on = line + [0.0, 0.008 + DEPTH + PUSH_PAST, 0.0]
    out = _lead_in("right", take + [0.0, 0.0, 0.04], R_GRIP, "over the hose", HOSE_OPEN)
    out.append(_w(max(start - LEAD_S, 0.3), take + [0.0, 0.0, 0.04], HOSE_OPEN, "wait for the valve to be held",
                  R_GRIP))
    out += [_w(0.8, take, HOSE_OPEN, "down round the hose, near its end", R_GRIP),
            _w(0.5, take, FINGER_CLOSED, "take it", R_GRIP),
            _w(0.6, take + [0.0, 0.0, 0.010], FINGER_CLOSED, "lift it out of its rests", R_GRIP),
            _w(1.0, line, FINGER_CLOSED, "line its end up with the barb's tip", R_GRIP),
            _w(2.5, on, FINGER_CLOSED, "push the hose over the barb", R_GRIP),
            _w(0.3, on, FINGER_CLOSED, "hold", R_GRIP),
            _w(0.4, on, FINGER_OPEN, "let go", R_GRIP),
            _w(0.6, on + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", R_GRIP)]
    end = np.array([tip[0], tip[1] + DEPTH, tip[2]])  # the hose's end, on the barb
    clamp_at = end - [0.0, CLAMP_BACK, 0.0]
    behind = clamp_at + [0.0, -CONTACT - 0.015, PUSH_Z]
    pushed = end + [0.0, -CLAMP_TO - CONTACT, PUSH_Z]
    out += [_w(1.2, behind + [0.0, 0.0, 0.04], -0.02, "over the hose, behind the clamp", R_GRIP),
            _w(0.8, behind, -0.02, "down round the hose behind the clamp", R_GRIP),
            _w(0.4, behind, FORK, "close the fingers round the hose", R_GRIP),
            _w(2.5, pushed, FORK, "push the clamp along the hose, over the barb", R_GRIP),
            _w(0.4, pushed, FINGER_OPEN, "let go", R_GRIP),
            _w(0.6, pushed + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", R_GRIP)]
    out += _go_home("right", pushed + [0.0, 0.0, 0.05], R_GRIP, FINGER_OPEN)
    done = sum(w.duration for w in out)
    return out, done


def demo():
    _, held_at = _left(0.0, 0.0)
    right, done = _right(held_at)
    left, _ = _left(done - 1.0, 0.0)
    total = max(sum(w.duration for w in left), done) + 3.0
    left, _ = _left(done - 1.0, total)
    right.append(_w(total - done, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
