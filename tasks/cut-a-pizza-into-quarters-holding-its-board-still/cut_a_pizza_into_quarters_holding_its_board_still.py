"""On a polished grey quartz worktop lies a round beech pizza board with a pepperoni pizza on it, its short handle to the
left. A pizza wheel rests in its wooden stand to the right.

Task: cut the pizza into quarters. Hold the board still by its handle with one hand; with the other take the pizza
wheel from its stand and run it across the pizza twice, once from back to front through the middle and once from right
to left through the middle, pressing down so it cuts. Then put the pizza wheel back in its stand and take both arms back
to where they started.

Why two hands: the worktop is polished and the board slides on it at about 0.5 N, while the wheel pressed onto the
pizza drags it along with a few newtons. Run across the pizza with nothing holding the board, the board slides along
with the wheel, and the wheel only cuts where it moves against a still board.

Run it:  inspect-robots run --embodiment yam_arena -E task=cut_a_pizza_into_quarters_holding_its_board_still.py \\
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
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the worktop ------------------------------------------------------------------------------------------------------
WX0, WX1, WY0, WY1, WT_H = 0.150, 0.480, -0.290, 0.200, 0.010
CT = TOP + WT_H
# -- the board: origin at the middle of its underside (on its feet) --------------------------------------------------
BX, BY = 0.330, -0.060
R_BD, T_BD, FEET = 0.105, 0.022, 0.006
BD_TOP = FEET + T_BD  # the board's top, above the worktop
HANDLE_A = math.pi / 2  # the handle sticks out to the left
HANDLE_L, HANDLE_W = 0.048, 0.024
HANDLE_R = R_BD + HANDLE_L / 2 - 0.006
# -- the pizza on the board -------------------------------------------------------------------------------------------
R_PZ, T_PZ = 0.085, 0.006
PZ_TOP = BD_TOP + T_PZ
SEG_N, SEG_STEP = 8, 0.020  # each cut line: 8 segments, 2 cm apart (from -7 to +7 cm)
SEG_TOL, SEG_ACROSS = 0.012, 0.014  # a segment is cut when the wheel passes within these of its middle (along, across)
# -- the pizza wheel: origin at the middle of its handle --------------------------------------------------------------
H_L, H_W = 0.100, 0.016
WHEEL_X, WHEEL_Z, WHEEL_R = 0.078, -0.022, 0.032  # the wheel's middle from the handle's middle (local frame)
TIP_Z = WHEEL_Z - WHEEL_R  # the wheel's edge below the handle's middle
# -- its stand --------------------------------------------------------------------------------------------------------
SX, SY, S_BASE = 0.295, -0.225, 0.010
# -- the rules -------------------------------------------------------------------------------------------------------
PRESS_Z = 0.005  # the wheel's edge within this of the pizza's top: pressing, so it cuts
STILL_MM, STILL_DEG = 1.5, 0.75  # a step's cutting counts only if the board moved less than this in that step

QUARTZ, QUARTZ_V = (0.30, 0.31, 0.33), (0.45, 0.46, 0.48)
BEECH, BEECH_D = (0.82, 0.64, 0.42), (0.66, 0.49, 0.30)
CRUST, CRUST_D, SAUCE, CHEESE = (0.86, 0.62, 0.30), (0.62, 0.38, 0.16), (0.74, 0.17, 0.09), (0.97, 0.86, 0.52)
PEPPERONI, BASIL, CUT = (0.66, 0.16, 0.10), (0.20, 0.50, 0.18), (0.40, 0.22, 0.10)
STEEL, ABS = (0.80, 0.81, 0.83), (0.10, 0.10, 0.11)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _worktop() -> Body:
    shapes = [BoxShape((0.0, 0.0, WT_H / 2), (WX1 - WX0, WY1 - WY0, WT_H), QUARTZ)]
    rng = np.random.default_rng(8)
    for _ in range(45):  # pale flecks in the polished quartz
        shapes.append(BoxShape((rng.uniform(-0.16, 0.16), rng.uniform(-0.24, 0.24), WT_H + 0.0002),
                               (rng.uniform(0.004, 0.010), rng.uniform(0.003, 0.007), 0.0004), QUARTZ_V,
                               collide=False, yaw=rng.uniform(0, math.pi)))
    return Body.build("worktop", pos=((WX0 + WX1) / 2, (WY0 + WY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=QUARTZ, shapes=shapes, friction=(0.10, 0.08), slick=True)


_SHAPE: dict = {"cuts": []}


def _segments():
    """The cut lines' segments in the board's frame: (line, x, y, yaw)."""
    out = []
    for line, (ux, uy) in enumerate(((1.0, 0.0), (0.0, 1.0))):
        for k in range(SEG_N):
            s = (k - (SEG_N - 1) / 2) * SEG_STEP
            out.append((line, s * ux, s * uy, math.atan2(uy, ux)))
    return out


def _board() -> Body:
    """A round beech board with a short handle and four feet, a pepperoni pizza on it, and its cut lines (hidden)."""
    shapes = [CylShape((0.0, 0.0, FEET + T_BD / 2), R_BD, T_BD, BEECH),
              CylShape((0.0, 0.0, FEET + T_BD - 0.0006), R_BD - 0.003, 0.0016, BEECH_D, collide=False)]
    shapes[-1] = CylShape((0.0, 0.0, BD_TOP + 0.0002), R_BD - 0.004, 0.0004, BEECH, collide=False)
    shapes.append(BoxShape((HANDLE_R * math.cos(HANDLE_A), HANDLE_R * math.sin(HANDLE_A), FEET + T_BD / 2),
                           (HANDLE_L, HANDLE_W, T_BD), BEECH, yaw=HANDLE_A))
    hx, hy = (HANDLE_R + HANDLE_L / 2 - 0.010) * math.cos(HANDLE_A), (HANDLE_R + HANDLE_L / 2 - 0.010) * math.sin(HANDLE_A)
    shapes.append(CylShape((hx, hy, FEET + T_BD + 0.0003), 0.004, 0.0008, BEECH_D, collide=False))  # its hanging hole
    for k in range(4):  # rubber feet
        a = math.pi / 4 + k * math.pi / 2
        shapes.append(CylShape((0.070 * math.cos(a), 0.070 * math.sin(a), FEET / 2), 0.010, FEET, (0.15, 0.15, 0.15)))
    shapes.append(CylShape((0.0, 0.0, BD_TOP + T_PZ / 2), R_PZ, T_PZ, CRUST))  # the pizza (crust edge)
    shapes.append(CylShape((0.0, 0.0, BD_TOP + T_PZ - 0.0012), R_PZ - 0.004, 0.0028, CRUST_D, collide=False))
    shapes.append(CylShape((0.0, 0.0, PZ_TOP + 0.0003), R_PZ - 0.010, 0.0006, SAUCE, collide=False))
    shapes.append(CylShape((0.0, 0.0, PZ_TOP + 0.0007), R_PZ - 0.014, 0.0006, CHEESE, collide=False))
    rng = np.random.default_rng(31)
    for k in range(10):  # pepperoni
        r = 0.055 * math.sqrt((k + 0.5) / 10)
        a = k * 2.4 + 0.3
        shapes.append(CylShape((r * math.cos(a), r * math.sin(a), PZ_TOP + 0.0012), 0.0105, 0.0007, PEPPERONI,
                               collide=False))
    for k in range(6):  # basil leaves
        r, a = rng.uniform(0.015, 0.060), rng.uniform(0, 2 * math.pi)
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), PZ_TOP + 0.0016), (0.014, 0.008, 0.0005), BASIL,
                               collide=False, yaw=rng.uniform(0, math.pi)))
    for line, x, y, yaw in _segments():  # the cuts, drawn as they are made
        _SHAPE["cuts"].append(len(shapes))
        shapes.append(BoxShape((x, y, PZ_TOP + 0.0021), (SEG_STEP + 0.0012, 0.0032, 0.0008), CUT, collide=False,
                               yaw=yaw))
    return Body.build("board", pos=(BX, BY, CT), mass=0.55, color=BEECH, shapes=shapes, friction=(1.0, 0.9),
                      com=(0.0, 0.0, 0.015))


def _stand() -> Body:
    """A beech stand: a base with a slot for the wheel at the front and a cradle for the handle's end at the back."""
    shapes = [BoxShape((0.0, 0.0, S_BASE / 2), (0.150, 0.050, S_BASE), BEECH)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((WHEEL_X, s * 0.0065, S_BASE + 0.009), (0.050, 0.006, 0.018), BEECH_D))  # wheel slot
        shapes.append(BoxShape((-0.045, s * 0.0125, S_BASE + 0.030), (0.016, 0.006, 0.060), BEECH_D))  # handle cradle
    shapes.append(BoxShape((-0.045, 0.0, S_BASE + 0.0255 - 0.005), (0.016, 0.019, 0.051), BEECH_D))  # cradle floor
    return Body.build("stand", pos=(SX, SY, CT), mass=0.3, kinematic=True, color=BEECH, shapes=shapes,
                      friction=(0.6, 0.5))


H_Z0 = S_BASE - TIP_Z  # the handle's middle above the stand's base, with the wheel resting in its slot


def _wheel() -> Body:
    """The pizza wheel: a black handle with flat sides, a steel neck and a steel cutting wheel at its front."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (H_L, H_W, H_W), ABS),
              BoxShape((-H_L / 2 + 0.004, 0.0, 0.0), (0.008, H_W + 0.001, H_W + 0.001), (0.75, 0.10, 0.10),
                       collide=False),
              BoxShape(((H_L / 2 + WHEEL_X) / 2, 0.0, WHEEL_Z / 2), (WHEEL_X - H_L / 2 + 0.010, 0.006, 0.006), STEEL,
                       quat_xyzw=(0.0, math.sin(0.30), 0.0, math.cos(0.30))),
              CylShape((WHEEL_X, 0.0, WHEEL_Z), WHEEL_R, 0.002, STEEL, axis="Y"),
              CylShape((WHEEL_X, 0.0, WHEEL_Z), 0.009, 0.010, (0.55, 0.56, 0.58), axis="Y")]
    return Body.build("wheel", pos=(SX, SY, CT + H_Z0), mass=0.12, color=ABS, shapes=shapes, friction=(1.0, 0.9),
                      com=(0.020, 0.0, -0.006))


_PATHS: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: note where the cut lines are drawn, and hide them."""
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "board":
        from pxr import UsdGeom

        _PATHS["cuts"] = [f"{prim_path}/body/s{i}" for i in _SHAPE["cuts"]]
        for p in _PATHS["cuts"]:
            UsdGeom.Imageable(stage.GetPrimAtPath(p)).MakeInvisible()


bodykit._author_sliders = _author_sliders

OBJECTS = [_worktop(), _stand(), _board(), _wheel()]
_SEGS = _segments()


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cut": [False] * len(_SEGS), "last": None, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PIZZA_DEBUG")), "steps": 0}


def _show(path: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(path)
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for p in _PATHS.get("cuts", []):
        _show(p, False)


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


def cuts_done(line: int) -> bool:
    return all(c for c, (ln, *_rest) in zip(_EP["cut"], _SEGS) if ln == line)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "board")
    pw, rw = _pose(base, origin, "wheel")
    yaw = math.degrees(math.atan2(rb[1, 0], rb[0, 0]))
    tip = pw + rw @ np.array([WHEEL_X, 0.0, TIP_Z])  # the wheel's lowest edge (with the wheel upright)
    q = rb.T @ (tip - pb)  # in the board's frame
    pressing = abs(q[2] - PZ_TOP) < PRESS_Z and math.hypot(q[0], q[1]) < R_PZ
    still = True
    if _EP["last"] is not None:
        lp, lyaw = _EP["last"]
        still = (float(np.linalg.norm((pb - lp)[:2])) * 1000.0 < STILL_MM
                 and abs((yaw - lyaw + 180.0) % 360.0 - 180.0) < STILL_DEG)
    _EP["last"] = (pb.copy(), yaw)
    if pressing and still:
        for i, (line, x, y, a) in enumerate(_SEGS):
            if _EP["cut"][i]:
                continue
            along = (q[0] - x) * math.cos(a) + (q[1] - y) * math.sin(a)
            across = -(q[0] - x) * math.sin(a) + (q[1] - y) * math.cos(a)
            if abs(along) < SEG_TOL and abs(across) < SEG_ACROSS:
                _EP["cut"][i] = True
                if "cuts" in _PATHS:
                    _show(_PATHS["cuts"][i], True)
    tilt_b = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[2, 2])))))
    in_stand = (math.hypot(pw[0] - SX, pw[1] - SY) < 0.025 and abs(pw[2] - (CT + H_Z0)) < 0.015
                and abs(float(rw[0, 0])) > 0.9)
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"both": cuts_done(0) and cuts_done(1), "one": cuts_done(0) or cuts_done(1), "in_stand": in_stand,
                "board_ok": tilt_b < 10.0 and WX0 < pb[0] < WX1 and WY0 < pb[1] < WY1}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, CT]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            cuts = "".join("#" if c else "." for c in _EP["cut"])
            print(f"[pizza] t={_DEBUG['steps'] / 20:5.1f}s fingers={[round(v, 1) for v in fj]} board={mm(pb)}"
                  f" yaw={yaw:.1f} wheel={mm(pw)} tip_on_board={np.round(q * 1000).astype(int).tolist()}"
                  f" press={pressing} still={still} cuts={cuts[:SEG_N]}|{cuts[SEG_N:]} stand={in_stand}"
                  f" L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: both cuts made all the way across, the pizza wheel back in its stand, the board flat on the worktop and both
    arms back where they started; 0.5: both cuts made; 0.25: one cut made; else 0.0. All judged at the end."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["both"]:
        return 1.0 if s["in_stand"] and s["board_ok"] and _arms_home() else 0.5
    return 0.25 if s["one"] else 0.0


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
    """Full score with the board and the pizza wheel at rest: the trial ends there."""
    return score(objects) >= 1.0 and objects["board"].resting() and objects["wheel"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the board's handle from above, its pads square to the handle's sides and its wrist leaning out,
# and holds it. The right takes the pizza wheel by its handle out of its stand, lowers the wheel onto the back edge of
# the pizza and runs it forward through the middle, pressing down; it lifts it, turns it a quarter turn and runs it from
# right to left through the middle. It puts the pizza wheel back in its stand; the left lets go and both arms go back to
# where they started.

LUG_FINGER = -0.016  # pre-open only a little wider than the handle: the first finger to touch shoves the board
LUG_ABOVE = 0.003  # the TCP this far above the handle's top: pinched about 1 cm deep, near the fingertips
HOLD_R = R_BD + 0.020  # where the left holds the handle, from the board's middle
LEFT_LEAN = math.radians(25.0)
WHEEL_FINGER = -0.016
PRESS = 0.003  # aim the wheel's edge this far into the pizza: it presses down
RUN = 0.080  # each cut runs from this far behind the middle to this far past it (on the pizza, inside its crust)
CUT_S = 3.2
# Where the board ends up once held: the left's pinch and the wheel landing nudge it about 1 cm and 4 deg on the polished
# worktop (measured); the cuts are aimed along its lines there.
HELD_DX, HELD_DY, HELD_YAW = 0.011, 0.0, math.radians(3.4)
OVER = 0.050
LEAD_S, LEAD_STEPS = 3.0, 15


def _about(axis, a: float) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx


def _rot(arm: str, quarter: bool = False) -> np.ndarray:
    """The left closes across the board's handle, leaned about its closing axis (wrist out); the right holds the pizza
    wheel straight down, closing across its handle: the wheel forward (+x), or a quarter turn left (+y)."""
    if arm == "left":
        g = grip_rotation(HANDLE_A - math.pi)
        return _about(g[:, 0], LEFT_LEAN) @ g
    return grip_rotation(math.pi / 2) if quarter else grip_rotation(0.0)


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
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = np.array([0.0, 0.0, OVER])

    def go(arm, t, pos, finger, label, n=1, quarter=None):
        p0, f0, q0 = cur[arm]
        q = q0 if quarter is None else quarter
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label,
                                      _rot(arm, q)))
        cur[arm] = (p1, finger, q)

    def wait(arm, t, label="wait"):
        p, f, q = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, _rot(arm, q)))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    top = np.array([BX + HOLD_R * math.cos(HANDLE_A), BY + HOLD_R * math.sin(HANDLE_A), CT + BD_TOP])
    hold = top - LUG_ABOVE * _rot("left")[:, 2]
    above = hold - 0.06 * _rot("left")[:, 2]
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", above, _rot("left")),
                                "over the board's handle", LUG_FINGER)
    cur["left"] = (above, LUG_FINGER, False)
    go("left", 1.2, hold, LUG_FINGER, "down round the handle", n=3)
    wait("left", 0.6, "settle")
    go("left", 0.6, hold, FINGER_CLOSED, "hold the board by its handle")

    def grip_for_tip(tip, quarter):  # the right's TCP (the wheel handle's middle) with the wheel's edge at ``tip``
        off = np.array([0.0, WHEEL_X, TIP_Z]) if quarter else np.array([WHEEL_X, 0.0, TIP_Z])
        return np.asarray(tip, float) - off

    take = np.array([SX, SY, CT + H_Z0])
    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
    plan["right"].append(Waypoint(dur("left"), tuple(float(v) for v in p0), WHEEL_FINGER,
                                  "wait at home until the board is held", r0))
    plan["right"] += _joint_path("right", np.array(READY, float), _good_ik("right", take + up, _rot("right")),
                                 "over the pizza wheel in its stand", WHEEL_FINGER)
    cur["right"] = (take + up, WHEEL_FINGER, False)
    go("right", 1.2, take, WHEEL_FINGER, "down round its handle", n=2)
    go("right", 0.6, take, FINGER_CLOSED, "take the pizza wheel")
    go("right", 1.0, take + [0.0, 0.0, 0.040], FINGER_CLOSED, "lift it out of its stand")
    ztop = CT + PZ_TOP

    def on_pizza(bx, by, z):  # a point on the held board's pizza (its own frame) in the world
        c, s = math.cos(HELD_YAW), math.sin(HELD_YAW)
        return [BX + HELD_DX + c * bx - s * by, BY + HELD_DY + s * bx + c * by, z]

    a0 = grip_for_tip(on_pizza(-RUN, 0.0, ztop + 0.015), False)
    go("right", 2.0, a0, FINGER_CLOSED, "carry it over the back edge of the pizza", n=4)
    go("right", 0.8, grip_for_tip(on_pizza(-RUN, 0.0, ztop - PRESS), False), FINGER_CLOSED, "set the wheel on the pizza")
    wait("right", 0.6, "press it down at the edge")
    go("right", CUT_S, grip_for_tip(on_pizza(RUN, 0.0, ztop - PRESS), False), FINGER_CLOSED,
       "run it forward through the middle, pressing", n=8)
    go("right", 0.6, grip_for_tip(on_pizza(RUN, 0.0, ztop + 0.020), False), FINGER_CLOSED, "lift it")
    b0 = grip_for_tip(on_pizza(0.0, -RUN, ztop + 0.020), True)
    go("right", 2.0, b0, FINGER_CLOSED, "turn it a quarter and carry it to the right edge", n=4, quarter=True)
    go("right", 0.8, grip_for_tip(on_pizza(0.0, -RUN, ztop - PRESS), True), FINGER_CLOSED, "set the wheel on the pizza")
    wait("right", 0.6, "press it down at the edge")
    go("right", CUT_S, grip_for_tip(on_pizza(0.0, RUN, ztop - PRESS), True), FINGER_CLOSED,
       "run it to the left through the middle, pressing", n=8)
    go("right", 0.6, grip_for_tip(on_pizza(0.0, RUN, ztop + 0.020), True), FINGER_CLOSED, "lift it")
    go("right", 2.4, take + [0.0, 0.0, 0.040], FINGER_CLOSED, "turn it back and carry it over its stand", n=5,
       quarter=False)
    go("right", 1.0, take + [0.0, 0.0, 0.003], FINGER_CLOSED, "lower it into its stand")
    go("right", 0.5, take + [0.0, 0.0, 0.003], WHEEL_FINGER, "let go")
    go("right", 0.8, take + up, WHEEL_FINGER, "rise clear")
    wait("left", dur("right") - 4.3 - dur("left"), "hold the board")
    go("left", 0.5, hold, LUG_FINGER, "let go of the handle")
    go("left", 0.8, above, LUG_FINGER, "rise clear")
    wait("right", max(0.0, dur("left") - dur("right")))
    for arm in ("left", "right"):
        p, f, q = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, _rot(arm, q)), np.array(READY, float),
                                 "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
