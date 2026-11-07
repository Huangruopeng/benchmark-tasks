"""On the station table stands a white ceramic double kitchen sink set in a grey worktop, both basins half full of
standing water: the shared drain below is clogged. A red rubber plunger stands in the right basin over its drain; a
black rubber drain stopper lies on the worktop in front of the left basin.

Task: unclog the sink. The two basins share one waste pipe, so a push of the plunger just blows water up out of the
other basin's drain unless that drain is shut: one hand puts the stopper on the left basin's drain and holds it down
while the other pumps the plunger over the right drain at least four times, until the water drains away. Then lift the
stopper off (the left basin drains too) and put it back on the worktop, stand the plunger on the worktop in front of
the right basin, and take both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=unclog_a_double_sink_holding_a_stopper_on_the_other_drain.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the sink: two basins side by side along y, set in a worktop ------------------------------------------------------
SX0, SX1 = 0.205, 0.315  # the basins' inside, front to back (x)
RB = (-0.150, -0.020)  # the right basin's inside (y)
LB = (0.020, 0.150)  # the left basin's inside
WALL = 0.005
FLOOR = TOP + 0.006  # the basins' floor
RIM = TOP + 0.045  # the worktop's top
FX0 = 0.140  # the worktop's front edge (the strip in front of the basins runs from FX0 to SX0)
WY0, WY1 = -0.180, 0.180  # the worktop's ends
DRAIN_R = np.array([(SX0 + SX1) / 2, (RB[0] + RB[1]) / 2])
DRAIN_L = np.array([(SX0 + SX1) / 2, (LB[0] + LB[1]) / 2])
WATER_Z = TOP + 0.020  # the standing water's surface (1.4 cm deep: the plunger's cup shows above it)
# -- the plunger: a rubber cup (origin: its rim's middle on the floor) and its handle, which slides into the cup as the
#    rubber squashes ----------------------------------------------------------------------------------------------------
CUP_R, CUP_H = 0.020, 0.020
SHAFT_R, SHAFT_L = 0.0055, 0.075
KNOB_R, KNOB_Z = 0.011, 0.080  # the handle's knob: its middle this far above the handle's origin (the cup's top)
SQUASH = 0.012  # the rubber squashes at most this far
STROKE_IN, STROKE_OUT = 0.008, 0.003  # a stroke: squashed this far, then back out to this
STROKES = 4
# -- the stopper (origin: its bottom's middle) --------------------------------------------------------------------------
STOP_R, STOP_H = 0.018, 0.008
STOP_KNOB_Z = 0.020  # its knob's middle above its bottom
STOP_START = np.array([0.172, 0.085])
PLUNGER_END = np.array([0.172, -0.085])

WHITE, STEEL, WORKTOP, RUBBER_RED, RUBBER_BLACK, BEECH, WATER = (0.93, 0.93, 0.91), (0.74, 0.76, 0.79), \
    (0.42, 0.43, 0.45), (0.78, 0.14, 0.12), (0.10, 0.10, 0.11), (0.80, 0.62, 0.40), (0.60, 0.76, 0.90)
WATER_OPACITY = 0.35
SPURTS = 4
PARKED_SPURTS = [(0.62, -0.30 + 0.03 * k, 0.40) for k in range(SPURTS)]
PARKED_WATER = {"water_right": (0.70, -0.20, 0.30), "water_left": (0.70, 0.20, 0.30)}

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _sink() -> Body:
    """The worktop with the two ceramic basins in it, a steel strainer in each basin's floor. Origin: the table under the
    worktop's front right corner."""
    x0, y0 = FX0, WY0
    shapes = []

    def box(xa, xb, ya, yb, za, zb, color, collide=True):
        shapes.append(BoxShape(((xa + xb) / 2 - x0, (ya + yb) / 2 - y0, (za + zb) / 2 - TOP),
                               (xb - xa, yb - ya, zb - za), color, collide=collide))

    back = SX1 + 0.04
    box(FX0, SX0 - WALL, WY0, WY1, TOP, RIM, WORKTOP)  # the strip in front
    box(SX1 + WALL, back, WY0, WY1, TOP, RIM, WORKTOP)  # the strip behind
    box(SX0 - WALL, SX1 + WALL, WY0, RB[0] - WALL, TOP, RIM, WORKTOP)  # right end
    box(SX0 - WALL, SX1 + WALL, LB[1] + WALL, WY1, TOP, RIM, WORKTOP)  # left end
    box(SX0 - WALL, SX1 + WALL, RB[1] + WALL, LB[0] - WALL, TOP, RIM, WORKTOP)  # between the basins
    for ya, yb in (RB, LB):  # each basin: floor and four walls, white ceramic
        box(SX0, SX1, ya, yb, TOP, FLOOR, WHITE)
        box(SX0 - WALL, SX0, ya - WALL, yb + WALL, TOP, RIM, WHITE)
        box(SX1, SX1 + WALL, ya - WALL, yb + WALL, TOP, RIM, WHITE)
        box(SX0, SX1, ya - WALL, ya, TOP, RIM, WHITE)
        box(SX0, SX1, yb, yb + WALL, TOP, RIM, WHITE)
    for d in (DRAIN_R, DRAIN_L):  # the strainers, flush with the floor
        shapes.append(CylShape((d[0] - x0, d[1] - y0, FLOOR - TOP + 0.0005), 0.014, 0.001, STEEL, collide=False))
        shapes.append(CylShape((d[0] - x0, d[1] - y0, FLOOR - TOP + 0.0008), 0.005, 0.001, (0.25, 0.25, 0.27),
                               collide=False))
    # the mixer tap behind, between the basins
    shapes += [CylShape((SX1 + 0.022 - x0, -y0, RIM - TOP + 0.03), 0.007, 0.06, STEEL, collide=False),
               BoxShape((SX1 + 0.005 - x0, -y0, RIM - TOP + 0.058), (0.035, 0.008, 0.008), STEEL, collide=False)]
    return Body.build("sink", pos=(x0, y0, TOP), mass=10.0, kinematic=True, color=WORKTOP, shapes=shapes,
                      friction=(0.8, 0.7))


def _cup() -> Body:
    """The plunger's red rubber cup, with a short neck the handle slides into."""
    shapes = [CylShape((0.0, 0.0, 0.006), CUP_R, 0.012, RUBBER_RED),
              CylShape((0.0, 0.0, 0.0145), CUP_R * 0.75, 0.005, RUBBER_RED),
              CylShape((0.0, 0.0, 0.0185), 0.009, 0.003, RUBBER_RED)]
    return Body.build("plunger_cup", pos=(DRAIN_R[0], DRAIN_R[1], FLOOR), mass=0.06, shapes=shapes,
                      friction=(1.2, 1.1), color=RUBBER_RED, com=(0.0, 0.0, 0.006))


def _handle() -> Body:
    """The plunger's beech handle with a round knob on top. Its origin is the cup's top."""
    shapes = [CylShape((0.0, 0.0, SHAFT_L / 2), SHAFT_R, SHAFT_L, BEECH),
              SphereShape((0.0, 0.0, KNOB_Z), KNOB_R, BEECH)]
    return Body.build("plunger_handle", pos=(DRAIN_R[0], DRAIN_R[1], FLOOR + CUP_H), mass=0.03, shapes=shapes,
                      friction=(1.4, 1.3), color=BEECH, com=(0.0, 0.0, 0.03), ignore=("plunger_cup",))


def _stopper() -> Body:
    """A black rubber sink stopper with a steel knob."""
    shapes = [CylShape((0.0, 0.0, STOP_H / 2), STOP_R, STOP_H, RUBBER_BLACK),
              CylShape((0.0, 0.0, STOP_H + 0.006), 0.0045, 0.012, STEEL),
              SphereShape((0.0, 0.0, STOP_KNOB_Z), 0.0075, STEEL)]
    return Body.build("stopper", pos=(STOP_START[0], STOP_START[1], RIM), mass=0.03, shapes=shapes,
                      friction=(1.2, 1.1), color=RUBBER_BLACK, com=(0.0, 0.0, 0.004))


def _water(name: str, ys) -> Body:
    """The standing water in one basin: a see-through slab (no collisions), lowered as the basin drains."""
    w = [BoxShape((0.0, 0.0, -0.001), (SX1 - SX0 - 0.002, ys[1] - ys[0] - 0.002, 0.002), WATER, collide=False)]
    return Body.build(name, pos=((SX0 + SX1) / 2, (ys[0] + ys[1]) / 2, WATER_Z), mass=0.1, kinematic=True,
                      color=WATER, shapes=w)


def _spurt(k: int) -> Body:
    return Body.build(f"spurt{k}", pos=PARKED_SPURTS[k], mass=0.002, color=WATER, friction=(0.9, 0.8),
                      damping=(0.3, 0.5), shapes=[SphereShape((0.0, 0.0, 0.0), 0.004, WATER)])


OBJECTS = [_sink(), _cup(), _handle(), _stopper(), _water("water_right", RB), _water("water_left", LB)] + \
    [_spurt(k) for k in range(SPURTS)]
# The handle slides down into the cup as the rubber squashes, and springs back out.
slider("squash", "plunger_cup", "plunger_handle", anchor=(DRAIN_R[0], DRAIN_R[1], FLOOR + CUP_H), axis="Z",
       limits=(-SQUASH, 0.0), stiffness=260.0, damping=4.0, target=0.0, max_force=200.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"effective": 0, "tried": 0, "down": False, "cleared": False, "drain_r": 0.0, "drain_l": 0.0,
            "spurt": -1.0, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SINK_DEBUG")), "steps": 0}
DT = 0.05
DRAIN_S = 3.0


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _see_through_water()


def _see_through_water() -> None:
    """Make the water slabs see-through (their material's opacity), once."""
    if _DEBUG.get("water_done"):
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Sdf, UsdShade

        n = 0
        for prim in get_current_stage().Traverse():
            path = str(prim.GetPath())
            if "/water_" not in path or "/Looks/" not in path or not prim.IsA(UsdShade.Shader):
                continue
            shader = UsdShade.Shader(prim)
            for name in ("opacity_constant", "opacity"):
                shader.CreateInput(name, Sdf.ValueTypeNames.Float).Set(WATER_OPACITY)
            shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
            n += 1
        if _DEBUG["on"]:
            print(f"[sink] made {n} water shader(s) see-through", file=sys.stderr, flush=True)
    except Exception as e:  # cosmetic only
        print(f"[sink] could not make the water see-through: {e}", file=sys.stderr, flush=True)
    _DEBUG["water_done"] = True


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


def _place(base, origin, name, pos, vel=None) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)
    try:
        v = [0.0] * 6 if vel is None else [*vel, 0.0, 0.0, 0.0]
        asset.write_root_velocity_to_sim(torch.tensor([v], dtype=torch.float32, device=asset.device))
    except Exception:  # a kinematic body has no velocity to set
        pass


def _kick(base, name, vel) -> None:
    import torch

    asset = base.scene[name]
    asset.write_root_velocity_to_sim(torch.tensor([[*vel, 0.0, 0.0, 0.0]], dtype=torch.float32, device=asset.device))


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _on_drain(p, r, drain, within) -> bool:
    return math.hypot(p[0] - drain[0], p[1] - drain[1]) <= within and abs(p[2] - FLOOR) <= 0.008 and _tilt(r) <= 15.0


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    cp, cr = _pose(base, origin, "plunger_cup")
    hp, hr = _pose(base, origin, "plunger_handle")
    sp, sr = _pose(base, origin, "stopper")
    tl, trc = _tcps(base, origin)
    _EP["tcps"] = (tl, trc)
    squash = float(np.dot(cr.T @ (hp - cp), [0.0, 0.0, 1.0])) - CUP_H  # 0 at rest, -SQUASH squashed
    plunger_on = _on_drain(cp, cr, DRAIN_R, 0.012)
    stopper_on = _on_drain(sp, sr, DRAIN_L, 0.010)
    knob = sp + sr @ np.array([0.0, 0.0, STOP_KNOB_Z])
    held = stopper_on and float(np.linalg.norm(tl - knob)) <= 0.02
    if not _EP["down"] and squash <= -STROKE_IN:
        _EP["down"] = True  # a push of the plunger
        if plunger_on and not _EP["cleared"]:
            _EP["tried"] += 1
            if held:
                _EP["effective"] += 1
                if _EP["effective"] >= STROKES:
                    _EP["cleared"] = True
            elif stopper_on:  # the push lifts the stopper off the other drain, and water spurts out under it
                _kick(base, "stopper", (0.08, 0.05, 0.55))
                _EP["spurt"] = 0.0
            else:  # water spurts up out of the open drain
                _EP["spurt"] = 0.0
    elif _EP["down"] and squash >= -STROKE_OUT:
        _EP["down"] = False
    if _EP["spurt"] >= 0.0:  # the spurt: a few drops thrown up out of the left drain, one after another
        k = int(round(_EP["spurt"] / DT))
        if k < SPURTS:
            a = k * 2.1
            _place(base, origin, f"spurt{k}", (DRAIN_L[0] + 0.006 * math.cos(a), DRAIN_L[1] + 0.006 * math.sin(a),
                                               FLOOR + 0.03), (0.06 * math.cos(a), 0.06 * math.sin(a), 1.0 + 0.1 * k))
            _EP["spurt"] += DT
        else:
            _EP["spurt"] = -1.0
    # draining: the right basin once the clog is pushed through; the left once the clog is through and its drain open
    if _EP["cleared"]:
        _EP["drain_r"] = min(1.0, _EP["drain_r"] + DT / DRAIN_S)
        if not stopper_on:
            _EP["drain_l"] = min(1.0, _EP["drain_l"] + DT / DRAIN_S)
    for name, frac, ys in (("water_right", _EP["drain_r"], RB), ("water_left", _EP["drain_l"], LB)):
        if frac >= 1.0:
            if not _EP["s"].get(name + "_parked"):
                _place(base, origin, name, PARKED_WATER[name])
                _EP["s"][name + "_parked"] = True
        elif frac > 0.0:
            _place(base, origin, name, ((SX0 + SX1) / 2, (ys[0] + ys[1]) / 2,
                                        WATER_Z - frac * (WATER_Z - FLOOR - 0.0015)))
    parked = {k: v for k, v in _EP["s"].items() if k.endswith("_parked")}
    stopper_back = FX0 + 0.005 <= sp[0] <= SX0 - WALL - 0.005 and 0.03 <= sp[1] <= WY1 - 0.01 \
        and abs(sp[2] - RIM) < 0.006 and _tilt(sr) <= 15.0
    plunger_back = FX0 + 0.005 <= cp[0] <= SX0 - WALL - 0.002 and WY0 + 0.01 <= cp[1] <= -0.03 \
        and abs(cp[2] - RIM) < 0.006 and _tilt(cr) <= 15.0
    _EP["s"] = {**parked, "stopper_back": stopper_back, "plunger_back": plunger_back}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            print(f"[sink] t={_DEBUG['steps'] / 20:5.1f}s squash={squash * 1000:6.1f}mm on={plunger_on} "
                  f"stopper_on={stopper_on} held={held} knob_d={np.linalg.norm(tl - knob) * 1000:5.0f}mm "
                  f"tried={_EP['tried']} eff={_EP['effective']} cleared={_EP['cleared']} "
                  f"drain=({_EP['drain_r']:.2f},{_EP['drain_l']:.2f}) cup={np.round(cp * 1000).astype(int).tolist()} "
                  f"cup_tilt={_tilt(cr):4.1f} stopper={np.round(sp * 1000).astype(int).tolist()} "
                  f"back=({stopper_back},{plunger_back}) "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000).astype(int).tolist()} score={score(None)}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the clog pushed through (four pushes of the plunger over the right drain while the stopper was held
    down on the left drain), both basins drained, the stopper back on the worktop in front of the left basin, the
    plunger standing on the worktop in front of the right basin, and both arms back where they started, at the end;
    0.5 with the clog pushed through; 0.25 with the plunger pushed over the right drain at some point; else 0.0."""
    if _EP["cleared"]:
        s = _EP["s"]
        if (_EP["drain_r"] >= 1.0 and _EP["drain_l"] >= 1.0 and s.get("stopper_back") and s.get("plunger_back")
                and _arms_home()):
            return 1.0
        return 0.5
    return 0.25 if _EP["tried"] >= 1 else 0.0


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
    """Full score with the plunger and the stopper still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["plunger_cup"].resting() and objects["plunger_handle"].resting()
            and objects["stopper"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the stopper by its knob off the worktop, sets it on the left drain and holds it down. The right
# hand takes the plunger by its knob and pumps it four times over the right drain; the water drains. The right lifts
# the plunger out and stands it on the worktop in front of the right basin; the left lifts the stopper off the drain
# (the left basin drains too) and puts it back on the worktop. Both arms go back to where they started.

G0 = grip_rotation(0.0)  # down, fingers closing along y
STOP_FINGER = -0.014
KNOB_FINGER = -0.018
PUSH = 0.022  # the right hand pushes the knob this far down for a stroke (the rubber takes 12 mm, the arm gives)
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


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    take = np.array([STOP_START[0], STOP_START[1], RIM + STOP_KNOB_Z])
    hold = np.array([DRAIN_L[0], DRAIN_L[1], FLOOR + STOP_KNOB_Z - 0.003])  # pressing it down a little
    out = _lead_in("left", take + [0.0, 0.0, 0.05], G0, "over the stopper's knob", STOP_FINGER)
    out += [_w(1.0, take, STOP_FINGER, "down round the knob", G0),
            _w(0.6, take, FINGER_CLOSED, "take the stopper", G0),
            _w(1.0, take + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it", G0),
            _w(1.5, hold + [0.0, 0.0, RIM + 0.05 - FLOOR], FINGER_CLOSED, "carry it over the left drain", G0),
            _w(1.3, hold + [0.0, 0.0, 0.01], FINGER_CLOSED, "lower it onto the drain", G0),
            _w(0.6, hold, FINGER_CLOSED, "press it down on the drain", G0)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), hold, FINGER_CLOSED, "hold the stopper down on the drain", G0),
            _w(1.2, hold + [0.0, 0.0, RIM + 0.05 - FLOOR], FINGER_CLOSED, "lift it off: the left basin drains too",
               G0),
            _w(1.6, take + [0.0, 0.0, 0.05], FINGER_CLOSED, "carry it back over the worktop", G0),
            _w(1.0, take + [0.0, 0.0, 0.002], FINGER_CLOSED, "set it down", G0),
            _w(0.5, take + [0.0, 0.0, 0.002], STOP_FINGER, "let go", G0),
            _w(0.8, take + [0.0, 0.0, 0.05], STOP_FINGER, "rise", G0)]
    out += _go_home("left", take + [0.0, 0.0, 0.05], G0, STOP_FINGER)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, STOP_FINGER,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    knob = np.array([DRAIN_R[0], DRAIN_R[1], FLOOR + CUP_H + KNOB_Z])
    end = np.array([PLUNGER_END[0], PLUNGER_END[1], RIM + CUP_H + KNOB_Z])
    lift = RIM + 0.012 - FLOOR  # the cup's bottom 1.2 cm over the worktop
    out = _lead_in("right", knob + [0.0, 0.0, 0.04], G0, "over the plunger's knob", KNOB_FINGER)
    out.append(_w(max(start - LEAD_S, 0.3), knob + [0.0, 0.0, 0.04], KNOB_FINGER,
                  "wait for the stopper to be held down on the other drain", G0))
    out += [_w(1.0, knob, KNOB_FINGER, "down round the knob", G0),
            _w(0.6, knob, FINGER_CLOSED, "take the plunger by its knob", G0)]
    for k in range(STROKES + 1):
        out += [_w(0.5, knob - [0.0, 0.0, PUSH], FINGER_CLOSED, "push the plunger down", G0),
                _w(0.5, knob + [0.0, 0.0, 0.002], FINGER_CLOSED, "let it spring back up", G0)]
    out.append(_w(DRAIN_S + 0.5, knob + [0.0, 0.0, 0.002], FINGER_CLOSED, "the water drains away", G0))
    done_at = sum(w.duration for w in out)
    out += [_w(1.4, knob + [0.0, 0.0, lift], FINGER_CLOSED, "lift the plunger out of the basin", G0),
            _w(1.6, end + [0.0, 0.0, 0.012], FINGER_CLOSED, "carry it over the worktop in front", G0),
            _w(1.0, end + [0.0, 0.0, 0.002], FINGER_CLOSED, "stand it on the worktop", G0),
            _w(0.5, end + [0.0, 0.0, 0.002], KNOB_FINGER, "let go", G0),
            _w(0.8, end + [0.0, 0.0, 0.04], KNOB_FINGER, "rise", G0)]
    out += _go_home("right", end + [0.0, 0.0, 0.04], G0, KNOB_FINGER)
    return out, done_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, done_at = _right(held_at + 0.5)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, KNOB_FINGER, "wait at the start", right[-1].yaw))
    left, _ = _left(done_at + 0.3, total)
    return {"left": left, "right": right}
