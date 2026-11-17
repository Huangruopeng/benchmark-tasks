"""On the station table stands the pivot of a loader arm: a yellow steel lug on a base plate with a steel pin through
it, and on the lug's top a straight grease nipple (DIN 71412 form A: M10x1 taper thread, 11 mm hex, 6.5 mm ball
head). To the left a lever grease gun lies in a cradle with its handle on top; its whip hose runs over the table to a
lock-on grease coupler standing in a steel holder cup.

Task: grease the pivot pin through its nipple with the grease gun without leaking grease at the coupler or the nipple,
then leave the coupler back in its holder cup and both arms back where they started.

The coupler's mouth (its four jaws, closed: a 7.2 mm bore with a 45-degree lead-in) goes right over the 6.5 mm ball
head until a nitrile seal ring 5 mm up sits on the ball's top. Pushed fully on with its thumb lever out, its jaws lock
behind the ball and hold it on against the grease pressure with no hand on it; it comes off only while the thumb lever
is pressed in. Not locked, the grease pressure pushes it back off the ball. The gun pumps while its handle is pushed
down: a handle stroke (lifted, then pushed back down) made through a sealed seat is a shot of grease into the pin's
bore; pushing the handle down while the coupler is not sealed on the nipple is a leak (latched for the whole trial).

Run it:  inspect-robots run --embodiment yam_arena -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, ball, hinge,  # noqa: E402
                     on_reset, on_step, plate, rotation, segment, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the loader pivot: a lug with a pin through it and a grease nipple on top ------------------------------------------
# Built from a 50 x 40 mm steel flat bar cut 56 mm long and stood on end (see the README's drawing): a 20.5 mm bore
# for the 20 mm pin, 20 mm above its foot; an M10x1 hole in its top for the nipple, and a 5 mm passage down to the bore.
MX, MY = 0.30, -0.08
LUG = (0.05, 0.04, 0.056)
BASE_T = 0.010
PIN_Z = 0.030  # the pin's axis above the table
PIN_R, BORE_D = 0.010, 0.0205  # the 20 mm pin (ISO 2340 B 20 x 100) in the lug's 20.5 mm drilled bore
BOSS_TOP = TOP + BASE_T + LUG[2]  # the lug's top face, where the nipple is screwed in
# the grease nipple, DIN 71412 form A (straight): M10x1 taper thread (in the lug), 11 mm hex, 6.5 mm ball head on a
# 4 mm neck, its top 10.5 mm above the lug (16 mm long, 5.5 mm of it thread)
HEX_AF, HEX_H = 0.011, 0.0035
NECK_R = 0.002
BALL_R = 0.00325
BALL_Z = BOSS_TOP + 0.00725  # the ball head's centre
NIPPLE_TOP = BALL_Z + BALL_R

# -- the lever grease gun (Lincoln 1145 in its high-pressure setting) lying in a cradle, its handle on top ------------
# It lies along y with its head toward the middle of the table and its outlet facing the robot's right. The handle is
# pivoted on the head and lies along the top of the barrel; lifting it draws grease into the pump, pushing it back down
# drives the plunger and pushes grease out. It has no return spring: it rests down on its stop.
GX = 0.33  # the gun's axis, this far forward
BARREL_R = 0.0286  # the 2-1/4 in. barrel (published)
BARREL_Z = 0.006 + BARREL_R  # its axis above the table, in the cradle
HEAD_Y0, HEAD_Y1 = 0.105, 0.180  # the cast head
BARREL_Y1 = HEAD_Y1 + 0.330  # the barrel's rear end
PIVOT = np.array([GX, 0.165, TOP + 0.090])  # the handle's pivot on the head
LEVER_L = 0.300  # from the pivot to the handle's end
GRIP0, GRIP1 = 0.190, 0.300  # its rubber grip, along the handle
GRIP_W, GRIP_H = 0.026, 0.020  # the grip's width and height
LEVER_KG, LEVER_COM = 0.30, 0.17  # the handle's mass and its centre of mass along it
LEVER_OPEN = 60.0  # degrees the handle opens at most
# Lincoln's figures for this gun in its high-pressure setting: full strokes up to 9 in. at the handle's grip, 1 oz of
# grease per 33 full strokes. Grease ~0.9 g/cm3, so a full stroke is ~0.95 cm3: ~0.018 cm3 per degree of handle swing.
FULL_CC = 28.35 / 33.0 / 0.9
FULL_DEG = math.degrees(9.0 * 0.0254 / ((GRIP0 + GRIP1) / 2))  # the full swing, the 9 in. measured at the grip's middle
CC_PER_DEG = FULL_CC / FULL_DEG
STROKE_TOP, STROKE_BOTTOM = 20.0, 3.0  # a stroke: the handle lifted past 20 degrees, then pushed back down within 3
PRESSURE_DEG = 3.0  # pushed down this far from where it was lifted to, the plunger is pushing grease out
REST_DEG = 1.5  # ... until the handle is down on its stop
STROKES = 4  # sealed strokes that push fresh grease through the pin's bore and out at both its ends
OUTLET = (GX, HEAD_Y0 - 0.006, TOP + 0.050)
# The handle's damper stands in for the grease's resistance (the gun's plunger pushing it through the hose, the
# coupler, the nipple's check valve and the pin's bore): at the demo's ~17 degrees/s it takes ~5 N at the robot's grip
# point (a real rig: at most 15 N, checked with a spring scale on the grip).
LEVER_DAMP = 0.075  # N m per degree/s
GRIP_R = 0.260  # where the robot's hand holds the grip, along the handle
DESIGN_DEG_S = 17.0
DESIGN_N = LEVER_DAMP * DESIGN_DEG_S / GRIP_R  # ~4.9 N at the robot's grip point
LINE_PA = LEVER_DAMP * DESIGN_DEG_S / (FULL_CC * 1e-6 / math.radians(FULL_DEG))  # the torque over the plunger's
# displacement per radian: ~12.5 bar in the line

# -- the lock-on coupler (origin: the middle of its mouth face, z up its axis) and its holder cup -------------------
HX, HY = 0.24, 0.0
CUP_R, CUP_H, CUP_FLOOR = 0.023, 0.016, 0.004  # the holder's outer radius, rim height and floor
SEAT_R, FUNNEL_Z, FUNNEL_R = 0.011, 0.007, 0.020  # the 22 mm seat the nose stands in, and its 45-degree funnel
IN_HOLDER_OFF, IN_HOLDER_UP, IN_HOLDER_TILT = 0.006, 0.006, 35.0  # the mouth down in the seat, the coupler upright
NOSE_R, NOSE_H = 0.009, 0.012  # the hardened steel nose, 18 mm across
BORE_R = 0.0036  # the mouth: a 7.2 mm bore over the 6.5 mm ball head
CHAMFER_R, CHAMFER_D = 0.0052, 0.0016  # its 45-degree lead-in, 10.4 mm across at the face
SEAL_Z, SEAL_T, SEAL_HOLE_R = 0.005, 0.002, 0.0023  # the nitrile seal ring 5 mm up the bore, its hole 4.6 mm across
BODY_AF, BODY_Z1 = 0.013, 0.038  # the 13 mm hex body above the nose, where it is held
SWIVEL_R, SWIVEL_Z1 = 0.006, 0.048  # the brass swivel on top
STUB_Z, STUB_L, STUB_R = 0.043, 0.030, 0.005  # its side outlet, where the hose comes in
COUPLER_KG = 0.07
SEAT_DEPTH = math.sqrt(BALL_R ** 2 - SEAL_HOLE_R ** 2)  # the ball's centre this far under the seal when seated
BALL_IN_MOUTH = SEAL_Z - SEAT_DEPTH  # the ball's centre this far up from the mouth face when seated (2.7 mm)
MOUTH_SEATED = BALL_Z - BALL_IN_MOUTH  # the mouth face's height when seated (1.05 mm over the hex)
SEAT_OFF, SEAT_LIFT, SEAT_SINK, SEAT_TILT = 0.0015, 0.0005, 0.001, 8.0  # the sealed-seat tolerances
# While the line is under pressure, the grease pushes a coupler on the ball back off with PUSH_N along its axis: the
# line's ~12.5 bar on the seal's 4.6 mm circle (16.6 mm2) is ~21 N.
SEAL_AREA = math.pi * SEAL_HOLE_R ** 2
PUSH_N = round(LINE_PA * SEAL_AREA)
PUSH_OFF, PUSH_LIFT, PUSH_TILT = 0.002, 0.001, 20.0  # ... while its seal is this close on the ball
# the thumb lever: a pad on the body's +y flat, sprung out 2.5 mm; pressed in (by a thumb, or by fingers closing across
# the body) it pulls the locking sleeve back and opens the jaws
THUMB_W, THUMB_H, THUMB_T, THUMB_Z = 0.006, 0.020, 0.0025, 0.025  # its width, height, thickness and middle height
THUMB_TRAVEL = 0.0025  # how far it presses in
THUMB_Y = BODY_AF / 2 + THUMB_TRAVEL + THUMB_T / 2  # its middle out from the axis when not pressed
THUMB_K, THUMB_PRELOAD = 3000.0, 0.001  # its spring (N/m) and preload (m): 3 N to start it, 10.5 N pressed fully in
THUMB_OPEN, THUMB_SHUT = 0.0015, 0.0008  # pressed in past THUMB_OPEN the jaws open; back out past THUMB_SHUT they close
LOCK_LIFT = 0.0001  # locked, the jaws hold the seal 0 to 0.2 mm off the ball's top (the rigid seal is not squeezed)

# -- the whip hose (18 in.): six segments on ball joints from the gun's outlet to the coupler's side outlet -----------
HOSE = [OUTLET, (GX, 0.062, TOP + 0.007), (0.40, 0.035, TOP + 0.007), (0.471, -0.034, TOP + 0.007),
        (0.37, 0.0, TOP + 0.007), (0.31, 0.0, TOP + 0.007), (HX + STUB_L, HY, TOP + CUP_FLOOR + STUB_Z)]
N_HOSE = len(HOSE) - 1

YELLOW, STEEL, DARK, RED, BRASS, BLACK, GREASE, ZINC = (0.93, 0.68, 0.10), (0.72, 0.74, 0.77), (0.20, 0.21, 0.23), \
    (0.72, 0.12, 0.10), (0.80, 0.66, 0.30), (0.06, 0.06, 0.06), (0.62, 0.45, 0.12), (0.62, 0.64, 0.66)
WOOD = (0.78, 0.62, 0.42)
GREASE_SPOTS = [(MX + 0.006, MY + 0.0235, TOP + PIN_Z + 0.017), (MX + 0.006, MY - 0.0235, TOP + PIN_Z + 0.017),
                (MX - 0.010, MY + 0.0235, TOP + PIN_Z + 0.015), (MX - 0.010, MY - 0.0235, TOP + PIN_Z + 0.015)]
PARKED = [(0.55, -0.40 + 0.03 * k, TOP - 0.12) for k in range(5)]  # under the table until squeezed out
LEAK_NEAR = 0.06  # a leak with the coupler's mouth this close to the nipple shows at the nipple

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _hexagon(af: float, z0: float, z1: float, color, collide: bool = True) -> list:
    """A hexagonal prism (``af`` across flats, flats facing +-y) from ``z0`` to ``z1`` on the body's axis."""
    side = af / math.sqrt(3.0)
    return [BoxShape((0.0, 0.0, (z0 + z1) / 2), (side, af, z1 - z0), color, yaw=k * math.pi / 3, collide=collide)
            for k in range(3)]


def _pivot() -> Body:
    """The loader arm's pivot: a base plate, a yellow lug, a steel pin through it (washers, split pins) with the
    arm's two side plates on it, and the grease nipple screwed into the lug's top. Origin on the table under the lug.
    Greasy steel: slick (low friction)."""
    shapes = [BoxShape((0.0, 0.0, BASE_T / 2), (0.12, 0.10, BASE_T), DARK),
              BoxShape((0.0, 0.0, BASE_T + LUG[2] / 2), LUG, YELLOW),
              CylShape((0.0, 0.0, PIN_Z), PIN_R, 0.100, STEEL, axis="Y")]
    for s in (-1.0, 1.0):
        shapes += [CylShape((0.0, s * 0.024, PIN_Z), 0.016, 0.004, ZINC, axis="Y", collide=False),  # washers
                   BoxShape((0.0, s * 0.031, PIN_Z), (0.042, 0.008, 0.034), YELLOW, collide=False),  # side plates
                   BoxShape((-0.035, s * 0.031, PIN_Z + 0.004), (0.035, 0.008, 0.026), YELLOW, collide=False),
                   BoxShape((0.0, s * 0.047, PIN_Z), (0.003, 0.003, 0.022), STEEL, collide=False)]  # split pins
    b = BOSS_TOP - TOP
    shapes += _hexagon(HEX_AF, b, b + HEX_H, BRASS)  # the nipple's hex (yellow-zinc-plated steel)
    shapes += [CylShape((0.0, 0.0, b + HEX_H + 0.00125), NECK_R, 0.0025, BRASS),  # its neck
               SphereShape((0.0, 0.0, BALL_Z - TOP), BALL_R, BRASS)]  # its ball head
    return Body.build("pivot", pos=(MX, MY, TOP), mass=5.0, shapes=shapes, kinematic=True, color=YELLOW,
                      friction=(0.15, 0.12), slick=True)


def _pump() -> Body:
    """The lever grease gun in its cradle: a plywood board with two saddles and straps, the plated barrel lying along
    y, its rear cap and follower rod, and the cast head with the handle's bracket and the outlet facing -y. Origin on
    the table under the head's middle."""
    y0 = (HEAD_Y0 + HEAD_Y1) / 2
    yb = (HEAD_Y1 + BARREL_Y1) / 2 - y0

    def y(v: float) -> float:
        return v - y0

    shapes = [BoxShape((0.0, y(0.32), 0.003), (0.080, 0.400, 0.006), WOOD),
              CylShape((0.0, yb, BARREL_Z), BARREL_R, BARREL_Y1 - HEAD_Y1, ZINC, axis="Y"),
              CylShape((0.0, y(BARREL_Y1 + 0.006), BARREL_Z), BARREL_R + 0.002, 0.012, STEEL, axis="Y"),
              CylShape((0.0, y(BARREL_Y1 + 0.040), BARREL_Z), 0.003, 0.056, STEEL, axis="Y", collide=False),
              CylShape((0.0, y(BARREL_Y1 + 0.068), BARREL_Z), 0.004, 0.050, STEEL, axis="X", collide=False),
              BoxShape((0.0, 0.0, 0.045), (0.060, HEAD_Y1 - HEAD_Y0, 0.078), DARK),  # the cast head
              BoxShape((0.0, y(PIVOT[1]), 0.087), (0.034, 0.026, 0.010), DARK, collide=False),  # the handle's bracket
              CylShape((0.0, y(PIVOT[1]), PIVOT[2] - TOP), 0.004, 0.040, STEEL, axis="X", collide=False),
              CylShape((0.0, y(HEAD_Y0 - 0.004), OUTLET[2] - TOP), 0.006, 0.010, BRASS, axis="Y", collide=False)]
    for yy in (0.24, 0.46):  # two wooden saddles with steel straps over the barrel
        shapes += [BoxShape((s * 0.033, y(yy), 0.020), (0.012, 0.030, 0.028), WOOD, collide=False) for s in (-1, 1)]
        shapes.append(CylShape((0.0, y(yy), BARREL_Z), BARREL_R + 0.0015, 0.020, DARK, axis="Y", collide=False))
    return Body.build("pump", pos=(GX, y0, TOP), mass=4.0, shapes=shapes, kinematic=True, color=ZINC)


def _lever() -> Body:
    """The gun's handle: a plated steel bar from the pivot along the top of the barrel, with a black rubber grip at its
    end. Origin at the pivot; it starts down on its stop, level."""
    shapes = [BoxShape((0.0, GRIP0 / 2, 0.0), (0.020, GRIP0, 0.010), STEEL),
              BoxShape((0.0, (GRIP0 + GRIP1) / 2, 0.0), (GRIP_W, GRIP1 - GRIP0, GRIP_H), BLACK),
              BoxShape((0.0, 0.030, -0.010), (0.014, 0.012, 0.016), STEEL, collide=False)]  # the toggle link
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=LEVER_KG, shapes=shapes, color=STEEL,
                      com=(0.0, LEVER_COM, 0.0), friction=(1.2, 1.1), ignore=("pump",))


def _holder() -> Body:
    """A steel holster for the coupler on a foot plate: a 22 mm seat 3 mm deep that the nose stands in, under a
    45-degree funnel 40 mm across at its rim, 16 mm above the table. Slick steel, so a coupler set down on the funnel
    slides into the seat. The seat wall, the funnel and the outer wall are 12-sided rings."""
    n = 12
    tan = math.tan(math.pi / n)
    shapes = [CylShape((0.0, 0.0, CUP_FLOOR / 2), CUP_R, CUP_FLOOR, STEEL),
              BoxShape((-0.035, 0.0, 0.0015), (0.03, 0.05, 0.003), DARK, collide=False)]
    for k in range(n):
        a = k * 2 * math.pi / n
        c, s = math.cos(a), math.sin(a)
        r = (SEAT_R + CUP_R) / 2
        shapes.append(BoxShape((r * c, r * s, (CUP_FLOOR + FUNNEL_Z) / 2),
                               (CUP_R - SEAT_R, 2 * CUP_R * tan + 0.0005, FUNNEL_Z - CUP_FLOOR), STEEL, yaw=a))
        t = 0.002
        out = np.array([c, s, -1.0]) / math.sqrt(2.0)
        p1 = np.array([SEAT_R * c, SEAT_R * s, FUNNEL_Z]) + out * t / 2
        p2 = np.array([FUNNEL_R * c, FUNNEL_R * s, CUP_H]) + out * t / 2
        shapes.append(plate(tuple(p1.tolist()), tuple(p2.tolist()), 2 * (FUNNEL_R + t) * tan + 0.001, t,
                            (c, s, -1.0), STEEL))
        r = CUP_R - 0.00075
        shapes.append(BoxShape((r * c, r * s, (FUNNEL_Z + CUP_H) / 2), (0.0015, 2 * CUP_R * tan + 0.0005,
                                                                         CUP_H - FUNNEL_Z), STEEL, yaw=a))
    return Body.build("holder", pos=(HX, HY, TOP), mass=1.0, shapes=shapes, kinematic=True, color=STEEL,
                      friction=(0.15, 0.12), slick=True)


def _hose(i: int) -> Body:
    a, b = np.asarray(HOSE[i], float), np.asarray(HOSE[i + 1], float)
    shapes = [segment((0.0, 0.0, 0.0), tuple((b - a).tolist()), 0.012, BLACK)]
    ign = ("pump", "lever") if i == 0 else ("pump", "lever", f"hose{i - 1}")
    return Body.build(f"hose{i}", pos=tuple(a.tolist()), mass=0.012, shapes=shapes, color=BLACK,
                      damping=(0.6, 0.6), friction=(0.8, 0.7), ignore=ign)


def _coupler() -> Body:
    """A lock-on grease coupler. Its hardened nose (18 mm across, 12 mm tall) holds four jaws that, closed, form a
    7.2 mm mouth with a 45-degree lead-in, and 5 mm up a nitrile seal ring with a 4.6 mm hole whose edge sits on the
    ball head's top; above the nose a black-oxide 13 mm hex body (where it is held, with the thumb lever on one flat)
    and a brass swivel with the hose's side outlet. The mouth's walls, lead-in and seal ring are 12-sided rings of
    boxes; the jaws' lock is the ``jaws`` joint. Origin: the middle of its mouth."""
    n = 12
    tan = math.tan(math.pi / n)
    shapes = []
    for k in range(n):
        a = k * 2 * math.pi / n
        c, s = math.cos(a), math.sin(a)
        r = (BORE_R + NOSE_R) / 2  # the bore's wall, from the lead-in up to the nose's top
        shapes.append(BoxShape((r * c, r * s, (CHAMFER_D + NOSE_H) / 2),
                               (NOSE_R - BORE_R, 2 * NOSE_R * tan + 0.0005, NOSE_H - CHAMFER_D), None, yaw=a))
        r = (CHAMFER_R + 0.0001 + NOSE_R) / 2  # the face ring round the lead-in
        shapes.append(BoxShape((r * c, r * s, CHAMFER_D / 2),
                               (NOSE_R - CHAMFER_R - 0.0001, 2 * NOSE_R * tan + 0.0005, CHAMFER_D), None, yaw=a))
        t = 0.0015  # the lead-in: a cone wall from the face's inner edge to the bore
        out = np.array([c, s, 1.0]) / math.sqrt(2.0)
        p1 = np.array([CHAMFER_R * c, CHAMFER_R * s, 0.0]) + out * t / 2
        p2 = np.array([BORE_R * c, BORE_R * s, CHAMFER_D]) + out * t / 2
        shapes.append(plate(tuple(p1.tolist()), tuple(p2.tolist()), 2 * CHAMFER_R * tan + 0.0006, t,
                            (c, s, 1.0), None))
        r = (SEAL_HOLE_R + BORE_R + 0.0001) / 2  # the seal ring
        shapes.append(BoxShape((r * c, r * s, SEAL_Z + SEAL_T / 2),
                               (BORE_R + 0.0001 - SEAL_HOLE_R, 2 * (BORE_R + 0.0001) * tan, SEAL_T), None, yaw=a))
    shapes += [CylShape((0.0, 0.0, (SEAL_Z + SEAL_T + NOSE_H) / 2), BORE_R + 0.0002, NOSE_H - SEAL_Z - SEAL_T, None),
               CylShape((0.0, 0.0, NOSE_H / 2), NOSE_R + 0.0003, NOSE_H, ZINC, collide=False)]  # the nose, drawn
    shapes += _hexagon(BODY_AF, NOSE_H, BODY_Z1, DARK)
    shapes += [CylShape((0.0, 0.0, (BODY_Z1 + SWIVEL_Z1) / 2), SWIVEL_R, SWIVEL_Z1 - BODY_Z1, BRASS),
               CylShape((STUB_L / 2, 0.0, STUB_Z), STUB_R, STUB_L, BRASS, axis="X")]
    return Body.build("coupler", pos=(HX, HY, TOP + CUP_FLOOR), mass=COUPLER_KG, shapes=shapes, color=ZINC,
                      com=(0.0, 0.0, 0.020), friction=(1.2, 1.1), ignore=(f"hose{N_HOSE - 1}",))


def _thumb() -> Body:
    """The coupler's thumb lever: a steel pad on the hex body's +y flat, sprung 2.5 mm out from it."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (THUMB_W, THUMB_T, THUMB_H), STEEL)]
    return Body.build("thumb", pos=(HX, HY + THUMB_Y, TOP + CUP_FLOOR + THUMB_Z), mass=0.012, shapes=shapes,
                      color=STEEL, friction=(1.2, 1.1), ignore=("coupler", f"hose{N_HOSE - 1}"))


def _grease(k: int) -> Body:
    """A dab of grease (drawn only), parked under the table until it appears: the first four squeeze out of the pin's
    bore, the fifth shows a leak."""
    shapes = [SphereShape((0.0, 0.0, 0.0), 0.0060, GREASE, collide=False),
              SphereShape((0.006, 0.0, -0.002), 0.0050, GREASE, collide=False),
              SphereShape((-0.005, 0.0, -0.003), 0.0045, GREASE, collide=False)]
    return Body.build(f"grease{k}", pos=PARKED[k], mass=0.01, shapes=shapes, kinematic=True, color=GREASE)


OBJECTS = [_pivot(), _pump(), _lever(), _holder()] + [_hose(i) for i in range(N_HOSE)] + [_coupler(), _thumb()] + \
    [_grease(k) for k in range(5)]
MOVING = ["lever", "coupler", "thumb"] + [f"hose{i}" for i in range(N_HOSE)]
# the handle: lifted, it turns up about the pivot (+x); it rests down on its stop at 0; a damper for the grease
hinge("pump_lever", "pump", "lever", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(0.0, LEVER_OPEN),
      damping=LEVER_DAMP)
slider("thumb_lever", "coupler", "thumb", anchor=(HX, HY + THUMB_Y, TOP + CUP_FLOOR + THUMB_Z), axis="Y",
       limits=(-THUMB_TRAVEL, 0.0), stiffness=THUMB_K, damping=5.0, target=THUMB_PRELOAD, max_force=60.0)
# the jaws' lock: switched on when the coupler is pushed fully onto the ball with its thumb lever out; it then holds the
# coupler on the nipple's axis with its seal on the ball (turning only slowly against the jaws' drag; the swivel takes
# the hose's twist) until the thumb lever is pressed in
thread("jaws", "pivot", "coupler", anchor0=(0.0, 0.0, BALL_Z - TOP), axis0=(0.0, 0.0, 1.0),
       anchor1=(0.0, 0.0, BALL_IN_MOUTH - LOCK_LIFT), axis1=(0.0, 0.0, 1.0), slide_limits=(-0.0001, 0.0001),
       turn_damping=20.0, slide_stiffness=20000.0, slide_damping=50.0, slide_max_force=200.0)
ball("hose_out", "pump", "hose0", anchor=HOSE[0], swing_damping=0.002, twist_damping=0.002)
for _i in range(1, N_HOSE):
    ball(f"hose_j{_i}", f"hose{_i - 1}", f"hose{_i}", anchor=HOSE[_i], swing_damping=0.002, twist_damping=0.002)
ball("hose_in", f"hose{N_HOSE - 1}", "coupler", anchor=HOSE[N_HOSE], swing_damping=0.002, twist_damping=0.002)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"armed": False, "pushing": False, "peak": 0.0, "low": 0.0, "strokes": 0, "shots": 0, "leak": False,
            "seated_ever": False, "tcps": None, "s": {}, "locked": False, "locked_ever": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GREASE_DEBUG")), "steps": 0}


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


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _set_force(asset, f_body) -> None:
    import torch

    t = torch.tensor(np.asarray(f_body, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _coupler_state(cp, cr) -> dict:
    """Where the coupler is against the nipple's ball head, measured in the coupler's own frame: ``off`` how far the
    ball's centre is from its axis, ``lift`` how far its seal is off the ball's top (negative: pressed into it)."""
    q = cr.T @ (np.array([MX, MY, BALL_Z]) - cp)
    off = math.hypot(q[0], q[1])
    lift = BALL_IN_MOUTH - q[2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[2, 2])))))
    seated = off <= SEAT_OFF and -SEAT_SINK <= lift <= SEAT_LIFT and tilt <= SEAT_TILT
    on_ball = off <= PUSH_OFF and lift <= PUSH_LIFT and tilt <= PUSH_TILT
    in_holder = (math.hypot(cp[0] - HX, cp[1] - HY) <= IN_HOLDER_OFF and cp[2] <= TOP + CUP_FLOOR + IN_HOLDER_UP
                 and tilt <= IN_HOLDER_TILT)
    return {"off": off, "lift": lift, "tilt": tilt, "seated": seated, "on_ball": on_ball, "in_holder": in_holder}


def _show_leak(base, origin, cp) -> None:
    """Show the leaked grease: on the lug's top by the nipple when the coupler is at it, else under its mouth."""
    if math.hypot(cp[0] - MX, cp[1] - MY) <= LEAK_NEAR:
        spot = (MX + 0.010, MY, BOSS_TOP + 0.003)
    elif math.hypot(cp[0] - HX, cp[1] - HY) <= CUP_R:
        spot = (cp[0], cp[1], TOP + CUP_FLOOR + 0.003)
    else:
        spot = (cp[0], cp[1], TOP + 0.003)
    _place(base, origin, "grease4", spot)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    cp, cr = _pose(base, origin, "coupler")
    _, lr = _pose(base, origin, "lever")
    tp, _ = _pose(base, origin, "thumb")
    c = _coupler_state(cp, cr)
    thumb_in = THUMB_Y - float((cr.T @ (tp - cp))[1])  # how far the thumb lever is pressed in
    # the jaws: pushed fully on (sealed) with the thumb lever out, they lock behind the ball; the thumb lever pressed in
    # opens them
    if _EP["locked"] and thumb_in >= THUMB_OPEN:
        set_thread_on("jaws", False)
        _EP["locked"] = False
    elif not _EP["locked"] and thumb_in <= THUMB_SHUT and c["seated"]:
        set_thread_on("jaws", True)
        _EP["locked"] = _EP["locked_ever"] = True
    up = math.degrees(math.atan2(lr[2, 1], lr[1, 1]))  # degrees the handle is lifted off its stop
    # the plunger: lifting the handle draws grease in; pushing it back down by PRESSURE_DEG from the highest point it
    # was lifted to drives the plunger and pushes grease out, until the handle is down on its stop (or lifted again)
    if _EP["pushing"]:
        _EP["low"] = min(_EP["low"], up)
        if up >= _EP["low"] + PRESSURE_DEG:
            _EP["pushing"], _EP["peak"] = False, up
    else:
        _EP["peak"] = max(_EP["peak"], up)
        if up <= _EP["peak"] - PRESSURE_DEG:
            _EP["pushing"], _EP["low"] = True, up
    pressure = _EP["pushing"] and up > REST_DEG
    # while the gun pushes grease, its pressure on the seal pushes a coupler on the ball back off along its axis (the
    # locked jaws hold it on)
    _set_force(base.scene["coupler"], (0.0, 0.0, PUSH_N) if pressure and c["on_ball"] else (0.0, 0.0, 0.0))
    if pressure and not c["seated"] and not _EP["leak"]:  # grease squirts out of an unsealed coupler: a leak
        _EP["leak"] = True
        _show_leak(base, origin, cp)
    if not _EP["armed"] and up >= STROKE_TOP:
        _EP["armed"] = True
    elif _EP["armed"] and up <= STROKE_BOTTOM:  # a stroke: lifted past STROKE_TOP and pushed back down
        _EP["armed"] = False
        _EP["strokes"] += 1
        if c["seated"] and not _EP["leak"]:  # pushed through a sealed seat, it drives grease through the pin's bore
            _EP["shots"] += 1
            if _EP["shots"] <= len(GREASE_SPOTS):
                _place(base, origin, f"grease{_EP['shots'] - 1}", GREASE_SPOTS[_EP["shots"] - 1])
    _EP["seated_ever"] = _EP["seated_ever"] or c["seated"]
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = dict(c, up=up)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0 or (pressure and c["off"] < 0.01) or (c["off"] < 0.003 and c["lift"] < 0.003):
            tl, trc = _EP["tcps"]
            print(f"[grease] t={_DEBUG['steps'] / 20:5.2f}s handle={up:5.1f}deg pushing={pressure} "
                  f"strokes={_EP['strokes']} "
                  f"shots={_EP['shots']} leak={_EP['leak']} coupler={np.round(cp * 1000, 1).tolist()} "
                  f"off={c['off'] * 1000:5.2f}mm lift={c['lift'] * 1000:6.2f}mm tilt={c['tilt']:5.1f} "
                  f"seated={c['seated']} on_ball={c['on_ball']} in_holder={c['in_holder']} "
                  f"thumb={thumb_in * 1000:4.2f}mm locked={_EP['locked']} "
                  f"turn={math.degrees(math.atan2(cr[1, 0], cr[0, 0])):6.1f}deg "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000, 1).tolist()} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the pin greased (at least four strokes of the gun's handle, each lifted past STROKE_TOP and pushed
    back down within STROKE_BOTTOM through a sealed seat, and no leak all trial), the coupler back in its holder cup and
    both arms back where they started; 0.5 with the pin greased at some point and no leak; 0.25 with the coupler sealed
    on the nipple at some point; else 0.0. A leak (the handle pushed down PRESSURE_DEG or more from where it was lifted
    to, off its stop, at any moment while the coupler is not sealed on the nipple) is latched and removes the greasing
    credit."""
    if _EP["shots"] >= STROKES and not _EP["leak"]:
        if _EP["s"].get("in_holder") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["seated_ever"] else 0.0


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
    """Full score with the coupler, the gun's handle and the hose still: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[name].resting() for name in MOVING)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down round the coupler's hex body in its holder, fingers across the thumb lever (pressing it in,
# so the jaws are open), takes it, lifts it out, carries it over the nipple, lowers it onto the ball head and presses
# it on (aiming PRESS below the seat, so the soft arm leans about 12 N on it). It eases off to a light lean and lets go:
# the thumb lever springs out and the jaws lock behind the ball. It rises clear and waits. The left hand comes down
# round the gun handle's grip and closes on it, then works the handle five strokes: lifts it 25 degrees and pushes it
# back down onto its stop, turning the wrist with it; then it lets go, rises and goes back to where it started. The
# right comes back down round the coupler, closes across the thumb lever (the jaws open), pulls the coupler off,
# carries it back over the holder, steadies it just above it, lowers it in and presses it into the seat, eases off,
# lets go and goes back to where it started.

GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the hex body's flats and the thumb lever)
GRIP_TCP = 0.027  # the TCP this far above the coupler's mouth when holding it by its hex body
PART_OPEN = -0.020
AIM = (-0.002, 0.0)  # aim the held coupler's mouth this far off the nipple's axis (measured grip offset)
PRESS = 0.030  # aim this far below the seat: the soft arm (~420 N/m) then presses the coupler on with ~12 N
EASE = 0.008  # then this far below it (a ~3 N lean) while letting go
CLEAR = 0.06  # how far the right hand rises off the locked coupler while the left pumps
SET_DOWN = 0.006  # aim the coupler's mouth this far below the holder's floor (a ~3-8 N push: it pushes a hose
# lying under it aside and slides the nose down the funnel into the seat)
PUMPS = 5
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


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


GRIP_L = grip_rotation(math.pi / 2)  # straight down, fingers closing along x, across the handle's grip
HANDLE_OPEN = -0.020
LIFT = 25.0  # degrees the left hand lifts the handle each stroke
PRESS_DEG = -3.0  # ... and where it aims when pushing it back down (below the stop, so it is pushed onto it)


def _handle(up: float, dz: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Where the left TCP holds the handle's grip (GRIP_R along it, ``dz`` above its middle) with the handle lifted
    ``up`` degrees, and the grip rotation turned with it."""
    r = _rx(math.radians(up))
    return PIVOT + r @ np.array([0.0, GRIP_R, dz]), r @ GRIP_L


def _right(regrip_at: float) -> tuple[list[Waypoint], float]:
    """The right hand; returns its waypoints and when it has let go of the locked coupler and risen clear."""
    take = np.array([HX, HY, TOP + CUP_FLOOR + GRIP_TCP])
    out = _lead_in("right", take + [0.0, 0.0, 0.05], GRIP, "over the coupler in its holder", PART_OPEN)
    carry_z = NIPPLE_TOP + 0.030 + GRIP_TCP
    over = np.array([MX + AIM[0], MY + AIM[1], carry_z])
    on = np.array([MX + AIM[0], MY + AIM[1], MOUTH_SEATED + GRIP_TCP + 0.002])
    press = on - [0.0, 0.0, PRESS + 0.002]
    ease = on - [0.0, 0.0, EASE + 0.002]
    clear = on + [0.0, 0.0, CLEAR]
    grip = on - [0.0, 0.0, 0.002]
    rim = np.array([HX, HY, TOP + CUP_H + 0.012 + GRIP_TCP])  # the coupler's mouth 12 mm over the holder's rim
    out += [_w(1.0, take, PART_OPEN, "down round its hex body, across the thumb lever", GRIP),
            _w(0.6, take, FINGER_CLOSED, "take it (the thumb lever pressed in: jaws open)", GRIP),
            _w(1.0, [HX, HY, carry_z], FINGER_CLOSED, "lift it out of the holder", GRIP),
            _w(1.6, over, FINGER_CLOSED, "carry it over the grease nipple", GRIP),
            _w(1.4, on, FINGER_CLOSED, "lower it over the ball head", GRIP),
            _w(0.8, press, FINGER_CLOSED, "push it fully on", GRIP),
            _w(0.4, press, FINGER_CLOSED, "hold it on a moment", GRIP),
            _w(0.5, ease, FINGER_CLOSED, "ease off to a light lean", GRIP),
            _w(0.6, ease, PART_OPEN, "let go: the thumb lever springs out and the jaws lock", GRIP),
            _w(0.8, clear, PART_OPEN, "rise clear of the locked coupler", GRIP)]
    released_at = sum(w.duration for w in out)
    out += [_w(max(regrip_at - released_at, 0.5), clear, PART_OPEN, "wait while the other hand pumps", GRIP),
            _w(1.2, grip, PART_OPEN, "come back down round the coupler", GRIP),
            _w(0.6, grip, FINGER_CLOSED, "close across the thumb lever: the jaws open", GRIP),
            _w(0.8, over, FINGER_CLOSED, "pull it off the nipple", GRIP),
            _w(1.6, [HX, HY, carry_z], FINGER_CLOSED, "carry it back over the holder", GRIP),
            _w(1.0, rim, FINGER_CLOSED, "down to just above the holder", GRIP),
            _w(0.5, rim, FINGER_CLOSED, "steady it there", GRIP),
            _w(1.0, take - [0.0, 0.0, SET_DOWN], FINGER_CLOSED, "lower it into the holder", GRIP),
            _w(0.4, take - [0.0, 0.0, SET_DOWN], FINGER_CLOSED, "press it into the seat", GRIP),
            _w(0.4, take + [0.0, 0.0, 0.002], FINGER_CLOSED, "ease off", GRIP),
            _w(0.5, take + [0.0, 0.0, 0.002], PART_OPEN, "let go", GRIP),
            _w(0.8, take + [0.0, 0.0, 0.05], PART_OPEN, "rise", GRIP)]
    out += _go_home("right", take + [0.0, 0.0, 0.05], GRIP, PART_OPEN)
    return out, released_at


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    """The left hand; returns its waypoints and when it is done pumping."""
    over, rot0 = _handle(0.0, 0.05)
    out = _lead_in("left", over, rot0, "over the gun handle's grip", HANDLE_OPEN)
    out.append(_w(max(start - LEAD_S, 0.3), over, HANDLE_OPEN, "wait for the coupler to be locked on", rot0))
    grip, _ = _handle(0.0, 0.002)
    out += [_w(0.8, grip, HANDLE_OPEN, "down round the grip", rot0),
            _w(0.6, grip, FINGER_CLOSED, "close on it", rot0)]
    for _ in range(PUMPS):
        for k in range(1, 5):
            p, r = _handle(LIFT * k / 4, 0.002)
            out.append(_w(0.35, p, FINGER_CLOSED, "lift the handle (draws grease into the gun)", r))
        for k in range(3, -2, -1):
            p, r = _handle(max(LIFT * k / 4, PRESS_DEG), 0.002)
            out.append(_w(0.35, p, FINGER_CLOSED, "push it back down (pushes grease into the pin)", r))
    out += [_w(0.5, grip, FINGER_CLOSED, "hold it down on its stop", rot0),
            _w(0.5, grip, HANDLE_OPEN, "let go", rot0),
            _w(0.8, over, HANDLE_OPEN, "rise off the handle", rot0)]
    done = sum(w.duration for w in out)
    out += _go_home("left", over, rot0, HANDLE_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, HANDLE_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, done


def demo():
    _, released_at = _right(0.0)
    _, pumped_at = _left(released_at + 0.3, 0.0)
    right, _ = _right(pumped_at + 0.5)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, PART_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(released_at + 0.3, total)
    return {"left": left, "right": right}
