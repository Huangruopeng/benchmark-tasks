# Fit the blue suction-cup tool to a robot's manual quick-change flange and lock it on

A robot arm, switched off for a tool change, reaches its wrist in low over the table, its tool flange facing the right
arm. It has just parked its gripper tool in the aluminium tool rack beside it; in the rack's other slot waits a blue
suction-cup tool. One hand takes the suction tool by its handle and holds its plate on the flange's two locating
dowels, its three lugs between the three claws of the flange's locking ring; the other presses the ring's lever down,
turning the ring until its claws have taken the plate, drawn it onto the flange and locked it, and the light on the
wrist turns from red to green. Then both hands let go, the tool hanging on the robot, and both arms go back to where
they started. The task is one task file,
[`lock_a_tool_onto_a_robots_quick_change_flange.py`](lock_a_tool_onto_a_robots_quick_change_flange.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a part in place against gravity with one hand while the other works a lock; picking the right
tool; finishing with both hands free.

Why two hands: nothing holds the tool on the dowels' tips until the ring has taken it, and the ring takes only a tool
that is already seated on the dowels while the ring is still open. With one hand (the right taking the tool and holding
it on the dowels' tips as in the demo, the left never moving) the demo scored 0.25: the tool sat seated with the ring
open for 6 s, and fell 54 mm as soon as the hand let go. (A real plate could also be pushed fully onto its dowels, where
it would hang; see "Simulation approximation".)

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The robot arm and the tool rack are kinematic, fixed
where they are put; the ring with its lever and the two tools are loose rigid bodies made of shapes from `bodykit.py`.
Every changer size below is a constant in the task file.

- The ring: a hinge about the flange's axis (a `Hinge` from `bodykit.py`), turned the locking way by pressing its lever
  down, between its stops at 1 degree past open and at 35 degrees. Ring and lever weigh 661 g with their centre of
  mass 10.8 mm toward the lever, so the lever's weight (0.07 N m) turns the ring 1.2 degrees at rest, as on the real
  one. A spring (0.06 N m per degree) holds the ring where it last stopped; after every control step, a ring pushed
  more than 2 degrees past its hold has given, and the task file moves the hold there. So the ring gives at 0.12 N m
  and stays where it is left, standing in for the wave washer's friction.
- The claws, lugs, dowels, bushings, screw counterbores, retaining disc and marks are drawn at their build sizes, but
  they collide with nothing: the tools pass through the ring. The tool plates have no bores, so in place of the dowels'
  tips two collision-only stops (`dowel_stops`), 6 mm across and 5.6 mm out of the face where the dowels stand (inside
  the drawn dowels), stop a plate pushed onto the dowels with their tips 0.4 mm into its bushings' chamfers. They are
  moved away while the ring holds a tool, and come back once no tool is where they stand.
- The lock: after every control step the task file checks each tool against the flange (`_seat_check`). It is seated
  when both dowels fit in its bushings at their full size, in a pose the real parts allow, every lug front is within
  the lips' reach and, to go on, its lugs are between the lips (see "What the dowels capture" below). A tool seated
  while the ring is within 8 degrees of open is taken as the ring passes 8 degrees, where the lips come over the lugs:
  a fixed joint (a `Glue` from `bodykit.py`, to a hidden fixed body under the table) switches on and holds the tool
  drawn home onto the face, centred and square. From 25 degrees it counts as locked, and the green light is moved over
  the red one. Turned back below 25 degrees, the tool no longer counts as locked and the light goes red. The joint
  switches off once the ring is back within 8 degrees of open, where the lugs line up with the gaps between the claws.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Changer design (buildable)

The changer is a manual locking-ring changer for small arms, a breech-lock. A ring with three claws turns on the
robot's flange, and the cam faces on the claws' lips hook three lugs on the tool plate and pull the plate onto the
flange's face. Every size is the one the simulation uses (the task file's constants). Rows marked "build only" are
parts the simulation does not draw.

Conventions:

- Positions along the axis are measured from the flange's mating face, positive into the robot. Negative means in front
  of the face, toward the tool.
- Angles round the axis are seen facing the flange from the tool side, counterclockwise from 12 o'clock. That is the
  way the ring turns when the lever is pressed down. "Out" is the distance from the axis.

### Section along the axis (above the axis)

| Along the axis (mm) | Out (mm) | Part |
|---|---|---|
| -11.0 to +4.0 | 38 to 44 | Claw walls (3 x 80 degrees of arc), one piece with the ring |
| -11.0 to -2.8 | 32 to 38 | Claw lips. Their back faces are the cams |
| -8.0 to 0 | 0 to 30 | Tool plate |
| -3.0 to 0 | 30 to 36 | The tool plate's lugs (3 x 24 degrees) |
| -6.0 to 0 | 22 (6 mm pins) | Dowels standing out of the face |
| 0 to 3.0 | 0 to 35 | The flange's front lip. Its front is the mating face |
| 3.0 to 4.0 | 32.25 to 34.75 | Wave washer (build only) |
| 3.0 to 22.0 | 0 to 32 | The flange's journal |
| 4.0 to 22.0 | 32 to 44 | Locking ring |
| 22.0 to 26.0 | 0 to 38 | Retaining disc |
| 26.0 | | The robot wrist's mechanical interface (ISO 9409-1-50-4-M6) |

### Round the axis (ring open)

| Angle | Robot side | Tool side |
|---|---|---|
| 0 (12 o'clock) | Round dowel (22 mm out); stop pin (35 mm out, in the retaining disc); the ring's index line; OPEN mark | Lug; bushing for the round dowel |
| 8 | Mark: end of the open window | |
| 25 | LOCK mark | |
| 45, 135, 225, 315 | M6 mounting screws on a 50 mm circle | |
| 60, 180, 300 (10, 6 and 2 o'clock) | Claw centres. Each claw spans 40 degrees either side (20 to 100, 140 to 220, 260 to 340), leaving 40-degree gaps centred on 0, 120 and 240 | |
| 90 (9 o'clock) | Lever | |
| 120, 240 (8 and 4 o'clock) | | Lugs |
| 180 (6 o'clock) | Diamond dowel (22 mm out) | Bushing for the diamond dowel |

### Parts

| Part | Size and shape | Fit, fixing, material |
|---|---|---|
| Flange body | Front lip 70 mm across, 3 mm thick. Journal 64 mm across, 3 to 22 mm behind the face. Four 6.4 mm holes with 11 x 4 mm counterbores, on a 50 mm circle at 45, 135, 225 and 315 degrees. A 31.5 mm spigot on its back | C45 steel, black oxide, face ground flat; journal g6. Bolted, with the retaining disc, to the wrist's ISO 9409-1-50-4-M6 interface by four M6 x 30 low-head cap screws (DIN 7984), centred by the spigot and a 6 mm pin |
| Dowels | Two, 6 mm across and 14 long, standing 6 mm out of the face, 22 mm out at 0 and 180 degrees (44 mm apart). Ends as ISO 8734 for a 6 mm pin (lead-in c about 1.2 mm). The 180-degree one is a diamond pin: two 2 mm lands facing 90 and 270 degrees, so it fixes only the turn | Hardened (ISO 8734, m6), pressed 8 mm into reamed 6 mm H7 holes |
| Locking ring | 88 mm across, 18 mm deep (4 to 22 mm behind the face), 64 mm bore | C45 steel, black oxide, machined in one piece with its claws. Bore H7 on the g6 journal: 0.010 to 0.059 mm running clearance, greased |
| Claws (3) | Centred on 60, 180 and 300 degrees with the ring open, each 80 degrees of arc. Wall 38 to 44 mm out, from the ring's front (4 mm behind the face) to 11 mm in front of the face. Lip 32 to 38 mm out at the front. The lips' inner front edges are chamfered 4 x 45 degrees: a funnel from 72 down to 64 mm across | |
| Cam face (each lip's back) | Starting from the lip's leading end (at 100, 220 and 340 degrees, the end that moves toward a lug when locking): (1) a 1 x 45 degree chamfer; (2) a lead-in ramp over 12 degrees of arc, from 9.0 to 3.6 mm in front of the face (5.4 mm rise, 37 degrees at 34 mm out); (3) a clamp ramp over 5 degrees, from 3.6 to 2.8 mm (0.8 mm rise, 15 degrees); (4) a flat dwell at 2.8 mm for the remaining 63 degrees | A seated plate's lugs present their front faces 3.0 mm in front of the face, so the dwell overlaps them by 0.2 mm. The ring moves 0.2 mm forward against the wave washer, which sets the clamp at about 16 N |
| Wave washer (build only) | Single turn, 4 waves, 0.45 mm spring steel, 64.5 mm inside and 69.5 mm outside, free height 1.75 mm | Between the lip's back (3 mm behind the face) and the ring's front (4 mm): 1.0 mm high, about 12.5 N. With a tool clamped, 0.8 mm high, about 16 N |
| Retaining disc | 76 mm across, 4 mm thick (22 to 26 mm behind the face). Four 6.6 mm holes, a 31.5 mm bore. Marks on its rim at 0 (OPEN), 8 and 25 degrees (LOCK) | C45 steel, clamped under the mounting screws. Its front is the ring's rear thrust face |
| Stops (build only) | A stop pin, 4 mm across and 8 long, pressed 4 mm into the disc's front, 35 mm out at 0 degrees. A groove in the ring's back face, 4.2 mm wide and 4.5 deep, on a 35 mm radius, its end centres at -35 and +1 degrees (ring open) | Pin hardened (ISO 8734, m6). It meets the groove's ends at 1 degree past open and at 35 degrees |
| Lever | A 12 x 12 mm bar. A stub on the ring at 90 degrees, centred 13 mm behind the face, out to 64 mm from the axis. An arm continues it out to 104 mm. There a second arm, its centre line 110 mm from the axis, runs back along the axis for 62 mm (7 to 69 mm behind the face) | Stub: steel, welded into a 4 mm deep pocket in the ring. Arms: 6082-T6 aluminium, orange-anodised. Half-lapped onto the stub (12 x 6 mm, 20 mm long) with two M4 screws, and joined at the corner with two M4 screws |
| Pad | 45 x 30 x 8 mm, its middle 110 mm from the axis and 63 mm behind the face, its top 14 mm above the lever's line, with a 6 x 6 mm lip along its arm-side edge | A 2 mm aluminium plate under 6 mm of black nitrile rubber (60 Shore A) |
| Ring and lever | 661 g: ring with claws 583 g, lever and pad 79 g. Centre of mass 10.8 mm toward the lever | |
| Tool plate | 60 mm across, 8 mm thick (0 to 8 mm in front of the face). Mating face flat, its rim chamfered 1 x 45 degrees | 7075-T6 aluminium, clear-anodised. The tool body bolts to its front |
| Lugs (3) | At 0, 120 and 240 degrees, each 24 degrees wide (13.7 mm at 33 mm out). From the rim (30 mm) to 36 mm out, 3.0 mm thick, flush with the mating face | Machined in one piece with the plate; front corners chamfered 0.5 mm |
| Dowel bushings | Two, at 0 and 180 degrees, 22 mm out (44 mm apart) | Hardened steel, 10 mm n6 x 8, pressed into the plate. Bores 6 mm E8 (0.008 to 0.034 mm clearance on the m6 dowels), with 1.5 x 45 degree lead-in chamfers: 1.5 mm wide and 1.5 mm deep, a 9 mm mouth |
| Tools | Suction tool 120 g, gripper tool 140 g. Each has a 16 mm square black acetal handle on top, 50 mm from the plate's face | Thin-walled aluminium bodies |

### What the dowels capture

This is the simulation's seating rule, the same for both tools (`_seat_check` and `_dowel_fit` in the task file). It
accepts a plate pose only where the full-size dowels physically fit in the bushings, so the offset and tilt it allows
shrink as the dowels go in.

For each dowel:

- **d** is how far its tip (its deepest edge) has gone past the plate's face. For a square plate, the gap at the
  plate's middle is 6 mm - d.
- **e** is how far its axis is off the bushing's axis where it crosses the plate's face.
- **t** is its tilt against the bushing's axis (the plate's tilt).

At depth d the dowel's widest point reaches e + d tan t + 3 mm / cos t from the bushing's axis. The hole there has a
radius of 3 mm + 0.004 mm + (1.5 mm - d) in the 45 degree chamfer (d up to 1.5 mm), and 3 mm + 0.004 mm in the bore
beyond it. So the dowel is clear of its bushing while

> e + d tan t + 3 mm x (1/cos t - 1) <= 0.004 mm + max(0, 1.5 mm - d)

- **In the chamfer** the axis may be off by up to 1.5 - d mm (+ 0.004 mm): 1.104 mm at d = 0.4 mm, 0.504 mm at
  d = 1.0 mm.
- **In the bore** it may be off by only 0.004 mm, and the tilt is then held to about atan(0.004 mm / d): 0.05 degrees at
  d = 5 mm. So the plate must have been centred by the chamfers before the gap closes.
- **Every build:** 0.004 mm is the fit's smallest radial clearance. A 6 mm E8 bore on an m6 dowel gives 0.008 to
  0.034 mm on the diameter, so 0.004 to 0.017 mm on the radius. Both dowels are checked in every direction as full
  6 mm pins (the diamond pin's relief only adds room), and the dowels' own ISO 8734 end chamfers are left out. So every
  accepted pose fits every build within tolerance.

Seated also needs:

- both tips past the plate's face (d > 0 for both dowels);
- the plate's face nowhere past the flange's: its gap at the middle minus 36 mm x sin t is at least 0;
- every lug front within the lips' reach: gap + 3.0 mm + 36 mm x sin t is at most 10.0 mm. The lead-in ramps start at
  9.0 mm, and their 1 mm chamfer reaches 10.0 mm;
- to go on, its lugs between the lips: the plate's turn against the ring's, plus asin(offset / 30 mm), is within the
  8 degrees the gaps leave.

**The largest offset accepted for a square, unturned plate.** Each row is printed from the task file and checked
against a brute-force test. The test samples the dowel's surface every 1 degree and 0.1 mm, and measures each point
against the bushing's chamfer and bore at the tightest fit. At zero tilt the rule is exact: the clearance at the limit
is 0, and 0.01 mm more intersects.

| Gap at the middle (mm) | d (mm) | Largest offset e (mm) | Clearance at that offset (µm) | Clearance 0.01 mm further (µm) |
|---|---|---|---|---|
| 6.00 | 0 | none: the tips have not reached the plate | | |
| 5.90 | 0.10 | 1.404 | 0 | -10 |
| 5.75 | 0.25 | 1.254 | 0 | -10 |
| 5.60 | 0.40 | 1.104 | 0 | -10 |
| 5.50 | 0.50 | 1.004 | 0 | -10 |
| 5.25 | 0.75 | 0.754 | 0 | -10 |
| 5.00 | 1.00 | 0.504 | 0 | -10 |
| 4.75 | 1.25 | 0.254 | 0 | -10 |
| 4.60 | 1.40 | 0.104 | 0 | -10 |
| 4.50 | 1.50 | 0.004 | 0 | -10 |
| 4.00 | 2.00 | 0.004 | 0 | -10 |
| 1.00 | 5.00 | 0.004 | 0 | -10 |
| 0.00 | 6.00 | 0.004 | 0 | -10 |

**Tilted plates.** A sample of accepted poses. The plate is shifted sideways (x) by the offset. Tilt about x pitches
it, so its two dowels go in to different depths; tilt about z leans both dowels sideways in their bushings. "Need" is
the larger of the two dowels' left sides and "room" the smaller right side. The rule is conservative when tilted: the
measured clearance exceeds room - need.

| Gap (mm) | Offset (mm) | Tilt (degrees, about) | d top / bottom (mm) | Need / room (mm) | Clearance (µm) |
|---|---|---|---|---|---|
| 5.60 | 1.000 | 0.04 (x) | 0.39 / 0.42 | 1.0003 / 1.0865 | 88.6 |
| 5.60 | 0.500 | 1.00 (x) | 0.07 / 0.84 | 0.5151 / 0.6678 | 217.9 |
| 5.25 | 0.250 | 1.00 (x) | 0.42 / 1.19 | 0.2712 / 0.3178 | 117.2 |
| 5.25 | 0 | 1.50 (x) | 0.25 / 1.40 | 0.0453 / 0.0998 | 128.0 |
| 5.00 | 0.500 | 0.04 (z) | 1.00 / 1.00 | 0.5007 / 0.5023 | 5.8 |
| 5.00 | 0 | 1.00 (x) | 0.67 / 1.44 | 0.0289 / 0.0678 | 89.1 |
| 4.60 | 0.004 | 1.00 (z) | 1.45 / 1.45 | 0.0298 / 0.0519 | 80.8 |
| 4.50 | 0 | 0.04 (x) | 1.49 / 1.52 | 0.0011 / 0.0040 | 3.0 |
| 4.00 | 0 | 0.04 (x) | 1.99 / 2.02 | 0.0014 / 0.0040 | 2.6 |
| 1.00 | 0 | 0.04 (x) | 4.99 / 5.02 | 0.0035 / 0.0040 | 0.5 |

**The review's example.** A plate 1 mm from the face has its dowels 5 mm in (d = 5 mm), in the bore. Each dowel must
then be within 0.004 mm of its bushing's axis:

| Gap 1.00 mm: offset (mm), tilt (degrees) | Accepted | Need / room (mm) | Clearance (µm) |
|---|---|---|---|
| 1.000, 0 | no | 1.0000 / 0.0040 | -996 (intersects) |
| 0.100, 0 | no | 0.1000 / 0.0040 | -96 |
| 0.010, 0 | no | 0.0100 / 0.0040 | -6 |
| 0.003, 0 | yes | 0.0030 / 0.0040 | 1 |
| 0.003, 0.05 | no | 0.0074 / 0.0040 | -3.4 |

**Over all poses.**

- **Grid:** 20,520 poses (gap 0 to 6.25 mm, offset 0 to 1.6 mm along x or z, tilt 0 to 3 degrees about x or z, both
  ways). The rule accepts 4,248, and none of them intersects in the brute-force test; the smallest clearance among
  them is 0.08 µm. The 364 accepted poses with a dowel in the bore have offsets of at most 0.002 mm and tilts of at
  most 0.1 degrees. 449 grid poses that would fit are rejected: the rule is conservative.
- **Random:** 20,000 poses (gap 0 to 7 mm, offset up to 2 mm in any direction, tilted or turned up to 3 degrees about a
  random axis, ring at 0 to 8 degrees). The rule accepts 2,762, and none intersects.

**In the simulation**, the plate rests on the stops with the dowels' tips 0.4 mm into its chamfers, so each dowel may
be up to 1.104 mm off. In the demo run, while the plate sat there with the ring open, d was 0.30 to 0.40 mm, e at most
0.78 mm against at least 1.16 mm of room, and the tilt at most 0.17 degrees.

The rest of the changer agrees with this rule:

- **Claws:** a seated plate's lug fronts are within 10.0 mm of the face, where the lips' lead-in ramps (9.0 mm, 10.0 mm
  with their chamfer) catch them. In the simulation the plate rests 5.63 mm out, its lug fronts at 8.63 mm. The ramp
  falls 0.45 mm per degree (5.4 mm over 12 degrees), so it reaches them at 8.8 degrees.
- **The lips' funnel:** the plate's rim (30 mm out) clears the lips' inner edge (32 mm) at any offset under 2 mm.
  Seated offsets are at most 1.504 mm.
- **Lugs:** the 40-degree gaps between the lips leave 8 degrees either side of a 24-degree lug. A plate shifted by an
  offset e moves a lug 30 mm out round by asin(e / 30 mm): 2.9 degrees at 1.5 mm.

**Taking the tool.** With a tool seated and the ring within 8 degrees of open, turning the ring past 8 degrees brings
the lips' leading ends over the lugs, and the tool can no longer come off. On the real changer the lead-in ramps then
draw it home along the dowels by 20 degrees, the chamfers centring it to the bore's clearance on the way, and the clamp
ramps pull it tight by 25. The simulation does this at once at 8 degrees: the fixed joint holds the plate on the face,
centred and square.

### Lever movement

The pad's top middle at each ring angle, printed from the task file:

| Ring turned (degrees) | Pad top drop | Pad top height above the table | What happens |
|---|---|---|---|
| -1 | 1.9 mm up | 100.9 mm | Open stop |
| 0 (at rest it sits at 1.2, under the lever's weight) | 0 | 99.0 mm | Open: the lugs pass between the lips |
| 8 | 15.4 mm | 83.5 mm | The lips reach the lugs. From here a plate can no longer go on or come off |
| 8 to 20 | to 38.5 mm | to 60.5 mm | The lead-in ramps draw the plate home |
| 20 to 25 | to 47.8 mm | to 51.2 mm | The clamp ramps pull it tight: 0.2 mm, about 16 N |
| 25 to 35 | to 65.6 mm | to 33.4 mm | On the flat dwell: locked (LOCK mark at 25). Hard stop at 35 |

To release, lift the pad. Below 25 degrees the lugs ride down the clamp ramps and the plate is no longer pulled tight.
Below 8 degrees the lugs line up with the gaps and the tool comes off.

### Forces

These are estimates, for greased steel with friction about 0.15.

- **Holding the ring:** the wave washer's friction on the lip and on the disc holds the ring with about 0.13 N m, so it
  stays where it is left. The lever's weight gives 0.07 N m toward locking.
- **Turning it:** pressing down takes about 0.5 N on the pad, rising to about 2.2 N on the clamp ramps. Lifting takes
  about 1.8 N.
- **Holding the tool:** the 16 N clamp gives about 0.55 N m against tipping. A 120 g tool whose weight acts 50 mm out
  needs 0.06 N m.

On a real rig the status light is optional. Two inductive proximity switches can drive it: one sees the plate in the
face, the other a flag on the ring from 25 degrees. The trial is judged from the marks either way.

### Simulation approximation (as in the fidelity note)

- **No contact:** the claws, lips, lugs and dowels are drawn at these sizes but do not touch anything. The lips are
  drawn without their chamfers, and with the lead-in ramp as two steps.
- **Dowel tips:** the tool plates have no bores, so two collision-only stops, 6 mm across and 5.6 mm out of the face,
  stand in for the dowels' tips: a pushed plate rests on them with the tips 0.4 mm into its chamfers. A real plate
  pushed on harder goes past the chamfers, which centre it, and then hangs on its dowels without a hand. In the
  simulation it cannot be pushed past the tips.
- **Seating and lock:** seating is the geometric check above. Because the drawn dowels do not centre it, a held plate
  keeps whatever offset and tilt it has inside that check until the ring takes it. From 8 degrees a fixed joint holds
  the tool drawn home, centred and square, with no play. A real tool is drawn home by the ramps between 8 and 20
  degrees, and hangs with some play below 25 degrees.
- **Ring friction:** the wave washer's friction is the moving-hold spring (it gives at 0.12 N m). The clamp ramps' extra
  torque, up to about 0.2 N m, is not modelled.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the suction tool still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lock_a_tool_onto_a_robots_quick_change_flange.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the blue suction-cup tool to the robot's tool flange and lock it on, leaving the gripper tool in the rack. Then let go of everything, with the tool hanging locked on the robot, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flange-dream
```

The scripted demo reaches full score:

1. The right hand comes straight down on the suction tool's handle, fingers closing across it, and lifts it out of the
   rack.
2. It carries the tool to the flange and holds its plate on the dowels' tips (on the stops), its lugs between the
   claws. On the way it turns the wrist 3.4 degrees about the vertical, against the twist the finger pads give a held
   tool, and pitches it back 0.6 degrees, against the pitch the tool's weight and the push at the handle give it, so
   the plate meets the dowels square. It aims 4.5 mm higher, against the hold offset measured in debug runs, and 2.5 mm
   past the tips, since the arm stops short of a pushed target.
3. The left hand comes straight down on the lever's pad, fingers closed, and presses it down along its swing to 32
   degrees. Past 8 degrees the claws take the plate and draw it home; from 25 degrees it is locked, and the light turns
   green.
4. The right lets go and draws back, and the left rises. The tool hangs on the robot, and both arms go back to where
   they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lock_a_tool_onto_a_robots_quick_change_flange.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the blue suction-cup tool to the robot's tool flange and lock it on, leaving the gripper tool in the rack. Then let go of everything, with the tool hanging locked on the robot, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flange-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flange-dream/*.json; do inspect-robots video "$log"; done
```

Set `FLANGE_DEBUG=1` to print what the score is built from twice a second:

- how far the ring has turned;
- how far the suction tool is from sitting on the flange (gap, offset and its two parts, tilt and its two parts, turn)
  and, for each dowel, d, e, need and room as in "What the dowels capture";
- which seating checks pass, whether it is seated, seated with the ring open, or held by the ring, and whether the
  stops are away;
- whether the gripper tool is held by the ring;
- whether both hands are clear;
- where each TCP is from the tool's handle and the lever's pad.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (235 s on the clock, with Dream answering in 0.71 s on average), and about 1 minute for the scripted demo (394 steps, 19.7 s of simulated time, 71 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-Z5S8AY. Its review said:

> **Not accepted** (TK-Z5S8AY).
>
> Clarity and reproducibility (not met): The seating predicate accepts a 1 mm dowel offset at a 1 mm face gap even
> though the full-diameter dowel would intersect the bushing bore.
>
> What to change: The mechanism dimensions and fidelity note are retained, but the previous seating-window issue
> remains open. For example, the current window permits a 1 mm dowel offset when the plate is 1 mm from the flange; the
> 6 mm dowel then projects 5 mm into the plate, beyond the 1.5 mm lead-in and into the closely fitted bore. Couple the
> permitted offset and tilt to insertion depth so every accepted pose is physically achievable, update the setup and
> fidelity note, and rerun the full-score demo with the corrected predicate.

The review before that (TK-UU8KFN) asked for a seating window physically achievable with the specified dowels and
bushings; the previous revision tightened the window, and this one completes it by coupling it to depth. The review
before that (TK-TECUBS) asked for the dowel locations, the cam and lug profiles, and how the ring is mounted, retained
and stopped; that was done and is kept. The first review (TK-S6GB3S) asked for a buildable flange, tool plate and lock,
with the simulation approximation kept in the fidelity note; that is kept too.

What changed in response:

- **Clarity and reproducibility (not met), the seating predicate:** the predicate is now geometric and depends on
  depth. For each dowel, d is how far its tip has gone into the plate, e is how far its axis is off the bushing's, and
  t is the tilt. A pose is accepted only while e + d tan t + 3 mm x (1/cos t - 1) <= 0.004 mm + max(0, 1.5 mm - d).
  - In the 1.5 mm chamfer the allowed offset is the chamfer's radius at that depth, 1.5 - d mm, plus 0.004 mm.
  - In the bore it is 0.004 mm, the smallest radial clearance of the 6 mm E8/m6 fit (0.008 to 0.034 mm on the
    diameter). The tilt is then held to about atan(0.004 mm / d).
  - Both dowels are checked as full 6 mm pins in every direction, at the tightest fit, without their end chamfers.
  - It also needs both tips past the plate's face, the plate's face nowhere past the flange's, every lug front within
    the lips' 10.0 mm reach, and, to go on, the lugs between the lips.
- **Couple the permitted offset and tilt to insertion depth:**
  - **Largest offset:** 1.404 mm at d = 0.1 mm, 1.104 mm at 0.4 mm, 0.504 mm at 1.0 mm, 0.104 mm at 1.4 mm, and
    0.004 mm from 1.5 mm on.
  - **Tilt in the bore:** at most about 0.05 degrees at d = 5 mm.
  - **The review's example is rejected:** a plate 1 mm from the face (d = 5 mm) needs each dowel within 0.004 mm. A
    1 mm offset intersects by 0.996 mm.
- **Every accepted pose physically achievable, checked against the geometry:**
  - The README's "What the dowels capture" gives the rule, a table of the largest accepted offset at each gap, and
    tables of tilted samples and of the review's example. All are printed from the task file.
  - Every row is checked by a brute-force test: the dowel's surface, sampled every 1 degree and 0.1 mm, against the
    bushing's chamfer and bore.
  - At zero tilt the rule is exact: the clearance is 0 at the limit, and 0.01 mm further intersects by 10 µm.
  - Of 20,520 grid poses, 4,248 are accepted, and none intersects (smallest clearance 0.08 µm). Of 20,000 random poses
    (also turned, and with the ring at 0 to 8 degrees), 2,762 are accepted, and none intersects.
- **What the simulation does now:**
  - The drawn dowels cannot centre a plate, so an uncentred plate pushed past the chamfers is not achievable and is no
    longer accepted.
  - Two collision-only stops, 6 mm across and 5.6 mm out of the face, stand in for the dowels' tips. A pushed plate
    rests on them with the tips 0.4 mm into its chamfers.
  - When the ring passes 8 degrees, where the lips come over the lugs, it takes a tool seated there. A fixed joint
    holds the tool drawn home onto the face, centred and square, which a real ring's ramps do by 20 degrees. From 25
    degrees the tool counts as locked.
  - Before, the window was a fixed box (offset 1.5 mm, gap 5.25 mm, tilt 1.95 degrees, turn 3.9 degrees), and the tool
    locked at 25 degrees from wherever it was held.
- **Setup and fidelity note updated:**
  - The setup states the rule, the 10 mm lug reach and the lugs-between-the-lips condition, and the taking at 8
    degrees (3,786 characters).
  - The fidelity note (485 characters) states the depth-coupled check, the stops and the fixed joint from 8 degrees.
  - The README's "How it is modelled" and "Simulation approximation" describe the stops. They also state that a real
    plate pushed past the chamfers would be centred by them and hang on its dowels.
- **Full-score demo re-run under the corrected predicate:**
  - The holding wrist is also pitched back 0.6 degrees, against the measured pitch from the tool's weight and the push
    at the handle, so both dowel tips reach their chamfers.
  - While the plate sat on the tips with the ring open, d was 0.30 to 0.40 mm, e at most 0.78 mm against at least
    1.16 mm of room, and the tilt at most 0.17 degrees. The ring took the tool at 8 degrees and locked it.
  - The scripted demo scores 1.0 (394 steps). Dream-1 scores 0.0 (1,200 steps): it reaches toward the suction tool
    in the rack and does not lift it. The one-hand check scores 0.25: the tool sat seated on the dowels' tips with the
    ring open for 6 s, and fell when the hand let go.
