# Fit the blue suction-cup tool to a robot's manual quick-change flange and lock it on

A robot arm, switched off for a tool change, reaches its wrist in low over the table, its tool flange facing the right
arm. It has just parked its gripper tool in the aluminium tool rack beside it; in the rack's other slot waits a blue
suction-cup tool. One hand takes the suction tool by its handle and holds its plate against the flange, its three lugs
between the three claws of the flange's locking ring; the other presses the ring's lever down, turning the ring until
its claws have drawn the plate onto the flange and locked it, and the light on the wrist turns from red to green. Then
both hands let go, the tool hanging on the robot, and both arms go back to where they started. The task is one task
file, [`lock_a_tool_onto_a_robots_quick_change_flange.py`](lock_a_tool_onto_a_robots_quick_change_flange.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a part in place against gravity with one hand while the other works a lock; picking the right
tool; finishing with both hands free.

Why two hands: nothing holds the tool up on the flange until it is locked, and the ring takes only a tool that is
already on the flange while the ring is still open. With one hand (the right taking the tool and holding it on the
flange as in the demo, the left never moving) the demo scored 0.25: the tool sat on the flange with the ring
open, and fell to the table as soon as the hand let go.

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
  they collide with nothing: the tools pass through the ring.
- The lock: after every control step the task file checks each tool against the flange. It is seated when each dowel
  is within 1.5 mm of its bushing's axis where the dowel's axis meets the plate's face (the diamond pin across its
  lands only), the plate's face is within 5.25 mm of the flange's face at its middle, its axis is within 1.95 degrees
  of the flange's, and it is turned within 3.9 degrees of handle-up. That is the range the bushings' lead-ins capture
  (see "What the dowels capture" below). A tool seated while the ring is within 8 degrees of open is locked on once
  the ring has turned 25 degrees: a fixed joint (a `Glue` from `bodykit.py`, to a hidden fixed body under the table)
  switches on and holds the tool square on the flange, and the green light is moved over the red one. Turned back
  below 25 degrees, the tool no longer counts as locked and the light goes red. The joint switches off once the ring
  is back within 8 degrees of open, where the lugs line up with the gaps between the claws.

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

This is the simulation's seating window, the same for both tools. A plate counts as seated, and the ring can take it,
only where the real dowels would catch it: both dowels inside their bushings' lead-in chamfers (1.5 x 45 degrees,
1.5 mm wide and 1.5 mm deep). Two things would add margin and are not counted: the bores' running clearance (0.004 to
0.017 mm a side) and the dowels' own ISO 8734 lead-in (about 1.2 mm).

| Limit | Value | Where it comes from |
|---|---|---|
| Each dowel's offset from its bushing's axis, where the dowel's axis meets the plate's face | 1.5 mm | The chamfer's width. A dowel whose edge (3 mm from its axis) lands inside the chamfer's 9 mm mouth (4.5 mm from the bore's axis) is pushed to the middle as the plate goes on: 4.5 - 3.0 = 1.5 mm. The round dowel is checked in every direction, the diamond pin only across its lands (sideways): its relieved sides leave the other direction free |
| Turn from handle-up | 3.9 degrees | A turn moves each bushing 22 mm x sin(turn) sideways, so asin(1.5 / 22) = 3.91 degrees uses the whole 1.5 mm. A plate both turned and off-centre sideways must keep its sideways offset plus 22 mm x sin(turn) within 1.5 mm, since one of the two dowels sees the sum |
| Tilt of the plate's axis | 1.95 degrees | The bushings are 44 mm apart. Tilted, one is 44 mm x sin(tilt) nearer the face than the other, and pushed on, it meets its dowel first. Both dowel tips are inside their 1.5 mm deep chamfers at once only while that difference is at most 1.5 mm: asin(1.5 / 44) = 1.95 degrees. Tilted more, the nearer dowel enters its bore proper, where the 0.008 to 0.034 mm clearance lets the plate tilt hardly at all, before the farther one is caught, and binds |
| Gap from the plate's face to the flange's face, at its middle | 5.25 mm | The dowels stand 6 mm out of the face. Tilted 1.95 degrees, the farther bushing (22 mm out) is 22 x sin(1.95 degrees) = 0.75 mm farther out than the middle: 6 - 0.75 = 5.25 mm, where both dowel tips have reached their chamfers |

With the plate square and not turned, the first limit puts its middle within 1.5 mm of the axis.

The rest of the changer agrees with this window:

- **Claws:** at the window's edge, the farthest lug corner (36 mm out) stands at most 5.25 + 36 x sin(1.95 degrees) +
  3.0 = 9.5 mm in front of the face. The lips' lead-in ramps start at 9.0 mm, and their 1 x 45 degree chamfer reaches
  10.0 mm, so they catch every lug.
- **The lips' funnel:** the plate's rim (30 mm out) clears the lips' inner edge (32 mm) at any offset under 2 mm. The
  funnel in front (4 x 45 degrees, from 72 down to 64 mm across) is only a guard: it brings a plate up to 6 mm off down
  to 2 mm. Counting their own 1.2 mm lead-ins, the dowels of a real changer catch about 1.5 + 1.2 = 2.7 mm, more than
  the funnel leaves; the window counts only the 1.5 mm.
- **Lugs:** the 40-degree gaps between the lips leave 8 degrees either side of a 24-degree lug. Inside the window a lug
  moves round at most 3.9 degrees from the turn plus 2.9 degrees from a 1.5 mm offset 30 mm out: 6.8 degrees.

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
- **Seating and lock:** seating is a pose check over the bushings' capture range above. Because the dowels do not
  touch, a held plate inside that range keeps whatever offset and tilt it has (up to those limits) until the lock
  pulls it square; a real plate is centred and squared by the dowels and the face as it goes on. The lock is a fixed
  joint, switched on when the ring reaches 25 degrees with a tool seated. It holds the tool square until the ring is
  back within 8 degrees of open; a real tool hangs there with some play once the ring is below 25 degrees.
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
2. It carries the tool to the flange and holds its plate against the flange's face, between the claws. On the way it
   turns the wrist 3.4 degrees about the vertical, against the twist the finger pads give a held tool, so the plate
   meets the flange square, and it aims 4.5 mm higher, against the hold offset measured in debug runs.
3. The left hand comes straight down on the lever's pad, fingers closed, and presses it down along its swing to 32
   degrees. The claws draw the plate on and lock it, and the light turns green.
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
- how far the suction tool is from sitting on the flange (gap, offset and its two parts, each dowel's offset from its
  bushing, tilt and its two parts, turn);
- whether it is seated with the ring open, and whether it is held by the lock;
- whether the gripper tool is held by the lock;
- whether both hands are clear;
- where each TCP is from the tool's handle and the lever's pad.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (243 s on the clock, with Dream answering in 0.76 s on average), and about 1 minute for the scripted demo (394 steps, 19.7 s of simulated time, 73 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-UU8KFN. Its review said:

> **Not accepted** (TK-UU8KFN).
>
> Clarity and reproducibility (not met): The claimed 6 mm lateral seating window exceeds what the specified 6 mm
> dowels and 1.5 mm bore lead-ins can capture.
>
> What to change: The revision supplies the requested mechanism dimensions and retains the simulation fidelity note.
> One point remains open: the stated 6 mm lateral seating range is not justified by the 6 mm dowels and 1.5 mm bore
> lead-ins, so the simulated lock can engage from a pose the physical dowels cannot capture. Please set the seating
> window to a physically achievable combination of lateral offset, tilt and face gap for the specified dowels and
> bushings, update the description and fidelity note accordingly, and rerun the full-score demo.

The review before that (TK-TECUBS) asked for the dowel locations, the mating cam and lug dimensions and profiles, and
how the ring is mounted, retained and stopped; that was done in the previous revision and is kept. The first review
(TK-S6GB3S) asked for a buildable flange, tool plate and lock, with dimensions and mounting, and the simulation
approximation kept in the fidelity note; that is kept too.

What changed in response:

- **Clarity and reproducibility (not met), the seating window:** it now follows from the dowels and bushings and is
  exactly what the scorer checks (constants in the task file computed from the dowel and bushing sizes). The window
  counts only the bushings' 1.5 x 45 degree lead-in chamfers, 1.5 mm wide and 1.5 mm deep. It leaves out the bores'
  0.004 to 0.017 mm running clearance and the dowels' own ISO 8734 lead-in (about 1.2 mm).
- **Lateral offset: 1.5 mm, was 6 mm.** Each dowel must be within 1.5 mm of its bushing's axis where it meets the
  plate's face: its edge, 3 mm from its axis, inside the chamfer's 4.5 mm mouth radius. The round dowel is checked in
  every direction, the diamond pin across its lands. So a square, unturned plate's middle is within 1.5 mm of the axis.
  Being checked at each dowel, it includes the turn: a turn moves each bushing 22 mm x sin(turn) sideways.
- **Tilt: 1.95 degrees, was 9.** The bushings are 44 mm apart. Both dowel tips are inside their 1.5 mm deep chamfers
  at once only while the tilt puts one bushing at most 1.5 mm nearer the face than the other: asin(1.5 / 44) = 1.95
  degrees.
- **Face gap: 5.25 mm at the plate's middle, was 6 mm.** The dowels stand 6 mm out. Tilted 1.95 degrees, the farther
  bushing is 22 x sin(1.95 degrees) = 0.75 mm farther out, so 6 - 0.75 = 5.25 mm keeps both tips in their chamfers.
- **Turn: 3.9 degrees, was 8:** asin(1.5 / 22) = 3.91 degrees.
- **Consistency checks:** at the window's edge the farthest lug stands at most 9.5 mm in front of the face, where the
  claws' lead-in ramps (9.0 mm, 10.0 mm with their chamfer) catch it. The lugs stay inside the 8 degrees they have in
  the claws' gaps (at most 3.9 + 2.9 = 6.8 degrees). With their own 1.2 mm lead-ins, the real dowels catch about
  2.7 mm, more than the 2 mm the lips' funnel leaves. The dowel ends are now given as the ISO 8734 standard ones (the
  last revision said a 1.5 mm rounded nose).
- **Description and fidelity note updated:** the setup now states the window and the bushings' bores and chamfers. The
  README has a new "What the dowels capture" table with each limit and its arithmetic, replacing "What the lead-ins
  take". The fidelity note (489 characters) states the window and adds that a held plate is not centred or squared by
  the dowels in the simulation.
- **Full-score demo re-run:** the hand that holds the tool now turns its wrist 3.4 degrees about the vertical against
  the finger pads' twist, so the plate meets the flange square. Its sideways aim moved from 3.5 mm to 0 to match. In a
  debug run, from seating until the lock engaged, each dowel was at most 0.77 mm from its bushing's axis, the tilt at
  most 0.41 degrees, the turn at most 0.47 degrees and the gap 0.2 to 1.0 mm. The scripted demo scores 1.0 (394
  steps); Dream-1 scores 0.0 (it reaches for the tool in the rack and does not lift it). The one-hand check still scores
  0.25: the tool is seated with the ring open and falls when the hand lets go.
