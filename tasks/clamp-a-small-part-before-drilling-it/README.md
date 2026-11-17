# Drill a 5 mm hole through a small aluminium plate at its punch mark on a bench drill press

An 8-inch bench drill press stands on the table. Its table carries a waxed, melamine-faced MDF spoil board, and on that
lies a loose aluminium plate with its punch mark 20 mm in front of the bit. A screw hold-down clamp stands beside the
plate: an aluminium arm on a riser, reaching over the plate's front-left part, with a star-knob screw through it whose
rubber tip, bonded to the screw, is one turn above the plate. The drill is a conventional one: the motor has its own
push-button switch (green start, red stop) on the front of the head and runs whether or not the quill moves, and the
quill is fed down by a three-spoke capstan handle on the right of the head, through a pinion on the quill's rack,
against a return spring. The instruction gives only the goal: drill right through the plate at the mark and leave the
drill switched off with its quill up, with the plate not moving and no part of either gripper within 5 cm of the bit or
the chuck while they turn. The policy has to work out the way: slide the plate until its mark is under the bit; hold it
there while the other hand turns the clamp's knob down onto it and on until the tip presses hard; then, with both hands
clear, switch the motor on, pull the feed handle down until the bit is through the plate and into the spoil board, let
the handle back up, and switch the motor off. The task is one task file,
[`clamp_a_small_part_before_drilling_it.py`](clamp_a_small_part_before_drilling_it.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests. Holding a part with one hand while the other tightens something on it is one step of this job, not the
whole of it:
- placing the part on the drill's table with its punch mark under the bit, to within 5 mm, and keeping it there while
  the clamp's turning rubber tip drags at it: the part has to stay undisturbed while the clamp is seated on it;
- tightening to a real clamping force: the knob is turned a full turn down onto the plate and then on, against a
  resistance that grows as the rubber compresses, until the tip presses at least 85 N (about 0.5 N m at the knob);
- running a real drill press in the order its safety needs, with both grippers kept 5 cm from the turning chuck: clamp
  first, start the motor, feed the bit through against the cutting thrust with the capstan, let it up, stop the motor.
  A part that is not clamped is caught by the bit and flung, a common drill-press injury.

Why two hands: the clamp's rubber tip turns with its screw and, once it presses, drags the plate round with it on the
slick spoil board, off its mark, unless the plate is held. In the one-hand check the right hand slid the plate into
place exactly as in the demo and went home before the tip came down, and the left tightened the clamp exactly as in the
demo, with nobody holding the plate: it scored 0.25. The tip turned the plate 5.3 degrees as it tightened and clamped it
with its mark 8.3 mm from the bit's axis (the limit is 5 mm). In the demo, with the right hand pressing the plate while
the left tightens, the mark is 3.3 mm from the axis when the bit goes through. One hand places and holds the plate while
the other tightens the clamp.

## The drill press, for a real trial

Any 8-inch (203 mm swing) bench drill press of the common 1/3 hp, five-speed class can be used, set up as below. The
simulated one has these dimensions, measured above the bench and from the spindle's axis.

| Part | Simulated (and what to set on a real one) |
|---|---|
| Base | cast iron, 290 x 190 x 40 mm, its front 16.5 cm ahead of the spindle |
| Column | 46 mm steel, 10.5 cm behind the spindle, 47 cm tall |
| Head | 14 cm wide, from 33.5 to 47.5 cm above the bench, its front 5.5 cm ahead of the spindle; belt cover on top to 55.5 cm, motor behind |
| Chuck and bit | 13 mm keyed chuck (JT33), 46 mm across and 76 mm long; a 5 mm high-speed-steel jobber drill 55 mm out of the chuck |
| Table | cast iron, 160 x 160 mm, raised until its top is 16.0 cm above the bench, so that the bit's tip is 5 mm above the plate |
| Spoil board | white melamine-faced MDF, 230 x 200 x 10 mm, waxed, bolted to the table with its back edge 8 cm behind the spindle, so it overhangs the table's front by 7 cm and each side by 2 cm |
| Quill | 50 mm of travel; the depth stop set at 24 mm, so the tip stops 9 mm into the 10 mm spoil board, short of the table |
| Return spring | lifts the quill with 40 N at rest, rising 530 N/m (66.5 N at 50 mm) |
| Feed handle | three spokes on the right of the head, on the pinion's shaft 39 cm above the bench and 2.5 cm behind the spindle; 10 mm steel spokes with 25 mm black ball knobs 14 cm from the shaft; at rest the front spoke points 70 degrees forward of straight down |
| Feed ratio | 15 mm of quill per radian of the handle (94 mm per turn), 9.3 to 1 at the knobs |
| Switch | a no-volt-release push-button switch box (70 x 60 x 40 mm) on the head's front, right of centre: a green 20 mm start button 2.2 cm right of the spindle and a red 28 mm stop button 5.8 cm right of it, both 40 cm above the bench; each presses in 4 mm and switches at 2 mm (4.8 to 7.2 N) |

Most 8-inch presses come with this kind of switch or with a paddle switch in the same place; either works the same. A
real trial is reset by pressing stop and letting the chuck stop, letting the quill up, backing the clamp's knob off to
one turn (1.25 mm) above the plate's top, and putting a fresh plate on its pencilled start outline, its punch mark 20 mm
in front of the bit's point (a fresh spot of spoil board under the bit once the old one is drilled).

## The hold-down clamp, for a real trial

Catalogue screw clamps usually come with swivel pads, which stay still on the work so that the screw's turning does not
drag or mark it (Carr Lane, for one, describes its swivel pad as one that "contacts the workpiece and remains stationary
while the clamp screw is tightened"). The task needs the plain kind, whose tip turns with the screw, so this clamp is
built from stock parts, all named below. The simulation has the same dimensions and the same mechanism.

| Part | What to buy or make | Source |
|---|---|---|
| Clamp screw | A star knob with a steel threaded stud: cast iron, 40 mm across its lobes, 25 mm tall, on an M8 x 40 mm blackened-steel stud (screwed in and cross-pinned). Part GN 6336.4-SG-40-M8-40. | J.W. Winco, [GN 6336.4 star knobs with steel threaded stud](https://www.jwwinco.com/en-us/products/quick-finder/GN-6336.4-Cast-Iron-Star-Knobs-with-Steel-Threaded-Stud?q=6336), US$13.75 |
| Rubber tip | A flat, push-on rubber clamp-screw tip for M8 and 5/16 in screws, 3/4 in (19.05 mm) across and 1/2 in (12.7 mm) long. Push it onto the stud's end and bond it with a drop of cyanoacrylate glue, so that it turns with the stud. | McMaster-Carr 5147A15 ([maker's listing](https://www.mcmaster.com/products/toggle-clamp-replacement-spindle-components/), "Clamp Holding Screw Tips", "Flat Push On") |
| Arm | 6061 aluminium flat bar, 25 x 12 mm, 64.5 mm long. An M8 x 1.25 tapped hole through its 12 mm thickness on its centre line, 12.5 mm from one end; a 10.5 mm clearance hole on its centre line, 52 mm from that end. | bar stock |
| Riser | 6061 aluminium square bar, 25 x 25 mm, 29.25 mm tall, with a 10.5 mm hole down its middle | bar stock |
| Bolt | M10 hex bolt and washer, down through the arm and the riser into an M10 threaded insert set in the spoil board (or through the board and a slot of the drill's table, with a nut below) | hardware |

Where it goes (seen from above, measured from the spindle's axis): the screw's axis 68 mm in front of the spindle and 40
mm left of it; the riser centred 68 mm in front and 79.5 mm left, 2 mm clear of the plate's left edge; the arm running
left-right from the riser's outer face to 12.5 mm right of the screw. The arm's underside is 19.25 mm above the plate's
top. At the start of a trial the tip is one turn (1.25 mm) above the plate.

## Is the clamp enough, and can the YAM tighten it?

Computed from handbook values:

- **Torque to force.** The M8 x 1.25 thread (pitch diameter 7.19 mm, 60-degree thread, steel in aluminium, friction
  about 0.15) needs 0.83 mm x F of torque to drive the stud against a load F (0.20 mm x F of that is the lead). The
  19.05 mm rubber tip turns on the plate under that load; with a rubber-on-aluminium friction coefficient of about 0.8
  and the mean friction radius of a flat disc (2/3 of 9.5 mm, 6.35 mm) it adds 5.08 mm x F. In all, 5.91 mm x F: a knob
  tightened to **0.5 N m presses about 85 N**, and 0.6 N m about 100 N (for a tip friction of 0.6 to 1.0, 0.5 N m gives
  108 to 70 N).
- **How far it turns.** Once the tip touches, the rubber under the stud's 8 mm end and the arm give 0.3 N per
  micrometre (computed), so 85 N takes 0.28 mm of travel: 82 degrees of knob turn past contact.
- **Workholding.** Drilling a 5 mm hole in aluminium at 0.05 to 0.1 mm per revolution takes about 0.13 to 0.25 N m
  (feed x diameter squared x 800 N/mm2 / 8000); the thrust only presses the plate down. At 85 N the tip's friction
  alone (0.8 x 85 = 68 N, 79 mm from the punch mark) resists 5.4 N m about the bit, twenty times the drilling torque.
- **The YAM.** 0.5 N m on the 40 mm star knob means the fingers push its lobes with about 17 N each, 15 mm from the
  axis, well inside the YAM's roughly 40 N per finger; the lobes bear on the finger pads, so the turn does not depend
  on friction, and the arm's wrist motors give far more than 0.5 N m. The knob turns about its own (vertical) axis; no
  grip has to hold a torque about a horizontal axis, where a parallel grip slips at 0.1 to 0.4 N m. (A 47 mm round
  knurled knob, tried first, slipped in the simulated grip with its tip pressing only 15 to 55 N: the hand has to lean
  45 degrees to reach a knob this high in front of the drill's head. The star knob's lobes do not slip.)
- **The holding hand.** While the tip turns on a held plate it drags it with 5.08 / 5.91 of the knob's torque, about
  0.43 N m at 0.5 N m. The hand pressing the plate's front-right part, 11.5 cm from the tip, resists that with about
  3.7 N sideways at its fingertips.

## Real-trial safety and rig protection

- **Power.** The drill is a real machine. Run it on its lowest belt speed (740 rpm on presses of this class) and plug it
  into a switched power strip within reach of the person running the trial, who switches it off at once if the plate
  moves while the bit turns or a gripper heads for the chuck. The press's no-volt-release switch keeps it off until
  start is pressed again. People nearby wear safety glasses.
- **Hands clear.** The score already fails a trial in which any part of either gripper comes within 5 cm of the turning
  bit or chuck; the person at the power strip stops such a trial.
- **Chips and edges.** Drill dry: no cutting fluid, which could reach the finger pads. A 5 mm hole through 10 mm of
  aluminium leaves a few curled chips: brush them off the plate, the board and the table with a bench brush (never by
  hand or with a gripper) before each trial, so the pressing gripper never meets one. Deburr the plates before use.
- **Nothing to contaminate the rig.** The plate, the clamp and the drill's handles are clean and dry; nothing greasy
  touches the grippers.
- **Cost.** The clamp's parts are a US$13.75 knob, a rubber tip, a few dollars of aluminium bar stock and an M10 bolt;
  the drill press is an ordinary 8-inch bench machine; each trial uses one 130 x 150 x 10 mm 6061 aluminium plate and a
  fresh spot of the spoil board.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The drill press's base, column, table, head, belt
cover, motor and switch box, the spoil board and the clamp's riser and arm are kinematic, fixed where they are put. The
plate, the clamp screw, the quill, the chuck (with the bit), the feed handle and the two buttons are loose rigid bodies
made of shapes from `bodykit.py`, joined to the fixed parts by PhysX joints.

- The feed: the quill slides in the head on a prismatic joint (to the depth stop), and the feed handle turns on a
  revolute joint. PhysX's rack-and-pinion joint does not run in this GPU scene, so the gear is emulated: before every
  physics step (1/240 s) the task file reads the quill's travel and the handle's turn (as travel: 15 mm per radian),
  and aims a stiff drive on each (100 kN/m at the quill, the same seen through the pinion at the handle) at the mean of
  the two. A torque on the handle reaches the quill as that torque over the pinion's radius, and the quill's loads come
  back to the handle, with about 0.7 mm of give between them. The return spring acts on the quill's drive.
- Drilling: the bit does not collide. While the chuck turns and the bit cuts at the bottom of its hole, the quill's
  drive damps the feed by the cutting thrust: 50 N per mm/s of feed in the aluminium (about 0.05 mm per revolution at
  2,000 rpm needs some 80 N) and 8 N per mm/s in the MDF; the hand on the handle feels it through the gear. The hole is
  as deep as the tip has cut while the chuck turned and the plate was clamped; the task file marks it on the plate where
  the bit is. The bit's tip pushed into a plate that is not clamped catches it and flings it; pushed into the plate while
  the chuck is not turning, it only presses on it (scored 0).
- The motor: the chuck turns on a revolute joint on the quill, driven by a velocity drive. Pressing the green button in
  past its 2 mm switch point starts it (the chuck spins up in about a quarter of a second), and pressing the red one
  stops it (the chuck runs down in about 3 s). The chuck turns at 180 rpm in the simulation, slow enough for the cameras
  to show it turning; a real one turns at 740 to 3,140 rpm.
- The clamp: the star knob, its stud and the bonded rubber tip are one body, hung in the arm on a thread joint (a D6
  joint from `bodykit.py` that lets it slide along and turn about the stud's axis). Before every physics step the task
  file reads how far the screw has turned and aims its slide drive 1.25 mm down per clockwise turn. The drive's
  stiffness, 300 N/mm (the rubber under the stud's end, with the arm's bending, computed above), is the clamp's
  spring: once the tip is on the plate each further degree of turn presses 1.04 N harder, so 85 N takes 82 degrees past
  contact, and the robot has to turn the knob against the growing friction of the tip on the plate (0.63, standing for
  the rubber's friction and the thread's under load). With no load the thread drags with at most 0.05 N m. The same
  friction drags a loose plate round on the waxed board (friction 0.05).
- Clearance: after every control step the task file takes about 38,000 surface points of each arm's hand and wrist (the
  two wrist links, the gripper body and both fingers, from the robot's collision meshes) and finds the closest one to
  the turning parts (the bit from its tip up, and the chuck with the spindle's nose). While the chuck turns faster than
  1 rad/s, any of them closer than 5 cm makes the trial unsafe.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

No third-party assets are used: every object is built in the task file from boxes, cylinders and spheres, and the robot
is the `yam_arena` embodiment's own.

## Scoring

Judged at the end of the trial, with latches for what happens during it. Each tier, with how a real trial measures it:

| Check | Simulation | Real trial |
|---|---|---|
| Clamped | the rubber tip on the plate (within 0.5 mm of its top) and pressing at least 85 N | the knob tightened to at least 0.5 N m, which presses about 85 N: after the trial, with the plate held down by hand, a spring scale looped over a lobe of the star knob, 20 mm from its axis, and pulled square to it clockwise reaches 25 N without the knob turning |
| At the mark | the punch mark within 5 mm of the bit's axis | lowered by hand with the motor off, the bit's point lands within 5 mm of the punch mark (ruler) |
| Drilled through at the mark | while the chuck turned and the plate was clamped, the bit's tip cut 11.5 mm below the plate's top (through the 10 mm plate, 1.5 mm into the board), the hole within 5 mm of the mark | a 5 mm gauge pin passes through the hole; calipers put its centre within 5 mm of the punch mark |
| Unsafe | while the chuck turned faster than 1 rad/s, any point of either hand or wrist came within 5 cm of the bit or the chuck | judged frame by frame from the overhead and wrist videos, with the 46 mm chuck as the scale |
| Spoilt | the turning bit met a plate that was not clamped, or the bit was pushed 0.5 mm into the plate while not turning | the turning bit caught the plate (it turned or jumped with the bit in it, seen in the video); or the quill was pushed down onto the plate with the chuck stopped |
| Left safe | the quill within 3 mm of its top stop; the motor off and the chuck slower than 1 rad/s | the depth scale at 0; the switch off and the chuck still |

1.0: drilled through at the mark, never unsafe, and at the end the plate still clamped and the drill left safe. 0.5:
never unsafe, and either drilled through at the mark but the drill left running or its quill down or the clamp
loosened, or the plate clamped with its mark within 5 mm of the bit's axis but not drilled through. 0.25: the plate
clamped but off its mark, or drilled through or clamped but unsafe at some point. 0.0: otherwise, or spoilt.

The scorer is `task_score`: 1.0 when `success()` (full score, with the plate, the quill, the feed handle, the clamp's
screw and the chuck still) has held for 1 s, otherwise `score()` at the end of the episode (see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

## Run and score a trial

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clamp_a_small_part_before_drilling_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drill a hole right through the aluminium plate at its punch mark with the drill press, then leave the drill switched off with its quill up. The plate must not move while it is drilled, and no part of either gripper may come within 5 cm of the bit or the chuck while they turn." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/drill-dream
```

The scripted demo, which reaches full score: the right hand comes down on the plate's front edge, fingers closed and
leaning toward the arms, presses it onto the board and slides it 20 mm back, bringing the punch mark under the bit; it
lifts off and comes down again on the plate's front-right part, pressing it onto the board. Meanwhile the left hand
comes down on the clamp's star knob, fingers closing across its lobes, and turns it clockwise in quarter-turn strokes
(grip, turn, let go, rise, turn back in the air, come down, grip again): four strokes bring the tip down onto the plate,
and a fifth and a shorter sixth press it to 115 N while the right hand holds the plate against the tip's drag. The left
rises and goes home. The right rises, presses the green start button (the chuck spins up), takes the feed handle's front
knob from the right and pulls it down 85 degrees: the bit comes down to the plate in 1.5 s, then feeds through it and
into the spoil board over 13 s, the handle pushing back with up to 8.1 N at the knob. It lets the handle back up under
control, lets go, presses the red stop button and goes home; the trial ends once the chuck has stopped and the full
score has held 1 s:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clamp_a_small_part_before_drilling_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drill a hole right through the aluminium plate at its punch mark with the drill press, then leave the drill switched off with its quill up. The plate must not move while it is drilled, and no part of either gripper may come within 5 cm of the bit or the chuck while they turn." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/drill-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/drill-dream/*.json; do inspect-robots video "$log"; done
```

Set `DRILL_DEBUG=1` to print what the score is built from twice a second: how far the clamp screw has turned and come
down, the tip's force, whether it is on the plate and the plate clamped, where the plate is, how it has turned and how
far its mark is from the bit's axis, the quill's travel and the feed handle's turn, the push on the handle's knob, the
buttons, the motor and the chuck's speed, the hole's depth and whether it is on the mark, the latches, and each arm's
closest clearance to the turning parts.

## Runtime per trial

On one RTX 5090: about 11 minutes for a Dream-1 trial of 1,800 steps (688 s on the clock, with Dream answering in 0.87 s on average), and about 8 minutes for the scripted demo (1,617 steps, 80.8 s of simulated time, 488 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-BUULQE. Its review said:

> Construct validity (not met): The simulated clamp reaches full-credit workholding at 6 N, a small fraction of the approximately 100 N specified for the real clamp.
>
> Clarity and reproducibility (not met): The custom 8 mm-per-turn hold-down clamp lacks a named source or sufficient construction details to reproduce its mechanism.
>
> What to change: The earlier press, clearance and through-hole changes are addressed. A new issue not raised in the previous feedback is that the simulated clamp reaches the scoring threshold at only 6 N, versus the approximately 100 N specified for the real trial. Model a realistic tightening resistance and clamping force, align the simulation and real scoring checks, and provide an updated full-score demo. Also specify an off-the-shelf clamp with the required rotating pad and travel, or provide construction drawings and parts sufficient to reproduce the clamp.

The review before that (TK-Y76NCR) asked for a representative lever-fed drill press with its own motor switch, its
dimensions for a real reset, and clearance measured from the whole gripper; that was done in the previous revision and
is kept. The one before (TK-RBRWL9) asked for the bit to go through into sacrificial backing and for workholding that
resists the drilling forces; that is kept too.

What changed in response:

- **Construct validity: a realistic clamping force and tightening resistance.** The soft 30 N spring that counted the
  plate clamped at 6 N is gone. The clamp screw now has a real M8 x 1.25 thread (1.25 mm per turn; it was 8 mm), and
  its slide drive has the stiffness of the rubber under the stud's end and the arm (300 N/mm, computed), so once the tip
  is on the plate each further degree of knob turn presses 1.04 N harder, and the knob turns against the tip's growing
  friction on the plate. Clamped now needs the tip pressing at least **85 N**, the force of a knob tightened to 0.5 N m
  (computed: 0.83 mm x F for the M8 thread, 5.08 mm x F for the rubber tip). Reaching it takes a full turn down onto
  the plate and 82 degrees more. In the demo the tip presses 115 N.
- **Simulation and real scoring aligned.** Both check the same clamp: tightened to 0.5 N m, about 85 N at the tip. The
  simulation reads the tip's force (at least 85 N, the tip within 0.5 mm of the plate); a real trial checks the torque
  after the trial with a spring scale looped over a lobe of the star knob, 20 mm from its axis: pulled clockwise to 25 N
  (0.5 N m), the knob must not turn. The old torque-screwdriver check is gone.
- **The YAM can deliver the torque.** 0.5 N m on the 40 mm star knob takes about 17 N per finger on its lobes, inside
  the YAM's roughly 40 N; the lobes bear on the pads, so the turn does not rest on friction, and the knob turns about
  its own vertical axis. (A 47 mm round knurled knob slipped in the simulated grip with its tip at 15 to 55 N, so the
  clamp uses the star knob.)
- **Clarity and reproducibility: off-the-shelf parts and construction details.** The clamp screw is a catalogue star
  knob on its own M8 x 40 steel stud (J.W. Winco GN 6336.4-SG-40-M8-40, cast iron, 40 mm, US$13.75) with a catalogue
  flat push-on rubber tip (McMaster-Carr 5147A15, 19.05 mm across) bonded to the stud's end, so that the pad turns with
  the screw, as the task needs; catalogue swivel pads are made not to. The arm (25 x 12 x 64.5 mm 6061 bar, M8 tapped
  hole 12.5 mm from its end, 10.5 mm hole at 52 mm) and the riser (25 x 25 x 29.25 mm 6061 bar) are dimensioned, with
  one M10 bolt, and the README gives where each part goes. Travel: the tip starts one turn (1.25 mm) above the plate and
  has 12 mm of travel before the knob meets the arm. The simulation uses the same dimensions and mechanism.
- **Updated full-score demo.** The demo scores 1.0 (1,617 steps): the right hand slides the plate into place and holds
  it, the left tightens the clamp in five quarter-turn strokes and a shorter sixth to 115 N, then the right drills
  through (hole 16.2 mm deep, 3.3 mm from the mark), lets the handle up and stops the motor.
- **Placing the part.** The plate now starts with its punch mark 20 mm in front of the bit, so setting the part on the
  drill's table under the bit is part of the job; the spoil board is 230 x 200 mm to carry it. The README names what the
  task tests beyond holding one part while the other hand tightens: placing the mark under the bit and keeping it there
  against the clamp's drag, tightening to a real clamping force, and running the press in the safe order with both
  grippers clear of the chuck. A new "Real-trial safety and rig protection" section covers power, chips and cost.
- **Re-runs.** Demo 1.0 (1,617 steps) and Dream-1 0.0 in the final batch run. One-hand check: 0.25 (the unheld plate
  turned 5.3 degrees and was clamped with its mark 8.3 mm off).
