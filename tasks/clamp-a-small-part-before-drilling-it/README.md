# Drill a 5 mm hole through a small aluminium plate at its punch mark on a bench drill press

An 8-inch bench drill press stands on the table. On its table lies a waxed MDF spoil board, and on that a loose
aluminium plate laid out the way a machinist lays out a hole: two scribed lines crossing on blue layout dye, a
90-degree centre-punch dimple 2.0 mm across where they cross, and an 8.0 mm check circle round it. The dimple starts
20 mm in front of the 5 mm bit. A screw hold-down clamp (a star-knob M8 screw whose bonded rubber tip turns with it)
reaches over the plate's front-left part, its tip one turn above the plate. The motor has its own push-button switch
(green start, red stop); a three-spoke capstan handle feeds the quill down against its return spring.

The instruction gives only the goal: drill right through the plate at its punch mark, then leave the drill switched
off with its quill up; the plate must not move while it is drilled, and no part of either gripper may come within 5 cm
of the turning bit or chuck. The way a machinist does it, which the policy has to find: with the motor off, bring the
bit's point down onto the plate and move the loose plate under it until the point drops into the punch mark, which
then holds the plate on the bit's axis; keep the point there while the clamp is tightened (its turning tip drags the
plate round); let the bit up, start the motor, feed the bit through into the spoil board, let it up, stop the motor.
*Introduction to Machining* (Washington SBCTC open textbook, CC BY 4.0),
[8.10 Drilling](https://openwa.pressbooks.pub/intromachining1/chapter/wa8-10/): "Gently bring the spotting tool down
to the center punched mark on the part with the right hand, while precisely positioning the vise with the left hand.
Leaving the vise unclamped will allow the vise to float on the table and more precisely align with the center punch
mark."

Files: [`clamp_a_small_part_before_drilling_it.py`](clamp_a_small_part_before_drilling_it.py), the `yam_arena` task
file, with the scorer and its docstring at the top and the punch-dimple rule below it;
[`drill_press.py`](drill_press.py), the scene (bodies, joints, the drive emulation, the hands' clearance);
[`drill_demo.py`](drill_demo.py), the scripted demo; [`bodykit.py`](bodykit.py), multi-shape bodies and joints.

## What it tests, and why two hands

- **Locating by the punch mark.** The hole counts only if it takes in the whole 2.0 mm dimple: its centre within
  1.5 mm of the mark's. The point finds the dimple only within 1.0 mm of it, so the plate has to be brought that close
  under the lowered point, and then the dimple centres it.
- **Keeping it located while the clamp goes on.** The clamp's tip turns with its screw and drags the plate round. One
  hand keeps the feed handle pulled down, so the point stays in the dimple and holds the plate on the axis, while the
  other turns the knob a full turn down and 82 degrees on, until the tip presses 85 N (0.5 N m at the knob).
- **Running the press in the safe order**, clamp before motor, with both grippers 5 cm clear of the turning chuck.

Why two hands: the clamp's turning tip drags the plate round unless the point is kept in the dimple while the clamp
goes on. In the one-hand check (the demo, but the right let the point up as soon as the plate was located, before the
clamp went on) the plate, located to 0.02 mm, turned 4.7 degrees as the clamp tightened and was clamped and drilled
with its mark 6.6 mm off: 0.25. In the demo, with the point held in the dimple at 15 N, it turned 0.7 degrees about the
point and the hole went in 0.31 mm from the mark.

## The plate and its punch mark, for a real trial

6061 aluminium, 130 x 150 x 10 mm, deburred. Lay out with blue layout fluid round the mark; scribe two lines crossing
on the plate's centre line 25 mm from its back edge; strike the crossing with a 90-degree automatic centre punch to a
dimple 2.0 mm across (1.0 mm deep); scribe a check circle 8.0 mm across round it with dividers. Check the dimple with a
10x measuring loupe (0.1 mm reticle): 2.0 +- 0.2 mm across. A 90-degree point is the usual centre-punch point and a
0.5-1 mm deep depression the usual centre-punch mark ([Tooled-Up centre-punch guide](https://www.tooled-up.com/centre-punches/cat/2000167/):
"typically ground to 90 degrees"; "a depression of 0.5-1mm proves sufficient for most applications"); this plate uses
the deep end, 1.0 mm, so the dimple is 2.0 mm across. A 5 mm hole takes in the whole dimple when its centre is within
2.5 - 1.0 = 1.5 mm of the dimple's; then its edge also stays inside the 8.0 mm check circle (4.0 - 2.5 = 1.5 mm).
Plates are laid out in a batch before the trials.

## The drill press and the clamp, for a real trial

Any 8-inch (203 mm swing) bench drill press of the common 1/3 hp, five-speed class, set as in the simulation
(measured from the spindle's axis, heights above the bench):

| Part | Simulated (and what to set on a real one) |
|---|---|
| Base, column, head | cast iron base 290 x 190 x 40 mm, its front 16.5 cm ahead of the spindle; 46 mm column 10.5 cm behind it, 47 cm tall; head 14 cm wide, 33.5-47.5 cm up, its front 5.5 cm ahead of the spindle |
| Chuck and bit | 13 mm keyed chuck, 46 x 76 mm; a 5 mm HSS jobber drill (118-degree point), 55 mm out of the chuck |
| Table and spoil board | table top 16.0 cm up, so the bit's point is 5 mm above the plate; white melamine MDF board 230 x 200 x 10 mm, waxed, back edge 8 cm behind the spindle |
| Quill | 50 mm travel, depth stop at 24 mm; return spring 40 N at rest, rising 530 N/m |
| Feed handle | three spokes on the pinion's shaft 39 cm up, 2.5 cm behind the spindle, 11 cm to its right; 25 mm ball knobs 14 cm from the shaft; 15 mm of quill per radian (9.3 to 1 at the knobs); front spoke at rest 70 degrees forward of straight down |
| Switch | no-volt-release push-button box on the head's front: green 20 mm start 2.2 cm right of the spindle, red 28 mm stop 5.8 cm right, both 40 cm up, pressing in 4 mm, switching at 2 mm (4.8-7.2 N) |

The clamp is built from stock parts (catalogue clamps have swivel pads, which do not turn on the work):

| Part | Source |
|---|---|
| Star knob, cast iron, 40 mm, on an M8 x 40 steel stud | J.W. Winco [GN 6336.4-SG-40-M8-40](https://www.jwwinco.com/en-us/products/quick-finder/GN-6336.4-Cast-Iron-Star-Knobs-with-Steel-Threaded-Stud?q=6336), US$13.75 |
| Flat push-on rubber tip for M8, 19.05 mm across, 12.7 mm long, bonded to the stud's end with cyanoacrylate | McMaster-Carr 5147A15 |
| Arm: 6061 bar 25 x 12 x 68.5 mm, M8 tapped hole 12.5 mm from one end, 10.5 mm hole at 56 mm | bar stock |
| Riser: 6061 bar 25 x 25 x 29.25 mm, 10.5 mm hole down its middle; M10 bolt into an insert in the board | bar stock, hardware |

The screw's axis is 68 mm in front of the spindle and 40 mm left of it; the riser 6 mm clear of the plate's left edge;
the tip starts one turn (1.25 mm) above the plate, with 12 mm of travel before the knob meets the arm.

Forces, computed from handbook values: the M8 x 1.25 thread takes 0.83 mm x F of torque, the 19.05 mm rubber tip
turning on the plate (friction 0.8, mean radius 6.35 mm) 5.08 mm x F, so **0.5 N m presses about 85 N**. The rubber
and the arm give 0.3 N per micrometre, so 85 N takes 82 degrees of knob turn past contact. At 85 N the tip's friction
(68 N, 79 mm from the mark) resists 5.4 N m about the bit, twenty times the 0.13-0.25 N m of drilling a 5 mm hole in
aluminium. 0.5 N m on the 40 mm star knob is about 17 N per finger on its lobes (YAM: about 40 N), the lobes bearing on
the pads. Holding the point in the dimple: the return spring needs 42.7 N at the quill at 5 mm (4.6 N at the knob);
each further newton on the plate adds 0.11 N at the knob, so a 15 N push takes 6.2 N at the knob, or 9.4 N where
the demo's hand works the spoke, 92 mm from the shaft.

## Real-trial safety and rig protection

- **Power and people.** Lowest belt speed (740 rpm on presses of this class); the drill plugged into a switched power
  strip in the operator's reach, switched off at once if the plate moves under the turning bit or a gripper heads for
  the chuck; safety glasses for everyone nearby. The no-volt-release switch keeps the motor off until start is pressed.
- **Grippers.** The score fails any trial with a gripper within 5 cm of the turning bit or chuck. Drill dry (no fluid
  near the pads); brush the chips off with a bench brush before each trial; deburr the plates.
- **Cost.** A US$13.75 knob, a rubber tip, aluminium bar, an M10 bolt, layout fluid, an automatic centre punch,
  dividers and a 10x measuring loupe; an ordinary 8-inch bench drill press; one plate and a fresh spot of spoil board
  per trial.

## How it is modelled

Isaac Sim 6.1 (PhysX), 20 Hz control. The drill's fixed parts, the board and the clamp's riser and arm are kinematic;
the plate, the clamp screw, the quill, the chuck with the bit, the feed handle and the buttons are rigid bodies on
PhysX joints. Before every physics step (1/240 s) the task file:
- **gear**: aims the quill's and the handle's drives at their mean position (15 mm per radian; PhysX's gear joint does
  not run on the GPU), with the return spring on the quill;
- **clamp thread**: aims the screw's slide drive 1.25 mm down per clockwise turn; its stiffness, 300 N/mm, is the
  rubber's, so each degree past contact presses 1.04 N harder; the tip's friction, 0.63, also stands for the thread's;
- **the punch dimple** (`_seat`): the bit has no collider. With the motor off, its point stops on the plate (a 50 kN/m
  stop on the quill, its force passed back to the handle) and, within 1.0 mm of the mark, rests on the dimple's rim
  (118-degree point, 31-degree flanks), sitting (1.0 mm - offset) x tan 31 degrees deeper; the rim pushes the plate to
  bring the dimple under the point, with up to 0.10 times the point's force while the plate moves toward it and 1.49
  times against a push away (friction 0.47, steel on aluminium, dry: tan(31 - 25.2) and tan(31 + 25.2) degrees). On
  the flat, the point holds the plate with that friction, sticking until it slips;
- **drilling**: while the chuck turns and the bit cuts, the cutting thrust damps the feed (50 N per mm/s in aluminium,
  8 in MDF); the hole is as deep as the tip has cut while the plate was clamped, and is marked where it is. The turning
  bit meeting an unclamped plate catches and flings it.
After every control step it checks each hand's clearance: about 38,000 surface points of each wrist and gripper (from
the robot's collision meshes) against the bit and the chuck. The chuck turns at 180 rpm in the simulation (real: 740
to 3,140 rpm) so the cameras show it. No third-party assets: every object is built from boxes, cylinders and spheres;
the robot is `yam_arena`'s own.

## Scoring

| Check | Simulation | Real trial |
|---|---|---|
| Clamped | the rubber tip on the plate (within 0.5 mm) pressing at least 85 N | after the trial, plate held down: a spring scale on a knob lobe, 20 mm from its axis, pulled clockwise to 25 N (0.5 N m): the knob does not turn |
| Drilled through at the mark | the turning bit's tip cut 11.5 mm below the plate's top while the plate was clamped; the hole's centre within 1.5 mm of the mark's (only the first hole counts) | a 5 mm gauge pin passes through; under the 10x loupe no part of the dimple is left beside the hole, and the hole's edge stays inside the 8.0 mm check circle |
| At the mark, not drilled | the plate clamped with its mark within 1.0 mm of the bit's axis | lowered by hand, motor off, the bit's point drops into the dimple |
| Unsafe | while the chuck turned faster than 1 rad/s, any point of either hand or wrist within 5 cm of the bit or chuck | frame by frame from the videos, the 46 mm chuck as the scale |
| Spoilt | the turning bit met a plate that was not clamped | the plate turned or jumped with the bit in it |
| Left safe | the quill within 3 mm of its top stop; motor off, chuck slower than 1 rad/s | depth scale at 0; switch off; chuck still |

1.0: drilled through at the mark, never unsafe, and at the end the plate clamped and the drill left safe. 0.5: never
unsafe, and either drilled through at the mark but the drill left running, its quill down or the clamp loosened, or
the plate clamped at the mark but not drilled through at it. 0.25: the plate clamped off its mark or drilled off it, or
clamped or drilled while unsafe. 0.0: otherwise, or spoilt. `task_score` gives 1.0 once `success()` (full score with
the plate, quill, handle, screw and chuck still) has held 1 s, else `score()` at the end.

## Run and score a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clamp_a_small_part_before_drilling_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drill a hole right through the aluminium plate at its punch mark with the drill press, then leave the drill switched off with its quill up. The plate must not move while it is drilled, and no part of either gripper may come within 5 cm of the bit or the chuck while they turn." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/drill-dream
```

`--policy yam_scripted` plays the demo. `-E object_jitter=0` keeps the layout fixed (no seeds needed).
`inspect-robots video <log>.json` writes one MP4 per camera. `DRILL_DEBUG=1` prints the score's inputs twice a second.
On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,800 steps (576 s on the clock, with Dream answering in 0.76 s on average), and about 9 minutes for the scripted demo (1,751 steps, 87.5 s of simulated time, 533 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## The scripted demo

The right hand forks the feed handle's front spoke: its fingers part-closed either side of the 10 mm spoke, just inside
the ball, so it can push the spoke either way (the YAM's V-shaped jaw pushes a squeezed ball out to its fingertips).
With the motor off it brings the bit's point down to just above the plate. The left hand comes down on the plate's
front edge, presses it onto the board, slides it 20 mm back and nudges it until the mark is under the point; the right
lowers the point, which drops into the dimple and centres the plate (the mark 0.32 mm from the axis as the point
came down, 0.03 mm once it was in), and presses it in at 15 N. Two parts of the plan follow the simulation, as a
person watches and feels the work (a scripted demo may read the simulator's state): the right hand's hold on the handle
is turned to keep the point's push at its target, and the left's nudge is aimed at the mark's offset from the bit's
axis until the point is in the dimple. The left lifts off and turns the clamp's star knob clockwise in quarter-turn
strokes (grip, turn, let go, rise, turn back in the air, come down, grip again) until the tip presses 114 N;
the clamp's drag turns the plate 0.7 degrees about the point, which keeps the mark 0.31 mm from the axis.
The right lets the handle up; the left presses the green start button and goes home; the right feeds the bit through
the plate into the spoil board over 13 s (the hole 12.3 mm deep, 0.31 mm from the mark), lets the
handle up, presses the red stop button and goes back the way it came. No hand came closer than 58 mm to the
turning bit or chuck. It has the full score from 76 s on; the run ends at 1,751 steps.

## Changes in this revision

This is a revision of TK-9HHDPF. Its review said:

> Flags: The task implementation is omitted from the code excerpt
>
> Construct validity (not met): A hole centred 5 mm from the punch mark can miss the mark entirely despite full credit for drilling at it.
>
> Scoring (not met): The 5 mm centre-error allowance rewards a hole that does not cover the punch mark, including the reported demo's 3.3 mm error.
>
> What to change: The clamp force, construction details and updated demonstration address the previous feedback. A new issue not raised before is the 5 mm allowed error: a 5 mm hole can entirely miss the punch mark and still receive full credit; the reported demo error is 3.3 mm. Tighten the hole-centre tolerance to a practically measurable value consistent with drilling at the mark, update the scorer and real-trial check, and provide a full-score demo that meets it.

Earlier reviews, all kept: TK-BUULQE asked for a realistic clamping force and an off-the-shelf or fully specified clamp
(85 N at 0.5 N m, star knob and parts list); TK-Y76NCR for a lever-fed drill press with its own motor switch, its
dimensions, and clearance measured from the whole gripper; TK-RBRWL9 for drilling through into sacrificial backing
with workholding that resists the drilling forces.

What changed in response:

- **Hole-centre tolerance: 5 mm to 1.5 mm, so the hole always takes in the punch mark** (scorer `HOLE_ON_MARK` at the
  top of the task file; scoring field). The plate now carries a real centre-punch dimple, 90 degrees, 2.0 mm across
  and 1.0 mm deep (the deep end of the cited 0.5-1 mm). The hole counts as at the mark only if its centre is within
  2.5 - 1.0 = 1.5 mm of the dimple's, which is exactly when the 5 mm hole contains the whole dimple. Check: the same
  demo steered to leave the mark 3 mm in front of the bit's axis drilled a hole 2.83 mm off and scored
  **0.25**.
- **Real-trial check, the same rule** (scoring field; README "Scoring"). A 5 mm gauge pin through the hole; under a
  10x measuring loupe no part of the dimple is left beside the hole, and the hole's edge stays inside the scribed
  8.0 mm check circle. Both hold exactly when the centres are within 1.5 mm (2.5 - 1.0 and 4.0 - 2.5), so the check is
  a look through a loupe, with no caliper reading. The partial-credit "at the mark, not drilled" tier tightened from
  5 mm to 1.0 mm: the bit's point, lowered by hand, drops into the dimple.
- **Locating the way machinists do it, by the punch mark** (task file `_seat`; setup, context and fidelity fields).
  With the motor off, the bit's 118-degree point stops on the plate and, within 1.0 mm of the mark, drops into the
  dimple, whose rim pushes the loose plate until the dimple is under the point (0.10 times the point's force) and holds
  it there against a push away (1.49 times; steel on aluminium, friction 0.47, dry). Off the dimple the point grips the
  flat with that friction. So lowering the bit into the punch mark with the clamp loose centres the part, and keeping
  the point pressed holds it while the clamp's turning tip drags at it.
- **Full-score demo that meets the tolerance** (1.0, 1,751 steps). The right hand brings the point down; the left
  slides the plate 20 mm under it and nudges it until the mark is under the point; the point drops into the dimple
  (the mark 0.32 mm off as it came down), centres the plate to 0.03 mm and holds it at 15 N while the left tightens the
  clamp to 114 N, which turns the plate 0.7 degrees about the point. The hole is 0.31 mm from the mark (limit 1.5 mm),
  12.3 mm deep. The right hand now forks the feed handle's spoke rather than squeezing its ball knob, so it can press
  the point and feed the bit without losing the handle; the clamp's riser stands 6 mm (was 2 mm) clear of the plate's
  edge, so the plate can be set without jamming on it.
- **Clamping without locating fails.** The same demo without the nudge (the plate slid 20 mm open loop, the point
  lowered where it landed, 2.77 mm from the mark, and held there while the clamp goes on, then drilled): the clamp's
  drag turned the plate 2.3 degrees about the point on the flat, and the hole went in 3.88 mm off the mark; it scored
  **0.25**.
- **Why two hands, re-checked.** One hand at a time (the point let up before the clamp went on): the located plate
  turned 4.7 degrees as the clamp tightened and was drilled 6.6 mm off the mark: 0.25.
- **Code excerpt.** The README is 20,245 characters (was 26,850). The task file now opens with the scorer:
  `score()` with its docstring, its thresholds and the checks it reads, in its first 105 lines; the scene is in
  `drill_press.py` and the demo in `drill_demo.py`.
- **Re-runs.** Demo 1.0 and Dream-1 0.0 in the final batch run.
