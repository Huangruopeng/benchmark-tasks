# Set a spring rat trap without letting its kill bar snap

The trap is a stock **Victor M326 Professional rat trap** (Woodstream), a wooden snap trap with a yellow plastic
trigger pedal, about $3 each
([Solutions Stores listing: $2.99 each, board 7 x 3-1/4 x 1/2 in](https://www.solutionsstores.com/victor-wooden-rat-snap-trap-m326)).
It is used as sold, without bait, screwed to an oak plank that lies loose on the table. Its kill bar lies sprung
forward and its arm bar stands up at the near end. The job is to set it, a familiar household chore:
- The trap ends set: the arm bar laid over the kill bar, its tip hooked under the catch on the trigger pedal, holding
  the kill bar back at the near end.
- The kill bar must never snap forward, and no gripper may be in its path while nothing holds it back.
- Both grippers end clear of the set trap, and both arms back where they started.

The instruction states only this goal and these constraints; how to get there is left to the policy. One way, as in
the scripted demo below: the left hand holds the plank down while the right pulls the kill bar up and over, against
its spring, and holds it down at the near end; the left pushes the arm bar over it and presses its tip down into the
catch; then the right eases off and lets go.

The task is one task file,
[`set_a_rat_trap_without_a_gripper_in_its_strike_path.py`](set_a_rat_trap_without_a_gripper_in_its_strike_path.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder. Every part is built from boxes and cylinders in the task file; there are no third-party
assets.

What it tests:
- working a strong spring with one hand (21 N at the striking bar to hold it pulled back) while the other sets a
  small latch;
- carrying a bar round an arc about its axle, the grip turning with it;
- hooking a 3 mm wire tip under a ledge that overhangs it by 1.2 mm;
- keeping out of a hazard's path;
- finishing with both hands free.

Why two hands: one hand holds the trap down while the other pulls the kill bar over, and then one hand holds the kill
bar back while the other hooks the arm bar; the spring pushes the kill bar forward as soon as it is let go, and only
the hooked arm bar holds it. In the one-hand check below, the right alone cannot even pull the kill bar back: with
nothing holding the plank, its pull drags the trap 17 cm across the table.

## The trap

| Part | Real trap (Victor M326) | Simulation |
|---|---|---|
| Board | pine, 178 x 83 x 12.7 mm (maker's listing) | the same, 16.8 to 34.6 cm forward on the centre line |
| Plank | oak, 32 x 22 x 2.5 cm, loose on the table, the trap screwed to it through two corners of its board | the same; plank and trap 1.47 kg |
| Kill bar | U of copper-plated steel wire: striking bar 74 mm from the axle, side arms 70 mm apart | the same; 15 g; the striking bar a 12 mm sleeve turning freely on it |
| Springs | two coils on the axle, 27 cm forward, 7 mm above the board | a hinge drive plus a constant preload torque pair (below) |
| Arm bar | 3 mm wire on a staple on the centre line, 8 mm from the near end; 106 mm from staple to tip | 3 mm square bar, the same length; 7 g |
| Trigger pedal | yellow plastic plate on a wire staple beside the coils, the catch on its near edge | 45 x 46 x 2.5 mm plate, 8 g, pivot 10 mm past the axle, with the catch drawn below |

The board is the maker's listed size; the other parts' sizes and places are design values scaled from the maker's
product photos.

**The spring** is calibrated to the published mechanical tests of this trap (Warburton, Poutu & Domigan 2002,
*Effectiveness of the Victor snapback trap for killing stoats*, DOC Science Internal Series 83, Table 1, the Victor
professional rat trap): static clamping force 8.2 N with the striking bar 10 mm off closed, and 3.16 J at impact. A
linear torsion spring through both gives 0.563 N m with the bar closed plus 0.326 N m per radian of travel:
- 8.2 N at 10 mm off closed (published: 8.2 N) and 8.8 N at 20 mm (published: 10.2 N);
- 3.16 J released from pulled right back (175 degrees) to 10 mm off closed (published: 3.16 J);
- 7.6 N at the striking bar to lift it off the board, and 1.56 N m pulled right back: 21 N at the striking bar.

## The catch

The real trial uses the trap's own catch, unmodified. The simulated catch works the same way: the arm bar's tip hooks
under a ledge on the pedal's near edge, the spring pushes it up against the ledge, and tipping the pedal slides the
ledge off the tip. Its dimensions (design values; pedal frame: from the pedal's pivot, x toward the trap's far end,
z up):

```
              45-degree chamfer, 3 mm high, from the ledge's near top edge
                  /|
   ledge, 3 mm   / | back wall, 4 mm thick (x = 4.5 to 8.5 mm),
     _________ /   |  up to 6 mm above the ledge's underside
    |_________|    |   <- the ledge's underside: 17.4 mm above the pivot (24.4 mm above the board)
 tip -> [==]       |      = the top of the arm bar's tip when the arm bar lies level
        notch      |
    x = 0.5 mm: the ledge's near edge          x = 4.5 mm: the notch's back wall
    the arm bar's tip ends at x = 1.7 mm, 1.2 mm under the ledge
 ==o======================================= pedal plate, 45 x 46 x 2.5 mm; pivot 7 mm above the board,
   pivot                                     10 mm past the axle; catch 12 mm wide
```

- The pedal turns on its staple from its stop (-0.3 degrees) to 6 degrees, far end down; a return spring (0.004 N m
  per degree) holds it on its stop with 0.005 N m.
- The tip pushes up on the ledge 0.5 to 1.7 mm past the pivot, so the loaded catch presses the pedal onto its stop: the
  latch holds by contact alone. Set, the arm bar holds the kill bar short of its stop (174.1 degrees in the demo; the
  stop is at 175), and the tip pushes up on the ledge with 4 N (21 N on the arm bar 20 mm from its staple, 106 mm to
  the tip).
- Hooking the tip: pressed down onto the chamfer, the tip tips the pedal back until the ledge clears it (4 degrees, the
  ledge sliding 1.2 mm), drops below the ledge, and the pedal springs back over it.
- Firing it: pressing the pedal's plate tips it the same way; at 4 degrees the ledge slides off the tip, and the kill
  bar throws the arm bar up and snaps forward.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The plank with the trap's board is one loose rigid
body; the kill bar, the striking bar's sleeve, the arm bar and the pedal are loose rigid bodies, each on a PhysX hinge
(a `Hinge` from `bodykit.py`). Everything the scorer checks is measured in the plank's own frame.

- **The spring.** The kill bar's hinge drive gives the 0.326 N m per radian; the task file adds the 0.563 N m preload
  as a constant torque pair, on the kill bar about its axle and the opposite one on the board (a hinge drive alone
  cannot hold a preload over 175 degrees of travel).
- **The latch is contact physics.** No rule holds or releases anything. The kill bar presses up on the arm bar, the arm
  bar's tip presses up under the ledge, and the pedal rests on its stop. A tip held near the catch, or resting on top
  of its ledge, holds nothing: the kill bar throws it off (see the checks below).
- **The sleeve.** The real 3 mm striking bar turns easily between rubber pads; a 12 mm bar pinched in the simulated
  V-jaw would drag. The sleeve turns freely on the bar, so the bar turns in the fingers as the real one does.
- **Reading the state.** After every control step the task file reads the poses: the kill bar's angle and its change
  over the step, the arm bar's tip in the pedal's frame (in the notch under the ledge, or not), and where the grippers
  are.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This gives
`score()` each body's own pose and lets the task file read the simulation after every control step.

## Scoring

The same rule in the real trial and in the simulation:

| Outcome | Real trial (the video, then a pencil) | Simulation |
|---|---|---|
| Snapped: 0.0 | the kill bar, once pulled back past 30 degrees, swings forward on its own | it swings forward more than 10 degrees within one 50 ms control step (200 degrees a second) |
| Hand in the strike path: 0.0 | a gripper inside the path of the unheld kill bar | a TCP or fingertip 2.5 to 9.5 cm from the axle, within 5.5 cm of the middle line, more than 15 degrees ahead of the kill bar, while no TCP is within 3.5 cm of the striking bar and the tip is not in the catch |
| Set: 0.5 | at the end, the kill bar held back by the arm bar, its tip under the catch's ledge, no gripper on either; then a light pencil touch on the pedal's far half fires the trap | the top of the tip's end in the notch (between the ledge's near edge and the back wall, at most 2.5 mm below the ledge's underside) and the kill bar at least 160 degrees back |
| Full: 1.0 | set, each gripper at least 4 cm from the striking bar and the arm bar's tip, both arms back where they started | the same; each TCP within 5 cm of its start |
| Pulled back: 0.25 | the kill bar pulled back over the top to the near end at some point | at least 165 degrees back |

The operator makes the pencil touch after the trial, hand clear, from the side. A trap that does not fire on it is not
set, whatever the video shows. The scorer is `task_score`: 1.0 once `success()` has held for 1 s (full score with the
kill bar and the arm bar still), otherwise `score()` at the end of the episode.

## Checks run

Each check is the scripted demo or a variant of it (`TRAP_DEMO=<name>`), at the run limit with `-E object_jitter=0`.

| Run | What it does | What happened | Score |
|---|---|---|---|
| Demo | the scripted demo below | full score: set at 174 degrees, both arms home (614 steps) | 1.0 |
| `near` | the left holds the arm bar's tip against the catch's chamfer, 7 to 8 mm from its hooked place and not under the ledge, with the kill bar pulled right back (the old rule latched here); the right lets go and leaves, then the left lets go | nothing latches: let go, the kill bar pushes the arm bar's tip up off the catch into the left's fingers; when the left rises, the kill bar snaps forward | 0.0 |
| `ontop` | the left lays the arm bar's tip on top of the catch's lip without pressing it in and leaves; the right lets go | the kill bar throws the arm bar off the lip and snaps forward within one 50 ms step | 0.0 |
| `onehand` | the right does its part; the left never moves | with nothing holding the plank, the right's pull drags the trap 17 cm across the table and the kill bar is never pulled back | 0.0 |
| `pencil` | the full demo, then, with both arms home, a closed fingertip touches the pedal's far half | the trap fires: the pedal tips 6 degrees, the arm bar flies back and the kill bar swings forward onto the fingertip | set (1.0) before the touch |
| Dream-1 | the instruction, 900 steps | both arms move near their bases and never reach the trap | 0.0 |

## Run and score a trial

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the rat trap, with its arm bar hooked in the catch holding the kill bar back, without ever letting the kill bar snap and without a gripper in the kill bar's path while nothing holds it back. Leave both grippers clear of the set trap and both arms back where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/trap-dream
```

The scripted demo reaches full score:
1. The left hand, leaning out to the left, holds the plank down beside the trap.
2. The right, leaning out to the right with its grip pitched back 25 degrees, pinches the striking bar where it lies on
   the board's far end, its far finger past the board's end.
3. It carries the bar up and over the top to the near end, leading it along its arc against the spring, the bar
   turning in its fingers, and presses it down there.
4. The left lets go of the plank, pushes the arm bar's upper end forward so that it falls over the kill bar, and
   presses the arm bar down near its tip until the tip drops past the chamfer under the ledge; it rises and withdraws.
5. The right eases up, the kill bar settling against the arm bar, opens its fingers and rises.
6. Both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the rat trap, with its arm bar hooked in the catch holding the kill bar back, without ever letting the kill bar snap and without a gripper in the kill bar's path while nothing holds it back. Leave both grippers clear of the set trap and both arms back where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/trap-demo
```

Add `TRAP_DEMO=near` (or `ontop`, `onehand`, `pencil`) in front of the scripted command for the checks above. Set
`TRAP_DEBUG=1` to print what the score is built from twice a second: the kill bar's angle, the arm bar's angle, the
pedal's tilt, the tip's place in the pedal's frame, whether it is in the catch, held by a hand, snapped or unsafe, and
where each TCP is.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/trap-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 900 steps (590 s on the clock, with Dream answering in 0.94 s on average), and about 5 minutes for the scripted demo (614 steps, 30.7 s of simulated time, 272 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-KJCHWS. Its review said:

> Construct validity (not met): The catch locks when the bar tip comes within 8 mm, without requiring the tip to physically engage the catch.
>
> Clarity and reproducibility (not met): The custom catch's dimensions and construction are not specified sufficiently to reproduce its latching mechanism for real trials.
>
> What to change: The earlier request to leave the trap loose and the category change have been addressed. A new issue not raised before is that the simulated catch locks when the bar tip is merely within 8 mm, without physical engagement. Please use a named, affordable rat trap or provide a reproducible catch design, and update the simulation so the bar must actually engage the catch before it latches.

The review before that (TK-T9Q5HL) asked for the trap not to be clamped to the table; the plank with the trap lies
loose, and still does. The first review (TK-5Y7J8Y) asked for the Societal relatability category and a household
framing; both are kept.

What changed in response:

- **A named, affordable trap (setup, README "The trap").** The trap is a stock Victor M326 Professional rat trap, used
  as sold (about $3; listing linked above). The real trial uses its own catch, so the latching mechanism comes with
  the product; the custom brass catch is gone. The simulation is rebuilt to the trap: board 178 x 83 x 12.7 mm (the
  maker's listing), kill bar, two-coil spring, wire arm bar on its staple, and the pedal carrying the catch.
- **The spring follows the published tests of this trap (setup, README "The trap").** Calibrated to 8.2 N clamping
  force at 10 mm and 3.16 J at impact (DOC Science Internal Series 83): 21 N at the striking bar to hold it pulled
  back, against 4 N before. The demo pulls it over against that spring.
- **The catch engages by contact (code, fidelity, README "The catch").** The 8 mm rule is gone; nothing in the task
  file latches or releases anything. The tip has to be pressed down a 45-degree chamfer, which tips the pedal back
  until the ledge clears it, and is then held under the 1.2 mm ledge only by the kill bar's push. Check: the demo
  sets the trap and scores 1.0 (the same result in three runs).
- **A tip merely near the catch does not latch (README "Checks run").** Check `near`: the arm bar's tip held 7 to 8 mm
  from its hooked place, not under the ledge, while the kill bar is pulled right back: nothing latches; when the
  hands let go, the kill bar snaps forward. Score 0.0.
- **A tip resting on the lip does not latch.** Check `ontop`: the tip set on top of the ledge and left there: the
  kill bar throws it off and snaps forward. Score 0.0.
- **The catch fires as the real one does.** Check `pencil`: after the full demo, a closed fingertip touches the
  pedal's far half: the pedal tips, the ledge slides off the tip and the trap fires.
- **The real-trial check is observable (scoring).** Set is read from the video (the kill bar back, the arm bar's tip
  under the ledge, no gripper on either), then confirmed by a light pencil touch on the pedal that must fire the
  trap.
- **Two hands are still needed.** Check `onehand`: with nothing holding the plank, the right's pull drags the trap
  17 cm and the kill bar is never pulled back. Score 0.0.
- **Scene changes that follow from the real trap (setup).** The plank is oak, 32 x 22 x 2.5 cm (plank and trap 1.47
  kg), so it stays down under the real spring while one hand holds it; snap detection reads the kill bar's angle
  change over each 50 ms step, since this spring swings the bar right forward within one step.
