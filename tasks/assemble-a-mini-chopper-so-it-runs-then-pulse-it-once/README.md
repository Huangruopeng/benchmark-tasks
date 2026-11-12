# Assemble a mini chopper so it is ready to run, then pulse it once

The parts of a cordless electric mini chopper (a small food processor of about 400 ml) stand on the table, all at real
size and mass: its clear bowl with a stainless pin up the middle and a handle on its left, the stainless S-blade on its
storage post, and the white motor top with its PULSE button and a twist grip on its side. The right hand lifts the
blade off its post by the top of its hub and lowers it over the bowl's pin, then sets the motor top on the bowl with
its arrow at the bowl's open mark. The left hand holds the bowl by its handle while the right takes the twist grip and
turns the motor top 30 degrees anticlockwise: the bayonet clicks home, its safety switch closes and the motor top's
light comes on. The left lets go and goes home; the right lets go, presses PULSE (the blade spins for a second) and
goes home. The task is one task file,
[`assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py`](assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: putting a household appliance together in the only order that works (blade on the pin, then the motor
top, twisted fully home); seating a hub over a pin; setting a part on with its mark lined up for a bayonet; holding a
light part still against a twist with the other hand; keeping the fingers off a blade; and checking the result by
running it.

Why two hands: the bayonet takes 0.3 N m to turn, while the bowl, its blade and the motor top together (0.46 kg on the
bowl's rubber foot ring) resist turning on the table with much less than that. Turned by its grip with nothing holding
the bowl, the motor top takes the bowl round with it and never locks. With one hand (the right doing its part exactly
as in the demo, the left never moving) the demo scored 0.25, the blade on its pin only: the twist turned the whole
bowl 28 degrees while the motor top turned less than 1 degree against it, so it never locked, and PULSE did nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The blade's storage post is fixed; the bowl, the
blade and the motor top are loose rigid bodies made of shapes from `bodykit.py`, at real size and mass.

- The bayonet: a `Thread` joint from `bodykit.py` (a PhysX D6 joint) between the bowl and the motor top, switched on
  when the motor top is set on the rim with its middle within 8 mm of the bowl's axis, tilted at most 8 degrees against
  it and its arrow within 14 degrees of the open mark (as a real bayonet only drops on in that position). It keeps the
  motor top on the bowl's axis, seated on the rim, free to turn between -18 and 31.5 degrees. A spring on its turn holds
  it where it was last left; after every control step, a motor top turned more than 2.5 degrees past that hold has
  slipped, and the task file moves the hold there (it takes 0.3 N m to turn: the friction of the lugs). Turned 28.5
  degrees from the open mark it clicks: the hold moves past the stop, and it stays locked.
- The safety switch counts as closed while the bayonet is locked; the light on the motor top comes on (a lit cap is
  moved onto it).
- The blade's hub (a ring of ten panels, open at the top) fits over the bowl's 8 mm pin with 2.5 mm of play; the pin's
  narrower tip leads it on. The blade is on its pin when its hub's axis is within 4 mm of the pin's, its bottom within
  4 mm of the floor and it is tilted at most 10 degrees.
- PULSE: a gripper's fingertip centre held on the button's top for 3 control steps. With the bayonet locked and the
  blade on its pin, the motor runs: the task file sets the blade spinning at 25 rad/s for 1 s.
- The blade rule: after every control step, each gripper's fingertip centre and its pads' tips are measured against
  the middle line of both blades' cutting parts (the 1.4 cm wide blades beyond 2 cm from the hub's axis); within
  1.5 cm of either (about 8 mm from its edges), the trial scores 0 for the rest of the trial.
- The clear bowl: its wall collides but is not drawn, so it looks clear; its rim, foot edge and four faint seams are
  drawn.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the bowl, the motor top and the blade still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so
every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Assemble the mini chopper so it is ready to run, then pulse it once. Touch the blade only by its hub. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chopper-dream
```

The scripted demo, which reaches full score: the right hand comes down round the top of the blade's hub, lifts the
blade off its post, carries it over the bowl and lowers the hub over the pin, and lets go. It takes the motor top round
its housing (it hangs 14 mm off the TCP along the closing axis, so the demo aims that much to the side), lifts it clear
of the rim, carries it over the bowl and sets it on, its arrow at the open mark, and lets go. Only then does the left
come in from its start, so the two wrists are never closer than about 15 cm over the bowl, and hold the bowl by its
handle. The right takes the twist grip and carries it round the axis 31.5 degrees, its grip turning with it: the motor
top clicks home and its light comes on. The right eases back 2 degrees and keeps hold while the left lets go (wide
open) and goes home; then the right lets go, rises clear above the motor top, presses PULSE with its closed fingers
(the blade spins for a second) and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Assemble the mini chopper so it is ready to run, then pulse it once. Touch the blade only by its hub. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chopper-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/chopper-dream/*.json; do inspect-robots video "$log"; done
```

Set `CHOP_DEBUG=1` to print what the score is built from four times a second: where the bowl is and how far it has
turned, whether the bayonet has caught the motor top and how far it is turned (and where its hold is), whether it is
locked, where the motor top and the blade sit against the bowl, whether the blade is on its pin, the blade rule, when
PULSE ran the motor, and where each TCP is, with the handle's position and both grippers' finger openings.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (491 s on the clock, with Dream answering in 0.56 s on average), and about 5 minutes for the scripted demo (872 steps, 43.6 s of simulated time, 271 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
