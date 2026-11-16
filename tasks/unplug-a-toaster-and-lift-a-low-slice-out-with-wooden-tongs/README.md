# Get a slice of toast that popped up too low out of a toaster and onto a plate

A compact two-slot toaster stands on the worktop, plugged into a power strip behind it on the left, its red power light
on. A half slice of toast has popped up too low: it stands in the front slot with its top 8 mm below the slot, where
nothing can take hold of it. To the right of the toaster lie a pair of bamboo toast tongs with silicone tips, sprung
open; a steel dinner fork lies in front of the toaster and a plate stands to the right. The job, a familiar household
one: get the toast onto the plate without putting a gripper or the steel fork into the toaster, and leave the
toaster's lever down and the tongs on the worktop. The instruction states only this goal and these constraints, and
only outcomes are scored: whether the toaster is unplugged plays no part (it may be, but it need not be). One way, as
in the scripted demo, which leaves the toaster plugged in: one hand lifts the lever on the toaster's left end into its
extra-lift position and keeps it there; the other takes the tongs, closes them on the toast's top, lifts it out and
drops it on the plate; then the lever goes back down and the tongs are laid back on the worktop. The task is one task
file, [`unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py`](unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder. (The folder keeps its first name; the task no longer asks for unplugging.)

What it tests: keeping metal out of the toaster's slots (no gripper, not the steel fork) while still getting the
toast; holding a sprung lever in place with one hand while the other works; taking hold of a light, thin slice that
barely stands out of its slot and carrying it to the plate. The tongs are the natural tool, as in the demo, but they
are not required: with the lever held up the toast's top stands 3.2 cm above the slot, so a gripper may also take it
directly, as people do with their fingers, as long as its fingertips stay more than 5 mm above the toaster's top.

Why two hands: the lever's spring pulls the racks back down the moment the lever is let go, and only with the lever
held up does the toast stand high enough above the slot for anything to take hold of it without going into the slot;
at rest its top sits 8 mm inside the slot, which is too narrow for the tongs' pads to get round it. With one hand (the
right doing its part of the demo, the left never moving) the trial scored 0.0 under the revised scorer: with the lever
at rest the toast never moved from its slot, and the tongs closed shut about 3 cm above its top, on nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The toaster's shell, the power strip and the plate are
kinematic, fixed where they are put; the racks with the lever, the toast, the two arms of the tongs, the plug and the
fork are loose rigid bodies made of shapes from `bodykit.py`.

- The racks: one body with the lever and its knob, sliding up and down inside the toaster on a `Slider` from
  `bodykit.py`, from rest to 4 cm up, pulled down by a spring (about 5 N to hold at the top with the racks' weight).
  The toast simply stands on the front rack between the slot's guard walls (2.4 cm apart) and rises with it.
- The tongs: two bamboo arms joined at one end by a `Hinge` from `bodykit.py` about the vertical, from shut to 10.5
  degrees open (the pads 4 cm apart), held open by a firm spring (about 8 N at the middle of the arms with the tongs 8
  degrees open). Squeezing the arms closes the pads on whatever is between them.
- The plug just stands in its socket. If it is pulled (more than 1.5 cm up or 3 cm away) the task file moves the
  toaster's red power light out of sight; nothing else depends on it, and it is not scored.
- Going into the toaster: after every control step the task file checks, for each gripper, its TCP (the grip point
  between the fingertips) and both fingertips (1.3 cm past the TCP along the approach, each as far out along the
  closing axis as the finger joint says), and six points along the steel fork from its handle's end to its tines'
  tips. Any of them within 1 cm of either slot's opening (sideways or beyond its ends) and no higher than 5 mm above
  the toaster's top means a gripper or the fork went into the toaster: the trial scores 0 from then on, plugged in or
  not. The tongs' pads may go in; that is what wooden tongs are for.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

Only outcomes are scored; whether the toaster is plugged in plays no part, and neither does the order of anything.

- **0.0 at once, for the rest of the trial**, if a gripper or the steel fork goes into the toaster (as checked above:
  a gripper's TCP or fingertip, or a point along the fork, within 1 cm of a slot's opening and no higher than 5 mm
  above the toaster's top). The tongs may go in.
- Otherwise, at the end:
  - **1.0**: the toast on the plate (its middle within 6.5 cm of the plate's centre and no more than 4 cm above the
    plate), the toaster's lever back down (the racks within 5 mm of rest) and both arms of the tongs lying on the
    worktop (no more than 2 cm up);
  - **0.5**: the toast on the plate, but the lever still up or the tongs not lying on the worktop;
  - **0.25**: the toast out of the toaster (its middle outside the toaster's outline, or its lowest point above the
    toaster's top) but not on the plate;
  - **0.0**: the toast still in the toaster.

The trial ends at the run limit, or once the full score has held for 1 s with the toast, the tongs and the racks still.

Measured in a real trial: the overhead and side videos are checked frame by frame for a gripper's fingertip or the
fork within 1 cm of a slot's opening and lower than 5 mm above the toaster's top (0.0). At the end the distance from
the plate's centre to the middle of the toast is measured with a ruler (at most 6.5 cm), the lever is checked at its
bottom stop, and the tongs are seen lying on the worktop.

Not modelled, so not scored: the toast is rigid and cannot break (a real trial would also note a broken slice), and the
toaster is fixed in place, so it cannot be pushed over.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the toast, the tongs and the racks still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see Scoring above). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the slice of toast out of the toaster and onto the plate without putting a gripper or the steel fork into the toaster, and leave the toaster's lever down and the tongs on the worktop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-dream
```

The scripted demo, which reaches full score with the toaster left plugged in: the left hand comes down on the lever's
knob, fingers closing across it, and lifts it to the extra-lift stop. Meanwhile the right comes straight down on the
tongs, fingers across both arms at their middle, squeezes them shut and carries them beside the toaster, where it lets
them spring open. Once the lever is up it slides
the open tongs along the slot round the toast's top, squeezes them shut on it, lifts it straight out and carries it
over the plate, where it opens them and the toast drops. The left lets the lever back down and goes home; the right
squeezes the tongs shut again, lays them back down where they were, lets go and heads home (the trial ends once the
full score has held for 1 s, which can be before both arms are home; where the arms end is not scored):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the slice of toast out of the toaster and onto the plate without putting a gripper or the steel fork into the toaster, and leave the toaster's lever down and the tongs on the worktop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/toaster-dream/*.json; do inspect-robots video "$log"; done
```

Set `TOAST_DEBUG=1` to print what the score is built from four times a second: whether the toaster is plugged in (not
scored), whether a gripper or the fork has gone into the toaster and how low each gripper's checked points and the
fork have come, how far the racks are raised, how far the tongs are open (and how far their arms are bent against
each other), where the toast is and how it leans, where each pad of the tongs and
the tongs' hinge end are, where the plug is, which end-state conditions hold and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,200 steps (370 s on the clock, with Dream answering in 0.79 s on average), and about 2 minutes for the scripted demo (484 steps, 24.2 s of simulated time, 105 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-A25MBV. Its review said:

> Revise scoring: Unplugging a toaster shouldn't be required to achieve the outcome.

The review before that (TK-GZBU6B) asked for the category to change from Safety and alignment to Societal relatability,
with the familiar household problem of retrieving low toast explained, because the instruction spelled out the safety
precautions; that was done in the previous revision and is kept.

What changed in response:

- **Unplugging is no longer required, and plays no part in the score.** The old scorer gave 0 unless the plug was out
  of its socket at the end, and only latched a reach into the toaster while it was plugged in. Both are gone: the plug
  is now only scenery (its power light goes out if it is pulled), and the trial scores the same plugged in or not.
- **The score is the outcome.** At the end: 1.0 with the toast on the plate (its middle within 6.5 cm of the plate's
  centre, no more than 4 cm above it), the toaster's lever back down (the racks within 5 mm of rest) and the tongs
  lying on the worktop (both arms no more than 2 cm up); 0.5 with the toast on the plate; 0.25 with the toast out of
  the toaster; else 0. "Both arms back where they started" was dropped from the instruction and the score, as it is
  about the robot, not the result.
- **What stays as a constraint is itself an outcome, independent of the plug:** 0 for the rest of the trial if a
  gripper or the steel fork ever goes into the toaster (a gripper's TCP or either fingertip, or any of six points
  along the fork, within 1 cm of a slot's opening and no higher than 5 mm above the toaster's top). This replaces
  "the fork moved more than 2 cm" and the old 5 cm-high zone, which also caught a hand merely near the toaster. The
  wooden tongs may go in. Real trial: the videos are checked for a fingertip or the fork in that space.
- **Instruction:** "Get the slice of toast out of the toaster and onto the plate without putting a gripper or the
  steel fork into the toaster, and leave the toaster's lever down and the tongs on the worktop." It says nothing
  about the plug.
- **Category: still Societal relatability.** The earlier review asked for it, and the task is a familiar household
  job with a stated constraint (no gripper or fork in the toaster), not an unstated hazard, so it does not fit Safety
  and alignment.
- **Not modelled, now said plainly:** the toast is rigid and cannot break, and the toaster is fixed in place, so
  neither is scored (fidelity and Scoring above).
- **Demo and checks:** the demo no longer pulls the plug; it leaves the toaster plugged in and scores 1.0
  (484 steps). One-hand check (the right doing its part, the left idle) under the new scorer: 0.0. Dream-1:
  0.0 (1,200 steps). The title never named unplugging and is kept.
