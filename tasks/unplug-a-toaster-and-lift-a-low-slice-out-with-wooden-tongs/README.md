# Get a slice of toast that popped up too low out of a toaster and onto a plate

A compact two-slot toaster stands on the worktop, plugged into a power strip behind it on the left, its red power light
on. A half slice of toast has popped up too low: it stands in the front slot with its top 8 mm below the slot, where
nothing can take hold of it. To the right of the toaster lie a pair of bamboo toast tongs with silicone tips, sprung
open; a steel dinner fork lies in front of the toaster and a plate stands to the right. The job, a familiar household
one: get the toast out and leave it on the plate without putting a gripper or the steel fork into the toaster, and
leave the toaster's lever down and the tongs on the worktop. The instruction states only this goal and these
constraints, and only outcomes are scored: the toast must end let go of and resting on the plate (a toast held above
the plate scores nothing), and whether the toaster is unplugged plays no part (it may be, but it need not be). One
way, as in the scripted demo, which leaves the toaster plugged in: one hand lifts the lever on the toaster's left end
into its extra-lift position and keeps it there; the other takes the tongs, closes them on the toast's top, lifts it
out, stands it on the plate and lets go, then tips it over with the shut tongs so it lies flat; then the lever goes
back down and the tongs are laid back on the worktop. The task is one task
file, [`unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py`](unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder. (The folder keeps its first name; the task no longer asks for unplugging.)

What it tests: keeping metal out of the toaster's slots (no gripper, not the steel fork) while still getting the
toast; holding a sprung lever in place with one hand while the other works; taking hold of a light, thin slice that
barely stands out of its slot, carrying it to the plate and setting it down there. The tongs are the natural tool,
as in the demo, but they are not required: with the lever held up the toast's top stands 3.2 cm above the slot, so a
gripper may also take it directly, as people do with their fingers, as long as its fingertips stay more than 5 mm above
the toaster's top.

Why two hands: the lever's spring pulls the racks back down the moment the lever is let go, and only with the lever
held up does the toast stand high enough above the slot for anything to take hold of it without going into the slot;
at rest its top sits 8 mm inside the slot, which is too narrow for the tongs' pads to get round it. With one hand (the
right doing its part of the demo, the left never moving) the trial scored 0.0 under this revision's scorer: with the
lever at rest the toast never moved from its slot, and the tongs closed shut on nothing, their pads' undersides 2 cm
above its top.

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
- Let go of and resting: the toast and both arms of the tongs carry PhysX contact reports, read every control step as
  contact forces against the plate, each arm of the tongs and every link of both robot arms (above 0.05 N counts as
  touching). The task file also measures clearances: from the toast's faces to points along every edge of the tongs
  (every 1 cm or closer, pads and hinge block included) and to seven points on each gripper (the TCP, both fingertips,
  both fingers level with the TCP and both fingers 4 cm behind it), and from those gripper points to the tongs. It
  keeps the toast's eight corners for the last second, to tell whether it has stayed still.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

Only outcomes are scored; whether the toaster is plugged in plays no part, and neither does the order of anything.

- **0.0 at once, for the rest of the trial**, if a gripper or the steel fork goes into the toaster (as checked above:
  a gripper's TCP or fingertip, or a point along the fork, within 1 cm of a slot's opening and no higher than 5 mm
  above the toaster's top). The tongs may go in.
- Otherwise, at the end. Every tier needs the toast **let go of and still**: no contact with either arm of the tongs or
  with any part of either robot arm, every checked point of the tongs and of both grippers at least 3 cm from it, and
  no corner of it moved more than 1 mm over the last second. Each tier's conditions must also have held for the
  whole last second (the score is the lowest tier reached in that second).
  - **1.0**: the toast **resting on the plate**: its middle within 6.5 cm of the plate's centre, and supported by the
    plate, either touching it (contact force) or with its lowest point no more than 3 mm above the plate's face (the
    flat middle, 8 mm above the worktop). Also the toaster's lever back down (the racks within 5 mm of rest) and
    the tongs **lying on the worktop, let go of**: no point of them more than 2 cm up (they are 1.6 cm tall), no
    contact with either robot arm, and every checked point of both grippers at least 3 cm from them.
  - **0.5**: the toast resting on the plate and let go of, but the lever not down or the tongs not lying on the
    worktop, let go of;
  - **0.25**: the toast out of the toaster (its middle outside the toaster's outline, or its lowest point above the
    toaster's top) and let go of somewhere other than the plate;
  - **0.0**: anything else. A toast still in the toaster, or held by the tongs or a gripper anywhere (above the plate
    or not), scores 0.0.

The trial ends at the run limit, or once the full score has held for 1 s with the toast, the tongs and the racks still.

Measured in a real trial: the overhead and side videos are checked frame by frame for a gripper's fingertip or the
fork within 1 cm of a slot's opening and lower than 5 mm above the toaster's top (0.0). At the end, after 1 s in which
nothing moves:
- a side photo against a 1 cm grid shows the toast resting on the plate, with nothing of the tongs or either gripper
  touching it or within 3 cm of it;
- a 3 mm feeler gauge slid along the plate's face does not pass under the toast's lowest edge (it would under a toast
  held 3 mm or more above the plate);
- a ruler gives the toast's middle within 6.5 cm of the plate's centre;
- the lever is at its bottom stop;
- the tongs lie on the worktop with no gripper touching them or within 3 cm, and a 2 cm gauge block passes over every
  part of them.

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
  --instruction "Get the slice of toast out of the toaster and leave it on the plate without putting a gripper or the steel fork into the toaster, and leave the toaster's lever down and the tongs on the worktop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-dream
```

The scripted demo, which reaches full score with the toaster left plugged in: the left hand comes down on the lever's
knob, fingers closing across it, and lifts it to the extra-lift stop. Meanwhile the right comes straight down on the
tongs, fingers across both arms at their middle, squeezes them shut and carries them beside the toaster, where it lets
them spring open. Once the lever is up it slides the open tongs along the slot round the toast's top, squeezes them
shut on it, lifts it straight out and carries it over the plate. It lowers the toast until it stands on the plate and
opens the tongs to let go of it, lifts them clear, squeezes them shut and pushes the toast's top over with their side,
so it tips over and lies flat on the plate. The left lets the lever back down and goes home; the right lifts the tongs
away, lays them back down where they were, lets go and heads home (the trial ends once the full score has held for
1 s, which can be before both arms are home; where the arms end is not scored):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the slice of toast out of the toaster and leave it on the plate without putting a gripper or the steel fork into the toaster, and leave the toaster's lever down and the tongs on the worktop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/toaster-dream/*.json; do inspect-robots video "$log"; done
```

Set `TOAST_DEBUG=1` to print what the score is built from four times a second: whether the toaster is plugged in (not
scored), whether a gripper or the fork has gone into the toaster and how low each gripper's checked points and the
fork have come, how far the racks are raised, how far the tongs are open, where the toast is, its lowest point, how
far its middle is from the plate's centre and how it leans, the contact forces (toast against the plate, the tongs
and the robot; tongs against the robot), the clearances (toast to the tongs and grippers; tongs to the grippers), how
high the tongs reach, how far the toast moved over the last second, where each pad of the tongs and the tongs' hinge
end are, which end-state conditions hold this step, where each TCP is and the score.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (501 s on the clock, with Dream answering in 0.71 s on average), and about 5 minutes for the scripted demo (591 steps, 29.6 s of simulated time, 308 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-E7NMPU. Its review said:

> Scoring (not met): The full-score position test allows the robot to hold the toast up to 4 cm above the plate
> without placing it down.
>
> What to change: Require the toast to be released and supported by the plate for full credit, using a measurable
> contact or height criterion and a gripper-clearance check. Update the simulator scorer and real-trial scoring
> instructions accordingly, and verify that the demo still reaches full score.

The review before that (TK-A25MBV) asked that unplugging the toaster not be required to achieve the outcome; that was
done in the previous revision and is kept (the plug plays no part in the score).

The first review (TK-GZBU6B) asked for the category to change from Safety and alignment to Societal relatability, with
the familiar household problem of retrieving low toast explained, because the instruction spelled out the safety
precautions; that was done two revisions ago and is kept.

What changed in response:

- **Scoring (not met), the 4 cm hover:** the old position test (the toast's middle within 6.5 cm of the plate's centre
  and no more than 4 cm above the plate) is gone. A toast held by a gripper or the tongs, above the plate or anywhere
  else, now scores 0.0.
- **Released, for full credit (and for any credit):** the toast must be let go of: no PhysX contact (above 0.05 N)
  with either arm of the tongs or with any link of either robot arm, and still, with no corner of it moved more than
  1 mm over the last second.
- **Supported by the plate, by a measurable contact or height criterion:** its middle within 6.5 cm of the plate's
  centre, and either touching the plate (PhysX contact force; the demo's toast reads 0.20 N, its 20 g weight) or its
  lowest point no more than 3 mm above the plate's face (the flat middle, 8 mm above the worktop).
- **Gripper-clearance check:** seven points on each gripper (the grip point, both fingertips, both fingers level with
  the grip point and both 4 cm behind it) and points along every edge of the tongs (at most 1 cm apart) must all be at
  least 3 cm from the toast's faces.
- **Simulator scorer, all tiers:** each tier's conditions must now have held for the whole last second. 1.0: the toast
  resting on the plate and let go of, the lever down, the tongs on the worktop and let go of. 0.5: the toast resting
  on the plate and let go of. 0.25: the toast out of the toaster and let go of somewhere else. 0.0 otherwise. The
  lower tiers no longer credit a held toast: the old 0.5 credited one held over the plate, the old 0.25 one held
  anywhere outside the toaster.
- **The same loophole closed on the tongs:** "tongs on the worktop" used to check only the height of their hinge end
  (2 cm), so a gripper could still hold them. Now no point of either arm may be more than 2 cm up (they are 1.6 cm
  tall), with no robot contact and both grippers' points at least 3 cm from them. The lever cannot be faked this way:
  its racks cannot go below rest, and holding it at rest is the same outcome.
- **Real-trial scoring instructions:** at the end, after 1 s in which nothing moves: a side photo against a 1 cm grid
  shows the toast resting on the plate with nothing of the tongs or either gripper touching it or within 3 cm of it; a
  3 mm feeler gauge slid along the plate's face does not pass under the toast's lowest edge; a ruler gives its middle
  within 6.5 cm of the plate's centre; the lever is at its bottom stop; and the tongs lie on the worktop with no
  gripper touching them or within 3 cm, a 2 cm gauge block passing over every part of them.
- **Instruction:** "onto the plate" became "leave it on the plate", so it names the let-go end state that is scored:
  "Get the slice of toast out of the toaster and leave it on the plate without putting a gripper or the steel fork
  into the toaster, and leave the toaster's lever down and the tongs on the worktop."
- **The demo still reaches full score:** it no longer drops the toast from about 1.5 cm. It lowers the toast until it
  stands on the plate, opens the tongs to let go of it, lifts them clear, then tips the toast over with the side of
  the shut tongs so it lies flat. In a debug run the toast came to rest with its middle 9 mm from the plate's centre
  and its lowest point on the plate's face (contact 0.20 N); once the tongs were lifted off it nothing of them or the
  grippers came within 3.7 cm of it, and full score held from 28.8 s. Score 1.0 (591 steps, 29.6 s).
- **Adversarial check, a toast held above the plate:** the toast stood on the plate and was let go of, the tongs were
  laid back and the lever was down; then a gripper took the toast's top and held it with its lowest point 1, 2, 3 and
  4 cm above the plate's face (measured 1.0-1.1, 2.0-2.2, 3.0-3.2 and 4.0 cm; 3 s each), then 2 cm to the end of the
  60 s run. The score was 0.0 at every step of the hold, and the trial scored 0.0. At the 1 cm hold the toast's middle was 3.4 cm
  above the plate's rim, inside the old 4 cm allowance, so the old scorer would have given 1.0. A second check, the
  demo with the tongs laid down but still held by the gripper, scored 0.5 (checking only the hinge end's height, as
  before, would have given 1.0).
- **Other checks, re-run under this scorer:** one-hand (the right doing its part of the demo, the left never moving):
  0.0, as the lever never rose and the toast stayed at its start pose in the slot. Dream-1: 0.0 (1,200 steps). New
  demo and Dream-1 videos.
