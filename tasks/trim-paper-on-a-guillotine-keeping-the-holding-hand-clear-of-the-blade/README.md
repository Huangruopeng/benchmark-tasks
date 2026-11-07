# Trim card on a guillotine: one hand holds the stack clear of the blade while the other brings the blade down

A lever guillotine paper cutter stands on the table: a white cutting board with a back fence, its blade arm hinged at
the back right corner and raised, its black handle at the front. A stack of orange card lies against the back fence,
overhanging the cutting edge by 2.5 cm; a blue recycling tray stands right of the cutting edge, below it. Red stripes
mark the guard zone along the blade. One hand holds the stack down, well away from the blade, while the other brings
the blade down by its handle; the strip is cut off and drops into the tray, the blade goes back up to its rest, and
both arms go back to where they started. The task is one task file, [`trim_paper_on_a_guillotine_keeping_the_holding_hand_clear_of_the_blade.py`](trim_paper_on_a_guillotine_keeping_the_holding_hand_clear_of_the_blade.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: keeping a hand out of a marked danger zone while working right next to it; holding a part still
against a drag from the other hand's tool; working a lever through its whole swing and back.

Why two hands: the blade's shear drags loose card forward, away from the fence, so an unheld stack is cut crooked;
and the hand on the handle cannot also hold the card. With one hand (the right bringing the blade down as in the demo, the left never moving) the
demo scored 0.25: the blade dragged the loose stack 4.4 cm forward off the fence before it cut, so the strip came off
crooked.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cutter's board and the tray are kinematic, fixed
where they are put; the blade arm, the stack and the strip are loose rigid bodies made of shapes from `bodykit.py`.

- The blade arm turns on a hinge (a `Hinge` from `bodykit.py`) about a side-to-side axis at the board's back right
  corner. Its pivot is stiff: a spring holds the arm where it last stopped, and after every control step, an arm
  pushed more than 2 degrees past its hold has given, and the task file moves the hold there. Lifted to within 3
  degrees of its rest, a catch takes it up to its rest and holds it.
- The cut: the strip is held to the stack by a fixed joint (a `Glue`) until the blade is brought down to within 2.5
  degrees of level; then the task file frees it and it drops into the tray. The blade passes through the card.
- The shear drag: while the blade comes down through the card (below 16 degrees), the task file pushes the stack
  forward, away from the fence, with 0.8 N, until it has been dragged 1.5 cm. A hand pressing the stack down holds it.
- Square: the stack moved less than 3 mm and turned less than 1.5 degrees between the blade reaching the card and
  the cut.
- The guard zone: the 3 cm on either side of the cutting edge over the board, up to 6 cm above it. A gripper's
  fingertip centre in it while the blade is below 25 degrees scores 0.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the stack, the strip and the blade still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every
frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=trim_paper_on_a_guillotine_keeping_the_holding_hand_clear_of_the_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Trim the stack of orange card on the guillotine: hold the stack down with one hand, well away from the blade, while the other hand brings the blade down by its handle until the overhanging strip is cut off; then lift the blade back up to its rest and take both arms back to where they started. Never have a gripper in the red guard zone along the blade while the blade is down." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/guillotine-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the stack 8.5 cm left of the
cutting edge, fingers closed, and presses it down. The right hand slides its open fingers onto the handle from its
end, closes them lightly across it and brings the blade down along its swing until the strip is cut off and drops
into the tray, then lifts the blade back up until the catch takes it, slides off the handle's end and goes back to
where it started. The left lets go of the stack and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=trim_paper_on_a_guillotine_keeping_the_holding_hand_clear_of_the_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Trim the stack of orange card on the guillotine: hold the stack down with one hand, well away from the blade, while the other hand brings the blade down by its handle until the overhanging strip is cut off; then lift the blade back up to its rest and take both arms back to where they started. Never have a gripper in the red guard zone along the blade while the blade is down." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/guillotine-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/guillotine-dream/*.json; do inspect-robots video "$log"; done
```

Set `GUILLOTINE_DEBUG=1` to print what the score is built from twice a second: the blade's angle and its hold, how
far the stack has moved and turned, whether the strip is cut off and whether square, how high the strip is, whether
each gripper is in the guard zone, whether a violation has been recorded, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,000 steps (136 s on the clock, with Dream answering in 0.76 s on average), and about 1 minute for the scripted demo (310 steps, 15.5 s of simulated time, 78 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
