# Light an emergency glow stick: bend it with both hands until its inner vial snaps, shake it, lay it in the tray

An unlit 30 cm emergency glow stick lies in its cardboard packet on the table: a clear tube of pale liquid with a thin
glass vial inside. Further out stands a long black slate tray. Both hands take the stick, one near each end, lift it
out of the packet and bend it at the middle until the vial inside snaps: a bright spot shows at the bend, where the two
liquids first mix. They straighten it and shake it forward and back three times, and it lights up green all along.
Then they lay it in the black tray, let go, and both arms go back to where they started. The task is one task file,
[`snap_a_glow_stick_and_shake_it_alight.py`](snap_a_glow_stick_and_shake_it_alight.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: working one object with both hands together (bending it by turning the two halves against each other
far enough, without dropping it), doing the steps in order (snap, then shake), and putting it down where it belongs.

Why two hands: the stick only bends when its two halves are turned against each other, and its tube is stiff enough to
carry its free half along straight. A hand on one half moves the whole stick. With one hand (the right doing its part
of the demo, the left never moving) the demo scored 0: the free half swung at most 10.6 degrees off straight as the
right turned its half, far short of the 30 degrees that snap the vial, and the stick was never lit. The left alone also
scored 0 (at most 9.3 degrees).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The packet and the tray are kinematic, fixed where
they are put; each half of the stick is a loose rigid body made of shapes from `bodykit.py`, the two joined at the
middle by a `Hinge` about the vertical axis (in the plane the stick lies in), with a spring that pulls it straight
(0.003 N m per degree) and stops at 60 degrees either way.

- The snap: after every control step the task file measures the angle between the two halves' axes; at 30 degrees or
  more the vial has snapped (it stays snapped).
- The shake: from the snap on, the task file follows the stick's middle and counts strokes: at least 2.5 cm one way,
  then at least 0.8 cm back. Six strokes (three shakes back and forth) light the stick.
- The glow: two short bright bands (after the snap) or two full-length green sleeves (once lit), drawn only, are
  moved onto the two halves after every control step; until then they wait under the table.
- In the tray: both halves inside the tray's rim, no more than 1.2 cm above lying flat on its floor.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
both halves of the stick still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0;
see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=snap_a_glow_stick_and_shake_it_alight.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Light the glow stick: pick it up out of its packet with both hands, one near each end, and bend it in the middle until the glass vial inside snaps. Then straighten it, shake it back and forth three times so that it lights up all along, lay it in the black tray, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/glow-dream
```

The scripted demo, which reaches full score: both hands come straight down on the stick, one on each half 11 cm out
from the middle, fingers closing across it, and lift it out of the packet. Each hand swings its half 22 degrees forward
round the middle, turning with it, so the stick bends 44 degrees and the vial snaps; then they straighten it. They shake
it forward and back three times (about 4 cm each way), carry it out over the black tray, lay it down in it, let go and
go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=snap_a_glow_stick_and_shake_it_alight.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Light the glow stick: pick it up out of its packet with both hands, one near each end, and bend it in the middle until the glass vial inside snaps. Then straighten it, shake it back and forth three times so that it lights up all along, lay it in the black tray, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/glow-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/glow-dream/*.json; do inspect-robots video "$log"; done
```

Set `GLOW_DEBUG=1` to print what the score is built from four times a second: where the stick's middle is, how far it
is bent (now and at most), whether the vial has snapped, how many shake strokes have counted, whether it is lit and
lying in the tray, how each half is turned, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (214 s on the clock, with Dream answering in 0.73 s on average), and about 1 minute for the scripted demo (391 steps, 19.6 s of simulated time, 80 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
