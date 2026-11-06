# Hold a backup wrench with one hand while the other turns a pipe union loose, then take the pipe section off

A short pipe line runs along the table: a galvanised-steel pipe section resting in two steel cradles, a union nut joining it to the next pipe
piece, and that piece screwed into a fixed elbow. Two steel wrenches are already on: a red one on the union nut, its handle reaching up to the
right, and a blue backup wrench on the pipe piece's hex, its handle reaching up to the left. One hand holds the backup
wrench down so the pipe piece cannot turn; the other presses the red wrench down, turning the union nut loose. Loose,
the union comes apart: the pipe section is lifted off its cradles and laid on the table, and both arms go back to where
they started. The task is one task file, [`break_loose_a_pipe_union_with_two_wrenches.py`](break_loose_a_pipe_union_with_two_wrenches.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: holding a reaction with one hand while the other applies a turning force; knowing which part to hold.

Why two hands: the union nut holds tighter than the pipe piece's joint at the elbow, so turning the nut with no backup
wrench held turns the whole pipe piece and loosens the wrong joint. With one hand (the right pressing the red wrench as
in the demo, the left never moving) the demo scored 0.0: the elbow joint turned 14 degrees loose, past the 12 that
counts as loosening the wrong joint.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradles and the elbow are kinematic, fixed where
they are put; the pipe section, the pipe piece and the union nut, each with its wrench, are loose rigid bodies made of
shapes from `bodykit.py`, their masses on the pipe's axis.

- The threads: the pipe piece turns in the elbow, and the union nut on the pipe piece, each on a hinge about the pipe's
  axis (a `Hinge` from `bodykit.py`) with a stop at its tight end. Each hinge has a spring holding it where it last
  stopped (the union nut about 0.3 N m before it gives, the elbow joint about 0.06 N m) and a drag while it turns.
  After every control step, a joint turned more than 2 degrees past its hold has given: the task file moves its hold
  there.
- The union: while the nut is tight, a fixed joint (a `Glue` from `bodykit.py`) holds the pipe section to the pipe
  piece; once the nut has turned 25 degrees loose, the task file switches it off and the section lies free in its
  cradles.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the pipe section laid on the table and
both arms back where they started, with the pipe section still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=break_loose_a_pipe_union_with_two_wrenches.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Break the pipe union loose: hold the blue backup wrench's handle down with one hand so the pipe piece cannot turn, and with the other press the red wrench's handle down, turning the union nut at least 25 degrees loose; then let go of both, lift the pipe section off its cradles and lay it on the table, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/union-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the blue wrench's pad, fingers closed,
and presses it down, holding the pipe piece. The right hand comes straight down on the red wrench's pad, fingers closed,
and presses it down along its swing, turning the union nut 40 degrees loose; the union comes apart. The left rises and
goes back to where it started; the right takes the pipe section by its middle, lifts it out of its cradles, lays it on
the table beside the pipe line and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=break_loose_a_pipe_union_with_two_wrenches.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Break the pipe union loose: hold the blue backup wrench's handle down with one hand so the pipe piece cannot turn, and with the other press the red wrench's handle down, turning the union nut at least 25 degrees loose; then let go of both, lift the pipe section off its cradles and lay it on the table, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/union-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/union-dream/*.json; do inspect-robots video "$log"; done
```

Set `UNION_DEBUG=1` to print what the score is built from twice a second: how far the union nut has turned loose on the
pipe piece, how far the pipe piece has turned in the elbow (and the most so far), whether both hands are clear, and
where each TCP is from its wrench's pad.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (143 s on the clock, with Dream answering in 0.67 s on average), and about 1 minute for the scripted demo (431 steps, 21.6 s of simulated time, 85 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
