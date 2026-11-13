# Grate a block of cheese on a box grater onto a cutting board

A steel box grater stands on a beech-wood cutting board with a block of cheese beside it. One hand holds the grater by the flat handle
reaching left from its top; the other takes the cheese, presses its end against the grating face and rubs it up and
down, so that shreds of cheese fall inside the grater onto the board. The task is one task file,
[`grate_cheese_holding_the_grater_steady.py`](grate_cheese_holding_the_grater_steady.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the grater is light (180 g) and tall, and its teeth bite on every stroke. Unheld, it tips over on the
first upstrokes: with the left hand kept clear, the scripted demo's strokes tip it to 55 degrees within a second and
grate 3 shreds before it falls (0.25). One hand holds it while the other grates.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The grater, the cheese, the board and the shreds are
rigid bodies made of shapes from `bodykit.py`; the board is kinematic, fixed where it is put.

- After every control step the task file checks whether the cheese's end is on the grating face (within 4 mm of it,
  inside its grating area) and how far it has moved up or down along it since the last step. The score is this
  cumulative rubbing distance (with the grater standing): 18 cm for full credit, 10 cm and 2 cm for the partial tiers,
  the same distance a real trial measures from video along the face. Each 2 cm rubbed also brings in one shred: twelve small sticks of cheese wait in a closed box high above the scene,
  out of the cameras' view, and the task file moves the next one inside the grater, just behind the face at the
  cheese's height, from where it falls onto the board.
- The teeth's bite: while the cheese is rubbed on the face, the task file pulls the grater along the stroke at the
  face, 60 N per m/s of rubbing, at most 4 N. Held by its handle the grater barely moves; unheld, an upstroke lifts its
  face side and tips it over.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes at least 18 cm rubbed on the face, the
grater standing on the board, the cheese put down and both arms back where they started, with the fallen shreds, the
grater and the cheese still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grate the block of cheese on the box grater onto the board. Then put the cheese down, leave the grater standing on the board and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the grater's handle near its end and
holds it. The right hand comes straight down on the cheese near its right end, lifts it, presses its left end on the
grating face and rubs it down and up the face six times (about 40 cm of rubbing), then puts it back down and goes
home. The left then lets go, takes the handle near its root, lifts the grater 3 cm and stands it on the board 7 cm to
the left, showing the grated cheese, and goes home too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grate the block of cheese on the box grater onto the board. Then put the cheese down, leave the grater standing on the board and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grate-dream/*.json; do inspect-robots video "$log"; done
```

Set `GRATE_DEBUG=1` to print what the score is built from (where the grater is and how tilted, whether the cheese is
rubbing on the face and at what height, how far it has been rubbed, how many shreds have dropped and lie on the board,
and where each gripper's TCP is) twice a second.

## A real trial

The same box grater (stainless, about 8 x 7 x 13 cm, 180 g, open at the bottom, grating face toward the grating hand,
flat handle on top), a 50 g block of semi-hard cheese (6 x 2.4 x 3.5 cm) and a beech board. Film the grating face from
the side; the score is the cumulative distance the cheese's end moves up and down the face while pressed on it with
the grater standing, read from the video (18 cm for full credit, as in the simulation), plus the same end state.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (408 s on the clock, with Dream answering in 0.70 s on average), and about 4 minutes for the scripted demo (773 steps, 38.6 s of simulated time, 254 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
