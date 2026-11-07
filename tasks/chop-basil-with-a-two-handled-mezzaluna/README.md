# Chop basil with a double-handled mezzaluna: lift it by both handles and rock its curved blade over the leaves

A beech chopping board lies on the table with a row of eight fresh basil leaves on it. In front of it a
double-handled mezzaluna, a curved stainless steel blade with an upright beech handle at each end, stands edge down
in a wooden rest. Both hands take it by its handles, lift it out of the rest together, set its curved edge down across
the basil and rock it from end to end, back and forth, until the leaves are chopped. Then they stand it back in its
rest and both arms go back to where they started. The task is one task file, [`chop_basil_with_a_two_handled_mezzaluna.py`](chop_basil_with_a_two_handled_mezzaluna.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: moving one rigid tool with two hands together (lifting, carrying, and rocking it so its curved edge
rolls along the board without lifting off), keeping the blade upright, then putting the tool back.

Why two hands: a double-handled mezzaluna is worked with a hand on each handle; held by one handle, its weight
swings it down round that hand instead of letting it rock. With one hand (the right doing its part of the demo, the
left never moving) the demo scored 0: the mezzaluna turned round the right hand's grip, its blade swung 3 cm off the
row of basil and its rocking went nowhere near the leaves; not one leaf was chopped (the left alone also scored 0).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The board and the rest are kinematic, fixed where
they are put; the mezzaluna is a loose rigid body made of shapes from `bodykit.py` (its blade ten flat segments along
the curve).

- The basil leaves do not collide (each is a few thin green shapes). After every control step the task file finds the
  lowest point of the blade's edge; while that point is within 6 mm of the board's top, the blade rocked no more than
  30 degrees from level and its plane within 15 degrees of upright, every leaf whose middle is within 1.2 cm of it
  (front to back and side to side) is chopped: the task file swaps the leaf for a little heap of green bits.
- In its rest: the edge's lowest point in the slot (within 6 mm front to back, 2 cm side to side, 8 mm of the slot's
  bottom), the blade upright within 10 degrees.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the mezzaluna still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=chop_basil_with_a_two_handled_mezzaluna.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Chop the basil with the mezzaluna: take it out of its rest by both handles, one in each hand, set its curved blade down across the row of basil leaves on the board and rock it from end to end, back and forth, until every leaf is chopped. Then stand the mezzaluna back in its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mezzaluna-dream
```

The scripted demo, which reaches full score: both hands come down on the handles, take them and lift the mezzaluna
out of its rest together, carry it over the basil and set its edge just over the board. They rock it from end to end
and back, twice, moving it along the row as it rolls on its curve, then lift it, carry it back over the rest, lower it
in, let go and go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=chop_basil_with_a_two_handled_mezzaluna.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Chop the basil with the mezzaluna: take it out of its rest by both handles, one in each hand, set its curved blade down across the row of basil leaves on the board and rock it from end to end, back and forth, until every leaf is chopped. Then stand the mezzaluna back in its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mezzaluna-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mezzaluna-dream/*.json; do inspect-robots video "$log"; done
```

Set `MEZZALUNA_DEBUG=1` to print what the score is built from four times a second: where the mezzaluna is, how far
it is rocked and leans, where its edge's lowest point is and how far above the board, whether it is cutting, how many
leaves are chopped, whether it is in its rest, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (189 s on the clock, with Dream answering in 0.70 s on average), and about 2 minutes for the scripted demo (585 steps, 29.2 s of simulated time, 115 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
