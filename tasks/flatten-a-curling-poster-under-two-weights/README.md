# Flatten a curling poster: hold each curled end flat with one hand while the other sets a brass weight on it

A paper poster that has been rolled up lies on the table, its middle flat and both ends curled up, each end wrapped
round a wooden hanging rail; two 500 g brass weights stand beside it. While one hand presses the poster's middle down,
the other draws the near rail out until the near end lies flat and holds it there; the first hand sets a weight on that
end. Then the far end is drawn out flat and held by one hand while the other sets the second weight on it. Both hands
let go and go back to where they started. The task is one task file,
[`flatten_a_curling_poster_under_two_weights.py`](flatten_a_curling_poster_under_two_weights.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: unrolling a springy sheet along its natural path by one edge; holding it flat while the other hand
places something on it; doing that at both ends in turn, with the hands swapping roles.

Why two hands: a curled end springs back up as soon as nothing holds it, so the hand that drew it flat cannot let go
to fetch a weight; and drawing the first end out drags the whole poster along unless the other hand holds its middle.
With one hand (the right doing its part as in the demo, the left never moving) the demo scored 0: the right drew the
near end out flat (dragging the poster 2 cm), but as soon as it let go to move on, the near end curled straight back up
with nothing on it; the far weight, set down on the still-curled far end, pressed it only partly flat (36 degrees).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The poster: a flat middle panel and, at each end, three rigid panels (3, 3 and 5.5 cm) joined by hinges (`Hinge`
  from `bodykit.py`) about lines across the poster. Each hinge's spring (0.005, 0.003 and 0.0016 N m per degree, from
  the middle out) pulls its fold up 35 degrees, against a stop, so each end curls up 105 degrees in all toward the
  middle. Laid flat, an end pushes back with 0.07-0.23 N m at its folds (damped, so a let-go end curls back up in about a second); a 500 g weight on its outer panel holds it
  down.
- The rails: a round beech rail at each end, joined to the outer panel by a free hinge along the rail, as a rail turns
  in the poster's hem, so a hand can hold the rail while the paper turns round it.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the poster and both weights still) has held for
1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flatten_a_curling_poster_under_two_weights.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Flatten the poster: draw each curled end out by its rail until it lies flat, and hold it there while the other hand sets a brass weight on it, one weight on each end. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/poster-dream
```

The scripted demo, which reaches full score: the left hand comes down between the curled ends and presses the poster's middle down, fingers closed. The right takes the near rail where it lies on top of its curl and draws it toward the arms along the unfolding path until the near end lies flat, and holds it down. The left rises straight up, takes the near weight by its knob and sets it on the near end's outer panel; the right lets go and moves aside. The left takes the far rail and draws it out flat the same way and holds it; the right takes the far weight and sets it on the far end. The left lets go, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flatten_a_curling_poster_under_two_weights.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Flatten the poster: draw each curled end out by its rail until it lies flat, and hold it there while the other hand sets a brass weight on it, one weight on each end. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/poster-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/poster-dream/*.json; do inspect-robots video "$log"; done
```

Set `POSTER_DEBUG=1` to print what the score is built from twice a second: for each end, the largest tilt of its
panels off the table, its rail's height, whether it counts as flat and which weight is on it; where the poster's middle
is, whether both arms are back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (151 s on the clock, with Dream answering in 0.76 s on average), and about 2 minutes for the scripted demo (889 steps, 44.5 s of simulated time, 123 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
