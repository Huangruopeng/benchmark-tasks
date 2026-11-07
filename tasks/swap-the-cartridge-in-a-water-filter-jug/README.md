# Change a water filter jug's cartridge: hold the jug down by its handle while the other hand pulls the old one out

An empty water filter jug stands on the table, lid off, with an old grey filter cartridge wedged in the seat of its top
reservoir. One hand holds the jug down by its handle while the other pulls the old cartridge up out of its seat and
drops it in the waste bin; then it takes the new cartridge, lowers it into the seat and pushes it down until it is
seated, and both arms go back to where they started. The task is one task file,
[`swap_the_cartridge_in_a_water_filter_jug.py`](swap_the_cartridge_in_a_water_filter_jug.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a light container down against a pull, a firm straight pull, putting the used part away, and
fitting the new one in line with its seat.

Why two hands: the cartridge's seal grips harder (about 2.5 N) than the 200 g jug weighs, so pulling the cartridge
lifts the whole jug unless the jug is held down. With one hand (the right doing its part of the demo, the left never
moving) the demo scored 0.0: the jug came up with the cartridge still in it, swung against the bin as it was carried
over, and fell back to the table with the old cartridge still seated.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The bin is kinematic, fixed where it is put; the
jug and both cartridges are loose rigid bodies made of shapes from `bodykit.py`.

- The seat: a cartridge in it is held on the seat's axis by a sliding joint (a `Thread` from `bodykit.py`) whose
  spring holds it where it is. After every control step, a cartridge moved more than 1 mm from its hold has slipped,
  and the task file moves the hold there: about 2.5 N moves it, and it stays where it is left. Pulled 2 cm up, it is
  out of the seat and free.
- A new cartridge lowered to within 2 cm above the seat, within 8 mm of its axis and 12 degrees of upright, is caught
  by the same kind of joint and can then be pushed down; within 4 mm of fully down it counts as seated.
- The seat is in the middle of the jug, over its centre of mass, and the handle 15 cm out: a gripper holding the
  handle keeps the jug down, while the jug itself would come up with the cartridge.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the jug and both cartridges still) has held for
1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_the_cartridge_in_a_water_filter_jug.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Change the filter cartridge in the water jug: hold the jug down by its handle with one hand while the other pulls the old grey cartridge up out of its seat, and drop the old cartridge in the bin. Then put the new white cartridge with the blue cap into the seat and push it down until it is seated. Let go of everything and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/jug-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the jug's handle, fingers closing
across its top bar, and holds the jug down. The right comes down on the old cartridge's cap, fingers closing across it,
pulls the cartridge up out of its seat and clear of the jug, carries it over the bin and drops it in. It takes the new
cartridge by its cap, carries it over the seat, lowers it in and pushes it down until it is seated, lets go and goes
back to where it started; the left lets go of the handle and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swap_the_cartridge_in_a_water_filter_jug.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Change the filter cartridge in the water jug: hold the jug down by its handle with one hand while the other pulls the old grey cartridge up out of its seat, and drop the old cartridge in the bin. Then put the new white cartridge with the blue cap into the seat and push it down until it is seated. Let go of everything and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/jug-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/jug-dream/*.json; do inspect-robots video "$log"; done
```

Set `JUG_DEBUG=1` to print what the score is built from twice a second: how far the old cartridge is up from seated
and whether it is out (and in the bin), where the new one is against the seat (up, off the axis, tilt) and whether it is
caught and seated, where the jug is, whether both hands are clear and both arms home.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (142 s on the clock, with Dream answering in 0.70 s on average), and about 1 minute for the scripted demo (532 steps, 26.6 s of simulated time, 79 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
