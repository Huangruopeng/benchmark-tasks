# Change an electric toothbrush's head: hold the handle, pull the worn head off and push a new one on

An electric toothbrush lies on its back on a dark grey granite bathroom counter, its worn round brush head still on its
steel shaft. A new head lies in its open blister pack to the right, and a small pedal bin stands near the front right.
One hand takes the toothbrush's handle and holds it. The other takes the worn head by its neck, pulls it straight off the
shaft and drops it in the bin; it then takes the new head out of its pack, lines it up in front of the shaft and pushes
it straight on, all the way. Both hands let go and both arms go back to where they started. The task is one task file,
[`change_the_brush_head_on_an_electric_toothbrush.py`](change_the_brush_head_on_an_electric_toothbrush.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding an object still against a pull and then a push; taking a part off and putting its replacement on
in the right order; lining a part up with a shaft.

Why two hands: a brush head sits on its shaft with a press fit that takes about 3 N to pull off or push on, and the
toothbrush lying on the counter slides at about 0.6 N, so pulled or pushed with nothing holding the handle,
the whole toothbrush slides along with the head. With one hand (the right doing its part as in the demo, the left
never moving) the demo scored 0: the pull dragged the toothbrush 4.7 cm across the counter with the worn head still
on its shaft, and the hand then carried the whole toothbrush off by the head.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The counter, the blister pack and the bin are
kinematic, fixed where they are put; the toothbrush and the two heads are loose rigid bodies made of shapes from
`bodykit.py`.

- The press fit: each head is held on the shaft by a `Thread` from `bodykit.py` (a joint that keeps the head on the
  shaft's axis and lets it slide along it), pulled by a stiff spring toward a hold point. After every control step the
  task file moves the hold point to wherever the head has been pushed or pulled more than 1 mm from it, so a head slides
  only against about 3 N and stays where it is left (stick-slip). The worn head starts on the shaft; once pulled out past
  the shaft's end (2.8 cm), its joint switches off and it is free. A new head whose collar is pushed onto the shaft's
  end (within 1 cm of its axis and 15 degrees of its line) catches the same way. The heads do not collide with the
  handle, so a head's collar can slide over the shaft.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the toothbrush and the new head still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=change_the_brush_head_on_an_electric_toothbrush.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Change the electric toothbrush's brush head: hold the toothbrush's handle with one hand. With the other, pull the worn brush head straight off its metal shaft and drop it in the bin, then take the new brush head from its pack and push it straight onto the shaft, all the way on. Then let go of both and take both arms back to where they started." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/brush-dream
```

The scripted demo, which reaches full score: the left hand takes the toothbrush's handle near its middle, its grip pitched back so its wrist stays clear of the right's, and holds it. The right, also pitched (fingers ahead of the wrist, to reach the shaft's end), takes the worn head by its neck, pulls it straight off the shaft (aiming 2 cm past, as the arm gives under the press fit), lifts it and drops it in the bin. It takes the new head out of its pack by the neck, carries it in front of the shaft, lines it up and pushes it straight on, all the way. Both hands let go and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=change_the_brush_head_on_an_electric_toothbrush.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Change the electric toothbrush's brush head: hold the toothbrush's handle with one hand. With the other, pull the worn brush head straight off its metal shaft and drop it in the bin, then take the new brush head from its pack and push it straight onto the shaft, all the way on. Then let go of both and take both arms back to where they started." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/brush-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/brush-dream/*.json; do inspect-robots video "$log"; done
```

Set `BRUSH_DEBUG=1` to print what the score is built from four times a second: where the toothbrush and both heads
are, how far each head is along the shaft and whether its joint is on, how far the new head is from the shaft's axis
and at what angle, whether the worn head is in the bin, the finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 800 steps (130 s on the clock, with Dream answering in 0.74 s on average), and about 1 minute for the scripted demo (525 steps, 26.2 s of simulated time, 89 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
