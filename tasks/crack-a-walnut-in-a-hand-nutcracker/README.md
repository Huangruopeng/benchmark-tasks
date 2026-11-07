# Crack a walnut: push a hand nutcracker's two handles together from both sides, then bowl the kernel and plate the shell

A steel hand nutcracker lies flat on a smooth worktop, its two long levers open in a V toward the arms, with a walnut
between its toothed jaws near the pivot; a white bowl waits on the right and a blue saucer on the left. The two handles
are pushed toward each other, one hand on each, until the jaws crack the shell. Then the kernel goes into the bowl, both
shell halves go onto the saucer, and both arms go back to where they started. The task is one task file,
[`crack_a_walnut_in_a_hand_nutcracker.py`](crack_a_walnut_in_a_hand_nutcracker.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: two opposing pushes on one free tool, kept in line so the tool closes instead of sliding away; then
sorting small pieces into two containers.

Why two hands: the nutcracker's spring holds its jaws open with 0.3-0.8 N m, which takes about 3.5 N on each handle to
overcome, and the 320 g tool slides on the worktop at about 1 N. Pushed on one handle only, it just slides across the
worktop with the walnut. With one hand (the right pushing its handle as in the demo, the left never moving) the demo
scored 0: the right hand pushed its handle in 3.5 cm, but the whole nutcracker turned on the worktop with the walnut, the levers never closed at all, and nothing cracked.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The nutcracker: two loose steel levers joined at the pivot by a hinge (a `Hinge` from `bodykit.py`) with a spring
  (0.04 N m per degree) that holds them open at 30 degrees each side of the middle line, against a stop; they close to
  20 degrees at most. Each lever runs straight out 20 cm from the pivot and then bends into a 9 cm red handle that runs
  along the middle line, so the two handles are pushed toward each other in line.
- The walnut: 3.5 cm across as seen, but the jaws touch a 2.4 cm core inside it, so they can crush the shell in. After
  every control step the task file works out the gap between the jaws where the walnut sits; once it is 2.9 cm or
  less, the walnut cracks: the task file moves it to a closed box out of sight above the scene and moves a kernel and
  two shell halves, waiting in that box, onto the worktop along the V, past the jaws.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the kernel, the shell halves, the bowl and the saucer
still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crack_a_walnut_in_a_hand_nutcracker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the walnut with the nutcracker: push its two handles toward each other, one hand on each, until the jaws crack the shell. Then put the kernel in the white bowl and both shell halves on the blue saucer, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/nut-dream
```

The scripted demo, which reaches full score: each hand comes down beside the outside of one handle, fingers closed,
and the two push the handles toward each other until the jaws crack the walnut. They draw back and the spring opens the
levers. The right hand takes the kernel to the bowl and goes back to where it started; then the left takes each shell
half to the saucer and goes back to where it started too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crack_a_walnut_in_a_hand_nutcracker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the walnut with the nutcracker: push its two handles toward each other, one hand on each, until the jaws crack the shell. Then put the kernel in the white bowl and both shell halves on the blue saucer, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/nut-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/nut-dream/*.json; do inspect-robots video "$log"; done
```

Set `NUT_DEBUG=1` to print what the score is built from twice a second: how far each lever is from the V's middle
line, the gap between the jaws where the walnut sits and where it sits along and across the V, whether it has cracked,
where the pivot and the kernel are, whether the kernel is in the bowl and how many shell halves are on the saucer,
whether both arms are back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,000 steps (190 s on the clock, with Dream answering in 0.88 s on average), and about 3 minutes for the scripted demo (694 steps, 34.7 s of simulated time, 151 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
