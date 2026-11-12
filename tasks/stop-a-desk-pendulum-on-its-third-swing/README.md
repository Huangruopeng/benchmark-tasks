# Start a desk pendulum from its start mark, holding its stand still, and press STOP on its third swing

A desk pendulum stands on a smooth table: a walnut base with a brass knob on each end, a post at the back and an arm
reaching forward over the base, from whose end a brass pendulum hangs, free to swing left and right. Its bob is a brass
cylinder with a long knurled peg sticking out of its front, and under the peg a brass scale is marked every
5 degrees, with a red start mark 20 degrees out to the right. A STOP button (a red mushroom on a yellow box) on the
base's front right corner works the pendulum's brake. One hand holds the stand still by the knob on its left end; the other
pushes the bob by its peg out to the red start mark and lets go. Then it counts: as the bob comes back to the right for
the third time, it presses STOP, and the brake stops the pendulum hanging straight down. Both hands let go and go back
to where they started. The task is one task file,
[`stop_a_desk_pendulum_on_its_third_swing.py`](stop_a_desk_pendulum_on_its_third_swing.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: memory. The policy has to count a repeating event that looks the same every time and act on the right
repetition. Every swing looks alike (the swing dies away by only about 1 percent a swing), nothing in view records how
many have gone by, and the right moment to press is one swing long, so a policy that only looks at the current frame
can do no better than press at a guess. It also tests starting the swing cleanly (out to the mark, then out of the way)
and not touching the bob while it swings.

Why two hands: the stand is light and the table smooth. Pushed out by its bob, an unheld stand slides away from the
hand long before the bob reaches the start mark (it starts to slide about 10 degrees out). With the right hand only
(doing its part of the demo, the left never moving) the demo scored 0: the stand slid 3.9 cm along with the bob, which
never got more than 9.4 degrees out from it, so the swing was never started from the mark. The left hand holds the
stand by a knob in line with the pivot, so the bob's push and swing cannot turn the stand about the hand.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand and the pendulum are loose rigid bodies
made of shapes from `bodykit.py`.

- The pendulum turns on the stand about the pivot's axis on a `Hinge` from `bodykit.py`, with next to no friction.
- The stand's feet are slick (friction 0.07), so it slides unless held.
- After every control step the task file measures the pendulum's angle against the stand, notes when the bob has been
  pushed out to the start mark and let go, counts each turn of the swing at its far end on the left, and watches the
  STOP button's cap for a fingertip. At the first press it turns the hinge's damping up (the brake), which stops the
  pendulum hanging straight down within about a second, and lights the brake lamp.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the stand and the pendulum still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75
or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=stop_a_desk_pendulum_on_its_third_swing.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Start the desk pendulum and stop it on its third swing. Hold the stand still by the brass knob on its left end the whole time, so it does not slide on the smooth worktop. With the other hand push the bob by its peg out to the red start mark on the scale, 20 degrees to the right, and let it go. Then count the swings: when the bob comes back to the right for the third time, press the red STOP button once, and its brake stops the pendulum hanging straight down. Don't touch the bob while it swings. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/pendulum-dream
```

The scripted demo, which reaches full score: the left hand comes down on the knob on the stand's left end and holds
it. The right comes down in front of the pendulum, leaning back so that it stays clear of the rod, its closed fingers
beside the bob's peg; it pushes the bob out along its arc to the red start mark, holds it there a moment and lifts
straight up off the peg: the pendulum swings.
The right goes over the STOP button and waits. Its waypoints are timed so that it presses STOP three swings after it let
go, as the bob comes back to the right for the third time; the brake stops the pendulum. Both hands let go and go home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=stop_a_desk_pendulum_on_its_third_swing.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Start the desk pendulum and stop it on its third swing. Hold the stand still by the brass knob on its left end the whole time, so it does not slide on the smooth worktop. With the other hand push the bob by its peg out to the red start mark on the scale, 20 degrees to the right, and let it go. Then count the swings: when the bob comes back to the right for the third time, press the red STOP button once, and its brake stops the pendulum hanging straight down. Don't touch the bob while it swings. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/pendulum-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pendulum-dream/*.json; do inspect-robots video "$log"; done
```

Set `PENDULUM_DEBUG=1` to print what the score is built from ten times a second: the pendulum's angle, whether the bob
has been pushed out to the mark and let go, the swings counted, whether a hand is at the bob, whether STOP has been
pressed and on which swing, how far the stand has moved, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,000 steps (339 s on the clock, with Dream answering in 0.67 s on average), and about 2 minutes for the scripted demo (389 steps, 19.4 s of simulated time, 142 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, the count to reach (three) is in the instruction, and the
scripted demo follows fixed waypoints.
