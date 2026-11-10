# Hide a ball under the middle cup, watch the three cups being shuffled, then lift only the cup that hides it

Three identical copper cups stand mouth down on three white rings in a row on a green felt mat. A red ball lies in a
small porcelain dish at the front of the mat, and a start button with a lamp stands at its front left. One hand lifts
the middle cup and holds it up out to the side while the other takes the ball from the dish and sets it on the middle
ring; the cup goes back down over it and the start button is pressed. With both arms back where they started, the lamp
blinks amber for 5 s, then the cups glide around the mat and trade places five times, and the lamp turns green. One hand
then lifts the cup the ball is now under and holds it up while the other takes the ball out; the cup goes back on its
ring, the ball back into the dish, and both arms go home. The task is one task file,
[`find_the_ball_under_the_shuffled_cups.py`](find_the_ball_under_the_shuffled_cups.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: memory. Once the cups stop they look exactly alike, so the only way to know which cup to lift is to have
kept track of the ball's cup through the swaps; a policy that acts only on the frame in front of it is right one time
in three. Lifting a wrong cup first scores 0, so it cannot look under each cup in turn. The swaps are drawn from the
trial's seed, so they cannot be learned by heart: the cup the ball ends under is drawn evenly from the three, and the
ball's cup always takes part in at least two of the five swaps. It also tests doing the job in the right order (hide,
start, keep still, find) and a two-handed reveal.

Why two hands: a cup held up over its ring hides the ring from above, so the hand holding it cannot also reach the ball.
The instruction says to hold the cup up while the other hand puts the ball in or takes it out, and that is checked every
control step: the ball arriving on the middle ring while the middle cup is not held up, or coming out from under its
cup after the shuffle while that cup is not held up, scores 0. With one hand (the right doing its part exactly as in the demo, the left never moving) the demo scored 0: with the middle cup still down, the ball the right carried over came down on the cup's top and rolled off, so it was never hidden.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mat, the dish and the button console are
kinematic, fixed where they are put; the three cups and the ball are loose rigid bodies made of shapes from
`bodykit.py`.

- Each cup is a copper cone of 28 wall panels, 8 cm across at its flared mouth and 6 cm tall, with a straight foot band
  5.4 cm across at the top (where it is held) and brass beads; it is hollow, so the 1.9 cm ball fits under it. Each
  ring has a little ball seat in its middle (eight felt nubs 3 mm tall) that keeps a ball from rolling about.
- The start button is a red cap on a spring (a slider to its console, 8 mm of travel); pushed 4 mm down, it starts the
  game. The lamp then blinks amber for 5 s (the countdown), stays amber while the cups move and turns green when
  they stop.
- The shuffle is a rule (the mat's hidden shuffler): after the countdown the task file moves the three cups every
  control step along fixed arcs, five swaps of 1.4 s with 0.35 s between them; in each swap two neighbouring cups trade
  places, one passing 6 cm behind the row and the other 3.5 cm in front of it. The ball, if it is under a cup, is centred
  under it as the shuffle starts and moves with it. The swaps come from the trial's seed (`swaps_for(seed)` in the task file).
- A cup counts as lifted once its mouth is 1 cm above the mat or it tips over 15 degrees.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. The task file also wraps
the embodiment's reset to read the trial's seed.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the cups and the ball still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0;
see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=find_the_ball_under_the_shuffled_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Play the cups-and-ball game. First hide the red ball under the middle cup: lift the middle cup with one hand and hold it up while the other hand takes the ball from the dish and puts it on the middle ring, then lower the cup over it. Press the start button and keep both arms where they started while the cups are shuffled: the lamp blinks amber, then the cups glide around and swap places, and the lamp turns green when they stop. Then lift the cup the ball is under (the first cup you lift must be that one) and hold it up while the other hand takes the ball out. Set the cup back down on its ring, put the ball back in the dish and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/shell-dream
```

The scripted demo, which reaches full score. It is written for the trial that `inspect-robots run` starts with its
default `--seed 0` (reset seed 1415158438), whose five swaps take the ball's cup from the middle ring to the right ring,
back to the middle and then to the left ring; with another `--seed` the swaps differ and the demo, which follows fixed
waypoints, lifts the wrong cup. The right hand takes the ball from the dish and waits with it out to the right; the left takes the middle cup
by its foot band and lifts it straight up, high enough to pass over the left cup, and holds it up out past the mat's left
edge (the arms' elbows meet over the middle if both work there at once). The right sets the ball in the middle ring's
seat and goes home; the left lowers the cup over it, presses the start button and goes home. Both arms keep still
through the countdown and the shuffle. Then the left lifts the cup now on the left ring and holds it up out to the left,
the right takes the ball out from under where it stood, the left puts the cup back and goes home, and the right drops
the ball into the dish and goes home. Closing the fingers on a cup's foot band shoves the light cup about 1.9 cm toward
the robot, and opening them shoves it back as far (measured, the same every time); the cups' wide mouths keep the
shoved wall clear of the ball.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=find_the_ball_under_the_shuffled_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Play the cups-and-ball game. First hide the red ball under the middle cup: lift the middle cup with one hand and hold it up while the other hand takes the ball from the dish and puts it on the middle ring, then lower the cup over it. Press the start button and keep both arms where they started while the cups are shuffled: the lamp blinks amber, then the cups glide around and swap places, and the lamp turns green when they stop. Then lift the cup the ball is under (the first cup you lift must be that one) and hold it up while the other hand takes the ball out. Set the cup back down on its ring, put the ball back in the dish and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/shell-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/shell-dream/*.json; do inspect-robots video "$log"; done
```

Set `SHELL_DEBUG=1` to print what the score is built from four times a second: the trial's seed and swaps, the phase
(hide, countdown, shuffle, find), where the ball and each cup are, which cups are lifted, whether the ball was hidden
under the middle cup when the shuffle started, which cup it is under, which cup was lifted first after the shuffle,
whether it was taken out while its cup was held up, the finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,500 steps (367 s on the clock, with Dream answering in 0.89 s on average), and about 5 minutes for the scripted demo (1,230 steps, 61.5 s of simulated time, 300 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

The swaps depend on the trial's seed (`swaps_for(seed)`), so evaluating with several `--seed` values (or `--epochs`)
gives different shuffles; the layout itself is fixed with `-E object_jitter=0`. The scripted demo is written for the
default `--seed 0`.
