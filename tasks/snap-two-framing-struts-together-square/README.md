# Snap two aluminium framing struts into a square corner on a push-in connector, and prove the joint by lifting the frame

Two lengths of 20 mm square aluminium framing strut lie on the table: strut A runs away from the arms, a die-cast corner
connector on its near end with its square peg pointing right and a spring catch on the peg; strut B lies to the right,
square to A, its hollow end facing the peg between the two strips of a framing jig. One hand holds strut A still while
the other pushes strut B's end straight along the jig onto the peg until the catch snaps up into the hole in B's wall.
Then strut A is let go and strut B is lifted, so the whole corner rises in one piece, showing the joint holds; the frame
is set down flat clear of the jig, let go, and both arms go back to where they started. The task is one task file,
[`snap_two_framing_struts_together_square.py`](snap_two_framing_struts_together_square.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a light part still against a push; pushing a part straight onto a peg past a spring catch;
checking the joint by lifting the assembly by the other part.

Why two hands: the catch has to be pressed down as strut B slides on, which takes a firm push, and strut A is light and
slides easily on the table. With one hand (the right pushing strut B as in the demo, the left never moving) the demo
scored 0: strut A slid 3 cm away and turned 10 degrees ahead of the push, and strut B's end never got onto the peg.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- Strut A and its connector: one rigid body. The catch is a small block on a prismatic joint in the peg's top (a
  `Slider` from `bodykit.py`), sprung up 3 mm above the peg (1.5 N per mm, 4.5 mm travel, its tip-side face bevelled 45 degrees); strut B's wall presses it down as B slides on, and it
  springs up into B's hole once B is fully on, so B can no longer come off.
- Strut B: one rigid body with a 14 mm square socket 4.6 cm deep in its end (the peg is 11 mm) and an 8 mm slot across
  the socket's top wall, 11 to 19 mm in from its end.
- Fully on: strut B's end within 3 mm of strut A's side with the catch up. Square: the two struts' axes 90 degrees
  apart, within 3 degrees.
- The jig: two fixed steel strips either side of strut B's end, 1.5 mm clear of it, that keep B's end in line with the
  peg as it is pushed on.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with both struts still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E
cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=snap_two_framing_struts_together_square.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Join strut B's hollow end onto the corner connector on strut A so that its catch locks and the corner is square, prove the joint by lifting the frame by strut B alone, then set the frame down flat on the table clear of the jig, let go, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/strut-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on strut A near its far end, fingers
closing across it, and holds it. The right hand takes strut B 11.5 cm from its open end, fingers closing across it, and
slides it straight along the jig onto the peg until the catch snaps up into its hole. The left lets go of strut A and
goes home; the right lifts strut B 7 cm, so the corner rises with strut A hanging on the joint, holds it there, moves it
4.5 cm clear of the jig, sets it down, lets go and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=snap_two_framing_struts_together_square.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Join strut B's hollow end onto the corner connector on strut A so that its catch locks and the corner is square, prove the joint by lifting the frame by strut B alone, then set the frame down flat on the table clear of the jig, let go, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/strut-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strut-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRUT_DEBUG=1` to print what the score is built from twice a second: how far strut B's end still is from strut A's
side, where the catch is, whether B is fully on and has snapped on, the angle between the struts, how high strut A's
near end is, whether the corner has been lifted by strut B alone, whether the frame lies flat, how far each TCP is from
each strut, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 900 steps (233 s on the clock, with Dream answering in 0.63 s on average), and about 2 minutes for the scripted demo (397 steps, 19.9 s of simulated time, 120 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
