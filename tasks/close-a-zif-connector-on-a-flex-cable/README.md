# Hold a flex cable fully in its connector with one hand while the other flips the connector's latch shut

A circuit board lies on the table with a ZIF (zero insertion force) connector on its right part: a socket with a flip
latch standing open behind it. A long flat flex cable lies on the board with its end at the socket's mouth. One hand
slides the cable straight into the socket, fully in, and holds it there; the other tips the latch over so it falls shut
on the cable, locking it in. Then both hands let go. The task is one task file,
[`close_a_zif_connector_on_a_flex_cable.py`](close_a_zif_connector_on_a_flex_cable.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: fine insertion of a thin flat part to within a couple of millimetres, holding it there against a push,
and timing the other hand's action on a small mechanism.

Why two hands: the socket's contacts push the cable back out as soon as it is let go, and the latch locks the cable
only if it is fully in when the latch snaps shut. One hand holds the cable in while the other closes the latch. With
one hand (the left hand sliding the cable in and holding it as in the demo, then letting go, the right never moving)
the demo scored 0.25: the cable went fully in, then popped back out of the socket when let go, with the latch still
open.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The board and the socket are kinematic, fixed where
they are put; the latch and the cable are loose rigid bodies made of shapes from `bodykit.py`.

- The latch: a flap on a hinge (a `Hinge` from `bodykit.py`) across the back of the socket. It starts leaning just past
  upright against its stop; tipped over, it falls shut by its own weight. Within 12 degrees of shut, the task file
  gives the hinge a spring that holds it shut.
- The contacts' push: after every control step, while the cable's end is in the socket and the cable is not locked,
  the task file pushes it back out with 0.05 N plus 40 N/m times its depth.
- The lock: when the latch snaps shut while the cable is fully in and straight, the task file fixes the cable where it
  is (a `Glue` from `bodykit.py`, to a fixed block hidden under the table) and stops the push.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_a_zif_connector_on_a_flex_cable.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Slide the flex cable straight into the connector until it is fully in and hold it there with one hand; with the other tip the connector's open latch over so it falls shut on the cable; then let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zif-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable near its far end, fingers
closed, presses it onto the board, slides it straight into the socket until it stops, fully in, and holds it there. The
right hand comes down behind the open latch, fingers closed, and tips it over forward; it falls shut and snaps. Then
both hands rise:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_a_zif_connector_on_a_flex_cable.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Slide the flex cable straight into the connector until it is fully in and hold it there with one hand; with the other tip the connector's open latch over so it falls shut on the cable; then let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zif-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/zif-dream/*.json; do inspect-robots video "$log"; done
```

Set `ZIF_DEBUG=1` to print what the score is built from twice a second: how deep the cable's end is in the socket and
how straight the cable lies, where the cable is, the latch's angle up from shut, whether the cable is fully in (now and
ever), latched and locked, whether both hands are clear, the contacts' push, and where each TCP is from the cable's end
and the latch's hinge.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (139 s on the clock, with Dream answering in 0.70 s on average), and under 1 minute for the scripted demo (238 steps, 11.9 s of simulated time, 39 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
