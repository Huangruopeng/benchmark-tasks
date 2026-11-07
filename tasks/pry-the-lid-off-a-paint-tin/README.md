# Pry the lid off a paint tin: hold the light tin down by its bail while the other hand hooks up the lid's three tabs

A small tin of paint stands on the table, its wire bail folded flat to the left; its steel lid is held on by three white
tabs on its rim, each a hook whose lip reaches in under the rolled bead round the tin's rim, and a steel lid lifter (a
claw on a short handle) stands in a holder to the right. One hand holds the tin down by its bail's grip while the other
takes the lifter and pries up each tab in turn, hooking the claw under the tab's lip and lifting until the tab turns up
and its lip swings out from under the bead. Once all three are up the lid is loose: the bail is let go, the lifter goes
back on its holder, the lid is lifted off by its knob and set down beside the tin, showing the paint, and both arms go
back to where they started. The task is one task file,
[`pry_the_lid_off_a_paint_tin.py`](pry_the_lid_off_a_paint_tin.py), for the `yam_arena` embodiment (scene, scoring and
the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding an object down against the other hand's pull; working a hook tool under an edge, at three places
in turn; finishing the job (tool put away, lid off, both hands free).

Why two hands: each tab takes about 10 N at its lip before it turns, and lifted at its rim the light 400 g tin tips up
at about 2 N, so hooked and pulled up with nothing holding the tin down, the tab tips the whole tin up instead of
turning. With one hand (the right prying as in the demo, the left never moving) the demo scored 0: the tin tipped over,
46 degrees.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The lid: a rigid disc resting on the tin's rim, its plug in the mouth, held down only by the three tabs: there is no
  joint between the lid and the tin, so the lid lifts off once the tabs are up and cannot be pulled off before.
- Each tab: a rigid hook on a hinge along the rim (a `Hinge` from `bodykit.py`; the task file turns the joint's frame to
  run along the rim), its lip under the tin's bead. A spring holds the tab where it last stopped; after every control
  step, a tab turned more than 2 degrees past its hold (about 10 N at its lip) has given, and the task file moves the
  hold there, so it stays up. Turned up 18 degrees, its lip has swung out from under the bead: pried. Pulling the lid up
  only presses the lips against the bead.
- The tin and its folded bail are one rigid body; the paint is a coloured disc inside it.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the tin, the lid and the lid lifter still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pry_the_lid_off_a_paint_tin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the paint tin: hold the tin down by its bail's black grip with one hand; with the other take the lid lifter out of its holder and pry up each of the three white tabs on the lid's rim, hooking the lifter's claw under the tab's lip and lifting until the tab turns up. Then let go of the bail, put the lifter back on its holder, lift the lid off by its knob and set it on the table beside the tin, and take both arms back to where they started. Don't let the tin tip over." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tin-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the bail's grip, fingers closing
across it, and holds the tin down. The right hand takes the lid lifter out of its holder by its handle and, for each tab
in turn, lowers its claw just outside the tab, below the rim, slides the claw in under the tab's lip and lifts: the tab
turns up. Once all three tabs are up the left lets go of the bail and goes home, and the right puts the lifter back on
its holder, lifts the lid off by its knob, sets it down on the table beside the tin and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pry_the_lid_off_a_paint_tin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the paint tin: hold the tin down by its bail's black grip with one hand; with the other take the lid lifter out of its holder and pry up each of the three white tabs on the lid's rim, hooking the lifter's claw under the tab's lip and lifting until the tab turns up. Then let go of the bail, put the lifter back on its holder, lift the lid off by its knob and set it on the table beside the tin, and take both arms back to where they started. Don't let the tin tip over." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tin-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tin-dream/*.json; do inspect-robots video "$log"; done
```

Set `TIN_DEBUG=1` to print what the score is built from twice a second: how far each tab is turned up and where its hold
is, whether all three are up, where the tin is and how far it is tilted, where the lid and the lifter are, whether the
lid is off and the lifter put down, whether both hands are clear, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (214 s on the clock, with Dream answering in 0.67 s on average), and about 3 minutes for the scripted demo (724 steps, 36.2 s of simulated time, 174 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
