# Grease a loader pivot: hold the push-on coupler on the nipple with one hand while the other pumps the pail's lever

On the table stands the pivot of a loader arm: a yellow steel lug with a steel pin through it and a grease nipple on
its top. To the left stands a grease pail with a lever pump on its lid; its black hose runs over the table to a
push-on coupler standing in a steel holder cup. One hand takes the coupler, presses it onto the nipple and holds it
there, while the other hand pumps the pail's lever down at least four full strokes, and grease squeezes out at the
pin. Then the coupler goes back in its holder and both arms go back to where they started. The task is one task file,
[`grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py`](grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py), for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py),
which it imports from the same folder.

What it tests: holding a part pressed on against a push while the other hand does repeated full strokes; timing the
two hands (pump only while the coupler is on); a precise placement onto a small fitting and back into a holder.

Why two hands: a push-on coupler is pushed off the nipple by the grease pressure as soon as nothing holds it on, and
the pump's lever is 29 cm away from the nipple. With one hand (the right taking the coupler and holding it on the
nipple as in the demo, the left never moving) the demo scored 0.25: the coupler sat on the nipple, but nothing was
pumped. Seated and let go, the coupler was pushed off and fell over on the table within a second.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The pivot, the pail with its pump and the holder
cup are kinematic, fixed where they are put; the lever, the five hose segments and the coupler are loose rigid bodies
made of shapes from `bodykit.py`.

- The lever turns on a sprung hinge (a `Hinge` from `bodykit.py`) about a side-to-side axis on the pump head; its
  spring holds it 15 degrees up, and it presses down 26 degrees at most.
- The hose is five stiff 12 mm segments joined by ball joints (`Ball`), from the pail's outlet to the coupler's elbow.
- The coupler's square nozzle (drawn round) has a 26 mm mouth that fits over the nipple's hex and ball head and sits
  on the boss under it.
- Grease pressure: after every control step the task file checks the coupler against the nipple. While its mouth is
  round the nipple (within 13 mm of its axis, at most 14 mm above the boss), it pushes the coupler up off the nipple
  with 2.5 N (and 0.2 N sideways), so a coupler stays on only while held on.
- A shot: the lever pressed down past 18 degrees while the coupler is seated; it has to come back to within 10
  degrees of its rest before the next press counts. For each of the first four shots a drawn dab of grease (kept
  under the table until then) is moved to the gap between the lug and the arm's side plates.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the coupler still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader pivot: take the grease coupler out of its holder cup and press it onto the grease nipple on top of the yellow pivot, holding it on (it is pushed off unless held) while your other hand pumps the grease pail's red-gripped lever down at least four full strokes; then pull the coupler off, put it back in its holder cup, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-dream
```

The scripted demo, which reaches full score: the right hand comes down round the coupler's grip sleeve in its holder,
takes it, lifts it out, carries it over the nipple and presses it down onto it, and holds it there. The left hand
comes down on the lever's grip pad, fingers closed, and pumps the lever down five full strokes, following its swing,
then rises and goes back to where it started. The right pulls the coupler off, carries it back over the holder,
lowers it into the cup, lets go and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader pivot: take the grease coupler out of its holder cup and press it onto the grease nipple on top of the yellow pivot, holding it on (it is pushed off unless held) while your other hand pumps the grease pail's red-gripped lever down at least four full strokes; then pull the coupler off, put it back in its holder cup, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grease-dream/*.json; do inspect-robots video "$log"; done
```

Set `GREASE_DEBUG=1` to print what the score is built from twice a second: how far the lever is pressed down, the
strokes and the shots so far, where the coupler is, how far its mouth is from the nipple's axis and above the boss,
its tilt, whether it is seated, pushed or in its holder, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,200 steps (306 s on the clock, with Dream answering in 0.69 s on average), and about 3 minutes for the scripted demo (643 steps, 32.1 s of simulated time, 207 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
