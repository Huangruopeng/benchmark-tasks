# Zip up a toddler's jacket lying on the table: hold it by its hem, push the zip's pin into the box, pull the zip up

A toddler's red fleece jacket lies open on the table, front up, its hem toward the arms. It closes with a separating
zip: the slider waits on the box at the bottom of the left front, and the pin at the bottom of the right front's tape
sticks out below the hem by its black moulded tab. The hem's drawcord ends in a cord lock standing on the table at the
jacket's left side. One hand holds the jacket still by that cord lock; the other pushes the pin up into place beside
the box, then takes the slider's pull and pulls the zip all the way up to the collar, closing it behind the slider.
Then both hands let go and both arms go back to where they started. The task is one task file,
[`zip_up_a_toddlers_jacket_holding_its_hem_down.py`](zip_up_a_toddlers_jacket_holding_its_hem_down.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a light, loose thing still with one hand while the other works small parts on it, in the order
a zip needs (pin first, then the slider), pushing and pulling far enough against resistance, and letting go cleanly.

Why two hands: the jacket weighs 250 g and slides on the table at about 1.2 N, while the pin takes about 2.5 N to push
home and the slider about 2.6 N to pull: pushed or pulled with nothing holding the jacket, the whole jacket slides
along instead. With one hand (the right doing its part of the demo, the left never
moving) the demo scored 0: pushing the pin's tab slid the whole jacket 20 cm up the table, the pin never went into the
box and the zip never ran. The left alone (holding the cord lock) also scored 0.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The jacket is one loose rigid body lying flat (its
body, collar, sleeves, pockets, zip tapes with their teeth, the box at the bottom of the zip and the cord lock standing
by the hem), and the slider and the pin are loose bodies made of shapes from `bodykit.py`, each joined to the jacket by
a straight slide along the zip (a `Slider` from `bodykit.py`).

- Resistance: after every control step the task file holds each of them where it was left, with a spring whose target
  moves only once the part is pushed more than 1.5 mm past it: about 2.5 N to push the pin and 2.6 N to pull the
  slider, the same whichever speed. Until the pin is in, the slider is held where it is (a zip won't run without it).
- The pin: pushed 1.7 cm up (of 2.2 cm), it is in: the task file pulls it the rest of the way and holds it there.
- The zip: the slider's travel up the jacket, from the box to its top stop under the collar (16.7 cm); the closed zip
  (a dark band with a chain of locked teeth) is drawn behind the slider as it goes, in nine lengths that follow the
  jacket. Zipped all the way up: within 1.5 cm of the top stop.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the jacket and the slider still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0;
see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=zip_up_a_toddlers_jacket_holding_its_hem_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Zip up the red jacket: hold the jacket still by the drawcord toggle at its hem with one hand. With the other, push the zip's pin, by its black tab, up into the box at the bottom of the zip, then take the slider's pull and pull the zip all the way up to the collar. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zip-dream
```

The scripted demo, which reaches full score: the left hand comes down on the cord lock at the left end of the hem,
leaning in a little, fingers closing across it, and holds the jacket still by it. The right hand comes straight down on
the pin's tab, fingers closing across it, and pushes the pin up into place beside the box. It lets go, comes down on the
slider's pull, takes it and pulls the zip up to its top stop under the collar. It lets go and goes back; the left lets
go of the cord lock and goes back too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=zip_up_a_toddlers_jacket_holding_its_hem_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Zip up the red jacket: hold the jacket still by the drawcord toggle at its hem with one hand. With the other, push the zip's pin, by its black tab, up into the box at the bottom of the zip, then take the slider's pull and pull the zip all the way up to the collar. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/zip-dream/*.json; do inspect-robots video "$log"; done
```

Set `ZIP_DEBUG=1` to print what the score is built from four times a second: how far the jacket has moved and turned,
how far the pin has been pushed up and whether it is in, how far the zip has been pulled (and what share of the way up),
how many lengths of it are drawn closed, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (402 s on the clock, with Dream answering in 0.61 s on average), and about 3 minutes for the scripted demo (448 steps, 22.4 s of simulated time, 170 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
