# Put a flute together: push the head joint into the body, turn it to line up the embouchure hole with the keys

An open flute case lies on the table: a black case lined with blue velvet, its lid open flat behind it. In it lie the
two parts of a silver flute: in the front row the long body with its keys, and in the back row the head joint, turned
in its slot so that its lip plate faces up and away from the arms. The left hand takes the body by its middle and the
right hand the head joint, both from above, and they lift them out of the case. Holding them up in front of the case,
the right hand pushes the head joint's tenon all the way into the socket at the end of the body, then turns the head
joint an eighth of a turn, until the embouchure hole in its lip plate sits on top, in line with the keys. The right
hand lets go and goes back; the left lays the assembled flute in the three velvet cradles on the open lid, keys up,
lets go and goes back too. The task is one task file,
[`assemble_a_flute_and_line_up_its_embouchure_hole_with_the_keys.py`](assemble_a_flute_and_line_up_its_embouchure_hole_with_the_keys.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: an assembly done in the air with both hands (holding one part still while the other hand pushes a
second part into it along its axis), a rotational alignment judged by eye (a hole lined up with a row of keys), and
putting the result down the right way up.

Why two hands: the body must be held up and still while the head joint is pushed into it and turned: lying in the
case it is pushed away along its slot, and the friction of the fit turns it round with the head joint. With one hand (the
right doing its part of the demo, the left never moving) the demo scored 0: the body stayed in its slot in the case
and the head joint, carried to where the body's socket would have been, was never caught. The left alone also scored
0: it held the body up in front of the case, but the head joint never left its slot.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The case is kinematic, fixed where it is put (in its
slots the parts lie on velvet saddles with a shallow groove; the open lid has a rim and three velvet cradles); the body
and the head joint are loose rigid bodies made of shapes from `bodykit.py`, round silver tubes. The keys, the lip plate
and the embouchure hole are drawn only.

- The fit: the two parts never collide. After every control step the task file finds where the head joint's tenon tip
  is against the body's socket. Once the tip is in the socket's mouth (within 8 mm of its axis, at most 1.2 cm in, the
  two axes within 12 degrees), it switches on a `Thread` joint from `bodykit.py` that keeps the head joint on the
  body's axis. It slides in against about 2 N, up to 2.5 cm, and turns against up to 0.06 N m (plus 0.05 N m per rad/s):
  both are spring drives whose targets the task file moves to wherever the head joint is pushed or turned once it has
  moved more than 1 mm or 2 degrees past them, so the fit stays where it is left (stick-slip). Pulled more than 4 mm
  back out of the mouth, the joint is switched off and the parts are apart.
- All the way in: the tenon pushed at least 2.2 cm in. Lined up: the head joint's lip plate turned within 16 degrees
  of the keys' line about the flute's axis.
- On the lid: the middles of both parts over the open lid, no more than 3 cm above its velvet, with the keys within
  45 degrees of facing straight up.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
both parts still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=assemble_a_flute_and_line_up_its_embouchure_hole_with_the_keys.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the flute together: take the body out of the case with one hand and the head joint with the other, push the head joint's tenon all the way into the socket at the end of the body, and turn the head joint until the hole in its lip plate lines up with the keys. Then lay the assembled flute on the open lid of the case, keys up, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flute-dream
```

The scripted demo, which reaches full score: the left hand takes the body 14 cm from its socket and the right hand
takes the head joint 7 cm back from its tenon's tip, both from above with the fingers closing across the tubes, and
they lift them out of the case. The left holds the body up 10 cm above the table in front of the case, its socket
pointing right, and pushes it back toward the head joint while the right brings the head joint's tenon to the socket's
mouth and pushes it in. Then the right leans 48 degrees about the flute's axis, turning the head joint until its lip
plate is on top (the grip slips a few degrees on the round tube), lets go and goes back to where it started. The left
carries the flute over the open lid, lays it in the three cradles, opens wide to let go and goes back too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=assemble_a_flute_and_line_up_its_embouchure_hole_with_the_keys.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the flute together: take the body out of the case with one hand and the head joint with the other, push the head joint's tenon all the way into the socket at the end of the body, and turn the head joint until the hole in its lip plate lines up with the keys. Then lay the assembled flute on the open lid of the case, keys up, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flute-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flute-dream/*.json; do inspect-robots video "$log"; done
```

Set `FLUTE_DEBUG=1` to print what the score is built from four times a second: where the two parts are, how far the
tenon's tip is off the socket's axis and how far in, the angle between the parts, whether the fit is caught, how far
the lip plate is turned off the keys, whether the keys face up, whether the head joint is all the way in and lined up,
whether the flute lies on the lid, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (227 s on the clock, with Dream answering in 0.80 s on average), and about 2 minutes for the scripted demo (587 steps, 29.4 s of simulated time, 117 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
