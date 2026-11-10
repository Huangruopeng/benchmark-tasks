# Turn a practice amp's volume all the way down to plug the guitar cable in, then set it back where it was

A small tweed-covered guitar practice amp stands switched on (its red pilot light lit) on a smooth grey desk pad, its
front toward the right arm. On its top are three black chicken-head knobs, VOLUME (nearest the arms), TONE and GAIN,
each with a white pointer line in a ring of printed ticks; VOLUME is turned part way up. A guitar cable's jack plug
lies on the pad in front of the INPUT socket on the amp's front. The volume has to come all the way down before the
plug goes in (or the amp pops), and then go back to where it was. Both arms go back to where they started. The task
is one task file,
[`turn_an_amps_volume_down_to_plug_in_then_set_it_back_where_it_was.py`](turn_an_amps_volume_down_to_plug_in_then_set_it_back_where_it_was.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: memory. The VOLUME knob's setting is in view only until the knob is turned down; after the plug is in,
the robot has to set it back from what it saw at the start, with nothing on the amp showing where that was. A policy
that acts only on the current frame can at best guess: a guess anywhere along the knob's 270 degrees lands within the
15 degrees allowed about one time in nine. It also tests getting a safety order right (volume down before plugging
in), turning a knob, plugging in a jack, and holding a light part still while the other hand works on it.

Why two hands: the amp is light (0.55 kg) and stands on slick glides (friction 0.05). The plug needs 1.5 N to go into
the socket, while an unheld amp slides on the pad at about 0.3 N, so pushing the plug in just pushes the amp away. One
hand steadies the amp by its carry handle while the other turns the knob and pushes the plug in. With one hand (the
right doing its part of the demo, the left never moving) the trial scored 0: the right could still turn the VOLUME
knob down on its own (the amp twisted 7 degrees as the knob reached its stop), but pushing the plug in slid the amp
4.6 cm across the pad and the plug never went home; the right's last turn then missed the knob, which had moved away
with the amp.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The desk pad is kinematic, fixed where it is put;
the amp, its three knobs and the plug are loose rigid bodies made of shapes from `bodykit.py`.

- The knobs: each is joined to the amp by a `Hinge` from `bodykit.py` about its own axis, from fully down (135 degrees
  anticlockwise of pointing to the amp's back) to full up (135 degrees clockwise). A spring holds the knob where it
  was last left, with its torque capped at 0.15 N m: after every control step, a knob pushed more than 2 degrees past
  its hold has turned, and the task file moves the hold there. So a knob turns only when pushed with about 0.15 N m and
  stays where it is left, as a pot's friction does.
- The INPUT socket: the cabinet's collision shapes leave a 15 mm channel behind the socket. Once the plug's tip enters
  the socket's mouth (within 8 mm of its axis, tilted 12 degrees or less), the task file switches on a `Thread` from
  `bodykit.py`, which holds the plug on the socket's axis and lets it slide along it against a capped spring (1.5 N)
  whose target the task file moves to wherever the plug has been pushed; so the plug slides in only when pushed with
  1.5 N and stays where it is left. Pulled back out, it is let go. At that first moment the task file also checks the
  VOLUME knob: if it is more than 12 degrees off fully down, the trial is marked as plugged in loud and scores 0 from
  then on.
- The pad is slick: the amp's contacts with it take the pad's low friction.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the amp, the VOLUME knob and the plug still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every
frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_an_amps_volume_down_to_plug_in_then_set_it_back_where_it_was.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Plug the guitar cable into the practice amp without a pop, then put its volume back. The amp is light and slides on its glides, so steady it by its carry handle with one hand while the other works. First turn the VOLUME knob (the black chicken-head knob nearest you) anticlockwise all the way down to its stop. Only then push the cable's jack plug fully into the INPUT socket on the amp's front: plugging in while the volume is up scores 0. Then turn the VOLUME knob back up to where it was set before you turned it down. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/amp-dream
```

The scripted demo, which reaches full score: the left hand comes down on the carry handle and holds the amp. The right
comes down on the VOLUME knob, takes its pointer and turns it anticlockwise, the wrist turning with it, all the way
down (aimed 5 degrees past, against the stop), lets go and rises. It takes the jack plug off the pad by its barrel,
lifts it to the socket's height, lines it up and pushes it home (aimed 1.5 cm past), lets go and rises. It takes the
VOLUME knob again and turns it clockwise back to where it was set, lets go, rises clear of the amp and goes home; then
the left lets go of the handle and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_an_amps_volume_down_to_plug_in_then_set_it_back_where_it_was.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Plug the guitar cable into the practice amp without a pop, then put its volume back. The amp is light and slides on its glides, so steady it by its carry handle with one hand while the other works. First turn the VOLUME knob (the black chicken-head knob nearest you) anticlockwise all the way down to its stop. Only then push the cable's jack plug fully into the INPUT socket on the amp's front: plugging in while the volume is up scores 0. Then turn the VOLUME knob back up to where it was set before you turned it down. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/amp-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/amp-dream/*.json; do inspect-robots video "$log"; done
```

Set `GUITARAMP_DEBUG=1` to print what the score is built from four times a second: each knob's turn on the amp, where
the amp is and how far it has turned, where the plug is and how far its tip is off the socket's axis and into it,
whether it is held in the socket and home, whether it went in loud, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,400 steps (485 s on the clock, with Dream answering in 0.61 s on average), and about 3 minutes for the scripted demo (738 steps, 36.9 s of simulated time, 170 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints. The
VOLUME knob's starting setting is part of the fixed layout; the policy sees it only at the start.
