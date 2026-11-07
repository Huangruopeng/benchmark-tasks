# Reload a long-arm stapler, then staple two sheets: one hand holds the sprung magazine out while the other lays in the staples

A long-arm stapler stands on the table, empty. Its staple magazine is a channel that slides out of the back of the
stapler's housing against a spring: pulled out, the channel's front part comes out into the open; let go, it springs
back in. One hand pulls the magazine out by its tab and holds it there; the other takes a strip of staples from a tray
and lays it in the open channel. The magazine is let slide back in, carrying the staples into the housing. Then a hand
slides two sheets of paper in under the stapler's head, up to the paper guide, presses the top arm down (a staple goes
through both sheets), pulls the stapled sheets back out onto the paper rest, and both arms go back to where they
started. The task is one task file,
[`reload_a_long_arm_stapler_then_staple_two_sheets.py`](reload_a_long_arm_stapler_then_staple_two_sheets.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the
same folder.

What it tests: holding a spring-loaded part open with one hand while the other places a small part into it, letting it
close gently, then using the tool it loaded: lining paper up in a throat, working a sprung lever, and taking the result
out.

Why two hands: the magazine springs shut as soon as it is let go, closing the channel inside the housing, and an empty
stapler staples nothing. With one hand (the right pulling the magazine out as in the demo, the left never moving) the
demo scored 0.0: the magazine was pulled out and slid back in empty, the strip still on its tray.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stapler and the tray are fixed; the magazine, the
top arm, the strip of staples and the two sheets are loose rigid bodies made of shapes from `bodykit.py`.

- The magazine: a channel on a slide (a `Slider` from `bodykit.py`) that runs inside the housing, from shut out to 20
  cm, with a spring (20 N/m) pulling it back in.
- The strip of staples: one rigid steel bar; it rides in the channel and goes into the housing with it.
- The top arm: a hinge (a `Hinge`) on the housing's back end, sprung up 4 degrees; pressed down flat, it pushes back
  with about 3 N at its pad.
- The sheets: stiff 3 mm boards; the base under them is slick, so a fingertip pressed on the top sheet slides both.
- Stapling: after every control step, the task file checks the stapler. When the top arm is pressed down at least 3
  degrees while the stapler is loaded (the strip in the channel, the magazine within 5 mm of shut) and both sheets lie
  flat on the base with the staple point at least 5 mm inside each, a staple goes in: a small staple body, until then
  hidden inside the lower sheet and held to it (a `Glue` from `bodykit.py`), moves onto the top sheet over the staple
  point and a second `Glue` holds the top sheet to it, so the two sheets move as one from then on.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with the
magazine, the strip, the top arm and both sheets still) has held for 1 s, otherwise `score()` at the end of the episode
(0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so
every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Reload the long-arm stapler, then staple the two sheets of paper together: with one hand pull its staple magazine out by the red tab at its back and hold it out; with the other take the strip of staples from the tray and lay it in the open channel; then let the magazine slide back in and let go. Next slide the two sheets on the paper rest in under the stapler's head, up to the paper guide, press the stapler's top arm down to staple them, and pull the stapled sheets back out from under the head onto the paper rest. Finally take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/stapler-dream
```

The scripted demo, which reaches full score: the right hand comes straight down on the magazine's tab, fingers closing
across it, pulls the magazine out 20 cm and holds it there. The left hand takes the strip of staples off its tray,
fingers closing across it, carries it over the open channel, sets it down on the channel's floor and lets go. The right
lets the magazine slide back in slowly, lets go of the tab and goes back to where it started. The left presses its
closed fingertips on the top sheet near its back edge and slides both sheets in under the head against the paper guide,
presses the top arm's pad down (the staple goes in), presses on the stapled sheets again and slides them back out onto
the paper rest, and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Reload the long-arm stapler, then staple the two sheets of paper together: with one hand pull its staple magazine out by the red tab at its back and hold it out; with the other take the strip of staples from the tray and lay it in the open channel; then let the magazine slide back in and let go. Next slide the two sheets on the paper rest in under the stapler's head, up to the paper guide, press the stapler's top arm down to staple them, and pull the stapled sheets back out from under the head onto the paper rest. Finally take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/stapler-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/stapler-dream/*.json; do inspect-robots video "$log"; done
```

Set `STAPLER_DEBUG=1` to print what the score is built from twice a second: how far the magazine is pulled out, whether
the strip is in the channel, how far the top arm is pressed, where the sheets are and whether they are under the head
and flat, whether they are stapled and clear of the head, whether both hands are clear, and where the TCPs are.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,400 steps (231 s on the clock, with Dream answering in 0.71 s on average), and about 2 minutes for the scripted demo (626 steps, 31.3 s of simulated time, 98 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
