# Find the buttons among four identical sewing boxes, opening each box only once and sliding each lid shut again

Four identical pine sewing boxes with sliding lids stand on a polished walnut sideboard, two on each side of the
middle, each with its closed end toward the middle and a small wooden knob at the outer end of its lid. One of them
holds a little clear box of buttons; the others hold thread spools, balls of wool, and a tape measure with tailor's
chalk. In front of the sideboard lies a sewing tray with a folded shirt that has lost a button. The robot looks for the
buttons one box at a time: one hand presses the box down by its closed end while the other slides the lid out by its
knob, and if the buttons are not there it slides the lid shut again and goes on to the next box. When it finds them it
lifts their box out onto the sewing tray beside the shirt, slides that lid shut and takes both arms home. The task is
one task file,
[`find_the_buttons_opening_each_sewing_box_only_once.py`](find_the_buttons_opening_each_sewing_box_only_once.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: memory of one's own search. With their lids shut the four boxes look exactly alike, so after the first
box or two nothing in view says which boxes have already been looked into. A policy that acts on the current frame
alone sooner or later opens a box it has already checked, and opening a box again after it was shut is latched as a
failure, as is having two boxes open at once or moving a box out of place (so checked boxes can't be set aside as a
marker). Which box holds the buttons depends on the seed, so a fixed habit (always the front right box, say) does not
pass either. Along the way: holding a light box still while working its stiff sliding lid, and lifting a small box out
of a narrow one.

Why two hands: a lid runs stiffly in its grooves (about 1.2 N to slide it), while the boxes stand on the polished
sideboard and slide at about 0.25 N. Pulled by its knob with nothing holding the box, the whole box comes along and the
lid does not open, so one hand presses the box's closed end down while the other slides the lid. With one hand (the left working the lids exactly as in the demo, the right never moving)
the demo scored 0: pulling the first box's lid slid the whole box about 14 cm along the sideboard while the lid stayed
shut, and a box moved more than 4 cm fails the trial.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The sideboard and the sewing tray are kinematic,
fixed where they are put; the four boxes, their lids and the button box are loose rigid bodies made of shapes from
`bodykit.py`. Each lid runs on a prismatic joint in its box, from shut to 12.5 cm out (a peg in the groove stops it
there); jointed bodies do not collide.

- The grooves: after every control step the task file moves each lid's drive target to where the lid is, so the
  drive holds the lid there with up to 1.2 N, like wood in a snug groove. Within 3 mm of shut the target is shut: the
  lid drops into its seat.
- The sideboard's contacts take the lower of the two frictions (0.05): a box slides at about 0.25 N.
- What each box holds: as the embodiment resets, the task file catches the trial's seed. The buttons are in box
  (seed mod 4) (0 back left, 1 front left, 2 front right, 3 back right); the other three hold the spools, the wool and
  the tape measure in an order also set by the seed. The spools, wool and tape are drawn shapes that are shown only in
  the box holding them; the button box is a loose body, moved into its box at the trial's first step (it starts in
  box 2, the default seed's box).
- Opened: a lid more than 10 mm out (the contents sit at least 2.5 cm from the closed end, so a crack shows nothing).
  Shut again: an opened lid back within 6 mm of shut. Opening a box again after that, two lids open at once, or a box
  moved more than 4 cm or tipped more than 15 degrees fails the trial for good.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the boxes, their lids and the button box still) has held for 1 s, otherwise `score()` at the end of the episode (0,
0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=find_the_buttons_opening_each_sewing_box_only_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The buttons for the shirt on the sewing tray are in one of the four pine boxes on the sideboard. Find them: open the boxes one at a time, sliding the lid out by its knob while your other hand holds the box down by its closed end, and slide each lid shut again before you open the next box. Never open the same box twice, and do not move the boxes from where they stand. When you find the buttons, take their little clear box out and put it on the sewing tray beside the shirt, slide that box's lid shut, and take both arms back to where they started." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/sewing-dream
```

The scripted demo, which reaches full score with the default seed (the buttons in box 2, front right): it looks in
box 0 (back left), then box 1 (front left), then box 2. For a box on the left, the right hand presses the box's closed
end down with its closed fingers, leaning against the pull, while the left takes the knob and slides the lid out to
its stop, pauses, and slides it shut again; for a box on the right the hands swap. In box 2 it finds the buttons: the
left lets go and goes home, the right lets go of the knob, lifts the button box out and sets it on the sewing tray
beside the shirt, then goes back over the knob; the left presses the box down again, the right takes the knob and
slides the lid shut, and both arms go home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=find_the_buttons_opening_each_sewing_box_only_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The buttons for the shirt on the sewing tray are in one of the four pine boxes on the sideboard. Find them: open the boxes one at a time, sliding the lid out by its knob while your other hand holds the box down by its closed end, and slide each lid shut again before you open the next box. Never open the same box twice, and do not move the boxes from where they stand. When you find the buttons, take their little clear box out and put it on the sewing tray beside the shirt, slide that box's lid shut, and take both arms back to where they started." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/sewing-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/sewing-dream/*.json; do inspect-robots video "$log"; done
```

Set `SEWING_DEBUG=1` to print what the score is built from four times a second: how far each lid is out, each box's
state (never opened, open, shut again), how far each box has moved and tipped, where the button box is and whether it
is on the tray, whether the buttons have been found, what failed the trial if anything, whether both arms are home, the
finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 2,000 steps (621 s on the clock, with Dream answering in 0.68 s on average), and about 7 minutes for the scripted demo (1,146 steps, 57.3 s of simulated time, 394 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

`inspect-robots run` passes every trial a seed (with `--seed 0`, its default, and one ad-hoc scene, the same seed every
time: 1415158438, which puts the buttons in box 2, the front right one). The task file uses it for one thing: which box
holds what, the buttons in box (seed mod 4). The scripted demo is a fixed list of waypoints for that default seed; with
another seed the buttons are elsewhere and the demo would search the wrong boxes. `-E object_jitter=0` keeps
everything else where the task file puts it.
