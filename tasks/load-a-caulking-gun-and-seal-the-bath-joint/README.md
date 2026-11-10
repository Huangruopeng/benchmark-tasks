# Load a caulking gun (one hand holds its release down, the other pulls the rod back) and seal a bath joint

An empty caulking gun lies level on the table, nozzle end to the left, on its pistol grip and a wooden rest block. Its
plunger rod is part way in. To the left is the white rim of a bath against a tiled wall, the joint between them not yet
sealed; in front of the gun a new cartridge of white silicone lies on a cardboard tray. One hand presses the gun's
thumb release lever down, which also holds the gun still, while the other takes the rod's hook and pulls the rod all
the way back. The first hand lets go of the lever, lifts the cartridge off its tray and lays it in the cradle, nozzle
through the front yoke; the other pushes the rod back in until its plate meets the cartridge. Then both hands lift the
gun, put the nozzle's tip into the joint at one end and draw it along to the other, laying a bead of sealant, set the
gun back on its grip and rest block, and both arms go back to where they started. The task is one task file,
[`load_a_caulking_gun_and_seal_the_bath_joint.py`](load_a_caulking_gun_and_seal_the_bath_joint.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: operating a tool's two controls at once with two hands (holding a release while pulling against it),
loading a part the right way round into a tool, then guiding the loaded tool along a line with both hands and putting
it back.

Why two hands: the rod is held by a ratchet and comes back out of the gun only while the release lever is held down,
and the lever is about 20 cm from the rod's hook. Pulled on without the lever pressed, the rod stays put and the whole
light gun slides along the table instead; and the cartridge only fits once the rod is all the way back. With one hand
(the right doing its part of the demo, the left never moving) the demo scored 0: the right pulled on the hook, the rod
stayed locked 7 to 8 cm in, the gun slid 2.6 cm across the table and off its rest, and no cartridge was ever loaded.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The bath rim with its tiled wall, the tray and the
rest block are kinematic, fixed where they are put; the gun's frame, its plunger rod, its release lever and the
cartridge are loose rigid bodies made of shapes from `bodykit.py`.

- The rod slides in the frame along the cradle's axis (a `Slider` from `bodykit.py`), from all the way back to its
  plate at the yoke. The ratchet: after every control step the task file reads the lever's angle. While the lever is
  pressed down at least 6 degrees the slide is free (light friction); otherwise a stiff spring holds the rod where it
  is against being pulled back, and its hold moves forward with the rod whenever the rod is pushed in more than a
  millimetre, so the rod goes in freely but cannot come back out.
- The lever is a sprung `Hinge` behind the back plate; pressing its pad turns it down, up to 12 degrees.
- The cartridge lies in the half-round cradle (three steel strips round its underside) and its nozzle drops into the
  yoke's V notch. Pushed in, the rod's plate presses it forward until its shoulder meets the yoke.
- The bead: after every control step the task file finds the nozzle's tip. While the cartridge is seated with the rod
  in against it and the tip is within 1.5 cm of the joint, each 1.6 cm length of the joint the tip passes over gets
  its bead: a white silicone segment, kept hidden inside the rim until then, is placed in the corner there. The
  trigger is not modelled.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the gun, its rod and the cartridge still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every
frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_a_caulking_gun_and_seal_the_bath_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Load the caulking gun and seal the joint between the bath rim and the tiles: with one hand press the gun's release lever down and keep it pressed while the other hand pulls the plunger rod back by its hook, all the way out; let go of the lever, lay the white cartridge from the tray in the gun's cradle with its nozzle through the front yoke, and push the rod back in until its plate meets the cartridge. Then lift the gun with both hands, put the nozzle's tip into the joint and run it along the joint from one end to the other, laying a bead of sealant. Put the gun back on its grip and rest block, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/caulk-dream
```

The scripted demo, which reaches full score: the left hand comes down on the release lever with closed fingers and
presses it, holding the gun; the right takes the rod's hook, fingers closing across its top bar, and pulls the rod all
the way back. The left lets go of the lever, takes the cartridge across its middle, carries it over the gun and lowers
it into the cradle, nozzle through the yoke, and lets go. The right pushes the rod back in until the plate meets the
cartridge and pushes it against the yoke. The left takes the yoke's near side, fingers closing along the gun; both
hands lift the gun, carry it to the joint's right end, lower the nozzle's tip into the joint and draw it along to the
left end, laying the bead. They carry the gun back over its rest, set it down, let go and go back to where they
started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_a_caulking_gun_and_seal_the_bath_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Load the caulking gun and seal the joint between the bath rim and the tiles: with one hand press the gun's release lever down and keep it pressed while the other hand pulls the plunger rod back by its hook, all the way out; let go of the lever, lay the white cartridge from the tray in the gun's cradle with its nozzle through the front yoke, and push the rod back in until its plate meets the cartridge. Then lift the gun with both hands, put the nozzle's tip into the joint and run it along the joint from one end to the other, laying a bead of sealant. Put the gun back on its grip and rest block, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/caulk-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/caulk-dream/*.json; do inspect-robots video "$log"; done
```

Set `CAULK_DEBUG=1` to print what the score is built from four times a second: the lever's angle and whether the rod
is free, where the rod's plate is and where the ratchet holds it, where the cartridge lies in the gun and whether it
is seated with the rod in against it, where the nozzle's tip is and whether it is in the joint, how many bead lengths
are laid, where the gun is and whether it is back on its rest, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (153 s on the clock, with Dream answering in 0.66 s on average), and about 2 minutes for the scripted demo (791 steps, 39.5 s of simulated time, 124 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
