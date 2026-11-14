# Load a dental syringe with an anaesthetic cartridge, ready for the needle

A stainless steel set-up tray, as laid out at a dental chair. On it a breech-loading aspirating dental syringe lies
level on a silicone syringe rest, its loading window up and its piston part way in, and the 1.8 ml glass anaesthetic
cartridge chosen for the patient lies in a cartridge rack, its dark rubber stopper toward the robot and its silver cap
away. The job is routine chairside work for dental assistants, hygienists and dentists: load the syringe so it is ready
for the needle, and leave it lying on its rest. That means drawing the piston fully back, dropping the cartridge into
the window with its rubber stopper toward the piston, and pushing the thumb ring firmly enough that the harpoon (the
barbed point on the piston) bites into the stopper, but not so hard that the stopper is driven into the cartridge. With
the harpoon set, a pull on the thumb ring draws the stopper back: that is how the dentist aspirates before injecting.
The task is one task file,
[`load_an_anaesthetic_cartridge_into_a_dental_syringe.py`](load_an_anaesthetic_cartridge_into_a_dental_syringe.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

The instruction gives the goal and the end state only ("Load the syringe with the anaesthetic cartridge, ready for the
needle, and leave it lying on its rest."). Which way round the cartridge goes, that the piston must come back first, and
how hard to push are left to the policy.

What it tests: a real clinical loading sequence on hand-sized parts at their real size (a 68 g syringe, a 6 g glass
cartridge 8.65 mm across), getting the cartridge's orientation right from how it looks, placing it into a 9.4 mm trough,
and a force-controlled push: at least 15 N to set the harpoon and well under 35 N, without the light syringe sliding off
its rest.

Hands: at the chair the job takes two hands, one holding the barrel while the other works the thumb ring, and the demo
does it that way: the 15 N push that sets the harpoon slides the light syringe along its rest unless the barrel is
held. The task does not claim that two hands are needed, though. In a one-hand trial (the right arm doing exactly its
part of the demo, the left never moving) the unheld syringe slid about 4 cm along its rest, rode up about 2.5 cm and
wedged, and the harpoon set anyway. The syringe then dropped back off its rest, 6 mm aside and 3 mm up, so that trial
scored 0.5; with a luckier landing one hand could score more.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray, the syringe rest and the cartridge rack are
kinematic, fixed where they are put; the syringe's frame, its piston, the cartridge and its rubber stopper are loose
rigid bodies made of shapes from `bodykit.py`.

- **Syringe and piston.** The syringe's frame is one steel body: the barrel, whose loading window is a trough open on
  top (9.4 mm bore, walls up to the axis, edges bevelled 1.5 mm), the closed front section and needle adapter, and
  the back end with the finger grip. The piston (rod, harpoon point and thumb ring) slides in it on a prismatic joint,
  from fully drawn back to fully in, with stick-slip friction: after every control step the task file moves the
  joint's spring hold to wherever the piston has been pushed or pulled more than 1 mm past it, so it takes about
  1.5 N to slide and stays where it is left.
- **Cartridge and stopper.** The glass cartridge is its own body; its open end (the stopper's end) is hollow, so the
  piston's head can follow the stopper in. The rubber stopper is a separate body on a slide joint inside the glass.
  Forward (into the solution) a stiff spring holds it until 35 N; pushed harder it slides in and stays (the task file
  moves the spring's hold). In a real sealed cartridge the stopper hardly moves before a needle pierces the
  diaphragm, and that much force cracks the glass or blows the diaphragm instead: the slide stands in for that damage,
  and a stopper driven in more than 2 mm zeroes the score for good. Backward a soft spring (2 N per mm, the vacuum a
  pulled-back stopper draws in a sealed cartridge) pulls it back where it was; pulled hard it can come up to 2 mm out
  past the glass's rim.
- **Harpoon.** When the harpoon's point meets a stopper's face square on (within 1.5 mm of its middle and 10 degrees
  of its axis), a joint between the piston and the stopper is switched on whose one free slide is the point's depth
  in the rubber. Until the barb is in, the rubber pushes the point back like a stiff spring (15 N at 1 mm), and backed
  off the face the point comes free again. Pushed 1 mm in, that is with 15 N, the rubber gives: the barb goes in to
  the rod's shoulder (3 mm) and the rubber closes behind it, so the stopper now follows the piston both ways. A pull
  of more than 40 N tears the barb out. The cartridge loaded the wrong way round meets the piston with its metal cap,
  and nothing catches.
- **Aspiration test.** Once the loaded syringe has lain still on its rest for 0.5 s with both hands at least 8 cm
  from it, the scorer pulls the thumb ring back 8 mm through the piston's own joint (a pull of at most 12 N, between
  the piston and the frame, so the syringe itself is not moved) and lets it back. The cartridge has 5 mm of play in
  the window, so the first part of the pull may only slide it back against the barrel's back end; the harpoon counts
  as set if the stopper then slid back at least 2 mm in its glass. The result stands while nothing is touched.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the syringe, its piston and the cartridge still)
has held for 1 s, otherwise `score()` at the end of the episode (0, 0.1, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it. The cameras render at 1280 x 704 for sharp
videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176`
gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_an_anaesthetic_cartridge_into_a_dental_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Load the syringe with the anaesthetic cartridge, ready for the needle, and leave it lying on its rest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dental-dream
```

The scripted demo, which reaches full score:
1. The left hand takes the barrel's front section; the right takes the thumb ring and draws the piston fully back.
   Both let go and the left goes home.
2. The right takes the cartridge from the rack across its middle, turns it a quarter turn so its stopper end points
   at the piston, holds it level over the window and lets it drop in.
3. The left takes the front section again and the right the thumb ring. The right pushes the ring firmly: the cartridge
   slides forward against the barrel's front, the harpoon's point meets the stopper and, at about 15 N, bites in. The
   push peaks at about 26 N, under the 35 N that would drive the stopper in.
4. The right eases off and pulls the ring gently back: the stopper follows (the cartridge slides back against the
   barrel's back end). It lets the ring forward again.
5. Both let go and go home. The scorer's aspiration test then pulls the ring 8 mm and the stopper slides back in its
   glass.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_an_anaesthetic_cartridge_into_a_dental_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Load the syringe with the anaesthetic cartridge, ready for the needle, and leave it lying on its rest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dental-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/dental-dream/*.json; do inspect-robots video "$log"; done
```

Set `DENTAL_DEBUG=1` to print four times a second what the score is built from:
- where the piston is and where its friction holds it;
- whether the harpoon has met a stopper, its depth in the rubber and whether the barb is in;
- how far the stopper is driven in or drawn back, and where its slide holds it;
- whether and how the cartridge lies in the barrel (and which way round);
- whether the syringe lies on its rest, the hands are clear and the syringe is still;
- the aspiration test's state and its last result (ring pulled, stopper followed);
- where both TCPs and the thumb ring are, and the score.

## How it differs from other syringe tasks

Drawing a dose from a vial (#1095), loading a filled syringe into a pump (#892) and purging a syringe's air bubble
(#1151) handle a plain plastic syringe. Here the syringe is a breech-loading dental syringe and the skill is loading
it: the piston drawn back first, a glass cartridge dropped in the right way round, and a push with a force window, firm
enough to set a barb and soft enough not to drive the stopper. No dental syringe is on the board.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 2,400 steps (555 s on the clock, with Dream answering in 0.89 s on average), and about 4 minutes for the scripted demo (707 steps, 35.4 s of simulated time, 235 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

The layout is the same in every trial; the task file uses no randomness. `-E object_jitter=0` keeps every object where
the task file puts it.
