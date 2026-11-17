# Assemble a mini chopper so it is ready to run, then pulse it once

The trial uses a real, everyday product: the **Black+Decker HC150B one-touch 1.5-cup electric chopper** (about
$15-21 at Amazon, Walmart, Target and Home Depot). It stands on the table at its listed size: its black motor base,
with a switch tower on its left, and the clear bowl already set on the base. Its two loose parts lie beside it: the blade
(a black hub with two stainless blades, standing on its hub) and the clear lid (lying flat, its long tab pointing at the
tower).

The job is the one the maker's manual gives, done before every use:
1. Put the blade into the bowl over the centre shaft, and turn it until it drops to the bottom. Its hub is keyed to the
   motor's drive.
2. Fit the lid with its seal pressed home and its tab lying over the tower.
3. Press the textured pad on the tab to pulse: the motor runs while it is held. It runs only when the lid is fitted
   (the cover interlock).

The task file is
[`assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py`](assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests:
- putting a household appliance together in the only order that works (blade, then lid, then the pulse);
- seating a keyed blade hub over a shaft: the key is hidden inside the hub, so the blade must be turned until it drops;
- pressing a sealed lid home with its tab lined up over the tower (its pin must go into the tower's hole);
- keeping the fingers off the blade;
- checking the result by running it once.

## The product, and where each feature comes from

| Feature the trial needs | Black+Decker HC150B (maker's manual and listing) |
| --- | --- |
| Blade on a shaft in the bowl | Manual: "Place the blade assembly into the bowl, over the center shaft. Rotate the blade until it rests on the bottom of the bowl." Part HC150B-02 (blade assembly). |
| Lid that locks on | Manual: "Fit the lid on the bowl, making sure the seal is secure." Parts HC150-01 (cover) and HC150-04 (cover seal). The lid is pressed on; it is not twisted. |
| Push button | Manual: "Press the textured area on the tab to pulse." The pad is on the lid's tab, which lies over the tower. |
| Interlock | Manual: "Be certain cover is locked securely in place before operating appliance. Do not attempt to defeat the cover interlock mechanism." |
| Bowl on the base | Manual: "Place the bowl onto the base, fitting the tabs on the base of the bowl into the slots on the base." Part HC150-03 (bowl). In the trial the bowl starts on the base. |
| Size, mass, power | Listing: 5 in tall, 6.5 in wide, 4 in deep; 1.2 lb; 1.5-cup capacity; one speed; 70 W rated ("150 W" in the marketing copy). |
| Price, availability | About $15-21 (Amazon $14.44 on sale and about $20 list, Walmart $20.99, autumn 2026); also at Target and Home Depot. |

Sources:
- Use and Care Manual HC150B:
  https://s7cdn.spectrumbrands.com/~/media/SmallAppliancesUS/Black%20and%20Decker/Files/HC150B%20Use%20and%20Care%20Manual.pdf
  (we read the copy at https://www.manualslib.mx/manual/289636/Blackplusdecker-Hc150B.html);
- maker's product page: https://blackanddeckerappliances.com/products/one-touch-1-5-cup-capacity-black-chopper-hc150b;
- listing with dimensions: https://www.target.com/p/-/A-93310378;
- price: https://slickdeals.net/f/20051334-14-44-black-decker-1-5-cup-electric-food-chopper-one-touch-pulse-150w-motor.

**What is measured and what is estimated.** The overall size, mass and capacity come from the retail listing, and the
steps above from the maker's manual.
Everything else in the simulation is our estimate, scaled from the maker's photos to the listed 5 x 6.5 x 4 in. This
covers each part's size, the split of the 1.2 lb between parts, the seal's push-on force (about 10 N) and the pad's
force (9 N at its switch point). The same goes for the hidden details: the blade's key is modelled as two notches at
the foot of its bore over a cross pin, and the interlock as a pin under the tab that must enter the hole in the tower's
top, under the pad. We have not measured a unit. None of these numbers changes the real trial, which uses the real
product.

## Arms

The job needs only one hand at a time, and both arms take part only because of where the parts are:
- the right arm seats the blade and fits the lid (both start on its side);
- the left arm presses the pad, on the tower to the left.
The two arms never work over the bowl together. Wrists closer than about 15 cm shove each other, so the left waits at
home while the right works.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The chopper, its drive, the blade, the lid and its pad
are loose rigid bodies made of shapes from `bodykit.py`, at the sizes and masses in `task.json`.
- **The chopper:** the motor base, its switch tower and the bowl are one body. The bowl stays in the base's slots
  throughout. It stands on rubber feet (friction 0.9).
- **The blade's key (real contact physics).** The motor's drive is a stainless cross pin, 20 mm long and 3 mm square,
  through the foot of the centre tube. The drive turns on its own axle (a revolute joint), and a spring (the stopped
  motor's gear) holds it there.
  - At the foot of the blade's bore are two notches. They pass the pin only when the blade is turned within about
    21 degrees of it, either way round.
  - Otherwise the bore's foot rests on the pin, and the blade stands 5 mm high.
  - The centre tube's rounded top leads the bore on.
  - The blade is on its drive when its hub's bottom is within 2.5 mm of the floor, its axis within 3 mm of the
    tube's, and it is tilted at most 8 degrees.
- **The lid's seal.** The seal ring fits inside the bowl's mouth with a lead-in.
  - Once it is in the mouth (centred within 6 mm, its plate within 7 mm of the rim, tilted at most 8 degrees), a
    `Thread` joint from `bodykit.py` (a PhysX D6 joint) holds it on the bowl's axis.
  - A spring on the joint's slide holds the lid where it was left. After every control step, a lid pushed (or pulled)
    3.3 mm past that hold has slipped, and the task file moves the hold there. So it takes about 10 N to push the lid
    home, or to pull it off.
  - Pressed home (its plate within 1.5 mm of the rim), the seal snaps past its lip and the hold moves to home.
  - Pulled up 9 mm, the lid is off: the joint switches off.
- **The interlock (real contact physics).** The pin under the tab must go into the hole in the tower's top before the
  lid can go home.
  - The hole is 24 x 14 mm; the pin is 7 mm across. The hole takes the pin when the lid is turned within 6.1 degrees.
  - A lid set on turned further rests on its pin, about 4.5 mm high, and is not fitted.
  - Fitted is a live state, worked out after every control step: the seal caught, its plate within 1.5 mm of the
    rim, the lid tilted at most 3 degrees, and turned within 7.6 degrees of the tower. The hole itself stops the pin at
    6.1 degrees, so the check allows 1.5 degrees for contact.
- **The pad.** The pad is its own body (a grey disc 2.2 cm across, 8 g) on a `Slider` (a PhysX prismatic joint) on the
  tab, with 3 mm of travel.
  - A spring drive, preloaded against its top stop, pushes it up: 6 N at rest, 9 N at 2 mm, 10.5 N at its stop
    (1,500 N/m).
  - After every control step the task file measures how far the pad is pushed down. Its switch closes at 2 mm and
    opens again above 1 mm.
  - The motor runs while the switch is closed and the lid is fitted. After every control step the task file then sets
    the drive turning at 25 rad/s, and the blade turns with it through its key.
  - Every motor start counts. A pulse is a start with the blade already on its drive. Full credit needs the motor
    started exactly once, with that start a pulse.
  - Pressing the pad also pushes the lid down, as pressing the real tab does.
  - Pressing the pad with the lid not fitted does nothing, and so does resting on it without pushing it 2 mm down.
- **The blade rule.** After every control step, each gripper's fingertip centre and its pads' tips are measured against
  the middle line of both blades' cutting parts (the 1.2 cm wide blades beyond 2 cm from the hub's axis). Within
  1.4 cm of either (about 8 mm from its edges), the trial scores 0 for the rest of the trial.
- **The clear parts.** The bowl's wall and the lid's plate collide but are not drawn, so they look clear. Their rims,
  seams and the oil holes are drawn.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose, and lets the task file act on the simulation after every control step.

## Real-trial safety and rig protection

- **Blade edges.** The blade's edges are sharp. Before the first trial, cover each blade's cutting edge with one layer
  of clear packing tape, which does not change how the blade fits or turns. The task lets the grippers touch only the
  hub, and a trial that brings a finger near a blade's edge scores 0. The blade can spin only with the lid fitted (the
  cover interlock), so no finger can reach it while it turns.
- **Electrics.** Plug the chopper into a GFCI-protected outlet, with the cord led away behind the base, clear of both
  arms. Unplug it to reset.
- **Food.** No food is used, and nothing contaminates the grippers: they touch only the dry blade hub, the lid and the
  pad. Each pulse is about a second, well under the manual's limit of a minute of continuous running.
- **Forces.** The forces are well inside the YAM's limits: about 10 N to press the lid home and about 9 N on the pad.
  The parts weigh 40 g (blade) and 60 g (lid).

## Third-party assets

None. Every shape is a box, cylinder or sphere built by the task file; the arms are yam_arena's own. We searched NVIDIA's
SimReady library (Hugging Face `nvidia/simready-assets`, at revision 9d4556bf, 2,679 models) for a blender, chopper or
food processor and found none.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 once `success()` has held for 1 s: full score (which includes both arms back
where they started) with the chopper, the lid, its pad and the blade still. Otherwise it gives `score()` at the end of
the episode: 0, 0.25, 0.5, 0.75 or 1.0 (see `task.json`). `-E object_jitter=0` keeps every object exactly where the
task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos.
Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the
policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Assemble the mini chopper so it is ready to run, then pulse it once. Touch the blade only by its hub. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chopper-dream
```

The scripted demo reaches full score. The left waits at home throughout the right's part.
1. **Blade (right).** The right comes down round the knob on the blade's hub, lifts the blade and carries it over the
   bowl, turned 40 degrees off the drive's cross pin. It lowers the hub over the centre tube, where the blade rests on
   the pin. It turns the blade until its notches meet the pin and it drops to the bottom, then lets go. Low over the
   bowl the right TCP falls about 6 mm short, so the demo aims that much further.
2. **Lid (right).** It rises well clear and takes the lid by its raised middle, fingers closing in line with the tab.
   The lid hangs 17 mm off the TCP along the closing axis, as wide parts do in this gripper, so the demo aims that much
   to the side. It carries the lid over the bowl with the tab over the tower, lowers the seal into the bowl's mouth and
   presses it home: the tab's pin goes into the tower's hole. It lets go and goes home.
3. **Pulse (left).** The left comes in on the tower's side and pushes the pad on the tab down with its closed fingers.
   The pads' tips are aimed 2.6 cm below its top, as the arm gives. The motor runs and the blade spins while the pad
   is held down. The left lets it up and goes home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Assemble the mini chopper so it is ready to run, then pulse it once. Touch the blade only by its hub. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chopper-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/chopper-dream/*.json; do inspect-robots video "$log"; done
```

Set `CHOP_DEBUG=1` to print what the score is built from, four times a second (every step while the pad is pushed
down):
- the chopper's place and turn;
- the lid's seal: caught, height above the rim and its hold, centring, tilt and turn, and whether it is fitted;
- the blade's place against its seat, its turn and tilt, the drive's turn, and whether the blade is on its drive;
- the pad's travel, its switch, the motor, and the starts and pulses so far;
- the blade rule;
- each TCP, with the lid's and the blade's offsets from the right TCP and both grippers' finger openings.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (545 s on the clock, with Dream answering in 0.72 s on average), and about 5 minutes for the scripted demo (770 steps, 38.5 s of simulated time, 326 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-CXHEDY. Its latest review (after our reply) said:

> We looked at this task again after your reply, and it can be revised.
>
> What to change: Please identify an affordable, available mini chopper whose blade, motor-top twist lock, button and interlock support the specified trial, and update the setup and simulation to match it. If using a custom-built mechanism instead, provide the drawings or fabrication files and parts needed to reproduce a working assembly, and demonstrate the revised setup if its operation changes.

Earlier reviews:
- **TK-Z7XA5C** asked for two things: a pulse that needs the button really pressed rather than a fingertip hovering, and a locked state checked live. Both were done in the previous revision and are kept. The pad is a sprung slide that must be pushed 2 mm, and "fitted" is worked out live after every step.
- **TK-CXHEDY's first decision** ("not sufficiently relatable for this category") was reconsidered after our reply, and the task was reopened for revision. The category stays Societal relatability, and the context is about the everyday kitchen job.

What changed in response:
- **An affordable, available chopper.** The trial now uses a stock Black+Decker HC150B one-touch 1.5-cup chopper, about $15-21 at Amazon, Walmart, Target and Home Depot. Its manual covers each feature the trial needs (see the table above, with sources):
  - the blade goes "over the center shaft" and is rotated "until it rests on the bottom of the bowl";
  - the lid is fitted "making sure the seal is secure";
  - "press the textured area on the tab to pulse";
  - "do not attempt to defeat the cover interlock mechanism".
- **Its real lock, not a twist.** The HC150B has its motor in the base, and its lid is pressed on, not twisted, so the motor-top twist lock is gone. The simulation now models the product:
  - a keyed blade drive: the blade rests 5 mm high on the drive's cross pin until it is turned within about 21 degrees of it, then drops;
  - a seal that takes about 10 N to press home;
  - a pin under the tab that must go into the tower's hole (within 6.1 degrees) before the lid can seat;
  - the pad on the tab (9 N at its 2 mm switch point), which runs the motor only while the lid is fitted.
- **Setup and simulation match the product.**
  - The scene is the HC150B at its listed size: modelled 12.9 x 16.1 x 10.6 cm and 0.55 kg, against the listed 5 x 6.5 x 4 in and 1.2 lb.
  - `task.json` gives every part's size, material, mass and place.
  - Sizes and forces the maker does not publish are marked as our estimates.
  - The old cordless motor-top chopper is gone.
- **No custom mechanism.** Nothing is custom-built: the real trial uses the stock product, so no drawings or fabrication files are needed.
- **The revised setup is demonstrated.** Its operation changed: the blade is turned onto its key, the lid is pressed on, and the other arm presses the pad on the tower's side. A new scripted demo shows each step and reaches 1.0 (770 steps of the 1,200 allowed); the new Dream-1 run scores 0.0. A separate interlock check scored 0.25: we ran the demo with the lid set on about 18 degrees off, so its pin rests on the tower's top and the lid cannot be pressed home. Its pad was then pushed fully down, 3.3 mm, past its switch point, and the motor never started (the blade on its drive is the only credit).
- **Also new.** A "Real-trial safety and rig protection" section (tape on the blade edges, GFCI outlet, no food) and a "Third-party assets" note (none used).
