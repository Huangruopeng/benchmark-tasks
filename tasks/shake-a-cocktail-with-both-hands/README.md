# Shake a drink with ice in a sealed Boston shaker and open it without spilling

A two-piece Boston shaker stands on the table: a 28 oz stainless steel tin holding the drink and four 2.5 cm ice cubes,
with an 18 oz tin resting mouth down in its mouth, not yet sealed. The robot seals the shaker (presses the small tin
home, so the press fit jams it in the large tin's taper), shakes it so the ice crashes from end to end with the
shaker sealed the whole time it is tipped, sets it down, breaks the seal by pulling the tins apart and lifts the small
tin off, leaving the drink in the large tin standing on the table without spilling any. In a real trial the drink is
water with a layer of dyed syrup under it, and the score measures that it was mixed (refractometer readings of the top
and the bottom) and that nothing was spilled (kitchen scale). The task is one task file,
[`shake_a_cocktail_with_both_hands.py`](shake_a_cocktail_with_both_hands.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The tins (32-sided cones of steel panels with a
floor), the ice cubes and the drink's beads are loose rigid bodies, so `bodykit.py` adds such bodies: importing it
hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()` each body's own pose,
and adds the joint that is the tins' press fit.

## The real trial

- **Shaker:** a standard 28 oz / 18 oz tin-on-tin set (weighted stainless steel tins as sold for bars): the large tin
  17.5 cm tall, 9.5 cm across the mouth and 7.0 cm across the base, 0.21 kg; the small tin 14.5 cm tall, 8.64 cm across
  its rim and 6.1 cm across its base, 0.14 kg. Set mouth down in the large tin's mouth, the small tin's rim sits 16 mm
  inside and lifts off freely; pressed 5 mm further in, it jams in the large tin's taper and the tins hold together
  until they are pulled (or knocked) apart.
- **Drink:** 90 ml of room-temperature water, then 30 ml of simple syrup (sugar and water 1:1 by weight, about
  50 °Brix, dyed blue) poured in slowly down the side so that it lies as a layer under the water: 120 ml, 127 g.
  Four 2.5 cm ice cubes (14 g each, from a standard tray) straight from a -18 °C freezer go in on top.
- **No spill:** both tins with the drink and the ice are weighed on a kitchen scale (1 g) before the trial; after it,
  both tins, wherever they ended, are wiped dry outside and weighed again with whatever is in them. The loss in grams
  is the millilitres spilled (melting ice changes nothing).
- **Mixed:** within 1 minute of the shaker being opened (by a person, if the robot did not open it), 1 ml is drawn
  with a pipette from 1 cm under the drink's surface and 1 ml from the large tin's floor, and each is read on a
  handheld Brix refractometer (0 to 32 °Brix, 0.2 °Brix divisions). Mixed means the two read within 1.0 °Brix of each
  other. Unshaken, the top reads about 0 and the bottom is off the top of the scale (counted as unmixed); mixed, both
  read about 13 to 15.
- **Sealed while tipped:** judged from the video: the tins never part, and no drink escapes, while the large tin is
  tipped more than 30 degrees.

## How the simulation measures the same

No liquid is simulated, so the simulation cannot measure mixing; it scores what the robot controls, which is what
mixes the drink in a real trial: shaking strokes, with the shaker sealed, that send the ice from end to end.

- **Sealed:** the press fit is a slide joint along the tins' axis. Resting, the small tin lifts off against 1.2 N;
  pushed in with more than 7.2 N it slides 5 mm further in, to 21 mm, and is sealed. A sealed small tin comes out
  only when pulled hard enough to draw it 5 mm out of its seat (12 N) for 0.15 s; then it slides out against 1.2 N.
  The large tin tipped more than 30 degrees while the shaker is not sealed scores 0 for the trial.
- **Shaken:** while the shaker is sealed and tipped past 45 degrees, at least 6 changes of direction of its middle
  along its own axis, each after a leg of 2.5 cm or more, and the ice cubes reaching alternate ends at least 8 times in
  all. A cube is at an end when its centre is within 9 cm of that end's floor and it touches that tin (a PhysX
  contact force), so each count is a cube carried at least 10 cm along the shaker into the other tin.
- **Spill:** the drink is 24 rigid beads, 8 mm and 5.3 g each (127 g, the drink's mass), each standing for 5 ml. A
  bead or a cube whose centre is outside both tins is spilled: at most 1 bead (5 ml) for 1.0 and 0.25, at most 3
  (15 ml) for 0.5, and no ice cube at all.
- **Open:** the small tin off, its rim at least 3 cm from the large tin's mouth, with the large tin standing upright
  on the table (tilted less than 10 degrees).

The two tins do not collide with each other (the joint holds them on one axis; seated, the simulated tins keep a
0.8 mm gap at the seal, narrower than a bead), and the beads and the ice do not collide with the arms: the YAM
fingers' collision hulls reach through a gripped tin's wall and would push what is inside out through it.

## Why two hands

The sealed tins hold together until they are pulled apart (12 N in the simulation; a real seal usually takes a firm
pull or a knock), and the full shaker weighs 0.53 kg, so one hand pulling the small tin just lifts the whole shaker
off the table: the large tin has to be held down by the other hand. One hand can seal and shake the shaker. In the
one-hand check (the right arm plays its demo part, the left stays at its rest pose) it sealed it and shook it (12
changes of direction, 20 ice passes), which held 0.25 while the sealed shaker stood on the table. Its pull then lifted
the still-sealed shaker 8.7 cm off the table, and the trial ended at 0.0. Bartenders also shake a Boston shaker with a
hand on each tin, so a seal knocked by the ice cannot open.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with both tins, the ice and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shake_a_cocktail_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Seal the shaker and shake the drink with its ice until mixed, then open the shaker, leaving the drink in the large tin standing on the table, without spilling any." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shaker-dream
```

The scripted demo, which reaches full score: the right hand grips the small tin from the right, the left grips the
large tin low from the left, pointing 20 degrees down. The right presses the small tin home (sealed). Together they
lift the shaker 3 cm, tip it forward onto its side and shake it: 12 legs that rock it between 65 and 107 degrees from
upright, moving it 3 cm along its axis each way with the sliding ice, with a short pause at each end, so the ice
slides from one end to the other and hits it. They stand it up and set it down; the left holds the large tin down
while the right pulls the small tin 10 cm straight up, breaking the seal, and holds it up over the open large tin
(anything wedged in the small tin can only drop back in), and the left lets go. The YAM fingers' collision shapes are
not quite parallel, so a gripped tin leans a few degrees in the hand; the demo turns each wrist to match:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shake_a_cocktail_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Seal the shaker and shake the drink with its ice until mixed, then open the shaker, leaving the drink in the large tin standing on the table, without spilling any." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shaker-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/shaker-dream/*.json; do inspect-robots video "$log"; done
```

Set `SHAKE_DEBUG=1` to print what the score is built from (the small tin's depth in the mouth, whether it is sealed,
how far the shaker is tipped, the changes of direction and ice passes so far, and anything outside the tins) twice a
second.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (561 s on the clock, with Dream answering in 0.67 s on average), and about 6 minutes for the scripted demo (804 steps, 40.2 s of simulated time, 342 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is a fixed list of waypoints. Keep
the jitter at 0: it shifts each free object on its own, and the two tins start joined by the press fit.

## Changes in this revision

This is a revision of TK-JE4KUU. Its review said:

> Construct validity (not met): An unsealed, loose-fitting cap and 6 mm beads cannot establish that a shaker retains liquid and mixes a drink as bartenders do.
>
> What to change: The capped-shaking requirement improves the previous version, but the specified real-world apparatus uses an unsealed cap and beads rather than a drink. Use a realistic cocktail shaker with water and ice, define a measurable mixing and no-spill outcome, and provide a full-score demo for the revised setup; explain any simulation approximations in the fidelity note.

The review before that (TK-CCNDMT) asked for every nonzero score to need the cap seated and held during the tipping and shaking, a demonstrated full score, and the Societal relatability category; that was done in the previous revision and is kept.

What changed in response:

- **Construct validity (an unsealed, loose cap and 6 mm beads):** the apparatus is now a standard two-piece Boston shaker whose tins seal by a friction press fit. The large tin is a 28 oz stainless steel tin, 17.5 cm tall, 9.5 cm across the mouth and 7.0 cm across the base, 0.21 kg. The small tin is an 18 oz tin, 14.5 cm tall, 8.64 cm across the rim and 6.1 cm across the base, 0.14 kg. The small tin starts resting mouth down in the large tin's mouth, its rim 16 mm inside, unsealed (it lifts off freely). Pressed 5 mm further in, to 21 mm, it jams in the large tin's taper. In simulation the fit is a slide joint along the tins' axis: a push of more than 7.2 N seats it, and a seated tin comes out only when pulled 5 mm out of its seat (12 N) for 0.15 s. The scorer reads this seal state at every step.
- **A realistic shaker with water and ice:** a real trial uses 90 ml of room-temperature water, with 30 ml of blue-dyed simple syrup (sugar and water 1:1, about 50 °Brix) poured in slowly so it lies as a layer under the water (120 ml, 127 g). Four 2.5 cm ice cubes (14 g each) go in straight from the freezer. In simulation the ice is four rigid cubes of the same size and mass. The drink is 24 rigid 8 mm beads, 5.3 g each (127 g, the drink's mass), each standing for 5 ml.
- **A measurable mixing outcome:** within 1 minute of opening, 1 ml drawn from 1 cm under the surface and 1 ml drawn from the large tin's floor are each read on a handheld Brix refractometer (0 to 32 °Brix, 0.2 °Brix divisions). Mixed means the two read within 1.0 °Brix of each other. Unshaken, the top reads about 0 and the bottom is off the scale; mixed, both read about 13 to 15. The simulation does not claim to measure mixing. It scores what causes it, shaking while sealed. While sealed and tipped past 45 degrees, the shaker needs at least 6 changes of direction along its axis, each after a leg of 2.5 cm or more. The ice cubes must also reach alternate ends at least 8 times in all. A cube is at an end when its centre is within 9 cm of that end's floor and a PhysX contact force shows it touching that tin, so each count is at least 10 cm of travel into the other tin.
- **A measurable no-spill outcome:** both tins with their contents are weighed on a kitchen scale (1 g) before and after the trial, outsides wiped dry (melting ice changes nothing). 1.0 allows at most 5 g lost (simulation: at most 1 of the 24 beads out). 0.5 allows at most 15 g (3 beads). Any ice cube out of the tins scores 0. The seal must also stay closed whenever the shaker is tipped. Tipping the large tin more than 30 degrees while it is not sealed scores 0; in a real trial this is judged from the video as the tins parting or drink escaping.
- **Every tier, real and simulated:** 1.0 needs the drink mixed (simulation: shaken while sealed), at most 5 ml spilled, and the shaker opened with the large tin standing upright on the table. Opened means the small tin's rim is at least 3 cm from the large tin's mouth. 0.5 is the same with up to 15 ml spilled. 0.25 is mixed (simulation: shaken) with at most 5 ml spilled, but the shaker left sealed. Each tier states its real-trial measurement and its simulation measurement, and the numbers correspond (1 bead = 5 ml = 5 g).
- **Full-score demo for the revised setup:** the new scripted demo scores 1.0. The right hand presses the small tin home, sealing the shaker. Both hands then lift, tip and shake it: 12 legs that rock it between 65 and 107 degrees from upright and move it 3 cm along its axis. This gives 12 changes of direction and 25 ice passes, and the seal gives at most 0.6 mm (it lets go at 5 mm). They stand the shaker on the table. The left holds the large tin down while the right pulls the small tin off, breaking the seal, and holds it up over the open large tin. Nothing was spilled: 0 beads and 0 ice cubes out of the tins. A separate full-length run of the same demo also scored 1.0 (804 steps). The demo, Dream-1 and setup media are new.
- **Simulation approximations explained in the fidelity note:** it says that no liquid is simulated and that mixing is measured only in a real trial. It gives the beads' mapping (5 ml, 5.3 g each), the joint that stands for the press fit (7.2 N to seat, 12 N held 0.15 s to part), and the tins' walls (32-sided cones, 2 mm thick against a real 0.6 mm). The README adds two simulation details. The tins do not collide with each other, because the joint holds them on one axis (seated, the simulated tins keep a 0.8 mm gap, narrower than a bead). The drink and the ice do not collide with the arms, because the fingers' collision hulls reach through a gripped tin's wall.
- **Two hands, checked:** opening needs the large tin held down, because the seal holds 12 N and the full shaker weighs 0.53 kg. In the one-hand check (the right arm plays its demo part, the left stays at rest), one hand sealed the shaker and shook it, with 12 changes of direction and 20 ice passes. That held 0.25 while the sealed shaker stood on the table. Its pull then lifted the still-sealed shaker 8.7 cm off the table, and the trial ended at 0.0.
- **Kept from the previous revision:** every nonzero score still needs the shaker closed while it is tipped and shaken; the closure is now a physical seal instead of a held, loose cap. The instruction states only the goal and its constraints: "Seal the shaker and shake the drink with its ice until mixed, then open the shaker, leaving the drink in the large tin standing on the table, without spilling any." The category stays Societal relatability.
