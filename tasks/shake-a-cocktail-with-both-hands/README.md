# Shake a drink with ice in a sealed Boston shaker and open it without spilling

A two-piece Boston shaker stands on the table: the large tin holding the drink and four 2.5 cm ice cubes, with the
small tin set straight into its mouth, resting, not yet sealed. Both tins are a real pair, and the simulation uses
its measured sizes, masses, depths and forces. The robot seals the shaker (presses the small tin home, so its rim jams in the
large tin's taper), shakes it so the ice crashes from end to end, sets it down, breaks the seal (a strike at the seam
lets in air, as bartenders do, and the tins are pulled apart) and lifts the small tin off, leaving the drink in the
large tin standing on the table without spilling any. Only shaking with the shaker sealed earns credit: a stroke counts
only while the small tin is seated at its sealed depth, at any tilt, and every nonzero score needs at least 6 such
strokes. Strokes, swirls or rocking with the shaker unsealed count for nothing, and moving the open drink scores 0. In
a real trial the drink is water with a layer of dyed syrup under it, and the score measures that it was mixed
(refractometer readings of the top and the bottom), that nothing was spilled (kitchen scale), and, frame by frame in
two side videos, that it was mixed only by shaking with the shaker sealed. The task is one task file,
[`shake_a_cocktail_with_both_hands.py`](shake_a_cocktail_with_both_hands.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses no third-party assets: every shape is
built in the task file.

`yam_arena` task files place single boxes, cylinders and spheres. The tins (32-sided cones of steel panels with a
floor), the ice cubes and the drink's beads are loose rigid bodies, so `bodykit.py` adds such bodies: importing it
hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()` each body's own pose,
and adds the joint that is the tins' press fit.

## Measured on the real pair

The shaker is a **Barfly M37009CP** copper-plated Boston shaker set (Mercer Culinary): a 28 oz and an 18 oz tin of
18/8 stainless steel with a copper-plated outside, sold as a pair
([WebstaurantStore item 470M37009CP](https://www.webstaurantstore.com/mercer-culinary-m37009cp-barfly-28-oz-18-oz-2-piece-copper-plated-shaker-tin-set/470M37009CP.html),
31.49 USD). Every number in this table was measured on that pair, clean and dry, at room temperature, with a digital
caliper (inch mode), a digital kitchen scale and a digital luggage scale. The photos are in [`measurement/`](measurement);
the only edit is a blur over the printed lettering on the marker ribbon (photos 07, 08, 10), for blind review.

| reading | value | instrument and method | photo |
|---|---|---|---|
| large tin mass | 228 g | kitchen scale, empty tin | [01](measurement/01_large_tin_mass_228g.jpg) |
| small tin mass | 192 g | kitchen scale, empty tin | [02](measurement/02_small_tin_mass_192g.jpg) |
| large tin, across the mouth | 91.44 mm (3.60 in) | caliper, outside | [03](measurement/03_large_tin_top_od_3.60in.jpg) |
| large tin, across the bottom | 68.07 mm (2.68 in) | caliper, outside | [04](measurement/04_large_tin_bottom_od_2.68in.jpg) |
| small tin, across the rim | 88.90 mm (3.50 in) | caliper, outside | [05](measurement/05_small_tin_top_od_3.50in.jpg) |
| small tin, across the bottom | 67.82 mm (2.67 in) | caliper, outside | [06](measurement/06_small_tin_bottom_od_2.67in.jpg) |
| heights | 178 mm and 127 mm | caliper | |
| **loose depth** | **33.78 mm** | small tin set straight into the large tin's mouth, not pressed; a band round the small tin marks the mouth, and the caliper reads from the band to the small tin's rim | |
| **sealed depth** | **42.16 mm** (1.66 in) | the same, after pressing the small tin in: 8.38 mm deeper | [07](measurement/07_sealed_pair_with_depth_band.jpg), [08](measurement/08_sealed_depth_band_to_rim_1.66in.jpg) |
| **seating force** | **14.90 N** (1,519 g) | large tin on the tared kitchen scale, small tin pressed in by hand until seated | [09](measurement/09_seating_force_1519g_14.90N.jpg) |
| **release force** | **15.69 N** (1.6 kg) | luggage scale, the sealed, unshaken pair pulled apart along its axis | [10](measurement/10_release_pull_luggage_scale.jpg) |

**What follows from the readings** (computed):

- **Wall:** 228 g of steel (7.93 g/cm3) over the large tin's outside area is 0.59 mm thick, so the large tin is
  90.25 mm across inside at the mouth.
- **Seal zone:** the small tin's 88.90 mm rim rests 33.78 mm down, where the large tin is therefore 88.90 mm across
  inside: its upper band narrows 0.0400 mm of diameter per mm of depth (the wall leans 1.1 degrees). Pressed 8.38 mm
  further, to 42.16 mm, the rim is 0.335 mm tight in that taper: the press fit.
- **Shaken:** shaking on ice chills the sealed air; the published vacuum after a 15 s shake on ice, 1.37 psi
  (9.45 kPa, [Cooking Issues, 2009](https://www.cookingissues.com/index.html%3Fp=1656.html)), pulls on the 88.57 mm seal
  circle (6,160 mm2) with 58 N, so a shaken seal holds 15.69 + 58 = 74 N.
- **Release:** a tap of 10 N or more where the tins meet (the large tin's top 25 mm) lets the air in (design value;
  the standard bar method, [Difford's Guide](https://www.diffordsguide.com/g/1150/how-to-make-cocktails/how-to-use-a-two-piece-cocktail-shaker):
  "tapping the base can ... at the point where it meets the upper can"); after that, the measured 15.69 N parts the
  tins, more than the full shaker's 5.9 N weight.
- **Seal line:** resting, a line drawn round the small tin at the mouth of the seated pair sits 8.38 mm above the
  mouth.

**One-time setup check, before the first session** (a caliper with a depth rod, at three points round the mouth):
set in, the empty small tin's rim rests 33.8 mm below the mouth and lifts off alone; pressed in with about 15 N (a
firm palm press) it reads 42.2 mm, each reading within 1.5 mm, and lifted by the small tin the seated pair brings the
empty large tin with it. Still seated, draw a fine marker line round the small tin at the large tin's mouth: the seal
line.

## What the simulation uses

The simulated fit uses the measured numbers: resting at 33.78 mm, sealed at 42.16 mm, seated by 14.9 N, parted by
15.69 N, plus the 58 N vacuum once shaken.

- **The tins:** 32-sided at the measured diameters and masses, with 2 mm walls and 5 mm floors (real 0.59 mm; thinner
  walls let fast ice through). The large tin's inside is the computed profile above: the seal zone from the mouth to
  50 mm deep (design value for its length), then a straight cone to the bottom. The large tin's wall is thickened
  outward and the small tin's inward, so the surfaces that meet at the seal are at their real size: at 33.78 mm deep
  the simulated rim just touches the large tin's inside, and at 42.16 mm it overlaps it by the 0.335 mm interference.
  Centres of mass where the sheet puts them. For solver stability the ice and the beads never move faster than
  4 m/s (shaken, they reach about 1.5 m/s).
- **Sealed:** the fit is a slide joint along the tins' axis whose coordinate is the rim's depth (the two tins do not
  collide with each other; the joint keeps the small tin on the axis). Off the wall the small tin slides freely. On
  it, a spring of 1.78 N per mm (the measured 14.9 N over the measured 8.38 mm) resists pushing it further in; at
  42.16 mm it is sealed and held at that seat by a stiffer spring that gives 0.5 mm under the measured 15.69 N
  release pull (design value). A sealed tin comes out only when pulled out of its seat past its hold for 0.15 s.
- **Vacuum and strike:** the vacuum grows as the shaking chills the air: each sealed stroke adds a sixth of its 58 N,
  so after the 6 needed for mixing the seal holds 74 N. A push or blow of at least 10 N across the tin's axis, for 2
  control steps (0.1 s), from either arm on the large tin's wall in its top 25 mm, read from PhysX contact forces,
  points and normals, lets the air in: the vacuum is gone, and builds again only with more shaking.

## The real trial

- **Shaker:** the measured pair above, checked once as above, with its seal line.
- **Drink:** 90 ml of room-temperature water, then 30 ml of simple syrup (sugar and water 1:1 by weight, about
  50 °Brix, dyed blue) poured in slowly down the side so that it lies as a layer under the water: 120 ml, 127 g.
  Four 2.5 cm ice cubes (14 g each, from a standard tray) straight from a -18 °C freezer go in on top. The small tin is
  set straight into the mouth, resting (its seal line 8.4 mm above the mouth).
- **Video:** besides the robot's own three cameras, two fixed cameras record the trial (phones on tripods:
  3840 x 2160 at 30 frames per second, the 2x or 3x lens framing about 50 cm across the shaker's working space, so
  1 mm is about 8 pixels), about 15 cm above the table and 1 m from the shaker's spot, at right angles to each other
  (one facing the robot, one from its side), so the shaker is side-on in at least one of them whichever way it is
  shaken. The judge steps through the frames; lengths are read against the tins (the large tin is 177.8 mm tall and
  92.1 mm across its mouth, so 2.5 cm is a seventh of its height and 3 cm a third of its mouth).
  - **Sealed, in a frame:** the small tin's rim is inside the large tin, no gap shows between the tins at the seam in
    the side-on view, and the seal line is level with the large tin's mouth edge (at most 2.0 mm above it, about
    16 pixels; resting unsealed it shows 8.4 mm above, about 67 pixels). The tins have parted once the line rises 3 mm
    or more, a gap opens at the seam, or drink escapes there.
  - **A sealed stroke:** a movement of the shaker along its own axis of at least 2.5 cm that ends in a change of
    direction, at any tilt, with the shaker sealed in every frame of it and no drink escaping at the seam. Only sealed
    strokes are counted. A stroke with the shaker unsealed in any of its frames, a swirl or rock of the open tin, or
    shaking with the small tin resting adds nothing. **At least 6 sealed strokes** are needed for any score; fewer
    (none at all, or a single tilt) scores 0.
  - **The open drink stays still:** while the shaker is not sealed (before the robot's press, and after the tins
    part), the large tin's mouth must move less than 3 cm in all (adding up every movement of 0.5 cm or more, frame by
    frame) and never tip more than 30 degrees. Moving the open tin more than that, at any angle (including within
    30 degrees of upright), scores 0 at every tier, whatever the refractometer reads.
  - **Opened:** at the end the small tin is off (its rim at least 3 cm from the large tin's mouth) and the large tin
    stands upright on the table (tilted less than 10 degrees).
- **No spill:** both tins with the drink and the ice are weighed on a kitchen scale (1 g) before the trial; after it,
  both tins, wherever they ended, are wiped dry outside and weighed again with whatever is in them. The loss in grams
  is the millilitres spilled (melting ice changes nothing).
- **Mixed:** within 1 minute of the shaker being opened (by a person, if the robot did not open it), 1 ml is drawn
  with a pipette from 1 cm under the drink's surface and 1 ml from the large tin's floor, and each is read on a
  handheld Brix refractometer (0 to 32 °Brix, 0.2 °Brix divisions). Mixed means the two read within 1.0 °Brix of each
  other. Unshaken, the top reads about 0 and the bottom is off the top of the scale (counted as unmixed); mixed, both
  read about 13 to 15.

## How the simulation measures the same

No liquid is simulated, so the simulation cannot measure mixing; it scores what the robot controls, which is what
mixes the drink in a real trial: strokes made with the shaker sealed that send the ice from end to end, and no movement
of the drink while the shaker is open.

- **The seal gate:** a stroke or an ice pass counts only while the fit's joint is engaged, the small tin has been
  pressed to its 42.16 mm sealed depth and not since pulled out past its hold, and its rim is still 40.16 mm deep or
  more (within 2.0 mm of its sealed depth; the real trial's "line at most 2.0 mm above the mouth"). Any control step
  outside the gate ends the stroke under way, so only strokes made wholly while sealed count, at any tilt.
- **Shaken:** at least 6 changes of direction of the shaker's middle along its own axis, each after a leg of 2.5 cm or
  more, and the ice cubes reaching alternate ends at least 8 times in all, all inside the seal gate. A cube is at an
  end when its centre is within 9 cm of that end's floor and it touches that tin (a PhysX contact force), so each pass
  is a cube carried at least 9.7 cm along the shaker into the other tin.
- **The open drink stays still:** while the shaker is not sealed, the large tin's mouth (the centre of its opening)
  is followed every control step, and each time it is 5 mm or more from where it was last counted, that distance is
  added. At 3 cm in all the trial scores 0, as does tipping the large tin past 30 degrees while not sealed.
- **Spill:** the drink is 24 rigid beads, 8 mm and 5.3 g each (127 g, the drink's mass), each standing for 5 ml. A
  bead or a cube whose centre is outside both tins is spilled.
- **Open:** the small tin off, its rim at least 3 cm from the large tin's mouth, with the large tin standing upright
  on the table (tilted less than 10 degrees).

The beads and the ice do not collide with the arms: the YAM fingers' collision hulls reach through a gripped tin's
wall and would push what is inside out through it.

## Scoring, real trial and simulation

Judged at the end of the trial, except the sealed-stroke count and the two open-drink rules, which cover the whole
trial.

- **0 at every tier** unless the shaker was shaken sealed: at least 6 sealed strokes (real trial: counted frame by
  frame in the side videos; simulation: inside the seal gate, plus 8 ice passes). Also 0 if the drink was moved while
  the shaker was not sealed (the mouth moving 3 cm or more in all, or the large tin tipped past 30 degrees), if any
  ice cube ends outside the tins, or if the large tin does not end standing upright on the table (tilted less than
  10 degrees).
- **1.0:** shaken sealed, mixed, at most 5 ml spilled, and the shaker opened. Real trial: the refractometer's top and
  bottom readings within 1.0 °Brix, and at most 5 g lost on the scale. Simulation: at most 1 of the 24 beads out of
  the tins.
- **0.5:** the same with up to 15 ml spilled (real trial: at most 15 g lost; simulation: at most 3 beads out).
- **0.25:** shaken sealed and mixed (simulation: shaken sealed), with at most 5 ml spilled, but the shaker left sealed.
- **Else 0.**

The trial ends at the run limit, or once the full score has held with everything still for 1 s.

## Checks of the scoring rules

Each is a copy of the scripted demo with one part changed, run in simulation and scored by the task's scorer
(`SHAKE_DEBUG=1` also prints what each gate counted, and the score without the gates):

- **(a) Swirl the open tin upright.** The right lifts the resting small tin off (never sealed) and holds it aside;
  the left lifts the open large tin, with the drink and ice in it, 1.5 cm, swirls it through three laps of a 1.5 cm
  circle, rocks it 20 degrees each way and sets it down (at most 12.9 degrees from upright). It ends opened, upright,
  with nothing spilled. **Score 0.0**: 0 sealed strokes and 0 sealed ice passes (the open-drink rule also tripped;
  with that rule off it still scores 0.0).
- **(b) Seal, then one slow tilt.** The demo's press seals the shaker (seated 27.1 s); both hands lift it and tip it
  slowly to 95.6 degrees and back over 13 s, set it down, and the demo's strike and pull open it, upright, with
  nothing spilled. **Score 0.0**: 0 sealed strokes and 0 sealed ice passes, short of the 6 and 8 needed.
- **(c) Shake without sealing.** The right never presses and holds the small tin 10 mm above its resting depth,
  neither hand squeezes, and both hands make the demo's whole shake (12 legs of 3 cm), then the strike and pull.
  Sealing is switched off in the check's copy of the task file (the sealed depth made unreachable), so the whole
  shake is made unsealed: 12 strokes and 28 ice passes, opened, nothing spilled. **Score 0.0**: 0 sealed strokes; the
  seal gate alone gives 0.0, and with every gate off the run would score 1.0.
- **(d) Swirl the open shaker first, then the full sealed demo.** Before the press, both hands lift the unsealed
  shaker 2 cm and swirl, shake and rock it upright (at most 14.8 degrees from upright); the full demo follows (13
  sealed strokes, 36 sealed ice passes, opened, nothing spilled). **Score 0.0**: the open-drink rule tripped at 9.5 s
  (37.2 mm of mouth travel); with that rule off the run would score 1.0.

## Why two hands

The sealed tins hold together until they are pulled apart: 15.69 N for the press fit alone (measured), and 74 N once
the shaking has chilled the air inside, until a strike or press at the seam lets the air in. The full shaker weighs
5.9 N, so one hand pulling the small tin lifts the whole shaker off the table: the large tin has to be held down by
the other hand. The strike needs the shaker held as well: standing alone, it tips over under a side push of 1.2 N at
the seam (its 5.9 N weight on its base's 34.0 mm radius, against a push 165 mm up). In a one-hand check of this version (the right arm plays its demo part, the left stays at its rest pose), the right sealed and shook the shaker (14 sealed strokes, 18 sealed ice passes), but alone it could not open it: its strike shoved the unheld shaker aside (a 13.3 N contact, too brief to let the air in), and its pull lifted the whole sealed shaker 95 mm off the table: 0.0. With both hands and the strike stopped 2 cm short of the tin, the vacuum held through the right's 10 cm pull (drawn 0.80 mm of its 2.35 mm give), which only lifted the sealed shaker 4.4 mm against the left's hold and tipped it 6.9 degrees: 0.25, shaken sealed but not opened. Bartenders also shake a Boston
shaker with a hand on each tin ("one hand on the top and the other gripping the base", in the guide cited above), so a
seal knocked by the ice cannot open.

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
upright, moving it 3 cm along its axis each way with the sliding ice, with a short pause at each end, so the ice slides
from one end to the other and hits it. They stand it up and set it down. The left holds the large tin down, upright;
the right lets go and strikes the large tin's seam from the side with its closed fingertips, about 15 mm below the
mouth, pushing toward the left hand, which lets the air in. It grips the small tin again and pulls it 10 cm straight
up, breaking the seal, and holds it up over the open large tin (anything wedged in the small tin can only drop back
in), while the left keeps holding the open large tin on the table (letting go drags it as the soft arm springs back,
and the open drink must not be moved). In a debug run (`SHAKE_DEBUG=1`) the press reached 15.5 N at the seat; the shake gave 13 sealed strokes and 42 ice passes, every one inside the seal gate (the small tin seated for 31.4 s, the seal giving at most 0.25 mm of the 2.0 mm the gate allows); the strike vented the seal at 13.5 N and peaked at 19.9 N; the pull drew the seal 0.62 mm (its unshaken hold, the measured 15.69 N, lets go at 0.5 mm) and it let go; the open drink moved 6.8 mm in all, under the 3 cm limit; nothing was spilled (0 beads, 0 ice cubes), and the run ended at full score after 943 steps. The YAM fingers' collision shapes are not quite parallel, so a gripped
tin leans a few degrees in the hand; the demo turns each wrist to match:

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

Every number above comes from the constants in the task file; this prints them:

```bash
python -c "import shake_a_cocktail_with_both_hands as m; m.print_numbers()"
```

Set `SHAKE_DEBUG=1` to print what the score is built from twice a second (the small tin's depth in the mouth, whether
it is sealed and seated, the vacuum's share and the hold, any blow on the seam, how far the shaker is tipped, the
sealed strokes and ice passes so far and those made at all, how far the open drink has moved, and anything outside the
tins).

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (516 s on the clock, with Dream answering in 0.67 s on average), and about 8 minutes for the scripted demo (943 steps, 47.1 s of simulated time, 450 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is a fixed list of waypoints. Keep
the jitter at 0: it shifts each free object on its own, and the two tins start on one axis, joined by the fit's joint.

## Changes in this revision

This is a revision of TK-ELFLBH. Its review said:

> Flags: Real tin fit and seal forces are calculated from assumed dimensions and material behavior, not reported measurements.
>
> Construct validity (not met): A robot could mix the drink by swirling the open tin upright and receive credit without sealing the shaker.
>
> Scoring (not met): The real-trial tiers test mixture and spillage but do not require a seal during mixing when the tin stays within 30 degrees of upright.
>
> What to change: Measure and report the loose and sealed depths and the seating and release behavior of the specified real tin pair; update the simulated fit and release behavior to match those measurements and provide a full-score demo for any corrected setup. Also revise the real-trial scoring so every nonzero score requires mixing by shaking with the shaker sealed, including when the tin stays within 30 degrees of upright; define how the video verifies this.

Earlier reviews, each done and kept: TK-PXNFCR (a real tin pair and a starting alignment that give the stated loose and
sealed positions, a simulation matched to it, a full-score demo); TK-JE4KUU (a realistic shaker with water and ice, a
measurable mixing and no-spill outcome, a full-score demo, the simulation approximations in the fidelity note);
TK-CCNDMT (the shaker closed during the tipping and shaking for every nonzero score, a demonstrated full score, the
Societal relatability category).

What changed in response:

- **Flag and request: the loose and sealed depths, seating and release of the real pair, measured and reported.**
  Where: "Measured on the real pair" above (readings, instruments and photos), `setup`, and the task file's FIT
  SPECIFICATION block. The task now names the pair it measured, a Barfly M37009CP copper-plated Boston shaker set:
  heights 178 and 127 mm, masses 228 and 192 g, mouth and bottom 91.44 and 68.07 mm, rim and bottom 88.90 and
  67.82 mm (digital caliper, digital kitchen scale); loose depth 33.78 mm and sealed depth 42.16 mm (caliper, from a
  band at the mouth to the rim); seating force 14.90 N (tared kitchen scale); release force 15.69 N (luggage scale,
  axial pull). A one-time setup check with a caliper depth rod confirms each pair before use.
- **Simulated fit and release matched to the measurements; full-score demo.** Where: the FIT SPECIFICATION block,
  `fidelity`, "What the simulation uses". The slide joint rests at 33.78 mm, seats at 42.16 mm under 14.9 N, and
  holds the measured 15.69 N unshaken (74 N with the published shaken vacuum, until a 10 N tap at the seam vents it).
  The large tin's inside profile is computed from these readings (a 0.0400 mm per mm seal zone meeting the 88.90 mm
  rim at 33.78 mm, 0.335 mm interference at 42.16 mm). Check: the demo scores 1.0 (943 steps), 13 sealed strokes and 42 sealed ice passes; vented, opened, nothing spilled.
- **Construct validity: swirling the open tin upright.** Where: `scoring`, `score()` and the seal gate in `_follow()`.
  Credit accrues only through sealed shaking: a stroke or an ice pass counts only while the fit's joint is engaged,
  the small tin has been pressed to its 42.16 mm sealed depth and its rim is still within 2.0 mm of it, at any tilt;
  any step outside the gate ends the stroke under way. Every nonzero tier needs at least 6 sealed strokes and 8
  sealed ice passes; moving the open drink 3 cm in all, or tipping it past 30 degrees, scores 0. Checks: (a) the open
  large tin swirled and rocked upright with the drink and ice in it, 0.0; (d) the open shaker swirled, shaken and
  rocked upright before the full sealed demo, 0.0 (1.0 without the open-drink rule).
- **Scoring: a seal during mixing, including within 30 degrees of upright.** Where: `scoring`, "Scoring, real trial
  and simulation" and "The real trial". The earlier 45-degree condition on strokes is gone: sealed strokes count at
  any tilt, and strokes, swirls or rocking with the shaker unsealed count for nothing at any tilt. Fewer than 6 sealed
  strokes scores 0 at every tier, in the real trial and in simulation. Checks: (b) sealed, then one slow tilt and
  back, 0.0; (c) the demo's whole shake with the shaker never sealed, 0.0 (1.0 with every gate off).
- **How the video verifies it.** Where: `scoring` and "The real trial". Two fixed 4K phone cameras at right angles,
  1 mm about 8 pixels, judged frame by frame: sealed in a frame means the rim inside the large tin, no gap at the
  seam, and the seal line at most 2.0 mm above the mouth edge (resting unsealed it is 8.4 mm above, the measured
  loose-to-sealed travel); a stroke counts only if the shaker is sealed in every frame of it and no drink escapes at
  the seam; 0 to 5 sealed strokes scores 0.
- **Two hands, re-checked on this version:** one hand sealed and shook the shaker but could not open it (its strike shoved the unheld shaker aside, and its pull lifted the whole sealed shaker): 0.0; with the strike stopped 2 cm short, the vacuum held through the 10 cm pull: 0.25, shaken sealed but not opened.
