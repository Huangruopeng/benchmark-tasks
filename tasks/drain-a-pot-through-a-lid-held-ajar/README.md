# Drain the water off a pan of dumplings into the sink, keeping every dumpling in the pan

A saucepan of dumplings in their water stands on a stove block with its lid on (a real trial uses six boiled mini
pierogies in 300 ml of warm water; the simulation stands in 20 beads of 15 ml each for the water). One arm holds the lid
slid slightly back, so that a narrow gap opens at the front rim, while the other lifts the pan by its handle and tips it
over the sink: the water runs out through the gap into the sink, the dumplings stay in. Then the pan goes back on the
stove. Tipped with the lid loose, the lid slides off and the dumplings follow the water, so the task needs both arms.
Only the water collected in the sink counts as drained; water poured anywhere else is a spill. The task is one task
file, [`drain_a_pot_through_a_lid_held_ajar.py`](drain_a_pot_through_a_lid_held_ajar.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pan (floor, wall panels and handle), the lid
(disc and knob) and each bead are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies:
importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each
body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pan and every bead still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0 by the water collected in the sink, the dumplings
left in the pan and where the pan ends; see `task.json`). `-E object_jitter=0` keeps every object exactly where the
task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos;
Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the
policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drain_a_pot_through_a_lid_held_ajar.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drain the water from the pan into the sink but keep all the dumplings in the pan, then set the pan back on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drain-dream
```

The scripted demo, which reaches full score: the right arm grips the lid's knob high up and slides the lid 2.5 cm back,
opening a gap of about 1.2 cm at the front rim, and keeps it pressed on; the left arm slides in from the left and grips
the handle with its wrist rolled so that the fingers close top and bottom (the pan's weight then turns it about an axis
the finger pads hold). Together they lift the pan 3 cm, carry it 4 cm forward and tip it 100 degrees about its front
rim over the sink, rock it a little to free the last of the water, stand it up and set it back on the stove. While the
pan is off the stove each hand aims 1 cm into the other along the pan's axis, and the left one aims the pan 10 degrees
back against the sag of its soft wrist joints, so that the lid stays where the right hand put it:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drain_a_pot_through_a_lid_held_ajar.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drain the water from the pan into the sink but keep all the dumplings in the pan, then set the pan back on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drain-demo
```

The same demo with the right arm left at rest (the left hand tips the pan alone) scores 0: the lid slides off as the
pan tips and all six dumplings fall into the sink with the water.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/drain-dream/*.json; do inspect-robots video "$log"; done
```

Set `DRAIN_DEBUG=1` to print what the score is built from (water beads in the sink, in the pan and spilled, also in
ml; dumplings kept, ever out of the pan and in the sink; how far the pan is tipped; where the lid sits on it) once a
second.

## A real trial

### The scene

Laid out as in the setup image, on a thin waterproof bench liner that covers the table under the stove and the basin
(the arm bases stay off it):

- **Stove:** a fixed round block standing in for a cold burner, 10 cm across and 13 cm tall (wood or any rigid
  material, dark grey), centred 27 cm forward of the line between the arm bases and 1 cm right of the centre line.
- **Pan:** a 12 cm stainless-steel milk pan, inside 11 cm across and 7.5 cm deep (0.7 L), with a short black handle
  pointing straight back toward the arms; 385 g. Its parts and masses are in the table below.
- **Lid:** a flat stainless-steel lid 13.6 cm across with no rim or skirt, so it overhangs the 11.16 cm rim by 1.2 cm
  all round, with a tall black knob; 119 g.
- **Sink:** a white polypropylene collecting tub with no drain, inside 16 cm square and 6.6 cm deep (1.7 L), with 4 mm
  walls and floor, centred 41 cm forward and 1 cm right of the centre line. It is a small plugged basin, smaller than
  a kitchen sink. It starts empty and dry.
- **In the pan:** six boiled mini pierogies (below) in 300 ml of tap water at 35 to 45 °C, about 4 cm deep with them
  in.

### The pan and the lid: dimensions, materials and masses

Every mass is density x volume, with 304 stainless steel at 7.93 g/cm³, aluminium at 2.70 g/cm³ and phenolic resin at
1.40 g/cm³. Lengths are in cm.

| Part | Material and size | Volume | Mass |
| --- | --- | --- | --- |
| Pan wall | stainless sheet 0.08 thick, inside radius 5.50, 7.5 tall | π (5.58² − 5.50²) × 7.5 = 20.89 cm³ | 165.6 g |
| Pan floor | stainless sheet 0.08 thick, 11.16 across | π × 5.58² × 0.08 = 7.83 cm³ | 62.1 g |
| Encapsulated base core | aluminium disc 9.5 across, 0.37 thick | π × 4.75² × 0.37 = 26.23 cm³ | 70.8 g |
| Base cap | stainless sheet 0.05 thick, 9.8 across | π × 4.9² × 0.05 = 3.77 cm³ | 29.9 g |
| Handle | solid phenolic bar 7 × 2.5 × 2.2, just under the rim | 38.50 cm³ | 53.9 g |
| Rivets | two steel rivets holding the handle | | 3 g |
| **Pan** | floor 0.08 + 0.37 + 0.05 = 0.5 thick | | **385 g** |
| Lid disc | stainless sheet 0.06 thick, 13.6 across | π × 6.8² × 0.06 = 8.72 cm³ | 69.1 g |
| Knob | solid phenolic block 2.2 front to back × 3.0 × 5.2 tall | 34.32 cm³ | 48.0 g |
| Screw | one M5 stainless screw holding the knob | | 2 g |
| **Lid** | | | **119 g** |

Any 12 cm stainless milk pan of this inside size with a short handle, and any flat 13.6 cm stainless lid with a knob of
this size, serve if the kitchen scale reads within 10% of these masses (pan 347 to 424 g, lid 107 to 131 g). For comparison, a solid steel
disc 13.6 cm across and 4 mm thick, as thick as the simulated lid, would weigh π × 6.8² × 0.4 × 7.93 = 461 g; the real
lid is 0.6 mm sheet.

In the simulation the pan and the lid have these masses (0.385 kg and 0.119 kg) and the same inside diameter, depth and
lid diameter, but the pan wall is 3 mm and the lid disc 4 mm thick, so that the contacts with the beads stay stable.
The simulated pan's rim is therefore 11.6 cm across and the lid overhangs it by 1 cm. The front gap the demo opens is
the same in both: the lid's edge is 1.3 cm outside the inner wall, so sliding the lid 2.5 cm back opens 1.2 cm. The
simulated water weighs 10 g rather than 300 g, so the hand that holds the pan lifts 385 + 78 + 10 = 473 g in the
simulation and 385 + 78 + 300 = 763 g in a real trial.

### The dumplings: Mrs. T's Mini Classic Cheddar Pierogies

Frozen potato-and-cheddar pierogies (filled dumplings), a national US brand. They come in a 28-count, 12.84 oz (364 g)
box and a 56-count, 25.6 oz family bag, sold at Walmart, Kroger, Stop & Shop, ShopRite and other grocers.

- **Listings:** the maker's page ([mrstspierogies.com, Mini Classic Cheddar](https://www.mrstspierogies.com/products/minis/minis-classic-cheddar/))
  gives "Serving Size 11 pieces (143g)" and "2.5 Servings per container"; a grocer's listing of the 28-count box
  ([Stop & Shop](https://stopandshop.com/groceries/product/mrs-ts-mini-classic-cheddar-pierogies-28-ct-frozen-12-8-oz-pkg/166763))
  gives "28 ct", "12.8 oz pkg", "Keep frozen" and the package's boil directions. So each piece weighs 143 / 11 = 13.0 g
  (364 g / 28 = 13.0 g). The listings give no piece dimensions; they call the pieces "bite-sized".
- **The package's boil directions:** "Boil 4 quarts water. Add frozen pierogies. Return to boil. Cook for 3 minutes.
  Drain & serve."

Preparation for each trial:

1. Keep the box in a freezer (−18 °C) until use. Take out 7 pierogies, still frozen: six for the trial and one spare.
2. Boil them as the package says: bring 4 quarts (3.8 l) of water to a full boil, add the 7 frozen pierogies, let it
   return to the boil and cook 3 minutes.
3. Lift them out with a slotted spoon into a colander and rinse them for 30 s under cold tap water. This cools them and
   washes off loose starch.
4. Pour 300 ml of tap water at 35 to 45 °C into the pan (mix hot and cold tap water and check it with a kitchen
   thermometer). With the slotted spoon, lower six unbroken pierogies into it, spread over the floor as in the setup
   image. The spare replaces one that split while boiling; otherwise it is thrown away.
5. Check that the water reads 35 to 45 °C, centre the lid on the pan and start the trial within 10 minutes of the end of
   the boil.

**Temperature: warm, not just off the boil.** The trial starts with the water at 35 to 45 °C, because the rig is
shared: 300 ml of near-boiling water spilled on the table could scald the person standing by the rig, and its steam
fogs the wrist cameras and condenses on the arms. The skill tested is the same at either temperature: the hand holding
the lid ajar and the hand tipping the pan face the same water, which pours freely both at 40 °C and at 95 °C (its
viscosity is 0.65 and 0.30 mPa·s). Fresh tap water rather than the starchy cooking water keeps the volume, the
temperature and the rig's cleanliness the same in every trial.

**Handling.** A fresh portion every trial. Used pierogies are thrown away, never reused or boiled again, because cooked
pierogies soften and split. A pierogi that splits during a trial counts where its larger part ends.

### Scoring a real trial

What counts as drained is the water actually collected in the sink, never the water the pan lost:

1. Lift any pierogies out of the basin with a slotted spoon, shake them over it and count them. They are lost
   dumplings and their weight never counts as water.
2. Pour the water in the basin into a measuring jug (10 ml marks), or weigh the basin with its water on a kitchen
   scale (1 g resolution) and take off its dry weight (1 g = 1 ml).
3. Count the pierogies in the pan and check that the pan stands back on the stove. On the trial video, check that no
   pierogi left the pan at any moment: one that left it and was put back is lost too.

Water on the table, the stove, the floor or the outside of the basin is spilled and earns nothing, and so does water
left in the pan.

| Score | Real trial | Simulation (each water bead = 15 ml) |
| --- | --- | --- |
| 1.0 | at least 240 ml collected in the sink, all 6 dumplings in the pan | at least 16 of the 20 water beads inside the basin, all 6 dumplings in the pan |
| 0.5 | at least 150 ml collected, at least 5 dumplings in the pan | at least 10 beads inside the basin, at least 5 dumplings in the pan |
| 0.25 | at least 75 ml collected, all 6 dumplings in the pan | at least 5 beads inside the basin, all 6 dumplings in the pan |
| 0.0 | anything less, or the pan not back on the stove (tilted 10 degrees or more, or its centre more than 4 cm off) | the same |

In simulation a water bead is collected only if its centre ends inside the basin: inside its four walls, above its
floor and below its rim. Dumplings that end in the sink are counted separately and are lost dumplings, not water. A
dumpling whose centre is outside the pan's inside at any control step is lost for the rest of the trial, even if it
is put back, so tipping everything into the sink and picking the dumplings back into the pan earns nothing (an offline
scorer check of that end state gives 0.0; the previous scorer gave it 1.0).

### Spill check

Pouring the water anywhere but into the sink earns no drain credit. The same scripted demo, altered to carry the pan
14.5 cm to the left of the sink and tip it there (the water lands on the table beside the sink), scores 0.0: 19 of the
20 water beads (285 ml) are spilled on the table and 1 (15 ml) bounces into the sink, far short of the 75 ml of the
lowest tier, while all 6 dumplings stay in the pan (in that run the pan also did not end back on the stove). A scorer
check with the pan standing on the stove and all 20 beads placed on the table beside the sink, on the table in front of
it, or on top of its wall likewise gives 0 ml drained and a score of 0.0, while the same beads inside the basin give
300 ml and 1.0.

## Real-trial safety and rig protection

- **No hot water, no steam.** The water and the pierogies are at 35 to 45 °C when the trial starts, and the stove is
  cold, so nobody can be scalded, nothing near the arms is hot and no steam reaches the wrist cameras.
- **Pads stay dry and clean.** The fingers touch only the pan's dry handle and the lid's dry knob. The water leaves
  through the gap at the front rim, below and ahead of both hands. The pierogies are rinsed and the water is fresh tap
  water, so no starch or grease reaches the rig. Pad covers are not needed, and would change the grip on the handle
  and the knob. If a pad does get wet, dry it with a lint-free cloth before the next trial. At the end of every session,
  wipe each pad with a cloth wrung out in clean water and dry it.
- **Spills.** The bench liner under the stove and the basin catches any spill, and the arm bases stand off it. At each
  reset the table, the liner and the stove are dried.
- **Food.** The pierogies stay frozen until they are boiled, are handled with a slotted spoon, and are thrown away after
  the trial. Nobody eats them.
- **Load and force.** The hand holding the pan lifts at most 763 g (7.5 N): the 385 g pan, 300 ml of water and six 13 g
  pierogies. The other hand holds the 119 g lid (1.2 N). These are light loads for fingers that manage roughly 40 N
  each, and the pan's weight rests on the lower finger of a top-and-bottom grip rather than on friction alone.
- **Nothing sharp or breakable.** The pan's rim and the lid's edge are rolled or deburred, the tub is moulded plastic,
  and nothing is glass.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,200 steps (628 s on the clock, with Dream answering in 0.68 s on average), and about 6 minutes for the scripted demo (616 steps, 30.8 s of simulated time, 376 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pan, the lid and the beads) shift.

## Assets and licences

None from third parties: every shape in the scene is a primitive (box, cylinder or sphere) built in the task file.

## Changes in this revision

This is a revision of TK-6ZKXXF. After our reply to its first decision, its review said:

> We looked at this task again after your reply, and it can be revised.
>
> What to change: The relatability objection is withdrawn. Two practical issues not raised in the earlier reviews still need correction: specify a repeatable dumpling type, preparation, temperature and handling procedure for each trial; and give physically consistent dimensions, materials and masses for the pan and lid. For example, the specified 13.6 cm steel lid disc alone would weigh about 0.46 kg, not 0.08 kg. Update the simulation or fidelity note to reflect any resulting differences from the real equipment.

The first decision on TK-6ZKXXF (not relatable enough for Societal relatability) was withdrawn by that review; the category is unchanged. The review before that (TK-9GWQ55) asked us to measure the water actually collected in the sink, exclude spills and keep lost dumplings out of the measured water; that was done in the previous revision and is kept. The first review (TK-ML6LPR) asked for real water and representative dumplings, a stated starting volume, measurable drained-volume thresholds and a check of the demo's final state; that was done and is kept (300 ml of water; 240, 150 and 75 ml tiers).

What changed in response:

- **A repeatable dumpling type.** The dumplings are now one named frozen product: Mrs. T's Mini Classic Cheddar Pierogies (potato-and-cheddar dumplings), a national US brand sold as a 28-count, 12.84 oz box and a 56-count family bag (Walmart, Kroger, Stop & Shop, ShopRite and others). The maker's page gives "Serving Size 11 pieces (143g)", so each weighs 13.0 g (364 g / 28 agrees); the listings give no piece dimensions ("bite-sized"). The README links the maker's page and a grocer's listing of the box.
- **Preparation.** As the package says: "Boil 4 quarts water. Add frozen pierogies. Return to boil. Cook for 3 minutes. Drain & serve." Seven pieces (six and a spare) go in straight from the freezer; they are then rinsed for 30 s under cold tap water and lowered with a slotted spoon into the pan's 300 ml of fresh tap water, spread over its floor.
- **Temperature.** The trial starts with the water at 35 to 45 °C, checked with a kitchen thermometer, within 10 minutes of the boil. Not just off the boil, because the rig is shared: near-boiling water could scald the person at the rig if spilled, and its steam fogs the wrist cameras and condenses on the arms. The skill is the same, since water pours freely at either temperature (viscosity 0.65 mPa·s at 40 °C, 0.30 at 95 °C). Fresh tap water rather than the starchy cooking water keeps the volume, temperature and rig the same in every trial.
- **Handling procedure for each trial.** A fresh portion of six every trial, handled only with a slotted spoon; the spare replaces one that split while boiling. Used pierogies are thrown away, never reused or reboiled. A pierogi that splits during the trial counts where its larger part ends. The reset notes give the whole sequence.
- **Physically consistent pan and lid.** Every mass is now density x volume (steel 7.93, aluminium 2.70, phenolic 1.40 g/cm³), with the arithmetic in a README table. Pan, 385 g: a 0.8 mm stainless wall, inside 11 cm across and 7.5 cm deep (165.6 g); a 5 mm encapsulated floor of 0.8 mm steel (62.1 g), a 9.5 cm x 3.7 mm aluminium disc (70.8 g) and a 0.5 mm steel cap (29.9 g); a solid phenolic handle 7 x 2.5 x 2.2 cm (53.9 g); two rivets (3 g). Lid, 119 g: a flat 0.6 mm stainless disc 13.6 cm across (69.1 g), a solid phenolic knob 2.2 x 3 x 5.2 cm (48.0 g) and an M5 screw (2 g). As the review says, a 4 mm steel disc of that size would be 461 g; the earlier text gave the simulation's thicknesses as if they were the real ones.
- **Simulation and fidelity note.** The simulation now uses the real masses: the pan went from 0.40 to 0.385 kg, the lid from 0.08 to 0.119 kg and each dumpling from 8 to 13 g. The fidelity note states what still differs: the simulated pan wall and lid disc are 3 and 4 mm thick (real 0.8 and 0.6 mm) for stable contacts, with the real masses and the real inside and lid diameters; the 20 water beads weigh 10 g, not 300 g, so the hand holding the pan lifts 0.47 kg in simulation and 0.76 kg in a real trial; the pierogies are rigid 24 mm balls of their real 13 g; temperature is not simulated.
- **Rig safety.** A new "Real-trial safety and rig protection" section covers the warm water, dry pads (the fingers touch only the dry handle and knob), the rinsed food, a bench liner for spills, food handling and the loads (763 g in the hand holding the pan, 119 g in the other).
- **Also closed: putting dumplings back.** A dumpling that leaves the pan at any time is now lost even if it is put back (a real trial checks this on the trial video; the simulation remembers any dumpling whose centre was ever outside the pan's inside). Before, tipping everything into the sink and picking the dumplings back into the pan would have scored 1.0; an offline scorer check of that end state now gives 0.0, and one dumpling out and back gives 0.5 at most.
- **Re-run with the new masses and scorer.** The scripted demo scores 1.0: all 20 water beads (300 ml) end in the sink, all 6 dumplings stay in the pan and none ever leaves it, and the pan stands back on the stove, tilted 4 degrees. With the right arm left at rest it scores 0 (the lid slides off and all 6 dumplings fall into the sink with the water). The beside-the-sink spill check scores 0.0: 19 of the 20 beads (285 ml) are spilled on the table and 1 (15 ml) lands in the sink. Both checks were run with the new masses before the put-back rule was added; that rule can only lower a score. Dream-1 scores 0.0 over 1,200 steps: it never lifts the pan. New demo and Dream-1 videos.
