# Shake a drink with ice in a sealed Boston shaker and open it without spilling

A two-piece Boston shaker stands on the table: a 28 oz stainless steel shaker tin holding the drink and four 2.5 cm
ice cubes, with the matching 18 oz tin set straight into its mouth, not yet sealed. Both tins are a real catalogue pair
at their listed sizes. The robot seals the shaker (presses the small tin home, so its rim jams in the large tin's
taper), shakes it so the ice crashes from end to end with the shaker sealed the whole time it is tipped, sets it down,
breaks the seal (a strike at the seam lets in air, as bartenders do, and the tins are pulled apart) and lifts the small
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

- **Shaker:** Acopa 28 oz full-size and 18 oz half-size stainless steel shaker tins (WebstaurantStore items
  176SHKTSL28 and 176SHKTSL18; each listing names the other as the tin it works with), 18/8 stainless steel, at their
  listed sizes (outside):
  - large tin: 7 in (177.8 mm) tall, 3 5/8 in (92.1 mm) across the top, 2 1/2 in (63.5 mm) across the bottom,
    shipping weight 0.41 lb (186 g);
  - small tin: 5 1/4 in (133.4 mm) tall, 3 7/16 in (87.3 mm) across the top (its rim), 2 3/8 in (60.3 mm) across the
    bottom, shipping weight 0.32 lb (145 g).
  The wall thickness is not listed; 0.5 mm fits the listings: 0.5 mm steel (7.93 g/cm3) over the large tin's listed
  shape weighs 185 g, and its inside is then 830 ml brim-full, the 28 oz (828 ml) it is sold as.
- **Where the tins meet, from those sizes:** the large tin's inside narrows straight from 91.1 mm at the mouth to
  62.5 mm at the bottom, 0.161 mm of diameter per mm of depth (its wall leans 4.6 degrees). Set straight in (mouth
  down, on the same axis), the small tin sinks until its 87.3 mm rim meets the wall all round:
  (91.075 - 87.3125) / 0.1607 = 23.4 mm below the mouth. Resting there it is not sealed: its 1.42 N weight wedges it
  only lightly, and it lifts off with 0.8 N more. Pressed 1.0 mm further in, to 24.4 mm, where the large tin is 87.15 mm
  across inside, the rim is 0.16 mm tight in the taper (the two thin walls flex to take it up): sealed. Bartenders
  often set the small tin in at a slant instead, so that one side is flush; straight in, as here, the rim meets the
  wall all round.
- **How hard the seal holds:** pressing the rim home takes about 15 N. In the 4.6 degree taper, with a friction
  coefficient of 0.3 (wet steel on steel), the press fit then holds 15 x (0.3 cos 4.6 - sin 4.6) / (0.3 cos 4.6 +
  sin 4.6) = 8.7 N, and it is self-locking (tan 4.6 degrees = 0.080, below 0.3), so it stays sealed when let go.
  Shaking adds a partial vacuum: the ice chills the air sealed inside from 20 C to 0 C, so it falls
  101.3 x 20 / 293 = 6.9 kPa (1.0 psi) below the air outside, which presses the tins together with another
  6.9 kPa x 0.0060 m2 (the 87.15 mm seal circle) = 41 N. (A published test of this effect, on the Cooking Issues
  blog in 2010, measured 1.37 psi in a sealed jar after shaking 100 ml of vodka with ice for 15 s; vodka chills
  below 0 C, to about -5.7 C in its worked example. It is why shaken tins stick.) Sealed and shaken, the shaker
  holds 8.7 + 41 = 50 N against a pull, more than a YAM arm pulls. Bartenders break the vacuum with a strike or a
  firm press on the large tin at the seam: the 25 mm of thin wall between the mouth and the seal flexes out of round
  and lets air past the rim (as a ring, 0.5 mm steel 25 mm wide and 45 mm in radius bends 0.149 x 45^3 /
  (193,000 x 25 x 0.5^3 / 12) = 0.27 mm per newton across, more than the 0.16 mm interference). With the air in, a
  pull of 8.7 N parts the tins, which is still more than the full shaker's 5.0 N weight.
- **Checking a real pair:** before the trials, set the small tin straight in and measure its rim's depth below the
  large tin's mouth with a caliper's depth rod: about 23.4 mm resting, and about 1 mm more when pressed home. The
  listed diameters are rounded to 1/16 in, and each 0.1 mm of diameter moves the resting depth 0.6 mm, so a real pair
  can sit several mm higher or lower; the scoring does not depend on the depth.
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

- **The tins:** 32-sided cones at the listed sizes, with 2 mm walls and 5 mm floors (real 0.5 mm; thinner walls let
  fast ice through). The large tin's wall is thickened outward and the small tin's inward, so the surfaces that meet
  at the seal are at their real size: at 23.4 mm deep the simulated rim just touches the large tin's inside, and at
  24.4 mm it overlaps it by 0.08 mm a side, the 0.16 mm interference. Masses 186 g and 145 g, centres of mass where
  the steel sheet puts them.
- **Sealed:** the fit is a slide joint along the tins' axis whose coordinate is the rim's depth (the two tins do not
  collide with each other; the joint keeps the small tin on the axis). Off the wall (more than 0.05 mm shallower than
  23.4 mm) the small tin slides freely. On it, a spring of 15 N per mm resists pushing it further in, so it rests at
  first contact and lifts off against 0.75 N (the real wedge: 0.8 N); pushed 1.0 mm in, to 24.4 mm (15 N), it is
  sealed and held at that seat by the same spring. A sealed tin comes out only when pulled out of its seat past its
  hold for 0.15 s: 8.7 N (0.58 mm) for the press fit alone, plus the vacuum.
- **Vacuum and strike:** the vacuum grows as the shaking chills the air: each change of direction counted while
  sealed (see Shaken) adds a sixth of its 41 N, so after the 6 needed for mixing the seal holds 50 N (it lets go
  3.3 mm out). A push or blow of at least 10 N across the tin's axis, for 2 control steps (0.1 s), from either arm on
  the large tin's wall in its top 25 mm (from the mouth down to the seal), read from PhysX contact forces, points and
  normals, lets the air in: the vacuum is gone, and builds again only with more shaking.
- **Shaken:** while the shaker is sealed and tipped past 45 degrees, at least 6 changes of direction of its middle
  along its own axis, each after a leg of 2.5 cm or more, and the ice cubes reaching alternate ends at least 8 times in
  all. A cube is at an end when its centre is within 9 cm of that end's floor and it touches that tin (a PhysX
  contact force), so each count is a cube carried at least 9.7 cm along the shaker into the other tin.
- **Spill:** the drink is 24 rigid beads, 8 mm and 5.3 g each (127 g, the drink's mass), each standing for 5 ml. A
  bead or a cube whose centre is outside both tins is spilled: at most 1 bead (5 ml) for 1.0 and 0.25, at most 3
  (15 ml) for 0.5, and no ice cube at all.
- **Open:** the small tin off, its rim at least 3 cm from the large tin's mouth, with the large tin standing upright
  on the table (tilted less than 10 degrees).

The beads and the ice do not collide with the arms: the YAM fingers' collision hulls reach through a gripped tin's
wall and would push what is inside out through it.

## Why two hands

The sealed tins hold together until they are pulled apart: 8.7 N for the press fit alone, and 50 N once the shaking
has chilled the air inside, until a strike or press at the seam lets the air in. The full shaker weighs 5.0 N, so one
hand pulling the small tin lifts the whole shaker off the table: the large tin has to be held down by the other hand.
The strike needs the shaker held as well: standing alone, it tips over under a side push of 1.0 N at the seam (its
5.0 N weight on its base's 31.75 mm radius, against a push 165 mm up). One hand can seal and shake the shaker. In the
one-hand check (the right arm plays its demo part, the left stays at its rest pose) it sealed and shook it (12 changes
of direction, 32 ice passes), and the sealed shaker stood back on the table at 0.25. The right's strike at the seam
then knocked it over onto its side (a single 25.7 N contact, too brief to let the air in), still sealed, and the trial
ended at 0.0. Without the strike the vacuum stays: in a check run of the two-hand demo with the strike stopped 2 cm
short of the tin, the seal held through the right's 10 cm pull (drawn 1.45 mm of its 3.3 mm give), the whole sealed
shaker was pulled 85 mm up out of the left hand, and the trial ended at 0.0. Bartenders also shake a Boston shaker
with a hand on each tin, so a seal knocked by the ice cannot open.

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
the right lets go and strikes the large tin's seam from the side with its closed fingertips, about 15 mm below the mouth,
pushing toward the left hand, which lets the air in. It grips the small tin again and pulls it 10 cm straight up,
breaking the seal, and holds it up over the open large tin (anything wedged in the small tin can only drop back in),
and the left lets go. In a debug run (`SHAKE_DEBUG=1`) the press reached 15.1 N at the seat; the shake gave 12 changes
of direction and 26 ice passes, with the seal giving at most 0.34 mm (with the vacuum it would let go at 3.3 mm); the
strike vented the seal at 12.6 N and peaked at 19.3 N, tipping the sealed shaker 15.8 degrees toward the left hand
before it stood back upright (0.0 degrees); the pull drew the seal 1.08 mm (its hold, 8.7 N, lets go at 0.58 mm) and
it let go; nothing was spilled (0 beads, 0 ice cubes), and the run ended at full score after 983 steps. The YAM
fingers' collision shapes are not quite parallel, so a gripped tin leans a few degrees in the hand; the demo turns each
wrist to match:

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
it is sealed, the vacuum's share and the hold, any blow on the seam, how far the shaker is tipped, the changes of
direction and ice passes so far, and anything outside the tins).

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (412 s on the clock, with Dream answering in 0.67 s on average), and about 9 minutes for the scripted demo (983 steps, 49.1 s of simulated time, 534 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is a fixed list of waypoints. Keep
the jitter at 0: it shifts each free object on its own, and the two tins start on one axis, joined by the fit's joint.

## Changes in this revision

This is a revision of TK-PXNFCR. Its review said:

> Construct validity (not met): At 21 mm depth the specified straight-tapered large tin is about 9.08 cm across inside, leaving a gap around the 8.64 cm small-tin rim rather than a press fit.
>
> Clarity and reproducibility (not met): The specified tin dimensions conflict with the stated 16 mm loose and 21 mm sealed coaxial starting and operating positions.
>
> Feasibility (not met): The scripted demo uses a simulated slide-joint seal that the specified real tin geometry does not establish.
>
> What to change: The stated straight-tapered tin dimensions leave approximately 4.4 mm of diametral clearance at the claimed 21 mm press-fit depth, so the real tins cannot seal concentrically as specified. Specify and verify a real tin pair and starting alignment that achieve the stated loose and sealed positions, update the simulation to match the measured geometry and sealing behavior, and provide a full-score demo for that corrected setup.

The review before that (TK-JE4KUU) asked for a realistic cocktail shaker with water and ice, a measurable mixing and no-spill outcome, a full-score demo for that setup and the simulation approximations in the fidelity note; that was done in the previous revision and is kept.

The first review (TK-CCNDMT) asked for every nonzero score to need the shaker closed during the tipping and shaking, a demonstrated full score, and the Societal relatability category; that was done in the first revision and is kept.

What changed in response:

- **Construct validity (a 4.4 mm gap at 21 mm instead of a press fit):** the shaker is now a real catalogue pair at its listed sizes, and the fit follows from them. Large tin: Acopa 28 oz full-size shaker tin (item 176SHKTSL28), 177.8 mm (7 in) tall, 92.1 mm (3 5/8 in) across the top and 63.5 mm (2 1/2 in) across the bottom, outside, 186 g (0.41 lb listed). Small tin: the matching Acopa 18 oz half-size tin (item 176SHKTSL18), 133.4 mm (5 1/4 in) tall, 87.3 mm (3 7/16 in) across its rim and 60.3 mm (2 3/8 in) across its bottom, 145 g (0.32 lb listed). The walls are 0.5 mm: not listed, but 0.5 mm steel over the large tin's shape weighs 185 g, and its inside then holds 830 ml brim-full, its 28 oz. So the large tin is 91.075 mm across inside at the mouth and narrows by 0.1607 mm per mm of depth. Set straight in, the small tin's 87.3125 mm rim meets the wall all round at (91.075 - 87.3125) / 0.1607 = 23.4 mm below the mouth. Pressed 1.0 mm further, to 24.4 mm, where the large tin is 87.15 mm across inside, the rim is 0.16 mm tight: a press fit, not a gap.
- **Clarity and reproducibility (dimensions that contradict the stated positions):** the starting alignment is coaxial (straight in, mouth down), and the stated positions are the ones these dimensions give. Resting, the rim sits at 23.4 mm, where it first touches the wall all round: not sealed, it lifts off with 0.8 N more than its 1.42 N weight. Sealed, it sits at 24.4 mm. The README shows the arithmetic and how to check a real pair (a caliper's depth rod at the rim). It also notes that the listings are rounded to 1/16 in, so a real pair can sit a few mm higher or lower (0.6 mm of depth per 0.1 mm of diameter); the scoring does not depend on the depth. The task file prints every number the text quotes.
- **Feasibility (a slide-joint seal the geometry does not establish):** the simulated tins now have the real surfaces where they meet: the large tin's inside and the small tin's outside. The extra simulated wall thickness (2 mm walls against a real 0.5 mm) goes outward on the large tin and inward on the small one. At 23.4 mm the simulated rim just touches the large tin's inside (0.00 mm gap); at 24.4 mm it overlaps it by 0.08 mm a side, the interference. The joint engages only there. Off the wall the small tin slides freely. At first contact a spring of 15 N per mm resists pushing it further in. Pushed the 1.0 mm home (15 N), it is sealed.
- **Sealing behaviour with real values:** the press fit's hold follows from the seating press and the 4.6 degree taper: 15 N x (0.3 cos 4.6 deg - sin 4.6 deg) / (0.3 cos 4.6 deg + sin 4.6 deg) = 8.7 N, taking a friction coefficient of 0.3 for wet steel. The fit is self-locking, since tan 4.6 deg = 0.080 is below 0.3. Shaking also chills the sealed air from 20 to 0 C, a vacuum of 101.3 kPa x 20 / 293 = 6.9 kPa (1.0 psi). That presses the tins together over the 87.15 mm seal circle with another 41 N; a published test of the effect measured 1.37 psi after a 15 s shake of vodka on ice. In simulation the vacuum builds by a sixth with each counted change of direction, so a shaken seal holds 50 N, more than a YAM arm pulls. Bartenders let the air in with a strike or a firm press at the seam: the 25 mm of 0.5 mm wall between the mouth and the seal bends about 0.27 mm per newton, more than the 0.16 mm interference. The simulation counts a side push of at least 10 N for 0.1 s on the large tin's top 25 mm, read from PhysX contact forces, points and normals; after it, 8.7 N parts the tins.
- **Full-score demo for the corrected setup:** the new scripted demo scores 1.0 (983 steps). In a debug run, the right pressed the small tin home with 15.1 N at the seat. Both hands then lifted, tipped and shook the sealed shaker: 12 legs between 65 and 107 degrees from upright, each moving it 3 cm along its axis. That gave 12 changes of direction and 26 ice passes, and the seal gave at most 0.34 mm (with the vacuum it lets go at 3.3 mm). They set it down, and the left held the large tin down, upright. The right struck the seam from the side with its closed fingertips: the seal vented at 12.6 N, and the push peaked at 19.3 N. The sealed shaker tipped 15.8 degrees and stood back upright. The right gripped the small tin again and pulled it 10 cm up, and the seal let go (drawn 1.08 mm, past its 0.58 mm hold). Nothing was spilled: 0 beads and 0 ice cubes out of the tins. The demo, Dream-1 and setup media are new.
- **Two hands, re-checked:** opening needs the large tin held down, because even after the strike the press fit holds 8.7 N and the full shaker weighs 5.0 N. The strike also needs the shaker held, because a standing shaker tips over under a 1.0 N side push at the seam. In the one-hand check (the right plays its demo part, the left stays at rest), one hand sealed and shook the shaker (12 changes of direction, 32 ice passes), and it stood sealed on the table at 0.25. The right's strike then knocked it over onto its side, still sealed, and the trial ended at 0.0. In a check run of the demo with the strike stopped 2 cm short, the vacuum held through the 10 cm pull (1.45 mm of its 3.3 mm give), the whole shaker was pulled 85 mm up out of the left hand, and the trial ended at 0.0.
- **Kept from the earlier revisions:** the real trial (90 ml of water with 30 ml of 50 °Brix syrup layered under it, four 14 g ice cubes, refractometer and kitchen-scale tiers, 1 bead = 5 ml = 5 g) and the rule that every nonzero score needs the shaker sealed while it is tipped and shaken. Also kept: the instruction (goal and constraints only) and the Societal relatability category.
