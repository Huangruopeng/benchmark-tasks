# Seal the joint between a bath and its tiles, starting from an empty caulking gun

An empty caulking gun lies level on the table, nozzle end to the left, on its pistol grip and a wooden rest block. Its
plunger rod is part way in. To the left is the white rim of a bath against a tiled wall, the joint between them not yet
sealed; in front of the gun a new 310 ml cartridge of grey sanitary silicone lies on a cardboard tray. The robot must
seal the joint with an unbroken bead of the silicone, without smearing silicone anywhere else, and leave the gun lying
on the table. The gun starts empty, so it has to be loaded, but only the outcome is scored: how much of the joint the
bead seals, how much silicone ended up anywhere else, and where the gun is left. The task is one task file,
[`load_a_caulking_gun_and_seal_the_bath_joint.py`](load_a_caulking_gun_and_seal_the_bath_joint.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder. (The folder keeps its first name.)

What it tests: operating a tool's controls with two hands (holding a release while pulling against it), loading a part
the right way round into a tool, then guiding the loaded tool along a line with both hands while metering the sealant
with the trigger, so that the bead is continuous, touches both surfaces and goes nowhere else. None of these steps is
scored for itself; without them there is no bead.

Why two hands: the rod is held by the gun's locking plate and comes back out only while the release lever is held
down, and the lever is about 20 cm from the rod's hook. Pulled on without the lever pressed, the rod stays put (pulled
harder, the light gun slides along the table instead); the cartridge only fits once the rod is all the way back. With
one hand (the right doing its part of the demo, the left never moving) the trial scored 0: the right pulled on the
hook, the rod stayed locked 8 cm in, no cartridge was loaded, and its squeezes of the trigger laid no sealant (0.00 ml
dispensed, none of the joint sealed, so it scores 0 under the revised scorer too). While sealing, the second hand on
the yoke holds the gun's roll and carries its front while the first works the trigger.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The bath rim with its tiled wall, the tray and the
rest block are kinematic, fixed where they are put; the gun's frame, its trigger, its gripping plate, its plunger rod,
its release lever, the cartridge and the cartridge's piston are loose rigid bodies made of shapes from `bodykit.py`.

The gun works as a real smooth-rod gun does:

- **Trigger.** A hinge below the rod, with a return spring (0.10 N m at rest, 0.24 N m fully squeezed) and a 36 degree
  stroke. Its blade leans forward and carries a rounded finger rest (a 12 mm bar) 36 mm below the pivot. Above the
  pivot a short arm (a round nub 5 mm from the pivot) pushes the gripping plate; the middle of a hand presses about
  50 mm below the pivot, so the gun's thrust ratio is 10:1 there (7:1 at the finger rest, where the robot squeezes). A
  full squeeze moves the plate about 2.5 mm.
- **Gripping plate.** A steel plate on the rod behind the cradle, on its own slide in the frame, pushed back against
  the trigger by its spring (5 N, 1 N/mm). Pushed forward by the trigger it carries the rod with it; let go, it slides
  back along the rod while the rod stays. In a real gun the plate tilts and binds on the smooth rod by friction. Here
  that binding is a one-way stop between the plate and the rod (a limit on a slide between them): the rod can always
  run forward through the plate, but not back past the stop, which the task file keeps 0.05 mm behind the plate
  except while the plate is moving forward. This is the first of the two rules in the drive.
- **Locking plate and release lever.** The rod slides in freely but cannot come back out unless the thumb release
  lever is pressed down at least 6 degrees (a sprung hinge, 12 degrees of travel). In a real gun a tilted plate binds
  the rod. Here a one-way stop on the rod's slide that the task file keeps 0.1 mm behind the rod as it goes in, and
  removes while the lever is pressed. This is the second rule.
- **Cartridge and piston.** A 310 ml cartridge: a 49 mm tube 205 mm long with a 46 mm bore, the piston 4 mm in from
  its back end on a slide in the tube. The rod's 40 mm push plate pushes the piston. The silicone resists the piston
  with 150 N (its flow pressure through the cut nozzle, about 0.09 MPa): a stiff spring on the piston's slide whose
  force is capped at 150 N and whose anchor the task file moves along behind the piston while it is pushed through.
  A push on the rod's hook (a YAM arm gives about 25 N) does not move it. Through the trigger's 10:1 it takes about
  15 N at the middle of the trigger plus the return springs; the YAM, pinching the trigger 36 mm below its pivot
  (7:1), needs about 26 N, within its 50 N.

The flow approximation (silicone is not simulated as a fluid):

- Volume pushed out = the piston's advance through the silicone x the 46 mm bore's area, 16.6 cm^2: 1.66 ml per mm of
  piston travel (the piston's elastic give below 150 N does not count). A full squeeze from the trigger's rest to its
  stop moves the piston about 2.4 mm, about 4 ml; the demo's strokes start from the holding grip and give about 2.7 ml
  each (10.85 ml in four, 6.5 mm of piston travel).
- It leaves the nozzle's tip a moment later: a first-order lag of 0.25 s, standing in for the sealant's compression in
  the cartridge, so the flow starts and stops a quarter of a second after the piston does.
- It is laid along the path of the nozzle's tip: every 5 mm of tip travel, 0.3 s or 0.4 ml, a stretch of bead whose
  section is the volume laid divided by the length of the path. A 6 mm bead is 28.3 mm^2, 0.0283 ml per mm of joint
  (a 310 ml cartridge lays about 11 m of it; the 192 mm joint takes 5.4 ml).
- A stretch counts as in the joint when the bead it makes touches both the rim's top and the tiles: the tip within the
  bead's radius plus 2 mm of each surface at both ends of the stretch. Its sealant is then shared out over the joint's
  1 mm lengths it covers (it runs its radius past the tip's path at each end). It is drawn as grey silicone in the
  corner: rounds sunk along the corner's bisector so that the section showing equals the section laid.
- A stretch that does not touch both surfaces never seals. If both its ends were laid with the tip within 1 cm of the
  tiles and within 1 cm of the rim's top (along the bath) it is near the corner: too thin or too far off to seal, but
  no mess. Otherwise it is mess: silicone on the tiles more than 1 cm above the rim, on the rim more than 1 cm out
  from the tiles, on the bath's front or the table. Both are drawn as a grey blob where the tip was. With no trigger
  squeeze there is no sealant: a nozzle drawn along the joint without it lays nothing.

Sizes. Everything is at real size and mass except, disclosed: the pistol grip reaches 9 cm below the rod (a typical
gun's about 12 cm), so the gun lies at a height both arms can reach; the cradle's back plate is open below the rod,
where the robot's fingers pass to the trigger; and the bath is a 34 cm length of its rim and tiled wall, the joint to
seal being the 19.2 cm the arms can reach. The silicone's 150 N resistance is an estimate for a general-purpose
silicone at room temperature (a real one varies with temperature and nozzle cut).

## Scoring

Only outcomes, read at the end, are scored. How the gun was loaded, how the trigger was worked, the order of things
and where the hands are play no part (an earlier version gave 0.25 for a loaded gun; that tier is gone).

- **Sealed fraction:** a 1 mm length of the 192 mm joint is sealed when it holds at least 12.6 mm^2 of sealant (a bead
  at least 4 mm across) laid in the joint; the sealed fraction is the share of the 192 lengths that are sealed, and
  the longest gap is the longest run of unsealed lengths, the joint's two ends included.
- **Mess:** the volume of sealant laid more than 1 cm from the corner, as above.
- **Gun lying on the table:** its grip's foot within 2 cm of the table top and the gun tilted at most 25 degrees.

At the end:

- **1.0**: at least 95% of the joint sealed with no gap longer than 2 mm, at most 0.1 ml of mess, and the gun lying
  on the table;
- **0.75**: sealed and clean the same, the gun not lying on the table (still held up, or left on the bath);
- **0.5**: at least 50% of the joint sealed (this includes a fully sealed joint with more than 0.1 ml of mess);
- **0.25**: at least 25% of the joint sealed;
- **0.0** otherwise.

The trial ends at the run limit, or once the full score has held for 1 s with the gun, its trigger, plate, rod, lever,
the cartridge and its piston still.

Measured in a real trial: photograph the joint from above with a steel rule laid along it. The sealed length is where
the bead is at least 4 mm wide and wets both the rim and the tile; the sealed fraction is that length over 192 mm and
the longest gap is the longest stretch without it, both read off the rule to the millimetre. Mess is all silicone more
than 1 cm from the corner (on the tiles, the bath, the table), scraped onto a square of foil and weighed on a 0.01 g
scale: sanitary silicones are 0.98 to 1.05 g/ml on their makers' data sheets, so 0.1 ml is about 0.1 g. Whether the gun
lies on the table on its grip is read from the same photograph.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (the full score, with the gun and its moving parts still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_a_caulking_gun_and_seal_the_bath_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Seal the joint between the bath and the tiles with an unbroken bead of the silicone, without smearing silicone anywhere else, and leave the gun lying on the table." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/caulk-dream
```

The scripted demo, which reaches full score from fixed waypoints. The left hand presses the release lever down with
closed fingers, holding the gun, while the right takes the rod's hook and pulls the rod all the way back. The left lets
go, lays the cartridge in the cradle nozzle through the yoke, and the right pushes the rod in until its plate meets the
piston. The left takes the yoke across its width and the right closes across the pistol grip and the trigger's finger
rest. They lift the gun, lean it nose down, set the nozzle's tip in the corner at the joint's right end and lay the
bead in four strokes. Each time the right squeezes the trigger until the silicone comes, then on to the trigger's stop
while both hands draw the nozzle about 7 cm along the corner; between strokes it lets the trigger come back while the
nozzle slides back 12 mm, so each stroke starts on the end of the last. They lift the nozzle out, set the gun back on
its grip and rest block, let go and move clear; the left ends over the joint, its wrist camera looking down at the
bead:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_a_caulking_gun_and_seal_the_bath_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Seal the joint between the bath and the tiles with an unbroken bead of the silicone, without smearing silicone anywhere else, and leave the gun lying on the table." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/caulk-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/caulk-dream/*.json; do inspect-robots video "$log"; done
```

Set `CAULK_DEBUG=1` to print what the score is built from four times a second: the lever's and the trigger's angles,
the gripping plate's travel and the stops behind it and behind the rod, the rod and the piston, the sealant pushed out,
laid in the joint, laid elsewhere and how much of that is mess, where the nozzle's tip is from the corner, the joint's
sealed fraction and longest gap, where the gun is and whether it lies on the table, and where each TCP is. Whether the
cartridge is seated and the push plate on its piston is printed too, for debugging only: it is not scored.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,500 steps (412 s on the clock, with Dream answering in 0.61 s on average), and about 14 minutes for the scripted demo (1,389 steps, 69.5 s of simulated time, 870 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-VB7239. Its review said:

> The score should be dependent on the outcome, not the procedure

The review before that (TK-5MD4DX) asked for a functional trigger, a flow approximation tied to its operation,
measurable real-world bead coverage and continuity requirements and a full-score demo, because the bead rule credited
nozzle movement without any sealant dispensed; that was done in the previous revision and is kept.

What changed in response:

- **Every tier is now an outcome, read at the end.** The 0.25 tier for a loaded gun (the cartridge seated and the push
  plate on its piston) is gone: loading is a step, not a result. The full score no longer asks for both grippers to
  end at least 6 cm from the gun (where the hands are). Nothing about the trigger, the order of steps or the hands is
  scored; the score reads only the bead in the joint, the silicone anywhere else, and where the gun lies.
- **Partial credit from partial outcomes:** 0.5 for at least 50% of the joint's 192 mm sealed, 0.25 for at least 25%,
  both counted over the joint's 1 mm lengths at the end (a length is sealed with at least 12.6 mm^2 of sealant, a bead
  at least 4 mm across, wetting both the rim and the tiles).
- **No silicone where it should not be (new):** sealant laid with the tip more than 1 cm from the tiles or from the
  rim's top, or beyond the bath's ends, is mess. Full score allows at most 0.1 ml; a fully sealed joint with more
  falls to 0.5. Real trial: the mess is scraped onto foil and weighed on a 0.01 g scale (sanitary silicones are 0.98
  to 1.05 g/ml, so 0.1 ml is about 0.1 g). The demo leaves 0.006 ml (from the nozzle lifted out after the last stroke).
- **The gun left lying on the table**, as the instruction asks, is kept as an end state: its grip's foot within 2 cm of
  the table top and tilted at most 25 degrees (0.75 without it).
- **Instruction and title state only the goal.** Instruction: "Seal the joint between the bath and the tiles with an
  unbroken bead of the silicone, without smearing silicone anywhere else, and leave the gun lying on the table."
  (it no longer says "load the gun"). Title: "Seal the joint between a bath and its tiles, starting from an empty
  caulking gun".
- **Adversarial check:** A trial that loads the gun exactly as the demo does (cartridge seated, push plate on its
  piston), lifts it off its rest and squeezes the trigger through four full strokes in the air, never at the joint,
  then sets it down on its grip and rest block, pushed out 10.99 ml, all of it mess (none in the joint, 0% sealed). It
  scores 0.0; the previous scorer would have given it 0.25 for the loaded gun.
- **Demo, one-hand and Dream-1 under the revised scorer:** The demo (unchanged) scores 1.0: 100% of the joint sealed
  with no gap (the thinnest 1 mm length holds 15.5 mm^2 against the 12.6 needed, the mean is 43.2 mm^2), 10.85 ml
  pushed out, 10.56 ml laid in the corner, 0.006 ml of mess, and the gun back on its grip and rest block. The one-hand
  trial of the previous revision pushed out 0.00 ml, so it scores 0 under the revised scorer too. Dream-1:
  0.0 (1,500 steps).
