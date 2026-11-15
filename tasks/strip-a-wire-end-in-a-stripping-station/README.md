# Strip 10 mm of insulation off a cable end in a lever-operated benchtop stripping station

A steel benchtop wire-stripping station stands on the table: a block with a funnel in its top leading down to two
blades, which close round whatever is between them while its lever is held down. A short single-core 35 mm² cable
of type H07V-K (a fine-stranded copper core 7.9 mm across under 1.2 mm of PVC insulation, 10.3 mm across overall)
with a plastic plug on one end stands upright beside it, its insulated end in a holder's 12 mm bore (0.85 mm of
clearance all round). One hand takes the cable by its plug and puts its free end down through the funnel, 10 mm past
the blades; the other presses the lever down (the cut takes about 20 N at the grip), closing the blades, and keeps it
down while the first pulls the cable straight back out (about 20 N). The cut insulation stays in the station and
10 mm of copper is bare. The task is one task file,
[`strip_a_wire_end_in_a_stripping_station.py`](strip_a_wire_end_in_a_stripping_station.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: setting an insertion depth to within 2 mm, and two hands working at once on one tool, one holding it
shut while the other pulls.

Why it matters: every conductor is stripped to length before it is terminated, so stripping is regular work in
electrical installation and assembly. Electricians held about 821,000 US jobs in 2025 (BLS Occupational Outlook
Handbook, which lists wire strippers among their commonly used tools); O*NET lists for them "Connect wires to circuit
breakers, transformers, or other components" (47-2111.00), and for their helpers "Strip insulation from wire ends,
using wire stripping pliers, and attach wires to terminals for subsequent soldering" (47-3013.00). In factories,
electrical and electronic equipment assemblers (246,300 jobs, BLS) and team assemblers (1,403,500, O*NET: "Provide
assistance in the production of wiring assemblies") strip leads for control panels, switchgear and harnesses, by hand or
on benchtop stripping tools like this one. Too short a strip and the terminal does not seat; too long and bare copper
shows.

Why two hands: the blades hold the cut insulation only while the lever is held down, and the cable has to be pulled out
at the same time; if the lever comes up first, the cut insulation comes back out on the cable. One hand keeps the lever
down while the other pulls the cable, so a single hand cannot strip it. Checked on this version: with the cable let go
in the station and the other hand pressing the lever, the cable dropped to the bore's floor (25.0 mm below the blades),
the closing lever cut all 12 rings (24 mm) and the closed blades held the cable, and letting the lever up put the
insulation back on the cable and let it go: 0.25.

## The real station

Every dimension here is the one the task file uses; the forces are computed in the task file from these dimensions
(`_blade_force`, `_cut_nm`), and the numbers below are printed from it.

**Block, funnel and bore.** A steel block 50 x 50 mm and 30 mm tall, fixed to the bench, its bore's axis 24 cm forward
and 12 cm left of the centre line. A steel funnel on top (1.6 mm wall) is 44 mm across at its rim, 40 mm above the
table, and narrows over 10 mm to a 12 mm throat at the block's top face. The blades slide there, 30 mm above the table.
Under them a bore 16 mm across goes down to a floor 5 mm above the table: a screw plug in the block's base, taken out
to empty the cut slugs. The station weighs about 0.65 kg (the block 0.55 kg, plus the funnel, the hinge bracket, the
blades, their carriers and the eccentric).

**Blades (V-blade angle and placement).** Two opposed full-radius V-blades of hardened tool steel, each 1.0 mm thick,
the one sliding on the other, their edges ground to an included angle of 20 degrees. Each blade's notch is a 90-degree
V entry ending in a semicircle 3.95 mm in radius (the "full radius" form of stripping-machine blades, whose semicircle
is sized to the conductor). They lie in the horizontal plane of the funnel's throat, 30 mm above the table, symmetric
about the bore's vertical axis, and slide toward each other along the lever's direction (left-right in the scene) in a
guide in the block's top face. A pinion between racks on the two blade carriers makes them move by equal amounts in
opposite directions, so they close on the bore's axis.
- Open (lever up), each blade stands 4.54 mm back from closed, and the two V's leave an opening whose inscribed circle
  is 12.0 mm across: (3.95 + 4.54) / sqrt 2 = 6.00 mm in radius, 0.85 mm all round the 10.3 mm cable.
- They first touch the insulation 3.30 mm from closed: the semicircles' ends are then sqrt(3.95^2 + 3.30^2) = 5.15 mm
  from the axis, the cable's outer radius.
- Closed (lever down, on its stop), the two semicircles make a 7.9 mm circle, the conductor's diameter, so they cut the
  1.2 mm insulation all the way round, as a stripping tool must (the knives "properly adjusted so the insulation is cut
  all the way through without nicking the ... conductor", Anixter, Wire Wisdom: Wire Strippability, Wiring Harness
  News, Jul/Aug 2021). The previous version closed them to 8.2 mm, which would have left a 0.15 mm web of PVC for the
  pull to tear (about 47 N at 12.5 N/mm2 over its 3.8 mm2); that is gone.

**Cam to blades (geometry and travel).** The lever's hub carries an eccentric (a circular cam) whose centre is 13.06 mm
off the hinge axis. It runs in a vertical slot in the right-hand blade carrier (an eccentric-and-yoke drive), so the
carrier moves by the eccentric's sideways offset. Over the lever's 20 degrees of travel the eccentric swings from
10 degrees before to 10 degrees past straight down, which moves each blade 2 x 13.06 mm x sin 10° = 4.54 mm, the whole
way from open to closed.
- The grip moves 0.20 m x 0.349 rad (20°) = 69.8 mm, so the lever-to-blade ratio is 69.8 / 4.54 = 15.4 on average (15.3
  at mid-stroke, 15.55 at the closed end).
- The blades touch the insulation with the lever 14.5 degrees from closed and cut over the last 14.5 degrees.

**Lever and spring.** The lever is a steel rectangular tube 24 x 16 mm with 1 mm walls, 24.5 cm long (2 cm behind the
hinge, 22.5 cm in front of it, reaching right), 146 g, with a red rubber grip sleeve 5 cm long and 3 mm thick (17 g)
centred 20 cm from the hinge: 163 g in all, its centre of mass 11.2 cm from the hinge. It is hinged on a bracket on the
block's right side, 52 mm above the table and 30 mm right of the bore's axis; closed, the grip's middle is 11 cm right
of the centre line. A torsion spring on the hinge pin lifts it to its open stop, 20 degrees up (the grip 6.8 cm above
where it closes).
- The spring's rate is 0.04 N m per degree. It is preloaded: wound 10 degrees with the lever on its open stop, it gives
  0.04 x 10 = 0.40 N m there.
- Closed, it is wound 30 degrees: 0.40 + 0.04 x 20 = 1.20 N m.
- The lever's weight pulls the other way with 0.163 kg x 9.81 x 0.112 m = 0.18 N m when level (closed), 0.17 N m at
  20 degrees.
- So holding the lever down with no cable in takes (1.20 - 0.18) / 0.20 m = 5.1 N at the grip, and the lever rests on
  its open stop with 0.40 - 0.17 = 0.23 N m to spare.

**Force to cut.** A sharp edge pushed into a ductile material has to wedge it apart: if the PVC's flow stress is taken
as its tensile strength, sigma, a wedge of included angle beta pushed h deep needs a force per unit length of edge of
F' = 2 sigma h (tan(beta/2) + mu), mu being the PVC-on-steel friction.
- With sigma = 12.5 N/mm2 (the minimum for PVC/C insulation in IEC 60227-1, Table 1; H07V-K's IEC counterpart is the
  60227 IEC 02 cable), beta = 20 degrees and mu = 0.2: F' = 9.41 N/mm per mm of depth, 11.3 N/mm at the full 1.2 mm.
- Closed, each blade's semicircle (12.4 mm of edge, 7.9 mm across) is at full depth, so it is pushed back with
  11.3 x 7.9 = 89 N along its travel, plus 13 N on the stretches of the V flanks that are inside the insulation: 102 N
  per blade.
- Through the eccentric (12.86 mm of blade travel per radian of lever at the closed end: 13.06 mm x cos 10°) and a
  mechanism efficiency of 0.9, both blades take 2 x 102 N x 0.01286 m / 0.9 = 2.91 N m at the hinge: 14.6 N at the
  grip.
- So the cut peaks at 14.6 + 5.1 = 19.7 N at the grip as the lever reaches its stop. The cut takes 0.20 J at the blades
  in all. A compound at 15 N/mm2 instead of 12.5 would need 17.5 N for the cut, 22.6 N in all.

**How the blades retain the cut sleeve.** Closed, the blades sit in the cut, their semicircles on the conductor. The
cut sleeve (10.3 mm outside, 7.9 mm bore) cannot pass the 7.9 mm opening, so as the cable is pulled up its top face
bears on the blades' undersides and the conductor slides out of it. The blades still cover 1.10 mm of its 1.2 mm wall
with the lever 4 degrees up from closed (each blade 0.90 mm back), which is how far the scoring lets the lever rise
before the strip fails. Off the conductor, the sleeve drops onto the bore's floor. If the lever goes up first, the
blades open past the cable and the cut sleeve stays on the conductor.

**Force to pull.** With the insulation cut through, pulling the cable out only has to slide the conductor out of the
10 mm sleeve. Strip forces (the force to pull a conductor out of a set length of insulation) are "typically in the
range of 5 to 25 pounds (22 N to 111 N)" (Anixter, as above), on test lengths commonly 25.4 mm (1 in). Friction scales
with the length, so a 10 mm sleeve takes about 9 to 44 N. The station is specified, and simulated, at 20 N.

**Checking a real station.** Before trials, a hand-held push-pull force gauge checks the station on three offcuts of
the trial cable. Closing the lever with a cable end in must peak at 15 to 25 N at the grip (5.1 N with no cable in),
and pulling a cut cable out of the closed blades must take 15 to 25 N. A cable batch outside these ranges is not used.

**The cable.** H07V-K 1 x 35 mm2 single core (a manufacturer's datasheet: a class 5 flexible copper conductor 7.9 mm
across, 1.2 mm of PVC insulation, 10.3 mm across overall, 340 g per metre), 12 cm long. Its top 3 cm is moulded into a
square grey ABS plug 16 mm across and 3 cm tall, so 9 cm of cable shows below the plug.
- 12 cm of cable at 340 g/m weighs 0.12 x 340 = 40.8 g.
- The plug is 16 x 16 x 30 mm of ABS (1.05 g/cm3) less the 10.3 mm cable through it: 5.18 cm3, 5.4 g.
- In all, 46.2 g. (The previous version counted only the 9 cm below the plug, 31 g, and an 8 g solid plug: 39 g.)
- Of the 340 g/m, 48 g/m is the PVC (34.3 mm2 at 1.4 g/cm3) and 292 g/m the copper; the 24 mm of insulation at the free
  end, the part that can be cut, is 1.15 g.

**The holder.** A small black ABS cup, 30 cm forward and 21 cm left of the centre line, with a 12 mm bore (0.85 mm of
clearance all round the 10.3 mm cable, and wider than the 11.7 mm maximum EN 50525-2-31 allows), 2 mm walls up to
1.8 cm above the table and a base 20 mm across and 4 mm thick, on which the cable's end stands: about 2.6 g.

**Scoring a real trial.** The stripped length is measured with a caliper from the conductor's end to the insulation's
cut edge (8 to 12 mm for 1.0), the cut sleeve is found on the bore's floor, and the insertion, the order of pull and
lever and the release are judged from the overhead video.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The station (a block with a funnel, the open blades'
12 mm opening at the funnel's throat, and a bore under it) and the cable's holder are kinematic, fixed where they are
put; the funnel is polished steel, so its contact with the cable takes the lower friction, 0.3. The lever turns on a
hinge (a `Hinge` from `bodykit.py`) with the torsion spring above (0.04 N m per degree, its free position 10 degrees
past the open stop), and its mass (163 g) and centre of mass (11.2 cm from the hinge) are those of the steel tube and
rubber sleeve. The cable is a rigid body of 45.1 g with its centre of mass 6.7 cm up from its end; the insulation over
its last 24 mm is 12 rings of 2 mm, 0.096 g each (46.2 g in all), each held to the cable by a fixed joint (a `Glue`
from `bodykit.py`), shown but not colliding. For contact, that insulated end is one hidden cylinder of the cable's full
10.3 mm width, part of the cable itself, so the end stands in the holder's 12 mm bore with its real clearance and has
to pass the blades' 12 mm opening as the insulated cable does. It stays on after the cut; the bare end touches nothing
later in the task. The blades themselves are not bodies:

- Cutting force: while the cable's insulated part is across the blades and they close into insulation not yet cut, the
  task file adds the torque the cut takes (`_cut_nm`, from the wedge model above, up to 2.91 N m) to the lever's
  spring, as a drive set after every control step. The robot has to press about 19.7 N at the grip to bring the lever
  onto its stop. Once the cut is made (or with no cable in), only the spring is left.
- The cut: the first time the lever closes (within 0.5 degrees of its stop) with the cable's end below the blades, the
  task file switches the rings below the blades from the cable to the station, fixed where they are, and holds the
  cable to a hidden fixed anchor with a fixed joint that PhysX breaks once the force on it passes 20 N (the pull) or
  the torque 2 N m.
- Stripped: if the cable's end comes up out past the blades while the lever is still within 4 degrees of closed, the
  cut rings stay in the station and drop to the bore's floor. If the lever comes up first, they go back onto the cable
  and the cable is let go.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the lever and the cable still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip 10 mm of insulation off the cable's free end in the stripping station, then let the station's lever back up and take your hands off it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable's plug, fingers closing
across it, lifts the cable straight out of its holder, carries it over the station (aiming 11 mm to the side, against
the measured lean of the cable in the fingers) and lowers its end through the funnel, 10 mm past the blades (measured:
9.9 mm below the blades when they closed, cut to the nearest 2 mm: 10 mm). The right hand comes straight down on the
lever's rubber grip, fingers closed, and presses the lever down hard, aiming 6 cm below the closed grip so that the
soft arm pushes through the cut's 19.7 N; it holds the lever there while the left pulls the cable straight up, aiming
6 cm high so that the arm pulls past the 20 N the closed blades hold it with (it came free once the arm's aim had risen
about 3 cm). Then the right rises and the lever springs back up:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip 10 mm of insulation off the cable's free end in the stripping station, then let the station's lever back up and take your hands off it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRIP_DEBUG=1` to print what the score is built from (the lever's angle, the cutting torque the lever meets,
how deep the cable's end is below the blades and how far off the bore's axis, how many rings were cut, whether it is
stripped, how far the cable has risen since the blades closed on it, and where each gripper's TCP is from the cable's
end and from the lever's grip) twice a second, and every 0.1 s while the lever cuts or the cable is held.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (536 s on the clock, with Dream answering in 0.73 s on average), and about 4 minutes for the scripted demo (514 steps, 25.7 s of simulated time, 214 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-NFPGN5. Its review said:

> Clarity and reproducibility (not met): The custom station lacks the blade angle, cam geometry and cutting-force details needed to reproduce its claimed cutting and slug retention.
>
> What to change: The domain, holder bore and conductor specification address the latest review. To complete the earlier station feedback, specify a reproducible stripper: give the V-blade angle and placement, cam-to-blade geometry and travel, how the blades retain the cut sleeve, and the force needed to cut and pull the specified cable, or identify a suitable existing tool. Reconcile the spring torque with its 12-degree travel and stiffness, including any preload, and correct or explain the cable mass: 340 g/m over 12 cm is 40.8 g before the plug, not 31 g.

The review before that (TK-UEJBT7) asked for the Impact on jobs domain, a holder bore that takes the cable, and a core
diameter reconciled with 35 mm²; that was done in the previous revision and is kept (H07V-K 1 x 35 mm², 7.9 mm
conductor, 10.3 mm overall, 12 mm holder bore).

The first review (TK-3V78DN) asked for a buildable station (blade geometry, lever-to-blade mechanism, sleeve retention,
dimensions and operating forces) and the holder's dimensions; this revision completes the station, as below.

What changed in response:

- **Clarity and reproducibility: a fully specified stripper.** No existing manual bench tool for 35 mm² cable that
  works by "close the blades, pull the cable" was found (closing-blade hand strippers stop at about 16 mm²; at 35 mm²
  such stations are pneumatic), so the station is now fully dimensioned instead, in a new section, "The real station",
  with every number taken from the task file.
- **V-blade angle and placement:** two opposed full-radius V-blades of hardened tool steel, 1.0 mm thick, edges ground
  to 20 degrees, each notch a 90-degree V entry ending in a semicircle 3.95 mm in radius. They slide toward each other
  along the lever's direction in the horizontal plane of the funnel's throat, 30 mm above the table, symmetric about
  the bore's axis, kept centred by a rack and pinion. Open, they leave a 12.0 mm opening (an inscribed circle 2 x (3.95
  + 4.54) / sqrt 2 = 12.0 mm across); closed, a 7.9 mm circle, the conductor's diameter, so they cut through the 1.2 mm
  insulation all round (the previous 8.2 mm would have left a 0.15 mm web for the pull to tear, about 47 N).
- **Cam-to-blade geometry and travel:** an eccentric on the lever's hub, 13.06 mm off the hinge axis, runs in a slot in
  the right blade carrier and swings from 10 degrees before to 10 degrees past straight down. Each blade moves
  2 x 13.06 mm x sin 10° = 4.54 mm while the grip moves 69.8 mm (15.4 to 1); they meet the insulation 14.5 degrees from
  closed. The lever's travel is now 20 degrees, not 12: over 12 degrees the eccentric would have needed 21.7 mm, and
  the cut alone would have taken 24.4 N at the grip (29.5 N with the spring).
- **How the blades retain the cut sleeve:** closed, the blades sit in the cut on the conductor; the sleeve (10.3 mm
  outside, 7.9 mm bore) cannot pass the 7.9 mm opening, so its top bears on their undersides and the conductor slides
  out of it. At 4 degrees from closed, the most the scoring allows, they still cover 1.10 mm of its 1.2 mm wall. Off
  the conductor it drops onto the bore's floor, a screw plug that is taken out to empty it.
- **Force to cut:** derived with a wedge model (F' = 2 sigma h (tan(beta/2) + mu): sigma = 12.5 N/mm², the minimum
  tensile strength of PVC/C insulation in IEC 60227-1; beta = 20 degrees; mu = 0.2): 11.3 N per mm of edge at full
  depth, 102 N per blade at closing, 2.91 N m at the hinge through the eccentric at 0.9 efficiency, 14.6 N at the grip,
  19.7 N with the spring.
- **Force to pull:** with the insulation cut through, only the sleeve's friction is left: strip forces are typically
  22-111 N (Anixter) on test lengths commonly 25.4 mm, so 9-44 N for the 10 mm sleeve; the station is specified at
  20 N. A real station is checked with a force gauge before trials: 15-25 N to close on a cable, 15-25 N to pull.
- **Spring torque reconciled:** the old numbers (0.06 N m per degree over 12 degrees is 0.72 N m, yet 1.2 N m closed)
  hid a 0.48 N m preload. The spring is now stated with its preload and matches the code: 0.04 N m per degree, wound
  10 degrees on the open stop (0.40 N m), 30 degrees closed: 0.40 + 0.04 x 20 = 1.20 N m, less the lever's 0.18 N m, so
  5.1 N at the grip with no cable in.
- **Cable mass corrected:** the cable is 12 cm long with its top 3 cm moulded into the plug, so it is 0.12 m x 340 g/m
  = 40.8 g of cable plus 5.4 g of ABS in the plug (16 x 16 x 30 mm less the cable through it, at 1.05 g/cm³): 46.2 g,
  not 39 g. In the simulation the cable body is 45.1 g with its centre of mass 6.7 cm up, and the 12 insulation rings
  are 0.096 g each (48 g/m of PVC), 46.2 g in all.
- **Other numbers corrected while re-checking:** the lever was "a steel bar 24 x 16 mm, 22.5 cm long, 150 g", but a
  solid steel bar that size weighs 0.68 kg and the bar was 24.5 cm long in the code; it is now a steel tube with 1 mm
  walls, 24.5 cm long, 163 g with its rubber sleeve, and the simulated lever has that mass and centre of mass. The
  station is 0.65 kg (it said 1 kg, more than a solid steel block 5 x 5 x 4 cm, 0.79 kg). The "4 cm block" is a 3 cm
  block with a 1 cm funnel on top, and the blades sit at the funnel's throat, 3 cm above the table, where the depth is
  now measured (before, they were at the funnel's rim). The holder's simulated mass is now its 2.6 g of ABS.
- **Simulation changed to match:** the blades' 12 mm opening is now a collision ring at the throat (the cable has to
  pass it, 0.85 mm all round); the closing lever meets the computed cutting torque, so the robot has to press about
  19.7 N; the closed blades hold the cut cable with a joint that breaks at 20 N, so the robot has to pull past 20 N;
  the cut sleeve drops onto the bore's floor. The funnel's steel-on-PVC friction is 0.3.
- **Re-run:** demo 1.0 three times in a row on this version: two debug runs (each time the end 9.9 mm below the blades
  when they closed, cut to 10 mm; the lever pushed through up to 2.56 N m of cutting torque; the cable came free once
  the arm's aim had risen about 3 cm) and the batch demo (514 steps). Dream-1 0.0 (1,200 steps). One-hand check 0.25
  (the cable let go in the station dropped to the bore's floor, 25.0 mm below the blades, all 12 rings were cut and the
  cable held, and letting the lever up put the insulation back on the cable).
