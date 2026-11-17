# Crimp a tube lug onto a 16 mm² cable with a bench-held battery crimper, then lay the cable back on its rest

A battery-powered hydraulic lug crimper (a CM-300G: 75 kN, dies for 16 to 300 mm², 18 V) lies on its side in a
plywood bench holder, strapped down, its head on the left inside a clear polycarbonate guard. Its head, which swivels on
its round neck as battery crimpers' heads do, is turned so its opening faces up. In the fixed half of its 16 mm² die lies a
tinned-copper tube lug for 16 mm² cable (a Klauke 3R8: barrel 5.5 mm inside, 8.5 mm outside, 13 mm long), its barrel
pointing at the arms, its palm on a lug holder behind the head. The crimper's grip points away from the arms on the
right, with its trigger at the top of the grip. A short 16 mm² single-core PVC cable (7 mm across; its stranded copper,
5.1 mm across, stripped 13 mm, the barrel's length) lies on two V-rests to the left. The instruction gives only the
goal. In the demo, one hand picks up the cable, pushes its bare end straight through the guard's feed hole into the
barrel until the insulation, too wide to enter, stops on the barrel's mouth, and holds it there; the other hand closes
round the crimper's grip and squeezes the trigger, as a hand does, and holds it while the ram closes the die on the
barrel; the ram goes back and the hand lets go. Then the first hand pulls the cable out with the lug crimped on its end,
holds it up to show it and lays it back on its rests, and both arms go back to where they started. The task is one
task file, [`crimp_a_terminal_onto_a_cable_end.py`](crimp_a_terminal_onto_a_cable_end.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: inserting a cable fully and straight into a close-fitting lug barrel (0.2 mm of play round the copper)
through a guard's feed hole, holding it there while the die closes, working the crimper's trigger for the whole cycle,
and taking the crimped cable out and putting it back where it lay.

Why it matters: power conductors of this size are terminated with crimped compression lugs, which then bolt onto
breakers, busbars, transformers and motor terminals. Lugging a cable end is part of the O*NET task statements for
electricians "Connect wires to circuit breakers, transformers, or other components" and "Install ground leads and
connect power cables to equipment, such as motors" (O*NET 47-2111.00). Electricians held about 821,000 US jobs in
2025 (BLS Occupational Outlook Handbook), and electrical and electronic equipment assemblers, who build control
panels and switchgear, another 246,300. A battery-powered hydraulic crimper with interchangeable dies is a
standard electrician's tool for lugs of this size: the lug sits in the die, the stripped end goes into the barrel, and the
trigger is held until the die has closed. Hydraulic crimping tool manuals give that order, for example ABB's H-CK 400:
"Place connector between the dies", "Insert the conductor into the connector", "Continue ... until the compression is
complete". Getting the strip length and the insertion right (the copper fully in the barrel, the insulation at its
mouth, the cable straight while the die closes) is what separates a crimp that passes the pull test from one that pulls
out or runs hot.

Two hands: as a person does it, one hand holds the cable in the barrel while the other works the tool. The crimper's
trigger is on the right and the cable's rests on the left, so the demo's left hand holds the cable and the right
squeezes the trigger. The fit itself keeps an inserted cable in line: with the left hand letting go right after
pushing the cable in, the crimp still caught it 12.7 mm deep and 2.3 degrees off its axis (that check scored 0.5,
since nothing then took the cable out). So the second hand is set by where the parts are, as at a real bench.

## The real hardware

Specifications and prices were read on the pages linked (2026-11-16/17). They are the sellers' and makers' published
figures; the parts we make are given by drawings below.

| Item | Product | Price | Specifications relied on |
| --- | --- | --- | --- |
| Battery hydraulic lug crimper, listed with its dies, two packs and charger | CM-300G "16-300mm Mini Hydraulic Battery Powered Cable Crimping Tool", Zhejiang Proudstools Machinery ([listing](https://proudstools.en.made-in-china.com/product/MJKYXqsulbUw/China-16-300mm-Mini-Hydraulic-Battery-Powered-Cable-Crimping-Tool.html)) | US$110-125 (one piece) | "Crimping Range: 16-300mm2; Crimping Force: 75kN; Stroke: 17mm; Voltage: DC18V/3.0ah; Accessories: Battery:2PC,Charger:1PC; Crimping Die: 16/25/35/50/70/95/120/150/185/240/300mm2"; package 58 x 44 x 15 cm, 10 kg. Pistol-shaped: head, neck, body with the motor, grip with the trigger at its top, 18 V pack under the grip (the listing's photos). |
| 16 mm² die | The 16 mm² pair of the tool's die set | listed with the tool | Crimps 16 mm² copper tube lugs, matching the lug below. Both halves are marked "16" with a paint pen at setup, so the size reads on the overhead camera. |
| Lug | Klauke 3R8, tubular cable lug, Cu tinned, 16 mm², M8 ([datasheet](https://static.rapidonline.com/pdf/04-6164_v1.pdf); [R-series features](https://www.bossard.com/au-en/eshop/compression-cable-lugs/klauke/p/27719)) | about $1 each | "3R8 16 16 M8 15 8,5 9,5 5,5 8,5 8,5 13 29 0,93": barrel 5.5 mm inside (d1), 8.5 mm outside (d4), 13 mm long (a); palm 15 mm wide (b); 8.5 mm hole (d2); 29 mm long (l); 9.3 g each. R-series lugs: "Simple cable entry due to precise internal chamfer" (modelled as 0.5 mm at 45 degrees, a design value). |
| Cable | H07V-R 16 mm² single-core PVC cable, black, cut 16 cm, stripped 13 mm | about $2 a metre | Class 2 stranded copper (7 strands of about 1.7 mm, 5.1 mm across), 1 mm PVC, 7 mm overall. |
| Bench holder | 18 mm plywood, two 4-inch C-clamps, two 25 mm hook-and-loop straps, wood screws (drawing below) | about $30 | Holds the tool on its side, axis 75 mm above the bench. |
| Guard | 3 mm clear polycarbonate sheet, cut to five pieces (drawing below) | about $15 | |
| Lug holder and cable rests | 3D-printed (PLA) or cut from hardwood (drawing below) | about $2 | |
| Checks | Kitchen scale; hanging scale to 50 kg | about $25 | Trigger force check; pull test. |

Total: about $210 with ten lugs and 2 m of cable, the crimper kit included. The same maker's EZ-300 (60 kN, 17 mm
stroke, the same eleven die sizes, 18 V 5.0 Ah; US$230-250,
[listing](https://proudstools.en.made-in-china.com/product/ZnaRjULHskrY/China-Ez-300-Orange-Battery-Powered-Hydraulic-Pliers-16-300-Sqmm-Hydraulic-Crimping-Tools.html))
works the same way and fits the same holder with the saddles' heights adjusted (about $335 in all).

The trigger is a cordless-tool trigger switch. Switches of this kind are rated at most 20 N over their whole travel
(Omron's C3AW, a trigger switch for brushless cordless tools: "Total operating force TTF max. 20 N", main switch on at
2.0 mm, total travel 8.5 mm). Before the first session, rest the tool's grip on a kitchen scale and press the trigger
until the motor runs: the reading must be no more than 2.0 kg (20 N). The YAM's fingers give about 40 N each, and a
squeeze across trigger and grip is balanced within the hand, so the arm itself carries no load.

The crimp force is set by the tool's pressure valve, which retracts the piston at the maximum operating pressure
(above), not by the battery's charge: a full pack and a half-empty one
crimp alike. A pack too flat to reach the valve's pressure stops short of a closed die, which leaves a partial
impression, scored as no crimp. Each session starts with a charged pack (the kit has two; the spare charges between
sessions).

Drawings of the parts we make (millimetres; x forward from the arms, y to the left, z up; the die's centre is the
origin in x and y):

- Bench holder: an 18 x 350 x 330 base (x from -100 to +250, y from -250 to +80), clamped to the bench at its two far
  corners with the C-clamps. On it, saddles glued and screwed under the tool: under the body (60 x 80 x 19.5, centred
  at x 0, y -170), under the battery (50 x 80 x 19.5, at x +190, y -170), under the motor (40 x 80 x 33, at x -60,
  y -170) and under the head (30 x 100 x 12, at x 0, y -10). The tool lies on them with its axis 75 above the bench,
  its grip pointing to +x. Two hook-and-loop straps, screwed to the base on both sides, go over the body (at y -150)
  and the battery (at x +190).
- Lug holder, behind the head: a palm rest 6.5 (x) by 20 (y) by 55.5 tall from the base, under the lug's palm (its top
  1.5 below the barrel's axis), and a stop 16 (x) by 24 (y) by 63 tall from the base, whose front face meets the palm's
  end at x +22.5.
- Guard, 3 mm clear polycarbonate, standing on the base round the head: front wall 168 x 132 (y by z) at x -27.5 with
  a 22 x 22 square feed hole centred on the barrel's axis (y 0, 57 above the base); back wall 168 x 132 at x +48; left
  wall 75.5 x 132 (x by z) at y +70; right wall 75.5 x 132 at y -98, with a 52 x 52 cut-out for the tool's neck
  (centred 57 above the base); lid 75.5 x 168 on top, hinged to the left wall with tape so a lug can be laid in. Join
  the corners with clear tape or 15 mm angle brackets.
- Cable rests: two V-blocks 16 (x) by 30 (y) by 26.5 tall, each with two 6 x 6 lips 16 long on top whose inside faces
  are 8 apart, standing 45 and 130 behind the cable's tip (its centre of mass, 80 behind the tip, lies between them),
  so that the cable's axis is 30 above the bench, its tip at x -90, y +100.

## How the crimper works, and how the task models it

The real cycle: squeezing the trigger runs the motor and pump; the ram drives the moving half of the die across the
head onto the barrel, against the fixed half, and the tool's pressure valve opens once the crimp has reached full force;
the ram then goes back and the die opens. A person holds the tool by its grip and squeezes the trigger with the fingers
of the same hand; here the holder takes the tool's weight and the robot's hand closes round the grip and squeezes the
trigger in the same way, its two fingers on the trigger's face and on the grip's back, 62 mm apart.

Battery crimpers of this class are listed with this cycle, for example the PZ-300
([listing](https://igeelee.en.made-in-china.com/product/pKiEzJSdJmAX/China-Pz-300-Mini-Battery-Cable-Lug-Terminal-Crimper-for-16-300mm2.html)): "All tools can be controlled by one trigger"; "an automatic retraction which returns the piston
into [its] starting position" once the maximum operating pressure is reached; "Crimping cycle: 3-6s"; "Stroke: 17mm";
"Accessories: crimping Dies. 16, 25, 35, 50, 70, 95, 120, 150, 185, 240,300mm2".

The simulation reproduces this cycle:

- The trigger is a body on a sprung slide at the top of the grip, 8.5 mm of travel: 6.0 N at rest, 8.4 N at 2.0 mm,
  where the motor switches on, and 16.2 N at its stop (design values inside the 20 N such switches are rated for).
- With the trigger in past its switch point, the ram closes the die over its 17 mm stroke in 3 s (a design value; tools
  of this class are listed at 3 to 6 s a crimp). Let go before the die has closed, and the ram stops where it is. Once
  the die has closed, the ram goes back over 2 s, held trigger or not, and runs again only after the trigger is let go.
  The die's first full closing is the crimp.
- The demo's right hand comes down open round the grip, closes, squeezing the trigger to its stop, holds it 3.4 s,
  longer than the die takes to close, and opens; the ram goes back and the die opens.
- The forces at the die are the tool's: the robot never supplies the crimping force, only the trigger's.

## Real-trial safety and rig protection

- Pinch point: the die is the only pinch point, and it moves only while the trigger is held in. It sits inside the
  clear guard; the guard's 22 mm feed hole passes the 7 mm cable but not a gripper finger (a YAM fingertip is about
  24 mm across), and the lid stays closed during a trial. The demo holds the cable 7.5 cm behind its tip, so the hand
  holding it stays outside the guard, about 2.5 cm from it. The trigger and grip are outside the guard.
- Stop: letting go of the trigger stops the ram; pulling the battery pack stops the tool.
- Tool and holder: the kit weighs 10 kg packed; the tool lies on its side, low, on three saddles, strapped down, on a
  base clamped to the bench. The largest push the robot puts on it is the cable's insertion, a few newtons, and the
  trigger squeeze, which is balanced within the hand.
- Pads: the grippers touch only the cable's PVC insulation and the tool's plastic trigger and grip, never the bare
  copper strands, the lug or the die. The stripped strands are twisted tight at reset so none fray against the pads.
  Thin nitrile pad covers (cut from nitrile gloves) may be fitted; wipe the pads with isopropyl alcohol at the end of a
  session. No oil, grease or liquid is used on the rig.
- The pull test is done by a person after the trial, away from the robot: the lug bolted to a fixed M8 stud and a
  hanging scale on the cable.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The holder, the crimper (head, fixed half of the die,
neck, body, motor, grip, battery), the guard and the lug holder form one fixed body; the trigger, the moving half of the
die, the lug (its barrel and palm), the cable's rests and the cable are bodies of their own, made of shapes from
`bodykit.py`.

- The trigger slides on a sprung prismatic joint (a `Slider` from `bodykit.py`), as above. The moving half of the die
  does not collide; the task file moves it as the ram runs. The hydraulics are not modelled.
- The lug is fixed in the die; in a real trial it lies in the fixed half with its palm on the holder against the stop,
  which takes the push of the cable. Its barrel is ten flat panels round a 5.5 mm bore, with a 0.5 mm, 45-degree chamfer
  at the mouth; the 5.1 mm copper has 0.2 mm of play in it.
- The crimp: when the die has closed fully for the first time, the task file reads how deep the cable's bare end is in
  the barrel and how far the cable is tipped from the barrel's axis, and, if the bare end is in the barrel, swaps the
  lug for a crimped one (its barrel drawn squeezed onto the copper, shown only), which from then on it moves with the
  cable after every control step, so the cable comes out with the lug on it. If the bare end is not in the barrel then,
  the barrel is crushed shut (a solid barrel takes its place) and nothing can go in any more. The simulation does not
  test the crimp's hold; a real trial adds a pull test (the lug must hold a steady 450 N pull on the cable, the UL 486A
  pull-out force for this size).
- The guard's clear panels are solid but not drawn (`visible=False` in `bodykit.py`); their edges and the feed hole's
  rim are drawn instead, so the cameras see through the guard as through clear polycarbonate.
- The tool's sizes are scaled from the maker's photos by its 18 V pack (115 mm long): head 152 mm long, die's centre to
  the trigger's face 143 mm and to the grip's back 205 mm, body 75 mm wide.
- Aids for the robot: the holder takes the place of the hand that holds the tool, and the lug holder of the hand that
  sets the lug in the die. The insertion and the crimp, the task's difficulty, are left to the robot.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

Third-party assets: none. Every body is built from primitive shapes in the task file; no meshes, scans or scenes by
others are used.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the cable laid back on its rests and both
arms back where they started, with the trigger and the cable still) has held for 1 s, otherwise `score()` at the end of
the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crimp the cable lug sitting in the crimping tool onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable 7.5 cm behind its tip,
fingers closing across it, lifts it off its rests, lines it up with the barrel 3 cm in front of the guard, brings its
tip through the feed hole to 5 mm and then 0.5 mm short of the barrel's mouth, letting the arm settle at each, then
feels for the mouth as a person would: it presses lightly (aiming 4 mm past the mouth) while moving the tip in widening
circles, out to 1.5 mm off the axis, until the tip drops into the barrel's entry chamfer. It then pushes the bare end
straight in, aiming 2.5 mm past full depth, so that the insulation stopping on the barrel's mouth sets the depth, and
holds it. In a debug run of this demo, 13.0 mm of the 13 mm copper was in the barrel when the die closed, the cable
0.8 degrees off the barrel's axis. The right hand comes down open round the crimper's grip, closes on the trigger and the grip's back,
squeezing the trigger to its stop, holds it 3.4 s while the die closes (the crimp), and opens; the ram goes back, and
the right rises and goes back to where it started. The left waits for the die to open, pulls the cable straight out
through the feed hole with the lug crimped on its end, holds it up to show it, lays it back on its rests and goes back
to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crimp the cable lug sitting in the crimping tool onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/crimp-dream/*.json; do inspect-robots video "$log"; done
```

Set `CRIMP_DEBUG=1` to print what the score is built from (the trigger's travel, the switch, the ram, how deep the bare
end is in the barrel, the cable's angle from the barrel's axis and its direction, how far its tip is off that axis,
whether and how it was crimped, and where each gripper's TCP is from the cable's tip and from the trigger) twice a
second.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (516 s on the clock, with Dream answering in 1.28 s on average), and about 4 minutes for the scripted demo (789 steps, 39.5 s of simulated time, 262 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-NEC2XT. Its review said:

> **Not accepted** (TK-NEC2XT).
>
> Clarity and reproducibility (not met): The least expensive stated setup still costs about $570 and requires existing shop air, leaving the requested affordable route unresolved.
>
> What to change: The named press, die and air supply address the sourcing details, but the affordability point is still open: the least expensive route listed is about $570 and assumes existing shop air. Please provide a repeatable setup costing no more than a few hundred dollars, with a specified matching crimper and die that retain the cable-insertion and crimping operation, and update the demo and setup to match.

Earlier reviews, each done in its revision and kept:

- TK-RMZFXT asked for a named press, matching 16 mm² die and air supply that reproduce the press's cycle, or
  construction files; the press, die, valve and air line were named, and the cycle modelled. This revision replaces that
  press with a battery crimper, so no air supply is needed.
- TK-K6ECAU asked to change the category to Impact on jobs, explain how lug crimping is part of electricians' regular
  work, and reconcile the demo's insertion depth with the minimum for full credit.
- TK-Y3RQVT asked for a matched cable, ring terminal and crimper with a realistic insertion depth, full-credit scoring
  that checks the completed crimp (a pull test, observable insertion marks) and an updated demo.

What changed in response:

- **A crimper and die costing a few hundred dollars, named and matched:** the pneumatic press, its pedal valve and
  the compressor are gone. The crimper is a battery-powered hydraulic lug crimper, the CM-300G (75 kN, dies for 16 to
  300 mm², 17 mm stroke, 18 V 3.0 Ah), listed by its maker at US$110-125 with dies for 16 to 300 mm², two packs and
  a charger; the die is its 16 mm² pair, matching the Klauke 3R8 16 mm² lug. The whole setup costs about US$210 with ten lugs and 2 m
  of cable, and needs no air or shop equipment ("The real hardware"; task.json setup).
- **The cable insertion and the crimp kept:** the lug still lies in the die, and the robot still pushes the stripped
  end through the guard's 22 mm feed hole into the 5.5 mm barrel (0.2 mm of play round the copper) until the insulation
  stops at the mouth, and holds it while the die closes. The crimp is still the die's first full closing, judged by
  the same window: 10.5 to 13.5 mm deep, within 8 degrees. The new demo scores 1.0 (789 of the 1,200 steps); a
  debug run of it measured the crimp at 13.0 mm deep and 0.8 degrees off the axis. Dream-1 scores 0.0.
- **The tool worked as a hand works it:** the second hand closes round the crimper's grip, its fingers on the
  trigger's face and the grip's back, 62 mm apart, and squeezes the trigger to its 8.5 mm stop. The task file models
  the trigger on a sprung slide (6.0 N at rest, 8.4 N at its 2.0 mm switch point, 16.2 N at its stop, inside the 20 N
  such trigger switches are rated for; a YAM finger gives about 40 N), and the ram, which closes the die over its
  17 mm stroke in 3 s while the trigger is held and goes back in 2 s once the die has closed. The scene, the scorer
  ("released": trigger let go, ram back, both TCPs at least 4 cm from the trigger), the demo, setup and scoring changed
  to match. A kitchen-scale check before the first session confirms the trigger runs the tool at 20 N or less.
- **Held in a simply built holder, stable:** the hand-held tool lies on its side, its axis 75 mm up, on saddles on an
  18 mm plywood base clamped to the bench, under two hook-and-loop straps (dimensioned drawing in "The real
  hardware"). The robot's only loads on it are the cable's push, a few newtons, and the trigger squeeze, which is
  balanced within the hand.
- **The die's size marked:** both halves of the 16 mm² die carry a "16" paint-pen mark, checked at each reset (setup,
  reset notes).
- **Charge does not change the result:** the crimp force is set by the tool's pressure valve, not by the pack. A pack
  too flat to reach the valve's pressure stops short of a closed die and leaves a partial impression, scored as no
  crimp (scoring); each session starts on a charged pack (reset notes).
- **Real-trial crimp check kept:** the full hexagonal impression of the closed die by eye, the caliper gap of at most
  2.5 mm at the barrel's mouth, and the 450 N pull test (scoring).
- **Checks run in the simulation:** three copies of the demo, scene and scorer unchanged:
  - the trigger let go 1.0 s into its hold, the ram stopped at 65 % of its stroke: no crimp, score 0.25 (insertion
    only);
  - the trigger squeezed before the cable is in: the die closes on the empty barrel and crushes it, score 0.0;
  - the left hand letting go of the cable right after inserting it: the crimp still caught it, 12.7 mm deep and
    2.3 degrees off the axis, score 0.5 (nothing took the cable out).
