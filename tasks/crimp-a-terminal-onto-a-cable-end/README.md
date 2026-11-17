# Crimp a tube lug onto a 16 mm² cable in a bench crimping press, then lay the cable back on its rest

A pneumatic lug crimper (an iCrimp AM-70, hex type) stands on the bench, pointing at the arms, its jaws inside a clear
polycarbonate guard. In the lower half of its open 16 mm² hexagonal die lies a tinned-copper tube lug for 16 mm² cable
(a Klauke 3R8: barrel 5.5 mm inside, 8.5 mm outside, 13 mm long), its barrel running left to right, its palm against a
lug stop. The press's pedal valve (a Camozzi 354N-925) stands on the bench to the right of the press, since the robot
has no feet, and a short
16 mm² single-core PVC cable (7 mm across; its stranded copper, 5.1 mm across, stripped 13 mm, the barrel's length)
lies on two V-rests to the left. The instruction gives only the goal. In the demo, one hand picks up the cable, pushes
its bare end straight through the guard's feed hole into the barrel until the insulation, too wide to enter, stops on
the barrel's mouth, and holds it there; the other presses the pedal valve down and holds it while the press's jaws
close and crimp the lug onto the cable, then lets it go, and the jaws open. Then the first hand pulls the cable out
with the lug crimped on its end, holds it up to show it and lays it back on its rests, and both arms go back to where
they started. The task is one task file,
[`crimp_a_terminal_onto_a_cable_end.py`](crimp_a_terminal_onto_a_cable_end.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: inserting a cable fully and straight into a close-fitting lug barrel (0.2 mm of play round the copper)
through a guard's feed hole, holding it there while the press closes, working the press's valve for the whole cycle,
and taking the crimped cable out and putting it back where it lay.

Why it matters: power conductors of this size are terminated with crimped compression lugs, which then bolt onto
breakers, busbars, transformers and motor terminals. Lugging a cable end is part of the O*NET task statements for
electricians "Connect wires to circuit breakers, transformers, or other components" and "Install ground leads and
connect power cables to equipment, such as motors" (O*NET 47-2111.00). Electricians held about 821,000 US jobs in
2025 (BLS Occupational Outlook Handbook), and electrical and electronic equipment assemblers, who build control
panels and switchgear, another 246,300. Where lugs are crimped in quantity, in panel and cable-assembly shops, it is
done on a bench press like this one, its pedal valve worked while the hands hold the parts. Getting the strip length
and the insertion right (the copper fully in the barrel, the insulation at its mouth, the cable straight while the die
closes) is what separates a crimp that passes the pull test from one that pulls out or runs hot.

Two hands: the press's pedal valve is to the right of the press and the cable's rests to the left, so the demo holds
the cable in the barrel with the left hand while the right works the valve. The fit itself keeps an inserted cable in
line: with the left hand letting go right after pushing the cable in, the crimp still caught it 12.7 mm deep and
2.3 degrees off its axis (a good crimp; that check then scored 0.5 only because nothing took the cable out afterwards).
So the second hand is not forced by the physics here, only by where the parts are.

## The real hardware

Prices and specifications were read on the pages linked (2026-11-16/17), except the compressor's, which are from
retailers' search listings. They are the sellers' and makers' published figures, not our own measurements. Values
marked "assumed" are not published; the trial does not depend on their exact value, and each can be checked on the
real unit.

| Item | Product (part number) | Price | Specifications relied on |
| --- | --- | --- | --- |
| Pneumatic lug crimper | iCrimp AM-70 Pneumatic Cable Lug Crimping Machine, hex type, 6-70 mm² ([icrimptools.com](https://www.icrimptools.com/products/icrimp-pneumatic-crimp-tool-for-6-70mm-battery-cable-lugs)); the same AM-70 from its maker ([listing](https://www.chinese-equipment.com/2-0t-pneumatic-terminal-crimping-machine-tool-am-70-crimping-non-insulated-cable-lugs-6-70-mm2-heavy-duty-cold-clamp-crimper)) | $509.99 | iCrimp: "AM-70 Pneumatic Cable Lug Crimping Machine, 6–70 mm² — Die Options $59.99 – $509.99". Maker: "Model: AM-70 Applicable scope: 6-70mm2 Appearance size: 510*150*195mm Weight: 18 (Kg) Working pressure: 2.0T ... Suggested inner diameter of trachea: 10 ... Air pressure range: 0.7-0.95MPA Shape after pressing: Hexagon/dot/B-shape", variant "AM-70 Hexagon Type". Worked by a pedal valve; jaws holding interchangeable round die pairs. |
| 16 mm² die | The AM-70 hex-type die set, supplied with the press: hexagonal die pairs marked G6/G50, G10/G35, G16/G25 and G70; the task uses the 16 mm² (G16) cavity | included | Hexagonal crimp for 16 mm² copper tube lugs, matching the lug below. |
| Pedal valve | Camozzi 354N-925, Series 3 pneumatic pedal, 5/2, G1/4 ([Acorn Industrial](https://www.acorn-ind.co.uk/p/camozzi/valves/354n-925-camozzi); [datasheet](https://docs.rs-online.com/059c/A700000015768714.pdf)) | $131.17 | "Actuating force at 6 bar = 17N; Operating pressure = 2,5 ÷ 8 bar; Flow rate = 650Nl/min"; 245.9 x 138 mm, body 67.5 mm, 152.9 mm at its high end; "can be operated as bistable or monostable" (set to monostable); "Construction Spool-type". At the trial's 7.5 bar its force may be a little above the published 17 N at 6 bar; the kitchen-scale check below covers it. |
| Air supply | Any shop compressor that keeps its tank above 0.75 MPa (109 psi), for example a 150 psi pancake compressor such as the Porter-Cable C2002, 6 gal, oil-free | about $140-180 | 150 psi (1.03 MPa) maximum, 2.6 SCFM at 90 psi (retail listings); its regulator set to 0.75 MPa, inside both the press's 0.7-0.95 MPa and the valve's 0.25-0.8 MPa. A 120 psi compressor may cycle below 0.75 MPa. |
| Air line | 10 mm PU tube (3 m), three G1/4 push-in fittings for 10 mm tube (valve ports 1, 2 and 4), two G1/4 silencers (ports 3 and 5), one 1/4 NPT industrial plug with a 1/4 NPT push-in fitting at the compressor | about $30 | Ordinary pneumatic fittings. Port 1 from the compressor; ports 2 and 4 to the press's two cylinder ports (its "close" and "open" sides). |
| Lug | Klauke 3R8, tubular cable lug, Cu tinned, 16 mm², M8 ([datasheet](https://static.rapidonline.com/pdf/04-6164_v1.pdf); [R-series features](https://www.bossard.com/au-en/eshop/compression-cable-lugs/klauke/p/27719)) | about $1 each | "3R8 16 16 M8 15 8,5 9,5 5,5 8,5 8,5 13 29 0,93": barrel 5.5 mm inside (d1), 8.5 mm outside (d4), 13 mm long (a); palm 15 mm wide (b); 8.5 mm hole (d2); 29 mm long (l); 0.93 kg per 100 (9.3 g each). R-series lugs: "Simple cable entry due to precise internal chamfer" (its size is not published; the simulation assumes 0.5 mm at 45 degrees). |
| Cable | H07V-R 16 mm² single-core PVC cable, black, cut 16 cm, stripped 13 mm | about $2 a metre | Class 2 stranded copper (7 strands of about 1.7 mm, 5.1 mm across), 1 mm PVC, 7 mm overall. |
| Guard | 3 mm clear polycarbonate sheet, cut to four pieces (below) | about $15 | |
| Lug stop and rests | Three small blocks, 3D-printed or cut from hardwood (below) | about $2 | |

Total: about $860. Before the first session, check the pedal valve on a kitchen scale: stand it on the scale and
press its high end down until the jaws move; the reading should be no more than about 2.0 kg (20 N). Low-cost route:
where the lab has a 0.75 MPa shop air line, the compressor is not needed (about $700), and the AM-70's own foot valve,
supplied with it, can replace the Camozzi valve if it passes the same check (about $570). The press is the one item
above a few hundred dollars. A cheaper bench tool cannot do this job for a robot: a 16 mm² hex crimp needs kilonewtons
at the die (the AM-70 is rated 2.0 t), which manual ratchet and hydraulic lug crimpers get from a person's full grip on
long handles or from many pump strokes, far beyond the YAM's roughly 40 N per finger, while the pneumatic press needs
only 17 N on its valve. The AM-10, a smaller pneumatic crimper ($219.99 from iCrimp, $137.90 from VEVOR), is listed at
13.5 kN for non-insulated terminals up to 16 mm² "with the correct die", but we found no hexagonal die for 16 mm² tube
lugs listed for it, so it is not used.

Drawings of the parts we make (millimetres; x forward from the arms, y to the left, z up):

- Guard, 3 mm clear polycarbonate, standing on the bench round the jaws and butting against the press's housing:
  left wall 195 x 160 (x by z), with a 22 x 22 square feed hole whose centre is 65 from its front edge and 75 above the
  bench (on the barrel's axis), its inside face 18 left of the barrel's mouth; right wall 195 x 160, 48 right of the
  centre line; front wall 75.5 x 160 (y by z) at 205 forward; lid 195 x 75.5 on top, hinged to the left wall with
  tape so a lug can be laid in. Join the corners with clear tape or 15 mm angle brackets.
- Lug stop: a block 24 (x) by 16 (y) by 81 (z), standing on the bench at the end of the lug's palm, its left face
  touching the palm's end (22.5 right of the centre line).
- Cable rests: two V-blocks 30 (x) by 16 (y) by 26.5 tall, each with two 6 x 6 lips 16 long on top whose inside faces
  are 8 apart, standing 45 and 130 from the cable's tip (its centre of mass, 80 from the tip, lies between them) so
  that the cable's axis is 30 above the bench.

## How the press works, and how the task models it

The real cycle (the press's cylinder takes air on both sides, one tube from the valve to each; the pedal valve is a
5/2 valve set to monostable): pressing the pedal's high end down past its switch point sends air to the cylinder's
closing side, and the jaws close and crimp; holding it down keeps them closed; letting it go springs the pedal back up,
the valve sends air to the opening side, and the jaws open. A person works the pedal with a foot; the robot has no
feet, so the pedal stands on the bench and a hand presses it. Nothing else changes: the same valve, the same force and
the same cycle.

The simulation reproduces this cycle:

- The pedal's cover is a body on a hinge at its low end, with a spring, between stops at rest and 6 degrees down. Its
  spring (0.14 N m per degree, preloaded 26 degrees past rest) and the cover's weight (0.25 kg, centre 12 cm from the
  hinge) give a press of 14.5 N at rest, 17.0 N at the switch point (5 degrees, 2.0 cm down at the press point, 23.1 cm
  from the hinge) and 17.5 N at the bottom stop (2.5 cm down): the 17 N is the maker's actuating force, and the travel
  (switch at 5 degrees, stop at 6) is assumed.
- With the cover past its switch point the valve is on, and the upper jaw closes over 1 s (assumed; not published);
  with it back above, the jaw opens over 1 s. The jaws' first full closing is the crimp.
- The demo's right hand presses the flat at the pedal's high end with closed fingers, aiming 7 cm below where the
  flat rests; the cover goes down to its bottom stop and stays there while the hand holds it, 1.8 s, longer than the
  jaws take to close; then the hand rises.
- Forces at the die are the press's: the robot never supplies the crimping force, only the valve's 17 N, within what
  a YAM arm gives.

## Real-trial safety and rig protection

- Pinch point: the jaws are the only pinch point, and they move only while the pedal is held down. They sit inside
  the clear guard; its 22 mm feed hole passes the 7 mm cable but not a gripper finger (a YAM fingertip is about 24 mm
  across), and the lid stays closed during a trial. The demo holds the cable 7.5 cm back from its tip, so the hand
  holding it stays outside the guard, about 3 cm from it.
- Air: the regulator is set to 0.75 MPa before each session; closing the compressor's outlet valve or pulling the quick
  coupler stops the press. The two silencers keep the valve's exhaust quiet.
- Pads: the grippers touch only the cable's PVC insulation and the pedal's plastic cover, never the bare copper strands,
  the lug or the die. The stripped strands are twisted tight at reset so none fray against the pads. Thin nitrile pad
  covers (cut from nitrile gloves) may be fitted, since the task needs no more grip than holding a 27 g cable and
  pressing a 17 N pedal; wipe the pads with isopropyl alcohol at the end of a session. No oil, grease or liquid is used.
- The pull test is done by a person after the trial, away from the robot: the lug bolted to a fixed M8 stud and a
  hanging scale on the cable.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The press (housing, handle, lower jaw and die, the
guard, the lug stop), the upper jaw, the lug (its barrel and palm), the pedal's body and cover, the cable's rests and
the cable are rigid bodies made of shapes from `bodykit.py`; the press, the lug and the rests are kinematic, fixed where
they are put.

- The pedal's cover turns on a hinge (a `Hinge` from `bodykit.py`), as above. The upper jaw does not collide; the task
  file turns it (15 degrees open to closed) as the valve drives it. The air cylinder is not modelled.
- The lug is fixed in the die; in a real trial it lies in the lower die with its palm against the stop, which takes the
  push of the cable. Its barrel is ten flat panels round a 5.5 mm bore, with a 0.5 mm, 45-degree chamfer at the mouth
  (assumed size, as above); the 5.1 mm copper has 0.2 mm of play in it.
- The crimp: when the jaws have closed fully for the first time, the task file reads how deep the cable's bare end is
  in the barrel and how far the cable is tipped from the barrel's axis, and, if the bare end is in the barrel, swaps the
  lug for a crimped one (its barrel drawn squeezed onto the copper, shown only), which from then on it moves with the
  cable after every control step, so the cable comes out of the press with the lug on it. If the bare end is not in the
  barrel then, the barrel is crushed shut (a solid barrel takes its place) and nothing can go in any more. The
  simulation does not test the crimp's hold; a real trial adds a pull test (the lug must hold a steady 450 N pull on the
  cable, the UL 486A pull-out force for this size).
- The guard's clear panels are solid but not drawn (`visible=False` in `bodykit.py`); their edges and the feed hole's
  rim are drawn instead, so the cameras see through the guard as through clear polycarbonate.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

Third-party assets: none. Every body is built from primitive shapes in the task file; no meshes, scans or scenes by
others are used.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the cable laid back on its rests and both
arms back where they started, with the pedal and the cable still) has held for 1 s, otherwise `score()` at the end of
the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crimp the cable lug sitting in the crimping press onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable 7.5 cm from its tip,
fingers closing across it, lifts it off its rests, lines it up with the barrel 3 cm outside the guard, brings its tip
through the feed hole to 5 mm and then 0.5 mm short of the barrel's mouth, letting the arm settle at each, then feels
for the mouth as a person would: it presses lightly (aiming 4 mm past the mouth) while moving the tip in widening
circles, out to 1.5 mm off the axis, until the tip drops into the barrel's entry chamfer. It then pushes the bare end
straight in, aiming 2.5 mm past full depth, so that the insulation stopping on the barrel's mouth sets the depth, and
holds it. Measured in the demo (`CRIMP_DEBUG=1`): 12.9 mm of the 13 mm copper in the barrel when the jaws
closed (inside the 10.5 to 13.5 mm window for full credit), so the insulation's end 0.1 mm from the mouth, and the
cable 1.3 degrees off the barrel's axis. The right hand comes straight down on the flat of the pedal's high end,
fingers closed, presses it down to its stop and holds it there 1.8 s while the jaws close (the crimp), and rises; the
pedal springs up, the jaws open, and the right goes back to where it started. The left waits for the jaws to open,
pulls the cable straight out through the feed hole with the lug crimped on its end, holds it up to show it, lays it back
on its rests and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crimp the cable lug sitting in the crimping press onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/crimp-dream/*.json; do inspect-robots video "$log"; done
```

Set `CRIMP_DEBUG=1` to print what the score is built from (the pedal's angle, the valve, the jaws, how deep the bare end
is in the barrel, the cable's angle from the barrel's axis and its direction, how far its tip is off that axis, whether
and how it was crimped, and where each gripper's TCP is from the cable's tip and from the pedal's press point) twice a
second.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (470 s on the clock, with Dream answering in 0.67 s on average), and about 4 minutes for the scripted demo (755 steps, 37.8 s of simulated time, 257 s on the clock, with other simulations sharing the GPU). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-RMZFXT. Its review said:

> **Not accepted** (TK-RMZFXT).
>
> Clarity and reproducibility (not met): The main pneumatic press mechanism and air supply have neither a named reproducible source nor construction details.
>
> What to change: The earlier category, job-context, matched-component and demo-depth points are addressed. One new issue was not raised before: please identify an affordable commercially available pneumatic crimping press, matching 16 mm² die and air supply that reproduce the specified lever-operated cycle, or provide construction files and complete operating specifications for an affordable equivalent.

The review before that (TK-K6ECAU) asked to change the category to Impact on jobs, explain how lug crimping is part of
electricians' regular work, and reconcile the demo's insertion depth with the minimum for full credit; that was done in
the previous revision and is kept. The first review (TK-Y3RQVT) asked for a matched cable, ring terminal and crimper
with a realistic insertion depth, full-credit scoring that checks the completed crimp (a pull test, observable
insertion marks) and an updated demo; that was done then and is kept, now with named parts.

What changed in response:

- **A named, commercially available pneumatic press:** the iCrimp AM-70 pneumatic cable lug crimper, hex type,
  6-70 mm² ($509.99). Its published figures are quoted with links in "The real hardware": 510 x 150 x 195 mm, 18 kg,
  2.0 t, 0.7-0.95 MPa, hexagonal crimp, worked by a pedal valve, 10 mm air tube. The scene's press is now this press
  at its published size: an orange housing holding the air cylinder, 51 cm long, pointing at the arms, with its jaws
  in front, inside a clear guard.
- **The matching 16 mm² die:** the 16 mm² cavity of the AM-70's hexagonal die set (die pair G16), supplied with the
  press. The lug is now a named lug matched to it, the Klauke 3R8 (16 mm², M8). The simulated lug has the maker's
  dimensions: barrel 5.5 mm inside, 8.5 mm outside and 13 mm long; 29 mm overall; palm 15 mm wide. The cable is
  H07V-R 16 mm² (5.1 mm of stranded copper, 7 mm over the PVC, 27 g for 16 cm), stripped 13 mm to the barrel's length.
- **The air supply:** a shop compressor that keeps its tank above 0.75 MPa, such as a 150 psi pancake compressor
  (Porter-Cable C2002, about $140-180). Its regulator is set to 0.75 MPa, inside both the press's 0.7-0.95 MPa and the
  valve's 0.25-0.8 MPa. The 10 mm air line and its ordinary fittings are listed port by port.
- **The cycle, reproduced in the task itself:** the real press has no lever. It is worked by a 5/2 pedal valve: held
  down, the jaws close and crimp; let go, they open. The lever, which carried the upper die, is gone from the scene,
  the scorer and the demo. They now model the pedal valve, a Camozzi 354N-925, whose actuating force is published as
  17 N at 6 bar:
  - its sprung cover needs 14.5 N at rest, 17.0 N at its switch point (2.0 cm down) and 17.5 N at its stop (2.5 cm
    down);
  - past the switch point the upper jaw closes over 1 s; let go, it opens over 1 s;
  - the crimp is the jaws' first full closing.

  The robot has no feet, so the pedal stands on the bench and a hand presses it. The demo's right hand holds it at its
  stop for 1.8 s, then lets go.
- **Construction details for what we make:** dimensioned drawings of the clear guard with its feed hole, the lug stop
  and the cable rests.
- **Affordability:** about $860 in all, of which the press is $510. Low-cost route: shop air in place of the compressor
  (about $700), and the press's own pedal valve in place of the Camozzi if a kitchen scale shows it switches at no more
  than 20 N (about $570). No cheaper tool works for a robot: manual 16 mm² lug crimpers need a person's full grip on
  long handles or many pump strokes, far beyond a YAM's roughly 40 N per finger, while the pneumatic press needs 17 N
  on its valve.
- **Also made real:** the 2 cm flared lead-in in front of the lug's mouth is gone, since no real press or lug has one.
  The cable now goes through the guard's 22 mm feed hole into the lug's 5.5 mm barrel, with 0.2 mm of play round the
  copper; the only entry aid is the lug's own internal chamfer (Klauke: "precise internal chamfer"; modelled as
  0.5 mm). A new section, "Real-trial safety and rig protection", covers the guarded pinch point, the air shut-off and
  the grippers' pads.
- **Re-run:** the new demo scores 1.0 (755 steps). In a debug run of it, 12.9 mm of the 13 mm copper was in the
  barrel when the jaws closed (inside the 10.5 to 13.5 mm window), the cable 1.3 degrees off the barrel's axis; the
  debug runs while the insertion was tuned measured 12.9 mm each time. A new Dream-1 run scores 0.0.
