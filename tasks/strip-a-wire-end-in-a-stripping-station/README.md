# Strip 10 mm of insulation off a cable end in a lever-operated benchtop stripping station

A steel benchtop wire-stripping station stands on the table: a block with a hole in its top, whose blades close round
whatever is in the hole while its lever is held down. A short single-core 35 mm² cable of type H07V-K (a fine-stranded
copper core 7.9 mm across under 1.2 mm of PVC insulation, 10.3 mm across overall) with a plastic plug on one end
stands upright beside it, its insulated end in a holder's 12 mm bore (0.85 mm of clearance all round). One hand takes
the cable by its plug and puts its free end 10 mm down into the hole; the other presses the lever down, closing the
blades, and keeps it down while the first pulls the cable straight back out. The cut insulation stays in the station
and 10 mm of copper is bare. The task is one task file,
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

Why two hands: the blades hold the cut insulation only while the lever is held down, and the cable has to be pulled
out at the same time; if the lever comes up first, the cut insulation comes back out on the cable. One hand keeps the
lever down while the other pulls the cable, so a single hand cannot strip it. Checked: with the cable let go in the
hole and the other hand pressing the lever, the cable dropped to the hole's bottom (33.8 mm deep), the closing blades
cut all 24 mm of insulation, and letting the lever up put it back on the cable: 0.25.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The station (a block with a funnel and a tube down
into it) and the cable's holder are kinematic, fixed where they are put; the lever turns on a hinge (a `Hinge` from
`bodykit.py`) with a spring that holds it up. The cable is a rigid body; the insulation over its last 24 mm is 12 rings
of 2 mm, each held to the cable by a fixed joint (a `Glue` from `bodykit.py`), shown but not colliding. For contact,
that insulated end is one hidden cylinder of the cable's full 10.3 mm width, part of the cable itself, so the end
stands in the holder's 12 mm bore with its real clearance and goes into the station's 16 mm hole as the insulated cable
does. It stays on after the cut; the bare end touches nothing later in the task.

- The cut: the first time the lever closes with the cable's end in the hole, the task file switches the rings below
  the blades (the station's top) from the cable to the station, fixed where they are.
- Stripped: if the cable's end comes up out past the blades while the lever is still down, the cut rings stay in the
  station. If the lever comes up first, they go back onto the cable.

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
across it, lifts the cable straight out of its holder, carries it over the station and lowers its end 10 mm into the
hole (measured: 9.6 mm below the blades when they closed, cut to the nearest 2 mm: 10 mm). The right hand comes
straight down on the lever's rubber grip, fingers closed, presses the lever down and holds it while the left pulls the
cable straight up out of the hole. Then the right rises and the lever springs back up:

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

Set `STRIP_DEBUG=1` to print what the score is built from (the lever's angle, how deep the cable's end is below the
blades and how far off the hole's axis, how many rings were cut, whether it is stripped, and where each gripper's TCP
is from the cable's end and from the lever's grip) twice a second.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,200 steps (387 s on the clock, with Dream answering in 1.01 s on average), and about 2 minutes for the scripted demo (514 steps, 25.7 s of simulated time, 102 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-UEJBT7. Its review said:

> Flags: Earlier review feedback was not supplied
>
> Importance (not met): Stripping cable is regular work for electricians and team assemblers, but this scene does not show robot building or data-center work.
>
> Clarity and reproducibility (not met): The 12 mm cable is specified to start inside an 11 mm holder bore, leaving the initial configuration physically inconsistent.
>
> What to change: Change the domain to Impact on jobs and explain how cable stripping fits regular electrical-assembly work. Correct the holder bore or cable diameter so the specified starting position is physically reproducible, and reconcile the stated core diameter with the 35 mm² cable specification.

The review before that (TK-3V78DN) asked for a buildable stripping station (blade geometry, lever-to-blade mechanism, slug-retention method, dimensions and operating forces) and the holder's dimensions. That was done in the previous revision and is kept: opposed hardened-steel V-blades closed by a cam on the lever's hinge, which keep the cut slug while closed. The holder's bore is corrected now, as below.

What changed in response:

- **Importance / category:** the category is now Impact on jobs; it was Impact on AI timelines, and the data-centre wording is gone. The context and a new "Why it matters" paragraph place stripping in regular electrical work, with sources:
  - every conductor is stripped to length before it is terminated;
  - electricians held about 821,000 US jobs in 2025 (BLS Occupational Outlook Handbook, which lists wire strippers among their commonly used tools), and O*NET lists for them "Connect wires to circuit breakers, transformers, or other components";
  - O*NET's task for electricians' helpers is "Strip insulation from wire ends, using wire stripping pliers, and attach wires to terminals for subsequent soldering";
  - in factories, electrical and electronic equipment assemblers (246,300 jobs, BLS) and team assemblers (1,403,500, O*NET: "Provide assistance in the production of wiring assemblies") strip leads for control panels, switchgear and harnesses.
- **Core diameter reconciled with 35 mm²:** the cable is now a real type, an H07V-K 1 x 35 mm² single core, with the dimensions from a manufacturer's datasheet:
  - a 35 mm² class 5 (IEC 60228) fine-stranded copper conductor 7.9 mm across, wider than a solid 6.7 mm rod of the same area because of the gaps between strands;
  - 1.2 mm of PVC insulation, 10.3 mm across overall (EN 50525-2-31 allows up to 11.7 mm);
  - 340 g per metre.

  The old 8 mm core and 12 mm overall diameter are gone. The task file, setup, fidelity note and README all use 7.9 mm and 10.3 mm now. The blades close to 8.2 mm, which leaves 0.15 mm all round over the 7.9 mm core. With its 8 g plug the 12 cm cable weighs 39 g.
- **Holder bore made physically consistent:** the bore is now 12 mm, so the 10.3 mm insulated end stands in it with 0.85 mm of clearance all round. It would also take a cable at the standard's 11.7 mm maximum. The cup has 2 mm walls up to 1.8 cm above the table and a base 20 mm across and 4 mm thick. In the simulation the insulated end now collides at its full 10.3 mm width, through a hidden cylinder on the cable (the 2 mm insulation rings are only shown). The simulated start is therefore the specified one: the insulated end in the bore, with no overlap.
- **Numbers re-checked against the code:**
  - holding the lever down takes about 5 N at the grip, not the 6 N stated before: the spring gives 1.2 N m, less 0.19 N m from the lever's own weight;
  - the scoring now gives a real-world measurement: a caliper on the bare length (8 to 12 mm for 1.0), the slug found in the station, and the video for the insertion, the order and the release.
- **Re-run on the revised geometry:**
  - new demo: 1.0, with the end 9.6 mm below the blades when they closed and 10 mm stripped;
  - new Dream-1 run: 0.0;
  - one-hand check: 0.25. The cable was let go in the hole while the other hand closed the lever; it dropped to the hole's bottom, 33.8 mm below the blades, all 24 mm of insulation was cut, and letting the lever up put the insulation back on the cable.
