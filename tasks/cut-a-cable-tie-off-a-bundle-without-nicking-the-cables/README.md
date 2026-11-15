# Cut a cable tie off a cable bundle without nicking the cables

A bundle of seven insulated cables lies across two beech blocks, held together by a black nylon cable tie between the
blocks. The tie's head sits on the near side of the bundle. A pair of flush cutters with red handles lies in a tray to
the right. Two hands cut the tie off:
- One hand presses the bundle down on its left block.
- The other takes the cutters by their handles and pushes the jaws onto the strap, right beside the tie's head, then
  squeezes. The tie springs open and drops onto the table.
- The cutters go back in the tray, the hand lets go of the bundle, and both arms go back to where they started.

The task is one task file,
[`cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py`](cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests:
- using a hand tool that has to be squeezed;
- placing its jaws to within a few millimetres;
- knowing where a cut is safe for the material underneath;
- steadying a light, round workpiece with the other hand;
- putting the tool back.

Why two hands: the bundle is light and round. When the cutters' jaws touch it, it rolls or slides away, and the tie's
head rolls round toward the jaws. In the one-hand check (re-run under this revision's scorer), the right hand did its
part as in the demo and the left never moved; it scored 0.25. The jaws pushed the bundle 1.3 cm across the blocks and
rolled the tie's head round into their mouth; the tips stayed 6.8 mm short of the strap's inner face, and the squeeze
cut nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The blocks and the tray are kinematic, fixed where
they are put. The other parts are loose rigid bodies made of shapes from `bodykit.py`.

- **The bundle.** One rigid body of seven cables, 120 g.
- **The tie.** Glued to the bundle by a fixed joint (a `Glue` from `bodykit.py`). Its strap lies on the six outer
  cables and bridges the six grooves between them, one of them just below the head.
- **The cutters.** Two steel halves joined at their pivot by a hinge (a `Hinge` from `bodykit.py`). A spring holds the
  jaws 8 mm open at the tips. A hand holding the handles lightly keeps them open; squeezing them shut turns the halves
  up to 16 degrees together, closing the jaws.
- **The cut (a simulation proxy).** After every control step, the task file checks for each moment the halves close
  past 10 degrees. If the strap is then truly between the blades, wherever along the strap, the tie is cut. That means
  all 4 mm of its width lies between the 8 mm-apart edges (its middle within 3 mm of the mouth's middle line), the
  tips are at or past its inner face (within 10.9 mm of the bundle's axis), the cutters point into the bundle (within
  50 degrees) and their pivot runs along the strap (within 50 degrees). When that holds:
  - the glue switches off;
  - the closed tie is moved out of sight;
  - an open tie takes its place, with its strap gone where the jaws closed, and drops to the table.
- **A nick (a simulation proxy).** Either of two things counts as a nick:
  - cutting the strap where it lies on a cable, with less than 0.5 mm between its inner face and the insulation where
    the jaws' middle closes on it: there the blades cannot get past the strap without closing on that cable. Over the
    six grooves the gap is up to 4.4 mm, so a clean cut is possible over any of them, near the head or not;
  - closing the jaws, at any time, with any part of a jaw's front pressed 0.5 mm or more into a cable.
- **Off the bundle.** The cut tie must end off the bundle (its middle 1.5 cm below the bundle's axis or 3 cm from it)
  for any credit for the cut.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, with the cutters and the bundle
still. Full score includes the cutters back in the tray, both hands let go and both arms back where they started.
Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the cable tie off the cable bundle with the flush cutters from the tray, without nicking any cable's insulation. Then put the cutters back on the tray's rests, leave the bundle across its blocks with nothing holding it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tie-dream
```

The scripted demo reaches full score:
1. The left hand comes straight down on the bundle above its left block, fingers closed, and presses it down.
2. The right comes straight down round the cutters' handles and closes on them just enough to hold them.
3. It lifts the cutters out of the tray and carries them, jaws first, level with the strap just below the tie's head.
4. It pushes the jaws in until they touch the bundle, the strap between the tips.
5. It squeezes the handles: the tie springs off and drops to the table.
6. It carries the cutters back, lays them on the tray's rests and lets go.
7. The left lets go of the bundle, and both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the cable tie off the cable bundle with the flush cutters from the tray, without nicking any cable's insulation. Then put the cutters back on the tray's rests, leave the bundle across its blocks with nothing holding it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tie-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tie-dream/*.json; do inspect-robots video "$log"; done
```

Set `TIE_DEBUG=1` to print what the score is built from twice a second:
- how far the cutters' halves are closed;
- where the jaws' tips are against the strap: the gap to it, the offset across it, and the distance along it from the
  head;
- whether the tie is cut, nicked, the cutters taken or back, and the bundle still on its blocks;
- whether both hands are clear;
- where each TCP is from the cutters' handles and from the bundle above its left block.

## A real trial and the simulation's proxies

The real-world score does not depend on where the strap was cut:

| | Real trial (the score) | Simulation (the proxy) |
|---|---|---|
| Cut | the strap is physically cut through and the tie has come off the bundle, wherever it was cut | the jaws close with the strap between the blades, anywhere along it; the swapped-in open tie falls clear (its middle 1.5 cm below the bundle's axis or 3 cm from it) |
| Nick: scores 0 | any nick, cut or crush mark found when every cable's insulation is inspected all round under a 10x magnifier after the trial | the strap cut where it lies on a cable (under 0.5 mm of gap beneath it), or a jaw pressed 0.5 mm or more into a cable as the jaws close |
| End state (1.0) | ruler against marks on the tray and the blocks; the robot's joint log for the arms | the same distances, from the simulated poses |
| 0.25 | the cutters lifted 2 cm out of the tray, from the overhead video | the same, from the simulated poses |

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 900 steps (337 s on the clock, with Dream answering in 0.77 s on average), and about 1 minute for the scripted demo (377 steps, 18.9 s of simulated time, 80 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-BEKN5K. Its review said:

> Scoring (not met): The stated location-based zero and beside-the-head success condition can reject a clean, fully severed tie cut elsewhere.
>
> What to change: Distinguish the simulation's location-based cut and nick proxies from the real-world score. In real trials, award cut credit when the tie is physically severed and off the bundle, regardless of cut location, and award zero for observed insulation damage.

The review before that (TK-KA9BNZ) asked for the tie to be physically severed rather than the jaws merely closing
near it, for the insulation to be inspected with zero for any nick, and for the workplace-maintenance category; that
was done in the previous revision and is kept.

What changed in response:

- **Real-world score, independent of where the tie was cut.** Cut credit needs only the strap physically cut through
  and the tie off the bundle, wherever it was cut. Every cable's insulation is then inspected all round under a 10x
  magnifier: any nick, cut or crush mark scores 0. The scoring text, this README and the table above state it.
- **Simulation proxies labelled as such**, in the scoring text, the fidelity note and this README: the cut (jaws
  closing with the strap between the blades) and the nick (where the strap was cut and how deep a jaw pressed).
- **The simulation no longer requires a cut beside the head.** The old rule (any cut more than 12 mm along the strap
  from the head scored 0) is gone. A cut now counts anywhere the strap bridges a gap: a nick needs the strap cut
  where it lies on a cable (under 0.5 mm of gap beneath it), or a jaw pressed 0.5 mm or more into a cable. The nick
  check now runs every time the jaws close, not only at the first closing. In a test run the cutters, pitched up, cut
  the strap 87 degrees round from the head (17 mm along it from the head's middle, over the groove below the bundle's
  near side, with 2.7 mm of gap under the strap), which the old rule scored 0: it now scores 1.0. A second test cut
  the strap where it lies on a cable (0.04 mm of gap beneath it): the tie came off, and it scores 0.
- The demo, the Dream-1 run and the one-hand check were redone under the new scorer: demo 1.0 (in a debug run it
  cut 7 mm along the strap from the head, with 1.9 mm of gap beneath it), Dream-1 0.0, one-hand 0.25.
