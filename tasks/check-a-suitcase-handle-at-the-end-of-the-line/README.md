# Check a suitcase's pull handle locks at every height at the end of the line, and tag it if one gives

A finished hard-shell cabin case lies on its front, back panel up, on the grey-felt inspection bench at the end of a
luggage line. Its twin-tube aluminium pull handle is closed. The handle locks closed and at four heights. It slides only
while the release button in its grip is held in. Some units come off the line with a worn lock: one height clicks, but
gives under a firm push along the rails. Red reject tags hang on a small stand at the front of the bench.

The robot is told: "This suitcase is at the end-of-line handle check. Check that its pull handle locks at every height;
if any height fails, hang a reject tag on the case. Leave the handle closed and both arms back where they started."

The task is one task file, [`check_a_suitcase_handle_at_the_end_of_the_line.py`](check_a_suitcase_handle_at_the_end_of_the_line.py),
for the `yam_arena` embodiment (scene, mechanism, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which
it imports from the same folder.

What it tests:
- **Working a button-gated telescoping handle stop by stop.** The pinch that holds the button in also draws the handle
  out; letting the button up lets the pin ride along the tube until it clicks into the next hole.
- **Load-testing each lock the way an inspector does.** The test is a push along the rails, with the release button
  left alone. Pushing on the button would release the lock, so the push has to come from beside it.
- **Holding the case.** The case is loose on felt that slides it at about 10 N, less than the push the test needs. One
  hand braces the case while the other loads the lock.
- **Deciding.** Nothing in the scene shows which unit this is. Only the handle's response to the push does. A worn
  height gives and slides back; the unit is then a reject and gets a tag.
- **Leaving the unit ready.** The handle goes back in and locks closed, and both arms return.

In the scripted demo (written for the default trial, where the 40 in lock is worn):
1. **Brace (0-5 s).** The left hand comes down between the casters at the case's wheel end. It rests its closed
   fingers against the end panel, so the case cannot slide away from a push along the rails.
2. **Pinch (0-5 s).** The right hand comes down over the grip, leaned 20 degrees, its pads open across it: one outside
   the end face, one in the finger gap. Pinching holds the button in, which frees the handle.
3. **Each height (5-28 s).** The right hand:
   - draws the handle out 25 mm past the height and lets the button up, so the pin rides along the tube;
   - opens, moves its pinch 40 mm along the grip beside the button, and pinches again;
   - pushes the handle back toward the case, aiming 70 mm past the height (about 20 N). The pin drops into its hole
     and the handle must hold;
   - eases off and pinches the button again.

   Heights 1 (364 mm) and 2 (415 mm) hold. Height 3 (466 mm) clicks, then gives under the push: the handle slides
   51 mm back and locks at height 2. The check stops there: the unit is a reject.
4. **Close (30-37 s).** With the button pinched in, the right hand pushes the handle all the way in, lets the button up
   (it clicks in closed), opens and goes back.
5. **Tag (36-55 s).** The left hand lets go of the case and goes up and over it. It takes the nearest red tag off the
   stand by its knob, carries it to the case's near side and hooks it over the side carry handle. It lets go, rises
   past the case's edge and goes back.
6. **Finish.** The trial ends once the full score has held for 1 s with the case and the tags still (about 59 s).

The good-unit plan (environment variable `SUITCASE_PLAN=pass`) tries all four heights the same way, closes the handle
and hangs no tag.

## Realism check

- **The real product.** Travelpro Platinum Elite Slim Carry-On Expandable Hardside Spinner 55 cm. The maker's EU
  listing gives:
  - 55 x 40 x 20 cm overall, wheels and handles included;
  - 3.5 kg and 39 L;
  - a 100% polycarbonate shell;
  - a PowerScope Lite extension handle with four stops at 36", 38", 40" and 42.5".

  Source: https://eu.travelpro.com/products/platinum%C2%AE-elite-slim-carry-on-expandable-hardside-spinner-55cm-55-x-40-x-20cm

  The Platinum Elite hardside line has aluminium corner guards, four double spinner casters and a grooved, textured
  shell. Source: https://travelpro.com/products/platinum-elite-hardside-carry-on-expandable-luggage-hardside-spinner-suitcase

  Modelled:
  - 55 x 40 x 20 cm overall, 3.5 kg. The shell, wheels and fixed parts weigh 3.12 kg; the handle's moving parts make
    up the rest.
  - A 443 x 366 x 200 mm body, with the casters 67 mm and the closed grip 40 mm beyond it.
  - The four stops, measured with the case standing as heights of the grip top from the floor (914, 965, 1016 and
    1080 mm). From a closed height of 550 mm, they are 364, 415, 466 and 530 mm out. The maker does not say where the
    heights are measured, so this is our reading.
- **The real handle hardware.** Luggage-handle makers give these figures (supplier data, not Travelpro's):
  - a release button of "5-8 N" with 4-6 mm of travel;
  - spring pins or Ø3 mm balls that drop into Ø4 mm holes in the tubes;
  - rail centres around 140-150 mm.

  Sources: https://www.jxluggageparts.com/7-engineering-secrets-for-choosing-telescopic-luggage-handles-that-importers-must-know/
  and https://www.jxluggageparts.com/how-telescoping-luggage-handles-work-the-engineering-behind-smooth-travel/

  JX lists the worn-lock defect modelled here: "a worn lock pin or ball causes auto-retraction". Modelled: a 4 mm
  button that holds 5 N at rest and 8 N fully in, rails 160 mm apart, and rectangular aluminium tubes of 22 x 13 and
  19 x 10 mm.
- **The real check.** A luggage final-inspection guide: "Each time a section is pulled out, verify that it is locked in
  place (after hearing a 'click,' apply slight downward pressure; it should not retract) ... Failure to lock in any
  section or a jammed button is considered a critical defect."
  Source: https://www.globalinsp.com/en/3561-html/

  Here the case lies flat, so "downward" is along the rails, toward the case. A critical defect stops the shipment; the
  unit is tagged as a reject. The usual red reject tag of nonconforming-material practice is used.
  The Chinese travel-goods standard QB/T 2155-2018 type-tests handles for 3000 extension cycles per QB/T 2919. That is
  a laboratory test, not the end-of-line check, so it is not modelled.
- **The real place and height.** The bench top is 0.76 m above the floor. With the case lying flat, the work is at
  0.94-0.96 m, inside the 0.87-1.17 m range recommended for standing light work. Grey felt protects the shells; its
  friction on polycarbonate (0.30) is an estimate.
- **Every real step.** For each height:
  1. Draw the handle out with the button held in.
  2. Let the button up.
  3. Let the pin click in.
  4. Push the handle toward the case without touching the button.
  5. Watch whether it holds.

  At the end, retract the handle, let it lock closed, and tag a reject. The inspection guide repeats the cycle 10-15
  times; one cycle is done here.

## The seeds

The unit depends on the trial's reset seed: variant = reset seed mod 5.

| Variant | Unit | `--seed` giving it |
| --- | --- | --- |
| 0 | good: every height locks | 1, 5, 6, 22 |
| 1 | the 36 in (364 mm) lock is worn | 8, 16, 23, 32 |
| 2 | the 38 in (415 mm) lock is worn | 2, 4, 9, 10 |
| 3 | the 40 in (466 mm) lock is worn | 0 (the default), 3, 7, 11 |
| 4 | the 42.5 in (530 mm, full height) lock is worn | 14, 15, 17, 18 |

inspect-robots' default `--seed 0` resets with seed 1415158438, which is variant 3. The scripted demo is written for
that trial. A worn lock holds 14 N and then gives: its pin is pushed out of its hole, and the handle slides back until
the pin drops into the next good hole below. Every other hole holds rigidly.

## Checks run on the simulator

The debug runs below use `SUITCASE_DEBUG=1`, which logs the mechanism every 0.25 s.

**Default trial (`--seed 0`, worn 40 in lock), demo plan: 1.0, twice** (the debug run, and the recorded 1280x704 demo)
- Heights 1 and 2 locked and held the push.
- Height 3 clicked, then gave: the handle slid from 466 to 415 mm and locked there.
- The handle went in and locked closed.
- The tag hung on the side carry handle.
- The case ended 13 mm and 0.6 degrees from where it lay.

**Good unit (`--seed 1`), `SUITCASE_PLAN=pass`: 1.0**
- All four heights locked and held, including full height.
- The handle locked closed and no tag was hung.
- The case ended 17 mm and 1.4 degrees off.

**One hand (`--seed 0`, `SUITCASE_ONEHAND=1`, the left arm left at rest): 0.0**
- Without the brace, every test push slides the case along the felt instead of loading the lock: 45 mm on the first
  push.
- The worn lock never gave, so the fault was never found.
- The open-loop plan's later pinch then missed the grip, and the hand shoved the case 444 mm.
- Score 0: unchecked, and no tag.

The environment switches change only the scripted demo or the debug output, never the scene or the scoring:
- `SUITCASE_PLAN=pass`: the good-unit plan.
- `SUITCASE_ONEHAND=1`: the one-hand check.
- `SUITCASE_DEBUG=1`: the mechanism log.
- `SUITCASE_VIEW`, `SUITCASE_EYE`, `SUITCASE_TGT`: move a wrist camera for close-up stills.

**Shortcuts the scoring refuses**
- **A tag without the check, or a pass without it, scores 0.** On a worn unit, 'checked' needs the worn lock to have
  given. On a good unit, it needs all four heights locked at some point.
- **The demo plan on a good unit scores 0**, because it tags a good unit. **The good-unit plan on the default trial
  scores 0**, because it leaves a worn unit untagged.
- **Pushing on the button does not test the lock.** It pulls the pins, so the handle slides at any height.
- **A tag lying on the case, or dropped on the bench, is not 'hung'.** On a good unit, a tag resting on the case counts
  as a wrong verdict.

## Scoring

Judged at the end of the run; tipping counts at any time.

| Check | What counts |
| --- | --- |
| Checked | Good unit: the handle was locked, pins in with the button out, at each of its four heights at some point. Worn unit: the worn lock was made to give at some point (its pin pushed out of its hole). |
| Verdict right | Good unit: no tag on the case. Worn unit: a tag hangs by its hook from the case (its side carry handle, or its closed grip), off the bench, with no gripper within 6 cm. |
| Handle closed | Closed and locked: within 3 mm of closed, pins in. |
| Case in place | Within 50 mm and 10 degrees of where it lay. |
| Arms back | Each gripper's fingertip centre (TCP) within 5 cm of its start. |

Tiers:
- **1.0**: checked, verdict right, handle closed, case in place, arms back. Success also needs 1 s with the case and the
  tags still.
- **0.5**: checked and verdict right, but the handle was left out, the case was dragged or turned further, or an arm is
  not back.
- **0.0**: anything else. That includes:
  - a wrong verdict: a tag on a good unit, or no tag on a worn one;
  - a verdict with no check behind it: tagging or passing a unit without working the handle;
  - the case tipped over (30 degrees) at any time.

On a real line all of this is read by hand and eye:
- push each height of the handle;
- see whether a red tag hangs on the case;
- check the handle is in and locked;
- measure the case's position on the bench.

The 'checked' part is seen in the run's video: the handle locked at each height, or the worn height giving under the
push.

## Simulation and the real case

Rigid bodies, PhysX at 240 Hz, 20 Hz control.

**The shell**
- The case's shell is a generated triangle mesh, for looks only: a rounded box (edge radii 45, 45 and 32 mm) with seven
  2.2 mm grooves on each panel, the handle's 12 mm recess in the top end, a zip band and eight corner guards.
- It collides as boxes: the body without its rounded edges, the recess cut out, and one box around each caster. The
  casters are fixed shapes. Lying on its front, the case does not stand on them.

**The handle**
- One slide carries the grip with its inner tubes (530 mm of travel). The middle tubes follow it on a second slide,
  for looks only; they carry no load.
- The release button is a 10 g body on a sprung slide in the grip's end face: 4 mm of travel, 5 N at rest, 8 N fully
  in.
- The lock pins and the rod from the button are not bodies. After every control step the task file reads the button's
  travel: the pins are out from 2.5 mm and back below 1.0 mm.
- With the pins back, the handle locks at the first hole it is within 3 mm of, or crosses: the slide's stops close to
  ±0.5 mm around it. Pressing the button opens them again.
- A worn hole is a spring (6000 N/m) capped at 14 N. Pushed further, its pin leaves the hole 4 mm past it, and the
  handle slides free until the next good hole.
- The tube bushings are damping, 25 N s/m (2.5 N at 0.1 m/s).

**Tags and friction**
- The tags are rigid red PVC cards (64 x 80 mm, 22 g) with a moulded hook and a knob on the hook. Real reject tags are
  often card on a string or wire, which a gripper cannot tie.
- Felt on the case: friction 0.30, so about 10 N slides the 3.5 kg case.

**Estimates**, to be measured on the real case when a trial is set up:
- the body's size behind the overall 55 x 40 x 20 cm;
- that the stop heights are measured to the grip top;
- tube and grip sizes and the finger gap;
- the button's force and travel (the supplier's 5-8 N and 4-6 mm range);
- the worn lock's 14 N;
- bushing and felt friction;
- the handle parts' masses (0.22 kg grip and inner tubes, 0.12 kg middle tubes, 0.01 kg button).


### Colliders

Millimetres from the midpoint between the arm bases (x forward, y to the robot's left), heights above the table top, with the handle closed. The handle slide moves the grip, inner tubes and button along -y; the middle tubes follow, inner section first. Everything else in the scene (the shell's rounded mesh, zip band, corner guards, casters' wheels, top carry handle, lock and print) is for looks only.

| Body | Collider | x | y | z (above the table top) |
| --- | --- | --- | --- | --- |
| felt bench cover (fixed) | box | 50 to 570 | -630 to 630 | 0 to 3 |
| case (free, 3.12 kg) | box | 93 to 131 | 150 to 166 | 110 to 138 |
| case (free, 3.12 kg) | box | 93 to 131 | 250 to 266 | 110 to 138 |
| case (free, 3.12 kg) | box | 93 to 105 | 153 to 264 | 114 to 134 |
| case (free, 3.12 kg) | box | 127 to 493 | -13 to 430 | 4 to 134 |
| case (free, 3.12 kg) | box | 127 to 204 | -13 to 430 | 134 to 204 |
| case (free, 3.12 kg) | box | 416 to 493 | -13 to 430 | 134 to 204 |
| case (free, 3.12 kg) | box | 204 to 416 | -1 to 430 | 134 to 204 |
| case (free, 3.12 kg) | box | 147 to 207 | 430 to 497 | 23 to 77 |
| case (free, 3.12 kg) | box | 147 to 207 | 430 to 497 | 131 to 185 |
| case (free, 3.12 kg) | box | 413 to 473 | 430 to 497 | 23 to 77 |
| case (free, 3.12 kg) | box | 413 to 473 | 430 to 497 | 131 to 185 |
| middle tubes (follow the handle) | box | 379 to 401 | 27 to 327 | 163 to 176 |
| grip and inner tubes (handle slide) | box | 225 to 395 | -51 to -31 | 152 to 186 |
| grip and inner tubes (handle slide) | cylinder r 10.0, 34 long along z | 225 | -41 | 169 |
| grip and inner tubes (handle slide) | cylinder r 10.0, 34 long along z | 395 | -41 | 169 |
| grip and inner tubes (handle slide) | box | 218 to 242 | -31 to 3 | 157 to 181 |
| grip and inner tubes (handle slide) | box | 378 to 402 | -31 to 3 | 157 to 181 |
| release button (button slide) | box | 285 to 335 | -55 to -49 | 161 to 177 |
| tag stand (fixed) | box | 32 to 112 | -183 to -103 | 3 to 11 |
| tag stand (fixed) | cylinder r 6.5, 128 long along z | 72 | -143 | 75 |
| tag stand (fixed) | sphere r 8.5 | 72 | -143 | 133 |
| tag stand (fixed) | cylinder r 4.0, 205 long along y | 72 | -37 | 133 |
| tag stand (fixed) | sphere r 5.5 | 72 | 70 | 133 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 51 to 54 | -130 to -66 | 31 to 112 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 50 to 56 | -110 to -86 | 110 to 118 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 50 to 55 | -102 to -94 | 116 to 138 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 50 to 94 | -102 to -94 | 138 to 142 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 88 to 94 | -102 to -94 | 124 to 142 |
| reject tag (each of three; this is the one at y -98, free, 22 g) | box | 64 to 80 | -104 to -92 | 142 to 164 |

## How this differs from the earlier suitcase-handle task (#2107)

#2107 asks a traveller's question: pull a case's handle out to full height and leave it locked there. Its skill is
keeping the button held through the first stop and letting it up at the top; it scores the handle's final position.

This task is a factory inspection, judged by its verdict:
- **A hidden fault.** The unit may be good, or worn at any one of four heights, depending on the seed.
- **Load-testing every lock.** The robot must try each lock the way an inspector does: let the button up, re-grip
  beside it and push along the rails.
- **Seeing and deciding.** It has to notice the handle give, and hang a reject tag from the stand on the case only
  then.
- **Leaving the unit ready.** The handle must end closed and locked.

A reject tag on a good unit, or no tag on a worn one, scores 0, and so does a verdict reached without working the
handle. The product (a four-stop Travelpro handle, 530 mm), the layout and the category (impact on jobs) also differ.
What the two share is the object, the case lying flat, and a hand that keeps the case still while the other works the
handle.

## Runtime per trial

On one RTX 5090: about 17 minutes for a Dream-1 trial of 2,400 steps (991 s on the clock, with Dream answering in 0.61 s on average), and about 8 minutes for the scripted demo (1,166 steps, 58.3 s of simulated time, 496 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Reset

Press the button and push the handle in until it clicks closed. Take any tag off the case and hang it back on the
stand's rod. Lay the case back on its front at the marked place, top end toward the robot's right. The worn lock is a
property of the unit: a real trial uses a unit with a known worn hole, or a good one.
