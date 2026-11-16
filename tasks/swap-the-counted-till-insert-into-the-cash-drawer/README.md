# Swap the counted till insert into the cash drawer and shut it on its catch

A shop counter at a shift change. A steel POS cash drawer of the common 16 x 16 inch size (411 x 415 x 110 mm) stands
on the counter with the POS tablet on top of it. Its drawer has sprung open toward the cashier, 345 mm, as it does when
the POS fires the release. In the drawer sits the outgoing till insert: a black ABS tray, 361 x 306 x 60 mm, with five
bill bays under black hold-down clips at the back and five coin cups at the front. With its coins and notes it weighs
1.53 kg. The counted insert for the next shift, the same model, lies on a grey rubber mat to the right of the drawer,
coins toward the arms. The counted insert carries a count slip and full, even stacks; the used one is uneven. The mat
to the left of the drawer is empty.

The robot is told: "Swap the till insert in the cash drawer for the counted one on the right-hand mat, coins at the
front and without spilling any coins. Leave the old insert on the empty mat, shut the drawer and take both arms back to
where they started."

In the scripted demo:
1. Both hands take the old insert by its cross wall, the wall between the coin cups and the bill bays. The right
   pinches it 15 cm right of the insert's middle, leaned 40 degrees toward the left. The left pinches it 6 cm left of
   the middle, straight down. Both close their fingers front and back across the wall.
2. They lift it straight up until its underside clears the pan's walls. They carry it level 40.5 cm to the left, the
   left hand leaning over to 30 degrees as it goes out, and set it down flat on the left mat.
3. Both arms go back toward their start. They take the counted insert on the right mat the same way, with the roles
   swapped: the left crosses over, leaned 40 degrees toward the right, and the right holds 6 cm right of the middle.
4. They lift it, carry it level over the drawer and lower it in. Its drafted walls find the pan, it settles on the
   pan floor, and they let go.
5. Both hands come down in front of the drawer and push it home, 345 mm, against its spring, until the catch drops in.
   They let go, and the catch holds the drawer shut.
6. Both arms go back to where they started.

The task is one task file,
[`swap_the_counted_till_insert_into_the_cash_drawer.py`](swap_the_counted_till_insert_into_the_cash_drawer.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests:
- Lifting a wide, loaded tray with no handle level, by the walls inside it, straight up out of a close-fitting pan.
- Carrying it 39 cm sideways with both hands and setting it down flat, without tipping out its coins.
- Lowering the counted tray into a pan with 5 mm of play on each side.
- Pushing a sprung drawer 345 mm home against its ejection spring until the catch holds it.

## Realism check

- **Where the drawer is.** A POS cash drawer sits on the counter under the terminal, or hangs under the counter on
  brackets. APG sells its Vasario 1616 as "Countertop; compatible under-counter brackets sold separately". Star's
  16 x 16 inch drawer has a top made to carry a POS terminal or tablet. This scene uses the countertop install, with a
  tablet POS on top. The drawer opens toward the cashier's side, as it does at a real till.
- **Real parts.**
  - **Drawer.** APG Vasario 1616: 411 x 415 x 110 mm (97 mm without feet), shipping weight 8.6 kg, on a "roller bearing
    suspension system" with a "latch ejection mechanism". A Star CD4-1616 is 410 x 415 x 114 mm and 7 kg.
  - **Insert.** APG VPK-15B-2A-BX, the fixed 5 bill / 5 coin till for the Vasario 1616: 14-3/16 x 12-1/16 x 2-3/8 in
    (360 x 306 x 60 mm), shipping weight 3 lb. An HP heavy-duty till is 389 x 281 x 59 mm.
  - **Drawer travel.** One vendor's guide gives about 380 mm of travel for a 420 mm deep drawer. The model uses 345 mm,
    the least that exposes the whole insert so it can come straight up out of the drawer.
  - **How it opens and closes.** The POS sends a pulse to a solenoid, which releases the latch, and a spring ejects the
    drawer. To close it, the cashier pushes it home until the latch catches. No published figure gives the spring's
    force. The model uses 6 N at full travel and 12 N at home (our estimate of a light push; check it with a force
    gauge).
  - **How the insert sits.** The insert lifts straight out ("Lift out the coin and bill tray for cash handling") and
    rests on runners on the pan floor. Its rim is flush with the pan's walls, and it fills the pan's width (APG product
    photo). It drops into no locating slots: the two small tabs at its back corners are drawn on only. The play in the
    pan is not published. We use 5 mm a side, our estimate: the 411 mm cabinet less a 361 mm insert leaves 50 mm for two
    steel walls, two roller slides and the pan's walls.
  - **Load.** A US float of about $40 in coins is 0.65 kg (rolls of quarters, dimes, nickels and pennies broken into
    their cups). The tray with its clips and steel bar is 0.83 kg, and the notes 0.05 kg.
- **Every step of the real job.** The job is to lift the old insert out of the drawer with both hands, keeping it level,
  and set it down. Then the cashier puts the counted insert in, coins at the front, and pushes the drawer home until
  it latches. Two steps are left out. The drawer starts open, because the POS opens it with a touchscreen command,
  not by hand. Carrying the old insert to the office is the next step, outside the till.
- **YAM limits.**
  - **Load.** The insert's 1.53 kg is shared: the hand near the centre line takes 29% and the outboard hand 71%. The
    grips pinch the insert's cross wall, which runs within 1 mm of its centre of mass, so the insert hardly pitches
    about the grips.
  - **Height.** To clear the pan's walls, the insert's underside must rise 76 mm, to 102 mm above the counter. That puts
    the TCP at
    TOP+0.181, including 15 mm for the arms' sag under load. The rule of thumb is TOP+0.16; the YAM's joint limits
    allow TOP+0.20 at this reach.
  - **Reach.** Each mat is 39 cm off the centre line, so the hand that crosses over grips at 24 cm across, leaned
    40 degrees. A straight-down grip reaches only 16 cm across.
  - **Push.** The push to close is 12 N at home, under the YAM's 19 N.
  - **Jaw.** The pinched wall tops are 4 mm thick, well inside the 93.9 mm jaw, and the hands are 21 cm apart.
  - **Placement.** The capture for the drop-in is +-7.3 mm: 5 mm of play plus 2.3 mm of wall draft. YAM placement
    error is 5-19 mm, so this is the task's main difficulty.
- **What was not made easier.** The insert, the pan, the play and the drawer's travel are at real size. The YAM can't
  pinch the real 2.5 mm walls, so the top 8 mm of the insert's internal walls are 4 mm thick (see Fidelity).
- **Sources.**
  - APG Vasario spec sheet, M-43-111: https://www.barcodesinc.com/media/pdf/APG/vasario.pdf
  - Vasario brochure: https://www.ers-online.co.uk/files/products/files/VB320-BL1616-B5.pdf
  - VPK-15B-2A-BX till: https://www.webstaurantstore.com/apg-vpk-15b-2a-bx-vasario-5-bill-5-coin-till-kit-for-vb320-1-bl1616/558PK15B2ABX.html
  - APG Vasario retail page (installation, till removal, open photo): https://posguys.com/cash-drawer_32/APG-Vasario-Manual_356/
  - Star CD4 spec sheet: https://media.starmicronics.com/hubfs/Spec%20Sheets/Cash-Drawer-%20CD4%20-Choice%20Series.pdf
  - STORETENDER Model 16 brochure: https://www.storetenderonline.com/wp-content/uploads/Brochure_Model_16_POS.pdf
  - HP Heavy Duty Cash Drawer QuickSpecs: https://cdn.iceshop.nl/objects/mmo_13727442_1780425942_8862_1056904.pdf
  - Cash drawer buyer's guide (travel, spring latch, shift-change swap): https://dcapos.com/what-is-a-cash-drawer/
  - O*NET 41-2011.00 Cashiers: https://www.onetonline.org/link/summary/41-2011.00

## Why it matters (category: Impact on jobs)

Cashiers (SOC 41-2011) number 3,106,300 in the US (BLS via O*NET, 2025). O*NET lists as a core task: "Count money in
cash drawers at the beginning of shifts to ensure that amounts are correct and that there is adequate change." Where
each cashier has their own counted till, the insert is changed at every shift: the old one comes out, the counted one
goes in. Retailers sell spare inserts as "perfect for giving each cashier their own till". A drawer maker's brochure
says to use a locking insert cover "to transfer cash and media when employees begin and end their work shifts".

## How this differs from the board

The nearest tasks on the board are #1777 (till-make-change), #1775 (refit a loaded drawer on its roller slides), #805
(spring-drawer-hold-and-retrieve) and #1228 (swap-tray-on-passing-cart).
- **#1777** counts change out of an empty till with oversized tokens. The difficulty there is arithmetic. No insert is
  lifted, carried, fitted or latched.
- **#1775** carries a whole drawer onto its slides above riding height. Here the drawer stays on its slides: a loaded
  insert is carried level into a pan with 5 mm of play, and the drawer is pushed home against its spring to its catch.
- **#805** holds a self-closing drawer open to take a block out. Here the spring pushes the drawer open, and the job
  ends with it latched shut.
- **#1228** swaps trays on a moving cart; its difficulty is the moving target.
- **This task.** A wide, loaded tray with no handle is lifted level by the walls inside it, out of a close-fitting
  pan, and set down flat. A second one is lowered into the same pan at real clearance, coins at the front. Then a
  sprung drawer is pushed home until it latches. RoboCasa has "close the drawer", but no benchmark has this insert swap.

## Why two hands

- **The pinch is too weak for one hand.** The insert has no handle, so a hand can only pinch one of the 4 mm wall tops
  inside it. The YAM's V-shaped jaw holds such a wall only to about 10 N before it squeezes the wall out toward its
  fingertips. The loaded insert weighs 15 N. In a debug run with one hand taking all the weight at the insert's
  middle, the insert slid out of the grip as it was lifted.
- **One grip off the middle tips it.** A single grip 15 cm off the middle would have to hold about 2 N m against
  tipping, and a pinch holds 0.2-0.4 N m.
- **No one arm reaches both mats.** Each mat is 39 cm off the centre line.
- **Cashiers use both hands too.** A cashier lifts a loaded insert with both hands to keep it level.

**One-hand check.** The left stayed at its start for the whole trial, and the right did exactly its part of the demo.
That trial scored 0:
- the old insert tipped 13 degrees and slid back into the pan;
- the counted insert tipped past 25 degrees as the right lifted it off the mat, which latches spilt coins.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. Everything here is built from shapes from
`bodykit.py`, at real size and mass.
- **Fixed parts.** The drawer's cabinet with the tablet on its stand, and the two mats.
- **Loose parts.** The drawer and both inserts.

- **The cabinet.**
  - Black steel, 411 x 415 x 110 mm on four 13 mm feet: base, top, sides, back, and a lip and side frames round the
    drawer opening.
  - A POS tablet on its stand sits on top, toward the back.
- **The drawer.**
  - A steel pan, inside 371 x 316 mm and 60 mm deep, with 2 mm walls, behind a 405 mm wide black front 15 mm thick.
    The front carries the lock and two media slots, drawn on.
  - 1.8 kg in all. The roller slides are drawn on.
  - It runs on a PhysX prismatic joint to the cabinet, 345 mm of travel.
  - **Ejection spring.** The joint's drive is a long preloaded spring, 17.4 N/m: it pushes the drawer open with 6 N at
    full travel and 12 N at home.
  - **Catch.** Once the drawer has been pushed within 1 mm of home, the task file moves the joint's lower stop to 2 mm
    short of home. That stop is the catch: the drawer can no longer spring open, and it rests on the catch 2 mm out.
    This is a live joint limit, set once and never undone in a trial.
  - Its contacts take the lower of the two frictions (steel, 0.35).
- **The inserts.** Each is one rigid body of 1.53 kg with its centre of mass where its parts put it: 32 mm in front of
  its middle, toward the coins, and 25 mm up.
  - Black ABS, 361 x 306 x 60 mm at the rim.
  - Two 8 mm runners underneath.
  - A 2.5 mm floor, and 2.5 mm outer walls drafted 2.5 degrees with a 4 mm rim.
  - A cross wall 120 mm behind the front, between the five coin cups and the five bill bays, and four dividers. The
    top 8 mm of these internal walls are 4 mm thick.
  - Drawn on: the coins (quarters, dimes, nickels, pennies), the notes under their black clips, the steel hold-down
    bar and the two rear tabs. The counted insert also carries a white count slip (10 x 7 cm) under one of its clips.
    Its five note stacks are even and its coin cups all full to one level. The used insert's stacks and cups are
    uneven, with one bay and one cup nearly empty. Both weigh 1.53 kg with the same centre of mass. The drawn coins
    and notes have no mass of their own.
  - Friction 1.0, for the pinch.
- **The mats.** Grey rubber, 370 x 360 x 3 mm, friction 0.9.

After every control step the task file reads:
- the drawer's travel;
- both inserts' poses and vertical speeds;
- each gripper's position.

From these it keeps the catch, the spill latch and the end state the score is built from.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: the full score, with the drawer and both
inserts still (each moved less than 0.5 mm over the last second). Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5, 0.75 or 1.0.

**Latched for the whole trial: spilt coins (score 0).**
- **Tipped.** Either insert tipped more than 25 degrees from level at any moment. Its coins would slide out of their
  cups. Real trial: any coin out of its cup, by eye on the video.
- **Dropped.** Either insert landed faster than 0.6 m/s, a fall of 2 cm or more. Real trial: any coin out of its cup,
  or a drop seen on the video.

**Judged at the end.**
- **Old insert on the mat.** All four corners of its underside lie inside the left mat's outline and within 5 mm of
  its surface, and it is tilted no more than 5 degrees. Real trial: it lies flat on the mat, by eye.
- **Counted insert seated.** Read in the drawer's own frame, so it still counts with the drawer shut:
  - all four corners of its underside lie inside the pan and within 3 mm of the pan floor;
  - it is turned no more than 10 degrees from square, with its coins at the front.
  Real trial: open the drawer from the POS and check that it sits flat (no corner raised past a 3 mm feeler), coins at
  the front.
- **Shut.** The catch has dropped in and the drawer rests on it, within 3.5 mm of fully home. Real trial: the drawer
  stays shut, and does not open when its front is pulled.
- **Arms home.** Each gripper's centre is within 5 cm of where it started.

**Tiers.**
- 1.0: the old insert on the left mat, the counted one seated with its coins at the front, the drawer shut on its
  catch, and both arms home.
- 0.75: all of that but the arms home.
- 0.5: the old insert on the mat and the counted one seated.
- 0.25: the old insert on the left mat.
- 0.0: anything less, or coins spilt.

Every tier needs the old insert on its mat, so leaving the old insert in the drawer, or putting the counted one in
without taking the old one out, scores 0. A counted insert sitting proud of the pan can't pass under the cabinet's
front lip, so the drawer can't latch over it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_the_counted_till_insert_into_the_cash_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swap the till insert in the cash drawer for the counted one on the right-hand mat, coins at the front and without spilling any coins. Leave the old insert on the empty mat, shut the drawer and take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/till-dream
```

`-E object_jitter=0` keeps every object exactly where the task file puts it. The scripted demo:
`--policy yam_scripted` in place of `--policy dream`.

## Fidelity

Every part is at real size and mass. The simplifications:
- **Coins and notes.** The coins are fixed mass in the insert's body, and the notes are drawn on. A tilt past 25
  degrees, or a drop of 2 cm or more, stands in for coins sliding out of their cups.
- **Wall tops.** The top 8 mm of the insert's internal walls (the cross wall and the dividers) are 4 mm thick; real
  moulded walls are about 2.5 mm. 4 mm is the thinnest wall the YAM's V-shaped jaw holds without squeezing it out.
- **Estimates.** The ejection spring (6 N fully out, 12 N home) and the 5 mm play each side are our estimates; neither
  is published.
- **Starting open.** The drawer starts open, as the POS leaves it after firing the solenoid release. That release is
  a touchscreen command, not a manipulation.
- **Rigid bodies.** The drawer's roller slides are an ideal prismatic joint, and the catch is the joint's stop, moved
  in once the drawer reaches home.

## Scores

- **Scripted demo: 1.0.** The full score held for 1 s at 56.0 s (1,119 steps). It scored 1.0 both before and after
  the two inserts were given their different looks, and three debug runs of the final demo also scored 1.0.
  - It lifted each insert with its underside 14 mm above the pan's walls, tilted at most 3.3 degrees (old insert) and
    6.8 degrees (counted insert).
  - It set the old insert on the mat 2 mm beyond the mat's middle and seated the counted one flat.
  - It pushed the drawer home onto its catch.
- **Dream-1: 0.0.** The run went the full 2,400 steps. The arms hovered over the drawer and both inserts without
  lifting either insert.
- **One-hand check: 0.0.** See "Why two hands".

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 2,400 steps (345 s on the clock, with Dream answering in 0.79 s on average), and about 6 minutes for the scripted demo (1,119 steps, 56.0 s of simulated time, 336 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

