# Rosin a violin bow: hold the rosin still with one hand and draw the bow's hair across it with the other

A full-size violin lies on a green felt cloth at the back of a black-lacquered music-room table, with its full-size bow
(75 cm, 60 g) laid beside it, hair down, the ebony frog to the right. In front, a cake of amber rosin sits at the back
end of a small beech holder with a raised grip across its front end, on the lacquer. The bow's hair is clean and pale.
The robot rosins the bow. One hand holds the rosin's holder by its grip, and the other picks the bow up by its frog,
lays the hair on the cake and draws the bow back and forth until the hair is coated white along most of its length.
Halfway through, the holding hand slides the rosin 23 cm to the left, so that the strokes reach the half of the hair
toward the head. Then the robot lays the bow back on the cloth beside the violin, and both arms go home.
The task is one task file,
[`rosin_a_violin_bow_holding_the_rosin_still.py`](rosin_a_violin_bow_holding_the_rosin_still.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: a two-hand everyday job with a long, light tool at its real size. The bow has to be held level by its
frog, 20 cm from its balance point, with the hair kept on a 5 cm cake while it travels some 30 cm. Meanwhile, the other
hand holds the light rosin without getting in the bow's way, and moves it once to reach the rest of the hair.

Why two hands: the rosin's holder stands on smooth lacquer and slides at about 0.03 N, while the horsehair grips the
rosin. Drawn across the cake with nothing holding it, the rosin simply travels along with the hair and nothing is
coated. One hand holds the holder by its grip while the other draws the bow.
With one hand (the right drawing the bow exactly as in the demo, the left never moving) the demo scored 0: the rosin
travelled along with the hair, up to 28 cm across the table, and no length of hair got any coating.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The lacquer, the cloth and the violin are kinematic,
fixed where they are put. The bow and the rosin's holder (with its cake) are loose rigid bodies made of shapes from
`bodykit.py`.

- The bow: an ebony frog block, the screw button, a stick of twelve straight lengths that taper from 8.4 to 5.8 mm with a
  slight camber, a head, and the horsehair, all at full size: 75 cm long, 60 g, balancing 19.8 cm in front of the frog's
  middle as a real bow does. The hair is one thin collision strip from the frog to the head. Its look is drawn in 12
  lengths of 5.5 cm; each has a pale "clean" strip and a slightly larger white "coated" strip, and the task file shows
  one or the other. The whitening reports rubbing that physically happened: it is not a stand-in for any manipulation.
- Coating: after every control step the task file finds the cake's top in the bow's frame. The hair lies on the cake
  when the cake is within the hair's width (plus the cake's radius) of the hair line, and its top is within 2.5 mm
  below to 3 mm above the hair's underside. While it lies there and the holder has moved less than 1.5 mm (and turned
  less than 0.75 degrees) since the last step, the cake's travel along the hair is added to every length within 0.6 of
  a cake radius of it. A length turns white once 10 cm has been added.
- The lacquer's contacts take the lower of the two frictions: the holder slides at 0.06. The hair and the cake grip
  each other at about 0.95.
- The bow is laid down when its frog and its head both lie on the cloth, level within 20 degrees.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives `score()`
each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both arms back
where they started, with the bow and the rosin still. Otherwise it gives `score()` at the end of the episode (0, 0.25,
0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it. The
cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=rosin_a_violin_bow_holding_the_rosin_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Rosin the violin bow. Pick the bow up by its black frog and draw its hair back and forth across the rosin cake while your other hand holds the rosin's holder still on the table by its grip, until the hair is coated white along most of its length; you may slide the rosin to a new spot between strokes. Then lay the bow back down on the green cloth beside the violin and take both arms back to where they started." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/rosin-dream
```

The scripted demo reaches full score:

1. The left takes hold of the rosin holder's grip, its fingers closing across it.
2. The right comes down round the frog, its fingers closing across it, and lifts the bow. It carries it forward over the
   rosin and lays the hair on the cake.
3. It draws the bow toward the tip and back three times, the frog travelling from 16 to 45 cm right of the cake. While
   drawing toward the frog, the grip is turned 2.5 degrees against the way the bow turns in the fingers.
4. It lifts the bow and holds it back over the cloth's edge. The left slides the rosin 23 cm to the left, and the right
   draws the bow four more times each way, the frog from 45 to 64 cm right of the cake.
5. It lifts the bow off, the left lets go, and the right lays the bow back on the cloth where it was.
6. Both arms go home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=rosin_a_violin_bow_holding_the_rosin_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Rosin the violin bow. Pick the bow up by its black frog and draw its hair back and forth across the rosin cake while your other hand holds the rosin's holder still on the table by its grip, until the hair is coated white along most of its length; you may slide the rosin to a new spot between strokes. Then lay the bow back down on the green cloth beside the violin and take both arms back to where they started." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/rosin-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rosin-dream/*.json; do inspect-robots video "$log"; done
```

Set `ROSIN_DEBUG=1` to print what the score is built from four times a second:

- where the holder has moved and whether it stayed still;
- the bow's pose;
- the cake's top in the bow's frame and the gap under the hair;
- whether the hair is on the cake;
- the travel added to each length, and how many are coated;
- whether the bow is laid down and whether both arms are home;
- the finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,500 steps (525 s on the clock, with Dream answering in 0.61 s on average), and about 6 minutes for the scripted demo (1,244 steps, 62.2 s of simulated time, 384 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders. These times were measured with other simulations sharing the GPU; alone it runs
faster.
