# Seat the filter cartridge with its flow arrow pointing downstream

A white filter cartridge with a black arrow on its top stands on a plate, and beside it a white filter housing has a
yellow pipe at each side. A label plate beyond the housing reads IN at the inlet's end and OUT at the outlet's, with an
arrow between them. The robot is told to fit the cartridge with its arrow pointing the way the water runs, from the
inlet to the outlet. The arrow starts pointing away from the arms, a quarter turn off, so putting the cartridge straight
in fits it sideways: the job is to read the marking and turn the part to match before seating it. A cartridge fitted
backwards passes unfiltered water and can be damaged.

The task needs one arm: the left lifts the cartridge, turns it in the hand and lowers it into the housing. The right
arm stays at its ready pose.

## Files

| File | What |
|---|---|
| `seat_the_filter_cartridge_with_its_flow_arrow_downstream.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes (the cartridge and its raised arrow, one rigid body), and per-step hooks |
| `plantkit.py` | A step clock, tilt and placement checks, and the shared colours |
| `test_seat_the_filter_cartridge_with_its_flow_arrow_downstream.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=seat_the_filter_cartridge_with_its_flow_arrow_downstream.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new cartridge into the filter housing with the arrow on its top pointing the way the water flows: from the inlet pipe towards the outlet pipe." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cartridge-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=seat_the_filter_cartridge_with_its_flow_arrow_downstream.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Fit the new cartridge into the filter housing with the arrow on its top pointing the way the water flows: from the inlet pipe towards the outlet pipe." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cartridge-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/cartridge-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the cartridge is seated with its arrow downstream and
has stood still for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_seat_the_filter_cartridge_with_its_flow_arrow_downstream.py
```

## Scoring

Scored from the end state. The cartridge is seated when its centre is within 1.2 cm of the housing's centre seen from
above, its bottom within 5 mm of the housing's floor, and it leans under 10 degrees. Its arrow is read from above, from
the cartridge's own frame: downstream when within 20 degrees of +y (from the inlet towards the outlet), backwards when
within 20 degrees of -y. Standing on the bench means upright (leaning under 10 degrees) on the table, the stand or the
label plate, outside the housing's footprint, with its bottom no more than 1 cm above the stand's top.

| Outcome | Score |
|---|---|
| Seated, arrow downstream | 1.0 |
| Seated, arrow backwards | 0 |
| Seated, arrow along x (sideways), as it came off the stand | 0.2 |
| Seated at an in-between angle | 0.2 |
| Not seated, standing on the bench, arrow downstream | 0.3 |
| Not seated and not turned downstream | 0 |
| Turned, but hung on the housing's rim or held inside it | 0 |
| Lying on its side, upside down, leaning on the rim, on a pipe or held up in the air | 0 |
| Off the table | 0 |
| Nothing done | 0 |

## Design notes

- **The marking is the job.** Seating the cartridge as it comes, arrow sideways, earns 0.2. Turning it to point
  downstream without fitting it earns more, 0.3, because reading the arrow and turning the part is what the task
  measures. Seated backwards, the one fit that does harm, scores 0, below every honest partial result.
- **A square cartridge in a square housing seats only four ways.** A 7 cm block in an 8.5 cm opening has 7.5 mm a side,
  so it goes down only within about 14 degrees of square, inside the 20-degree bands, and a seated cartridge reads
  plainly as downstream, backwards or sideways. The in-between row can't happen on the bench; it is scored 0.2 should
  it. Pushed into a corner its centre is 10.6 mm off, inside the 1.2 cm allowed. A place 3 mm off in x and y and 3
  degrees off square still clears the inner walls by 2.7 mm at the bottom corners.
- **The arrows show in the top camera.** Projected through the calibrated top camera at Dream-1's 320 x 176, the
  cartridge's 1.8 cm shaft is 3.8 px wide on the stand and 3.1 px seated, and its 5 cm head is 10.5 and 8.7 px. The
  head steps down from 5 cm to 3.4 and 1.8 cm wide, so it reads as a point rather than a T that could be either end.
  The block and its arrow are one rigid body, so the arrow turns with it and is read from its own frame.
- **The label plate says which way the water runs.** Beyond the housing, flat on the table, a white plate carries IN at
  the inlet's end, OUT at the outlet's and an arrow between them pointing from IN to OUT. Its arrow is a size larger
  than the cartridge's (2.2 cm shaft, 6 cm head: 3.4 and 9.2 px), since it is further from the camera. The letters
  are 4 cm tall with 1 cm strokes: plain in the 640 x 352 demo video and on a real bench, small at 320 x 176, where the
  arrow carries the cue.
- **Seated means let go.** The floor check is 5 mm and the demo lets go with the cartridge's bottom 7 mm up, so a
  cartridge still held in the fingers isn't seated; it counts once it drops.
- **Turned but hung up scores nothing.** A cartridge turned the right way but resting on the housing's rim, or held
  inside it, scores 0, below the 0.3 for one left turned on the stand: it is neither fitted nor set down. This is by
  design; it can be lifted and lowered again.
- **Refitting is allowed.** Nothing latches: a cartridge seated backwards, lifted out and refitted the right way scores
  1.0, as on a real job, where nothing flows until the bowl is closed and the valve opened. The step hook keeps only a
  record of when the cartridge was lifted, turned and seated.
- **The turn.** The demo grips the cartridge with the fingers along x, on the faces at the arrow's two ends, and sets it
  down with them along y: a quarter turn anticlockwise in the hand, taking the arrow from +x to +y. Gripping along y
  and turning to +90 degrees would need joint6 past its 120-degree limit; this way the wrist stays between -34 and +59
  degrees.
- **A 7 cm grasp.** The cartridge is wider than the 2 to 6 cm the grasps elsewhere use, to stand for a real cartridge;
  the open fingers (9.4 cm) leave 1.2 cm a side as they come down round it, and the pads close on its top 1.9 cm.
- **Clearances.** It is let go with the pad tips 12 mm above the rim, so the fingers never enter the housing. Carried,
  its bottom clears the rim and the pipes by 5.6 cm. The stand sits at x 17, y 20, so at the pick the pads (2 cm either
  side along y) stay 1.15 cm clear of the housing in plan. Placed from `station.wrist_camera`, the wrist camera's body
  keeps at least 6.6 cm from every fixture throughout the demo. The plan ends with the arm moved aside to its own side,
  clear of the top camera's view, though the trial ends at 1.0 before it gets there.
- **Unintended strategies.** Lying on its side, upside down (the arrow then faces the floor), leaning on the rim,
  balanced on a pipe, held up in the air or knocked off the table: 0. Turning it on the stand or the table by pushing it
  round: 0.3, the same as turning it in the hand, since that is the turn the task asks for. Dropping it in from above:
  counts only if it ends seated, upright and the right way round.
- **The rule behind it.** The cartridge maker's installation instructions say to fit the cartridge with its arrow in the
  direction of flow. The International Plumbing Code has plumbing materials installed to the standards they are
  approved under, or else to the manufacturer's instructions
  ([IPC 303.2](https://codes.iccsafe.org/content/IPC2021P1/chapter-3-general-regulations)), and the Uniform Plumbing Code
  has drinking water treatment units meet NSF/ANSI 42 or 53 (UPC 611.1, Table 611.1;
  [how the plumbing codes apply to water treatment](https://wcponline.com/2021/02/15/plumbing-codes-revisited-how-they-apply-to-water-treatment/)).
- **What isn't modelled.** No water flows: the pipes only show the direction, the label plate's letters and arrow are
  flat black boxes, and the green tag on the inlet is a plain patch. A real cartridge is round, seals on a gasket and
  sits in a bowl that screws onto a head; here a square block sits in a square cup and nothing seals. A backwards fit
  is scored, not simulated.

## Runtime and seeds

- The scripted demo takes about 15 s of simulated time (305 steps at 20 Hz). In the offline replay the cartridge drops
  into place about 12.1 s in, and the trial ends at 1.0 when that has held for 1 s, about 13.05 s in (261 steps).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
