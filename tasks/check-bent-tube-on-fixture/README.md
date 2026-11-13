# Check a bent tube on its fixture

Check the bent tube on its checking fixture: lift it off the bench with both hands, one on each end leg, lower it level
so all four legs drop into their slots together, and let go so it lies in them under its own weight.

Brake, fuel and hydraulic lines and furniture frames come off CNC tube benders by the thousand. The first tube of
every set-up, and samples from each lot, are checked on a checking fixture built for that one part number: a plate with
a nest block under each straight, each block's slot as wide as the tube plus the part's profile tolerance, the blocks'
heights setting the tube's shape in height. The operator places the tube in the fixture; if it drops into every slot
without force and is not held up anywhere, the tube is good. A 3-D tube has legs at different heights running in
different directions, so it goes in only when it is lowered level, every leg over its slot at the same moment;
offered one leg at a time, the rigid tube catches on a block's rim and passes for a reject. Done by inspectors,
testers, sorters, samplers and weighers (O*NET 51-9061.00, 602,000 US jobs): "Position products, components, or parts
for testing."

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=check_bent_tube_on_fixture.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Check the bent tube on its checking fixture: lift it off the bench with both hands, one on each end leg, lower it level so all four legs drop into their slots together, and let go so it lies in them under its own weight." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. The output of `--scorer task_score` is the
score, taken at the end of the run. Each trial allows 2,400 control steps (120 seconds at 20 Hz). There are no seeds:
the layout is fixed and nothing is jittered. The scripted demo lasts 26.0 s. `success()` requires a score of 1.0.

`pytest` in this folder runs the generated offline checks (no GPU): the scorer on the start, the goal and the named
layouts, and the demo's reach and length.

## The parts

Millimetres; x forward from the arm bases, y left, z up from the bench top. Like every checking fixture, the tube and
its fixture are made for one part number from the data below: the tube bent from stock SAE J526 tube to the bender
points, the nest blocks cut from acetal (POM) bar, the base from 6061 tooling plate, each block placed by the table's
positions, headings and floor heights.

**Tube.** 1/2 in OD x 0.035 in wall (12.7 x 0.89 mm) welded low-carbon steel tube to SAE J526, yellow zinc plated,
0.1738 lb/ft (0.2587 kg/m). Bent on a rotary-draw bender with a 1.5 in (38.1 mm) centre-line-radius die; ends
square-cut and deburred; 535.2 mm developed, 0.139 kg. Bender data (centre-line intersection points as it sits in the
fixture):

| Point | x | y | z | Bend |
| --- | --- | --- | --- | --- |
| A (left end) | 270 | 227 | 61.35 | |
| 1 | 270 | 145 | 61.35 | 46.0 deg |
| 2 | 315 | 90 | 26.35 | 81.9 deg |
| 3 | 235 | 0 | 26.35 | 83.3 deg |
| 4 | 315 | -90 | 26.35 | 82.0 deg |
| 5 | 270 | -147 | 81.35 | 51.3 deg |
| E (right end) | 270 | -229 | 81.35 | |

Straights between tangent points: 65.8 (left end leg), 30.0, 53.5 (left low leg), 53.4 (right low leg), 39.7 and
63.7 (right end leg). The four horizontal straights are the legs the fixture checks; they lie at three heights and
run in three directions.

**Checking fixture**, bolted to the bench. Base: 6061 aluminium tooling plate 90 x 410 x 12.7 (x 232 to 322, y -205 to
205). Four black acetal (POM) nest blocks, 20 long along their leg and 40 wide, screwed to the plate under the legs:

| Block | Leg | Centre (x, y) | Turned (deg) | Slot floor above the bench |
| --- | --- | --- | --- | --- |
| 1 | left end leg | 270.0, 175.2 | -90.0 | 55.0 |
| 2 | left low leg | 275.3, 45.3 | -131.6 | 20.0 |
| 3 | right low leg | 275.3, -45.3 | -48.4 | 20.0 |
| 4 | right end leg | 270.0, -179.3 | -90.0 | 75.0 |

Each slot is 14.7 wide (the tube plus the part's +-1.0 mm profile tolerance, i.e. a 2 mm profile zone) with a flat
floor where the seated leg's underside lies. Its straight walls rise 12.35 above the floor (6 above a seated tube's
centre line), then 45-degree lead-in chamfers widen the mouth by 6 a side, to 26.7 at the block's top, 18.35 above
the floor. On each end leg the block sits 4 past the bend and the hand's pinch 25 past the block.

**Start.** The tube lies on the bench 115 in front of where it sits in the fixture, turned the same way, resting on
its two low legs (their centre line 6.35 up; its centre of mass is 18.8 inside the support they give), so the left end
leg is 35 and the right end leg 55 above the bench, each with room under it for the fingers. Both grippers open at
the ready pose.

## Scoring

Read from the tube's pose and the grippers at 20 Hz; the score is the value at the end.

- **Seated (per block):** at the block's middle, the leg's underside at most 1.5 mm above the slot floor (a 1.5 mm
  feeler gauge does not pass under it) and no more than 2 mm into it (contact tolerance), its centre line within 2 mm
  of the slot's centre line, and the leg running along the slot (within 10 degrees). Unseated legs sit at least
  12 mm higher: on a wall's top or on a chamfer.
- **Released:** both grippers' centre points (between the pads) at least 30 mm from the tube's surface (its bounding
  box would span the whole fixture).
- **At rest:** no point of three reference points (both cut ends and the low corner) moves more than 1 mm over 0.5 s.

Score: **1.0** all four legs seated, released and at rest, held for 1 s; **0.5** all four seated but still held or
not yet at rest; **0.25** two or three seated, released and at rest for 1 s (in the fixture but propped on a rim);
**0** otherwise, including the tube still on the bench, held clear of the fixture, lying on top of the blocks, or
fewer than two legs seated. **Zero for the rest of the run:** the tube falls faster than 0.6 m/s at any time (a drop of
more than about 18 mm, which can bend a tube), or it leaves the bench.

Full credit is seating without force: it is read only once both grippers are 30 mm clear of the tube, so each leg must
lie in its slot under the tube's own weight. A press while holding adds nothing: a real tube that is out of tolerance
can be forced into its slots but springs back out when let go, and the simulated tube is rigid and matches the
fixture, so a press can neither seat a leg that is not over its slot nor hold one down after release. In a real
trial: a 1.5 mm feeler gauge at each block's middle, a ruler for the 30 mm hand clearance, and the video.

## The demo

Both hands work at once, one pinch on each end leg (fingers closing across the leg), and every move is the same for
both hands, so the tube stays level and the two pinches never pull against each other. Twice the demo reads where the
tube is (after the lift, and again 20 mm above the blocks) and shifts both hands' remaining targets by the difference
(at most 8 mm), which takes out the held tube's sag before the legs reach the slots. Times in seconds.

| Time | Both hands |
| --- | --- |
| 0.0-2.5 | Open, over the end legs, 60 mm above the pinch points |
| 2.5-4.0 | Pads down beside the legs (pad tips 10 mm below each leg's centre line) |
| 4.0-5.2 | Pinch both legs |
| 5.2-5.6 | Settle |
| 5.6-7.6 | Lift the tube level, 60 mm, off the bench |
| 7.6-8.4 | Hold (first read at its end) |
| 8.4-11.4 | Carry 115 mm forward over the fixture, every leg 21 mm above its block's top |
| 11.4-12.9 | Lower until the legs are 20 mm above seated (1.7 mm above the blocks' tops) |
| 12.9-13.5 | Hold (second read at its end) |
| 13.5-16.5 | Straight down into all four slots together, until the legs are 0.5 mm above the slot floors |
| 16.5-17.3 | Hold there |
| 17.3-18.5 | Let go: the tube drops the last 0.5 mm onto the floors under its own weight (it is never pressed in) |
| 18.5-20.0 | Rise 60 mm clear |
| 20.0-22.0 | Park |
| 22.0-26.0 | Hold clear |

Why two hands: the bent tube's centre of mass is in the air between its legs, 30 mm from the nearest point of the
tube, and a pinch on a round tube resists turning about the fingers' axis and about the tube's own axis only by pad
friction (roughly 0.08-0.15 N m). Held by one end leg, the tube hangs 0.29 N m off the fingers' axis and droops; the
best single pinch, on the low corner, leaves 0.04 N m to friction, enough to hold it still but little margin to keep
it level through a carry. Two pinches, one on each end leg, hold it by position: only a turn about the line through
them is left to friction, and the centre of mass is 8 mm from that line's vertical plane and 24 mm below it
(0.011 N m). The 12.7 mm tube keeps both fingers of each gripper in their spring range, so each pinch centres itself
on its leg. The scoring does not check which hands were used.

## Fidelity and deviations

- Rigid bodies. The tube is one rigid body shaped from its true centre line: real cylinders on the two end legs and
  convex 24-sided pieces elsewhere (bends cut into wedges of at most 12 degrees, 0.21 mm chord sag; flats tangent to
  the 12.7 mm circle). It is solid rather than hollow; the mass is the true tube's, and the inertia differs slightly.
- The tube matches its fixture exactly. Springback of a forced, out-of-tolerance tube is not simulated, which is why
  only where each leg lies is scored.
- Slot walls are convex pieces with their lead-in chamfers; the slot floors are flat.
- Real fixtures often add end-of-tube checks (a head pin or a length mark at each cut end); this fixture checks the
  four legs only.
- Friction (zinc-plated steel on acetal and on aluminium, 0.25 static, 0.20 dynamic) is an estimate.
- Offline, chains of 12.7 mm balls stand in for the tube's mesh pieces, and boxes for the chamfered walls.
