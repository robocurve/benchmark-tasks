# Fit a U-joint cross longer than its yoke's ear gap: push one journal through, swing it level, slide it back to centre

A driveshaft service step at Spicer 1310 size. A universal-joint cross goes into a tube yoke whose bearing cups are
out, so each ear has a bare 27 mm cross hole. The source is the Spicer driveshaft service manual's reassembly steps:
"Position the journal cross in the yoke cross holes", then "Move one end of the journal cross to cause a trunnion to
project through the cross hole beyond the outer machined face of the yoke ear"
([Spicer Driveshafts Service Manual DSSM3264, page 17, steps 2 and 3](https://www.truckid.com/images/spicer/pdf/driveshafts-service-manual.pdf)).
Driveline and heavy-truck technicians do this every time they replace a U-joint. The task ends at step 2, the cross
sitting in both holes; until it does, no bearing cup can go on.

The hard part is the geometry. The cross's pair of journals is 73 mm tip to tip and the ears are 59.8 mm apart, so
each tip overlaps its ear by 6.6 mm: set down level, the cross lands on the ears' tops. Centred between the ears it
can only turn within 9.5 degrees of line, where its journals are already in the holes; at every other angle a journal
meets an ear or the yoke's crotch. So one journal has to go into its hole at an angle and be pushed at least 11.4 mm
past its ear's inner face, out through the 11 mm ear (the manual's step 3), before the other journal can swing into
its hole. Then the cross is slid back until both journals are in their holes by the same depth. The 27 mm holes leave
6 mm of radial play round the 15 mm journals and nothing stops the cross at centre, so it has to be centred by position.

## The scripted demo
`demo()` uses the right arm. It grips the top journal, lifts the cross out of its holder, and lowers it between the
ears with the flat pair turned 76 degrees. It then swings the pair level while pushing the right journal through the
right hole, until the cross is 11 mm right of centre and that journal 17.6 mm past its ear's inner face. It slides the
cross back to centred and lets go. The journals settle onto the bottoms of their
holes.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=u_joint_cross.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the orange cross in the blue yoke: push one of its two flat journals through one ear's hole, swing the cross level so the other flat journal lines up with the other ear's hole, then slide it back until both journals are in their holes by the same depth." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/u-joint-cross-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=u_joint_cross.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the orange cross in the blue yoke: push one of its two flat journals through one ear's hole, swing the cross level so the other flat journal lines up with the other ear's hole, then slide it back until both journals are in their holes by the same depth." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/u-joint-cross-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/u-joint-cross-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
Either pair of journals can be the one that goes in; the pair nearer the holes' axis is scored.

| Condition | Number | Check on a real bench |
|---|---|---|
| journal in its hole | cross centre between the ears' inner faces; tip at least 1 mm past its ear's inner face; tip centre within 8 mm of the holes' axis | the journal is visible inside the hole from outside the ear |
| installed | both journals of the pair in their holes, each tip at least 3.5 mm past its ear's inner face (6.6 mm when centred: within 3.1 mm of centred); pair within 5 degrees of the holes' axis | a depth gauge into each hole from outside reads 4.4 mm, give or take 3.1 mm |

| Score | Condition |
|---|---|
| 1.0 | installed and still (under 1 cm/s and 0.5 rad/s) for 1 s (ends the trial) |
| 0.75 | installed at the last step, but not still for 1 s |
| 0.5 | both journals in their holes, but off centre or turned |
| 0.25 | one journal in its hole, for example the cross pushed deep into one ear with the other journal out |
| 0 | the cross in its holder, resting on the ears' tops, outside the yoke, or on the table |

The hooks don't see the grippers, so a cross held installed and still counts the same as one let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo reaches success at about 34 s.
- `python -m pytest -q test_u_joint_cross.py` checks the scoring and the demo's reach without the simulator.
- `python make_meshes.py .` rewrites the two USD files; the tests check they match.

## Fidelity
The yoke is fixed, as if clamped in a vise, with simple slab ears round the holes. The cross is one rigid steel body,
200 g, with smooth journals (no needle rollers, seals or grease fitting). The hole size and the distance over the ears'
outer faces are the Spicer 1310 catalog's figures (1.062 in bearing caps, 3.219 in over the caps,
[Spicer K350 catalog, Outside Snap Ring style, kit 5-153X](https://cdn.shopify.com/s/files/1/0077/1538/9503/files/K350-2-DAG-122017.pdf)).
The journals' tip-to-tip span (73 mm), their diameter, the ear thickness (and so the 59.8 mm ear gap), the hub size
and the cross's mass are not in the catalog; they are our estimates, set to look like a 1310 cross. Every object has rubber-like friction (1.2 static, 1.0 dynamic).
