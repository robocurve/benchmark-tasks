# Hang a hotel hanger back on its loose anti-theft ring

A hotel closet rod carries two stamped-steel security rings (the 1.5 in A-ring sold for ball-top hangers). Ring A
holds hanger A; ring B is empty. Hanger B, a 17-1/2 in wooden suit hanger with a chrome ball top, lies flat on a shoe
box. The robot must put hanger B's ball into ring B's cup and leave it hanging there.

The ring is loose on the rod: it slides along it, swings about it, and turns up to about 30 degrees about the
vertical, where it stays. The cup is the ring's bottom tab bent out into a small shelf with a 2.8 mm slot open at its
edge and a low lip, so the ball can only go in from the side, held over the lip, and then be let down. Pushed by the
hanger, the 12 g ring slides or turns away. The demo holds the ring square with one hand while the other brings the
ball in; a careful single hand can also do it (the `nohold` run below), and the score only judges the end state.

## Run and score a trial

```
inspect-robots run --embodiment yam_arena -E task=hotel_hanger.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Hang the hotel hanger lying on the shoe box back on the empty security ring on the closet rod, with its ball top resting in the ring's cup. The ring is loose: it slides along the rod and turns on it at a touch. Leave the hanger hanging free and square to the rod like the hanger beside it, the ring where it was, and take both hands away." \
  --scorer task_score --max-steps 800 --policy <policy> --seed <seed>
```

`dev/run.sh <name> <policy> <seed> 800 [HANGER_FAIL=...]` does the same and writes a per-step log (hanger B's ball
in ring B's frame, whether each hanger is seated, ring B's place and turn, hanger B's heading, the fingers' clearance)
and, at the end, the score report. The score is read at the end of the trial (`SCORE_OVER = "end"`).

- Run limit: 800 steps (40 s at 20 Hz). The scripted demo's plan takes about 690 steps.
- Runtime per trial: about 2.5 minutes wall clock for the scripted demo on one GPU, start-up included. The scripted
  episode ends when its plan ends (712 steps); other policies run to the limit.
- Seeds: any. Each seed shifts ring B up to 40 mm along the rod and turns it 8 to 22 degrees either way about the
  vertical, and shifts hanger B and its shoe box up to 10 mm in x and 20 mm in y and turns them up to 10 degrees.
  The framework turns `--seed N` into an episode seed, so seed 0 is not the reference layout.

| Seed | Ring B shift along the rod | Ring B turn | Hanger B shift (x, y) | Hanger B turn |
|---|---|---|---|---|
| 0 | +16.4 mm | +19.4 deg | (-6.2, +3.8) mm | -1.4 deg |
| 1 | +28.4 mm | -15.9 deg | (-2.5, -17.0) mm | +9.0 deg |
| 2 | +19.6 mm | +15.5 deg | (+4.6, +17.5) mm | +7.1 deg |

## Scripted demo

`--policy yam_scripted` plays the demo. It reads object poses from the simulator to plan its motions and moves the
parts only through the grippers, by contact. The right hand comes in from the front and pinches ring B's plate at the
tab's front corner, which turns the ring square to the rod and holds it there. The left hand comes down 60 mm in front of
hanger B's hump, slides in level and pinches the hump across its thickness where it sticks out past the shoe box,
lifts it, turns it square to the rod while still flat and stands it up, brings the ball beside the cup
and over the lip, and lets it down into the cup. Twice on the way (beside the cup and over it) it re-reads where the
ball and the cup are and corrects its remaining moves. Then the left hand opens and backs straight out along the
hanger, and the right hand lets go of the ring.

## Results (final code)

| Run | Policy | Seed | Score | Steps | End state |
|---|---|---|---|---|---|
| demo | yam_scripted | 0 | 1.0 | 712 | ball seated (0.0 / 4.8 / -28.7 mm in ring B's frame), square 0.9 deg, ring B moved 3.9 mm, ring A 3.8 mm, hands clear, still (video run) |
| demo | yam_scripted | 1 | 1.0 | 712 | seated, square 1.8 deg, ring B 0.9 mm, ring A 3.8 mm |
| demo | yam_scripted | 2 | 1.0 | 712 | seated, square 0.8 deg, ring B 6.7 mm, ring A 3.7 mm |
| nothing | yam_scripted, `HANGER_FAIL=nothing` | 1 | 0.0 | 800 | untouched start: hanger B still on the shoe box |
| fixed plan | yam_scripted, `HANGER_FAIL=fixed` | 1 | 0.0 | 712 | plan made for the reference layout: ball 70 mm out from the cup and 227 mm low, not seated |
| ring not held | yam_scripted, `HANGER_FAIL=nohold` | 0 | 1.0 | 654 | seated, square 0.4 deg; the ring kept its 18 deg turn and moved 3.4 mm: a careful single hand also succeeds |
| hanger held | yam_scripted, `HANGER_FAIL=holding` | 0 | 0.0 | 800 | seated, but the left hand never lets go (fingers within 30 mm) |
| Dream-1 | dream | 0 | 0.0 | 800 | hanger B never lifted off the shoe box; ring B untouched (0.1 mm) |

Wrong outcomes (`HANGER_FAIL=...`, the same demo with one thing changed): `nothing` (no motion), `fixed` (the plan
made for the reference layout, no poses read), `nohold` (the ring is never held), `holding` (the left hand keeps hold
of the hanger).
