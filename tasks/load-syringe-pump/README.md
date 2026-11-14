# Load a filled syringe into a syringe pump: flange in its slot, driver up to the plunger without pushing it

A syringe pump stands on the table with its plunger driver pulled back, and a filled syringe lies beside it on two
stands. The robot loads it the way a nurse does: it lays the barrel in the pump's cradle with the flange at the barrel's
end down in the flange slot, then slides the plunger driver along its track until it is up against the end of the
plunger, without pushing the plunger in. The steps are from the Medfusion 3500 quick reference guide: "Make sure the
flange on the syringe barrel is secure in the syringe barrel flange clip", then "gently advance the plunger driver toward
the syringe plunger" until "it is flush with the syringe plunger".

Both halves are physical. The 2.5 mm flange has to go straight down into a 3.5 mm slot while the barrel settles into
its cradle, so the syringe has to arrive over the slot within a few millimetres and level. Then the driver has to be
brought up to a plunger that is free to move: overshoot by a millimetre and the plunger goes in (a dose the patient
wasn't meant to get), stop short and the pump would siphon or under-deliver. One arm loads the syringe; the other works
the driver.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. The standard `yam_arena` embodiment runs it: importing
the task file installs the pump (`mechanism.py`) and the syringe and plunger (`parts.py`) into the scene, so nothing needs
installing.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_syringe_pump.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the filled syringe into the syringe pump. Lay it in the pump's cradle with the flange at the end of its barrel down in the flange slot, then slide the grey plunger driver along its track until it is up against the end of the plunger, without pushing the plunger in." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/load-syringe-pump-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_syringe_pump.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the filled syringe into the syringe pump. Lay it in the pump's cradle with the flange at the end of its barrel down in the flange slot, then slide the grey plunger driver along its track until it is up against the end of the plunger, without pushing the plunger in." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/load-syringe-pump-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/load-syringe-pump-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the syringe is seated, the driver is up against the
plunger and everything has been still for 1 s; otherwise it scores the last step.

## Scoring
The syringe is **in the cradle** when its barrel's nose end rests on the cradle's rods (the axis there within 3 mm of
the cradle's centre line sideways, 2.5 mm of its seated height and 5 mm of its seated place along the cradle) and the barrel is within 15 degrees of the cradle's
line. It is **seated** when the axis is within 1 mm of its seat (sideways and up), level and square to the cradle within
1.5 degrees, and the flange's centre is within 0.8 mm of the slot's. The plunger is **in the barrel** when its stopper is
within 1 mm of the axis and its rod within 5 degrees of it.

| State | Score |
|---|---|
| Plunger pushed in more than 1 mm at any point (a dose given), even if pulled back later | 0 |
| Plunger drawn back more than 3 mm at any point, or out of the barrel | 0 |
| Syringe not in the cradle: on its stands, held above, off the side or off the rods' end, turned end for end | 0 |
| In the cradle, not seated: flange on the body beside its slot, syringe cocked | 0.2 |
| Seated, driver still back (or its face more than 0.2 mm past the thumb disc: it holds nothing) | 0.5 |
| Seated, driver within 5 mm short of the thumb disc | 0.75 |
| Seated, driver from 1 mm short of the disc to 0.2 mm into it (contact overlap), everything still (under 1 cm/s and 0.5 rad/s; the driver moving under 0.5 mm a step) | 0.9 |
| The same, held 1 s: the trial ends there | 1.0 |

Possible scores: 0, 0.2, 0.5, 0.75, 0.9, 1.0. A trial that ends without the full state having held 1 s scores its last
step, so a syringe that settles only in the final second gets 0.9. Ramming the driver home pushes the plunger in and
scores 0, and so does pulling the plunger out of the barrel. On a real pump: a side video for the flange in its slot,
the driver at the thumb disc and the plunger never moving off its graduation.

## Design notes
- **Where the pieces come from.** `mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223
  (MIT License; the notice is at the top of the file): the pump is a fixed body with the driver on a sliding joint, and
  `score()` reads the driver's position. `parts.py` (ours) builds the barrel and the plunger as loose parts from boxes.
- **How precise it has to be.** The flange has 0.5 mm of play a side in its slot. The barrel's axis has to end within
  1 mm of its seat, and the driver from 1 mm short of the thumb disc to 0.2 mm into it (the official demo ends at
  -0.007 mm, touching) while the plunger moves less than 1 mm in. A run that
  seated the syringe and then drew the plunger 27 mm back, nearly out of the barrel, scored 0 (official sim, 2026-11-07).
- **The cradle** is two rods the barrel rests between, touching it 45 degrees either side of its lowest line, so a
  barrel set down 2 mm off the line rolls onto it (official sim). What decides the score is the flange in its 3.5 mm slot
  and the driver at the plunger, not where the arm let go across the cradle.
- **The flange slot's mouth** is rounded (5 mm), as a flange clip's lead-in is. With square edges the flange landed on
  the pump's body beside the slot and the syringe sat cocked 8 degrees: the misloaded flange PMDA warns about.
- **The plunger.** The stopper slides with little friction (a real one needs a firm push to start), so any contact
  moves it a little. The scorer allows 1 mm of plunger travel in and 3 mm out: the driver can come up against the thumb
  disc, but driving the plunger in, or drawing it back or out, scores 0. The plunger's travel is recorded from the
  reset, so a push that is later pulled back still counts.
- **Sizes** are a stand-in for a 60 mL syringe (barrel 29.6 mm across).

## Runtime and seeds
- Per trial on one L40S: the scripted demo reaches 1.0 at step 575 (29 s of simulated time, about 2 min of wall clock
  with stored frames); Dream-1 uses all 1,200 steps (about 4 min) and scored 0.0: its arms moved over the table, but
  the syringe stayed on its stands. On the revised task (2026-11-12) the stock Dream adapter rejected Dream-1's first
  reply (a left gripper of -0.062, past its 0.05 slack around 0..1) on both tries; with that slack widened to 0.1 for
  one run, Dream-1 again used all 1,200 steps and scored 0.0, the syringe on its stands.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start. The demo also reaches 1.0 started 0.05
  and 0.10 s late (`demo(delay=...)`), and with the syringe set down 2 mm either way across the cradle or 1 mm either
  way along it (official sim, no stored frames, 2026-11-07). After the revision (official sim, inspect-robots 1.0.0,
  2026-11-12): the demo reaches 1.0 at step 575, also started 0.05 and 0.10 s late, and ends with the driver 0.007 mm
  into the thumb disc; the same demo with the driver aimed 5 mm past touching pushes the plunger in and scores 0.
- `test_load_syringe_pump.py` checks the scoring without a GPU (`python -m pytest -q test_load_syringe_pump.py`): the
  start, every credit level, each limit just inside and just outside (seat, tilt, yaw, flange, in the cradle, driver,
  stillness and spin, a driver still moving, plunger in, out and in its barrel), a push or pull later undone, the
  plunger pulled out and left anywhere or turned end for end, the driver run past a plunger drawn out of its way, a
  push then a retract (0 for the rest of the trial), negative gaps, the syringe off the rods' end or turned end for end, the
  1 s hold (the complete state alone scores 0.9), the layout, the setup text's numbers, and the demo's reach and
  delay. A mutation check (35 deliberate breaks of the scorer, layout and demo, each run against the tests) caught 35
  of 35 before the revision (2026-11-07).
