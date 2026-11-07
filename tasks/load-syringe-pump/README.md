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
The syringe is **in the cradle** when its barrel's axis is within 3 mm of the cradle's centre line sideways, at most
15 mm above its seat and within 15 degrees of the cradle's line. It is **seated** when the axis is within 1 mm of its
seat, level within 1.5 degrees, and the flange's centre is within 0.8 mm of the slot's. The plunger is **pushed in**
when its stopper has moved more than 1 mm toward the nose.

| State | Score |
|---|---|
| Plunger pushed in, at any point of the rest: a dose given | 0 |
| Syringe not in the cradle: on its stands, held above, off the side | 0 |
| In the cradle, not seated: flange on the body beside its slot, syringe cocked | 0.2 |
| Seated, driver still back | 0.5 |
| Seated, driver within 5 mm of the thumb disc | 0.75 |
| Seated, driver within 1 mm, everything still, held 1 s | 1.0 |

Possible scores: 0, 0.2, 0.5, 0.75, 1.0. Ramming the driver home pushes the plunger in and scores 0. On a real pump: a
side photo for the flange in its slot and the driver at the thumb disc, and the barrel's graduations for the plunger
unmoved.

## Design notes
- **Where the pieces come from.** `mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223
  (MIT License; the notice is at the top of the file): the pump is a fixed body with the driver on a sliding joint, and
  `score()` reads the driver's position. `parts.py` (ours) builds the barrel and the plunger as loose parts from boxes.
- **How precise it has to be.** The flange has 0.5 mm of play a side in its slot. The barrel's axis has to end within
  1 mm of its seat, and the driver within 1 mm of the thumb disc while moving the plunger no more than 1 mm. Set down
  with its axis 1.55 mm off the cradle's line, the syringe did not seat (0.2; official sim, traced, 2026-11-06).
- **The flange slot's mouth** is rounded (5 mm), as a flange clip's lead-in is. With square edges the flange landed on
  the pump's body beside the slot and the syringe sat cocked 8 degrees: the misloaded flange PMDA warns about.
- **The plunger.** The stopper slides with little friction (a real one needs a firm push to start), so any contact
  moves it a little. The scorer allows 1 mm of plunger travel: the driver can come up against the thumb disc, but
  driving it home pushes the plunger in and scores 0.
- **Sizes** are a stand-in for a 60 mL syringe (barrel 29.6 mm across).

## Runtime and seeds
- Per trial on one L40S: the scripted demo reaches 1.0 at step 578 (29 s of simulated time, about 3 min of wall clock
  with stored frames); Dream-1 uses all 1,200 steps (about 6 min) and scored 0.0: its right arm reached over the syringe
  on its stands and never lifted it.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_load_syringe_pump.py` checks the scoring without a GPU (`python -m pytest -q test_load_syringe_pump.py`): the
  start, every credit level, the driver, seat and flange limits either side, a pushed plunger, a cocked syringe, one off
  the cradle or held above it, motion, the layout and the setup text's positions, and the demo's reach. A mutation check
  (16 deliberate breaks of the scorer, layout and demo, each run against the tests) catches 16 of 16.
