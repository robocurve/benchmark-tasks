# Release a ratchet strap, lever held, handle folded flat

**Instruction:** "Release the strap so it goes slack, and let go."

A cooler stands on a section of pickup-bed floor, strapped down across its lid by a 1 in ratchet tie-down hooked to
stud rings on two lengths of L-track. The ratchet sits on the strap's right leg with its spool axis pointing at the
robot and its handle closed. To free the strap, the lock release on the handle has to be pulled and the handle swung
through 180 deg until it lies flat and locks open; only then does the spool turn free. Swung open without the
release, the handle stops hard at the crank stop at 115 deg, which looks fully open while the strap stays tight; let
go short of flat, and the cam that lifts the holding pawl pushes it back. The release is a sprung tab on the handle
itself, so the same pinch that swings the handle has to keep it pulled through the half-turn.

The maker's words (Uline ratchet tie-down instructions IH-1217, https://img.uline.com/is/content/uline/IH-1217pdf):
step 3, if the handle is closed and locked, pull on the lock release handle to disengage; step 5, to release tension,
pull on the lock release handle and rotate the main handle 180 deg to lock it in the open position, and the mandrel
is free to turn and release the webbing. Rhino USA's guide says the same: swing the handle all the way open until it
is flat (https://www.rhinousainc.com/blogs/news/how-to-release-a-ratchet-strap).

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key configured, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=release_a_ratchet_strap_to_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Release the strap so it goes slack, and let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/ratchet-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=release_a_ratchet_strap_to_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Release the strap so it goes slack, and let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/ratchet-demo
```

The run prints `task_score`, scored on the final state (or it stops once 1.0 has held).

- One fixed layout (`object_jitter=0`); the seed changes nothing. Runtime per trial: up to 1,200 steps (60 s of sim
  time at 20 Hz); the scripted demo reaches full score at about 25 s (499 steps).
- Demo modes (`KIT_DEMO`): `demo` (default), `none` (the arms stay home), `crank_stop`, `pump`, `early_letgo`,
  `short_of_flat`, `flat_then_back`, `held`, `lock_then_close`, `shove` (see the table below).
- Tests: `PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` in this folder.

## Runs

In the simulator (Isaac, one fixed layout): the scripted demo scored 1.0 at about 25 s (500 steps of 1,200): the
release pulled 8 mm from 6.5 s, the handle swung 0 to 180 deg from 7 to 19.5 s, the release let go at 20.5 s and the
handle locked flat (strap slack), then the gripper backed clear and the handle held at 180.00 deg untouched. Dream-1
scored 0.0 over all 1,200 steps (the handle never left its closed notch). Offline MuJoCo check of every demo mode (physcheck, rigid friction); the demo also scored 1.0 with handle
friction +-20%, the release spring x0.5 and x2, handle mass +-25% and the cam doubled to 0.6 N m (sweep run before the bed mass was set to its real 12 kg):

| Mode | What it does | Expected | MuJoCo check |
|---|---|---|---|
| demo | release pinched, handle swung to flat, let go there | 1.0 | 1.0 |
| none | arms stay home | 0 | 0 |
| crank_stop | release pulled only to free the closed notch, then swung by the grip alone; stops at 115 deg | 0 | 0 |
| pump | as crank_stop, then folded back unreleased (winds 3 teeth) | 0 | 0 |
| early_letgo | release held to 140 deg, let go there | 0.25 | 0.25 |
| short_of_flat | release held to 170 deg, let go; the cam pushes it back | 0.25 | 0.25 |
| flat_then_back | to flat, backed off to 168 deg, let go | 0.5 | 0.5 |
| held | locked flat, still holding the handle | 0.75 | 0.75 |
| lock_then_close | locked flat, then the release pulled again and folded back to 140 deg | 0.5 | 0.5 |
| shove | the left hand pushes the cooler 30 mm | 0 | 0 |

## Scoring (the scorer is `grade()` in the task file)

| Score | State (first that fits) |
|---|---|
| 0 (sticky) | the cooler moved more than 10 mm or tilted more than 5 deg at any time, or the strap was wound 2 or more teeth tighter |
| 1.0 | locked flat (within 3 deg of flat, release in the flat notch), strap slack, both grippers 30 mm or more from the ratchet, handle still (less than 1 deg over the last 1 s) |
| 0.75 | locked flat and slack, but still held or not still |
| 0.5 | the handle came within 5 deg of flat at some point but is not locked flat now |
| 0.25 | the handle got past the crank stop (125 deg or more) but never reached flat, and is not locked flat |
| 0 | anything else: the untouched start, a handle opened only within its crank range, a strap wound tighter at the end |

**Measuring it in a real trial.** Handle angle: a protractor or phone level on the handle against the base (1 deg),
and a 5 N push on the grip toward closed (10 N scale) moves a locked handle less than 3 mm. Slack: a 2 N spring scale
hooked under the strap at the middle of the lid lifts it 30 mm or more (steel rule). Clearance and stillness: the
video over the last second. Teeth: the base pawl's clicks on the video, or chalk marks on the spool's wound webbing.
Cooler: tape marks at two corners of its base and a phone level on the lid.

## Parts (real bench)

- Kinedyne 711281 1 in x 12 ft wire-hook ratchet tie-down (4-pack 711281PK, about 23 USD) with its 800 wide-handle
  ratchet (59 x 41 x 121 mm, 0.34 kg, https://kinedyne.com/Kinedyne_1_Wide_Handle_Ratchet).
- Coleman Chiller 28 qt cooler with wheels (20 x 13.5 x 16.5 in, 7.95 lb), six 1 L water bottles inside.
- Two 24 in aluminium L-track sections with single-stud fittings and stainless O-rings.
- A 80 x 74 cm section of 3/4 in plywood with a ribbed rubber bed mat, screwed to the table.

## Safety

A real trial is safe to run with people nearby: no sharps, heat or power. The arm needs at most about 14 N (the
release spring fully pulled) and about 3.5 N at the grip against the cam; the strap's 200 N is let off by the spool
turning free once the handle locks flat, so the handle does not kick. The floor section is screwed to the table and
the cooler stays strapped to it until release. When resetting, keep fingers out from between the handle's side plates
and the base (a pinch point) while pumping the handle.

## Model notes

- The ratchet's internals (release spring 10-14 N, crank stop 115 deg, cam 0.3 N m over 160-178 deg, flat notch at
  178 deg, 11 teeth) are estimates to measure on the bought unit; the cam stands for the handle's push needed to lift
  the loaded base pawl near flat (about 3.5 N at the grip).
- The webbing is drawn, not simulated as cloth; a declared rule holds 200 N while the base pawl holds and drops it to
  0 once the handle locks flat, when the drawn leg swings out slack. The ratchet's base is fixed on its strap (a real
  one rocks a few mm under the swing).
- The release slide is a hinge on the handle with a 200 mm virtual pivot, so its 8 mm travel is straight to 0.2 mm.
- The floor section is a free body of its real 12 kg standing on the table (screwed down on the real bench).
- No third-party assets: every part is built from boxes at the products' sizes.
