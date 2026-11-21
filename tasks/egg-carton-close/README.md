# Close a dozen-egg carton with its locking flap tucked inside the lid

A moulded-pulp dozen-egg carton stands open on the counter, holding twelve wooden stand-in eggs. The lid stands back
past upright, held open by the springy pulp hinge, and the front locking flap hangs outward. The robot must close the
carton so it stays shut, without pressing on the eggs or damaging the carton. The scene, scorer and scripted demo are
in `egg_carton.py`.

Closing an egg carton is something almost every adult does several times a week. The pulp carton only stays shut
when the front flap is folded up inside the lid's front edge: a bead along the top of the flap then sits above a bead
inside the lid's skirt, and the flap's own spring keeps it pressed outward under the skirt. Shut the lid over a flap
that is still hanging out and the lid springs back open. A tucked flap that is let go creeps back out (soft pulp, about
5 degrees a second), so the flap has to be held in until the lid is most of the way down, then the lid brought the
rest of the way down over it. People do this with two hands: one folds and holds the flap, the other brings the lid
over. They do it gently: pressing the lid or the flap onto the eggs cracks them, and a dropped or upended carton spills
them. Latching takes a light press, and a carton can be lifted or nudged gently without harm. The scripted demo does
the same with the two arms.

## Origin

First proposed in pull request #2368 (reviewed as TK-MFPNK8, then TK-F84WBY); carried forward here with the changes below.

## Revision (review TK-3JWKR5)

| Reviewer point | What changed |
|---|---|
| A harmless carton rotation over 20 degrees triggers simulated damage; compute tilt from the carton's vertical axis | `_tilt` now measures the angle between the carton's vertical axis now and at the start, so turning it on the tabletop never counts; tipping off flat still does past 20 degrees. Tests: a 40-degree turn on the table is not damage, a 90-degree turn reads 0 tilt, a 25-degree tip reads 25 and is damage. |
| A shift over 1 cm halves the score despite a successful closure; award the same score for an intact, latched carton | The 1 cm rule and the 0.5 tier are gone: an intact, latched, released carton scores 1.0 wherever it has been slid. Damage and real tipping keep their zeros. Test: a carton slid 3 cm and closed scores 1.0. |

## Claimed revision

This revision builds on pull request #2368, whose hold lapsed without a revision. It answers the review of TK-F84WBY:

| Reviewer point (TK-F84WBY) | What changed |
|---|---|
| "A 500 g increase on the scale measures any downward force on the carton, not specifically pressure on eggs" | The scale is gone. A real trial now judges damage to the eggs and carton directly, after the trial and from video: a new dent or crease in the pulp over any egg (the pulp is softer than the wooden eggs, so a press onto an egg dents it there), a torn flap button, lid hole, hinge or tray, an egg out of its cup, the carton dropped more than 5 cm or tipped past 20 degrees, or a fingertip on an egg. A carton only qualifies if latching it by hand leaves no dent, so ordinary latch pressure cannot set the record. |
| "a 3 mm lift also permanently zeros an otherwise intact closure" | The lift zero is gone in both the real trial and the simulation. Lifting, nudging or setting the carton down gently is allowed; only a drop counts (in the simulation, the carton falling faster than 1 m/s, a fall of about 5 cm). The 3 degree tip limit, which a nudge could also reach, is now 20 degrees (one end raised about 10 cm). |
| "corresponding simulation checks that allow ordinary latch pressure and safe handling" | The simulated press check now counts only once the carton is latched, when any further push on the lid goes into the eggs, and only a press of more than 5 N held for 0.25 s: the press that carries the lid's bead over the flap's comes before latching and is not counted, and the bump of the bead snapping in is too brief. The lid and flap angle limits (onto the egg tops and the front row) are unchanged. New offline tests (`test_egg_carton.py`) check that a 4 N latch press with an 8 N snap, a 5 mm and a 3 cm lift, and a 5 degree tip all keep full credit, and that a held 12 N press on the latched lid, a drop and an upended carton score 0. |

The sourcing, egg stand-in and setup answers to the first review (TK-MFPNK8), below, were accepted and are kept.

## Earlier revision

Revision 1 of pull request #2368 answered the review of TK-MFPNK8:

1. **Construct validity:** "The score permits closing the carton after crushing its eggs or damaging the pulp,
   although either would defeat the real task." The score keeps a damage record over the whole trial; once it is
   set, the trial scores 0 whatever the end state. Its conditions (revised above) are listed under Scoring. The
   scripted wrong attempts `crush`, `flap_press` and `egg_touch` each reach a properly locked end state and are
   meant to score 0 (table below).
2. **Scoring:** "awards full credit without checking whether the eggs and carton remain intact." Full and half credit
   both need the damage record to be clear, and the record is part of `score()` in the simulation. In a real trial
   it is judged by the checks under Scoring.
3. **Clarity and reproducibility:** "No source or build method specifies a repeatable carton with the stated
   full-width flap, beads and spring behavior, and the eggs' real-trial type is unspecified." The carton is a blank
   natural-pulp 2 x 6 flat-top dozen carton with a front locking flap, about 30.5 x 10.2 x 7.6 cm and 54 g; for
   example Hatching Time 12-Egg Cartons (2x6), Natural Pulp, Flat Top Blank, SKU EC-FT2x6-25 (packs of 25). The flap's
   two moulded buttons and the lid's matching holes are modelled as the two beads.

   Setup and reset:
   1. Open the lid back to 100 degrees (past upright), fold it back and forth twice so it rests there, and let the
      flap hang out.
   2. A carton qualifies if, shut over an untucked flap and released, its lid springs back past 45 degrees within 2 s,
      and if, with the flap tucked and the lid closed over it by hand with a normal press, it stays shut and the pulp
      shows no new dent over the eggs.
   3. Replace the carton after 10 trials or at the first new dent or tear.

   The eggs are named in the next point.
4. **Feasibility:** "Unspecified eggs could break and contaminate the shared rig, so a safe, repeatable substitute or
   protection is needed." The eggs are 12 solid, unfinished wooden craft eggs, 4.4 cm across and 6.4 cm long, about
   35 g each; for example Hygloss Wooden Eggs, 1-3/4 x 2-1/2 inch, 12 pack. They cannot break or leak, and they are the
   same in every trial. The simulation uses the same size and mass, so the carton with eggs is 0.5 kg.

The simulated carton is now welded into one piece (see Simulation notes), so the score's lift and tip checks apply to
the whole carton.

## Files

| File | What |
|---|---|
| `egg_carton.py` | The yam_arena task file: objects, `score()` and the scripted demo |
| `partkit.py` | Rigid parts built from boxes and cylinders (from the accepted `din-rail-mount` task) |
| `hingekit.py` | Sprung revolute hinges on partkit parts (written for this task family, MIT) |
| `gripkit.py` | Gripper orientation helpers for the scripted demo |
| `test_egg_carton.py` | Offline tests of the damage record (no simulator) |
| `task.json` | The submission details |

## Run a trial

In the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/egg-carton-close/egg_carton.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the egg carton." \
  --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream
```

Scripted demo: the same flags with `--policy yam_scripted --max-steps 380`. The score is printed as `task_score`.
Wrong attempts for the shortcut checks: set `EGG_VARIANT` to `lid_only`, `slow`, `crush`, `flap_press`, `egg_touch` or
`nothing` before the scripted run. `EGG_DEBUG=1` prints the lid and flap angles, carton movement and tilt, the
finger-to-egg gap and the damage record every 10 steps.

## Scoring

The lid angle is 0 when shut, 100 degrees as found and negative when pressed past shut. The flap angle is 0 when
upright, positive when tucked in toward the eggs and -60 degrees as found.

- **Damage record (whole trial):** set at the first step at which any of these happens (simulation, with the real
  trial's check in brackets):
  - the lid is more than 6 degrees past shut, pressed onto the egg tops (a new dent or crease in the lid top);
  - the flap is more than 16 degrees inside upright, pressed onto the front row (a new crease in the flap);
  - once the carton is latched (lid within 5 degrees of shut, flap within 6 degrees of upright or further in), more
    than 5 N is carried by the lid's and flap's hinges together for 0.25 s: the latched lid pushed down onto the eggs
    (a new dent in the pulp over any egg). The press that latches it comes before and is not counted;
  - a finger is within 5 mm of an egg's exposed part (a fingertip on an egg in the wrist-camera video);
  - the carton falls faster than 1 m/s, a drop of about 5 cm (the video), or tips more than 20 degrees off flat (its vertical axis; turning it on the table is not tipping) (an egg out
    of its cup, or the video);
  - the carton's two halves are pulled 3 mm apart (a tear in the tray, or a flap button, lid hole or hinge torn).

  Lifting, nudging or setting the carton down gently is not damage. Front-row egg sides do not count while the flap
  stands between the finger and the egg. The record is cleared only when the scene is reset. In a real trial the
  dent, crease and tear checks are made after the trial against a photo taken before it, at each of the 12 egg
  positions, and all 12 eggs must still be in their cups.
- **1.0**: no damage, and at the end:
  - the lid is within 5 degrees of shut (and no more than 6 degrees past it);
  - the flap is within 6 degrees of upright or further in (inside the skirt, where its bead holds the lid);
  - no finger is within 3 cm of the carton's closed outline;
  - the lid has been still (moving under 0.5 degrees) for the last 1 s.
- **0** otherwise.

Without the flap inside, nothing holds the lid down, and the pulp hinge opens it again as soon as it is let go.

### Shortcut checks (scripted, same scene)

| Attempt | What happens | Score |
|---|---|---|
| `EGG_VARIANT=lid_only`: shut the lid with the flap left hanging out | the lid springs back open when released | 0 |
| `EGG_VARIANT=slow`: tuck the flap, let go, close the lid 2.5 s later | the flap has crept out; the lid springs back open | 0 |
| `EGG_VARIANT=crush`: lock it, but keep pressing the closed lid down onto the eggs | locked at the end, but the press loaded the latched lid and flap hinges with over 5 N | 0 (rerun on this revision) |
| `EGG_VARIANT=flap_press`: fold the flap hard in against the eggs | locked at the end, but the flap went past 16 degrees | 0 |
| `EGG_VARIANT=egg_touch`: rest a finger on a front egg, then close it properly | locked at the end, but a finger touched an egg | 0 |
| Hold the lid shut at the end | fingers within 3 cm of the carton | 0 |
| `EGG_VARIANT=nothing` | the carton stays open | 0 |

## Tests

From this folder in the Arena environment: `python -m pytest test_egg_carton.py`. They drive `score()` on handcrafted
carton states with set hinge forces, with no simulator: ordinary latch pressure, a gentle lift and a slight tip keep
full credit; a held press on the latched lid, the lid or flap pressed onto the eggs, a drop and an upended carton
score 0.

## Results

Runs of this revision (egg_carton.py SHA-256 prefix 160654f75ce8: vertical-axis tilt, no shift penalty), seed 0,
`object_jitter=0`, logs in `runs/`:

- Scripted demo: **1.0** at step 366 of 2,400 (`runs/demo_log.json`); the lid locked, the damage record stayed clear.
- `test_egg_carton.py`: 12 passed, including a 40-degree turn on the table (no damage) and a closure after a 3 cm
  slide (1.0).
- Dream-1: **0.0** at the 2,400-step limit, rerun on this revision (2026-11-20). The arms hover beside the carton and
  the lid stays open; nothing is closed or latched.
- `EGG_VARIANT=crush` rerun under the latched 0.25 s hold: **0**.

The submitted videos are from these runs (the demo video shows the top camera enlarged beside both wrist cameras).
Results of the earlier revision, before the damage-check changes:

- Scripted demo: **1.0**, twice. The lid locked at 2.2 degrees with the flap 0.8 degrees in, the carton moved 0.4 mm,
  the peak press (lid near shut) was 1.3 N, and the damage record stayed clear.
- Dream-1: **0.0** over 2,400 steps. The arms stay beside the carton, and the right one reaches over its back twice and
  nudges it 3 mm. The lid stays open and the flap out; the damage record stayed clear.
- Every shortcut variant in the table above: **0**.

## Simulation notes

- Rigid bodies built from boxes and cylinders. The carton is modelled as two rigid halves (front and back rows of six
  eggs, 0.25 kg each) joined by a fixed joint along the middle of the cup tray, so it moves as one 0.5 kg piece. Two
  halves are needed because the lid and the flap must be able to touch each other: the lid's hinge belongs to the back
  half and the flap's hinge to the front half.
- Hinges are revolute joints with torsion springs (`hingekit.py`):
  - lid: 0.02 N m/rad (about 0.35 N at the lid's front edge when shut); it stops on the egg tops 12 degrees past shut;
  - flap: 0.0086 N m/rad (about 0.3 N at its top when upright) with 0.1 N m s/rad damping (soft pulp: a tucked flap
    creeps back out at about 5 degrees a second); it stops on the front eggs 18 degrees inside upright.

  The lock is two 3 to 4 mm beads: one inside the lid's front skirt, one along the top of the flap with a 45 degree
  lead-in above it.
- The eggs are 4.4 cm cylinders showing 2 cm above the 4.5 cm cup tray.
- The press force is the joint reaction force of the lid's and the flap's hinges, read from PhysX
  (`get_link_incoming_joint_force`). Once latched, the flap props the lid, so a push on the closed lid loads both
  hinges; on a real carton the lid would rest on the egg tops. The scripted demo's own press stays under 5 N
  (1.3 N peak); lifting the flap with the lid still open loads the flap hinge with up to about 4.5 N, which is why the
  force only counts once the carton is latched.
- The eggs are part of the carton's two rigid halves, so they cannot leave their cups in the simulation: the drop
  and tip limits stand in for spilling them.
- The fingers, for the egg-contact check, are a 1 cm-radius rod from the TCP to 2.4 cm below it. It is placed from the
  gripper body (13.4 cm above the TCP in a down grip, measured on both arms against the demo's commanded TCP).
- The carton is placed once at reset; nothing is moved or edited during a trial.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every object is built from
plain boxes and cylinders and released under the task licence. `partkit.py` is reused from the accepted
`din-rail-mount` task in this repository (same licence as the repository); `hingekit.py` and `gripkit.py`
were written for this task family and are MIT.
