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
over. They do it gently: pressing the lid or the flap onto the eggs cracks them, and a lifted or tipped carton spills
them. The scripted demo does the same with the two arms.

## Revisions

Revision 1 answers every point of the review of TK-MFPNK8:

1. **Construct validity:** "The score permits closing the carton after crushing its eggs or damaging the pulp,
   although either would defeat the real task." The score now keeps a damage record over the whole trial; once it is
   set, the trial scores 0 whatever the end state. It is set by any of the following:
   - more than 5 N pressed onto the eggs through the lid and flap while the lid is within 15 degrees of shut (in the
     simulation, the force carried by the lid's and the flap's hinges; a locked flap props the lid, so a hard press
     on a closed carton loads both);
   - the lid pressed more than 6 degrees past shut (onto the egg tops; the lid stops on them at 12 degrees);
   - the flap pressed more than 16 degrees inside upright (onto the front row; it stops on it at 18 degrees);
   - any finger within 5 mm of an egg's exposed top;
   - the carton lifted 3 mm or tipped 3 degrees.

   The scripted wrong attempts `crush`, `flap_press` and `egg_touch` each reach a properly locked end state and score
   0 (table below).
2. **Scoring:** "awards full credit without checking whether the eggs and carton remain intact." Full and half credit
   now both need the damage record to be clear, and the record is part of `score()` in the simulation. In a real
   trial the same conditions are judged from:
   - a kitchen scale under the carton, tared with the carton on it, whose display is in the overhead camera's view:
     a reading above 500 g at any time is a press onto the eggs;
   - all 12 eggs still seated in their cups;
   - no new dent, crease or tear in the lid top, flap, skirt or hinges, against a photo taken before the trial (a pulp
     lid pressed onto hard wooden eggs dents);
   - no fingertip touching an egg in the wrist-camera video.
3. **Clarity and reproducibility:** "No source or build method specifies a repeatable carton with the stated
   full-width flap, beads and spring behavior, and the eggs' real-trial type is unspecified." The carton is a blank
   natural-pulp 2 x 6 flat-top dozen carton with a front locking flap, about 30.5 x 10.2 x 7.6 cm and 54 g; for
   example Hatching Time 12-Egg Cartons (2x6), Natural Pulp, Flat Top Blank, SKU EC-FT2x6-25 (packs of 25). The flap's
   two moulded buttons and the lid's matching holes are modelled as the two beads.

   Setup and reset:
   1. Open the lid back to 100 degrees (past upright), fold it back and forth twice so it rests there, and let the
      flap hang out.
   2. A carton qualifies if, shut over an untucked flap and released, its lid springs back past 45 degrees within 2 s,
      and if, with the flap tucked and the lid closed over it, it stays shut.
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

- **Damage record (whole trial):** set at the first step at which any of these happens:
  - more than 5 N is carried by the lid's and flap's hinges together while the lid is within 15 degrees of shut
    (a real trial uses a kitchen scale under the carton: more than 500 g above tare);
  - the lid is more than 6 degrees past shut;
  - the flap is more than 16 degrees inside upright;
  - a finger is within 5 mm of an egg's exposed part;
  - the carton is lifted more than 3 mm or tipped more than 3 degrees.

  Front-row egg sides do not count while the flap stands between the finger and the egg. The record is cleared only
  when the scene is reset.
- **1.0**: no damage, and at the end:
  - the lid is within 5 degrees of shut (and no more than 6 degrees past it);
  - the flap is within 6 degrees of upright or further in (inside the skirt, where its bead holds the lid);
  - the carton is within 1 cm of where it started;
  - no finger is within 3 cm of the carton's closed outline;
  - the lid has been still (moving under 0.5 degrees) for the last 1 s.
- **0.5**: the same, but the carton was pushed more than 1 cm.
- **0** otherwise.

Without the flap inside, nothing holds the lid down, and the pulp hinge opens it again as soon as it is let go.

### Shortcut checks (scripted, same scene)

| Attempt | What happens | Score |
|---|---|---|
| `EGG_VARIANT=lid_only`: shut the lid with the flap left hanging out | the lid springs back open when released | 0 |
| `EGG_VARIANT=slow`: tuck the flap, let go, close the lid 2.5 s later | the flap has crept out; the lid springs back open | 0 |
| `EGG_VARIANT=crush`: lock it, but keep pressing the closed lid down onto the eggs | locked at the end, but the press loaded the lid and flap hinges with over 5 N | 0 |
| `EGG_VARIANT=flap_press`: fold the flap hard in against the eggs | locked at the end, but the flap went past 16 degrees | 0 |
| `EGG_VARIANT=egg_touch`: rest a finger on a front egg, then close it properly | locked at the end, but a finger touched an egg | 0 |
| Hold the lid shut at the end | fingers within 3 cm of the carton | 0 |
| `EGG_VARIANT=nothing` | the carton stays open | 0 |

## Results

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
  (`get_link_incoming_joint_force`). The scripted demo's own press stays under 5 N; lifting the flap with the lid
  still open loads the flap hinge with up to about 4.5 N, which is why the force only counts with the lid near shut.
- The fingers, for the egg-contact check, are a 1 cm-radius rod from the TCP to 2.4 cm below it. It is placed from the
  gripper body (13.4 cm above the TCP in a down grip, measured on both arms against the demo's commanded TCP).
- The carton is placed once at reset; nothing is moved or edited during a trial.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every object is built from
plain boxes and cylinders and released under the task licence. `partkit.py` is reused from the accepted
`din-rail-mount` task in this repository (same licence as the repository); `hingekit.py` and `gripkit.py`
were written for this task family and are MIT.
