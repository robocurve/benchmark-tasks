# Flush an IV saline lock and clamp its line while the plunger is still pressed

A peripheral IV catheter is taped into a training forearm pad, with a short extension set running to a needleless
connector in a securement clip and an open pinch clamp on its tubing. A prefilled saline flush syringe (5 mL in a
10 mL barrel) is already screwed onto the connector. "Flush the IV saline lock with the syringe and clamp the line."

The skill is the positive-pressure flush. One hand pushes the plunger and keeps pushing. The other squeezes the pinch
clamp shut while the plunger is still under pressure. Only then does the first hand let go. With a
negative-displacement connector, the standards' order is flush, clamp, disconnect: if the thumb comes off the
plunger while the line is still open, the syringe's rubber stopper (squeezed by the push) springs back and draws
blood into the catheter tip, where it can clot and block the IV.

## Run

From the repo root, with Isaac Lab-Arena installed (the Robocurve Brev launchable, `source /data/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/flush-saline-lock-clamp-before-release/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Flush the IV saline lock with the syringe and clamp the line." \
      --scorer task_score --max-steps 1200
    python -B tasks/flush-saline-lock-clamp-before-release/test_task.py    # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 1200` for a Dream-1 trial. The scene is deterministic (no seeds, `object_jitter=0`).
The scripted demo is 21.9 s (438 steps); it reaches full score and holds it from step 370, so a 1,200-step trial
(1 minute) is ample. About 2 minutes of wall time per trial on an L40S.

Debug and checks:
- `FLUSH_DEBUG=1` prints the plunger (mL pushed), the clamp jaw, contacts and the reflux flag every step.
- `DEMO_LET_GO_FIRST=1` runs the negative control: the right arm lets go of the plunger before the clamp shuts. It
  scores 0 (reflux latched at step 293).

Files: `task.py` (scene, plunger law, release tracker, scorer, demo), `test_task.py`, and the kits it imports,
copied here: `yamkit.py` (props, sliding links, latches, gripper states), `mechkit.py` (the clamp's return spring)
and `guardkit.py` (gripper contact checks).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.
Everything lies along one line at x 30.

| Object | What it is | Where |
|---|---|---|
| Forearm pad | pink skin-tone silicone IV training pad, 12 x 24 x 3.2 cm, blue vein line | x 24 to 36, y 5 to 29, fixed |
| IV catheter | clear hub, blue wings, under a 5 x 3 cm clear film dressing | taped on the pad at y 8.6 |
| Extension set | clear PVC tubing, 3.6 mm outside | from the hub, along the pad and off its end to the connector |
| Needleless connector | clear body 1.2 cm across, 2.5 cm long, blue septum cap; negative-displacement type | y -3 to -0.5, 3.6 cm up |
| Securement clip | white plastic, 3.2 x 2.2 x 3 cm, under the connector | fixed to the table |
| Pinch clamp | white plastic; fixed jaw on the tubing's +x side, moving jaw on its -x side, each 3 mm thick, 1.4 cm along the tubing, 3 cm tall (2.6 to 5.6 cm up); ratchet teeth catch when squeezed shut | on the tubing at y 2.2; open at the start (moving jaw 4.5 mm back) |
| Flush syringe | 10 mL barrel (14.5 mm bore, 16.5 mm outside), clear with a printed 0.5 mL scale, prefilled with 5 mL of 0.9% saline; white plunger, 22 mm thumb press (3 mm thick) | screwed onto the connector, pointing -y; flange at y -11.5, thumb press back face at y -16.0, 3.6 cm up |

Start: clamp open, 5 mL in the syringe, plunger at the 5 mL mark. 3.0 cm of plunger travel empties it (6.05 mm per mL).

Mechanisms:
- The plunger is a slide along y with a law (`Plunger` in task.py) that stands for the fluid. It moves only as fast as
  saline can leave through the catheter: a damper of 250 N·s/m (about 1.5 N on the thumb press per 1 mL/s), which
  rises with the inverse square of the clamp's opening as the tubing is pinched. Its lower stop follows it in, so it
  cannot be drawn back. Once the clamp is shut the line is closed, and the plunger is locked where it was.
- The clamp's moving jaw is a slide along x with a light return spring (30 N/m, an implicit drive) and a latch: within
  0.8 mm of shut its ratchet catches and holds it shut for the rest of the trial.

## Scoring

The let-go rule is latched for the whole trial. Everything else is read from the final state, which must hold 1 s
(`SCORE_OVER = "end"`).

| Outcome | Score |
|---|---|
| A gripper off the plunger (thumb press or rod) for 0.15 s or more after pushing at least 0.1 mL in that contact, while the clamp is still open, at any time (latched) | 0 |
| A gripper still touching the thumb press, the rod or the clamp at the end (its own shape within 2 mm) | 0 |
| Plunger or clamp jaw still moving faster than 2 mm/s | 0 |
| Clamp not shut (its teeth not caught) | 0 |
| Less than 1.0 mL flushed | 0 |
| 1.0 mL or more, less than 3.0 mL | 0.25 |
| 3.0 mL or more, less than 4.8 mL | 0.5 |
| 4.8 mL or more (the whole syringe) | 1.0 |

Possible scores: 0, 0.25, 0.5, 1.0. Where the arms end up does not count.

Why each rule, and how a real trial checks it:
- Let go under pressure, latched: this is the one rule the procedure exists for. A real trial checks on the video that
  the clamp's teeth had caught before the pushing gripper left the thumb press, and looks for blood in the clear
  catheter hub. The 0.1 mL threshold ignores a hand that only touched the plunger without pushing it, and the
  0.15 s threshold ignores contact chatter during a push. Clamping afterwards does not undo it, since the blood is
  already in the catheter tip.
- Saline flushed: read from the syringe's printed scale (or measured as plunger travel, 6.05 mm per mL). 1.0 needs the
  whole 5 mL (the plunger within 1.2 mm of the barrel's end). 0.5 at 3.0 mL is over twice the line's internal volume
  (catheter plus a 10 cm extension set, about 1.2 mL), the usual minimum. 0.25 at 1.0 mL is the catheter and
  connector cleared once.
- Clamp shut at the end: the line left open is not clamped, as the instruction asks, and is open to reflux on the next
  bump.
- Hands off at the end: a hand still on the plunger or the clamp means the job is not finished. Real trial: the end of
  the video shows both grippers off.
- Doing nothing, pushing without clamping, and clamping first all score 0. Clamping first closes the line, so the
  plunger cannot move and nothing is flushed.

Construct notes: the push force is scaled to what one YAM arm pushes sideways comfortably (about 1.5 N per mL/s). A
hand pushes harder on a real 22 G catheter, but what is scored is the order of push, clamp and release, not the
pressure.

## Real-world build

- An IV training arm or venipuncture practice pad with a simulated blood bag at about 10 cm head, and a 20 or 22 G
  peripheral IV catheter placed and taped in it.
- A short (about 10 cm) extension set with a white pinch clamp and a negative-displacement needleless connector
  (a basic split-septum type), primed with saline, its connector held in an adhesive securement clip on the table.
- Prefilled 10 mL saline flush syringes with a 5 mL fill (sold as 5 mL-in-10 mL flush syringes).
- A small ruler beside the syringe, or its printed scale, to read the plunger's travel, and a camera on the clamp
  and the thumb press. Trials use saline only, and the catheter sits in a training pad, so they are safe.

## How it differs from nearby board tasks

- Scrub the hub before attaching the flush syringe: disinfection before the flush. Here the flush is already
  attached, and the skill is the timing of push, clamp and release.
- Draw a dose from a vial, purge a syringe of air, load a syringe pump: one-device syringe handling with no second
  hand timed against the first.
- Spike an IV bag and prime the drip chamber; load an IV set into a pump: setting up a line, with no clamping order.
- Bleed a brake caliper, shutting the bleeder before the pedal comes back up: a similar "close before release"
  timing, in hydraulics. Here a single flush with a ratcheting pinch clamp in clinical nursing, scored by the volume
  flushed rather than air removed.

## Sources

- The flush, clamp, disconnect sequence for negative-displacement needleless connectors: Infusion Therapy Standards
  of Practice (Gorski et al.), as tabulated in nursing skills references; CNSA, "Needleless Connectors & Patency".
- Employment: BLS OEWS May 2024, registered nurses (29-1141), about 3.3 million.

## License

MIT, as the repository. All meshes are built in code from boxes and cylinders; no third-party assets.

## Pushing by the rod

The plunger's white rod has a collider, so a hand can push the plunger by gripping the rod without touching the thumb press. The release tracker therefore counts a hand on the thumb press or on the rod (a 4 mm margin round the rod) as a hand on the plunger. `test_task.py` covers it: a rod push followed by a release with the clamp open scores 0 (even if the clamp is shut afterwards); a rod push with the clamp shut under pressure scores 1.0; resting a hand on the rod without pushing is not a release.
