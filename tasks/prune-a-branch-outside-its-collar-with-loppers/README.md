# Prune a branch off just outside its collar with two-handed bypass loppers

A short log lies across the table with one branch growing up out of it. At the branch's base is its collar, the
slight swelling where branch and trunk tissue meet, with its top edge where the swelling meets the branch (no painted mark). A pair of bypass
loppers lies open on the table in front of the arms. "Prune this branch off the log with the loppers."

Loppers need both hands. Each arm holds one handle, the tool is lifted and carried so its open jaws go round the
branch, and the handles are pulled together to cut. Where the cut lands is the skill. The pruning standard
(ANSI A300 Part 1) calls for a removal cut just outside the branch collar. A flush cut into the collar wounds the trunk
and lets decay in, and a cut further out leaves a stub that dies back. The instruction does not say where to cut.

## Run

From the repo root, with Isaac Lab-Arena installed (the Robocurve Brev launchable, `source /data/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/prune-a-branch-outside-its-collar-with-loppers/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Prune this branch off the log with the loppers." \
      --scorer task_score --max-steps 1200
    python -B tasks/prune-a-branch-outside-its-collar-with-loppers/test_task.py    # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 1200` for a Dream-1 trial. The scene is deterministic (no seeds, `object_jitter=0`).
The scripted demo is 20.6 s (412 steps); the cut is made at about step 240. About 2 minutes of wall time per trial on
an L40S.

Debug and checks:
- `LOP_DEBUG=1` prints the jaws' opening, the pivot, where the branch crosses the jaws' plane, the cut and the grippers
  every step.
- `DEMO_STUB_CM=<cm>` moves the demo's aim, for negative controls: `-3.5` cuts into the collar (scores 0), `0` leaves a
  stub (scores 0.25).

Files: `task.py` (scene, branch hinge law, cut tracker, scorer, demo), `test_task.py`, and the kit it imports, copied
here: `yamkit.py` (modeled props, the free-floating loppers mechanism, the branch hinge, gripper states).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Log | bark-covered hardwood log section, 9 cm across, 30 cm long | lying along y, centered x 52, y 0, fixed; top 9 cm up |
| Branch | 2 cm across, 20 cm tall above its collar, a small side twig near its top | straight up from the log's top at x 52, y 0 |
| Branch collar | bark-covered swelling 3.4 cm across, 1.2 cm tall, unmarked | at the branch's base; its top edge 10.2 cm above the table |
| Bypass loppers | 45 cm (18 in), 0.54 kg; dark hook and polished curved blade (8 cm jaws, 6 mm thick) on a pivot bolt; aluminum shafts with green rubber grips 12 cm long, oval 2.8 x 2.0 cm, centered 30 cm from the pivot; each handle bends 14 degrees off its jaw's line (grips 14.5 cm apart with the jaws shut) | loose, lying flat, jaws open 35 degrees toward +x, pivot at x 38, y 0.4; hook handle toward the left arm, blade handle toward the right |

Mechanisms:
- The loppers are one free-floating articulation: arm A (hook, left handle) is the base, and arm B (blade, right
  handle) turns about the pivot's axis. The joint stops 3 degrees past shut (the handles' bumper) and 42 degrees open.
  The pivot has a little friction (0.05 N·m).
- The branch above the collar is a link on a hinge at the collar's top edge. It is held upright (a stiff drive) until it
  is cut, then released, and it falls over the log away from the robot. The branch has no collider in the jaws' path,
  so the jaws can close through it. The cut is read geometrically (below).

## Scoring

The first cut counts. It is made when the jaws close (below 4 degrees open) with the branch in the throat: 1.5 to
7.5 cm from the pivot, between the hook and the blade, with the jaws' plane within 30 degrees of square to the branch.
The cut height is where the jaws' plane crosses the branch's axis at that moment. The score is the stub left above the
collar's top edge.

| Outcome | Score |
|---|---|
| No cut (the jaws never closed on the branch in the throat, square to it) | 0 |
| Cut below the collar's top edge (into the collar: a flush cut) | 0 |
| Stub 0 to 1.5 cm above the collar's top edge | 1.0 |
| Stub over 1.5 cm, up to 3 cm | 0.5 |
| Stub over 3 cm | 0.25 |

Possible scores: 0, 0.25, 0.5, 1.0. Where the loppers and the arms end up does not count.

Why each rule, and how a real trial checks it:
- Just outside the collar (0 to 1.5 cm): the removal cut the standard asks for. It keeps the collar, which seals the
  wound, and leaves no stub. Real trial: a ruler from the collar's top edge to the stub's cut face.
- Into the collar (below the collar's top edge): a flush cut, which wounds the trunk's defenses. It scores 0 however neat it is.
- Short stub (1.5 to 3 cm) 0.5, longer stub 0.25: the branch is off, but the stub dies back and invites decay. The
  further out, the worse.
- The first cut counts: trimming a high stub back to the collar afterwards is the three-cut method for heavy limbs,
  not what a 2 cm branch needs. Here it would also let a policy hack at the branch until it lands.
- The throat (1.5 to 7.5 cm from the pivot) and 30 degrees of square: bypass loppers cut in the throat, between the
  hook and the blade. At the tips or at a steep angle the blade skids off a live branch. Real trial: the video.

## Real-world build

- 18-inch (45 cm) bypass loppers with rubber grips (any common garden brand).
- A hardwood log section about 9 cm across and 30 cm long, fixed to a board, with a 2 cm socket drilled in its top.
- Branches: 2 cm green hardwood sticks (willow or hazel cuttings, or dowel wrapped in bark-textured tape) with a
  turned beech collar ring glued at the base. Each trial uses one 20 mm beech dowel (kiln-dried, soaked 24 h, used within 2 h).
- Trials cut a 2 cm green stick with hand loppers on a fixed log, so they are safe. Keep the blade's sheath on between
  trials.

## How it differs from nearby board tasks

- Trim a hedge with hedge shears, sparing the flowers: a two-handed cutting tool used across a surface, scored on what
  is left uncut. Here a single cut is placed precisely on one branch relative to an anatomical landmark.
- Cut a seized padlock off with bolt cutters: both handles closed on a target, with no "where along it" standard to
  meet.
- Cut a taut cord; cut a cable tie off a bundle: one-handed cutters, scored on not damaging the neighbors.

## Sources

- ANSI A300 (Part 1) pruning standard, as summarized by extension services and arborist bodies: branch removal cuts
  just outside the branch collar; flush cuts are not acceptable; do not leave stubs.
- Employment: BLS OEWS May 2024, landscaping and groundskeeping workers, about 943,000.

## License

MIT, as the repository. All meshes are built in code; no third-party assets.

## Cutting resistance, bounded cuts and the branch stock

- The blade arm's joint carries a law (`CutResist`): while the jaws bite an uncut branch in the throat (from 26 degrees open down to the 4 degree close, the jaws' plane within 30 degrees of square and crossing the branch) a constant torque of 1.2 N m (4 N at the 30 cm grip) pushes them open, so closing needs a steady force throughout the cut. It is scaled to what one YAM arm can close (a real cut of a 2 cm hardwood branch needs about thirty times that). It stops once the cut is made.
- The jaws' plane must cross the branch itself: closing above its tip is not a cut (offline test).
- The collar is unmarked. Real stock: a 20 mm beech dowel, kiln-dried then soaked 24 h, used within 2 h, in a turned beech collar ring; measure the cutting force at the grip with a luggage scale for each batch.
