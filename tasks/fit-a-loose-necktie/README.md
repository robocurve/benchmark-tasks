# Thread and fit a loose necktie

Pass the free wide blade through the front loop of a pre-wrapped Simple knot, snug it, then use the narrow blade to slide the knot to the collar. Leave both blades neat and both hands clear. This revision adds the substantive threading step requested in the latest review; it is not a full sequence from an unwrapped tie.

The native demo scores **1.0 in 1,150 actions (57.5 seconds)**. Dream-1 completes its **1,800-action limit at 0.0**. Both traces independently replay exactly in the pinned Linux runtime. The videos show the same reset and model, including the unfinished threading step.

## Setup and reset

Coordinates are metres: X away from the arms, Y left, Z up; table height 0.7475. A rigid plastic neck, 110 mm diameter and 100 mm high, is centred at (0.400, 0, 1.250). Its lower collar reference is Z=1.200. Two 50 × 55 × 8 mm flaps and a black centre mark define the fit. The steel support has a 180 × 160 × 12 mm base, 25 × 25 × 455 mm post and 210 × 25 × 25 mm crosspiece. `task.py` fixes their poses.

Use a reversible woven polyester tie, 1.26 m long, 70 mm wide at the wide tip, 35 mm at the narrow end and about 1.2 mm thick. Prepare the **Simple/Oriental knot only up to the front-loop threading step**:

1. Facing the collar, put the reverse face outward, wide end W to the left and narrow end N to the right.
2. Bring W behind N from left to right, then across N's front from right to left. This creates the loose front band.
3. Bring W upward through the neck opening from below. **Stop here:** let W drape outside the front band. Do not tuck it down behind that band. N hangs behind the loose wrap.

The robot must lift W's tip, put it down through the front band's opening, draw it through, snug the wrap, then regrasp N for fitting. Pulling W outside the loop cannot form the knot. The order follows [Ties.com's Simple-knot instructions](https://www.ties.com/how-to-tie-a-tie/simple); the last threading step belongs to the robot trial.

Set the loose wrap centre to (0.319, 0.025, 1.090), 74 mm wide and 57 mm high. Its top is 81.5 mm below the collar. The free W section is about 330 mm long from its exit above the front band at (0.329, 0.025, 1.136); it drapes over the outer front edge and hangs loose. N emerges behind the wrap at (0.327, 0.025, 1.0615), with about 85.5 mm exposed. The model settles W under gravity before frame zero. Match the native setup views within 5 mm using a ruler. The diagram distinguishes the prepared wraps from the unfinished passage.

Both hands start in the standard open YAM ready pose. Remove the preparer's hands and all temporary aids before starting. No clips, tape, pins, keeper loop or hidden support hold the knot or ends. If the tie will not retain this loose pre-wrap, use another reversible tie of the specified dimensions. Restore the same layout each trial and replace damaged fabric. There is no person.

## Run and judge

Use the pinned Python/Isaac/YAM runtime in `ENVIRONMENT.json`:

```sh
python tasks/fit-a-loose-necktie/run.py --mode demo --out /tmp/tie-demo
python tasks/fit-a-loose-necktie/run.py --mode dream --out /tmp/tie-dream
python /tmp/tie-demo/source/rescore.py /tmp/tie-demo
python /tmp/tie-dream/source/rescore.py /tmp/tie-dream
```

Both policies have the same scene, material and 1,800-control limit (90 seconds at 20 Hz). On an L40S, the recorded demo took 14.1 minutes and Dream took 20.5 minutes of wall time, including startup and concurrent validation work. Dream used 221 requests in one session. The scripted plan sends bounded robot joint targets. All camera frames, measured hand poses, material measurements, reaction loads, events and source hashes are recorded. CPU rescoring reconstructs the reduced material model from the measured hands; it is not a new native physics run.

Judge after both hands release for 5 seconds. Award 0.5 if the intact threaded knot is at most 70 mm wide and high, with its top within 60 mm below the collar edge. Full credit also requires its top within 20 mm, its centre within 20 mm of the collar mark, the wide blade flat within 30 degrees of untwisted, its tip 350–440 mm below the collar edge, and the narrow blade behind it, within 15 mm laterally and with its tip at least 10 mm higher. Otherwise 0. Pulled-out ends, damaged fabric or added support score 0. Use front/rear photos, video and a ruler.

No credit is possible until the wide end has passed down through the actual front loop and the knot remains threaded. In the reduced model, the free tip must cross both sleeve openings and draw at least 230 mm below the lower opening; merely crossing in a front image or pulling beside the band does not count. The physical judge checks the knot's under/over structure in front and side close views, then uses the ruler and five-second released hold in `JUDGING.md`.

## Approximation and failure

The 33-node wide-blade centreline sags under gravity and uses distance constraints and an open sleeve collision approximation. Finite pad-volume capture moves only the contacted material point. The loop is the already-prepared front wrap, not an added rigid fixture. Detailed fold thickness, self-contact and arbitrary retying are omitted. A numerical constraint-overload guard stops the demonstration rather than allowing an overstretched ribbon to earn credit.

After actual downward passage and sufficient free-end feed, the model changes to the previously disclosed reduced sliding representation. Its blade and wrap lengths are set from the current contacted point to preserve that point and the total 1.26 m material budget. Wide-blade feed snugs the wrap; only a narrow-blade grasp permits upward fitting. Released tails drape approximately. The knot remains on the collar's front plane. These are task-model approximations, not calibrated cloth mechanics.

A regularized 3 N sliding resistance applies opposing loads to the native grippers during relative feed. Grips can slip; empty or dragged-off fingers cannot transport cloth. The model does not claim full knot tying, dressing a person or realistic textile damage forces. All geometry and diagrams are original procedural work; no third-party task meshes are used.
