# Reinstall a dishwasher's filters

The robot reinstalls a Whirlpool dishwasher's two-part filter, reversing the removal steps on Whirlpool's help page.
The tub floor is in front of it with both filters out on the table. It slides the flat lower filter back under the two
locating tabs at the back of the floor, so its round opening lines up with the tub's. It lowers the cylindrical upper
filter assembly into that opening, where the cup's three foot lugs rest on the collar until it is turned clockwise to the
gaps and drops. Turned clockwise until it stops, about a quarter turn, the lugs go under the lips and past a spring detent pin, which
springs back up behind a lug. Then it lets go.

Files: `dishwasher_filter.py` (the task), `mechanism.py` (the scene builder), `task.json`, `test_dishwasher_filter.py`.

## Sources

| What | Value | Source |
|---|---|---|
| Removal | "Remove the filter by turning the Upper Filter Assembly ¼ turn counterclockwise and lifting it out. Grasp the Lower Filter in the circular opening, lift slightly, and pull forward to remove." | producthelp.whirlpool.com, cleaning the filters |
| Reinstall | "Be sure the Lower Filter is securely in place, and the Upper Filter Assembly is locked into place. If the Upper Filter Assembly turns freely, it is not locked into place." | same page |
| How often | "As a general rule, clean your filter once a month" | same page |
| Upper filter size | 3-1/4 in diameter x 5 in high | parts listing for W10872845 |

The reinstall order and the clockwise quarter turn are the removal reversed. Whirlpool publishes no part drawings. The
lower filter's size, the tabs, the bayonet collar and funnel, the lugs, the neck and the detent pin are ours, and so
are the masses and frictions. `task.json` `setup` lists every value. It is generated from the task file's constants.

## Sim results

yam_arena, scripted demo, one fixed start (`object_jitter=0`), 20 Hz, 2,400-step limit. The filmed demo scored 1.0 with success at step 779 (38.95 s) and took 236 s of wall time on a shared GPU. The one Dream-1 run scored 0 over 2,400 steps. Both runs hashed `dishwasher_filter.py`, `mechanism.py` and `task.json` before starting; the hashes match the files here (`task.json` as it was before its `scores` line was filled in and its `category` was corrected from "Societal relatability", a value the repo's checker rejects, to "Societal relevance"; neither field reaches the sim).

## Sim against real

- The real lock is a snap in the plastic. Here a sloped spring pin stands in the lugs' path: a lug presses it down as it
  passes and it springs back up behind the lug, so a locked cup does not turn freely. As a lug rides over it, it pushes 0.5 N,
  under the cup's weight.
- The cup's shoulder sits 1 mm over the lower filter's ring, so a locked cup keeps the lower filter under its tabs and
  the lower filter cannot be put in after the cup.
- The lower filter slides on two 1 mm runners. Sliding across the floor's many convex pieces snagged it in sim.
- The side rails' inner top edges have a 2.5 mm 45 deg bevel. A lower filter carried a little turned landed on a
  plain rail's top and jammed; the bevel slides it in square.
- The tub is only its floor, sump and tabs, with no walls or racks, and its height is set by the arms' reach.
- The filters' mesh is solid. The scorer reads poses and the hand hulls, not forces.

## Tests

`test_dishwasher_filter.py` checks the geometry with convex-hull gaps: the lower filter fits only under the tabs, the cup
drops only at the gaps, turns clockwise to the stop, cannot lift out when locked, and the pin is pressed only while a lug
passes, and that a locked cup keeps the lower filter in and out. It also checks the scoring ladder, the generated text and the demo's IK. Every demo waypoint must sit under the
right arm's measured reach.

## Third-party code

`mechanism.py` is adapted from the MIT-licensed `mechanism.py` of Robocurve PR #223, as used by earlier board tasks.
There are no meshes or other assets: every shape is a primitive or convex hull built in the task file.
