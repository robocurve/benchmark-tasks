# Public PR comparison

Recheck: 415 public PRs through #416 were available on 2026-11-05 KST. All newly
added titles and matching descriptions were reviewed; no explicit nested-tray
separation task was found. This is a search result, not a guarantee against near
duplicates or a finding that the benchmark's meaningful-novelty criterion is met.
The current loose-clearance, grasp-tab design can be solved with basic pick-and-place;
holding the outer tray is not mandatory. Treat it as a demo/recording example until
novelty against the board and other benchmarks is reviewed more fully.

Checked 308 open and closed PRs returned by GitHub CLI on 2026-11-05 KST (through #309).
Reviewed all titles and searched descriptions for nesting, denesting, trays and separation.
No matching tray-denesting task was found in that snapshot. The private task board was
not accessible without login; it should be checked again before public submission.

Closest tasks:

* [#28, extract a box and restack](https://github.com/robocurve/benchmark-tasks/pull/28):
  remove the bottom box from a three-box stack and restore the other two. Its unstacking
  and placement skills overlap with this example, although it has no enclosing tray rim.

* [#116, separate a snap-together case](https://github.com/robocurve/benchmark-tasks/pull/116):
  overcome a tongue/spring-plunger connection by pulling two halves apart. This task instead
  lifts an unlatched open tray out of a surrounding rim before translating it; no latch is modelled.
* [#206, transfer a loaded tray](https://github.com/robocurve/benchmark-tasks/pull/206):
  transfer the support of a tray containing tins. Here the containers themselves are nested,
  empty objects to separate; no cargo/support handoff is required.
* [#286, clear a bussing tray](https://github.com/robocurve/benchmark-tasks/pull/286):
  sort glasses and stack plates using a dish-pit rule. Here the tested operation is geometric
  extraction of one tray from another, not sorting contents.

These differences concern manipulation geometry and success conditions, not recolouring
or reordering existing objects. Generic unstacking is a familiar robotics skill; novelty
against every external benchmark has not been established.

## Preparation recheck, 2026-11-05

The later public snapshot contains 420 PRs through #421. Titles and descriptions were
searched again, and #28/#116 file lists and public intake comments were checked. No
explicit same denest-trays description was found; this does not establish acceptance
or meaningful novelty. Both similar tasks were still awaiting review in that snapshot.

The official LIBERO task
[pick the bowl on a ramekin and place it on a plate](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/bddl_files/libero_spatial/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate.bddl)
also starts with one vessel supported by another and relocates it. It is not an identical
scene or goal, but demonstrates why generic lift-and-place skill alone is a weak novelty
argument. Here the surrounding tray rim and two return bays differ, but loose clearance
and large grasp tabs reduce the need for specialized denesting or bimanual coordination.

The login-protected board and a comprehensive LIBERO/RoboCasa/SimplerEnv/BEHAVIOR/
DataCenterBench/RoboDojo comparison remain incomplete. Do not interpret this document
as a novelty clearance; recheck current claims before submission.
