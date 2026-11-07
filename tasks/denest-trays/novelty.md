# Comparison with existing tasks

On 2026-11-05, the authenticated official task board returned 500 entries. Titles,
instructions, keywords and setup text were checked for nesting, denesting, trays and
separation. No explicit task with the same nested-tray separation and two-bay placement
goal was found. Earlier public checks covered 420 PRs through #421.

## Closest board entries

* `TK-C5ZD2T`, [extract a box and restack](https://github.com/robocurve/benchmark-tasks/pull/28):
  remove the bottom box from a three-box stack, then restore the other two. Both involve
  lifting and placement; this task instead clears an enclosing tray rim and parks both
  containers separately.
* `TK-H8GC97`, [separate a snap-together case](https://github.com/robocurve/benchmark-tasks/pull/116):
  overcome a connection by pulling two halves apart. These trays have no latch: the inner
  tray must clear the outer rim before lateral transport.
* `TK-PRTNGA`, [transfer a loaded tray](https://github.com/robocurve/benchmark-tasks/pull/206):
  transfer support of a tray containing tins. Here the containers themselves are nested
  and empty; there is no cargo or support handoff.
* `TK-SG3296`, [clear a bussing tray](https://github.com/robocurve/benchmark-tasks/pull/286):
  sort glasses and stack plates. This task separates two containers and puts each in its bay.

All four were pending in the authenticated snapshot. These comparisons describe the
objective and contact geometry; they do not establish that reviewers will find the
skill meaningfully different.

## Benchmark overlap and limitations

The official LIBERO task
[pick the bowl on a ramekin and place it on a plate](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/bddl_files/libero_spatial/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate.bddl)
also starts with one vessel supported by another and relocates it. Generic lift-and-place
skill overlaps. The surrounding rim and separate return bays differ here, but loose
clearance and large grasp tabs reduce the need for specialized denesting or two-arm
coordination. This scene does not simulate suction, elastic jamming or snap-fit retention.
The scripted demo uses both arms; the scorer accepts any valid separation strategy.
