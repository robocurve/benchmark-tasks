> **Revision note.** Everything below was measured on the earlier code from pull request #2426. That code used the joint-angle and joint-gap integrity limits this revision replaces (see README, **Real-trial ring checks**). None of it has been rerun on this revision, and the hashes below do not match the revised `task.py`, `geometry.py` or `test_scoring.py`.

The recorded full demonstration and three physical controls used identical runtime code, assets and task.json metadata. The controls used the existing no-record/snapshots mode, preserving physical actions, dynamics, numeric traces and native engagement/final images while skipping continuous video. All ran on physical GPU 0. Only the task.json score-report string was filled after those measurements. The final runtime source hashes match all four runs. README explanations and commands were updated separately; each control's source ZIP copies the full trial's frozen runtime snapshot, including its earlier README. The documentation changes are recorded in the package metadata.

| Trial | Final score | Actions | Wall minutes | Result |
| --- | ---: | ---: | ---: | --- |
| Recorded complete demonstration | 1.0 | 2302 | 78.51 | Two distinct turns, released key, closed spring, stopped crank and clear grippers held for 2 seconds |
| Stop after partial route | 0.5 | 1155 | 34.18 | 1 through-hole strand; key remains held, no full success |
| Direct placement after opening | 0 | 537 | 12.74 | No full success; integrity at finish: False |
| Offset key hole | 0 | 955 | 21.38 | No full success; integrity at finish: True |
| Native resets after success | 0 throughout | 2 × 200 | Included above | Both returned to unthreaded, closed initial state |
| Numerical scorer cases | 31 passed | — | — | Includes exact polygon faces, unchanged 49/51 µm contact boundary cases, finite plate thickness, interior wire bulges, tangencies, distinct turns and incomplete tips |

The full demonstration took 115.10 simulated seconds. The recorded trial plus reset checks took 4710.5 wall seconds on an RTX 5090. Control is 20 Hz; native physics is 1,000 Hz with 50 substeps per action, including standard Inspect Robots evaluations. The run limit is 2,400 actions.

At completion the material crossings were 359.552 degrees apart along the continuous spiral. Clearance from the nearest free end was 36.501 mm. Key speed was 1.0468 mm/s; key spin 0.03042 rad/s; crank speed -0.00104 rad/s. Conservative gripper/finger surface clearance was 220.541 mm from the key and 222.770 mm from the ring. Maximum flex during the full trial was 10.870 degrees; maximum joint endpoint gap was 1.731e-07 m.

The trace field `ring_vertical_span_m` is the rotor-local axial centreline span used for spring closure, despite its legacy name.

The first through-thickness strand capture occurred at action 577; two distinct turns were first measured at 1791. The raw partial-score channel remains 0.75 during the final hold. The upstream YAM TaskScore returns 1.0 when native termination_reason is success after the 2-second geometric hold.

Every physical control first lifted the actual key at least 30 mm and opened the actual splitter at least 3 mm. Their raw actions share the full demonstration's pickup/opening prefix. The native close-up snapshots document those physical interactions; a control is not counted solely because its final score is low.

The direct-placement control is an excessive-deformation negative, not a clean contact-free miss. Its final native witness image shows the key head pressing the opened coil. It accepted no strand and stopped when measured flex reached 24.01288 degrees, exceeding the unchanged 24-degree integrity limit. The peak joint endpoint gap across the trial was 0.149 µm, and the final gap was 0.089 µm. The hole centre was still 8.62 mm forward of the eventual placement target when the runner stopped. This trial demonstrates rejection of the forced shortcut; it does not claim a benign placement or a released result.

The offset-hole control remained mechanically intact throughout. Native frame 560 shows the spring end meeting the outer edge of the key bow while the hole stays radially offset. Peak flex was 14.495 degrees. No strand was accepted at any action. After the gripper opened, the key fell to the table; its final hole-centre height was 0.750657 m. The ring closed passively, and the score stayed zero.

The controller uses fixed setup geometry and robot inverse kinematics only. Privileged object measurements are confined to scoring and evidence logging. All fixture springs are permanently enabled. Task progress never teleports, snaps or attaches objects, and never applies goal-directed object forces. The initially free key has no joint; the ring's permanent visible clamp is part of the disclosed workholder.

All three procedural USD assets rebuilt byte-for-byte from the supplied generator in a separate folder.

The complete evidence bundle contains robot Actions, every measured trace row, native camera recordings, result JSON, exact source hashes and the shared frozen runtime source snapshot. The setup image combines the initial overview, ring witness and key-detail renders from the full demonstration's reset. The H.264 demonstration is a synchronized four-camera composite with no invented imagery.

Earlier failed development runs and rejected CPU paths remain separate in the evidence directory. They include an unstable finer-link ring, failed grasps, inter-arm interference and fixture obstructions. In particular, demo08_handwheel ended at native score 0.5 after 2,313 actions despite visibly released retention: its second strand had about 0.05–0.06 mm numerical rim overlap, outside the unchanged 0.05 mm allowance. Its original source, video and score remain intact. The final source checks the actual 32 polygon walls and uses uniform 30 µm positive rest/80 µm contact offsets on all authored collision shapes; their combined 0.16 mm contact envelope is smaller than the 0.3 mm resting coil gap. No success, motion or integrity threshold was relaxed. Neither the earlier partial runs nor the separate zero-score crank-only diagnostic establish full task success.

Task SHA-256: `9cc334be1b365bcc006dd07e356efd18787336543f653d5032dcba5ce28fa9e1`

Ring asset SHA-256: `a6e4ddf15c50c887f20e4bd0bef261e7b88e608a196270dc2e2887fbb89901c9`

Dream-1 beta completed all 2,400 actions (120 simulated seconds) with native task_score 0.0, max_steps termination and zero errored trials. Wall time was 2189.8 seconds (36.50 minutes). The client recovered from 49 session-expiration messages and 3 hosted pool-full messages within this single trial. Runtime Python and USD hashes match the reviewed demonstration and physical controls. The model video contains all 2,401 frames from each of the three policy cameras, is H.264/yuv420p at 20 fps, and passes a complete decode check. Frames at 0, 40, 80 and 120 simulated seconds were inspected; no team identifiers were seen. DREAM_EVALUATION.json records the measured result and artifact hashes.

The signed-in board check at 2026-11-17 04:06 UTC covered all 587 current active/planned entries. Its category counts match the board summary, and all 165 held revision entries were nonfinal and within 48 hours. No direct match was identified. OVERLAP_REVIEW.json compares the accepted circlip and O-ring tasks and the held key-cabinet task. The earlier submission's media are not reused; this revision's `task.json` has no media ID until new runs are uploaded.
