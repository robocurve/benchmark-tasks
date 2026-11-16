# Cut bottom tape, unfold all flaps and flatten a carton — incomplete feedback draft

This is an incomplete draft for early feedback on task relevance, novelty and physical fidelity. It is not presented as meeting the competition feasibility or media requirements. No completed model result or full-score demonstration is claimed. `task.json` and board media are deliberately absent until valid evidence exists; please keep this PR in draft.

Starting with an empty bottom-up carton, cut its bottom seam and both taped end folds with the guarded cutter, park the tool, physically unfold all four bottom flaps, then collapse the intact wall tube flat along its creases. There are no grasp handles or pre-cut tape.

Implemented: twelve articulated board panels, native crease/flap joints, tape-cut geometry, numeric scoring, cutter manipulation and incomplete flap controllers. Historical fresh sealed cut011 completed all 158 cut cells and parked the released cutter, reporting 0.25 for the cutting milestone. It did not open or flatten the box. Later opening probes failed; they are not full-task demonstrations.

The current source fixes an angle alias that could misclassify inward motion as a full opening: it tracks raw native relative poses continuously at 240 Hz and fails closed on missing samples. The revised API binding passed 87 CPU checks and an independently audited 180-control-row native capture. Historical wrapped-angle logs are not evidence of opening under the corrected scorer.

Unfinished: reliable flap acquisition/opening and complete collapse, fresh full-score replay, genuine Dream attempt and full media. The intended distinction from #1858 is starting with real bottom tape and requiring cutting plus all four bottom flaps to open before intact collapse. The material model uses rigid panels and crease joints rather than continuum corrugated-board deformation.
