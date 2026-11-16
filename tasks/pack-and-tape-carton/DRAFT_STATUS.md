# Pack and seal a carton using an ordinary loose tape roll — incomplete feedback draft

This is an incomplete draft for early feedback on task relevance, novelty and physical fidelity. It is not presented as meeting the competition feasibility or media requirements. No completed model result or full-score demonstration is claimed. `task.json` and board media are deliberately absent until valid evidence exists; please keep this PR in draft.

Pack two objects, close four flaps and apply packing tape from a normal loose roll across the seam and down both ends, cut the strip and leave the seal retained after releasing the tools. No pre-laid strip or tool-path-triggered tape deposition is intended.

Implemented: carton/product geometry, product-placement controller, ordinary open-core roll and finite ribbon accounting, native roll/ribbon bench probes, a candidate contact adhesion law and numeric scoring. The roll is 48 mm wide, 100 mm outside diameter with a 76.2 mm core and an initially flush end.

Measured evidence: the original product-placement attempt reported native score 0; an offline rescore assigned the placement-only milestone 0.25. This is not a completed packing/sealing score. A native material peel coupon passed, but it used diagnostic loads and does not demonstrate robot peeling. No full tape application or sealed-box demonstration exists.

Unfinished: physical robot peeling/payout, flap closure, force-coupled adhesion validation, application/cutting/retention controller, genuine Dream attempt and complete media. Candidate adhesive constants are uncalibrated. The intended distinction from #1893 is manipulation and material payout from an ordinary roll rather than a prepared tape arrangement; feedback on that distinction is requested before further GPU investment.
