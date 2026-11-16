# Service a crowded 4×2 Ethernet panel — incomplete feedback draft

This is an incomplete draft for early feedback on task relevance, novelty and physical fidelity. It is not presented as meeting the competition feasibility or media requirements. No completed model result or full-score demonstration is claimed. `task.json` and board media are deliberately absent until valid evidence exists; please keep this PR in draft.

Press the ordinary spring tab of the marked plug, withdraw it, transfer it to the marked empty port, insert it until the latch re-engages, and release both hands while keeping six neighboring plugs connected.

Implemented: native open sockets, spring-return clip/hook retention, jointed flexible cable chains, numeric state-based scoring, and a controller covering the full cycle. The intended distinction from #2141 is constrained access among six connected cables plus transfer, insertion and released retention; novelty remains a question for review.

Measured evidence: isolated prepress tracking passed. The last completed clip attempt (031) reached the contact stroke but stopped at 37.2 s on a 0.1872 mm non-target housing clearance; clip travel stayed below the 1.8 mm release threshold. No successful unplug or target retention is demonstrated. Candidate 27f296a shifts the contact point rearward 0.5 mm; its 121 CPU checks passed. A new native checkpoint is in progress and is not evidence of success. The model/clock capture helpers have CPU checks only and have not been integrated natively.

Unfinished: physical release and the complete cycle; fresh feedback-free replay; real Dream attempt; full demonstration/media. Connector dimensions and cable mechanics are approximate; electrical continuity and cable damage are not simulated.
