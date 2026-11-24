"""Validate the treatment, matching controls, contact channels and observed scores."""

import json
import sys
from pathlib import Path

folder = Path(sys.argv[1])
out = []
for mode, expected in [("reference", 1.0), ("fixed_heel_on_foam", 0.0), ("idle", 0.0)]:
    a, b = [
        json.loads((folder / condition / (mode + "-contact.json")).read_text())
        for condition in ("stock", "filter")
    ]
    assert a["actions_sha256"] == b["actions_sha256"], mode
    assert a["sources_sha256"] == b["sources_sha256"], mode
    assert a["fixture_sha256"] == b["fixture_sha256"], mode
    assert a["body_targets"] == [] and len(b["body_targets"]) == 18, mode
    assert b["robot_contact_steps"]["sole"] == 0, mode
    for item in [a, b]:
        assert item["physics_steps"] == 3360 and item["score"] == expected, mode
        assert item["sole_colliders"] == ["/Bedside/cushion/sole"], mode
    if mode == "idle":
        assert all(v == 0 for c in (a, b) for v in c["robot_contact_steps"].values())
        assert max(abs(c["cushion_displacement_x_mm"]) for c in (a, b)) < 1
    else:
        assert a["robot_contact_steps"]["sole"] > 0
        assert all(c["robot_contact_steps"]["foam"] > 0 for c in (a, b))
        assert min(c["cushion_displacement_x_mm"] for c in (a, b)) > 250
    out.extend([a, b])
(folder / "comparison.json").write_text(json.dumps(out, indent=2) + "\n")
print(
    "Passed six fresh standalone comparisons and all three paired action-hash checks."
)
