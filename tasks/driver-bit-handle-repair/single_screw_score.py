"""Score tiers for one installed bit and one powered drawer-knob screw.

The caller supplies facts measured by the physical task. This function does
not infer installation, tightening or load-bearing from intended actions.
"""


def score_from_evidence(*, assembly_pass: bool, load_pass: bool, return_pass: bool) -> float:
    if not assembly_pass:
        return 0.0
    if not load_pass:
        return 0.2
    if not return_pass:
        return 0.8
    return 1.0
