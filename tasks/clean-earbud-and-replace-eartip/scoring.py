"""Pure scoring policy for the earbud-service task.

Keeping this module independent of YAM Arena makes the acceptance-critical state
machine testable without Isaac Sim.  Cleaning evidence is episode history;
placement, disposal, release, and rest are live end-state observations.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Evidence:
    old_tip_removed: bool = False
    old_tip_in_discard: bool = False
    mesh_marks_clean: int = 0
    mesh_marks_total: int = 8
    contact_marks_clean: int = 0
    contact_marks_total: int = 6
    new_tip_seated: bool = False
    tools_released: bool = False
    earbud_released: bool = False
    objects_resting: bool = False


def _enough(clean: int, total: int) -> bool:
    """At least 80%, rounded up; an empty inspection set never passes."""
    return total > 0 and clean * 5 >= total * 4


def score_evidence(e: Evidence) -> float:
    """Return the highest completed milestone in the fixed scoring ladder."""
    removed_and_discarded = e.old_tip_removed and e.old_tip_in_discard
    mesh_clean = _enough(e.mesh_marks_clean, e.mesh_marks_total)
    contacts_clean = _enough(e.contact_marks_clean, e.contact_marks_total)

    score = 0.15 if removed_and_discarded else 0.0
    if removed_and_discarded and mesh_clean:
        score = 0.40
    if removed_and_discarded and mesh_clean and contacts_clean:
        score = 0.60
    if removed_and_discarded and mesh_clean and contacts_clean and e.new_tip_seated:
        score = 0.85
        if e.tools_released and e.earbud_released and e.objects_resting:
            score = 1.0
    return score
