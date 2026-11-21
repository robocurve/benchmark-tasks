"""The Inspect Robots scorer: the score ``judge_midi.judge`` gives the trial's hits (written as MIDI) and events,
taken from the last step (the adapter judges the hits when the end has been clean for 1 s, and at the step limit).
Pure Python."""

from dataclasses import dataclass

CONTROL_HZ = 20
KEYS = ("score", "why", "bpm", "rms_ms", "max_error_ms", "H", "G", "E", "wrong", "extra", "arm_contacts", "rests",
        "count_in", "accent", "clean_end", "end_detail", "pair_spread_max_ms", "hits", "notes")


def summary(result):
    """The judged result without its per-hit rows (for each step's info and the scorer trace)."""
    out = {k: result.get(k) for k in KEYS}
    if result.get("accent_detail"):
        out["accent_detail"] = result["accent_detail"]
    return out


@dataclass(frozen=True)
class DrumScorer:
    name: str = "drum_groove_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["drum_grade"]
        if grade.get("score") is None:
            return Score(value=0., explanation="The trial was not judged at its last step")
        return Score(value=grade["score"],
                     explanation=f"{grade['why']}: {grade['H']}/{grade['notes']} notes, {grade['G']}/8 pairs together, "
                                 f"{grade['E']} extra or wrong hits, fitted {grade['bpm']} bpm, RMS {grade['rms_ms']} "
                                 f"ms, accents {grade['accent']}, clean end {grade['clean_end']}")


def make_scorer():
    return DrumScorer()
