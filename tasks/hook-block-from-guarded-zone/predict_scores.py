"""Predicted scores for demonstration and ablation modes of hook-block-from-guarded-zone.

Honest numbers: all offline predictions before GPU verification.
"""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "reach_over": 0.00,
    "drop_rod_in_zone": 0.25,
    "leave_block_outside": 0.75,
    "settle": 0.00,
    "dream": 0.00,
}


def predict(mode: str) -> float:
    return PREDICTIONS[mode]


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"{mode:20s}: {score:.2f}")
