"""Predicted scores for sliding-tile-sort-three-tiles.

Honest numbers: all offline predictions before GPU verification.
"""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "wrong_order": 0.50,
    "lift_tile": 0.00,
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

