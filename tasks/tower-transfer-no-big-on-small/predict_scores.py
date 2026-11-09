"""Predicted scores for demonstration and ablation modes of tower-transfer-no-big-on-small.

Honest numbers: all offline predictions before GPU verification.
"""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "big_on_small": 0.00,
    "leave_one": 0.50,
    "wrong_target": 0.50,
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

