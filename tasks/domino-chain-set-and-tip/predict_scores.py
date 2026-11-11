"""Predicted scores for demonstration and ablation modes of domino-chain-set-and-tip.

Honest numbers: all offline predictions before GPU verification.
"""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "gap_too_wide": 0.25,
    "tip_too_early": 0.25,
    "no_tip": 0.50,
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
