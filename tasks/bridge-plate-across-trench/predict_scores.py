"""Predicted scores for demonstration and ablation modes of bridge-plate-across-trench.

Honest numbers: all offline predictions before GPU verification.
"""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "no_bridge_push": 0.00,
    "bridge_short_overlap": 0.00,
    "skip_target": 0.50,
    "settle": 0.00,
    "dream": 0.00,
}


def predict(mode: str) -> float:
    return PREDICTIONS[mode]


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"{mode:22s}: {score:.2f}")
