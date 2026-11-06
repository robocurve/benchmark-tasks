"""Predicted scores for all demo modes in lockout-tagout-before-service."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "cover_first": 0.00,
    "no_pin": 0.25,
    "no_test": 0.50,
    "no_hold": 0.75,
    "order_swapped": 0.00,
    "settle": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")

