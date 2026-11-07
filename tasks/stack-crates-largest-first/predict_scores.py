"""Predicted scores for all demo modes in stack-crates-largest-first."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "wrong_order": 0.00,
    "offset": 0.50,
    "leave_small": 0.50,
    "knock": 0.75,
    "settle": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")

