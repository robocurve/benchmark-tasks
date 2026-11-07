"""Predicted scores for all demo modes in estop-before-clearing-jam."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "no_estop": 0.00,
    "start_early": 0.50,
    "skip_reset": 0.50,
    "leave_stopped": 0.50,
    "settle": 0.00,
    "dream": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")
