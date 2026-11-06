"""Predicted scores for all demo modes in spring-drawer-hold-and-retrieve."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "no_hold": 0.25,
    "pinch": 0.00,
    "leave_open": 0.75,
    "settle": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")

