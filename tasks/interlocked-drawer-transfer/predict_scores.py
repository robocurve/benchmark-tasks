"""Predicted scores for all demo modes in interlocked-drawer-transfer."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "park": 1.00,
    "both_open": 0.25,
    "leave_open": 0.75,
    "wrong_drawer": 0.25,
    "block_dropped": 0.25,
    "settle": 0.00,
    "dream": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")
