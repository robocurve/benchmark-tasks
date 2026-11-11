"""Predicted score outputs across all operating modes for clear-lane-then-push-block-to-dock."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "push_first": 0.00,
    "hit_post": 0.25,
    "stop_halfway": 0.50,
    "settle": 0.00,
    "dream": 0.00,
}


def predict(mode: str) -> float:
    return PREDICTIONS.get(mode, 0.0)


if __name__ == "__main__":
    import sys

    m = sys.argv[1] if len(sys.argv) > 1 else "demo"
    print(f"Predicted score for mode '{m}': {predict(m):.2f}")

