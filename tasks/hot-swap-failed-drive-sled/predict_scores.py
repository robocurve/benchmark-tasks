"""Predicted scores for all demo modes in hot-swap-failed-drive-sled."""

from __future__ import annotations

PREDICTIONS: dict[str, float] = {
    "demo": 1.00,
    "no_latch": 0.00,
    "wrong_bay": 0.00,
    "wrong_spare": 0.50,
    "release_early": 0.00,
    "settle": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, score in PREDICTIONS.items():
        print(f"  {mode:20} -> {score:.2f}")

