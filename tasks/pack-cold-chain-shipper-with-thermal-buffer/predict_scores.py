"""Predicted scores across demonstration and ablation modes for pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

PREDICTIONS = {
    "demo": 1.00,
    "no_buffer_violation": 0.25,
    "brick_only": 0.25,
    "buffer_without_carton": 0.50,
    "settle": 0.00,
    "dream": 0.00,
}


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


def predict(mode: str) -> float:
    return PREDICTIONS[mode]


def main():
    print("Mode Predictions for pack-cold-chain-shipper-with-thermal-buffer:")
    for mode, expected in PREDICTIONS.items():
        print(f"  {mode:<24}: {expected:.2f}")


if __name__ == "__main__":
    main()
