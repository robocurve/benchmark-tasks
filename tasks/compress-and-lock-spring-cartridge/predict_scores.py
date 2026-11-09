"""Predicted scores for compress-and-lock-spring-cartridge across all ablation modes."""

PREDICTIONS = {
    "demo": 1.00,
    "order_bar_first_arm": 1.00,
    "one_arm": 0.25,
    "release_early": 0.25,
    "bar_first": 0.00,
    "no_release": 0.50,
    "settle": 0.00,
    "dream": 0.00,
}


def predict(mode: str) -> float:
    return PREDICTIONS[mode]


def predict_all() -> dict[str, float]:
    return dict(PREDICTIONS)


if __name__ == "__main__":
    for mode, s in PREDICTIONS.items():
        print(f"{mode:22} {s:.2f}")
