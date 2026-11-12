"""Predicted benchmark scores across demonstration and ablation modes."""

from __future__ import annotations

PREDICTIONS = {
    "demo": {
        "predicted_score": 1.00,
        "reason": "Both incompatible chemical cleaner bottles safely segregated into separate storage cabinet bays, fully seated in recessed wells, arms retracted to standby, held 2.0 s.",
    },
    "same_bay_violation": {
        "predicted_score": 0.50,
        "reason": "Both bottles are carried to spots 110 mm apart between the wells. Measured: both are moved but neither seats in a well (they rest on the cabinet rim), so the score is 0.50 and the 120 mm latch is not triggered.",
    },
    "one_bottle_only": {
        "predicted_score": 0.25,
        "reason": "Only one bottle placed into a cabinet bay, the other remains untouched at staging.",
    },
    "settle": {
        "predicted_score": 0.00,
        "reason": "Both arms remain idle at home; neither bottle is moved or placed.",
    },
}


def main():
    print(f"{'Mode':<22} | {'Predicted Score':<15} | Reason")
    print("-" * 80)
    for mode, data in PREDICTIONS.items():
        print(f"{mode:<22} | {data['predicted_score']:<15.2f} | {data['reason']}")


if __name__ == "__main__":
    main()
