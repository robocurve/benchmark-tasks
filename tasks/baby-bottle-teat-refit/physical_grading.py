"""Record a collar-only lift and, for a complete bottle, the 10 s leak inspection."""
import argparse
import json


def grade(retention, seams, inspection, setup="valid", teat_fit="unclear"):
    """A clearly failed full assembly can earn half credit for a proven retained pair."""
    if setup == "invalid":
        return {"score": None, "reason": "Restore dry equipment, parts and the 20 mL prefill; repeat."}
    if teat_fit == "failed":
        return {"score": 0, "reason": "Parts remained held, or the teat was loose, folded or separated."}
    if teat_fit == "retained" and retention == "held" and inspection == "complete" and seams in ("dry", "outlet-only"):
        return {"score": 1, "reason": "Released bottle and teat remained attached and both interfaces passed."}
    full_failed = retention in ("failed", "cross-threaded") or seams in ("bottle-collar", "teat-base", "both")
    if full_failed and teat_fit == "retained":
        return {"score": 0.5, "reason": "The teat stayed correctly captured in the lifted collar; full assembly failed."}
    return {"score": None, "reason": "The observation does not distinguish the possible scores; repeat from reset."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--teat-fit', choices=('retained', 'failed', 'unclear'), default='unclear',
                        help='After release, observe the whole rim and lift only the collar. See README for the retained-pair test.')
    parser.add_argument('--retention', required=True, choices=('held', 'failed', 'cross-threaded', 'unclear'),
                        help='Does the bottle follow and remain attached? Record visible thread misalignment as cross-threaded. A bottle that fails to follow or separates during the lift fails.')
    parser.add_argument('--seams', required=True, choices=('dry', 'outlet-only', 'bottle-collar', 'teat-base', 'both', 'unclear'))
    parser.add_argument('--inspection', required=True, choices=('complete', 'incomplete'),
                        help='Complete means both wetted interfaces observed during the 10 s inversion.')
    parser.add_argument('--setup', default='valid', choices=('valid', 'invalid'))
    print(json.dumps(grade(**vars(parser.parse_args()))))
