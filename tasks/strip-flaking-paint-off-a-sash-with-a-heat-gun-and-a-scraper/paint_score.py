"""The scoring rules, kept free of the simulator so the tests can run them on their own.

Each patch of old paint is a row of flakes. A flake is off once it has moved from where it was stuck (``moved`` m, or
``tilted`` degrees): the only way it moves is a blade or another hard edge pushing it hard enough, while the patch is
hot, to break its bond. A patch is stripped once at least ``frac`` of its flakes are off, which is what a photograph of
the patch shows: the share of its area that is bare wood.
"""

OFF_MOVE = 0.0015  # m: a flake still stuck to the wood cannot move this far
OFF_TILT = 8.0  # degrees
STRIPPED_FRAC = 0.8


def flake_off(moved: float, tilted: float) -> bool:
    return moved > OFF_MOVE or tilted > OFF_TILT


def removed(off_row) -> float:
    """The share of one patch's paint taken off (its flakes are equal in area)."""
    return sum(1 for f in off_row if f) / len(off_row)


def stripped(off_row, frac: float = STRIPPED_FRAC) -> bool:
    return removed(off_row) >= frac - 1e-9


def tier(off, cracked: bool, gun_home: bool, scraper_home: bool, arms_home: bool) -> float:
    """0.0 once the glass has cracked. Otherwise 1.0 with every patch stripped, the heat gun back on its stand, the
    scraper back on its rest and both arms home; 0.5 with every patch stripped; 0.25 with at least one; else 0.0."""
    if cracked:
        return 0.0
    n = sum(1 for row in off if stripped(row))
    if n == len(off):
        return 1.0 if (gun_home and scraper_home and arms_home) else 0.5
    return 0.25 if n >= 1 else 0.0
