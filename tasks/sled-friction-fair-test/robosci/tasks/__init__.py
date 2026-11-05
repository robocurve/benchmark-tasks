"""Task registry, auto-discovered: every module in this package that defines ``SPEC``
(task card), ``TASK`` (scene/monitor/scorer class) and ``POLICIES`` (oracle / guess /
noop generators) is a task. ``ORDER`` fixes the listing order (ability pairs); modules
not in it come after, alphabetically. A module that fails to import is skipped with a
warning so one task under construction cannot break the others."""

from __future__ import annotations

import importlib
import pkgutil
import warnings

ORDER = (
    "pointer_backlash", "level_reversal",      # instrument calibration (new)
    "roll_race", "heat_rods",                  # comparative measurement
    "blown_fuse", "two_capsules",              # efficient experiment design
    "conductors", "sink_float",                # property testing
    "switch_lamp_map", "lever_box",            # causal discovery
    "faulty_circuit", "gear_train",            # fault diagnosis
    "friction_fair_test", "shake_table",       # controlled experiments
    "ramp_shot", "hooke_spring",               # quantitative prediction
    "hidden_weight_bar", "sealed_box_ball",    # hidden structure
    "biased_spinner", "warm_cup",              # noise and replication
)

_names = sorted((m.name for m in pkgutil.iter_modules(__path__) if not m.name.startswith("_")),
                key=lambda n: (ORDER.index(n) if n in ORDER else len(ORDER), n))
_mods = []
for _n in _names:
    try:
        _m = importlib.import_module(f"{__name__}.{_n}")
    except Exception as e:  # noqa: BLE001
        warnings.warn(f"robosci task module {_n!r} skipped: {e!r}", stacklevel=1)
        continue
    if all(hasattr(_m, a) for a in ("SPEC", "TASK", "POLICIES")):
        _mods.append(_m)

TASKS = {m.SPEC.key: m.TASK for m in _mods}
POLICIES = {m.SPEC.key: m.POLICIES for m in _mods}
SPECS = tuple(m.SPEC for m in _mods)
SPEC_BY_KEY = {s.key: s for s in SPECS}
