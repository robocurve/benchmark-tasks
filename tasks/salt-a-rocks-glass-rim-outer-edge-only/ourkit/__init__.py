"""ourkit: our shared mechanism kit for yam_arena tasks. Copy the whole folder into a task unchanged
(``python3 .planning/kit/sync_kit.py tasks/<task>``); fix bugs in the kit, never in a task's copy.

- ``ourkit.scene``: :func:`mech`, :func:`slider`, :func:`hinge`, :func:`box`, :func:`span`; :class:`MechState`; hooks
  (:func:`on_step`, :func:`on_reset`, :func:`pre_reset`); runtime writes (:func:`set_targets`, :func:`set_gains`,
  :func:`set_limits`).
- ``ourkit.rules``: :class:`Rules` with :class:`Latch`, :class:`Detent`, :class:`Couple`, :class:`Weld`.
- ``ourkit.demo``: :class:`Plan`, :class:`Live` targets, ``grasp``/``press``/``push``/``ease``/``follow``/``corrected``
  on ``primitives.py``, cheat modes (``KIT_DEMO``), :func:`choose`, :func:`together`.
- ``ourkit.score``: :class:`Sticky`, :class:`Ever`, :class:`Settle`, :func:`ladder`, :func:`hands_clear`.
- ``ourkit.run_record``: the GPU trace. ``ourkit.testkit`` and ``holdcheck.py``: the 7-test template's checks.
- ``primitives.py`` and ``tiltgrip.py``: byte-identical copies of the pipeline's.
"""

import sys as _sys

from . import tiltgrip as _tiltgrip

_sys.modules.setdefault("tiltgrip", _tiltgrip)

VERSION = "1.0.0"

from . import demo, rules, run_record, scene, score  # noqa: E402
from .rules import Couple, Detent, Latch, Rules, Weld  # noqa: E402
from .scene import (LATEST, Joint, Look, Mech, MechState, box, hinge, hook, mech, mech_state, on_reset,  # noqa: E402
                    on_step, pin, pre_reset, recolor, set_gains, set_limits, set_targets, slider, span)
from .score import Ever, Settle, Sticky, hands_clear, ladder, within  # noqa: E402

__all__ = ["VERSION", "demo", "rules", "run_record", "scene", "score", "Couple", "Detent", "Latch", "Rules", "Weld",
           "LATEST", "Joint", "Look", "Mech", "MechState", "box", "hinge", "hook", "pin", "mech", "mech_state",
           "on_reset", "on_step", "pre_reset", "recolor", "set_gains", "set_limits", "set_targets", "slider", "span", "Ever", "Settle",
           "Sticky", "hands_clear", "ladder", "within"]
