# This file is adapted from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a spring
# plunger", tasks/spring-plunger-panel/mechanism.py), which is published under the MIT License. Only its hooks on
# yam_arena's embodiment are kept here (the mechanism articulation, its state and the camera recorder are dropped);
# changes: functions registered with ``on_step`` run with the environment before each step is scored, and the
# ``on_reset`` functions run after every reset. The original licence:
#
#   MIT License
#
#   Copyright (c) 2026 the authors of this task
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
#   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#   the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#   OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Hooks on yam_arena's embodiment for a task file that must see the arms, without modifying yam_arena.

- before each control step is scored, functions registered with :func:`on_step` run with the Isaac Lab environment
  (the technique of robocurve/benchmark-tasks tasks/analytical-balance/labkit.py ``on_step``: wrap the embodiment's
  ``_task_step``, which builds the object states and calls success() and score());
- after every ``YamArenaEmbodiment.reset``, functions registered with :func:`on_reset` run.

The embodiment module is patched when it is imported (or at once, if it already is). Nothing here imports Isaac Sim,
so a task file that uses it still loads before the simulator starts. The registries live in this one module, so every
copy of a task file loaded in a process (each registers its own functions) is fed.
"""

from __future__ import annotations

import importlib.abc
import sys

__all__ = ["on_reset", "on_step", "install"]

_RESET: list = []
_STEP: list = []


def on_reset(fn):
    """Register ``fn()`` to run at the end of every reset."""
    _RESET.append(fn)
    return fn


def on_step(fn):
    """Register ``fn(environment)`` to run after every control step of a task file's trial, before that step's
    object states are read and success() and score() are called."""
    _STEP.append(fn)
    return fn


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_arm_watch_patched", False):
        return
    reset, task_step = cls.reset, cls._task_step

    def patched_reset(self, scene, *, seed=None):
        observation = reset(self, scene, seed=seed)
        for fn in _RESET:
            fn()
        return observation

    def patched_task_step(self, *args, **kwargs):
        # WHY: step() calls self._task_step() only when a task file is loaded; it reads the object states and calls
        # success() and score() for this step, so a hook run here is always fresh for the step being scored
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *args, **kwargs)

    cls.reset, cls._task_step = patched_reset, patched_task_step
    cls._arm_watch_patched = True


_PATCHES = {"yam_arena.inspect_embodiment": _patch_embodiment}


class _PatchOnImport(importlib.abc.MetaPathFinder):
    """Finds the modules in ``_PATCHES`` with the other finders, and patches each right after it executes."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        if spec.loader is None:
            return None
        execute, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            execute(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    """Patch the yam_arena modules already imported, and the others when they are. Safe to call again."""
    for name, patch in _PATCHES.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _PatchOnImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _PatchOnImport())


install()
