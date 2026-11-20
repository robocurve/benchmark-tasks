"""The scorer's table on hand-built records, with no simulator: python check_score.py"""
import importlib.util
import pathlib
import sys
import types

# task.py imports the simulator's modules at import time; the scoring table (_tier, score) needs none of them
for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
class _Any:
    """Stands in for any mesh or part the scene builds at import."""
    def __getattr__(self, name):
        return self
    def __call__(self, *a, **kw):
        return self
    def __iter__(self):
        return iter(())
for fn in ("Mesh", "box", "cylinder", "sphere", "lathe", "prism", "join", "tube", "col_box", "col_cylinder", "col_hull", "slider", "hinge", "mechanism", "free",
           "fixed", "on_step", "on_reset"):
    setattr(mk, fn, _Any())


def _quat_matrix(q):
    import numpy as np
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


mk.quat_matrix = _quat_matrix
mk.ENV_PATH = "/World/envs/env_0"
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)

base = {"units": 12, "shield_in": True, "shot_ok": True, "wasted": False, "touched": False, "dropped": False, "in_tray": True, "clear": True}
cases = [
    ({}, 1.0),
    ({"shot_ok": False}, 0.2),  # no safety shot: the dose counts for nothing
    ({"units": 13}, 0.6),  # dialled one over and left there
    ({"units": 12, "in_tray": False}, 0.6),  # the dose set, the pen not put back
    ({"clear": False}, 0.6),  # still held
    ({"shield_in": False}, 0.8),  # the shield not in its tray
    ({"wasted": True}, 0.2),  # a push with the needle below horizontal
    ({"touched": True}, 0.5),  # the needle touched something
    ({"dropped": True}, 0.0),
    ({"shield_in": False, "shot_ok": False}, 0.0),
]
bad = 0
for change, want in cases:
    T.PEN.last = {**base, **change}
    T.PEN.full = T._tier(T.PEN.last) == 1.0  # held 2 s wherever every part is met
    got = T.score(None)
    ok = abs(got - want) < 1e-9 and T.success(None) == (got == 1.0)
    bad += not ok
    print(("ok  " if ok else "FAIL"), change, got, "want", want)
T.PEN.last, T.PEN.full = dict(base), False
unheld = T.score(None)  # a run ending inside the 2 s hold
print(("ok  " if unheld == 0.6 else "FAIL"), "every part met but not yet held 2 s at the end", unheld, "want", 0.6)
bad += unheld != 0.6 or T.success(None)
print("all pass" if not bad else f"{bad} failed")
