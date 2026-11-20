"""gripkit: YAM grip settings for a yam_arena task file.

What it does:
- `use_rubber_pads()` gives both YAM finger pads (links tip_left / tip_right, both arms) a rubber material at run
  time. The stock YAM pads have no physics material, so PhysX uses its default 0.5 / 0.5; real YAM pads are
  rubber-faced. The binding goes through the PhysX tensor API (`root_view.set_material_properties`, the same call
  Isaac Lab's material randomizer uses), so it works however the arm USD is instanced, and it is applied on the first
  control step, before any grasp. PhysX averages the two touching materials, so the pad-object friction becomes
  (pad + object) / 2.


Import it from task.py next to yamkit:

    import gripkit
    gripkit.use_rubber_pads()          # before the simulator builds the scene; idempotent

Plain Python at import time; touches Isaac only inside the simulator, through yam_arena.scene's object_states hook.
"""

import importlib.abc
import importlib.util
import os
import sys

# -- materials ---------------------------------------------------------------------------------------------------

DEFAULT_PAD = (0.5, 0.5)  # PhysX default material: what the stock YAM pads get (no material bound)
RUBBER_PAD = (1.0, 0.8)  # static, dynamic: rubber pad faces (rubber on card/plastic/wood 0.6-1.2)
TIP_LINKS = ("tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")

_STATE = {"pads": None, "bound": 0, "hooked": False}


def pad_shape_ranges(link_paths, shapes_per_link, tips=TIP_LINKS):
    """(start, end) shape-index ranges of the fingertip links in an articulation's material buffer. Shapes are
    stored link by link in `link_paths` order, `shapes_per_link[i]` per link (same layout Isaac Lab's
    randomize_rigid_body_material relies on)."""
    out, start = [], 0
    for path, n in zip(link_paths, shapes_per_link):
        if str(path).rstrip("/").rsplit("/", 1)[-1] in tips:
            out.append((start, start + int(n)))
        start += int(n)
    return out


def _bind(env) -> int:
    """Set the pad material on every fingertip shape of both arms. Returns how many shapes were set."""
    import torch
    import warp as wp

    static, dynamic = _STATE["pads"]
    base = env.unwrapped
    n = 0
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError:
            continue
        view = art.root_view
        counts = [art._physics_sim_view.create_rigid_body_view(p).max_shapes for p in view.link_paths[0]]
        mats = wp.to_torch(view.get_material_properties()).clone()
        for a, b in pad_shape_ranges(view.link_paths[0], counts):
            mats[:, a:b, 0] = static
            mats[:, a:b, 1] = dynamic
            mats[:, a:b, 2] = 0.0
            n += b - a
        ids = torch.arange(mats.shape[0], dtype=torch.int32)
        view.set_material_properties(wp.from_torch(mats.contiguous(), dtype=wp.float32), wp.from_torch(ids, dtype=wp.int32))
    return n


def _patch(scene) -> None:
    if getattr(scene, "_gripkit", False):
        return
    scene._gripkit = True
    inner = scene.object_states

    def object_states(env, names):
        if _STATE["pads"] is not None and not _STATE["bound"]:
            _STATE["bound"] = _bind(env) or -1
            if os.environ.get("GRIPKIT_LOG"):  # probe log only
                print(f"gripkit: pad material {_STATE['pads']} bound to {_STATE['bound']} fingertip shapes", flush=True)
        return inner(env, names)

    scene.object_states = object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _patch_late(module)


def _patch_late(module) -> None:
    # yamkit (if imported) replaces object_states in its own hook; wrap after it so both run.
    yk = sys.modules.get("yamkit")
    if yk is not None and not getattr(module, "_yamkit", False):
        yk._patch(module)
    _patch(module)


def use_pads(static: float, dynamic: float) -> None:
    """Bind (static, dynamic) friction to both arms' finger pads on the first control step."""
    _STATE["pads"] = (float(static), float(dynamic))
    if _STATE["hooked"]:
        return
    _STATE["hooked"] = True
    if "yam_arena.scene" in sys.modules:
        _patch_late(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, _Hook())


def use_rubber_pads() -> None:
    """The recommended setting: rubber pad faces (1.0 / 0.8)."""
    use_pads(*RUBBER_PAD)


def pads_from_env(var: str = "GRIP_PADS") -> str:
    """'rubber' (default) or 'default' (stock pads) from an environment variable; applies it and returns the name."""
    name = os.environ.get(var, "rubber")
    if name == "rubber":
        use_rubber_pads()
    elif name != "default":
        raise ValueError(f"{var} must be 'rubber' or 'default', got {name!r}")
    return name
