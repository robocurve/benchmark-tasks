"""Close-up cameras for watching a task run (used by log_cooler.py, log_berry.py and the check_cams_*.py wrappers).

Import it before Isaac Sim starts: install() adds a `latch_cam` to yam_arena's cameras, install_many() several named
ones (none of them in the observations the policy sees). If LATCH_FRAMES names a directory, save_frame() saves each
camera's image after every control step as <step>.npy: the single latch_cam's in that directory, several cameras' each
in a subdirectory named after it. frames_to_mp4.py tiles them under the robot's three cameras.
"""

import importlib.abc
import importlib.util
import os
import sys

import numpy as np

EYE_FROM_LATCH = np.array([0.075, -0.06, 0.055])  # world offset: past the backplate side, beyond the slot's latch end, up
FOCAL_MM = 12.0
BRIGHTEN = (0.45, 1.3)  # gamma and gain on the saved images (this panel only; the robot's cameras are untouched)
# LATCH_EYE="x,y,z" (world offset from the aim point, m) and LATCH_FOCAL (mm) override these, for trying views


def _look_at_xyzw(eye, look):
    """Isaac Lab "world" camera convention: looks along its +x, up is its +z. (The matrix-to-quaternion step is
    scipy's, which stays exact when looking nearly straight down, where the trace formula divided by about 0.)"""
    from scipy.spatial.transform import Rotation

    x = (look - eye) / np.linalg.norm(look - eye)
    y = np.cross([0.0, 0.0, 1.0], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    q = Rotation.from_matrix(np.stack([x, y, z], axis=1)).as_quat()  # x, y, z, w
    return tuple(float(v) for v in q)


def install(latch_world, eye_from=None, focal_mm=None):
    """Add the latch camera, aimed at latch_world (world frame), from eye_from (world offset from it)."""
    if os.environ.get("LATCH_EYE"):
        eye_from = [float(v) for v in os.environ["LATCH_EYE"].split(",")]
    focal = float(os.environ.get("LATCH_FOCAL") or focal_mm or FOCAL_MM)
    install_many([("latch_cam", latch_world, EYE_FROM_LATCH if eye_from is None else eye_from, focal)])


_NAMES: list = []


def install_many(cams):
    """Add cameras given as (name, aim point (world), eye offset from it (world), focal length (mm))."""
    specs = []
    for name, look, eye_from, focal in cams:
        look = np.asarray(look, float)
        eye = look + np.asarray(eye_from, float)
        specs.append((name, eye, _look_at_xyzw(eye, look), float(focal)))
        _NAMES.append(name)

    def patch(module):
        import dataclasses

        from yam_arena.rig import Camera, Lens

        cls = module.BimanualYamEmbodiment
        if getattr(cls, "_latch_cam", False):
            return
        orig = cls._camera_cfg
        base = module.ArenaCameraCfg
        base_names = {f.name for f in dataclasses.fields(base)} if dataclasses.is_dataclass(base) else set()

        def camera_cfg(self, height, width, overview):
            cfg = orig(self, height, width, overview)
            fields = [(f.name, f.type, getattr(cfg, f.name)) for f in dataclasses.fields(cfg) if f.name not in base_names]
            for name, eye, quat, focal in specs:
                cam = Camera(tuple(float(v) for v in eye), quat, "world", Lens.from_focal(focal))
                prim = "".join(w.capitalize() for w in name.split("_"))
                fields.append((name, module.CameraCfg, module.CameraCfg(
                    prim_path="{ENV_REGEX_NS}/" + prim, height=height, width=width, data_types=["rgb"],
                    spawn=module._pinhole(cam, height, width, near=0.01), offset=module._offset(cam),
                    renderer_cfg=module.RENDERER)))
            return module.make_configclass("YamCameraCfg", fields, bases=(module.ArenaCameraCfg,))()

        cls._camera_cfg = camera_cfg
        cls._latch_cam = True

    class Finder(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path, target=None):
            if fullname != "yam_arena.embodiment":
                return None
            sys.meta_path.remove(self)
            try:
                spec = importlib.util.find_spec(fullname)
            finally:
                sys.meta_path.insert(0, self)
            exec_module = spec.loader.exec_module

            def exec_and_patch(module):
                exec_module(module)
                patch(module)

            spec.loader.exec_module = exec_and_patch
            return spec

    if "yam_arena.embodiment" in sys.modules:
        patch(sys.modules["yam_arena.embodiment"])
    else:
        sys.meta_path.insert(0, Finder())


_STEP = {"k": 0}


def save_frame(environment) -> None:
    """Save each close-up camera's current image (call once per control step)."""
    out = os.environ.get("LATCH_FRAMES")
    if not out:
        return
    scene = environment.unwrapped.scene
    for name in _NAMES:
        try:
            cam = scene[name]
        except KeyError:
            continue
        rgb = cam.data.output["rgb"]
        rgb = rgb.torch if hasattr(rgb, "torch") else rgb
        d = out if _NAMES == ["latch_cam"] else os.path.join(out, name)
        os.makedirs(d, exist_ok=True)
        img = rgb[0, ..., :3].cpu().numpy().astype(np.float32) / 255.0
        img = np.clip(255.0 * BRIGHTEN[1] * img ** BRIGHTEN[0], 0, 255)  # close-ups look into shadows
        np.save(f"{d}/{_STEP['k']:05d}.npy", np.ascontiguousarray(img, dtype=np.uint8))
    _STEP["k"] += 1
