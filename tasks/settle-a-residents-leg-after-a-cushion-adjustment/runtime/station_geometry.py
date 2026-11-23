"""Expose the task's MuJoCo wheels to a compatible station interpreter for one run."""

import os
import platform
import shutil
import subprocess
import sys
import sysconfig
import tempfile
from pathlib import Path

# Include wheel metadata and any wheel-bundled shared libraries. The arena
# continues to provide its own NumPy, SciPy, PyTorch and Isaac packages.
PACKAGES = {
    "mujoco": "mujoco",
    "glfw": "glfw",
    "OpenGL": "pyopengl",
    "absl": "absl_py",
    "etils": "etils",
}
ABI = "import platform, sysconfig; print(sysconfig.get_config_var('SOABI'), platform.machine())"
PROBE = "import mujoco as m; assert m.__version__ == '3.3.7'; m.MjModel.from_xml_string('<mujoco/>')"


def expose_wheels(directory):
    site = Path(sysconfig.get_path("platlib"))
    for module, distribution in PACKAGES.items():
        for path in (
            site / module,
            site / (distribution + ".libs"),
            *site.glob(distribution + "-*.dist-info"),
        ):
            if not path.exists():
                continue
            target = directory / path.name
            try:
                target.symlink_to(path, target_is_directory=path.is_dir())
            except OSError:
                if path.is_dir():
                    shutil.copytree(path, target)
                else:
                    shutil.copy2(path, target)


def main():
    arena, *args = sys.argv[1:]
    root = Path(__file__).resolve().parents[1]
    local_abi = f"{sysconfig.get_config_var('SOABI')} {platform.machine()}"
    arena_abi = subprocess.check_output([arena, "-c", ABI], text=True).strip()
    with tempfile.TemporaryDirectory(
        prefix="heel-station-", dir=root / "runtime"
    ) as tmp:
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(filter(None, (tmp, env.get("PYTHONPATH"))))
        if local_abi == arena_abi:
            expose_wheels(Path(tmp))
            print("Using the task environment's MuJoCo geometry wheels.", flush=True)
        else:
            # pip runs in the task environment but selects wheels for the arena's ABI.
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pip",
                    "--python",
                    arena,
                    "install",
                    "--no-deps",
                    "--target",
                    tmp,
                    "mujoco==3.3.7",
                    "glfw",
                    "PyOpenGL",
                    "absl-py",
                    "etils",
                ],
                check=True,
            )
        subprocess.run([arena, "-c", PROBE], env=env, check=True)
        return subprocess.run(
            [arena, "-m", "isaac.qualify", *args], env=env, cwd=root
        ).returncode


if __name__ == "__main__":
    sys.exit(main())
