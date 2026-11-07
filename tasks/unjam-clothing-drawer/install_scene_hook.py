"""Install a narrow, reversible custom-scene hook in the pinned YAM adapter.

No dependencies, robot gains, or ordinary task behaviour are changed.
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import tempfile

BASE_SHA256 = "7c4081aa811b2839f908d533e12885dac7a0aaed62ff8da59e36995b50b87809"
MARKER = "# ROBOCURVE_CUSTOM_OBJECTS_V1"
OLD = "scene=Scene([*task_room(task.scene, embodiment.rig.top_cam), *(task_object(o) for o in task.objects)]),"
NEW = "scene=Scene([*task_room(task.scene, embodiment.rig.top_cam), *_custom_task_objects(task)]),"
ANCHOR = "def build_task_environment(embodiment, task: task_file.TaskFile, control_hz: float | None = None):"
HOOK = '''# ROBOCURVE_CUSTOM_OBJECTS_V1
def _custom_task_objects(task):
    builder = getattr(task.module, "build_scene_objects", None)
    if builder is None:
        return [task_object(obj) for obj in task.objects]
    objects = list(builder())
    names = [obj.name for obj in objects]
    if len(names) != len(set(names)) or set(names) != set(task.names):
        raise ValueError("build_scene_objects() must return exactly the declared OBJECTS names")
    return objects


'''


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def patched(original: bytes) -> bytes:
    if digest(original) != BASE_SHA256:
        raise ValueError("scene.py differs from the inspected version; refusing to patch it")
    text = original.decode()
    if text.count(OLD) != 1 or text.count(ANCHOR) != 1:
        raise ValueError("Expected integration points were not found exactly once")
    return text.replace(OLD, NEW).replace(ANCHOR, HOOK + ANCHOR).encode()


def atomic_write(path: Path, data: bytes) -> None:
    fd, name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
        os.chmod(name, path.stat().st_mode & 0o777)
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--undo", action="store_true")
    parser.add_argument("--source", type=Path, help="Explicit scene.py path (normally autodetected)")
    args = parser.parse_args()
    if args.check and args.undo:
        parser.error("Choose --check or --undo, not both")
    if args.source:
        path = args.source.resolve()
    else:
        import yam_arena
        path = Path(yam_arena.__file__).resolve().parent / "scene.py"
    backup = path.with_name("scene.py.before_drawer_hook")
    current = path.read_bytes()
    if MARKER.encode() in current:
        if not backup.is_file() or digest(backup.read_bytes()) != BASE_SHA256:
            raise SystemExit("Hook found, but its original backup is missing or different. No changes made.")
        if current != patched(backup.read_bytes()):
            raise SystemExit("Patched file has other edits. No changes made; preserve and review those edits.")
        if args.undo:
            atomic_write(path, backup.read_bytes())
            print("Restored original:", path)
        else:
            print("PASS: custom-scene hook is installed:", path)
        return
    if args.undo:
        if digest(current) == BASE_SHA256:
            print("Already using the original scene.py")
            return
        raise SystemExit("Unrecognized source; no changes made")
    candidate = patched(current)
    if args.check:
        raise SystemExit("Compatible source, but hook is not installed. Run this command without --check.")
    if backup.exists():
        if backup.read_bytes() != current:
            raise SystemExit("A different backup already exists. No changes made.")
    else:
        backup.write_bytes(current)
    atomic_write(path, candidate)
    print("Installed custom-scene hook:", path)
    print("Backup:", backup)
    print("Undo: python", Path(__file__).resolve(), "--undo")


if __name__ == "__main__":
    main()
