"""Pass requested image size to Isaac's cameras, not just observation resizing.

Run once in the documented environment before recording. Fails on an unknown
runtime rather than silently claiming native 720p from upscaled 360p frames.
"""
from pathlib import Path
from importlib.util import find_spec


def main():
    spec = find_spec('yam_arena')
    path = Path(next(iter(spec.submodule_search_locations))) / 'inspect_embodiment.py'
    old = 'enable_cameras=True, wrist_cam_mount=self.wrist_cam_mount, contact_sensors=False, rig=self.rig'
    new = old + ', cam_height=self.cam_height, cam_width=self.cam_width'
    source = path.read_text()
    if new in source:
        print('Native camera dimensions already forwarded')
        return
    if source.count(old) != 1:
        raise RuntimeError('Unknown camera constructor; inspect runtime before recording')
    backup = path.with_suffix('.py.pre-native-resolution')
    if not backup.exists():
        backup.write_text(source)
    path.write_text(source.replace(old, new))
    print('Native camera dimensions forwarded; restart the simulation process')


if __name__ == '__main__':
    main()
