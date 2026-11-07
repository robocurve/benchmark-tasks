"""Optional, guarded camera-resolution forwarding for the pinned YAM adapter.

Makes requested frames above 640x360 render at their real requested size.
The existing lower-resolution runs keep their original native 640x360 rendering.
"""
from pathlib import Path
import argparse
import hashlib
import os
import tempfile

BASE_SHA="1b0cbbcdf98b7385e4d213e88bd59c296639bb7031c80e8d73efdce0eabcc4c6"
MARKER="# DRAWER_NATIVE_CAMERA_RESOLUTION_V1"
OLD='''        emb = BimanualYamEmbodiment(
            enable_cameras=True, wrist_cam_mount=self.wrist_cam_mount, contact_sensors=False, rig=self.rig
        )'''
NEW='''        # DRAWER_NATIVE_CAMERA_RESOLUTION_V1
        emb = BimanualYamEmbodiment(
            enable_cameras=True, wrist_cam_mount=self.wrist_cam_mount, contact_sensors=False, rig=self.rig,
            cam_width=max(640, self.cam_width), cam_height=max(360, self.cam_height),
        )'''


def patch(data):
    if hashlib.sha256(data).hexdigest()!=BASE_SHA:raise ValueError("Unknown adapter source. No changes made.")
    text=data.decode()
    if text.count(OLD)!=1:raise ValueError("Camera constructor did not match exactly. No changes made.")
    return text.replace(OLD,NEW).encode()


def write(path,data):
    fd,temp=tempfile.mkstemp(prefix=path.name+".",dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as f:f.write(data)
        os.chmod(temp,path.stat().st_mode&0o777);os.replace(temp,path)
    finally:Path(temp).unlink(missing_ok=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--undo',action='store_true')
    parser.add_argument('--source',type=Path)
    args=parser.parse_args()
    if args.check and args.undo:parser.error('Choose --check or --undo')
    if args.source:path=args.source.resolve()
    else:
        import yam_arena
        path=Path(yam_arena.__file__).resolve().parent/'inspect_embodiment.py'
    backup=path.with_name('inspect_embodiment.py.before_hq_cameras')
    current=path.read_bytes()
    if MARKER.encode() in current:
        if not backup.is_file() or current!=patch(backup.read_bytes()):
            raise SystemExit('Source or backup has other edits. No changes made.')
        if args.undo:write(path,backup.read_bytes());print('Restored original camera adapter')
        else:print('PASS: native high-resolution camera forwarding installed')
        return
    candidate=patch(current)
    if args.undo:print('Already using original camera adapter');return
    if args.check:raise SystemExit('Compatible source; run this installer without --check first')
    if backup.exists() and backup.read_bytes()!=current:raise SystemExit('Different backup exists. No changes made.')
    if not backup.exists():backup.write_bytes(current)
    write(path,candidate)
    print('Installed native camera resolution forwarding:',path)
    print('Original backup:',backup)


if __name__=='__main__':main()
