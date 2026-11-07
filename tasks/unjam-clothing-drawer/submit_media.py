#!/usr/bin/env python3
"""Stage existing recordings and invoke the repository's official uploader."""
import argparse
import getpass
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def media_files(folder):
    if not folder.is_dir():
        return None
    model = next((folder / n for n in ('model_run.mp4', 'dream.mp4') if (folder / n).is_file()), None)
    image = next((folder / n for n in ('setup.jpg', 'setup.png', 'setup.jpeg') if (folder / n).is_file()), None)
    demo = folder / 'demo.mp4'
    if model and image and demo.is_file():
        return {'model_run.mp4': model, 'demo.mp4': demo, 'setup' + image.suffix: image}
    return None


def validate(files):
    ffprobe = shutil.which('ffprobe')
    if not ffprobe:
        raise SystemExit('ffprobe is required to check the videos. Install ffmpeg in this environment first.')
    for name, path in files.items():
        limit = 50_000_000 if name.endswith('.mp4') else 10_000_000
        if not 0 < path.stat().st_size <= limit:
            raise SystemExit(f'{path.name}: empty or larger than the upload limit ({limit:,} bytes).')
        if name.endswith('.mp4'):
            result = subprocess.run([ffprobe, '-v', 'error', '-select_streams', 'v:0',
                                     '-show_entries', 'stream=codec_name', '-of', 'json', str(path)],
                                    check=True, capture_output=True, text=True)
            streams = json.loads(result.stdout).get('streams', [])
            if not streams or streams[0].get('codec_name') != 'h264':
                raise SystemExit(f'{path.name}: expected H.264 video; convert it before uploading.')
        else:
            with path.open('rb') as handle:
                magic = handle.read(8)
            valid = magic == b'\x89PNG\r\n\x1a\n' if path.suffix == '.png' else magic.startswith(b'\xff\xd8\xff')
            if not valid:
                raise SystemExit(f'{path.name}: does not have the expected image signature.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-media', type=Path, help='Folder containing dream/model_run.mp4, demo.mp4 and setup.jpg/png')
    parser.add_argument('--team', default='T-20', help='Robocurve team code (default: T-20)')
    parser.add_argument('--prepare-only', action='store_true', help='Stage and check files without uploading or asking for a token')
    args = parser.parse_args()
    task = Path(__file__).resolve().parent
    repo = task.parents[1]
    uploader = repo / 'tools/upload_media.py'
    if task.name != 'unjam-clothing-drawer' or not uploader.is_file():
        raise SystemExit('Run this helper from the task folder in the benchmark-tasks repository.')
    if args.source_media:
        source = args.source_media.expanduser().resolve()
        files = media_files(source)
        if files is None:
            raise SystemExit('Source folder must contain model_run.mp4 (or dream.mp4), demo.mp4 and setup.jpg/png.')
    else:
        root = Path.home() / 'yam-eval'
        candidates = [*root.glob('drawer-submit-v6-media-*'), root / 'drawer-submission-media']
        candidates = [p for p in candidates if media_files(p)]
        if candidates:
            source = max(candidates, key=lambda p: p.stat().st_mtime)
        elif media_files(task / 'media'):
            source = task / 'media'
        else:
            raise SystemExit('No complete prepared media folder found. Supply --source-media /path/to/folder.')
        files = media_files(source)
    print('Source:', source, flush=True)
    validate(files)
    staged = task / 'media'
    staged.mkdir(exist_ok=True)
    destinations = {}
    for name, path in files.items():
        destination = staged / name
        if path.resolve() != destination.resolve():
            # Copy to a temporary sibling, preserving the previous staged file on failure.
            temporary = destination.with_name(destination.name + '.copying')
            try:
                shutil.copy2(path, temporary)
                temporary.replace(destination)
            finally:
                temporary.unlink(missing_ok=True)
        destinations[name] = destination
        print(f'Staged {name}: {destination.stat().st_size:,} bytes', flush=True)
    print('Task tag: unjam-clothing-drawer; original recordings unchanged.', flush=True)
    if args.prepare_only:
        print('Preparation complete. No upload was attempted.')
        return
    team = args.team.strip()
    if not team or ':' in team or any(c.isspace() for c in team):
        raise SystemExit('Invalid team code.')
    print(f'Upload team: {team}. Enter the Robocurve token for this team, not a Dream Labs or GitHub key.', flush=True)
    # Always ask afresh so an expired or incorrect inherited token is not silently reused.
    try:
        token = getpass.getpass('Robocurve team token: ').strip()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit('\nNo token entered; nothing uploaded.')
    if not token:
        raise SystemExit('No token entered; nothing uploaded.')
    env = os.environ.copy()
    env.update(ROBOCURVE_TEAM=team, ROBOCURVE_TOKEN=token)
    # Use the documented board; do not inherit a stale endpoint override.
    env.pop('ROBOCURVE_BOARD', None)
    screenshot = next(p for name, p in destinations.items() if name.startswith('setup.'))
    result = subprocess.run([sys.executable, str(uploader), str(task),
                             '--video', str(destinations['model_run.mp4']),
                             '--demo', str(destinations['demo.mp4']),
                             '--screenshot', str(screenshot)], cwd=repo, env=env)
    if result.returncode:
        print('Upload failed. Staged files remain available; no Git actions were taken.', file=sys.stderr)
        print('If the response is 403, the organiser must verify the team/token or server access; moving files cannot fix it.', file=sys.stderr)
        raise SystemExit(result.returncode)
    spec = json.loads((task / 'task.json').read_text())
    if not isinstance(spec.get('media'), str) or not spec['media'].strip():
        raise SystemExit('Uploader exited successfully but task.json has no media ID. Do not commit yet.')
    print('Media linked:', spec['media'])
    print('Review and commit task.json, then push the existing PR branch. This helper does not commit or push.')


if __name__ == '__main__':
    main()
