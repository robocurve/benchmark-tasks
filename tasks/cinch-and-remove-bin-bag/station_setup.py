"""Common standard YAM station configuration for the three development tasks.

Matches PR #1906's station rig. Import Isaac-dependent helpers only after AppLauncher.
The 2.5 mm work mat preserves the prototypes' physical z=0.750 surface.
"""
from dataclasses import asdict
from pathlib import Path
import hashlib
import json
import numpy as np

CONTROL_HZ = 20
POLICY_CAMERAS = ('top_cam', 'left_cam', 'right_cam')
REFERENCE = 'https://github.com/robocurve/benchmark-tasks/pull/1906'
REFERENCE_COMMIT = 'f2ac49344836c2cd9ddbb36017ce18b80d4a6a52'

def rig():
    from yam_arena import station
    return station.rig()

def scene_assets():
    from yam_arena import station
    from yam_arena.scene import _station_room, _stand_part, _box
    assets = list(_station_room())
    assets.extend(_stand_part(name, *part) for name, part in station.camera_stand(rig().top_cam).items())
    # An explicit physical mat, not a change to calibrated arm mounts or table height.
    thickness = 0.750 - station.TABLE_TOP
    assert 0 < thickness < 0.005
    assets.append(_box('work_mat', (0.80, 1.00, thickness),
                       (0.35, 0.0, station.TABLE_TOP + thickness/2), (0.28, 0.31, 0.33)))
    return assets

def setup_manifest(out, embodiment, task, mode, source_dir):
    from yam_arena import station
    r = embodiment.rig
    if r.bases != station.BASES:
        raise RuntimeError('Task arm mounts differ from the calibrated YAM station')
    manifest = {'task': task, 'mode': mode, 'reference_pr': REFERENCE,
        'reference_commit': REFERENCE_COMMIT, 'rig': asdict(r),
        'table_top_m': station.TABLE_TOP, 'work_surface_m': 0.750,
        'control_hz': CONTROL_HZ, 'policy_cameras': list(POLICY_CAMERAS),
        'overview_is_diagnostic_only': True, 'full_score_verified': False,
        'sources': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sorted(Path(source_dir).glob('*.py'))}}
    Path(out, 'setup_manifest.json').write_text(json.dumps(manifest, indent=2))

def array(value):
    if hasattr(value, 'torch'):
        value = value.torch
    return value.detach().cpu().numpy()

def save_views(base, out, name='setup'):
    from PIL import Image, ImageDraw
    views = []
    for camera in (*POLICY_CAMERAS, 'overview_cam'):
        if camera not in base.scene.sensors:
            raise RuntimeError('Required camera missing: '+camera)
        rgb = array(base.scene.sensors[camera].data.output['rgb'])[0, :, :, :3].astype('uint8')
        im = Image.fromarray(rgb)
        im.save(Path(out, name+'_'+camera+'.png'))
        views.append((camera, im.resize((640, 360))))
    sheet = Image.new('RGB', (1280, 768), '#16202b')
    draw = ImageDraw.Draw(sheet)
    for i, (camera, im) in enumerate(views):
        x, y = (i % 2)*640, (i // 2)*384
        draw.text((x+12, y+6), camera, fill='white')
        sheet.paste(im, (x, y+24))
    sheet.save(Path(out, name+'.png'))

def setup_result(out, steps):
    Path(out, 'setup_result.json').write_text(json.dumps({
        'setup_complete': True, 'steps': steps, 'score': None,
        'full_task_success': False, 'scope': 'station setup and settling only'
    }, indent=2))

