"""Author a previously measured loaded wheel shape before physics attaches.

The reference was settled by native contact/constraints with no robot. Clocking
the entire assembly preserves that internal geometry. This is a reset candidate,
not a task-time body writer, controller, or success certificate.
"""
import hashlib
import json
from pathlib import Path
import numpy as np


def load_configuration():
    path = Path(__file__).with_name('assets') / 'preloaded-reset-candidate.json'
    data = json.loads(path.read_text())
    if data['schema'] != 'wheel-preloaded-reset-v1' or len(data['bodies']) != 129:
        raise ValueError('Incomplete native preloaded configuration')
    for row in data['bodies']:
        p, q = np.asarray(row['position_in_hub_m']), np.asarray(row['rotation_in_hub_xyzw'])
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all():
            raise ValueError('Invalid native preloaded body geometry')
        if abs(float(q @ q) - 1) > 1e-5:
            raise ValueError('Non-unit preloaded rotation')
    return data, hashlib.sha256(path.read_bytes()).hexdigest()


def author_preloaded_configuration(stage, metadata):
    """Called only during scene construction, before native stage attachment."""
    from pxr import Gf, UsdGeom
    data, digest = load_configuration()
    if metadata['reset_profile'] != data['source_reset_profile']:
        raise ValueError('Preloaded candidate is specific to the two-defect reset')
    if abs(np.degrees(metadata['nipple_flat_phase_rad']) - data['source_nipple_flat_phase_deg']) > 1e-6:
        raise ValueError('Preloaded candidate requires the measured 45-degree nipple phase')
    root = metadata['hub_path'].rsplit('/', 1)[0]
    for row in data['bodies']:
        prim = stage.GetPrimAtPath(root + '/' + row['body'])
        if not prim:
            raise ValueError('Preloaded body missing: ' + row['body'])
        x, y, z, w = map(float, row['rotation_in_hub_xyzw'])
        matrix = Gf.Matrix4d(1.)
        matrix.SetRotate(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        matrix.SetTranslateOnly(Gf.Vec3d(*map(float, row['position_in_hub_m'])))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp(opSuffix='preloadedReset').Set(matrix)
    metadata['preloaded_reset'] = {'candidate': True, 'asset_sha256': digest,
        'source_report_sha256': data['source_report_sha256'],
        'operation': 'Author loaded internal geometry once, before physics attachment; zero initial velocities'}
    return metadata['preloaded_reset']
