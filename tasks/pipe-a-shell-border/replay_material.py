"""Replay recorded native inputs to check material coupling independently.

This is a prescribed-input diagnostic, never a robot demonstration. It uses
every saved physics-step pose and velocity with the unchanged material model.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from native import Coupling


def replay(directory, output):
    frozen_material=directory/'task-source'/'material.py'
    if (frozen_material.exists() and hashlib.sha256(frozen_material.read_bytes()).digest()
            !=hashlib.sha256(Path(__file__).with_name('material.py').read_bytes()).digest()):
        raise ValueError('Material source changed: run replay with the recorded task-source modules')
    source=directory/'tool-substeps.npz'
    with np.load(source,allow_pickle=False) as data:
        rows=data['values'];columns=data['columns'].tolist()
    expected=(['time_after_s','dt_s']+
        ['barrel_'+x for x in ('x','y','z','qx','qy','qz','qw')]+
        ['barrel_'+x for x in ('vx','vy','vz','wx','wy','wz')]+
        ['plunger_'+x for x in ('x','y','z','qx','qy','qz','qw')]+
        ['piston_m','piston_velocity_m_s','pressure_reaction_n','seal_force_n','emitted_m3'])
    if columns!=expected or rows.ndim!=2 or rows.shape[1]!=len(expected) or not np.isfinite(rows).all():
        raise ValueError('Complete finite native input columns are required')
    report=json.loads((directory/'preview.json').read_text())
    dt=float(report['physics_dt_s'])
    # The first recorder stored scalars after NumPy's weak-scalar promotion
    # through a float32 array. Native poses/velocities remain exact; recover
    # the known uniform integration step from its float64 simulator readback.
    scalar_rounding='tool_substep_recording_precision' not in report
    time_tolerance=max(1e-8,float(np.spacing(np.float32(len(rows)*dt)))) if scalar_rounding else 1e-8
    if (not len(rows) or dt<=0 or not np.allclose(rows[:,1],dt,atol=dt*1e-7,rtol=0)
            or not np.allclose(np.arange(1,len(rows)+1)*dt,rows[:,0],atol=time_tolerance,rtol=0)):
        raise ValueError('Native time sequence is incomplete or discontinuous')
    start=time.monotonic();coupling=Coupling(boundary_velocity=report.get('material_boundary_velocity','native'));maximum=np.zeros(5)
    coupling.prime_boundary(rows[0,2:9])
    for row in rows:
        coupling.advance(row[2:9],row[9:15],row[15:22],dt)
        actual=np.array([coupling.last_piston_m,coupling.last_piston_velocity_m_s,
            coupling.last_reaction_n,coupling.last_seal_force_n,coupling.material.emitted_m3])
        maximum=np.maximum(maximum,np.abs(actual-row[22:27]))
    output.mkdir(parents=True,exist_ok=False)
    coupling.material.save(output/'material.npz')
    with np.load(directory/'material.npz',allow_pickle=False) as native:
        if native['volume'].shape!=coupling.material.volume.shape:
            raise ValueError('Saved material grid differs from current model')
        volume_error=np.abs(coupling.material.volume-native['volume'])
    report={'kind':'prescribed_input_replay_not_robot_evidence','native_steps':len(rows),
        'simulated_seconds':float(rows[-1,0]),'wall_seconds':time.monotonic()-start,
        'input_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'recorded_scalar_float32_rounding':scalar_rounding,
        'replayed_dt_from_native_report_s':dt,
        'maximum_absolute_errors':dict(zip(expected[22:27],maximum.tolist())),
        'maximum_cell_volume_error_m3':float(volume_error.max()),
        'sum_absolute_volume_error_m3':float(volume_error.sum()),
        'balance':coupling.material.balance()}
    (output/'replay.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(replay(args.directory,args.out),indent=2))
