"""Offline pressure-spreading experiment, not a robot demo or accepted score.

This diagnostic prescribes the measured-body inputs to Coupling. It tests the
task's constitutive law without claiming a robot demonstration.
The material sees only the nozzle aperture, reservoir pressure and deposited
surface. The prescribed motion is separate and never part of the material law.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from material import Icing, IcingParameters
from native import Coupling,rotation
from scoring import assess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--force',type=float,default=7.)
    p.add_argument('--head-fraction',type=float,default=.4)
    p.add_argument('--dx',type=float,choices=(.001,.0005,.00025),default=.0005)
    p.add_argument('--physics-hz',type=int,choices=(240,480,960),default=240)
    p.add_argument('--material-hz',type=int,choices=(480,960,1920,3840),default=480)
    p.add_argument('--star-depth',type=float,choices=(.28,.4,.5),default=.28)
    p.add_argument('--shear-yield-pa',type=float,choices=(800.,1200.),default=800.)
    p.add_argument('--density-kg-m3',type=float,choices=(850.,1000.),default=850.)
    a=p.parse_args()
    if not 4<=a.force<=10 or not .2<=a.head_fraction<=.6:p.error('Use bounded material-probe parameters')
    a.out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    m=Icing(replace(IcingParameters(),maximum_substep_s=1/a.material_hz,star_depth=a.star_depth,
                    shear_yield_pa=a.shear_yield_pa,density_kg_m3=a.density_kg_m3),
            dx=a.dx,xlim=(.22,.25),ylim=(-.06,.06))
    coupling=Coupling(m)
    quat=np.array((0.,-np.sin(np.pi/8),np.cos(np.pi/8),0.));R=rotation(quat);axis=R[:,1]
    dt=1/a.physics_hz;trace=[];previous=None
    def prescribed_sample(tip,force):
        nonlocal previous
        q=(m.emitted_m3+force/m.p.piston_area_m2*m.p.initial_volume_m3/m.p.bulk_modulus_pa)/m.p.piston_area_m2
        centre=np.asarray(tip)-R@np.array((0.,.08,0.))
        barrel=np.r_[centre,quat];plunger=np.r_[centre+axis*(q-.06),quat]
        velocity=np.zeros(6) if previous is None else np.r_[(centre-previous)/dt,[0.,0.,0.]]
        previous=centre.copy();coupling.advance(barrel,velocity,plunger,dt)
        trace.append({'time_s':m.time_s,'nozzle_m':list(map(float,tip)),'prescribed_force_n':force,
                      'prescribed_piston_m':q,'emitted_ml':m.emitted_m3*1e6})
    for shell in range(6):
        start=-.040+shell*.015
        for step in range(round(.8/dt)):
            s=step/round(.8/dt)
            fraction=max(0.,(s-a.head_fraction)/(1-a.head_fraction))
            force=a.force+(3.3-a.force)*fraction
            prescribed_sample((.235,start+.012*fraction,.8035),force)
        for _ in range(round(.04/dt)):prescribed_sample((.235,start+.012,.8035),0.)
    for _ in range(round(.5/dt)):prescribed_sample((.235,.05,.85),0.)
    m.save(a.out/'material.npz')
    report={'kind':'offline_constitutive_candidate','robot_demo':False,
            'status':'Uncalibrated shape experiment; no task feasibility claim',
            'wall_seconds':time.monotonic()-started,'balance':m.balance(),
            'cell_size_m':m.dx,'physics_hz':a.physics_hz,
            'assumptions':vars(m.p),
            'source_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                             for name in ('probe_shell_shape.py','material.py','native.py','scoring.py')},
            'shape_diagnostic':assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=m.emitted_m3,
                                     lost_m3=m.lost_m3,airborne_m3=float(m.air_volume.sum()),
                                     damage_depth_m=m.cake_damage_depth_m,completed=True,tools_clear=True),
            'trace':trace}
    (a.out/'probe.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='trace'},indent=2))


if __name__=='__main__':main()
