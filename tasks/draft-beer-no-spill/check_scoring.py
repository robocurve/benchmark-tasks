"""Pure scoring checks. These do not run or validate physics."""
import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('beer_score',Path(__file__).with_name('scoring.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def check():
    score=module.BeerScore()
    center=np.array([.3,0,.7475])
    rotation=np.eye(3)
    assert score.update([],center,rotation)['score']==0
    # A stream above the aperture is pending and earns nothing.
    assert score.update((np.array([[0,0,.2]])+center),center,rotation)['pending_particles']==1
    clean=score.update((np.array([[0,0,.05]])+center),center,rotation)
    assert clean['score']==module.QUANTUM_ML/module.TARGET_ML
    clean=score.update((np.array([[0,0,.05],[0,0,.06]])+center),center,rotation)
    assert clean['score']==2*module.QUANTUM_ML/module.TARGET_ML
    spilled=score.update((np.array([[.1,0,.05],[0,0,.06]])+center),center,rotation)
    assert spilled['score']==0 and spilled['spill_latched']
    assert score.update((np.array([[0,0,.05],[0,0,.06]])+center),center,rotation)['score']==0
    try:score.update((np.array([[0,0,.05]])+center),center,rotation)
    except ValueError:pass
    else:raise AssertionError('Missing particles must invalidate evaluation')
    recovered=score.update((np.array([[0,0,.05],[0,0,.06]])+center),center,rotation)
    assert recovered['score']==0 and recovered['retained_ml']==2*module.QUANTUM_ML
    # Returned central splashes remain uncredited while above the opening.
    splash=module.BeerScore()
    splash.update([center+[0,0,.05]],center,rotation)
    airborne=splash.update([center+[0,0,.14]],center,rotation)
    assert airborne['score']==0 and not airborne['spill_latched'] and airborne['pending_particles']==1
    assert splash.update([center+[0,0,.05]],center,rotation)['score']>0
    # Escaping laterally above the opening permanently fails, even if recaptured.
    assert splash.update([center+[.05,0,.14]],center,rotation)['spill_latched']
    assert splash.update([center+[0,0,.05]],center,rotation)['score']==0
    # A world-vertical return through a tilted opening stays in its footprint.
    c,s=np.cos(np.deg2rad(20)),np.sin(np.deg2rad(20))
    tilt=np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
    entered=center+tilt@np.array([0.,0.,.10])
    raised=entered+np.array([0.,0.,.13])
    assert np.linalg.norm(((raised-center)@tilt)[:2])>module.OUTER_RADIUS
    tilted_splash=module.BeerScore()
    assert tilted_splash.update([entered],center,tilt)['score']>0
    airborne=tilted_splash.update([raised],center,tilt)
    assert not airborne['spill_latched'] and airborne['pending_particles']==1 and airborne['score']==0
    assert tilted_splash.update([entered],center,tilt)['score']>0
    # Actual horizontal escape above the tilted mouth still latches zero.
    assert tilted_splash.update([raised+[.08,0.,0.]],center,tilt)['spill_latched']
    assert tilted_splash.update([entered],center,tilt)['score']==0
    # Below the rim, crossing the physical sidewall remains a spill.
    tilted_wall=module.BeerScore()
    assert tilted_wall.update([center+tilt@np.array([.035,0.,.05])],center,tilt)['score']>0
    assert tilted_wall.update([center+tilt@np.array([.040001,0.,.05])],center,tilt)['spill_latched']
    # A numerical wall/base contact band is uncredited, not a physical escape.
    boundary=module.BeerScore()
    boundary.update([center+[.035,0,.010]],center,rotation)
    band=boundary.update([center+[.036002,0,.010]],center,rotation)
    assert band['score']==0 and band['pending_particles']==1 and not band['spill_latched']
    assert boundary.update([center+[.035,0,.010]],center,rotation)['score']>0
    assert boundary.update([center+[.040001,0,.010]],center,rotation)['spill_latched']
    assert boundary.update([center+[0,0,.05]],center,rotation)['score']==0
    baseband=module.BeerScore().update([center+[0,0,.007999]],center,rotation)
    assert baseband['score']==0 and baseband['pending_particles']==1 and not baseband['spill_latched']
    assert module.BeerScore().update([center+[0,0,-.000001]],center,rotation)['spill_latched']
    assert module.BeerScore().update([center+[.038,0,.01]]*15000,center,rotation)['score']==0
    sideways=np.array([[1.,0,0],[0,0,1],[0,-1,0]])
    assert module.BeerScore().update([[.3,.2,.749]],[.3,0,.85],sideways)['spill_latched']
    far_center=np.array([2.,0.,.7475])
    assert module.BeerScore().update([far_center+[0,0,.05]],far_center,rotation)['score']>0
    print('Scoring arithmetic and permanent spill latch checks passed; native physics not established.')

if __name__=='__main__':check()
