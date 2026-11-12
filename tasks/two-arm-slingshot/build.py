"""Generate the fixed toy slingshot, native passive spring and robot-only plan."""
import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).parent
TOP = .7475


def box(size, position, color, **kw):
    return dict(type='box', size=size, position=position, color=color, **kw)


def body(name, shapes, mass, position=(0, 0, 0), static=False):
    # Uniform density for compound boxes; decorative pieces carry no mass.
    pieces = [s for s in shapes if s.get('collision', True)]
    volumes = [8 * math.prod(s['size']) for s in pieces]
    com = [sum(v*s['position'][j] for v,s in zip(volumes,pieces))/sum(volumes) for j in range(3)]
    tensor = [[0.]*3 for _ in range(3)]
    for v,s in zip(volumes,pieces):
        m = mass*v/sum(volumes)
        d = [s['position'][j]-com[j] for j in range(3)]
        for j in range(3):
            for k in range(3):
                tensor[j][k] += m*(sum(d[t]**2+s['size'][t]**2/3 for t in range(3) if t!=j)
                                   if j==k else -d[j]*d[k])
    return dict(name=name, shapes=shapes, mass=mass, position=list(position),
                static=static, center_of_mass=com, inertia=dict(tensor=tensor))


def asset(name, bodies, **kw):
    return dict(name=name, bodies=bodies, solver_position_iterations=32,
                solver_velocity_iterations=8, **kw)


def build():
    wood = [.45,.23,.09,1]
    dark = [.12,.13,.16,1]
    frame = [box([.07,.095,.009],[0,0,.009],wood,friction=[.25,.18]),
             box([.012,.075,.009],[0,0,.035],wood)]
    for y in (-.070,.070):
        frame.append(box([.012,.012,.052],[0,y,.087],wood))
    # The left arm pinches this raised crossbar across its narrow dimension.
    frame += [box([.008,.060,.008],[x,.13,.10],wood) for x in (-.055,.055)]
    frame += [box([.063,.009,.009],[0,.19,.10],wood),
              box([.063,.008,.008],[0,.07,.10],wood),
              box([.012,.012,.033],[0,0,.049],wood),
              box([.12,.005,.005],[-.10,0,.08],dark)]
    pouch = [box([.030,.030,.003],[0,0,-.028],dark),
             box([.004,.030,.020],[-.030,0,-.005],dark),
             box([.003,.030,.003],[.032,0,-.022],dark)]
    # Low side rails retain the preloaded sphere without blocking the fingers.
    pouch += [box([.03,.003,.005],[0,y,-.020],dark) for y in (-.032,.032)]
    # A U carriage visibly connects the pouch floor around the horizontal guide.
    pouch += [box([.012,.003,.015],[0,y,-.046],dark) for y in (-.009,.009)]
    pouch += [box([.012,.012,.003],[0,0,-.060],dark)]
    assets = {'launcher':asset('Launcher',[
        body('frame',frame,.30),body('pouch',pouch,.025,[-.04,0,.13])],joints=[dict(
            name='elastic_draw',type='prismatic',parent='frame',child='pouch',
            position_parent=[-.04,0,.13],axis='X',lower=-.18,upper=.025,
            stiffness=45.,damping=.04,target=0.)])}
    radius=.025; mass=.035
    assets['ball']=asset('Ball',[dict(name='ball',mass=mass,
        inertia=dict(diagonal=[.4*mass*radius**2]*3),shapes=[dict(type='sphere',
        size=[radius],color=[.85,.07,.05,1],friction=[.8,.65])])])
    # Catch tray extends the station tabletop; no object is attached to it.
    floor=[box([.34,.23,.012],[.85,0,TOP-.0115],[.16,.28,.37,1]),
           box([.012,.23,.20],[1.19,0,TOP+.20],[.16,.28,.37,1])]
    floor += [box([.34,.012,.15],[.85,y,TOP+.15],[.16,.28,.37,1]) for y in (-.242,.242)]
    assets['fixture']=asset('Fixture',[body('fixture',floor,1.,static=True)])
    poses={'launcher':[.30,-.05,TOP], 'ball':[.26,-.05,TOP+.13], 'fixture':[0,0,0]}
    for tower,x in enumerate((.61,.67),1):
        for level in range(3):
            name=f'tower{tower}_{level}'
            assets[name]=asset(name,[body(name,[box([.015,.035,.025],[0,0,0],
                [.30+.12*level,.65,.25,1])],.018)])
            poses[name]=[x,-.075,TOP+.0255+.05*level]
    cfg=dict(kind='two_arm_slingshot',
        instruction='Hold the slingshot frame with the left arm. Pull back and release the loaded ball with the right arm to knock down both towers. Keep both arms behind the launch line.',
        table_top=TOP,control_hz=20,max_steps=1200,hold_s=1.,
        initial_tcp=[.30,.14,.92],initial_rotation_xyzw=[-math.sqrt(.5),math.sqrt(.5),0,0],
        initial_grip=1.,poses=poses,handle_offset=[0,.19,.10],
        pouch_rest=[-.04,0,.13],overview_position=[-.12,0,2.55],overview_target=[.55,0,.75])
    plan=[]
    def move(phase,arm,p,grip,seconds=2):
        plan.append(dict(phase=phase,seconds=seconds,**{arm:dict(position=p,
            rotation_xyzw=cfg['initial_rotation_xyzw'],grip=grip)}))
    move('approach frame handle','left',[.30,.14,TOP+.10],1.,3)
    move('brace frame','left',[.30,.14,TOP+.10],0.,1)
    move('approach loaded ball','right',[.26,-.05,.92],1.,3)
    move('lower around ball','right',[.26,-.05,TOP+.13],1.,2)
    move('grasp ball against pouch','right',[.26,-.05,TOP+.13],.35,1)
    move('draw elastic pouch','right',[.11,-.05,TOP+.13],.35,3)
    plan.append(dict(phase='hold draw',seconds=1))
    move('release shot','right',[.11,-.05,TOP+.13],1.,.2)
    move('withdraw right hand','right',[.11,-.15,.92],1.,2)
    plan.append(dict(phase='wait for towers to settle',seconds=5))
    direct=copy.deepcopy([stage for stage in plan if 'left' not in stage])
    data={'geometry.json':assets,'pilot.json':cfg,'trajectory.json':dict(
        recovery=plan,direct=direct,hold=[dict(phase='idle baseline',seconds=6)])}
    for name,value in data.items():
        (ROOT/name).write_text(json.dumps(value,indent=2)+'\n')
    print('Generated slingshot; scripted controls:',sum(round(s['seconds']*20) for s in plan))


if __name__=='__main__':
    build()
