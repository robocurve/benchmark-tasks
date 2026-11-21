"""Protocol verification with synthetic independent instrument inputs, not physics evidence."""
import pathlib,sys,types,os,tempfile,atexit,shutil
_scratch=tempfile.mkdtemp(prefix="beam-protocol-");os.environ["BEAM_RECORD_DIR"]=_scratch;os.environ["BEAM_STATUS_PATH"]=_scratch+"/runtime-status.json";atexit.register(shutil.rmtree,_scratch)
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import task
class Fake:
 t=0.
 def states(self):return self.objects
s=Fake();inp={'f':0.,'support':0.,'hands':np.array([[1.,1.,1.]])}
task.hand_state=lambda sim:[inp['hands']]
task.forces=lambda sim:(inp['f'],inp['support'] if inp.get('one_support') else inp['support']/2,0. if inp.get('one_support') else inp['support']/2,inp.get('arm_force',0.))
task.render_displays=lambda *args:None

def tick(delta=0.,f=0.,support=.5,pos=None,hand=None,n=1,one_support=False,arm_force=0.,yaw=0.):
 inp.update(f=f,support=support,one_support=one_support,arm_force=arm_force,hands=np.array([[1.,1.,1.]]) if hand is None else np.array([hand]))
 for i in range(n):
  beam=task.BEAM.state(pos=(task.X,0,task.BEAM_Z) if pos is None else pos,joints={'bend':delta/1000},quat_xyzw=(0,0,np.sin(yaw/2),np.cos(yaw/2)))
  s.objects={o.name:o.state() for o in task.OBJECTS};s.objects['polymer_beam']=beam;s.t+=.05;task.step(s)
 return task.score(s.objects)
def reset():
 task.reset_state();s.t=0.;tick()
def eq(name,value,want):
 assert abs(value-want)<1e-8,(name,value,want,task._state)
 print('PASS',name,value)
def first():return tick(1.,14.,14.5,n=23)
def both():
 first();tick(0.,0.,.5,n=2);return tick(2.,28.,28.5,n=23)
reset();eq('released placement after1s',tick(n=21),.25)
reset();eq('short hold not credited',tick(1.,14.,14.5,n=18),0.)
reset();eq('independent first reading14N, not nominal17.78N',first(),.5)
eq('no second record without intermediate unload',tick(2.,28.,28.5,n=23),.5)
tick(0.,0.,.5,n=2);eq('ordered second reading',tick(2.,28.,28.5,n=23),.75)
eq('rack short hold',tick(pos=task.RACK_A,n=35),.75)
eq('rack released2s',tick(pos=task.RACK_A,n=7),1.)
reset();eq('2mm before1mm earns placement only',tick(2.,28.,28.5,n=23),.25)
reset();first();eq('arm touches loaded beam',tick(1.,14.,14.5,hand=(task.X,0,task.BEAM_Z)),0.);eq('violation persists after recovery',tick(n=30),0.)
reset();first();eq('missing support under load',tick(1.,14.,0.),0.)
reset();first();eq('slip1.1mm',tick(1.,14.,14.5,pos=(task.X+.0011,0,task.BEAM_Z)),0.)
reset();eq('overbend4.1mm',tick(4.1,50.,50.5),0.)
reset();eq('off table',tick(pos=(task.X,0,task.TOP-.03)),0.)
reset();both();eq('residual deflection0.3mm blocks completion',tick(.3,pos=task.RACK_A,n=50),.75)
reset();both();eq('hand30mm exclusion',tick(pos=task.RACK_A,hand=(task.RACK_A[0],task.RACK_A[1],task.RACK_A[2]+.015),n=50),.75)
reset();first();eq('out of support bounds',tick(1.,14.,14.5,pos=(task.X+.004,0,task.BEAM_Z)),0.)
reset();eq('window lower limit0.8mm inclusive',tick(.8,14.,14.5,n=23),.5)
reset();eq('window upper limit1.2mm inclusive',tick(1.2,14.,14.5,n=23),.5)
reset();eq('outside window0.799mm',tick(.799,14.,14.5,n=23),.25)
reset();eq('outside window1.201mm',tick(1.201,14.,14.5,n=23),.25)
reset();eq('centering2mm inclusive',tick(pos=(task.X+.002,0,task.BEAM_Z),n=23),.25)
reset();eq('centering2.001mm excluded',tick(pos=(task.X+.002001,0,task.BEAM_Z),n=23),0.)
reset();first();eq('slip1mm inclusive',tick(1.,14.,14.5,pos=(task.X+.001,0,task.BEAM_Z)),.5)
reset();eq('bend4mm inclusive',tick(4.,50.,50.5),0.);assert not task._state['invalid'],'4 mm must not latch'
reset();first();tick(0.,0.,.5,n=2);eq('lost support in intermediate unload',tick(pos=task.RACK_A),0.)
reset();first();tick(0.,0.,.5,n=2);eq('slip during intermediate unload remains invalid',tick(0.,0.,.5,pos=(task.X+.0011,0,task.BEAM_Z)),0.)
reset();both();eq('loaded hand contact outside support bounds',tick(2.,28.,28.5,pos=(task.X+.004,0,task.BEAM_Z),hand=(task.X+.004,0,task.BEAM_Z)),0.)
reset();eq('50N upper boundary allowed',tick(3.9,50.,50.5),0.);assert not task._state['invalid'],'50 N must not latch';eq('50.001N excursion latches zero',tick(3.9,50.001,50.5),0.);assert task._state['invalid']
reset();eq('90degree beam cannot earn placement',tick(yaw=np.pi/2,n=25),0.)
reset();eq('one roller only cannot earn placement',tick(one_support=True,n=25),0.)
reset();first();eq('one roller lost while loaded',tick(1.,14.,14.5,one_support=True),0.)
reset();tick(n=25);eq('native arm loading latches zero even when geometric hand cloud is clear',tick(1.,0.,20.,arm_force=19.5),0.)
reset();eq('2mm diagonal centering limit is Euclidean',tick(pos=(task.X+.0015,.0015,task.BEAM_Z),n=25),0.)
reset();first();eq('both contacts lost during zero-force intermediate unload',tick(0.,0.,0.),0.);eq('contact recovery cannot restore sequence',tick(2.,28.,28.5,n=23),0.)
reset();first();eq('one contact lost during zero-force intermediate unload',tick(0.,0.,.5,one_support=True),0.);eq('single-contact recovery cannot restore sequence',tick(2.,28.,28.5,n=23),0.)
reset();first();eq('half-mm lift with lost contacts during unload',tick(0.,0.,0.,pos=(task.X,0,task.BEAM_Z+.0005)),0.)
reset();eq('arm loading before placement hold is latched',tick(0.,0.,20.,arm_force=19.5),0.);assert task._state['invalid'];eq('pre-hold arm loading cannot recover',first(),0.)
def slide(n,rate):
 sc=0.
 for i in range(n):sc=tick(pos=(task.X-.0012+rate*.05*i,0,task.BEAM_Z))
 return sc
reset();eq('bar turned end for end still counts as placed',tick(yaw=np.pi,n=25),.25)
reset();eq('beam sliding 2.4 mm/s between frames earns no placement',slide(25,.0024),0.)
reset();eq('1.6 mm/s frame-to-frame motion is under the 2 mm/s stillness limit',slide(25,.0016),.25)
reset();both();eq('return to the near cradle also completes',tick(pos=task.RACK_B,n=42),1.)
print('All synthetic protocol checks passed. These are scorer checks, not robot demonstrations.')
