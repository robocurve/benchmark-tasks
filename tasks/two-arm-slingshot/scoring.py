"""State rubric only; synthetic checks do not demonstrate native feasibility."""
import math
import numpy as np


class SlingshotScore:
    def __init__(self,cfg):
        self.cfg=cfg
        self.unsafe=False
        self.drawn=False
        self.launched=False
        self.previous_ball=None
        self.previous_forward_speed=0.
        self.stable=[0,0]

    def evaluate(self,states,hands,grips):
        expected={'launcher/frame','launcher/pouch','ball',
                  *(f'tower{t}_{i}' for t in (1,2) for i in range(3))}
        try:
            valid=set(states)==expected and np.shape(hands)==(2,3) and np.shape(grips)==(2,)
            valid &= np.isfinite(hands).all() and np.isfinite(grips).all()
            valid &= all(-.05<=g<=1.05 for g in grips)
            for p,r,v in states.values():
                p,r,v=map(np.asarray,(p,r,v))
                valid &= p.shape==(3,) and r.shape==(3,3) and v.shape==(6,)
                valid &= np.isfinite(p).all() and np.isfinite(r).all() and np.isfinite(v).all()
                valid &= np.allclose(r.T@r,np.eye(3),atol=.001) and abs(np.linalg.det(r)-1)<.001
        except (TypeError,ValueError):
            valid=False
        if not valid:
            self.unsafe=True
            return dict(score=0.,success=False,unsafe_event=True,invalid_state=True)
        top=self.cfg['table_top']
        fp,fr,_=map(np.asarray,states['launcher/frame'])
        pp,_,_=map(np.asarray,states['launcher/pouch'])
        bp,_,bv=map(np.asarray,states['ball'])
        frame_error=float(np.linalg.norm(fp-self.cfg['poses']['launcher']))
        draw=float(self.cfg['pouch_rest'][0]-(fr.T@(pp-fp))[0])
        handle=fp+fr@self.cfg['handle_offset']
        braced=np.linalg.norm(hands[0]-handle)<.04 and grips[0]<.4
        self.unsafe |= frame_error>.045 or fr[2,2]<math.cos(math.radians(15))
        self.unsafe |= any(h[0]>.55 for h in hands)
        for p,_,_ in states.values():
            self.unsafe |= not (-.12<p[0]<1.22 and abs(p[1])<.27 and top-.04<p[2]<1.5)
        if draw>.10 and not self.launched:
            self.unsafe |= not braced
            self.drawn |= braced and grips[1]<.8 and np.linalg.norm(hands[1]-bp)<.05
        crossed=self.previous_ball is not None and self.previous_ball[0]<.53<=bp[0]
        if crossed and not self.launched:
            # A collision can reverse velocity within the same 20 Hz crossing sample.
            forward_speed=max(self.previous_forward_speed,float(bv[0]))
            self.launched=bool(self.drawn and braced and grips[1]>.8 and forward_speed>.5
                               and np.linalg.norm(hands[1]-bp)>.12)
            self.unsafe |= not self.launched
        self.previous_ball=bp.copy()
        self.previous_forward_speed=float(bv[0])
        fallen=[]
        for index,t in enumerate((1,2)):
            name=f'tower{t}_2';p,r,v=map(np.asarray,states[name])
            moved=np.linalg.norm(p-self.cfg['poses'][name])>.015
            self.unsafe |= moved and not self.launched
            down=p[2]<top+.065
            quiet=all(np.linalg.norm(states[f'tower{t}_{i}'][2][:3])<.04 and
                      np.linalg.norm(states[f'tower{t}_{i}'][2][3:])<.3 for i in range(3))
            self.stable[index]=self.stable[index]+1 if down and quiet and self.launched else 0
            fallen.append(self.stable[index]>=20)
        score=0. if self.unsafe else sum(fallen)/2
        return dict(score=score,success=score==1.,unsafe_event=bool(self.unsafe),
                    drawn=bool(self.drawn),launched=self.launched,draw_m=draw,
                    frame_error_m=frame_error,braced=bool(braced),towers_down=sum(fallen),
                    right_opening=float(grips[1]),ball_forward_speed_m_s=float(bv[0]))
