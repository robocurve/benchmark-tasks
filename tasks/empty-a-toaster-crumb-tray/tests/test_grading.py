"""Outcome and boundary cases; native motion is validated by complete rollouts."""
import numpy as np
import pytest
from rdb_toaster_tray import layout as L
from rdb_toaster_tray.grading import ToasterGrade

ZERO=np.zeros((len(L.LINKS),6))
def read(g,p,**kw):
 return g.update(p,ZERO,robot_tray_contacts=kw.get('tray',False),robot_toaster_contacts=kw.get('toaster',False),robot_crumb_contacts=kw.get('crumb',False))
def baseline():
 p=L.START_POSES.copy()
 # Place actual chunks on the floor rather than leave reset's 0.6 mm settling gap.
 p[L.CRUMBS,2]-=.0006
 return p

def dumped():
 p=baseline()
 for i,j in enumerate(L.CRUMBS):
  p[j,:3]=L.BIN_CENTER+[(-.095+(i%8)*.027),(-.073+(i//8)*.036),L.BIN_FLOOR+L.CRUMB_SIZES[i,2]/2]
 return p

def complete_prefix(g):
 p=baseline();p[L.TRAY,:3]+=np.array((-.285,0,0))
 p[L.CRUMBS,:3]+=np.array((-.285,0,0))
 r=read(g,p);assert r['withdrawn'] and r['score']==.25
 p=dumped();assert read(g,p)['emptied']
 return p

def test_idle_does_not_earn_cleaning_credit():
 g=ToasterGrade()
 for _ in range(300):r=read(g,baseline())
 assert r['score']==0. and not r['withdrawn'] and not r['finished']

def test_direct_robot_debris_contact_latches_zero():
 g=ToasterGrade();r=read(g,baseline(),crumb=True)
 assert r['score']==0. and r['finished'] and r['void']
 assert read(g,dumped())['score']==0.

def test_one_piece_on_worktop_or_left_in_tray_loses_emptying_credit():
 g=ToasterGrade();p=complete_prefix(g)
 p[L.CRUMBS[-1],:3]=[.44,-.24,.004]
 r=read(g,p);assert r['score']==0. and r['pieces_unaccounted']==1
 p[L.CRUMBS[-1]]=baseline()[L.CRUMBS[-1]]
 assert read(g,p)['score']==.25

def test_falling_piece_can_cross_rim_before_being_caught():
 g=ToasterGrade();p=complete_prefix(g);j=L.CRUMBS[0];original=p[j].copy()
 p[j,:3]=L.BIN_CENTER+[L.BIN_INNER[0]/2+.003,0,L.BIN_RIM+.015]
 r=read(g,p);assert r['score']==0. and r['void'] is None
 p[j]=original
 assert read(g,p)['score']==.6

def test_emptied_tray_left_partly_out_cannot_complete():
 g=ToasterGrade();p=complete_prefix(g);p[L.TRAY,0]-=.010
 for _ in range(300):r=read(g,p)
 assert r['score']==.6 and not r['tray_seated'] and not r['finished']

@pytest.mark.parametrize('sample_hz',[20,240,960])
def test_first_sample_does_not_earn_elapsed_time(sample_hz):
 g=ToasterGrade(sample_hz=sample_hz);p=complete_prefix(g)
 for _ in range(sample_hz-1):r=read(g,p)
 assert r['score']==.6 and r['stable_seconds']<1.
 r=read(g,p);assert r['score']==1. and r['stable_seconds']==1.

def test_final_robot_contact_resets_free_stability_interval():
 g=ToasterGrade();p=complete_prefix(g)
 for _ in range(120):read(g,p)
 r=read(g,p,tray=True);assert r['stable_samples']==0 and r['score']==.6
 for _ in range(240):r=read(g,p)
 assert r['score']==.6
 assert read(g,p)['score']==1.

def test_reversed_tray_is_not_a_valid_reinsertion():
 g=ToasterGrade();p=complete_prefix(g)
 # Same position, upside-down: an ordinary pull handle cannot earn an installed state.
 p[L.TRAY,3:]=[np.sqrt(.5),np.sqrt(.5),0.,0.]
 for _ in range(300):r=read(g,p)
 assert not r['tray_seated'] and r['score']==.6


def test_chunk_inside_bin_touching_inner_wall_is_not_a_spill():
 g=ToasterGrade();p=dumped();i=0;j=L.CRUMBS[i]
 # The chunk is rotated90deg, so authored Y is its world X extent.
 p[j,:3]=L.BIN_CENTER+[L.BIN_INNER[0]/2-L.CRUMB_SIZES[i,1]/2-.00001,0,L.BIN_FLOOR+L.CRUMB_SIZES[i,2]/2]
 r=read(g,p);assert r['void'] is None and r['crumbs_in_bin']==L.N_CRUMBS


def test_visible_home_template_accepts_small_offset_and_rejects_proud_pull():
 g=ToasterGrade();p=complete_prefix(g);p[L.TRAY,0]-=.004
 for _ in range(241):r=read(g,p)
 assert r['score']==1.
 p[L.TRAY,0]-=.003
 assert read(g,p)['score']==.6

def test_submerging_loaded_tray_in_bin_is_not_emptying():
 g=ToasterGrade();p=complete_prefix(g)
 p=baseline();offset=np.array([*L.BIN_CENTER[:2],.04])-p[L.TRAY,:3]
 p[L.TRAY,:3]+=offset;p[L.CRUMBS,:3]+=offset
 r=read(g,p);assert r['crumbs_in_tray']==40 and r['crumbs_in_bin']==0 and r['score']==.25
