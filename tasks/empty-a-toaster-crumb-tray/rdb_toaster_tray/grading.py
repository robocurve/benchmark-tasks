"""Score visible piece counts and the external position of the released tray pull."""
from dataclasses import dataclass
import numpy as np
from . import layout as L

@dataclass
class ToasterGrade:
 sample_hz:float=240.
 void:str|None=None
 withdrawn:bool=False
 emptied:bool=False
 stable_samples:int=0
 def update(self,poses,velocities,*,robot_tray_contacts,robot_toaster_contacts,robot_crumb_contacts):
  p=np.asarray(poses,dtype=float);v=np.asarray(velocities,dtype=float)
  if p.shape!=(len(L.LINKS),7) or v.shape!=(len(L.LINKS),6) or not np.isfinite(p).all() or not np.isfinite(v).all():
   self.void='invalid native body state';return self.result(0.)
  if robot_crumb_contacts:self.void='a robot directly touched bread crumbs'
  rt=L.quat_matrix(p[L.TRAY,3:]);rb=L.quat_matrix(p[L.TOASTER,3:]);tray=p[L.TRAY,:3]
  if np.any(p[:,2]<-.010):self.void='a task object fell off the worktop'
  # The unplugged appliance is an ordinary free rigid body on rubber feet.
  # It may be steadied by hand; moving the housing to dump its contents is not
  # the requested removable-tray cleaning operation.
  housing_shift=float(np.linalg.norm(p[L.TOASTER,:3]-L.TOASTER_CENTER))
  if housing_shift>.020 or rb[2,2]<np.cos(np.deg2rad(10.)):
   self.void='the toaster was displaced or tipped instead of removing its tray'
  floor=p[L.TRAY,:3]+L.box_corners((L.TRAY_SIZE[0],L.TRAY_SIZE[1],L.TRAY_WALL))@rt.T
  floor_in_body=(floor-p[L.TOASTER,:3])@rb
  if float(floor_in_body[:,1].min())>L.TOASTER_FRONT+.005:self.withdrawn=True
  # Count pieces at the scoring observation. Crossing a rim in flight has no penalty.
  centers = p[L.CRUMBS, :3]
  bin_local = centers - L.BIN_CENTER
  in_bin = (np.abs(bin_local[:, :2]) <= L.BIN_INNER / 2).all(axis=1)
  in_bin &= (bin_local[:, 2] >= L.BIN_FLOOR) & (bin_local[:, 2] < L.BIN_RIM)
  tray_local = (centers - tray) @ rt
  in_tray = (np.abs(tray_local[:, :2]) <= L.TRAY_SIZE[:2] / 2).all(axis=1)
  in_tray &= (tray_local[:, 2] >= 0) & (tray_local[:, 2] <= L.TRAY_WALL)
  # A loaded tray dipped into the bin has not been emptied.
  in_bin &= ~in_tray
  all_bin = bool(in_bin.all())
  accounted = bool((in_tray | in_bin).all())
  if self.withdrawn and all_bin:
   self.emptied = True

  # Compare the visible left/right ends of the pull with a housing-fixed home photo.
  # Checking the floor behind the fascia and the upright pull rejects reversed trays.
  pull_ends = L.HANDLE_LOCAL + np.array([[-.022, 0, 0], [.022, 0, 0]])
  observed = (tray + pull_ends @ rt.T - p[L.TOASTER, :3]) @ rb
  home = pull_ends + np.array([0, 0, L.TRAY_CENTER[2]])
  pull_error = float(np.abs(observed - home).max())
  seated = bool(pull_error <= .005 and rt[:, 2] @ rb[:, 2] > 0
                and floor_in_body[:, 1].max() <= L.TOASTER_FRONT + .005)
  free = not (robot_tray_contacts or robot_toaster_contacts or robot_crumb_contacts)
  if not self.void and self.withdrawn and self.emptied and all_bin and seated and free:
   self.stable_samples += 1
  else:
   self.stable_samples = 0
  seconds = max(0, self.stable_samples - 1) / self.sample_hz
  score = 0.
  if not self.void and self.withdrawn and accounted:
   score = 1. if seconds >= 1. else .6 if all_bin else .25
  return self.result(score,tray_position_m=tray.tolist(),housing_shift_m=housing_shift,
                     crumbs_in_tray=int(in_tray.sum()),crumbs_in_bin=int(in_bin.sum()),
                     pieces_unaccounted=int((~(in_tray | in_bin)).sum()),pull_error_mm=1000*pull_error,
                     tray_seated=seated,released=free,stable_seconds=seconds)
 def result(self,score,**values):return dict(score=float(score),finished=self.void is not None or score==1.,void=self.void,withdrawn=self.withdrawn,emptied=self.emptied,stable_samples=self.stable_samples,sample_hz=self.sample_hz,**values)

@dataclass(frozen=True)
class ToasterScorer:
 name:str='toaster_tray_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No native task steps')
  g=record.steps[-1].result.info['toaster_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return ToasterScorer()
