"""CPU-only integrity and reset checks for time-only playback."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from clock_policy import Clock

class ClockTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.folder=Path(self.tmp.name)
  self.actions=np.zeros((3,14));self.actions[:,0]=[.1,.2,.3]
  self.meta=dict(action_count=3,total_steps=5,control_hz=20,action_lower=[-1]*14,action_upper=[1]*14)
  self.freeze()
 def freeze(self):
  data=json.dumps(self.actions.tolist()).encode();meta=json.dumps(self.meta).encode()
  (self.folder/'clock_actions.json').write_bytes(data);(self.folder/'clock_metadata.json').write_bytes(meta)
  (self.folder/'clock_manifest.py').write_text('DATA_SHA256='+repr(hashlib.sha256(data).hexdigest())+'\nMETADATA_SHA256='+repr(hashlib.sha256(meta).hexdigest())+'\n')
 def test_sequence_terminal_hold_and_copied_actions(self):
  c=Clock(self.folder)
  for i in range(5):
   a,stop=c.command(i/20);self.assertAlmostEqual(a[0],self.actions[min(i,2),0]);self.assertEqual(stop,i==4)
   a[:]=99
  self.assertAlmostEqual(c.actions[0,0],.1)
 def test_reset_and_repeat(self):
  c=Clock(self.folder);c.command(0);c.command(0);c.command(.05);c.reset();c.command(0)
 def test_fresh_nonzero_rejected(self):
  with self.assertRaises(ValueError):Clock(self.folder).command(.05)
 def test_skip_reverse_and_bad_times(self):
  for t in [.1,-.05,.001,float('nan'),float('inf'),True]:
   c=Clock(self.folder);c.command(0)
   with self.assertRaises(ValueError):c.command(t)
  c=Clock(self.folder);c.command(0);c.command(.05)
  with self.assertRaises(ValueError):c.command(0)
 def test_hash_tampering(self):
  (self.folder/'clock_actions.json').write_text('[]')
  with self.assertRaises(ValueError):Clock(self.folder)
 def test_invalid_dataset(self):
  for value in [float('nan'),2.]:
   self.actions[0,0]=value;self.freeze()
   with self.assertRaises(ValueError):Clock(self.folder)
 def test_invalid_protocol(self):
  for key,value in [('total_steps',2401),('action_count',2),('control_hz',10)]:
   old=self.meta[key];self.meta[key]=value;self.freeze()
   with self.assertRaises(ValueError):Clock(self.folder)
   self.meta[key]=old
 def test_manifest_is_data_only(self):
  for text in ["raise RuntimeError()", "DATA_SHA256='a'*64", "DATA_SHA256='a'", ""]:
   (self.folder/'clock_manifest.py').write_text(text)
   with self.assertRaises((ValueError,TypeError)):Clock(self.folder)

class ExportTests(unittest.TestCase):
 def test_failed_teacher_requires_diagnostic_label(self):
  from export_clock import export
  with tempfile.TemporaryDirectory() as root:
   capture=Path(root)/'capture';capture.mkdir();out=Path(root)/'out'
   (capture/'result.json').write_text(json.dumps({'task_score':.25}))
   with self.assertRaisesRegex(ValueError,'teacher did not complete'):export(capture,out)
 def test_success_export_and_missing_control_rejection(self):
  from export_clock import export
  from yam_arena import ik
  with tempfile.TemporaryDirectory() as root:
   capture=Path(root)/'capture';capture.mkdir();out=Path(root)/'out'
   action=np.r_[(ik.LOWER+ik.UPPER)/2,.5,(ik.LOWER+ik.UPPER)/2,.5].tolist()
   (capture/'result.json').write_text(json.dumps({'task_score':1.}))
   trace=capture/'trajectory.jsonl';trace.write_text(json.dumps({'time':.05,'action_14d':action})+'\n')
   metadata=export(capture,out);self.assertFalse(metadata['fresh_clock_replay_validated']);self.assertFalse(metadata['diagnostic_only'])
   c=Clock(out);np.testing.assert_allclose(c.command(0)[0],np.asarray(action,dtype=np.float32))
   trace.write_text(json.dumps({'time':.10,'action_14d':action})+'\n')
   with self.assertRaisesRegex(ValueError,'control interval'):export(capture,Path(root)/'bad')

if __name__=='__main__':unittest.main()
