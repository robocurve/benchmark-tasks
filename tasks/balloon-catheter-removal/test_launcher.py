"""Validate resource ownership and current-GPU selection without Docker changes."""
import json
import subprocess
import unittest
from unittest.mock import patch
import teleop_launcher as launch

class LauncherSafetyTests(unittest.TestCase):
    def test_foreign_container_never_stopped(self):
        labels={'robocurve.uid':str(launch.UID+1),'robocurve.task':launch.HERE.name,'robocurve.mode':'quest'}
        result=subprocess.CompletedProcess([],0,stdout=json.dumps(labels))
        with patch.object(launch.subprocess,'run',return_value=result) as run:
            with self.assertRaisesRegex(RuntimeError,'different ownership'):
                launch.own_stop({'containers':['someone-elses-container'],'output':'/tmp/not-used'})
            self.assertEqual(run.call_count,1)
            self.assertEqual(run.call_args.args[0][1],'inspect')

    def test_gpu_is_chosen_from_live_capacity(self):
        result=subprocess.CompletedProcess([],0,stdout='0, 50000, 99\n1, 8000, 0\n2, 20000, 5\n')
        with patch.object(launch,'call',return_value=result):
            self.assertEqual(launch.select_gpu(None),2)
            with self.assertRaises(RuntimeError):launch.select_gpu(1)

    def test_current_user_ownership_labels(self):
        cmd=launch.base_container('test-only',0)
        self.assertIn('robocurve.uid='+str(launch.UID),cmd)
        self.assertIn('robocurve.task='+launch.HERE.name,cmd)
        self.assertIn('--init',cmd)

if __name__=='__main__':unittest.main()
