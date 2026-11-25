"""Evidence views must never become policy images or mutate the observation."""
import os
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import patch
import numpy as np
from observer import EVIDENCE_VIEWS, camera_metadata, recording_images


class ArrayTensor:
    def __init__(self,array):self.array=array
    def __getitem__(self,key):return ArrayTensor(self.array[key])
    def cpu(self):return self
    def numpy(self):return self.array


class ObserverTests(TestCase):
    def test_recording_adds_views_without_changing_policy_input(self):
        native={name:np.zeros((176,320,3),dtype=np.uint8)
                for name in ('top_cam','left_cam','right_cam')}
        obs=SimpleNamespace(images=native)
        raw=np.full((1,352,640,4),127,dtype=np.uint8)
        scene={name:SimpleNamespace(data=SimpleNamespace(output={'rgb':ArrayTensor(raw)}))
               for name in EVIDENCE_VIEWS}
        emb=SimpleNamespace(_base=SimpleNamespace(scene=scene))
        with patch.dict(os.environ,{'FURNITURE_SLIDERS_OBSERVER':'1'}):
            images=recording_images(emb,obs)
        self.assertEqual(set(obs.images),{'top_cam','left_cam','right_cam'})
        self.assertEqual(set(images),set(native)|set(EVIDENCE_VIEWS))
        for name in native:self.assertIs(images[name],native[name])
        for name in EVIDENCE_VIEWS:
            self.assertEqual(images[name].shape,(352,640,3))
            self.assertTrue(images[name].flags.c_contiguous)
        self.assertFalse(camera_metadata()['policy_inputs'])

    def test_disabled_observer_never_reads_scene(self):
        obs=SimpleNamespace(images={'top_cam':object()})
        with patch.dict(os.environ,{'FURNITURE_SLIDERS_OBSERVER':'0'}):
            images=recording_images(None,obs)
        self.assertEqual(images,obs.images)
        self.assertIsNot(images,obs.images)


if __name__=='__main__':main()
