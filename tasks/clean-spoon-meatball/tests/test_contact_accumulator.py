import unittest
import os
import numpy as np
try:
    import warp as wp
except ImportError:
    wp=None


@unittest.skipIf(wp is None,'Run in the supplied Isaac environment for Warp')
class ContactAccumulatorTests(unittest.TestCase):
    def test_substep_counts_match_reference_and_detect_overflow(self):
        from contact_accumulator import accumulate
        rng=np.random.default_rng(42)
        counts=rng.integers(0,6,size=(6,9),dtype=np.int32)
        starts=np.arange(54,dtype=np.int32).reshape(6,9)*5
        distances=rng.choice(np.array([-.002,0,.0004,.0005,.0006,.003],np.float32),270)
        starts[-1,-1]=271;counts[-1,-1]=1
        expected=np.zeros((6,10),np.int32)
        for i,j in np.ndindex(counts.shape):
            n=int(counts[i,j]);a=int(starts[i,j])
            if not n:continue
            if a+n>len(distances):expected[i,-1]=1
            elif np.any(distances[a:a+n]<=np.float32(.0005)):expected[i,j]=3
        device='cuda:0' if os.environ.get('YAM_TEST_GPU')=='1' else 'cpu'
        values=[wp.array(x,device=device) for x in (counts,starts,distances)]
        totals=wp.zeros((6,10),dtype=wp.int32,device=device)
        for _ in range(3):wp.launch(accumulate,dim=(6,9),inputs=[*values,totals],device=device)
        np.testing.assert_array_equal(totals.numpy(),expected)
