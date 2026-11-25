"""Geometric checks for the continuous floor and real mounting cutouts."""
from collections import Counter
import unittest
import numpy as np
from floor_geometry import panel_mesh


class FloorMeshTests(unittest.TestCase):
    def setUp(self):
        self.vertices,self.faces=panel_mesh((.42,0,.7525),(.8,1.1,.01))
        self.v=np.asarray(self.vertices)

    def test_closed_manifold_and_outward_volume(self):
        edges=Counter((a,b) for f in self.faces for a,b in zip(f,f[1:]+f[:1]))
        for (a,b),count in edges.items():
            self.assertEqual(count,1)
            self.assertEqual(edges[b,a],1)
        triangles=self.v[np.asarray(self.faces)]
        volume=np.sum(np.einsum('ij,ij->i',triangles[:,0],np.cross(triangles[:,1],triangles[:,2])))/6
        expected=(.8*1.1-2*.08*.22)*.01
        self.assertAlmostEqual(volume,expected,12)

    def test_no_interior_wall_and_cutouts_remain_open(self):
        triangles=self.v[np.asarray(self.faces)]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        for t,n in zip(triangles,normals):
            center=t.mean(0)+[.42,0,.7525]
            self.assertGreater(np.linalg.norm(n),1e-8)
            if abs(n[2])<1e-12 and abs(center[0]-.1)<1e-10:
                self.assertTrue(.195<abs(center[1])<.415)
        # The top triangles cover the intended panel area, with no duplicate faces.
        top=normals[:,2]>0
        self.assertAlmostEqual(normals[top,2].sum()/2,.8*1.1-2*.08*.22,12)
        centers=triangles[top].mean(1)+[.42,0,.7525]
        self.assertFalse(any((c[0]<.1 and .195<abs(c[1])<.415) for c in centers))


if __name__=='__main__':unittest.main()
