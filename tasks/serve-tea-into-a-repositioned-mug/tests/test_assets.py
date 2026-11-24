"""Catch uncredited mesh substitutions and dependencies on host scene assets."""
from hashlib import sha256
import unittest
from xml.etree import ElementTree

from tea.scene import ASSETS, build


class AssetInventoryTests(unittest.TestCase):
    def test_inventory_covers_upstream_files_in_scene(self):
        checks = (ASSETS / "UPSTREAM_SHA256SUMS").read_text().splitlines()
        digests = {}
        for line in checks:
            digest, name = line.split("  ")
            digests[name] = digest
        scene = ElementTree.fromstring(build())
        expected_meshes = {el.attrib["file"] for el in scene.iter("mesh")}
        self.assertEqual(len(expected_meshes), 9)
        self.assertEqual(expected_meshes, {p.name for p in ASSETS.glob("*.stl")})
        self.assertEqual(set(digests), expected_meshes | {"yam.urdf", "LICENSE.i2rt"})
        self.assertEqual(len(checks), len(digests))
        for name, digest in digests.items():
            with self.subTest(name=name):
                self.assertEqual(sha256((ASSETS / name).read_bytes()).hexdigest(), digest)
        external_filenames = {e.attrib["file"] for e in scene.iter() if "file" in e.attrib}
        self.assertEqual(external_filenames, expected_meshes)
        self.assertEqual((ASSETS / "SOURCE_COMMIT").read_text().strip(),
                         "120c3c81400171174604e503943f8d1ebc891058")


if __name__ == "__main__":
    unittest.main()
