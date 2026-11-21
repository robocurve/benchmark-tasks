"""Install only the robot meshes required by the pinned ABC station scene."""

import argparse
from hashlib import sha256
from io import BytesIO
from pathlib import Path
import tarfile
from urllib.request import urlopen
from xml.etree import ElementTree as ET

MODEL_DIR = Path(__file__).resolve().parent / "assets/abc_sim/models"
ARCHIVE = "https://abc-data.timehorizons.org/sim_assets/i2rt_yam.tar.gz"
ARCHIVE_BYTES = 4566683
DIGEST = "ed5144c0b4ae55da75a3701b5bad9ca3599977cc8b73c59996e5e1633749aad7"


def install(payload, model_dir):
    if len(payload) != ARCHIVE_BYTES or sha256(payload).hexdigest() != DIGEST:
        raise ValueError("Official asset archive size or SHA-256 does not match the pinned manifest")
    model = ET.parse(model_dir / "yam_bimanual_empty.xml")
    mesh_files = [mesh.attrib["file"] for mesh in model.findall("asset/mesh") if "file" in mesh.attrib]
    mesh_dir = model_dir / "assets/i2rt_yam/assets"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=BytesIO(payload), mode="r:gz") as archive:
        for filename in mesh_files:
            source = archive.getmember(f"i2rt_yam/assets/{filename}")
            if not source.isfile():
                raise ValueError(f"Expected a regular mesh file for {filename}")
            with archive.extractfile(source) as mesh:
                (mesh_dir / filename).write_bytes(mesh.read())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="Use a local copy of the official archive")
    args = parser.parse_args()
    if args.archive is not None:
        payload = args.archive.read_bytes()
    else:
        with urlopen(ARCHIVE, timeout=30) as response:
            payload = response.read(ARCHIVE_BYTES + 1)
    install(payload, MODEL_DIR)


if __name__ == "__main__":
    main()
