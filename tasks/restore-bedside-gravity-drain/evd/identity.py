from hashlib import sha256
from pathlib import Path

from evd.layout import ROOT


def digest(path):
    h = sha256()
    with Path(path).open("rb") as f:
        for data in iter(lambda: f.read(1024 * 1024), b""):
            h.update(data)
    return h.hexdigest()


def source_id():
    paths = [
        ROOT / "task.py",
        ROOT / "checks/scorer.py",
        *sorted((ROOT / "evd").glob("*.py")),
        *sorted((ROOT / "assets").glob("*")),
    ]
    return sha256(
        "".join(f"{p.relative_to(ROOT)} {digest(p)}\n" for p in paths).encode()
    ).hexdigest()
