import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
if str(ROOT / "tools/stubs") not in sys.path:       # last: a real Inspect Robots, where installed, comes first
    sys.path.append(str(ROOT / "tools/stubs"))
