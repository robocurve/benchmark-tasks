import sys as _sys
from pathlib import Path as _Path

# Import normally so dataclasses can resolve their module in sys.modules.
_sys.path.insert(0, str(_Path(__file__).resolve().parent))

from task import *
from task import __doc__
