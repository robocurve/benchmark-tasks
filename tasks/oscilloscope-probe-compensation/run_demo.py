#!/usr/bin/env python3
"""Run this folder's native task without a sibling repository package."""
from pathlib import Path
import sys


TASK = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK / 'native'))

if __name__ == '__main__':
    from resist_isaac.runner import main
    main(TASK)
