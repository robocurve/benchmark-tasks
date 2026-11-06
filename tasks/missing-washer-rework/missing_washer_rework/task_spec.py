# SPDX-License-Identifier: MIT
"""Repair the capped blue fixture to match the complete fixed reference.

All pieces are lightweight rigid plastic. The orange square spacer has a real
square opening. The grey protective cap is open underneath and gravity-seated:
it is neither threaded nor a simulated elastic snap. The blue target fixture
is loose and can be stabilized by its raised handle. The reference is fixed.
"""
from pathlib import Path
from yam_arena.task_file import Box
from missing_washer_rework.contract import *
SCENE = 'station'
SUCCESS_HOLD_S = 1.5
SCORE_OVER = 'end'
ASSETS = Path(__file__).parent/'assets'
COMPOUND_ASSETS = {name: str(ASSETS/(kind+'.usda')) for name,kind in {
 'fixture':'fixture','washer':'washer','cap':'cap',
 'reference_fixture':'reference_fixture','reference_washer':'washer','reference_cap':'cap'}.items()}
FIXED_ASSETS = REFERENCE_NAMES
OBJECTS = [
 Box('fixture',pos=FIXTURE_POS,size=FIXTURE_SIZE,mass=.30,color=(.08,.32,.82)),
 Box('cap',pos=CAP_INITIAL,size=CAP_SIZE,mass=.045,color=(.52,.55,.60)),
 Box('washer',pos=WASHER_INITIAL,size=WASHER_SIZE,mass=.035,color=(1.,.32,.03)),
 Box('reference_fixture',pos=REFERENCE_POS,size=FIXTURE_SIZE,color=(.20,.24,.30)),
 Box('reference_washer',pos=(REFERENCE_POS[0],REFERENCE_POS[1],REFERENCE_POS[2]+.015),size=WASHER_SIZE,color=(1.,.32,.03)),
 Box('reference_cap',pos=(REFERENCE_POS[0],REFERENCE_POS[1],REFERENCE_POS[2]+.022),size=CAP_SIZE,color=(.52,.55,.60)),
 Box('parking_pad',pos=PARK_POS,size=PARK_SIZE,color=(.18,.68,.35)),
 Box('washer_pad',pos=(WASHER_INITIAL[0],WASHER_INITIAL[1],TABLE_TOP-.002),size=(.09,.09,.004),color=(.85,.84,.78)),
]
def success(objects):
 return False
def score(objects):
 return 0.0
