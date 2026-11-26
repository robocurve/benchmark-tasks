from pathlib import Path
import json
ASSET=Path(__file__).parent/'assets'/'anti-backlashnut-preload'
P=json.loads((ASSET/'parameters.json').read_text());E=P['estimates'];U=P['published'];REAR_ZERO=.040515
