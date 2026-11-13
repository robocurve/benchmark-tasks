"""Export original embedded USD props using the provisioned native Isaac environment."""
import argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);args=p.parse_args()
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True).app
try:
    from task import create_assets
    args.output.mkdir(parents=True,exist_ok=False)
    print(create_assets(args.output))
finally:
    app.close()
