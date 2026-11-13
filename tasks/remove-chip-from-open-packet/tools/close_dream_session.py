"""Preserve the real bridge summary if the simulator worker skipped atexit."""
import argparse
import json
from pathlib import Path
import urllib.request


def close(run):
    destination = run / 'dream_session.json'
    if destination.exists():
        return
    episode = json.loads((run / 'dream_episode.json').read_text())

    def post(route):
        request = urllib.request.Request(
            episode['server_url'].rstrip('/') + '/' + route,
            data=json.dumps({'episode': episode['episode']}).encode(),
            headers={'Content-Type': 'application/json'},
        )
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)

    summary = post('summary')
    summary['summary_capture'] = 'Explicit post-run capture from the live bridge after the simulator worker exited.'
    destination.write_text(json.dumps(summary, indent=2) + '\n')
    post('close')
    print(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    close(parser.parse_args().run)
