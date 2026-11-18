"""Prepare a fresh evaluator-only output directory without starting the simulator."""
from pathlib import Path


def prepare_evidence_directory(task, value):
    task = Path(task).expanduser().resolve()
    directory = Path(value).expanduser().resolve()
    if directory == task or task in directory.parents:
        raise ValueError("Native evidence must be outside the task source folder")
    directory.mkdir(parents=True, exist_ok=False)
    return directory
