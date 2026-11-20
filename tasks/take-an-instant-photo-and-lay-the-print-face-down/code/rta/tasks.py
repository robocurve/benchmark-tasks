"""Registered Inspect Robots tasks: each memory task file as a ``Task`` of seeded scenes."""

from __future__ import annotations

from pathlib import Path

from inspect_robots.registry import task
from inspect_robots.scene import Scene
from inspect_robots.task import Epochs, Task

from rta import TASKS_DIR, spec
from rta.scoring import milestone_scorers, q_score, rta_success


def memory_task(name: str, task_file: Path, *, num_scenes: int, epochs: int, max_seconds: float) -> Task:
    loaded = spec.load(task_file)
    scenes = [
        Scene(id=f"seed-{i}", instruction=loaded.instruction, init_seed=i, metadata={"task_file": str(loaded.path)})
        for i in range(num_scenes)
    ]
    return Task(
        name=name,
        scenes=scenes,
        scorer=[q_score(), rta_success(), *milestone_scorers(loaded.milestone_names)],
        max_seconds=max_seconds,
        epochs=Epochs(count=epochs, reducer="mean"),
    )


@task("rta/box_retrieval")
def box_retrieval(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Hidden-object retrieval, three identical boxes (chance 1/3)."""
    return memory_task("rta/box_retrieval", TASKS_DIR / "t01_box_retrieval" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/box_retrieval_two")
def box_retrieval_two(num_scenes: int = 10, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Hidden-object retrieval, two identical boxes (chance 1/2)."""
    return memory_task("rta/box_retrieval_two", TASKS_DIR / "t01b_box_retrieval_two" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/remember_sort")
def remember_sort(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Remember-and-sort: a colour cue is shown and removed; hold the bin, put the matching block in (chance 1/4)."""
    return memory_task("rta/remember_sort", TASKS_DIR / "t02_remember_sort" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/button_sequence")
def button_sequence(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """Interrupted sequence: reproduce a demonstrated order of four button presses with the matching arms (chance 1/24)."""
    return memory_task("rta/button_sequence", TASKS_DIR / "t03_button_sequence" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/lockout")
def lockout(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Lockout: press START only on the machines nobody is inside (safety; chance 1/4)."""
    return memory_task("rta/lockout", TASKS_DIR / "s01_lockout" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/who_ordered")
def who_ordered(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """Serve each drink to the place whose order token had its colour (jobs; chance 1/6)."""
    return memory_task("rta/who_ordered", TASKS_DIR / "j03_who_ordered" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/incubation_order")
def incubation_order(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Move the two longest-incubated tubes to the centrifuge, oldest first (jobs; chance 1/12)."""
    return memory_task("rta/incubation_order", TASKS_DIR / "j04_incubation_order" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/hot_hob")
def hot_hob(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Plastic box on the cold hob plate, milk pan on a hot one (safety; chance 1/4; burns zero the score)."""
    return memory_task("rta/hot_hob", TASKS_DIR / "s03_hot_hob" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/teach_points")
def teach_points(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Put the puck on the first taught point and the pin on the second (robot building; continuous memory)."""
    return memory_task("rta/teach_points", TASKS_DIR / "a05_teach_points" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/sprayed_lanes")
def sprayed_lanes(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 100.0) -> Task:
    """Lay four instruments on the drying mat, avoiding the two sprayed lanes (safety; chance 1/6)."""
    return memory_task("rta/sprayed_lanes", TASKS_DIR / "s04_sprayed_lanes" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/empty_cans")
def empty_cans(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 100.0) -> Task:
    """Empty cans to the recycling bin, full cans to the fridge rack (societal; chance 1/6)."""
    return memory_task("rta/empty_cans", TASKS_DIR / "h04_empty_cans" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/cooling_iron")
def cooling_iron(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 32.0) -> Task:
    """Wait 10 s after the light goes out, then box the soldering iron within 20 s (safety; timing)."""
    return memory_task("rta/cooling_iron", TASKS_DIR / "s05_cooling_iron" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/slide_staining")
def slide_staining(num_scenes: int = 6, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Stain 8-12 s, rinse 4-6 s, rack (jobs; the robot keeps its own time)."""
    return memory_task("rta/slide_staining", TASKS_DIR / "j02_slide_staining" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/memory_module")
def memory_module(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """Pull the module the indicator flagged, tray it, seat the spare (robot building; chance 1/4)."""
    return memory_task("rta/memory_module", TASKS_DIR / "a04_memory_module" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/drive_bays")
def drive_bays(num_scenes: int = 12, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """Return every drive sled to the bay it came from (data centre; chance 1/6)."""
    return memory_task("rta/drive_bays", TASKS_DIR / "a01_drive_bays" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/restock")
def restock(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 45.0) -> Task:
    """Restock the product a customer took from under the hood (jobs; chance 1/4)."""
    return memory_task("rta/restock", TASKS_DIR / "j01_restock" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/dropped_cell")
def dropped_cell(num_scenes: int = 8, epochs: int = 1, max_seconds: float = 80.0) -> Task:
    """Quarantine the dropped battery cell, charge the other three (safety; chance 1/4)."""
    return memory_task("rta/dropped_cell", TASKS_DIR / "s02_dropped_cell" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/fetch_shown")
def fetch_shown(num_scenes: int = 20, epochs: int = 1, max_seconds: float = 80.0) -> Task:
    """Put the three groceries whose packages were shown into the basket (societal; chance 1/20)."""
    return memory_task("rta/fetch_shown", TASKS_DIR / "h05_fetch_shown" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/rejected_gear")
def rejected_gear(num_scenes: int = 7, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """Scrap the gear the inspector rejected and refill the pocket it came from (robot building; chance ~1/14)."""
    return memory_task("rta/rejected_gear", TASKS_DIR / "a03_rejected_gear" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/memory_game")
def memory_game(num_scenes: int = 6, epochs: int = 1, max_seconds: float = 240.0) -> Task:
    """Collect the three token pairs from under the cups, pair by pair (societal; chance 1/15)."""
    return memory_task("rta/memory_game", TASKS_DIR / "h02_memory_game" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/lunch_boxes")
def lunch_boxes(num_scenes: int = 4, epochs: int = 1, max_seconds: float = 200.0) -> Task:
    """One bar in each of four identical closed lunch boxes, one open at a time (societal; own progress)."""
    return memory_task("rta/lunch_boxes", TASKS_DIR / "h03_lunch_boxes" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/desk_restore")
def desk_restore(num_scenes: int = 6, epochs: int = 1, max_seconds: float = 120.0) -> Task:
    """Put each thing from the basket back where it was on the desk (societal; continuous layout memory)."""
    return memory_task("rta/desk_restore", TASKS_DIR / "h01_desk_restore" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/reach_probe")
def reach_probe(num_scenes: int = 1, epochs: int = 1, max_seconds: float = 640.0) -> Task:
    """Reach probe: each arm visits a grid of hover points (scripts/reach_map.py renders docs/REACH.md)."""
    return memory_task("rta/reach_probe", TASKS_DIR / "t04_reach_probe" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/bulb_change")
def bulb_change(num_scenes: int = 4, epochs: int = 1, max_seconds: float = 150.0) -> Task:
    """Switch the lamp off, unscrew the old bulb (2 turns), screw in the new one (2 turns), switch back on (touching a bulb while ON zeroes the score)."""
    return memory_task("rta/bulb_change", TASKS_DIR / "l01_bulb_change" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/bluetooth_pairing")
def bluetooth_pairing(num_scenes: int = 6, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """H06: bluetooth pairing."""
    return memory_task("rta/bluetooth_pairing", TASKS_DIR / "h06_bluetooth_pairing" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/magnet_notice")
def magnet_notice(num_scenes: int = 2, epochs: int = 1, max_seconds: float = 50.0) -> Task:
    """J05: magnet notice."""
    return memory_task("rta/magnet_notice", TASKS_DIR / "j05_magnet_notice" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/dj_mix")
def dj_mix(num_scenes: int = 4, epochs: int = 1, max_seconds: float = 55.0) -> Task:
    """L02: DJ mix (match tempo, start on beat one, crossfade before the track ends)."""
    return memory_task("rta/dj_mix", TASKS_DIR / "l02_dj_mix" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/instant_photo")
def instant_photo(num_scenes: int = 2, epochs: int = 1, max_seconds: float = 60.0) -> Task:
    """H07: instant photo."""
    return memory_task("rta/instant_photo", TASKS_DIR / "h07_instant_photo" / "task.py", num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)


@task("rta/sketch")
def sketch(task_file: str, num_scenes: int = 2, epochs: int = 1, max_seconds: float = 12.0) -> Task:
    """Scene check for a task file under development: spawn, play the observe phase, render (oracle may be a stub)."""
    return memory_task("rta/sketch", Path(task_file), num_scenes=num_scenes, epochs=epochs, max_seconds=max_seconds)
