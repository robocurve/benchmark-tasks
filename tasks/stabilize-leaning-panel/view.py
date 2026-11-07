"""Interactive panel preview with pause/reset, camera presets and looping demo."""

import argparse
import time
from pathlib import Path

from inspect_robots import Scene
from inspect_robots.types import Action
from run import install
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.scripted import YamScriptedPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo", action="store_true", help="replay panel stabilization on a loop"
    )
    args = parser.parse_args()
    install()
    env = YamArenaEmbodiment(
        task=str(Path(__file__).with_name("task.py")),
        object_jitter=0,
        control_hz=20,
        cam_width=640,
        cam_height=360,
        headless=False,
    )
    scene = Scene(
        id="leaning-panel-preview",
        instruction="Hold the leaning panel, seat the orange stopper, then withdraw.",
    )
    policy = YamScriptedPolicy()
    policy.bind(env.info)
    obs = env.reset(scene, seed=0)
    policy.reset(scene)
    ready = Action(data=[*READY, 1.0, *READY, 1.0])
    import yam_arena.inspect_embodiment as ie
    from omni import ui

    controls = {"paused": False, "reset": False, "demo": args.demo}
    presets = {
        "Front": ((-0.18, -0.01, 1.08), (0.30, 0, 0.87)),
        "Overview": ((-0.50, -0.60, 1.35), (0.28, 0, 0.85)),
        "Close panel": ((0.04, -0.14, 1.03), (0.30, 0, 0.85)),
    }

    def camera(name):
        eye, target = presets[name]
        env._base.sim.set_camera_view(eye=eye, target=target)

    def toggle_pause():
        controls["paused"] = not controls["paused"]

    def reset():
        controls["reset"] = True

    def toggle_demo():
        controls["demo"] = not controls["demo"]
        controls["reset"] = True

    panel = ui.Window("Leaning panel preview", width=480, height=360)
    panel.width = 480
    panel.height = 360
    panel.position_x = 1350
    panel.position_y = 70
    with (
        panel.frame,
        ui.VStack(
            spacing=10, style={"Label": {"font_size": 22}, "Button": {"font_size": 22}}
        ),
    ):
        ui.Label("Panel support / physical foot stopper", height=30)
        status = ui.Label("", height=30)
        ui.Button("Pause / Resume", clicked_fn=toggle_pause, height=44)
        with ui.HStack(spacing=8, height=44):
            ui.Button("Reset layout", clicked_fn=reset)
            ui.Button("Demo / Static", clicked_fn=toggle_demo)
        with ui.HStack(spacing=6, height=44):
            for name in presets:
                ui.Button(name, clicked_fn=lambda name=name: camera(name))
        ui.Label("Scroll: zoom   Alt-drag: orbit", height=30)
        ui.Label("Select object + F: focus", height=30)
    camera("Overview")
    try:
        while ie._APP.is_running():
            start = time.monotonic()
            if controls["reset"]:
                obs = env.reset(scene, seed=0)
                policy.reset(scene)
                controls["reset"] = False
            mode = (
                "Paused"
                if controls["paused"]
                else "Demo running"
                if controls["demo"]
                else "Static layout"
            )
            status.text = f"{mode}  |  {obs.state_time:.1f} s"
            if controls["paused"]:
                # Pump only the app/viewport: no environment or policy time advances.
                ie._APP.update()
                time.sleep(0.02)
                continue
            action = policy.act(obs).actions[0] if controls["demo"] else ready
            result = env.step(action)
            obs = result.observation
            if controls["demo"] and (result.terminated or obs.state_time > 50):
                print(
                    "Preview episode:",
                    result.termination_reason,
                    result.info,
                    flush=True,
                )
                controls["reset"] = True
            time.sleep(max(0, 0.05 - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        panel.visible = False
        env.close()


if __name__ == "__main__":
    main()
