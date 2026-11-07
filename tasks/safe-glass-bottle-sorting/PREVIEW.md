# Common preview controls

Open the forwarded noVNC desktop and click inside the Isaac Sim 3D view so it
receives mouse and keyboard input.

## Bottle rack preview panel

| Button | What it does |
| --- | --- |
| Pause / Resume | Freeze or continue the task motion and physics stepping; camera navigation stays available. |
| Reset layout | Put every bottle and arm back at its starting pose and restart the demo clock. |
| Demo / Static | Switch between the repeating scripted sort and a stationary starting layout; also resets. |
| Front | View both shelf openings from the robots' side. |
| Overview | Show the rack, staging pedestal and both arms. |
| Close rack | Inspect the tight gaps and front edges. |

Use these task buttons to pause/reset the demonstration. Isaac Sim's general
Play/Stop toolbar controls the timeline and does not reset the script's own state.

## Move the camera

| Input | Effect |
| --- | --- |
| Mouse wheel | Zoom in/out. |
| Alt/Option + left-drag | Orbit around the focused object. |
| Middle-button drag | Pan sideways/up/down. |
| Right-button drag | Look around. |
| Hold right mouse + W/A/S/D | Fly forward/left/back/right. |
| Select an object, then F | Focus the view on that object. |

If you get lost, click **Front** or **Overview**. A mouse with a middle button is
convenient; with a trackpad, camera presets plus right-mouse/WASD avoid needing it.
The noVNC side tab contains a fullscreen button for a larger desktop view. Camera
navigation changes only your view, not the calibrated cameras in evaluation videos.

NVIDIA's [viewport navigation reference](https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_viewport/navigation.html)
describes these standard mouse and keyboard controls.

## Start or stop preview

On the GPU VM:

```bash
source ~/IsaacLab-Arena/yam.env
cd ~/robocurve-workspaces/benchmark-tasks
python tasks/safe-glass-bottle-sorting/view.py --demo
```

Omit `--demo` to start with the static layout. Stop a foreground preview with
Ctrl+C in the terminal that launched it. The loop runs more slowly than real time
on the VM; pausing is useful for inspecting a grasp or the shelf clearances.
