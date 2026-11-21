"""Show the dropped front chain edge-on, then show the same view through the repair."""

from PIL import Image, ImageDraw
import mujoco as mj
from video import Recorder, BG, INK, MUTED, W, H, camera
from fixtures import AXLES

GOLD = "#ffbb50"
BLUE = "#76b9ff"


class FaultRecorder(Recorder):
    def attach(self, trial):
        if trial.fixture.lift_pick:
            trial.model.vis.quality.offsamples = 0
        super().attach(trial)
        # Only change render colors. Contacts, masses and joint positions stay identical.
        m = trial.model
        for i in range(m.ngeom):
            name = m.geom(i).name or ""
            if name.startswith(("roller", "cheek")):
                m.geom_rgba[i] = [0.94, 0.48, 0.035, 1]
            elif name.startswith(("front_tooth", "front_rim")):
                m.geom_rgba[i] = [0.08, 0.38, 0.72, 1]
        for i in range(m.ntendon):
            if m.tendon_width[i] > 0.001:
                m.tendon_rgba[i] = [0.94, 0.48, 0.035, 1]
        self.cameras[0] = camera(AXLES[0] + [0.005, -0.019, 0.035], 0.155, 80, -36)

    def draw(self):
        canvas = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(canvas)
        s = self.trial
        read = s.reading()
        for renderer, cam, xy in zip(
            self.renderers, self.cameras, [(8, 50), (540, 50)]
        ):
            renderer.update_scene(s.data, camera=cam, scene_option=self.scene_option)
            renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = not s.fixture.lift_pick
            canvas.paste(Image.fromarray(renderer.render()), xy)
        d.text((12, 11), "Dropped bicycle chain", font=self.heading, fill=INK)
        d.text((335, 15), "Chain", font=self.body, fill=GOLD)
        d.text((435, 15), "Front teeth", font=self.body, fill=BLUE)
        d.text((550, 302), "Turns since start", font=self.body, fill=INK)
        for y, title, value in [
            (333, "Crank", read["front_from_reset"]),
            (362, "Rear sprocket", read["rear_from_reset"]),
        ]:
            d.text((550, y), title, font=self.body, fill=MUTED)
            d.text((750, y), f"{abs(value):.2f}", font=self.body, fill=INK)
        if read["score"] == 1:
            caption = "Repaired. The rear sprocket follows the crank."
        elif s.fixture.lift_pick and s.phase in (
            "Approach the hanging upper span",
            "Lower beside the loose chain",
            "Slide the pick under the chain",
            "Lift the loose chain above the teeth",
            "Carry rollers across the tooth tips",
            "Lower rollers onto the front ring",
            "Let the teeth support the upper run",
        ):
            caption = s.phase + "."
        elif "Test the dropped" in s.phase:
            caption = (
                "Before repair: the crank turns and the rear sprocket stays still."
            )
        elif s.phase in (
            "Approach the dropped upper run",
            "Lower fingers beside the upper run",
            "Guide the rollers to the tooth plane",
            "Feed the first links onto the teeth",
        ):
            caption = "The left arm guides the chain sideways onto the front teeth."
        elif s.phase in (
            "Turn the crank through two revolutions",
            "Withdraw guiding hand",
            "Withdraw the pedal socket",
            "Check both sprockets after release",
        ):
            caption = (
                "The chain feeds around the front teeth and drives the rear sprocket."
            )
        else:
            caption = (
                "At reset: the chain rests on the smooth grey sleeve beside the teeth."
            )
        d.text((12, 414), caption, font=self.body, fill=INK)
        self.final_pixels = canvas.tobytes()
        self.proc.stdin.write(self.final_pixels)
        if not self.frames:
            canvas.save(self.path / "setup.png")
        self.frames += 1
