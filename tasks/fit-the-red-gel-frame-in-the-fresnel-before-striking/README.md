# Fit the red gel frame in a 3 in fresnel before the lamp is struck

Put the red gel in the light.

```bash
TASK_DEMO=demo   # 1.0: push the red frame to the stop and let go
TASK_DEMO=short  # 0.5: stop short of the stop
TASK_DEMO=idle   # 0.0: do nothing
TASK_DEMO=naive  # 0.0: arm stays aside
```

## Setup
An Altman Lighting CAT #100 3 in Inky (black epoxy sandtex body, 3 in Pyrex stepped lens, plated-steel yoke, 510-B-style C-clamp, black focus thumbscrew and door knob) sits on the bench on its back, lens pointing up, as when a lamp is gelled before it is hung. It is unpowered; there is no lamp and no heat. A 3.75 × 3.75 in (Altman 100-CFB size) aluminium colour frame already holds a pre-cut red gel and starts 30 mm short of the stop, already in both colour-frame holders. The red face is a clear table-middle cue. A second, blue, frame lies in a shallow gel box beside the lamp and is not in the holders. All props are rigid and start on fixed marks.

## Scoring
As seen in the overhead video, the red frame is in both colour-frame holders. At the end, after 1 s at rest and with the gripper clear of the frame: 1.0 once the frame is on the stop (within 3 mm) and has been let go; 0.5 if the frame has been pushed at least 10 mm from the start but is still short of the stop, or if it is on the stop but has not been let go or is not yet still; 0 for the start pose itself (30 mm short, already in the holders) and for any path that never pushes. Possible scores: 0, 0.5, 1.0.

## Design notes
- **Why it matters.** A colour frame holding gel gets hot once an incandescent fixture has been on. Gels have to sit in the fixture's colour runners, not on the lens (CINE-340; University of Rochester lighting-lab notes). Pre-hang practice is to seat the framed gel while the lamp is still on the bench, before it is struck.
- **Where the cue is.** The 95 mm red gel is a horizontal square in the table middle, with the black body, yoke, C-clamp and blue distractor around it.
- **Distinct from.** Upstream #1330 stacks gels on a lightbox to match a colour; that is colour matching, not seating a frame in a fixture's runners.
- **Honest limits / REACH trade-off.** The real Altman 100 is about 7.5 in (190 mm) along its axis. Sitting that barrel on its back would put the lens out of a straight-down grip, so the lamp body is 70 mm high. That is a REACH trade-off, flagged, not a hidden shrink. The red frame is an X-slider in the holders (PhysX contactOffset is 4 mm, so a free frame in real 3–5 mm runners is not used). No lamp is struck; the trial is cold. A sliver of the stepped lens is visible beside the offset frame at the start.

## Reset
Slide the red frame back 30 mm short of the stop, still in both holders. The blue frame stays in the gel box. The lamp is unpowered.

## Real trials
A used 3 in fresnel, two colour frames and pre-cut gel, well under the about $100 bench-trial cost. The trial is cold and unpowered.

## Fidelity
See task.json. The fixture is dressed from the Altman 100 (black sandtex, plated yoke and C-clamp, black hardware, 3 in stepped lens, 3.75 in frame). The 70 mm body versus the real 190 mm barrel is the flagged REACH trade-off.

## Scores
- Scripted demo (yam_arena, 320 × 176, seed 0, Brev L40S, v6b): 1.0, success at step 184 of 500 (seated at −1 mm).
- TASK_DEMO=short: 0.5 (17.5 mm remaining).
- TASK_DEMO=idle: 0.0.
- TASK_DEMO=naive: 0.0 (arm stays aside).
- Dream-1 (dreamlabs/dream-1-beta-flash, seed 1, 320 × 176, 22 Nov 2026 on a Brev L40S): 0.0 in 500 steps, not refused (20 requests, 20 attempts, term=max_steps).
