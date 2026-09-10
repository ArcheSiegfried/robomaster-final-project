# Code-assistant rules

- Work only inside the assigned module and its tests unless the project maintainer explicitly coordinates a shared-interface change.
- Treat `models.py`, `runtime.py`, `motion_output.py` and `main.py` as shared boundaries. Do not casually refactor, rename or broaden them.
- Every detector consumes `FramePacket.image`; modules must not initialize a camera, robot or chassis.
- Task code returns `TaskUpdate` and optional `MotionCommand`; it must never call the RoboMaster SDK directly.
- Keep task steps non-blocking. Persist progress in small state fields and return once per frame.
- Do not add frameworks, plugin systems or placeholder modules without an accepted need.
- Never run `main.py`, connect a robot, start a video stream or send motion commands during automated work. Use offline images, synthetic frames and fake outputs only.
- Include focused tests and report the exact command and result with every handoff.
