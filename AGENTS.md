# Code-assistant rules

- Before changing anything, read `README.md`, `PROJECT_HANDOVER.md`, `MODULE_GUIDE.md`, `TASKS.md`, `COLLABORATION_GUIDE.md`, and `DELIVERABLES.md`; then inspect the current branch and working tree.
- Work only on the assigned `feat/...`, `fix/...`, or `chore/...` branch created from `integration`. Do not make business changes directly on `main` or `integration`.
- Limit edits to the selected work package and its tests. Do not opportunistically refactor, rename, or move unrelated code.
- Treat `models.py`, `camera_source.py`, `runtime.py`, `motion_output.py`, and `main.py` as high-conflict public files. A change requires prior coordination by the integration lead, an interface-version decision, and integration tests.
- Every detector consumes the shared `FramePacket` or its `image`; modules must not initialize another camera, robot, chassis, or video subscription.
- Task code returns `VisualDetection`, `TaskUpdate`, and optional `MotionCommand`. It must never call the RoboMaster SDK or bypass `MotionOutput`.
- Keep task steps non-blocking. Store small state between frames and return once per call; do not use long sleeps, unbounded loops, or unlimited motion.
- Do not add plugin frameworks, event buses, complex inheritance, empty feature modules, or new dependencies without an accepted need.
- Never run `main.py`, connect a robot, start a video stream, or send real motion commands during code-assistant work. Use offline images, synthetic frames, fake clocks, and fake chassis only.
- Never invent test results, real-robot stability, field rules, or member contributions. Every handoff must include exact commands, actual results, and explicit unverified items.
