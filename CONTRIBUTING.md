# Contributing

This is a hobbyist, learning-driven robotics build (webcam-controlled tendon hand). Contributions,
forks, and issues are welcome — especially if you build your own copy and hit something this repo
gets wrong.

## Ways to help

- **Try the vision pipeline** (`vision/`) on your own webcam/hand and report tracking issues.
- **Build the hardware** from `hardware/BOM.md` and share what you'd change (parts, wiring, STL choice).
- **Improve the simulation** (`sim/`) — e.g. tune joint masses/inertia, add the right hand, add wrist rotation.
- **Fix or expand the docs** — see the separate docs repo (linked in `README.md`) for the reference
  documentation site; this repo's own `README.md` and code comments cover just the essentials.

## Workflow

1. Fork the repo, create a branch off `main`.
2. Keep commits scoped — one logical change per commit, with a message that explains *why*, not just what.
3. Open a PR describing what you tested (e.g. "ran `hand_tracker.py --no-serial` on a Logitech C920").
4. For hardware/sim changes that affect the InMoov-derived hand model, note *why* in the PR — this
   model has several deliberate adaptations from its upstream source (see `CLAUDE.md`), so unexplained
   geometry/joint changes are hard to review.

## Code style

- Python: no enforced formatter yet — match the surrounding file's style.
- Arduino: keep `hand_controller.ino` dependency-free beyond the standard `Servo` library.
- Shell examples in docs/comments should be fish-shell compatible (this project's dev machine uses fish).

## Reporting issues

Open a GitHub issue. Include your OS, Python version, and (for hardware issues) which servos/power
supply you're using — most of the open questions in `hardware/BOM.md` are hardware-variant-dependent.
