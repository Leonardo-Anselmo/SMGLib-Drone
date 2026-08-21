# Social-IMPC-DR Landing-Pad Changelog

This changelog summarizes the research extension layered onto the upstream
Social-IMPC-DR implementation.

## Unreleased

### Project consistency

- Renamed ambiguous modules to descriptive `snake_case` filenames.
- Unified CLI, simulation, solver, plotting, and settings references.
- Corrected the `trajectory.svg` filename.
- Replaced submission-specific and contributor-specific operational comments
  with implementation-focused documentation.
- Standardized documentation paths and text-file encoding.

## Combined landing-pad study

### Coordination tracks

- Added the plugin-based `yield_control/` track with configurable selectors,
  yielders, lifecycles, and negotiators.
- Added the `trajectory_planner/` track with baseline speed scaling,
  finite-horizon lookahead, and an optional LLM advisor.
- Added one-way and round-trip delivery lifecycles with per-drone home pads.

### Simulation integration

- Added the landing-pad environment and shared-goal visualization.
- Added cargo type, time-to-expiry, and patient-acuity metadata.
- Added medical-priority scoring and configurable score weights.
- Added per-step controller decisions to animation labels.
- Added configuration-driven experiment commands and scenario files.

### Evaluation

- Added time-to-goal, velocity-change, path-deviation, makespan, success-rate,
  and flow-rate outputs.
- Added reproducible animations for yield-control and trajectory-planner
  scenarios.

The original implementation notes remain available in
[`UPSTREAM_README.md`](UPSTREAM_README.md).
