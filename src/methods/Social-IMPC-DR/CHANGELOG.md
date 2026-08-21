# Changelog — Social-IMPC-DR (Landing Pad Extension)

All notable changes from the original Social-IMPC-DR codebase.
For project overview, architecture, results, and usage, see [README.md](README.md).

---

## [Final submission interface] — Track-first commands and planner modes

### Added
- `yield_control/` package name for Track 1's single-winner yield method.
- `trajectory_planner/` package with:
  - `baseline.py` for the current working speed-scaling planner,
  - `llm_advisor.py` for the existing explanation helper,
  - `registry.py` for `baseline`, `llm`, `lookahead`, and `compare_all`
    mode names,
  - placeholders for Shariq's future trajectory LLM method and Leonardo's
    future look-ahead planner.

### Changed
- `app2_standardized.py` now supports track-first commands:
  - `python app2_standardized.py yield_control configs/track_policy_*.json`
  - `python app2_standardized.py trajectory_planner baseline configs/track_trajectory_*.json`
  - `python app2_standardized.py trajectory_planner llm configs/track_trajectory_*.json`
  - `python app2_standardized.py trajectory_planner lookahead configs/track_trajectory_*.json`
  - `python app2_standardized.py trajectory_planner compare_all configs/track_trajectory_*.json`
- `test.py` delegates trajectory controller construction to
  `trajectory_planner.registry`.

### Compatibility
- `landing_pad` remains the internal environment for both tracks.
- Legacy imports from `trajectory_planner_controller.py` and
  `llm_advisor.py` still work through compatibility shims.

---

## [Combined submission] — Include Track 1 and Track 2 as separate modules

### Added
- `yield_control/` package from Track 1, including the selector, yielder,
  lifecycle, negotiator, registry, and controller modules.
- `configs/track_policy_*.json` scenarios and matching
  `logs/Social-IMPC-DR/animations/track_policy_*.gif` outputs.

### Changed
- `test.py` now supports two explicit landing-pad dispatch paths:
  `policy_recipe` builds the Track 1 `PolicyYieldController`, while
  `round_trip_params` builds the Track 2 `TrajectoryPlannerController`.
- `app2_standardized.py` now reads either a Track 1 `policy` block or
  Track 2 `use_trajectory_planner: true` fields from scenario JSON.
- `README.md` now documents both tracks as separate modules for one PR.

### Compatibility
- The two controller designs remain separate. The combined branch does
  not merge policy/yield behavior into the trajectory planner or restore
  the old Phase-era inheritance chain.

---

## [Track 2 LLM advisor] — Modular explanation layer for the planner

### Added
- `llm_advisor.py` with `TrajectoryLLMAdvisor`, a helper adapted from
  Shariq's Phase 5 LLM work without importing the old yield/orbit/
  negotiation controller chain. It owns Anthropic API calls, prompt
  construction, response parsing, cache handling, and summary logging.
- `configs/track_trajectory_llm_explain.json`, a Track 2-named scenario
  that enables the advisor in `llm_mode: "explain"`.

### Changed
- `TrajectoryPlannerController` accepts an optional `llm_advisor`.
  In explanation mode, `_replan(...)` still computes the deterministic
  schedule first, then asks the advisor to explain the computed order,
  `T_arrive`, `Vmax`, and expiry risk. The advisor does not change
  scores, trajectories, speed caps, targets, or FSM transitions in the
  shipped config.
- `app2_standardized.py` parses `use_llm_advisor`, `llm_mode`,
  `llm_cache_steps`, and optional `llm_model` from scenario JSON.
- `test.py` constructs the advisor only when requested and prints the
  LLM advisor summary after the run.

### Compatibility
- This is not the old `LLMController(NegotiationController)` design.
  Track 2 stays `LandingPadController -> TrajectoryPlannerController`,
  with the LLM as a helper object. The old single-winner LLM negotiation
  approach remains a Track 1 / historical Phase 5 idea.

---

## [Track 2 cleanup, part 2] — Inline FSM into the planner; one controller for Track 2

### Removed
- `round_trip_controller.py` — the round-trip FSM, target swapping
  helpers, TTE countdown, and home-pad rendering hint were inlined into
  `trajectory_planner_controller.py`. Track 2 no longer carries a
  separate controller class for the lifecycle layer; the planner is
  the only Track 2 controller.

### Changed
- `TrajectoryPlannerController` now extends `LandingPadController`
  directly. It owns:
  - the FSM (`INBOUND -> UNLOADING -> OUTBOUND -> DONE`),
  - `_state` / `_unload_remaining` / `_trips_done` bookkeeping,
  - `_return_points` and the `n_trips >= 2` home-pad gating,
  - `bind(target)`, `_park`, `_switch_target`,
  - the TTE countdown (used by `priority_score` inside `_replan`),
  - the speed-scaling planner itself.
- `test.py` no longer imports `RoundTripController`. The
  `landing_pad` dispatch path is single-branch: planner or `ValueError`.
- `app2_standardized.py` no longer reads the `round_trip` flag from
  scenario JSON. The only flag that matters is `use_trajectory_planner`,
  with `n_trips: 1` for one-way and `n_trips: >= 2` for shuttle.

### Verified
- `track_trajectory_oneway.gif` is byte-identical to the pre-inline run
  (118 steps, 83.3 avg completion, 100% success).
- `track_trajectory_round_trip.gif` is byte-identical to the pre-inline
  run (232 steps, 202.7 avg completion, 100% success).

---

## [Track 2 cleanup, part 1] — Drop yield/orbit/negotiation chain

### Removed
- `priority_manager.py`, `orbit_controller.py`, `negotiation_controller.py`,
  `llm_controller.py` — these were dragged in only as transitive base
  classes for the `RoundTripController` constructor chain. Their selection
  logic (closest-first/priority/orbit/expiry-guard/eta-switch) is never
  exercised by `TrajectoryPlannerController` and belongs to Track 1.

### Changed
- `RoundTripController` extends `LandingPadController` directly
  (was `NegotiationController`). Its `__init__` no longer takes
  `orbit_radius`, `orbit_speed`, `safe_distance`, `nominal_speed`,
  `eta_threshold`, or `use_hysteresis`. The `time_to_expiry` countdown
  that used to come from `PriorityManager.step_update` is now inlined.
- `TrajectoryPlannerController.__init__` drops the orbit/eta/hysteresis
  kwargs that only existed to satisfy the deleted constructor chain.
- `test.py` no longer imports `PriorityManager` / `OrbitController` /
  `NegotiationController`. The `landing_pad` dispatch routes only to
  the planner or the round-trip baseline; anything else raises
  `ValueError` with a pointer to `research/track-policy-yield`.
- `app2_standardized.py` no longer builds `orbit_params` /
  `negotiation_params`. Scenario JSON's `use_orbit` / `use_negotiation`
  flags are now ignored (and have been dropped from the shipped
  `track_trajectory_*.json` configs).

### Behaviour delta
- The pad-busy safety net used to inherit `freeze_yielding` from
  `OrbitController`, which moved near-pad inbound drones along a
  circular orbit even though Track 2 never wanted holding patterns.
  Without that vestigial chain the safety net now uses
  `LandingPadController.freeze_yielding` (true zero-velocity freeze),
  which matches the README's documented intent. The one-way GIF
  changes slightly as a result (avg completion 84.0 -> 83.3 steps,
  same success rate); the round-trip GIF is byte-identical.

---

## [Track 2] — Trajectory planner only; legacy phase configs removed

### Added
- **`trajectory_planner_controller.py`**: speed-scaling planner for
  simultaneous inbound flight; ranks drones by priority and assigns
  per-drone `Vmax` from `max_speed`, `min_separation`, and `unload_steps`
  with replanning whenever the inbound set changes.
- **`configs/track_trajectory_oneway.json`** and
  **`configs/track_trajectory_round_trip.json`** scenarios.
- Wiring in `test.py` (`use_trajectory_planner` selects
  `TrajectoryPlannerController`) and `app2_standardized.py` (parses
  planner-specific fields and tags the output GIF as `planner`).

### Removed (still available on `dev/jinyoon`)
- `configs/phase1_landing_pad.json`, `configs/phase2_landing_pad.json`,
  `configs/phase3_orbit.json`,
  `configs/phase4_negotiation.json`,
  `configs/phase4_negotiation_no_hysteresis.json`,
  `configs/phase4_expiry_guard_test.json`,
  `configs/phase4_eta_switch_test.json`,
  `configs/phase6_round_trip.json` — these exercise the single-winner
  yield family which lives on `research/track-policy-yield`, not Track 2.
- `test_phase2.py` — Phase-era standalone test script.

### Migration
- Use `track_trajectory_*.json` on this branch for all planner runs.
- For yield/orbit/negotiation experiments, switch to the
  `research/track-policy-yield` branch and use `track_policy_*.json`.

---

## [Phase 6] — Round-Trip Scenarios

### Added
- **`round_trip_controller.py`** (`RoundTripController(NegotiationController)`): per-drone trip-state FSM
  (`INBOUND → UNLOADING → OUTBOUND → DONE`). Drones now land, sit on the pad
  for `unload_steps` ticks, then fly back to their start position; this can
  repeat for `n_trips` round trips before the drone is retired.
- **Pad-busy guard**: while any drone is `UNLOADING`, all other inbound
  drones orbit instead of converging on the occupied pad (avoids MPC
  infeasibility from trying to land on top of another drone).
- **Inbound-only negotiation**: only `INBOUND` drones compete for the pad
  slot; `OUTBOUND` drones are processed by MPC (so they actually fly home)
  but never yield to the pad and never participate in priority scoring.
- **`bind(target)` hook on the controller**: gives the controller a
  reference to test.py's per-drone target list so it can swap goals between
  `PAD_CENTER` and the return point.
- **`all_finished()` controller hook** (added to `LandingPadController`,
  overridden in `RoundTripController`): the simulation now ends when every
  drone reaches `DONE`, not when `target_reached[j]` is briefly true during
  unloading.
- **`configs/phase6_round_trip.json`** scenario: 3 drones, 2 trips each
  (6 landings total), `unload_steps=5`.

### Changed
- `test.py`: `select_active_drone` is now invoked for every active count
  (≥ 1, was > 1) so the round-trip pad-busy guard can fire even when only
  one drone is currently INBOUND. The completion check now defers to
  `controller.all_finished(...)` instead of `all(target_reached[...])`.

### Fixed
- `_switch_target` now also writes the new goal into `SET.target[j]`.
  Without this, `run.run_one_agent()` re-asserts the original goal via
  `agent.change_target(SET.target[agent.index])` on every MPC step,
  silently undoing the round-trip goal swap and freezing the drone in
  place. Also resets `term_overlap`, `term_overlap_again`, `term_index`,
  `eta`, and `term_last_pos` so the MPC doesn't think it has already
  converged on the previous goal.

### Verified
- Solo (1 drone, 2 trips): 149 steps, 100% success.
- Simple (2 drones, 1 trip each, opposite corners): 134 steps, 100%.
- Full (3 drones, 2 trips each = 6 landings): 250 steps, 100%.
- Phase 4 regression (no round-trip): unchanged at 103 steps, 100%.

---

## [Phase 4.1] — Handoff Hysteresis + Smooth Yield Release

### Added
- **Top-level winner hysteresis** in `priority_manager.py` — once a drone wins
  the active slot, it keeps it until it lands (`_held_winner`, `_held_method`,
  `_held_winner_step`). Only the Expiry Guard can override mid-handoff.
  Eliminates per-step chattering when two drones have nearly equal scores or
  ETAs (previously caused the Phase 4 winner to flip every step).
- **`use_hysteresis` toggle** plumbed end-to-end through `PriorityManager`,
  `OrbitController`, `NegotiationController`, `test.py`, and `app2_standardized.py`.
  Default `true`. Set `"use_hysteresis": false` in any scenario JSON to restore
  the original chattering behavior for side-by-side demos.
- **`configs/phase4_negotiation_no_hysteresis.json`** — chattering "before"
  demo (~23 release events, lands at step 117) paired with the default
  `phase4_negotiation.json` (2 release events, lands at step 103).

### Changed
- **`orbit_controller.py`** — orbit state (`{center, angle}`) is no longer
  purged when a drone briefly exits the yield set. State is only re-initialized
  if the drone has drifted more than `1.5 × orbit_radius` from its last orbit
  center. Prevents orbit re-anchoring jolts during rapid winner swaps.
- **`negotiation_controller.py`** — removed the per-rule internal hysteresis
  on `ETA Switch`; hysteresis is now uniformly handled at the top level so all
  three rules (Expiry Guard, ETA Switch, plain priority) share the same
  commit-and-complete semantics.

### Fixed
- **Backward jolt on yield → active handoff** in `landing_pad.py reset_mpc()`:
  - `cost_index` was being reset to `0`, which anchors *all* horizon steps at
    the goal and pulls the drone violently backward on the first MPC step.
    Changed to `K` so only the terminal step is anchored (matching steady-state).
  - Velocity is now zeroed on release so leftover orbit tangential velocity
    doesn't get carried into the descent leg.

---

## [Phase 4] — ETA-Aware Negotiation with Expiry Enforcement

### Added
- **`negotiation_controller.py`** — `NegotiationController(OrbitController)`
  - Overrides `negotiation_hook()` with two-rule logic: Expiry Guard and ETA Switch
  - Expiry Guard: if `time_to_expiry < distance / nominal_speed`, strict priority wins
  - ETA Switch: if score gap < `eta_threshold`, closer drone wins
  - Returns `None` if neither rule fires (falls through to normal priority)
- **`configs/phase4_negotiation.json`** — 3-drone demo scenario; ETA Switch inverts landing order vs Phase 3
- **`configs/phase4_expiry_guard_test.json`** — 2-drone isolated test for Expiry Guard rule
- **`configs/phase4_eta_switch_test.json`** — 2-drone isolated test for ETA Switch rule

### Changed
- **`test.py`** — Added `negotiation_params` kwarg to `PLAN()`; controller selection
  picks `NegotiationController` when `negotiation_params` is provided
- **`app2_standardized.py`** — Reads `use_negotiation`, `nominal_speed`, `eta_threshold`
  from scenario JSON; builds `negotiation_params` dict and passes to `PLAN()`
  — GIF filename uses `test_name` field from JSON if present

### Changed (all scenario configs)
- Added `test_name` field to all configs so output GIF filename matches scenario name

---

## [Phase 3] — Holding Orbit Patterns

### Added
- **`orbit_controller.py`** — `OrbitController(PriorityManager)`
  - Overrides `freeze_yielding()` to move waiting drones in a circular orbit
  - `_orbit_state` dict: maps drone index → `{center, angle}`
  - Each step: `angle += orbit_speed`; position and velocity set analytically
  - Safe-distance repulsion: orbit center pushed outward if active drone approaches
  - `pre_traj` filled with predicted orbit positions for active drone's MBVC constraints
- **`configs/phase3_orbit.json`** — 3-drone scenario with orbit params

### Changed
- **`test.py`** — Added `orbit_params` kwarg to `PLAN()`; controller selection
  picks `OrbitController` when both `cargo_configs` and `orbit_params` are provided
- **`app2_standardized.py`** — Reads `use_orbit`, `orbit_radius`, `orbit_speed`,
  `safe_distance` from scenario JSON; builds `orbit_params` dict and passes to `PLAN()`

---

## [Phase 2] — Dynamic Priority Integration

### Added
- **`priority.py`** — Standalone scoring module
  - `priority_score()` → weighted float in [0, 1]
  - `rank_drones()` → sorted list by descending score
  - Weights: cargo 35%, expiry 30%, distance 15%, acuity 20%
- **`priority_manager.py`** — `PriorityManager(LandingPadController)`
  - Overrides `select_active_drone()` to pick highest-priority drone
  - Overrides `step_update()` to decrement `time_to_expiry` each step
  - Falls back to distance-based selection if no cargo config is present
- **`test_phase2.py`** — Standalone 3-drone test script (no animation)
- **`configs/phase2_landing_pad.json`** — 3-drone scenario config with mixed cargo

### Changed
- **`uav.py`** — Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes
  (backward-compatible; defaults to `None` if not set)
- **`test.py`** — Controller instantiation checks `cargo_configs` to choose between
  `LandingPadController` and `PriorityManager`
- **`app2_standardized.py`** — Reads drone cargo attributes from scenario JSON;
  passes them to `PLAN()` as `cargo_configs`

---

## [Phase 1] — Baseline Bottleneck Simulation

### Added
- **`landing_pad.py`** — `LandingPadController` base class
  - `select_active_drone()` — picks closest active drone; others yield
  - `freeze_yielding()` — zeros velocity for waiting drones
  - `cleanup_landed()` — teleports landed drones off-screen to unblock pad
  - `reset_mpc()` — warm-start reset when drone transitions yielding → active
  - `update_idle_positions()` — keeps animation position arrays aligned
  - `step_update()` — per-step hook (no-op; extensible)
  - `negotiation_hook()` — pre-selection hook (no-op; Phase 4 override point)
- **`configs/phase1_landing_pad.json`** — 2-drone scenario config, no priority
- **`configs/priority_config.json`** — Priority scoring weights and cargo/acuity tables
- **`src/utils.py`** — Added `landing_pad` environment type with shared goal,
  wall obstacle layout, and pad visualization marker

### Changed
- **`run.py`** — Replaced hardcoded `if 2 <= index <= 21` with `SET.num_moving_drones`
- **`avoid.py`** — Added `wall_collision_multiplier` and `env_type` parameters
- **`SET.py`** — Added `ENV_TYPE` and `num_moving_drones` global parameters
- **`test.py`** — Refactored simulation loop to delegate to controller classes;
  added early exit when all drones land
- **`app2_standardized.py`** — Added `landing_pad` scenario branch;
  added config-file mode (`python app2_standardized.py landing_pad configs/X.json`)

### Fixed
- MPC infeasibility from `4× r_min` inflation in `avoid.py` → removed inflation,
  use turn-based yielding instead
- Drone blocked by landed drone at pad → `cleanup_landed()` teleports to (100, 100)
- Drone oscillating near goal after long hold → `reset_mpc()` on yield release
- Animation desync from unequal position array lengths → `update_idle_positions()`

---

## Files Summary

### New files
| File | Phase | Purpose |
|---|---|---|
| `landing_pad.py` | 1 | Base controller — mutual exclusion, yielding, MPC reset |
| `priority.py` | 2 | Scoring math — `priority_score()`, `rank_drones()` |
| `priority_manager.py` | 2 | Priority-based controller — extends `LandingPadController` |
| `test_phase2.py` | 2 | Standalone Phase 2 test (no animation) |
| `orbit_controller.py` | 3 | Orbit holding controller — extends `PriorityManager` |
| `negotiation_controller.py` | 4 | Negotiation controller — extends `OrbitController` |
| `configs/phase1_landing_pad.json` | 1 | 2-drone closest-first scenario |
| `configs/phase2_landing_pad.json` | 2 | 3-drone priority-based scenario |
| `configs/phase3_orbit.json` | 3 | 3-drone orbit holding scenario |
| `configs/phase4_negotiation.json` | 4 | 3-drone ETA Switch demo (compare with Phase 3) |
| `configs/phase4_negotiation_no_hysteresis.json` | 4.1 | Same scenario with `use_hysteresis: false` for chattering comparison |
| `configs/phase4_expiry_guard_test.json` | 4 | 2-drone Expiry Guard test scenario |
| `configs/phase4_eta_switch_test.json` | 4 | 2-drone ETA Switch test scenario |
| `configs/priority_config.json` | 2 | Priority scoring weights, cargo weights, acuity scores |

### Modified files
| File | What changed |
|---|---|
| `app2_standardized.py` | Config-file mode, landing pad branch, cargo/orbit/negotiation passthrough, GIF naming, `use_hysteresis` flag |
| `test.py` | Controller delegation, cargo/orbit/negotiation params, `use_hysteresis` plumbing, early exit |
| `landing_pad.py` | `reset_mpc()` now sets `cost_index = K` and zeros velocity — fixes handoff jolt |
| `priority_manager.py` | Top-level winner hysteresis (`_held_winner`); `use_hysteresis` ctor flag |
| `orbit_controller.py` | Persistent orbit state across brief releases (drift-based re-init); `use_hysteresis` flag |
| `negotiation_controller.py` | Removed per-rule hysteresis (now top-level); `use_hysteresis` flag |
| `uav.py` | Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes |
| `run.py` | Fixed hardcoded drone index threshold |
| `avoid.py` | Wall collision multiplier, env_type support |
| `SET.py` | Added `ENV_TYPE`, `num_moving_drones` globals |
| `src/utils.py` | Added `landing_pad` environment type |

### Deleted files
| File | Reason |
|---|---|
| `cargo_configs.json` | Replaced by per-scenario configs in `configs/` |
| `docs/PROJECT_GUIDELINE.md` | Content consolidated into README.md |

### Unchanged files
`others.py`, `dynamic.py`, `plot.py`, `plot_standardized.py`