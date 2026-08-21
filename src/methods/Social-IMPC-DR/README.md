# Social-IMPC-DR Landing-Pad Extension

This directory contains the SMGLib implementation of Infinite-Horizon Model
Predictive Control with Deadlock Resolution and the landing-pad coordination
study built on top of it.

The landing-pad environment supports two independent coordination tracks:

- **Yield control:** one selected drone approaches the pad while the remaining
  drones freeze or orbit.
- **Trajectory planning:** all inbound drones remain active while the planner
  assigns speed caps and arrival slots.

Both tracks share the MPC solver, cargo-priority model, lifecycle data,
configuration format, metrics, and animation renderer.

## Setup

Use Python 3.10 and install the pinned dependencies:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

The graphical interface additionally requires a working Qt display. Command-
line experiments can run headlessly when Matplotlib is configured with a
noninteractive backend.

## Entry points

| File | Purpose |
|---|---|
| `experiment_cli.py` | Config-driven landing-pad and research scenarios |
| `standard_cli.py` | Original doorway, hallway, and intersection CLI |
| `gui.py` | Original PyQt graphical interface |
| `simulation.py` | Shared simulation loop used by every entry point |

The repository-level `run_simulation.py` invokes `standard_cli.py` for normal
SMGLib scenarios.

## Yield-control track

The `yield_control/` package composes one implementation of each role:

- **Selector:** `closest_first` or `priority`
- **Yielder:** `freeze` or `orbit`
- **Lifecycle:** `one_way` or `round_trip`
- **Negotiator:** `expiry_guard`, `eta_switch`, or `llm_negotiator`

Each simulation step produces a decision with a consistent shape:

```python
{
    "allowed": int | None,
    "yielding": set[int],
    "method": str,
    "scores": dict[int, float],
}
```

The controller applies negotiators in configuration order, then falls back to
the selector. Optional winner hysteresis prevents rapid switching between
near-equal candidates.

## Trajectory-planner track

The baseline planner ranks inbound drones by medical priority and assigns each
one an arrival time and maximum cruise speed. For drone `k`:

```text
T_arrive[k] = max(
    distance[k] / max_speed,
    T_arrive[k-1] + unload_steps,
    T_arrive[k-1] + min_separation / speed[k-1],
)

speed[k] = min(distance[k] / T_arrive[k], max_speed)
```

The lookahead planner evaluates finite-horizon landing orders with the same
speed and separation constraints. The `llm` mode retains the baseline planner
and adds an optional explanation or score-advisory layer. The `compare_all`
mode retains its existing baseline-only behavior.

Planner scenarios use this lifecycle:

```text
INBOUND -> UNLOADING -> OUTBOUND -> INBOUND ... -> DONE
```

The planner recomputes its schedule whenever the inbound set changes. A
pad-busy safety guard freezes inbound drones that drift inside `safe_distance`
while another drone is unloading.

## Directory layout

```text
Social-IMPC-DR/
├── experiment_cli.py          # config-driven research entry point
├── standard_cli.py            # standard SMGLib entry point
├── gui.py                     # PyQt interface
├── simulation.py              # shared loop and metrics collection
├── settings.py                # shared runtime settings
├── solver.py                  # CVXPY optimization step
├── collision_avoidance.py     # inter-agent and wall constraints
├── agent.py                   # UAV state and MPC matrices
├── simulation_utils.py        # data collection and obstacle helpers
├── plotting.py                # trajectory rendering
├── standardized_plotting.py   # comparison-oriented rendering helpers
├── landing_pad.py             # landing-pad MPC helpers
├── priority.py                # cargo-priority scoring
├── configs/                   # scenario and priority configuration
├── trajectory_planner/        # baseline, lookahead, registry, LLM advisor
├── yield_control/             # selectors, yielders, lifecycles, negotiators
├── UPSTREAM_README.md         # original Social-IMPC-DR instructions
└── CHANGELOG.md               # landing-pad extension history
```

`trajectory_planner_controller.py` and the top-level `llm_advisor.py` remain
small compatibility imports for older scripts.

## Scenarios

| Configuration | Track | Purpose |
|---|---|---|
| `track_policy_baseline_closest.json` | Yield | Closest-first selection and freezing |
| `track_policy_priority.json` | Yield | Medical-priority selection |
| `track_policy_orbit_hold.json` | Yield | Orbit holding behavior |
| `track_policy_negotiation.json` | Yield | Priority and ETA negotiation |
| `track_policy_negotiation_no_hysteresis.json` | Yield | Negotiation without winner retention |
| `track_policy_negotiation_expiry_guard.json` | Yield | Expiry override |
| `track_policy_negotiation_eta_switch.json` | Yield | ETA-based switching |
| `track_policy_round_trip.json` | Yield | Round-trip lifecycle |
| `track_policy_llm_negotiator.json` | Yield | Optional LLM negotiator |
| `track_trajectory_oneway.json` | Planner | One delivery per drone |
| `track_trajectory_round_trip.json` | Planner | Repeated shuttle deliveries |
| `track_trajectory_llm_explain.json` | Planner | Baseline schedule with LLM explanation |

## Commands

Run commands from this directory so generated CSV and text metrics stay with
the IMPC-DR implementation:

```bash
# Yield control
python experiment_cli.py yield_control configs/track_policy_baseline_closest.json
python experiment_cli.py yield_control configs/track_policy_llm_negotiator.json

# Trajectory planning
python experiment_cli.py trajectory_planner baseline configs/track_trajectory_oneway.json
python experiment_cli.py trajectory_planner lookahead configs/track_trajectory_oneway.json
python experiment_cli.py trajectory_planner baseline configs/track_trajectory_round_trip.json
python experiment_cli.py trajectory_planner llm configs/track_trajectory_llm_explain.json
```

The legacy environment-first form remains supported:

```bash
python experiment_cli.py landing_pad configs/track_trajectory_oneway.json
```

## Configuration

Shared scenario fields include:

- `test_name`, `env_type`, `verbose`
- `num_moving_drones`, `min_radius`, `wall_collision_multiplier`
- `epsilon`, `step_size`, `k_value`, `max_steps`
- per-drone `start`, `goal`, `cargo_type`, `time_to_expiry`, and
  `patient_acuity`

Yield-control scenarios provide a `policy` object with `selector`, `yielder`,
`lifecycle`, optional `negotiators`, and component-specific parameter maps.

Trajectory-planner scenarios provide `use_trajectory_planner`, `n_trips`,
`unload_steps`, `max_speed`, `min_separation`, `safe_distance`, and
`nominal_speed`. Optional LLM fields include `use_llm_advisor`, `llm_mode`,
`llm_model`, and `llm_cache_steps`.

## Outputs

- Animations: `logs/Social-IMPC-DR/animations/<test_name>.gif`
- Velocity history: `avg_delta_velocity_robot_<id>.csv`
- Path deviation: `path_deviation_robot_<id>.csv`
- Time to goal: `ttg_impc_dr.csv`
- Completion step: `completion_step.txt`

LLM integrations read `ANTHROPIC_API_KEY` from the environment. API failures
fall back to the configured non-LLM selector or planner behavior.
