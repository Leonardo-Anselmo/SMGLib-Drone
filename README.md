# SMGLib Drone Coordination Research

This repository extends the Social Mini-Games Library (SMGLib) with a
multi-drone landing-pad study. The original doorway, hallway, and intersection
environments remain available alongside two landing-pad coordination tracks:

- **Yield control** selects one drone to approach while the others freeze or
  orbit.
- **Trajectory planning** keeps inbound drones moving and assigns arrival
  order through speed limits.

The project is based on [CRAL-UVA/SMGLib](https://github.com/CRAL-UVA/SMGLib)
and retains its BSD 3-Clause license and citation information.

## Requirements

- Python 3.10
- A virtual environment is recommended.
- The IMPC-DR solver uses CVXPY and SCS. Optional GUI workflows also require
  PyQt5.
- LLM-assisted scenarios require `ANTHROPIC_API_KEY`; all non-LLM scenarios
  run without it.

Install the shared dependencies:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

The pinned IMPC-DR environment is documented in
`src/methods/Social-IMPC-DR/requirements.txt`.

## Running simulations

Launch the original SMGLib environment selector:

```bash
python run_simulation.py
```

Run landing-pad experiments from the IMPC-DR directory:

```bash
cd src/methods/Social-IMPC-DR

# Yield-control examples
python experiment_cli.py yield_control configs/track_policy_baseline_closest.json
python experiment_cli.py yield_control configs/track_policy_priority.json

# Trajectory-planner examples
python experiment_cli.py trajectory_planner baseline configs/track_trajectory_oneway.json
python experiment_cli.py trajectory_planner lookahead configs/track_trajectory_oneway.json
python experiment_cli.py trajectory_planner baseline configs/track_trajectory_round_trip.json
```

Additional scenario files in `configs/` cover orbit holding, negotiation,
round trips, and optional LLM assistance. The `compare_all` mode retains its
existing baseline-only behavior; run `baseline`, `lookahead`, and `llm`
separately when comparing planner modes.

## Landing-pad architecture

```text
experiment_cli.py
└── simulation.py
    ├── solver.py                     # MPC optimization loop
    ├── landing_pad.py                # shared landing-pad MPC helpers
    ├── yield_control/                # selector/yielder/lifecycle/negotiator plugins
    └── trajectory_planner/           # baseline, lookahead, and LLM advisor
```

The three upstream algorithm implementations remain under `src/methods/`:

```text
src/methods/
├── Social-CADRL/
├── Social-IMPC-DR/
└── Social-ORCA/
```

Generated animations are written under `logs/<method>/animations/`. Per-run
IMPC-DR metrics include time-to-goal, velocity change, path deviation,
makespan, success rate, and flow rate.

## Documentation

- [`src/methods/Social-IMPC-DR/README.md`](src/methods/Social-IMPC-DR/README.md)
  describes landing-pad configuration and controller behavior.
- [`docs/flow_rate_metrics.md`](docs/flow_rate_metrics.md) explains metric
  calculations.
- [`docs/cadrl_integration.md`](docs/cadrl_integration.md) covers the CADRL
  environment.
- [`docs/landing_pad_research_notes.md`](docs/landing_pad_research_notes.md)
  records the landing-pad study progression.

## Attribution

Please cite the original SMGLib work when using this repository:

```bibtex
@article{chandra2025multi,
  title   = {Multi-robot navigation in social mini-games: Definitions, taxonomy, and algorithms},
  author  = {Chandra, Rohan and Singh, Shubham and Luo, Wenhao and Sycara, Katia},
  journal = {arXiv preprint arXiv:2508.13459},
  year    = {2025}
}
```

See [`LICENSE`](LICENSE) for the BSD 3-Clause terms.
