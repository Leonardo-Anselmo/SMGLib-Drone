# Social-CADRL Integration

Social-CADRL is available through the repository-level `run_simulation.py`
interface. It runs in an isolated virtual environment because its TensorFlow,
Gym, and NumPy requirements differ from the other navigation methods.

## Layout

```text
SMGLib-Drone/
├── run_simulation.py
├── logs/Social-CADRL/
└── src/methods/Social-CADRL/
    ├── config.json
    ├── envs/
    ├── experiments/
    └── venv/                  # generated on first use; ignored by Git
```

## Running through SMGLib

```bash
python run_simulation.py
```

Select Social-CADRL and then choose the doorway, hallway, or intersection
environment. On first use, `setup_cadrl_environment()` in `run_simulation.py`
creates `src/methods/Social-CADRL/venv/` and writes a setup marker after the
environment is ready.

## Running directly

The scenario scripts are under `src/methods/Social-CADRL/experiments/src/`.
After the isolated environment has been created:

```bash
cd src/methods/Social-CADRL/experiments/src
../../venv/bin/python run_scenarios.py --scenario doorway --agents 2
```

Windows users should invoke the corresponding interpreter under
`venv/Scripts/`.

## Data flow

1. `run_social_cadrl()` collects the selected environment and agent settings.
2. The settings are translated into the CADRL coordinate system.
3. CADRL executes in its isolated environment.
4. Generated animations are copied to `logs/Social-CADRL/animations/`.
5. The shared metrics layer reports trajectory, success, makespan, and flow
   information.

## Coordinate systems

| Method | Coordinate convention |
|---|---|
| Social-ORCA | Grid coordinates with a bottom-left origin |
| Social-IMPC-DR | Continuous environment coordinates |
| Social-CADRL | Centered continuous coordinates |

The integration layer converts standardized scenario positions to the format
expected by each method.

## Troubleshooting

### NumPy compatibility errors

Use the CADRL virtual environment created by `run_simulation.py`. Installing
the root requirements directly into that environment can replace the older
versions required by CADRL.

### Missing `gym_collision_avoidance`

Delete the incomplete `src/methods/Social-CADRL/venv/` directory and rerun the
Social-CADRL option so the setup process can recreate it.

### No animation output

Check the CADRL process output and confirm that Matplotlib can use an available
display or noninteractive backend. Final animations should appear under
`logs/Social-CADRL/animations/`.
