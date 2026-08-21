# Landing-Pad Coordination Research Notes

These notes summarize the progression from a single-winner landing policy to
the two coordination tracks now implemented in Social-IMPC-DR.

## Problem

Multiple drones approach one landing pad while carrying cargo with different
medical priorities and expiration windows. Collision avoidance alone does not
decide which drone should occupy the pad first, how the others should wait, or
how repeated deliveries should be scheduled.

The study separates those decisions into two approaches:

1. **Yield control:** choose one winner and explicitly control the waiting
   drones.
2. **Trajectory planning:** keep all drones moving and coordinate their arrival
   times.

Both approaches reuse the same Social-IMPC-DR solver and landing-pad
environment so their coordination logic can be compared independently of the
underlying collision-avoidance method.

## Yield-control progression

### Closest-first baseline

The first policy allows the closest drone to continue and freezes the others.
It establishes mutual exclusion at the pad but does not account for cargo
urgency.

### Medical-priority selection

Priority selection combines cargo type, patient acuity, time to expiry, and
distance to the pad. The score weights live in
`src/methods/Social-IMPC-DR/configs/priority_config.json`.

### Orbit holding

Orbit yielding replaces stationary waiting with circular holding patterns.
The selector remains unchanged; only the yielder plugin changes.

### Negotiation and hysteresis

Expiry and ETA negotiators can override the nominal selector in edge cases.
Winner hysteresis retains a valid decision across steps to avoid rapid
switching and repeated MPC resets.

### Optional LLM negotiation

The LLM negotiator uses the same override interface as rule-based negotiators.
Missing credentials or API failures fall through to the remaining configured
negotiators and selector.

## Trajectory-planner progression

### Baseline speed scaling

The baseline planner ranks inbound drones and converts the order into arrival
slots and maximum speeds. Pad occupancy, unloading time, and minimum separation
constrain every slot after the first.

### Finite-horizon lookahead

The lookahead planner evaluates candidate landing orders and minimizes
priority-weighted delay plus a cargo-expiry penalty. It inherits the baseline
planner's lifecycle and MPC integration.

### Optional LLM advisor

The planner advisor can explain schedules or propose score adjustments. The
default explanation mode does not change targets, speed caps, or lifecycle
state.

## Shared lifecycle

Round-trip scenarios use a common conceptual state machine:

```text
INBOUND -> UNLOADING -> OUTBOUND -> INBOUND ... -> DONE
```

One-way scenarios stop after the first completed delivery. Round-trip
scenarios send each drone to its home pad before it re-enters the inbound
queue.

## Current architecture

```text
experiment_cli.py
└── simulation.py
    ├── landing_pad.py
    ├── yield_control/
    │   ├── controller.py
    │   ├── selectors.py
    │   ├── yielders.py
    │   ├── lifecycles.py
    │   └── negotiators.py
    └── trajectory_planner/
        ├── baseline.py
        ├── lookahead.py
        ├── llm_advisor.py
        └── registry.py
```

The scenario animations under `logs/Social-IMPC-DR/animations/` correspond to
the JSON configurations under `src/methods/Social-IMPC-DR/configs/`.
