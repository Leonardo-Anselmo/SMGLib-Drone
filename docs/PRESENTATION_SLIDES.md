# Priority-Aware Multi-Drone Landing Pad Coordination
### Slide-prep notes — Phases 1 → 4.1

Each section below is sized for **one slide** (sometimes two): a short
"what's on the slide" block, then speaker notes you can read aloud. Image
hooks point at the GIFs already in `logs/Social-IMPC-DR/animations/`.

---

## 1. Motivation & Problem Statement

**Slide content**
- A fleet of hospital delivery drones must share **one landing pad**.
- Only one drone can land at a time → the pad is a hard bottleneck.
- Each drone carries different cargo: **organ, blood product, medication, equipment** — with very different urgency.
- Naïve "first come, first served" hurts patients: a routine equipment delivery can block an organ transplant.
- **Goal:** a coordination policy that lands the *most critical* cargo first, without collisions, while keeping every drone in motion.

**Speaker notes**
- Open with the realism: hospitals already use rooftop drone deliveries; the bottleneck is the pad, not the airspace.
- Frame the problem as scheduling under safety constraints: collision-free flight + medical urgency + finite landing capacity.
- Tease the result: by Phase 4 we can re-order landings dynamically based on cargo, expiry, distance, and patient acuity — not just arrival time.

---

## 2. Background — Why Social-IMPC-DR

**Slide content**
- We build on **Social-IMPC-DR** (Infinite-Horizon Model Predictive Control with Deadlock Resolution).
- Each drone runs its own MPC every step; collision avoidance comes from **MBVC** (Model-Based Voronoi Constraints) — a half-plane between every pair of drones.
- The framework already gives us:
  - Decentralized, real-time MPC at ~10 Hz.
  - Provable collision-free trajectories.
  - A built-in deadlock-resolution layer (drones politely yield instead of freezing forever).
- What it does **not** give us: priority, scheduling, or any notion that "drone A's cargo matters more than drone B's."
- Our extension: a **central referee layer** above the per-drone MPC that decides *who* goes and *when*, while the original MPC keeps every drone safe.

**Speaker notes**
- Explain MPC in one sentence: "each drone solves a small optimization every step to plan its next few seconds, then re-solves."
- Justify the choice: Social-IMPC-DR is open-source, decentralized, and already handles the hardest part — collision-free flight. We don't have to reinvent that.
- Make the architectural split clear: physics + safety stays decentralized (in MPC), policy + priority is centralized (the controller hierarchy we add). The two layers communicate through a tiny interface (yield / proceed).

---

## 3. Phase 1 & 2 — Baseline Bottleneck & Priority Scoring

**Slide content**
- **Phase 1 — Baseline.** One pad, mutual exclusion. Closest drone proceeds; everyone else freezes in place.
  - 2 drones, organ + equipment. Organ lands at step 51, equipment at step 102.
  - Establishes the "yield-and-proceed" pattern that the rest of the project builds on.
- **Phase 2 — Priority scoring.** Replace "closest" with a **weighted priority score**.
  - Inputs: cargo type, time-to-expiry, distance to pad, patient acuity.
  - Weights: cargo 35 %, expiry 30 %, distance 15 %, acuity 20 %.
  - 3 drones, mixed cargo. The organ now lands first **even when it starts farthest away**.

**Visuals**
- `phase1_landing_pad.gif` (Phase 1 baseline)
- `phase2_landing_pad.gif` (Phase 2 priority)

**Speaker notes**
- Phase 1 is the proof that the bottleneck is real: even with only 2 drones, you have to serialize.
- Phase 2 is where the project earns its name: "priority-aware." Show the GIF and point out that the organ leapfrogs the closer routine drone.
- Mention the architecture: `LandingPadController` → `PriorityManager` is a clean subclass override. We only changed *who is allowed to move*, not *how* they move.
- Result: 0 priority inversions, 0 collisions, 100 % delivery success.

---

## 4. Phase 3 — Circular Holding Orbits

**Slide content**
- **Problem with Phase 1/2:** waiting drones freeze mid-air. Unrealistic and visually awkward.
- **Solution:** yielding drones fly **small circular holding patterns** around their current position until released.
- Implementation:
  - `OrbitController` (subclass of `PriorityManager`) overrides only the "freeze" behavior.
  - Orbit kinematics are computed analytically — no MPC for orbiting drones (fast, predictable).
  - The **predicted future orbit** is fed into the active drone's MBVC, so its MPC sees the orbiting drones as moving obstacles and plans around them.
  - **Safe-distance push:** if the active drone wanders too close to an orbit, the orbit center is pushed outward to keep a safety gap.
- 3 drones, mixed cargo: organ lands first (step 41), then medication, then equipment. Same priority order as Phase 2, but everyone stays airborne.

**Visuals**
- `phase3_orbit.gif`

**Speaker notes**
- Stress that this is more than cosmetic — orbiting drones are now *moving obstacles*, which exercises the MPC's collision avoidance under realistic conditions.
- Architectural payoff: only one method (`freeze_yielding`) had to be overridden. The Phase 2 priority logic is reused unchanged.
- We deliberately picked a centralized orbit (rather than letting each drone's MPC fly to a holding waypoint) so the safety guarantee is **trivially provable**: only one drone is ever moving toward the pad.

---

## 5. Phase 4 — Pre-Contact Negotiation

**Slide content**
- **Problem with Phase 3:** strict priority ranking can be unfair in two cases.
  1. A high-priority drone with plenty of time blocks a closer drone whose cargo will **expire** before its turn.
  2. The top two drones are nearly tied on score, but the one slightly ahead is much **closer** — making the closer drone wait wastes time.
- **Solution: two arbitration rules, applied before priority scoring.**
  - **Expiry Guard.** If any waiting drone's `time_to_expiry < distance / nominal_speed`, that drone wins immediately, regardless of score. (It will literally expire if it waits.)
  - **ETA Switch.** If the score gap between the top two drones is below a threshold (default 0.15), the **closer** drone wins instead. Avoids penalizing nearby drones for tiny score differences.
  - If neither rule fires → fall back to the Phase 3 priority order.
- Implementation: `NegotiationController` (subclass of `OrbitController`) overrides one method, `negotiation_hook()`. The same yielding / orbiting machinery is reused.

**Visuals**
- `phase4_negotiation.gif` — main 3-drone demo (ETA Switch reorders the top two)
- `phase4_expiry_guard.gif` — isolated 2-drone test of Expiry Guard
- `phase4_eta_switch.gif` — isolated 2-drone test of ETA Switch

**Speaker notes**
- Walk through one numerical example: D0 = medication/urgent (close, score 0.639), D1 = organ/critical (farther, score 0.768). Score gap 0.129 < 0.15 → ETA Switch fires → D0 lands first. The organ still lands well within its expiry window.
- Emphasize: rules don't replace priority, they **modulate** it at the edges. Most decisions still flow through the Phase 2 score.
- Set up Phase 5: these are hand-coded rules. Will an LLM produce the same decisions, better decisions, or worse ones? That's the next experiment.

---

## 6. Phase 4.1 — System Hysteresis

**Slide content**
- **Problem we observed during Phase 4:** when two drones have nearly equal scores, the ETA Switch re-evaluates every step and the active winner can flip back and forth — causing visible jitter and frequent MPC resets.
- **Fix: commit-and-complete.** Once a drone wins the active landing slot, it keeps it **until it lands**. Only the Expiry Guard is allowed to preempt mid-handoff (because expiring cargo is genuinely an emergency).
- **Bonus fixes bundled in:**
  - Smooth handoff: when a drone is released from holding, we now zero its tangential orbit velocity and reset its MPC cost so it doesn't jolt backward on the first solve.
  - Persistent orbit state: brief winner swaps no longer reset the orbit center, so the holding pattern stays continuous.
- **Toggle for demos:** add `"use_hysteresis": false` to any scenario JSON to restore the original chattering behavior. Used to produce side-by-side comparison GIFs.

**Visuals — side-by-side comparison**
| Hysteresis OFF (chattering) | Hysteresis ON (committed) |
|---|---|
| `phase4_negotiation_no_hysteresis.gif` | `phase4_negotiation.gif` |
| ~23 winner swaps, lands at step 117 | 2 swaps total, lands at step 103 |

**Speaker notes**
- This slide is about **engineering polish** rather than algorithmic novelty: same Phase 4 rules, but now applied with a memory.
- Frame the toggle as a research feature: it lets us show the problem (chattering) and the fix (commit-and-complete) on the same scenario, so reviewers can see exactly what hysteresis buys us.
- The Expiry Guard exception is the one safety valve: hysteresis must never trap us into delivering an expired organ.
- Transition to Phase 5: with hysteresis in place, our Phase 4 rules are now stable enough to be a fair baseline for the LLM ablation.

---

## Appendix — Architecture diagram (suggested slide visual)

```
LandingPadController              ← Phase 1: closest-first
    │
    └── PriorityManager           ← Phase 2: priority-score-first
            │                       (Phase 4.1 hysteresis lives here)
            └── OrbitController   ← Phase 3: circular holding
                    │
                    └── NegotiationController  ← Phase 4: expiry + ETA rules
                            │
                            └── (LLMController, Phase 5 — planned)
```

Each subclass only overrides what it changes. The same simulation loop in
`test.py` runs every phase — only the controller class swaps.
