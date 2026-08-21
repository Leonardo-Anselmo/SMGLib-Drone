"""
Phase 1 — Landing Pad Controller

Manages the single-pad bottleneck: mutual exclusion (one drone lands at a
time), yielding logic, MPC warm-start reset, and landed-drone cleanup.

Usage inside the simulation loop (test.py):
    controller = LandingPadController()
    ...
    controller.cleanup_landed(agent_list, target_reached, num_moving, K)
    result = controller.select_active_drone(agent_list, active, step, verbose)
    controller.freeze_yielding(agent_list, result["yielding"])
    released = controller.get_released_drones(process_indices, prev_yielding)
    controller.reset_mpc(agent_list, released, verbose)
    controller.update_idle_positions(agent_list, process_indices, target_reached, target, num_moving)
"""

import numpy as np

PAD_CENTER = np.array([0.0, 0.0])


class LandingPadController:
    """Phase 1 baseline: closest-drone-first greedy landing policy."""

    # ------------------------------------------------------------------
    # Landed-drone cleanup
    # ------------------------------------------------------------------
    @staticmethod
    def cleanup_landed(agent_list, target_reached, num_moving_drones, K):
        """Teleport landed drones to (100, 100) so they don't block the pad."""
        for j in range(num_moving_drones):
            if target_reached[j]:
                agent_list[j].p = np.array([100.0, 100.0])
                agent_list[j].v = np.zeros(2)
                agent_list[j].state = np.append(agent_list[j].p, agent_list[j].v)
                agent_list[j].pre_traj = np.tile(
                    np.array([100.0, 100.0]), (K + 1, 1)
                )

    # ------------------------------------------------------------------
    # Negotiation hook (no-op in Phase 1/2; override for Phase 4)
    # ------------------------------------------------------------------
    def negotiation_hook(self, agent_list, active_drones, step):
        """Called before ``select_active_drone``.

        Override in a subclass to implement pairwise negotiation or
        LLM-based arbitration.  Return ``None`` to let the normal
        selection proceed, or return a result dict (same schema as
        ``select_active_drone``) to override the decision entirely.
        """
        return None

    # ------------------------------------------------------------------
    # Yielding decision  (Phase 1 — distance-based)
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        """Pick the closest active drone; everyone else yields.

        Returns
        -------
        dict with keys:
            allowed : int | None
                Index of the drone that may proceed.
            yielding : set[int]
                Indices of drones that must hold position.
            method : str
                Selection method name ("distance", "priority", etc.).
            scores : dict[int, float]
                Per-drone score used for the decision.
        """
        if len(active_drones) <= 1:
            idx = active_drones[0] if active_drones else None
            return {"allowed": idx, "yielding": set(),
                    "method": "distance", "scores": {}}

        # Check negotiation hook first
        override = self.negotiation_hook(agent_list, active_drones, step)
        if override is not None:
            return override

        distances = [
            (j, np.linalg.norm(agent_list[j].p - PAD_CENTER))
            for j in active_drones
        ]
        distances.sort(key=lambda x: x[1])
        allowed_idx = distances[0][0]

        if verbose and step % 20 == 0:
            print(
                f"  Pad yielding: drone {allowed_idx} approaching "
                f"(dist={distances[0][1]:.2f}), "
                f"others holding: {[d[0] for d in distances[1:]]}"
            )

        yielding = {j for j in active_drones if j != allowed_idx}
        scores = {j: dist for j, dist in distances}
        return {"allowed": allowed_idx, "yielding": yielding,
                "method": "distance", "scores": scores}

    # ------------------------------------------------------------------
    # Freeze helpers
    # ------------------------------------------------------------------
    @staticmethod
    def freeze_yielding(agent_list, yielding_drones):
        """Zero velocity for every yielding drone."""
        for j in yielding_drones:
            agent_list[j].v = np.zeros(2)
            agent_list[j].state = np.append(agent_list[j].p, agent_list[j].v)

    @staticmethod
    def get_released_drones(process_indices, prev_yielding):
        """Return indices of drones that were yielding last step but are active now."""
        return [j for j in process_indices if j in prev_yielding]

    @staticmethod
    def reset_mpc(agent_list, released_indices, verbose):
        """Warm-start MPC reset for drones just released from holding."""
        for j in released_indices:
            agent_list[j].pre_traj = np.tile(
                agent_list[j].p, (agent_list[j].K + 1, 1)
            )
            # Kill the tangential orbit velocity so the first MPC solve
            # doesn't have to flip direction (which looks like a backward
            # dart in the animation).
            agent_list[j].v = np.zeros_like(agent_list[j].v)
            agent_list[j].state = np.append(agent_list[j].p, agent_list[j].v)
            # Restore terminal-only cost weighting (matches __init__ default).
            # Resetting to 0 forces the solver to anchor every horizon step at
            # the target, producing a visible backward jolt on first solve.
            agent_list[j].cost_index = agent_list[j].K
            if verbose:
                print(f"  Drone {j} released from holding — MPC reset")

    # ------------------------------------------------------------------
    # Position bookkeeping for idle / landed drones
    # ------------------------------------------------------------------
    @staticmethod
    def update_idle_positions(agent_list, process_indices, target_reached,
                              target, num_moving_drones):
        """Append current position to history for drones that skipped MPC."""
        for j in range(num_moving_drones):
            if j not in process_indices:
                if target_reached[j]:
                    agent_list[j].position = np.block(
                        [[agent_list[j].position], [target[j]]]
                    )
                else:
                    agent_list[j].position = np.block(
                        [[agent_list[j].position], [agent_list[j].p]]
                    )

    # ------------------------------------------------------------------
    # Per-step hook (no-op in Phase 1; overridden in Phase 2)
    # ------------------------------------------------------------------
    def step_update(self, agent_list, target_reached, num_moving_drones):
        """Called at the end of each simulation step. Override for extra logic."""
        pass

    # ------------------------------------------------------------------
    # Termination check (overridden in Phase 6 for round-trip lifecycle)
    # ------------------------------------------------------------------
    def all_finished(self, target_reached, num_moving_drones):
        """Return True when the simulation can terminate. By default, all
        drones have reached their goals. Round-trip controllers override
        this so the loop doesn't exit while drones are between trips."""
        return all(target_reached[:num_moving_drones])
