"""Finite-horizon lookahead trajectory planner.

This module implements Leonardo's lookahead variant of the trajectory
planner track. It reuses the baseline trajectory planner's lifecycle logic
(INBOUND -> UNLOADING -> OUTBOUND -> DONE), but overrides the replanning
step so landing order is chosen by evaluating future candidate schedules
instead of simply sorting by current priority score.
"""

import itertools
import numpy as np

from landing_pad import PAD_CENTER
from priority import priority_score

from .baseline import (
    TrajectoryPlannerController,
    OUTBOUND,
)


class LookaheadTrajectoryPlannerController(TrajectoryPlannerController):
    """Trajectory planner that chooses landing order using finite-horizon lookahead."""

    # ------------------------------------------------------------------
    # Lookahead scheduler: evaluate candidate landing orders
    # ------------------------------------------------------------------

    def _choose_lookahead_order(self, info):
        """Choose the best landing order from all candidate orderings.

        The baseline planner sorts drones greedily by current priority score.
        This function instead checks every possible order of the current
        inbound drones, estimates the arrival schedule for that order, and
        chooses the order with the lowest total cost.

        The cost function is intentionally simple:
        - high-priority drones are penalized more heavily for waiting,
        - drones that arrive after cargo expiry receive a large extra penalty.
        """
        best_order = None
        best_cost = float("inf")

        # Try every possible ordering of the inbound drones
        # Fine for small fleet numbers but not easily scalable
        for order in itertools.permutations(info):
            T_prev = 0.0                    # Arrival time assigned to previous drone
            v_prev = self._max_speed        # Previous drone's assigned speed cap
            total_cost = 0.0                # Objective value for this candidate order

            for rank, d in enumerate(order):
                dist = d["dist"]
                score = d["score"]

                # Earliest physically possible arrival under the speed limit
                T_phys = dist / max(self._max_speed, 1e-6)

                # Pad constraint: next drone should not arrive until unloading clears
                T_pad = T_prev + self._unload_steps

                # Separation constraint: keep next arrivals spaced apart
                T_sep = T_prev + self._min_separation / max(v_prev, 1e-6)

                # The final assigned arrival time must satisfy all active constraints
                # For the first drone, there is no previous pad/separation constraint
                T_arrive = max(
                    T_phys,
                    T_pad if rank > 0 else 0.0,
                    T_sep if rank > 0 else 0.0,
                )

                # Convert the assigned arrival time into a cruise speed cap
                # This mirrors the baseline planner and lets MPC execute the schedule
                v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)

                # Objective term 1: priority-weighted delay
                # Higher-priority drones incur larger cost when scheduled later
                total_cost += score * T_arrive

                # Objective term 2: expiry risk
                # Late arrivals receive a large penalty to discourage expired delivery
                tte = d.get("time_to_expiry", 300.0)
                expiry_lateness = max(0.0, T_arrive - tte)
                total_cost += 100.0 * expiry_lateness

                # Store this drone's timing as the reference for the next slot
                T_prev = T_arrive
                v_prev = v_assign

            # Keep the candidate ordering with the best global objective value
            if total_cost < best_cost:
                best_cost = total_cost
                best_order = list(order)

        # Fallback keeps behavior safe if info is empty
        return best_order if best_order is not None else info

    # ------------------------------------------------------------------
    # Planner override: score, schedule, and push speed caps to MPC
    # ------------------------------------------------------------------

    def _replan(self, agent_list, inbound, step):
        """Compute arrival slots and speed caps for inbound drones.

        This overrides the baseline planner's greedy replanning step. The rest
        of the controller behavior, including lifecycle state transitions,
        target switching, unloading, and outbound handling, is inherited from
        the baseline trajectory planner.
        """
        if not inbound:
            self._last_schedule = {}
            return

        # 1) Build the planning table for all drones currently approaching pad
        # Each entry stores the drone index, distance-to-pad, priority score,
        # and time-to-expiry used by the lookahead objective
        info = []
        for j in inbound:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))

            # Reuse the same medical priority score as the baseline/Track 1 logic
            # so the only difference is the scheduling method, not the score
            score = priority_score(
                cargo_type=getattr(a, "cargo_type", "equipment"),
                time_to_expiry=float(getattr(a, "time_to_expiry", 300.0)),
                distance_to_pad=dist,
                patient_acuity=getattr(a, "patient_acuity", "routine"),
            )

            info.append({
                "idx": j,
                "dist": dist,
                "score": score,
                "time_to_expiry": float(getattr(a, "time_to_expiry", 300.0)),
            })

        # Optional LLM advisor hook
        # Makes the lookahead planner compatible with the same interface as baseline
        if self._llm_advisor is not None:
            adjusted_scores = self._llm_advisor.maybe_adjust_scores(
                agent_list, info, step
            )
            if adjusted_scores is not None:
                for d in info:
                    d["score"] = adjusted_scores[d["idx"]]

        # Store latest scores for logging/visualization used elsewhere
        self._last_scores = {d["idx"]: d["score"] for d in info}

        # 2) Choose landing order using finite-horizon lookahead rather than
        # greedy score sorting
        info = self._choose_lookahead_order(info)

        if step % 25 == 1:
            order = [d["idx"] for d in info]
            print(f"  [Lookahead Planner] landing order: {order}")

        # 3) Convert the selected order into concrete arrival slots and speed caps
        # These constraints match the baseline planner:
        # T_phys: cannot arrive faster than max speed allows
        # T_pad: must wait for the previous drone to unload
        # T_sep: must preserve minimum time/spatial separation
        schedule = {}
        T_prev = 0.0
        v_prev = self._max_speed

        for rank, d in enumerate(info):
            j = d["idx"]
            dist = d["dist"]

            T_phys = dist / max(self._max_speed, 1e-6)
            T_pad = T_prev + self._unload_steps
            T_sep = T_prev + self._min_separation / max(v_prev, 1e-6)

            T_arrive = max(
                T_phys,
                T_pad if rank > 0 else 0.0,
                T_sep if rank > 0 else 0.0,
            )

            # Vmax is the main control signal passed down to MPC
            v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)

            schedule[j] = {
                "T_arrive": T_arrive,
                "Vmax": v_assign,
                "rank": rank,
                "dist": dist,
            }

            T_prev = T_arrive
            v_prev = v_assign

        # 4) Push speed caps to each drone
        for j, sch in schedule.items():
            agent_list[j].Vmax = sch["Vmax"]

        # 5) Outbound drones are leaving the pad, so they are not part of the
        # inbound landing queue and can travel at the normal maximum speed
        for j in range(len(agent_list)):
            if self._state.get(j) == OUTBOUND:
                agent_list[j].Vmax = self._max_speed

        # Cache the selected schedule so the controller does not replan every
        # step unless the inbound set changes or the state gets messy 
        self._last_schedule = schedule
        self._last_inbound = frozenset(inbound)
        self._planner_dirty = False

        if step % 25 == 1:
            ranked = sorted(schedule.items(), key=lambda kv: kv[1]["rank"])
            summary = ", ".join(
                f"D{j}: v={s['Vmax']:.3f} T={s['T_arrive']:.1f}"
                for j, s in ranked
            )
            print(f"  [Lookahead Planner] step {step} schedule -> {summary}")

        # Preserve compatibility with the LLM advisor's explanation pathway
        if self._llm_advisor is not None:
            self._llm_advisor.maybe_explain_schedule(
                agent_list, info, schedule, step
            )
