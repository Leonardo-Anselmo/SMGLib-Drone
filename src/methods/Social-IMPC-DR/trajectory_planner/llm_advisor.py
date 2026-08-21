"""
Track 2 LLM advisor for the trajectory planner.

This module adapts the reusable parts of Shariq's Phase 5 LLM work
(API call, prompt construction, parsing, caching, and summary logging)
without importing the old yield/orbit/negotiation controller chain.

The advisor is intentionally not a controller. It is a helper used by
TrajectoryPlannerController after the planner computes its normal schedule.
In the default "explain" mode it does not change scores, Vmax, targets, or
state transitions.
"""

import json
import os
import urllib.error
import urllib.request


_API_URL = "https://api.anthropic.com/v1/messages"
_API_VER = "2023-06-01"
_MODEL = "claude-haiku-4-5-20251001"
_MAX_TOKENS = 180
_TIMEOUT = 8


class TrajectoryLLMAdvisor:
    """LLM helper for explaining or advising Track 2 planner schedules."""

    def __init__(self, mode="explain", model=_MODEL, cache_steps=25):
        self.mode = mode or "explain"
        self.model = model or _MODEL
        self.cache_steps = max(1, int(cache_steps))
        self._api_key = os.environ.get("ANTHROPIC_API_KEY", "")
        self._cached_key = None
        self._cached_step = -(10 ** 9)
        self._cached_text = None
        self.explanation_log = []
        self.score_adjust_log = []

        if self.mode not in {"explain", "score_adjust"}:
            print(f"  [LLM Advisor] Unknown mode {self.mode!r}; using explain.")
            self.mode = "explain"

        if not self._api_key:
            print(
                "\n  [LLM Advisor] WARNING: ANTHROPIC_API_KEY is not set.\n"
                "  Planner behavior will continue unchanged, but no LLM "
                "explanations or score adjustments will be produced.\n"
                "  Set it with:  set ANTHROPIC_API_KEY=your-key-here\n"
            )

    # ------------------------------------------------------------------
    # Explanation mode: no behavior change
    # ------------------------------------------------------------------
    def maybe_explain_schedule(self, agent_list, ranked_info, schedule, step):
        """Explain the planner's current schedule if explanation mode is active."""
        if self.mode != "explain":
            return None

        prompt = self._build_explanation_prompt(agent_list, ranked_info, schedule, step)
        cache_key = ("explain", self._schedule_signature(ranked_info, schedule))
        text = self._call_with_cache(prompt, cache_key, step)
        if text is None:
            return None

        entry = {
            "step": step,
            "mode": "explain",
            "order": [d["idx"] for d in ranked_info],
            "text": text,
        }
        self.explanation_log.append(entry)
        print(f"  [LLM Advisor] step {step}: {text}")
        return text

    # ------------------------------------------------------------------
    # Score-adjust mode: future behavior-changing path
    # ------------------------------------------------------------------
    def maybe_adjust_scores(self, agent_list, info, step):
        """Return LLM-adjusted scores when mode is score_adjust.

        On any failure, returns None so the planner keeps its deterministic
        priority scores unchanged.
        """
        if self.mode != "score_adjust":
            return None

        prompt = self._build_score_prompt(agent_list, info, step)
        cache_key = ("score_adjust", tuple((d["idx"], round(d["score"], 4))
                                           for d in info))
        text = self._call_with_cache(prompt, cache_key, step)
        if text is None:
            return None

        try:
            adjusted = self._parse_scores(text, [d["idx"] for d in info])
        except ValueError as exc:
            print(f"  [LLM Advisor] score parse failed: {exc}")
            return None

        self.score_adjust_log.append({
            "step": step,
            "scores": dict(adjusted),
            "raw": text,
        })
        return adjusted

    # ------------------------------------------------------------------
    # Prompt builders
    # ------------------------------------------------------------------
    def _build_explanation_prompt(self, agent_list, ranked_info, schedule, step):
        lines = [
            "You are reviewing a medical drone trajectory planner.",
            "The planner already computed the arrival order and speed caps.",
            "Do not change the schedule. Explain it briefly for a project report.",
            "",
            f"Step: {step}",
            "Computed schedule:",
        ]

        for d in ranked_info:
            j = d["idx"]
            a = agent_list[j]
            sch = schedule[j]
            expiry_margin = float(getattr(a, "time_to_expiry", 0.0)) - sch["T_arrive"]
            lines.append(
                f"  Drone {j}: rank={sch['rank']}, cargo={getattr(a, 'cargo_type', 'equipment')}, "
                f"acuity={getattr(a, 'patient_acuity', 'routine')}, "
                f"tte={getattr(a, 'time_to_expiry', 0.0):.0f}, "
                f"score={d['score']:.3f}, dist={d['dist']:.2f}, "
                f"T_arrive={sch['T_arrive']:.1f}, Vmax={sch['Vmax']:.3f}, "
                f"expiry_margin={expiry_margin:.1f}"
            )

        lines += [
            "",
            "Respond in one short paragraph, 2-3 sentences max.",
            "Mention the main medical reason for the order and any expiry risk.",
        ]
        return "\n".join(lines)

    def _build_score_prompt(self, agent_list, info, step):
        lines = [
            "You are a medical priority scoring assistant for drone deliveries.",
            "Adjust each deterministic planner score only if medical urgency or",
            "expiry risk clearly warrants it.",
            "",
            f"Step: {step}",
            "Drones:",
        ]

        for d in info:
            j = d["idx"]
            a = agent_list[j]
            lines.append(
                f"  Drone {j}: cargo={getattr(a, 'cargo_type', 'equipment')}, "
                f"acuity={getattr(a, 'patient_acuity', 'routine')}, "
                f"tte={getattr(a, 'time_to_expiry', 0.0):.0f}, "
                f"dist={d['dist']:.2f}, base_score={d['score']:.3f}"
            )

        lines += [
            "",
            "Respond with exactly one line per drone:",
            "SCORE <index> <value between 0.0 and 1.0>",
            "Example: SCORE 0 0.82",
        ]
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # LLM call, cache, parsers
    # ------------------------------------------------------------------
    def _call_with_cache(self, prompt, cache_key, step):
        if not self._api_key:
            return None

        fresh_cache = (
            self._cached_text is not None
            and self._cached_key == cache_key
            and step - self._cached_step < self.cache_steps
        )
        if fresh_cache:
            return self._cached_text

        try:
            text = self._call_llm(prompt)
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            print(
                f"  [LLM Advisor] call skipped/failed for model "
                f"{self.model!r}: HTTP {exc.code} {exc.reason} {detail}"
            )
            return None
        except Exception as exc:
            print(
                f"  [LLM Advisor] call skipped/failed for model "
                f"{self.model!r}: {exc}"
            )
            return None

        self._cached_key = cache_key
        self._cached_step = step
        self._cached_text = text
        return text

    def _call_llm(self, prompt):
        payload = json.dumps({
            "model": self.model,
            "max_tokens": _MAX_TOKENS,
            "messages": [{"role": "user", "content": prompt}],
        }).encode("utf-8")

        req = urllib.request.Request(
            _API_URL,
            data=payload,
            headers={
                "Content-Type": "application/json",
                "x-api-key": self._api_key,
                "anthropic-version": _API_VER,
            },
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
            data = json.loads(resp.read().decode("utf-8"))

        return data["content"][0]["text"].strip()

    @staticmethod
    def _parse_scores(text, active_ids):
        active_set = set(active_ids)
        scores = {}
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 3 and parts[0].upper() == "SCORE":
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                except ValueError:
                    continue
                if idx in active_set:
                    scores[idx] = min(1.0, max(0.0, val))

        missing = [idx for idx in active_ids if idx not in scores]
        if missing:
            raise ValueError(f"missing scores for drones {missing}")
        return scores

    @staticmethod
    def _schedule_signature(ranked_info, schedule):
        return tuple(
            (
                d["idx"],
                round(d["score"], 4),
                round(schedule[d["idx"]]["T_arrive"], 2),
                round(schedule[d["idx"]]["Vmax"], 4),
            )
            for d in ranked_info
        )

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def print_summary(self):
        print("\n" + "=" * 58)
        print("  TRACK 2 LLM ADVISOR SUMMARY")
        print("=" * 58)
        print(f"  Mode: {self.mode}")

        if self.mode == "explain":
            if not self.explanation_log:
                print("  No LLM explanations were produced.")
            else:
                print(f"  Explanations produced: {len(self.explanation_log)}")
                for entry in self.explanation_log:
                    order = " -> ".join(f"D{idx}" for idx in entry["order"])
                    print(f"  Step {entry['step']:>3} order {order}:")
                    print(f"    {entry['text']}")
        elif self.mode == "score_adjust":
            if not self.score_adjust_log:
                print("  No LLM score adjustments were produced.")
            else:
                print(f"  Score adjustments produced: {len(self.score_adjust_log)}")
                for entry in self.score_adjust_log:
                    print(f"  Step {entry['step']:>3}: {entry['scores']}")

        print("=" * 58 + "\n")
