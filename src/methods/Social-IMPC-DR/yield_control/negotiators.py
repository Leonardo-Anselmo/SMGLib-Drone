"""Negotiators may override the selector decision before hysteresis runs.

Each negotiator returns either a full decision dict or ``None`` (in which
case the next negotiator, then the selector, is consulted). They are
peers: the controller calls them in the order listed in the recipe and
takes the first non-None result.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional
import urllib.error
import urllib.request

import numpy as np

from priority import priority_score

from .context import Context, PAD_CENTER


_API_URL = "https://api.anthropic.com/v1/messages"
_API_VER = "2023-06-01"
_MODEL = "claude-haiku-4-5-20251001"
_MAX_TOKENS = 120
_TIMEOUT = 8


class Negotiator:
    name: str = "negotiator"

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        raise NotImplementedError


def _gather_info(candidates: List[int], ctx: Context, nominal_speed: float):
    info = []
    for j in candidates:
        a = ctx.agent_list[j]
        dist = float(np.linalg.norm(a.p - PAD_CENTER))
        eta = dist / max(nominal_speed, 1e-6)
        tte = float(a.time_to_expiry)
        s = priority_score(
            cargo_type=a.cargo_type,
            time_to_expiry=tte,
            distance_to_pad=dist,
            patient_acuity=a.patient_acuity,
        )
        info.append({
            "idx":   j,
            "dist":  dist,
            "eta":   eta,
            "tte":   tte,
            "score": s,
        })
    return info


class ExpiryGuard(Negotiator):
    """If any candidate's TTE is below its ETA, that drone wins.

    Among multiple expiring candidates, the most urgent (lowest tte/eta
    ratio) is chosen. This negotiator is also treated as authoritative
    by the hysteresis layer — i.e. it can break a previously held
    winner, where other negotiators cannot.
    """

    name = "expiry_guard"

    def __init__(self, nominal_speed: float = 0.1) -> None:
        self.nominal_speed = nominal_speed

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        if len(candidates) <= 1:
            return None
        info = _gather_info(candidates, ctx, self.nominal_speed)
        scores_map = {d["idx"]: d["score"] for d in info}

        expiring = [d for d in info if d["tte"] < d["eta"]]
        if not expiring:
            return None
        expiring.sort(key=lambda d: d["tte"] / max(d["eta"], 1e-6))
        winner = expiring[0]
        allowed_idx = winner["idx"]
        yielding = {j for j in candidates if j != allowed_idx}
        if ctx.step % 10 == 0:
            print(
                f"  [Negotiation] Expiry Guard at step {ctx.step}: "
                f"drone {allowed_idx} wins "
                f"(tte={winner['tte']:.1f} < eta={winner['eta']:.1f})"
            )
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores_map,
        }


class EtaSwitch(Negotiator):
    """Tie-break by ETA when the top two priority scores are too close.

    If ``|score_top - score_second| < eta_threshold`` the closer drone
    wins; otherwise this negotiator returns ``None`` and lets normal
    selection proceed.
    """

    name = "eta_switch"

    def __init__(
        self,
        nominal_speed: float = 0.1,
        eta_threshold: float = 0.15,
    ) -> None:
        self.nominal_speed = nominal_speed
        self.eta_threshold = eta_threshold

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        if len(candidates) <= 1:
            return None
        info = _gather_info(candidates, ctx, self.nominal_speed)
        scores_map = {d["idx"]: d["score"] for d in info}

        info_by_score = sorted(info, key=lambda d: d["score"], reverse=True)
        top, second = info_by_score[0], info_by_score[1]
        if abs(top["score"] - second["score"]) >= self.eta_threshold:
            return None

        info_by_eta = sorted(info, key=lambda d: d["eta"])
        winner = info_by_eta[0]
        allowed_idx = winner["idx"]
        yielding = {j for j in candidates if j != allowed_idx}
        if ctx.step % 20 == 0:
            print(
                f"  [Negotiation] ETA switch at step {ctx.step}: "
                f"drone {allowed_idx} wins by proximity "
                f"(score gap={abs(top['score'] - second['score']):.3f} "
                f"< threshold={self.eta_threshold}, "
                f"eta={winner['eta']:.1f} steps)"
            )
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores_map,
        }


class LLMNegotiator(Negotiator):
    """Claude-driven negotiator adapted from Shariq's Phase 5 controller.

    This preserves Shariq's internal method names while exposing the flat
    policy-yield plugin entrypoint (`override`). Unlike the old
    `LLMController(NegotiationController)`, failure returns None so the next
    negotiator or selector can decide.
    """

    name = "llm_negotiator"

    def __init__(
        self,
        nominal_speed: float = 0.1,
        llm_model: str = _MODEL,
        llm_cache_steps: int = 10,
        use_llm_hook: bool = True,
        use_llm_score: bool = False,
    ) -> None:
        self._nominal_speed = nominal_speed
        self._llm_model = llm_model or _MODEL
        self._llm_cache_steps = max(1, int(llm_cache_steps))
        self._use_llm_hook = bool(use_llm_hook)
        self._use_llm_score = bool(use_llm_score)

        self._initial_configs = []
        self._cached_result = None
        self._cached_step = -(10 ** 9)
        self._cached_active_set = None
        self.llm_decision_log = []

        self._api_key = os.environ.get("ANTHROPIC_API_KEY", "")
        if not self._api_key:
            print(
                "\n  [LLMNegotiator] WARNING: ANTHROPIC_API_KEY is not set.\n"
                "  LLM calls will fall through to the normal policy recipe.\n"
                "  Set it with:  set ANTHROPIC_API_KEY=your-key-here\n"
            )

    # ------------------------------------------------------------------
    # Policy-yield plugin entrypoint
    # ------------------------------------------------------------------
    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        return self.negotiation_hook(ctx.agent_list, candidates, ctx.step)

    # ------------------------------------------------------------------
    # Shariq-compatible method names
    # ------------------------------------------------------------------
    def negotiation_hook(self, agent_list, active_drones, step):
        """Ask Claude which drone should land next.

        Returns None on any failure so the policy-yield controller can
        continue with the next negotiator or selector.
        """
        if not self._use_llm_hook or len(active_drones) <= 1:
            return None

        active_set = frozenset(active_drones)
        fresh_cache = (
            self._cached_result is not None
            and active_set == self._cached_active_set
            and step - self._cached_step < self._llm_cache_steps
        )
        if fresh_cache:
            return self._cached_result

        prompt = self._build_prompt(agent_list, active_drones, step)
        try:
            raw_text = self._call_llm(prompt)
            result = self._parse_response(raw_text, agent_list, active_drones)
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            print(
                f"  [LLM] step {step}: fall through "
                f"for model {self._llm_model!r}: "
                f"HTTP {exc.code} {exc.reason} {detail}"
            )
            return None
        except Exception as exc:
            print(f"  [LLM] step {step}: fall through ({exc})")
            return None

        self._cached_result = result
        self._cached_step = step
        self._cached_active_set = active_set

        self.llm_decision_log.append({
            "step": step,
            "allowed": result["allowed"],
            "method": result["method"],
            "reason": result.get("reason", ""),
        })
        return result

    def _score_active(self, agent_list, active_drones):
        """Optionally score active drones with the LLM.

        Kept from Shariq's structure for future scoring-mode scenarios.
        The current LLM negotiator config uses `negotiation_hook`.
        """
        if not self._use_llm_score:
            return self._formula_scores(agent_list, active_drones)

        prompt = self._build_scoring_prompt(agent_list, active_drones)
        try:
            raw_text = self._call_llm(prompt)
            return self._parse_scores(raw_text, active_drones)
        except Exception as exc:
            print(f"  [LLM scorer] fallback to formula ({exc})")
            return self._formula_scores(agent_list, active_drones)

    def _build_prompt(self, agent_list, active_drones, step):
        """Compose the LLM decision prompt."""
        lines = [
            "You are coordinating medical drone deliveries to a single hospital "
            "landing pad. Only one drone can land at a time.",
            "",
            f"Step: {step}",
            "Drones waiting to land:",
        ]

        for j in active_drones:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            eta = dist / max(self._nominal_speed, 1e-6)
            lines.append(
                f"  Drone {j}: cargo={a.cargo_type}, "
                f"acuity={a.patient_acuity}, "
                f"time_to_expiry={a.time_to_expiry:.0f} steps, "
                f"ETA={eta:.0f} steps"
            )

        lines += [
            "",
            "Which drone should land first? Consider medical urgency and expiry risk.",
            "",
            "Respond with EXACTLY this format:",
            "DRONE <index> because <one short sentence reason>",
            f"where <index> is one of: {list(active_drones)}",
        ]
        return "\n".join(lines)

    def _build_scoring_prompt(self, agent_list, active_drones):
        """Compose the LLM scoring prompt."""
        lines = [
            "You are a medical priority scoring system for drone deliveries.",
            "Score each drone's landing priority from 0.0 (lowest) to 1.0 (highest).",
            "",
            "Drones:",
        ]

        for j in active_drones:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            eta = dist / max(self._nominal_speed, 1e-6)
            lines.append(
                f"  Drone {j}: cargo={a.cargo_type}, "
                f"acuity={a.patient_acuity}, "
                f"time_to_expiry={a.time_to_expiry:.0f} steps, "
                f"ETA={eta:.0f} steps"
            )

        lines += [
            "",
            "Respond with EXACTLY this format (one line per drone):",
            "SCORE <index> <value>",
            "Example:  SCORE 0 0.82",
        ]
        return "\n".join(lines)

    def _call_llm(self, prompt):
        """Send the prompt to Claude and return the raw text response."""
        if not self._api_key:
            raise RuntimeError("ANTHROPIC_API_KEY not set")

        payload = json.dumps({
            "model": self._llm_model,
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

    def _parse_response(self, text, agent_list, active_drones):
        """Parse 'DRONE <index> because <reason>' into a decision dict."""
        allowed_idx = None
        reason = ""

        for line in text.splitlines():
            stripped = line.strip()
            if stripped.upper().startswith("DRONE"):
                parts = stripped.split(None, 2)
                if len(parts) >= 2:
                    try:
                        idx = int(parts[1])
                    except ValueError:
                        continue
                    if idx in active_drones:
                        allowed_idx = idx
                        reason = parts[2] if len(parts) > 2 else ""
                        break

        if allowed_idx is None:
            raise ValueError(f"Could not parse drone index from: {text!r}")

        yielding = {j for j in active_drones if j != allowed_idx}
        print(f"  [LLM] Drone {allowed_idx} selected - {reason}")
        return {
            "allowed": allowed_idx,
            "yielding": yielding,
            "method": "llm_hook",
            "scores": self._formula_scores(agent_list, active_drones),
            "reason": reason,
        }

    def _parse_scores(self, text, active_drones):
        """Parse 'SCORE <index> <value>' lines into {drone_idx: float}."""
        scores = {}
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 3 and parts[0].upper() == "SCORE":
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                except ValueError:
                    continue
                if idx in active_drones:
                    scores[idx] = float(np.clip(val, 0.0, 1.0))

        missing = [j for j in active_drones if j not in scores]
        if missing:
            raise ValueError(f"LLM did not score drones: {missing}")
        return scores

    def print_llm_summary(self):
        """Print a summary of all LLM arbitration decisions."""
        print("\n" + "=" * 50)
        print("  LLM ARBITRATION SUMMARY")
        print("=" * 50)

        if not self.llm_decision_log:
            print("  No LLM calls were made.")
        else:
            print(f"  Total LLM calls: {len(self.llm_decision_log)}")
            print()
            for entry in self.llm_decision_log:
                reason = entry.get("reason", "")
                print(f"  Step {entry['step']:>3}: Drone {entry['allowed']} selected")
                if reason:
                    print(f"           {reason}")

        print("=" * 50 + "\n")

    @staticmethod
    def _formula_scores(agent_list, active_drones):
        scores = {}
        for j in active_drones:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            scores[j] = priority_score(
                cargo_type=a.cargo_type,
                time_to_expiry=a.time_to_expiry,
                distance_to_pad=dist,
                patient_acuity=a.patient_acuity,
            )
        return scores
