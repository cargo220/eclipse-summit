"""Pure arbitration for applying height AI proposals to the servo target.

Mirrors cmd_vel_logic.py's manual-priority pattern: eclipse_test_controller
(the sole Dynamixel bus owner) calls arbitrate_height_ai_command() once per
HEIGHT_AI_APPLY_DT tick and only ever writes result.target_down_mm to the
servo when result.apply is True. Manual D-pad input always wins — the
caller passes its current height_state string in, and this module never
inspects the AI-specific state itself, so the existing invariant that
height_step_callback sets height_state to the manual value on every call is
enough to guarantee manual priority.

Defense-in-depth clamp order for a proposal reaching the servo:
  proposal (untrusted)
    -> clamp_height_down_mm (height_table.py, 14-91 mm)
    -> deadband check (this module, servo-noise filter)
    -> rate limit (this module, mm/s)
    -> clamp_height_down_mm again (re-clamp after the rate-limited step)
    -> height_ticks_for_down_mm -> tick clamp -> GroupSyncWrite (existing,
       in eclipse_test_controller.py — outside this module)
"""

from dataclasses import dataclass
from typing import Optional

from eclipse_pkg.height_table import clamp_height_down_mm

HELD_MANUAL_PRIORITY = 'held_manual_priority'
HELD_STALE_PROPOSAL = 'held_stale_proposal'
HELD_WITHIN_DEADBAND = 'held_within_deadband'
APPLIED_RATE_LIMITED = 'applied_rate_limited'
APPLIED_FULL_STEP = 'applied_full_step'


@dataclass(frozen=True)
class HeightAiArbitrationResult:
    apply: bool
    target_down_mm: float
    reason: str


def arbitrate_height_ai_command(
    current_down_mm: float,
    height_state: str,
    manual_state: str,
    proposal_down_mm: Optional[float],
    proposal_fresh: bool,
    dt_sec: float,
    max_rate_mm_per_s: float,
    deadband_mm: float,
) -> HeightAiArbitrationResult:
    """Decide whether an AI height proposal may move the servo this tick."""
    current_down_mm = float(current_down_mm)

    if height_state == manual_state:
        return HeightAiArbitrationResult(
            apply=False,
            target_down_mm=current_down_mm,
            reason=HELD_MANUAL_PRIORITY,
        )

    if proposal_down_mm is None or not proposal_fresh:
        return HeightAiArbitrationResult(
            apply=False,
            target_down_mm=current_down_mm,
            reason=HELD_STALE_PROPOSAL,
        )

    clamped_proposal = clamp_height_down_mm(proposal_down_mm)
    delta = clamped_proposal - current_down_mm

    if abs(delta) <= deadband_mm:
        return HeightAiArbitrationResult(
            apply=False,
            target_down_mm=current_down_mm,
            reason=HELD_WITHIN_DEADBAND,
        )

    max_step = max(0.0, float(max_rate_mm_per_s)) * max(0.0, float(dt_sec))
    if abs(delta) > max_step:
        step = max_step if delta > 0 else -max_step
        target_down_mm = clamp_height_down_mm(current_down_mm + step)
        return HeightAiArbitrationResult(
            apply=True,
            target_down_mm=target_down_mm,
            reason=APPLIED_RATE_LIMITED,
        )

    target_down_mm = clamp_height_down_mm(current_down_mm + delta)
    return HeightAiArbitrationResult(
        apply=True,
        target_down_mm=target_down_mm,
        reason=APPLIED_FULL_STEP,
    )
