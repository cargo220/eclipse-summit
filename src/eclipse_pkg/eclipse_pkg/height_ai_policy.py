"""Pluggable height-decision policy for TARS.

eclipse_test_controller calls HeightPolicy.propose() in-process and only
ever writes the servo through height_ai_arbitration.py. The learned
artifact is an outcome predictor f(s, h') -> (delta_ekf, delta_traction).
propose() is the grid search around that predictor: score candidate
heights and penalize predicted EKF drops.

A missing or invalid checkpoint falls back to StubHoldPolicy. A skeleton
checkpoint (zero weights) loads the grid path but still holds the current
height, so launch wiring can be tested before any training run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

from eclipse_pkg.height_table import HEIGHT_MAX_DOWN_MM, HEIGHT_MIN_DOWN_MM


SCHEMA_ID = "tars-height-outcome-v1"
HEIGHT_GRID_STEP_MM = 5.0
DEFAULT_STATE_FEATURES = (
    "cmd_v",
    "cmd_w",
    "ekf_speed",
    "wheel_odom_speed",
    "traction_efficiency",
    "height_current_down_mm",
    "current_std_avg",
    "probe_angle",
)


def dataset_height_grid_mm():
    """14–91 mm table range at 5 mm, plus the unaligned max if needed."""
    step = float(HEIGHT_GRID_STEP_MM)
    lo = float(HEIGHT_MIN_DOWN_MM)
    hi = float(HEIGHT_MAX_DOWN_MM)
    if step <= 0.0:
        return (lo,)
    count = int((hi - lo) // step)
    points = [round(lo + i * step, 6) for i in range(count + 1)]
    if points[-1] < hi - 1e-9:
        points.append(hi)
    return tuple(points)


DEFAULT_CANDIDATES_MM = dataset_height_grid_mm()
OUTCOME_NAMES = ("delta_ekf_speed", "delta_traction_efficiency")
DEFAULT_EKF_DROP_WEIGHT = 1.0


@dataclass(frozen=True)
class HeightObservation:
    """One policy input sample.

    ``state`` uses the same key names as
    eclipse_ai_controller.height_ai_state_snapshot().
    """

    current_down_mm: float
    height_state: str
    stamp_sec: float
    state: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class HeightProposal:
    down_mm: float
    confidence: float
    reason: str
    source: str


class HeightPolicy(Protocol):
    def propose(self, observation: HeightObservation) -> HeightProposal: ...


def stub_hold_policy(observation: HeightObservation) -> HeightProposal:
    """Pure function: always propose holding the current height."""
    return HeightProposal(
        down_mm=observation.current_down_mm,
        confidence=0.0,
        reason="stub_hold",
        source="stub",
    )


class StubHoldPolicy:
    """Policy wrapper around stub_hold_policy for the HeightPolicy protocol."""

    def propose(self, observation: HeightObservation) -> HeightProposal:
        return stub_hold_policy(observation)


class OutcomeGridPolicy:
    """Score a height grid with a linear outcome predictor and pick one."""

    def __init__(
        self,
        feature_names: Sequence[str],
        feature_mean: Sequence[float],
        feature_scale: Sequence[float],
        weights: Sequence[Sequence[float]],
        intercept: Sequence[float],
        candidates_mm: Sequence[float],
        source: str,
        ekf_drop_weight: float = DEFAULT_EKF_DROP_WEIGHT,
    ):
        self.feature_names = tuple(feature_names)
        n_in = len(self.feature_names) + 1
        if len(feature_mean) != n_in or len(feature_scale) != n_in:
            raise ValueError("feature_mean/scale must cover state features plus h'")
        if len(weights) != len(OUTCOME_NAMES) or any(len(row) != n_in for row in weights):
            raise ValueError("weights must be (2, n_features+1)")
        if len(intercept) != len(OUTCOME_NAMES):
            raise ValueError("intercept must have one entry per outcome")
        self.feature_mean = tuple(float(v) for v in feature_mean)
        self.feature_scale = tuple(float(v) for v in feature_scale)
        self.weights = tuple(tuple(float(v) for v in row) for row in weights)
        self.intercept = tuple(float(v) for v in intercept)
        self.candidates_mm = tuple(float(v) for v in candidates_mm)
        if not self.candidates_mm:
            raise ValueError("candidates_mm must not be empty")
        self.source = str(source)
        self.ekf_drop_weight = float(ekf_drop_weight)

    def predict(self, state: Mapping[str, Any], target_down_mm: float) -> tuple[float, float]:
        raw = [_state_number(state, name) for name in self.feature_names]
        raw.append(float(target_down_mm))
        features = []
        for value, mean, scale in zip(raw, self.feature_mean, self.feature_scale):
            if scale == 0.0:
                features.append(0.0)
            else:
                features.append((value - mean) / scale)
        outcomes = []
        for row, bias in zip(self.weights, self.intercept):
            outcomes.append(bias + sum(w * x for w, x in zip(row, features)))
        return outcomes[0], outcomes[1]

    def propose(self, observation: HeightObservation) -> HeightProposal:
        current = float(observation.current_down_mm)
        best_down_mm = current
        best_reward = None
        best_delta_ekf = 0.0
        for candidate in self.candidates_mm:
            delta_ekf, _delta_traction = self.predict(observation.state, candidate)
            reward = _ekf_reward(delta_ekf, self.ekf_drop_weight)
            if best_reward is None or reward > best_reward + 1e-12:
                best_reward = reward
                best_down_mm = candidate
                best_delta_ekf = delta_ekf
            elif best_reward is not None and abs(reward - best_reward) <= 1e-12:
                if abs(candidate - current) < abs(best_down_mm - current):
                    best_down_mm = candidate
                    best_delta_ekf = delta_ekf
        return HeightProposal(
            down_mm=best_down_mm,
            confidence=0.0,
            reason=f"grid_max_ekf:{best_delta_ekf:+.4f}",
            source=self.source,
        )


def write_skeleton_checkpoint(path: str | Path) -> Path:
    """Write a zero-weight checkpoint that loads but still holds height."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    n_in = len(DEFAULT_STATE_FEATURES) + 1
    payload = {
        "schema": SCHEMA_ID,
        "feature_names": list(DEFAULT_STATE_FEATURES),
        "feature_mean": [0.0] * n_in,
        "feature_scale": [1.0] * n_in,
        "weights": [[0.0] * n_in, [0.0] * n_in],
        "intercept": [0.0, 0.0],
        "outcome_names": list(OUTCOME_NAMES),
        "candidates_mm": list(DEFAULT_CANDIDATES_MM),
        "ekf_drop_weight": DEFAULT_EKF_DROP_WEIGHT,
    }
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return out


def load_policy(model_path: str, logger=None) -> HeightPolicy:
    """Resolve the active policy for model_path, falling back to the stub."""
    model_path = (model_path or "").strip()
    if not model_path:
        return StubHoldPolicy()

    import os

    if not os.path.isfile(model_path):
        if logger is not None:
            logger.warn(
                f'height AI: model_path "{model_path}" not found, '
                "falling back to stub policy"
            )
        return StubHoldPolicy()

    try:
        return _load_model_policy(model_path)
    except Exception as exc:  # noqa: BLE001 — any load failure must fall back
        if logger is not None:
            logger.error(
                f'height AI: failed to load model "{model_path}" '
                f"({exc}), falling back to stub policy"
            )
        return StubHoldPolicy()


def _load_model_policy(model_path: str) -> OutcomeGridPolicy:
    with open(model_path, encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError("checkpoint must be a JSON object")
    schema = payload.get("schema")
    if schema != SCHEMA_ID:
        raise ValueError(f"unsupported checkpoint schema: {schema!r}")
    outcome_names = tuple(payload.get("outcome_names") or ())
    if outcome_names != OUTCOME_NAMES:
        raise ValueError(f"outcome_names must be {OUTCOME_NAMES}, got {outcome_names}")
    return OutcomeGridPolicy(
        feature_names=payload["feature_names"],
        feature_mean=payload["feature_mean"],
        feature_scale=payload["feature_scale"],
        weights=payload["weights"],
        intercept=payload["intercept"],
        candidates_mm=payload.get("candidates_mm", DEFAULT_CANDIDATES_MM),
        source=f"model:{model_path}",
        ekf_drop_weight=float(payload.get("ekf_drop_weight", DEFAULT_EKF_DROP_WEIGHT)),
    )


def _ekf_reward(delta_ekf: float, drop_weight: float) -> float:
    if delta_ekf < 0.0:
        return delta_ekf * (1.0 + max(drop_weight, 0.0))
    return delta_ekf


def _state_number(state: Mapping[str, Any], name: str) -> float:
    value = state.get(name)
    if isinstance(value, bool) or value is None:
        return 0.0
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if number != number:  # NaN
        return 0.0
    return number
