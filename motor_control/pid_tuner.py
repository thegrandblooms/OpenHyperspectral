"""
PID Auto-Tuner for OpenHyperspectral gimbal motor.

Uses Optuna (TPE Bayesian optimization) to systematically explore the PID
parameter space while streaming real-time encoder data to compute motor
performance metrics (rise time, overshoot, oscillation, smoothness, etc.).

Designed to be driven from spectrumboi.py via callback interface — the tuner
does NOT own the serial port; it receives send/read functions from the caller.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Dict, List, Optional

import numpy as np

try:
    import optuna
    from optuna.importance import get_param_importances

    optuna.logging.set_verbosity(optuna.logging.WARNING)
except ImportError:
    optuna = None  # type: ignore[assignment]

if TYPE_CHECKING:
    import optuna as _optuna_type  # noqa: F811

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class MoveMetrics:
    """Performance metrics for a single motor move."""
    rise_time_s: float = 0.0
    settling_time_s: float = 0.0
    overshoot_deg: float = 0.0
    overshoot_ratio: float = 0.0
    oscillation_count: int = 0
    steady_state_error_deg: float = 0.0
    settle_accuracy_deg: float = 0.0
    smoothness: float = 0.0
    accel_variance: float = 0.0
    move_distance_deg: float = 0.0
    completed: bool = False
    timeout: bool = False


@dataclass
class TrialResult:
    """Aggregated result for one Optuna trial (full movement pattern)."""
    trial_num: int = 0
    score: float = 0.0
    params: Dict[str, float] = field(default_factory=dict)
    metrics: Dict[str, float] = field(default_factory=dict)
    move_metrics: List[MoveMetrics] = field(default_factory=list)


# ---------------------------------------------------------------------------
# MovementAnalyzer
# ---------------------------------------------------------------------------

class MovementAnalyzer:
    """Compute performance metrics from encoder time-series for a single move."""

    def __init__(
        self,
        position_tolerance_deg: float = 0.5,
        settling_window_s: float = 0.2,
        steady_state_window_s: float = 0.5,
    ):
        self.pos_tol = position_tolerance_deg
        self.settling_window = settling_window_s
        self.ss_window = steady_state_window_s

    @staticmethod
    def _angle_error(pos: float, target: float) -> float:
        """Signed shortest-path error accounting for 0/360 wraparound."""
        err = pos - target
        if err > 180.0:
            err -= 360.0
        elif err < -180.0:
            err += 360.0
        return err

    def analyze_move(
        self,
        samples: List[dict],
        start_position_deg: float,
        target_deg: float,
    ) -> MoveMetrics:
        m = MoveMetrics()

        if len(samples) < 3:
            m.timeout = True
            return m

        ts = np.array([s["timestamp_ms"] for s in samples], dtype=np.float64) / 1000.0
        pos = np.array([s["position_deg"] for s in samples], dtype=np.float64)
        vel = np.array([s["velocity_deg_s"] for s in samples], dtype=np.float64)

        ts -= ts[0]  # Relative time from move start
        duration = ts[-1] if ts[-1] > 0 else 1.0

        # Signed move distance (shortest path)
        move_dist = self._angle_error(target_deg, start_position_deg)
        m.move_distance_deg = abs(move_dist)

        if m.move_distance_deg < 0.1:
            m.completed = True
            return m

        # Error array (signed, shortest path)
        error = np.array([self._angle_error(p, target_deg) for p in pos])
        abs_error = np.abs(error)

        # -- Rise time: time to reach 90% of move distance --
        displacement = np.array(
            [self._angle_error(p, start_position_deg) for p in pos]
        )
        threshold_90 = 0.9 * move_dist
        if move_dist > 0:
            crossed = np.where(displacement >= threshold_90)[0]
        else:
            crossed = np.where(displacement <= threshold_90)[0]
        m.rise_time_s = float(ts[crossed[0]]) if len(crossed) > 0 else duration

        # -- Settling time & settle accuracy --
        settle_start_idx = None
        for i in range(len(ts)):
            if abs_error[i] < self.pos_tol and abs(vel[i]) < 1.0:
                if settle_start_idx is None:
                    settle_start_idx = i
                elif ts[i] - ts[settle_start_idx] >= self.settling_window:
                    m.settling_time_s = float(ts[settle_start_idx])
                    m.settle_accuracy_deg = float(abs_error[settle_start_idx])
                    m.completed = True
                    break
            else:
                settle_start_idx = None

        if not m.completed:
            m.settling_time_s = duration
            m.settle_accuracy_deg = float(abs_error[-1])
            m.timeout = True

        # -- Overshoot: how far past the target in the direction of movement --
        if move_dist > 0:
            # Positive move: overshoot = max position - target (if position exceeds target)
            m.overshoot_deg = max(0.0, float(np.max(pos) - target_deg))
        else:
            # Negative move: overshoot = target - min position (if position undershoots target)
            m.overshoot_deg = max(0.0, float(target_deg - np.min(pos)))
        m.overshoot_ratio = m.overshoot_deg / m.move_distance_deg

        # -- Oscillation count: sign changes of error after first arrival --
        first_arrival = np.where(abs_error < self.pos_tol * 2)[0]
        if len(first_arrival) > 0:
            post_arrival_error = error[first_arrival[0] :]
            if len(post_arrival_error) > 1:
                signs = np.sign(post_arrival_error)
                signs = signs[signs != 0]  # Remove exact zeros
                if len(signs) > 1:
                    sign_changes = np.sum(np.diff(signs) != 0)
                    m.oscillation_count = int(sign_changes // 2)

        # -- Steady-state error: mean |error| over final ss_window --
        ss_mask = ts >= (ts[-1] - self.ss_window)
        if np.any(ss_mask):
            m.steady_state_error_deg = float(np.mean(abs_error[ss_mask]))

        # -- Smoothness: integral of |jerk| --
        dt = np.diff(ts)
        valid_dt = dt > 0
        if np.sum(valid_dt) > 2:
            accel = np.diff(vel)[valid_dt[: len(np.diff(vel))]] / dt[
                valid_dt[: len(np.diff(vel))]
            ]
            m.accel_variance = float(np.var(accel)) if len(accel) > 1 else 0.0

            dt2 = dt[1:][valid_dt[1 : len(accel) + 1] if len(accel) > 1 else []]
            if len(accel) > 1 and len(dt2) > 0:
                valid_dt2 = dt2 > 0
                if np.any(valid_dt2):
                    jerk = np.diff(accel)[: len(dt2)][valid_dt2] / dt2[valid_dt2]
                    ts_jerk = ts[2 : 2 + len(jerk)]
                    if len(jerk) > 1 and len(ts_jerk) == len(jerk):
                        _trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))
                        m.smoothness = float(_trapz(np.abs(jerk), ts_jerk))

        return m


# ---------------------------------------------------------------------------
# MotorTuner
# ---------------------------------------------------------------------------

class MotorTuner:
    """Optuna-based PID auto-tuner that communicates via callbacks."""

    # Parameter search space
    PARAM_RANGES = {
        "pid_p_pos": (1.0, 50.0, "linear"),
        "pid_d_pos": (0.0, 2.0, "linear"),
        "pid_i_pos": (0.0, 5.0, "linear"),
        "pid_p_vel": (0.05, 1.0, "linear"),
        "pid_i_vel": (0.5, 20.0, "linear"),
        "lpf_vel": (0.001, 0.1, "log"),
    }

    # Test move distances (degrees). Each distance is tested from 4 quadrant
    # positions (0, 90, 180, 270) to average out encoder/motor eccentricity.
    MOVE_DISTANCES = [+30.0, -30.0, +60.0, -60.0, +10.0, -10.0]
    QUADRANT_ORIGINS = [0.0, 90.0, 180.0, 270.0]

    # Scoring weights (lower total = better)
    WEIGHTS = {
        "rise_time": 1.0,
        "settling_time": 3.0,
        "overshoot_ratio": 10.0,
        "oscillation_count": 2.0,
        "smoothness": 0.5,
        "steady_state_error": 5.0,
        "settle_accuracy": 8.0,
    }

    # Safety
    MAX_OVERSHOOT_DEG = 15.0
    MAX_OSCILLATIONS = 10
    MOVE_TIMEOUT_S = 5.0
    SETTLE_DETECT_S = 0.3
    INTER_MOVE_DELAY_S = 0.5
    PENALTY_SCORE = 1000.0

    def __init__(
        self,
        send_fn: Callable[[str], None],
        set_param_fn: Callable[[str, float], None],
        get_stream_fn: Callable[[], List[str]],
        on_trial_complete_fn: Callable[[int, float, dict, dict], None],
        n_trials: int = 50,
        timeout_minutes: float = 10.0,
    ):
        self.send_fn = send_fn
        self.set_param_fn = set_param_fn
        self.get_stream_fn = get_stream_fn
        self.on_trial_complete_fn = on_trial_complete_fn
        self.n_trials = n_trials
        self.timeout_minutes = timeout_minutes

        self.analyzer = MovementAnalyzer()
        self.study: Optional[optuna.Study] = None
        self._origin_deg: float = 0.0
        self._start_time: float = 0.0
        self._enc_buffer: List[dict] = []

    # -- Public API --

    def run(self, should_continue_fn: Callable[[], bool]) -> None:
        """Run the full optimization. Blocks until done or stopped."""
        if optuna is None:
            raise ImportError("optuna is required: pip install optuna")

        self._start_time = time.time()
        self._should_continue = should_continue_fn

        # Flush any stale encoder data
        self.get_stream_fn()
        time.sleep(0.3)

        # Determine current position as origin
        self._origin_deg = self._read_current_position()
        logger.info("PID tuner origin: %.1f deg", self._origin_deg)

        # Create study
        sampler = optuna.samplers.TPESampler(
            n_startup_trials=min(10, self.n_trials // 2),
            multivariate=True,
            seed=42,
        )
        pruner = optuna.pruners.MedianPruner(
            n_startup_trials=5,
            n_warmup_steps=2,
        )
        self.study = optuna.create_study(
            study_name="pid_tuning",
            direction="minimize",
            sampler=sampler,
            pruner=pruner,
        )

        # Optimize with callbacks
        self.study.optimize(
            self._objective,
            n_trials=self.n_trials,
            timeout=self.timeout_minutes * 60,
            callbacks=[self._check_stop_callback],
        )

    def get_param_importance(self) -> Dict[str, float]:
        """Return parameter importance dict from completed study."""
        if self.study is None or len(self.study.trials) < 5:
            return {}
        try:
            return get_param_importances(self.study)
        except Exception:
            return {}

    # -- Optuna objective --

    def _objective(self, trial: optuna.Trial) -> float:
        if not self._should_continue():
            raise optuna.TrialPruned()

        # Sample parameters
        params = {}
        for key, (lo, hi, scale) in self.PARAM_RANGES.items():
            if scale == "log":
                params[key] = trial.suggest_float(key, lo, hi, log=True)
            else:
                params[key] = trial.suggest_float(key, lo, hi)

        # Apply parameters to firmware
        for key, value in params.items():
            self.set_param_fn(key, value)
        time.sleep(0.15)  # Let firmware process all set commands

        # Run movement pattern: for each move distance, test from 4 quadrant
        # positions (0, 90, 180, 270) to average out eccentricity errors.
        all_metrics: List[MoveMetrics] = []
        score_sum = 0.0
        step_idx = 0

        # Flush encoder buffer before starting moves
        self._flush_and_collect_enc()

        for dist in self.MOVE_DISTANCES:
            quadrant_metrics: List[MoveMetrics] = []

            for origin in self.QUADRANT_ORIGINS:
                if not self._should_continue():
                    raise optuna.TrialPruned()

                # Move to quadrant origin first
                origin_pos = self._wrap_angle(origin)
                self.send_fn(f"m {origin_pos:.2f}")
                self._wait_for_settle(origin_pos)
                time.sleep(0.15)

                # Now execute the test move from this quadrant
                target = self._wrap_angle(origin + dist)
                start_pos = self._read_current_position()
                self._enc_buffer.clear()
                self.send_fn(f"m {target:.2f}")

                samples = self._wait_for_settle(target)
                metrics = self.analyzer.analyze_move(samples, start_pos, target)
                quadrant_metrics.append(metrics)

                # Safety abort
                if metrics.overshoot_deg > self.MAX_OVERSHOOT_DEG:
                    return self.PENALTY_SCORE
                if metrics.oscillation_count > self.MAX_OSCILLATIONS:
                    return self.PENALTY_SCORE

                time.sleep(self.INTER_MOVE_DELAY_S)

            # Average the 4 quadrant results for this move distance
            avg_m = self._average_metrics(quadrant_metrics)
            all_metrics.append(avg_m)

            move_score = self._score_move(avg_m)
            score_sum += move_score
            step_idx += 1

            # Report intermediate for pruning (one step per move distance)
            trial.report(score_sum / step_idx, step=step_idx - 1)
            if trial.should_prune():
                raise optuna.TrialPruned()

        # Aggregate
        avg_score = score_sum / len(self.MOVE_DISTANCES)

        metrics_summary = {
            "avg_rise_time": float(np.mean([m.rise_time_s for m in all_metrics])),
            "avg_settling_time": float(
                np.mean([m.settling_time_s for m in all_metrics])
            ),
            "avg_overshoot": float(np.mean([m.overshoot_deg for m in all_metrics])),
            "avg_oscillations": float(
                np.mean([m.oscillation_count for m in all_metrics])
            ),
            "avg_ss_error": float(
                np.mean([m.steady_state_error_deg for m in all_metrics])
            ),
            "avg_settle_accuracy": float(
                np.mean([m.settle_accuracy_deg for m in all_metrics])
            ),
            "avg_smoothness": float(np.mean([m.smoothness for m in all_metrics])),
        }
        trial.set_user_attr("metrics", metrics_summary)

        # Notify UI
        self.on_trial_complete_fn(trial.number + 1, avg_score, params, metrics_summary)

        return avg_score

    # -- Scoring --

    def _score_move(self, m: MoveMetrics) -> float:
        w = self.WEIGHTS
        score = (
            w["rise_time"] * m.rise_time_s
            + w["settling_time"] * m.settling_time_s
            + w["overshoot_ratio"] * m.overshoot_ratio
            + w["oscillation_count"] * m.oscillation_count
            + w["smoothness"] * min(m.smoothness, 100.0)
            + w["steady_state_error"] * m.steady_state_error_deg
            + w["settle_accuracy"] * m.settle_accuracy_deg
        )
        if not m.completed or m.timeout:
            score += 50.0
        return score

    @staticmethod
    def _average_metrics(metrics_list: List[MoveMetrics]) -> MoveMetrics:
        """Average MoveMetrics across quadrant positions to cancel eccentricity."""
        n = len(metrics_list)
        if n == 0:
            return MoveMetrics(timeout=True)
        if n == 1:
            return metrics_list[0]
        return MoveMetrics(
            rise_time_s=sum(m.rise_time_s for m in metrics_list) / n,
            settling_time_s=sum(m.settling_time_s for m in metrics_list) / n,
            overshoot_deg=sum(m.overshoot_deg for m in metrics_list) / n,
            overshoot_ratio=sum(m.overshoot_ratio for m in metrics_list) / n,
            oscillation_count=round(sum(m.oscillation_count for m in metrics_list) / n),
            steady_state_error_deg=sum(m.steady_state_error_deg for m in metrics_list) / n,
            settle_accuracy_deg=sum(m.settle_accuracy_deg for m in metrics_list) / n,
            smoothness=sum(m.smoothness for m in metrics_list) / n,
            accel_variance=sum(m.accel_variance for m in metrics_list) / n,
            move_distance_deg=sum(m.move_distance_deg for m in metrics_list) / n,
            completed=all(m.completed for m in metrics_list),
            timeout=any(m.timeout for m in metrics_list),
        )

    # -- Encoder data handling --

    def _flush_and_collect_enc(self) -> List[dict]:
        """Drain $ENC lines from stream, parse, return as dicts."""
        raw_lines = self.get_stream_fn()
        parsed = []
        for line in raw_lines:
            d = self._parse_enc_line(line)
            if d is not None:
                parsed.append(d)
        return parsed

    @staticmethod
    def _parse_enc_line(line: str) -> Optional[dict]:
        """Parse '$ENC,ts,pos,vel,target' into dict."""
        if not line.startswith("$ENC,"):
            return None
        try:
            parts = line[5:].split(",")
            if len(parts) < 4:
                return None
            return {
                "timestamp_ms": int(parts[0]),
                "position_deg": float(parts[1]),
                "velocity_deg_s": float(parts[2]),
                "target_deg": float(parts[3]),
            }
        except (ValueError, IndexError):
            return None

    def _read_current_position(self) -> float:
        """Read current position from encoder stream. Waits up to 0.5s."""
        for _ in range(25):  # 25 * 20ms = 500ms
            samples = self._flush_and_collect_enc()
            if samples:
                return samples[-1]["position_deg"]
            time.sleep(0.02)
        return self._origin_deg

    def _wait_for_settle(self, target_deg: float) -> List[dict]:
        """Wait for motor to reach target, collecting encoder samples."""
        collected: List[dict] = []
        start_time = time.time()
        settle_start: Optional[float] = None

        while time.time() - start_time < self.MOVE_TIMEOUT_S:
            time.sleep(0.02)

            new_samples = self._flush_and_collect_enc()
            collected.extend(new_samples)

            if not collected:
                continue

            latest = collected[-1]
            error = abs(MovementAnalyzer._angle_error(
                latest["position_deg"], target_deg
            ))
            vel = abs(latest["velocity_deg_s"])

            if error < self.analyzer.pos_tol and vel < 1.0:
                if settle_start is None:
                    settle_start = time.time()
                elif time.time() - settle_start >= self.SETTLE_DETECT_S:
                    return collected
            else:
                settle_start = None

        return collected

    # -- Helpers --

    @staticmethod
    def _wrap_angle(deg: float) -> float:
        """Wrap angle to [0, 360)."""
        return deg % 360.0

    def _check_stop_callback(self, study: optuna.Study, trial) -> None:
        """Optuna callback: stop if time exceeded or externally stopped."""
        elapsed = time.time() - self._start_time
        if elapsed > self.timeout_minutes * 60:
            study.stop()
        if not self._should_continue():
            study.stop()
