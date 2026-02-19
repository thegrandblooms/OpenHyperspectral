"""
PID Auto-Tuner for OpenHyperspectral gimbal motor.

Uses Optuna (TPE Bayesian optimization) to systematically explore the PID
parameter space while streaming real-time encoder data to compute motor
performance metrics (rise time, overshoot, oscillation, smoothness, etc.).

Designed to be driven from spectrumboi.py via callback interface — the tuner
does NOT own the serial port; it receives send/read functions from the caller.
"""

from __future__ import annotations

import json
import logging
import os
import time
import webbrowser
from dataclasses import dataclass, field
from datetime import datetime
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
    velocity_reversals: int = 0
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
        # Use displacement (wrap-safe) to find how far past the target we went.
        if move_dist > 0:
            m.overshoot_deg = max(0.0, float(np.max(displacement) - move_dist))
        else:
            m.overshoot_deg = max(0.0, float(move_dist - np.min(displacement)))
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

        # -- Velocity reversals: sign flips in velocity (catches vibration
        #    even when motor never reaches target) --
        vel_signs = np.sign(vel)
        vel_signs = vel_signs[vel_signs != 0]  # drop zero-velocity samples
        if len(vel_signs) > 1:
            m.velocity_reversals = int(np.sum(np.diff(vel_signs) != 0))

        # -- Steady-state error: mean |error| over final ss_window --
        ss_mask = ts >= (ts[-1] - self.ss_window)
        if np.any(ss_mask):
            m.steady_state_error_deg = float(np.mean(abs_error[ss_mask]))

        # -- Smoothness: integral of |jerk| --
        # Resample at valid (non-zero dt) intervals to keep arrays aligned.
        try:
            dt = np.diff(ts)
            ok = dt > 0
            if np.sum(ok) > 2:
                dt_ok = dt[ok]
                vel_ok = vel[:-1][ok]  # velocities at start of each interval
                vel_next = vel[1:][ok]
                accel = (vel_next - vel_ok) / dt_ok
                m.accel_variance = float(np.var(accel)) if len(accel) > 1 else 0.0

                if len(accel) > 2:
                    dt_a = dt_ok[1:]  # intervals between accel samples
                    jerk = np.diff(accel) / dt_a
                    ok_j = dt_a > 0
                    if np.any(ok_j):
                        # Timestamps for jerk samples (midpoints of accel intervals)
                        ts_ok = ts[:-1][ok]
                        ts_j = 0.5 * (ts_ok[1:-1] + ts_ok[2:])
                        jerk_v = np.abs(jerk[ok_j[: len(jerk)]])
                        ts_j = ts_j[: len(jerk_v)]
                        if len(jerk_v) > 1 and len(ts_j) == len(jerk_v):
                            _trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))
                            m.smoothness = float(_trapz(jerk_v, ts_j))
        except (IndexError, ValueError):
            pass  # Non-fatal — leave smoothness at default 0.0

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

    # Test movement sequence: walk through quadrant positions with alternating
    # forward/backward direction. Covers all 4 quadrants (cancels eccentricity)
    # and exercises both CW and CCW movement.
    # 0 → 90(fwd) → 180(back) → 270(fwd) → 0(back) → repeat
    MOVE_SEQUENCE = [0.0, 90.0, 180.0, 270.0, 0.0]

    # Scoring weights (lower total = better)
    WEIGHTS = {
        "rise_time": 1.0,
        "settling_time": 3.0,
        "overshoot_ratio": 10.0,
        "oscillation_count": 2.0,
        "velocity_reversals": 3.0,
        "smoothness": 0.5,
        "steady_state_error": 5.0,
        "settle_accuracy": 8.0,
    }

    # Safety
    MAX_OVERSHOOT_DEG = 15.0
    MAX_OSCILLATIONS = 10
    MAX_VELOCITY_REVERSALS = 5
    MOVE_TIMEOUT_S = 2.0
    SETTLE_DETECT_S = 0.3
    INTER_MOVE_DELAY_S = 0.5
    PENALTY_SCORE = 1000.0

    def __init__(
        self,
        send_fn: Callable[[str], None],
        set_param_fn: Callable[[str, float], None],
        get_stream_fn: Callable[[], List[str]],
        on_trial_complete_fn: Callable[[int, float, dict, dict], None],
        on_trial_start_fn: Optional[Callable[[int, dict], None]] = None,
        n_trials: int = 50,
        timeout_minutes: float = 10.0,
        move_timeout_s: float = MOVE_TIMEOUT_S,
    ):
        self.send_fn = send_fn
        self.set_param_fn = set_param_fn
        self.get_stream_fn = get_stream_fn
        self.on_trial_complete_fn = on_trial_complete_fn
        self.on_trial_start_fn = on_trial_start_fn
        self.n_trials = n_trials
        self.timeout_minutes = timeout_minutes
        self.MOVE_TIMEOUT_S = move_timeout_s

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

        # Notify UI that a new trial is starting (with the params we're about to test)
        if self.on_trial_start_fn is not None:
            self.on_trial_start_fn(trial.number + 1, params)

        # Apply parameters to firmware
        for key, value in params.items():
            self.set_param_fn(key, value)
        time.sleep(0.15)  # Let firmware process all set commands

        # Walk through quadrant positions: 0→90→180→270→0
        # Alternates forward/backward, covers all quadrants (cancels eccentricity).
        all_metrics: List[MoveMetrics] = []
        score_sum = 0.0
        n_moves = len(self.MOVE_SEQUENCE) - 1  # transitions between positions

        # Flush encoder buffer before starting moves
        self._flush_and_collect_enc()

        # Move to starting position
        self.send_fn(f"m {self.MOVE_SEQUENCE[0]:.2f}")
        self._wait_for_settle(self.MOVE_SEQUENCE[0])
        time.sleep(0.15)

        for i in range(n_moves):
            if not self._should_continue():
                raise optuna.TrialPruned()

            target = self.MOVE_SEQUENCE[i + 1]
            start_pos = self._read_current_position()
            self._enc_buffer.clear()
            self.send_fn(f"m {target:.2f}")

            samples = self._wait_for_settle(target)
            try:
                metrics = self.analyzer.analyze_move(samples, start_pos, target)
            except Exception as exc:
                logger.warning("analyze_move error (move %d): %s", i, exc)
                metrics = MoveMetrics(timeout=True)
            all_metrics.append(metrics)

            # Safety abort — still count as a completed trial so UI stays updated
            if metrics.overshoot_deg > self.MAX_OVERSHOOT_DEG:
                self._notify_trial_complete(
                    trial, self.PENALTY_SCORE, params, all_metrics, n_moves,
                    abort_reason=f"overshoot {metrics.overshoot_deg:.1f}°")
                return self.PENALTY_SCORE
            if metrics.oscillation_count > self.MAX_OSCILLATIONS:
                self._notify_trial_complete(
                    trial, self.PENALTY_SCORE, params, all_metrics, n_moves,
                    abort_reason=f"{metrics.oscillation_count} oscillations")
                return self.PENALTY_SCORE
            if metrics.velocity_reversals > self.MAX_VELOCITY_REVERSALS:
                self._notify_trial_complete(
                    trial, self.PENALTY_SCORE, params, all_metrics, n_moves,
                    abort_reason=f"{metrics.velocity_reversals} vel reversals")
                return self.PENALTY_SCORE

            move_score = self._score_move(metrics)
            score_sum += move_score

            # Report intermediate for pruning
            trial.report(score_sum / (i + 1), step=i)
            if trial.should_prune():
                raise optuna.TrialPruned()

            time.sleep(self.INTER_MOVE_DELAY_S)

        # Aggregate
        avg_score = score_sum / n_moves
        self._notify_trial_complete(trial, avg_score, params, all_metrics, n_moves)

        return avg_score

    def _notify_trial_complete(
        self, trial, score, params, all_metrics, n_moves, abort_reason=None,
    ):
        """Build metrics summary and notify UI callback."""
        metrics_summary = {
            "avg_rise_time": float(np.mean([m.rise_time_s for m in all_metrics])),
            "avg_settling_time": float(
                np.mean([m.settling_time_s for m in all_metrics])
            ),
            "avg_overshoot": float(np.mean([m.overshoot_deg for m in all_metrics])),
            "avg_oscillations": float(
                np.mean([m.oscillation_count for m in all_metrics])
            ),
            "avg_vel_reversals": float(
                np.mean([m.velocity_reversals for m in all_metrics])
            ),
            "avg_ss_error": float(
                np.mean([m.steady_state_error_deg for m in all_metrics])
            ),
            "avg_settle_accuracy": float(
                np.mean([m.settle_accuracy_deg for m in all_metrics])
            ),
            "avg_smoothness": float(np.mean([m.smoothness for m in all_metrics])),
        }
        if abort_reason:
            metrics_summary["abort_reason"] = abort_reason
        trial.set_user_attr("metrics", metrics_summary)
        self.on_trial_complete_fn(trial.number + 1, score, params, metrics_summary)

    # -- Scoring --

    def _score_move(self, m: MoveMetrics) -> float:
        w = self.WEIGHTS
        score = (
            w["rise_time"] * m.rise_time_s
            + w["settling_time"] * m.settling_time_s
            + w["overshoot_ratio"] * m.overshoot_ratio
            + w["oscillation_count"] * m.oscillation_count
            + w["velocity_reversals"] * m.velocity_reversals
            + w["smoothness"] * min(m.smoothness, 100.0)
            + w["steady_state_error"] * m.steady_state_error_deg
            + w["settle_accuracy"] * m.settle_accuracy_deg
        )
        if not m.completed or m.timeout:
            score += 50.0
        return score

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

    # -- HTML report --

    def generate_report(
        self,
        trial_scores: List[tuple],
        best_params: Dict[str, float],
        best_metrics: Dict[str, float],
        param_importance: Dict[str, float],
        output_dir: str = ".",
    ) -> str:
        """Generate a self-contained HTML report and return the file path."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pid_tuning_report_{timestamp}.html"
        filepath = os.path.join(output_dir, filename)

        # Collect per-trial data from the Optuna study for richer charts
        trial_data = []
        if self.study is not None:
            for t in self.study.trials:
                if t.state.name == "COMPLETE":
                    trial_data.append({
                        "number": t.number + 1,
                        "score": t.value,
                        "params": t.params,
                        "metrics": t.user_attrs.get("metrics", {}),
                    })

        # Fall back to the simple (trial_num, score) list if study unavailable
        scores_json = json.dumps(
            [{"trial": n, "score": s} for n, s in trial_scores])
        trial_data_json = json.dumps(trial_data)
        best_params_json = json.dumps(best_params, indent=2)
        best_metrics_json = json.dumps(best_metrics, indent=2)
        importance_json = json.dumps(
            sorted(param_importance.items(), key=lambda x: x[1], reverse=True))

        best_score = min((s for _, s in trial_scores), default=0)
        n_trials = len(trial_scores)
        duration_min = (time.time() - self._start_time) / 60 if self._start_time else 0

        html = _REPORT_TEMPLATE.format(
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            n_trials=n_trials,
            best_score=f"{best_score:.3f}",
            duration_min=f"{duration_min:.1f}",
            scores_json=scores_json,
            trial_data_json=trial_data_json,
            best_params_json=best_params_json,
            best_metrics_json=best_metrics_json,
            importance_json=importance_json,
            move_sequence=" → ".join(f"{p:.0f}°" for p in self.MOVE_SEQUENCE),
            move_timeout_s=self.MOVE_TIMEOUT_S,
        )

        with open(filepath, "w") as f:
            f.write(html)
        logger.info("PID tuning report saved to %s", filepath)
        return filepath


# ---------------------------------------------------------------------------
# HTML report template (self-contained, uses Chart.js from CDN)
# ---------------------------------------------------------------------------

_REPORT_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PID Tuning Report — {timestamp}</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<style>
  :root {{
    --bg: #1a1a2e; --surface: #16213e; --card: #0f3460;
    --accent: #e94560; --accent2: #53d8fb; --text: #eee; --muted: #889;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    font-family: 'Segoe UI', system-ui, sans-serif;
    background: var(--bg); color: var(--text);
    padding: 24px; line-height: 1.5;
  }}
  h1 {{ color: var(--accent2); margin-bottom: 4px; font-size: 1.6em; }}
  h2 {{ color: var(--accent2); margin: 24px 0 12px; font-size: 1.2em;
        border-bottom: 1px solid #334; padding-bottom: 4px; }}
  .subtitle {{ color: var(--muted); font-size: 0.9em; margin-bottom: 20px; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
           gap: 16px; margin-bottom: 20px; }}
  .card {{
    background: var(--surface); border-radius: 10px; padding: 18px;
    border: 1px solid #253; box-shadow: 0 2px 8px rgba(0,0,0,0.3);
  }}
  .stat {{ text-align: center; }}
  .stat .value {{ font-size: 2em; font-weight: 700; color: var(--accent); }}
  .stat .label {{ color: var(--muted); font-size: 0.85em; }}
  .chart-container {{ position: relative; height: 260px; }}
  table {{ width: 100%; border-collapse: collapse; }}
  th, td {{ text-align: left; padding: 6px 12px; border-bottom: 1px solid #253; }}
  th {{ color: var(--accent2); font-weight: 600; font-size: 0.85em; }}
  td {{ font-family: 'Cascadia Code', 'Fira Code', monospace; font-size: 0.9em; }}
  .bar-cell {{ position: relative; }}
  .bar {{ position: absolute; left: 0; top: 2px; bottom: 2px;
          background: var(--accent); opacity: 0.25; border-radius: 3px; }}
  .param-name {{ color: var(--accent2); }}
  .good {{ color: #4ecdc4; }} .warn {{ color: #ffe66d; }} .bad {{ color: var(--accent); }}
  .copy-btn {{
    background: var(--card); color: var(--accent2); border: 1px solid var(--accent2);
    border-radius: 5px; padding: 6px 16px; cursor: pointer; font-size: 0.85em;
    margin-top: 8px; transition: 0.2s;
  }}
  .copy-btn:hover {{ background: var(--accent2); color: var(--bg); }}
  .footer {{ text-align: center; color: var(--muted); font-size: 0.8em; margin-top: 32px; }}
</style>
</head>
<body>

<h1>PID Auto-Tune Report</h1>
<p class="subtitle">{timestamp} &nbsp;|&nbsp; Move pattern: {move_sequence}
   &nbsp;|&nbsp; Timeout: {move_timeout_s}s/move</p>

<!-- Summary stats -->
<div class="grid">
  <div class="card stat">
    <div class="value">{n_trials}</div><div class="label">Trials</div>
  </div>
  <div class="card stat">
    <div class="value">{best_score}</div><div class="label">Best Score</div>
  </div>
  <div class="card stat">
    <div class="value">{duration_min} min</div><div class="label">Duration</div>
  </div>
</div>

<!-- Charts row -->
<div class="grid">
  <div class="card">
    <h2>Score vs Trial</h2>
    <div class="chart-container"><canvas id="scoreChart"></canvas></div>
  </div>
  <div class="card">
    <h2>Parameter Importance</h2>
    <div class="chart-container"><canvas id="importanceChart"></canvas></div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>Parameter Exploration</h2>
    <div class="chart-container"><canvas id="paramChart"></canvas></div>
  </div>
  <div class="card">
    <h2>Metrics Over Trials</h2>
    <div class="chart-container"><canvas id="metricsChart"></canvas></div>
  </div>
</div>

<!-- Best parameters table -->
<div class="card">
  <h2>Best Parameters</h2>
  <div id="paramsTable"></div>
  <button class="copy-btn" onclick="copyCommands()">Copy All Commands</button>
</div>

<!-- Best metrics -->
<div class="card">
  <h2>Best Performance Metrics</h2>
  <div id="metricsTable"></div>
</div>

<!-- All trials table -->
<div class="card">
  <h2>All Trials</h2>
  <div style="max-height:400px; overflow-y:auto;">
    <div id="allTrialsTable"></div>
  </div>
</div>

<p class="footer">Generated by OpenHyperspectral PID Auto-Tuner (Optuna TPE)</p>

<script>
const scores = {scores_json};
const trialData = {trial_data_json};
const bestParams = {best_params_json};
const bestMetrics = {best_metrics_json};
const importance = {importance_json};

// Shared chart defaults
Chart.defaults.color = '#889';
Chart.defaults.borderColor = '#253';
Chart.defaults.font.family = "'Segoe UI', system-ui, sans-serif";

// 1. Score vs Trial (with running-best overlay)
(() => {{
  if (!scores.length) {{
    document.getElementById('scoreChart').parentElement.innerHTML =
      '<p style="color:var(--muted);text-align:center;padding:40px">No trial data recorded</p>';
    return;
  }}
  const labels = scores.map(s => s.trial);
  const vals = scores.map(s => s.score);
  let runBest = []; let best = Infinity;
  vals.forEach(v => {{ best = Math.min(best, v); runBest.push(best); }});

  new Chart(document.getElementById('scoreChart'), {{
    type: 'line',
    data: {{
      labels,
      datasets: [
        {{ label: 'Trial Score', data: vals, borderColor: '#e94560',
           backgroundColor: 'rgba(233,69,96,0.1)', fill: true,
           pointRadius: 2, borderWidth: 1.5, tension: 0.1 }},
        {{ label: 'Best So Far', data: runBest, borderColor: '#53d8fb',
           borderWidth: 2, borderDash: [5,3], pointRadius: 0, fill: false }}
      ]
    }},
    options: {{
      responsive: true, maintainAspectRatio: false,
      plugins: {{ legend: {{ position: 'top', labels: {{ boxWidth: 12 }} }} }},
      scales: {{ y: {{ title: {{ display: true, text: 'Score (lower=better)' }} }} }}
    }}
  }});
}})();

// 2. Parameter Importance (horizontal bar)
(() => {{
  if (!importance.length) {{
    document.getElementById('importanceChart').parentElement.innerHTML =
      '<p style="color:var(--muted);text-align:center;padding:40px">Need 5+ trials for importance analysis</p>';
    return;
  }}
  new Chart(document.getElementById('importanceChart'), {{
    type: 'bar',
    data: {{
      labels: importance.map(i => i[0]),
      datasets: [{{ data: importance.map(i => i[1]),
        backgroundColor: importance.map((_, idx) =>
          `hsl(${{200 + idx * 30}}, 70%, 55%)`),
        borderRadius: 4 }}]
    }},
    options: {{
      indexAxis: 'y', responsive: true, maintainAspectRatio: false,
      plugins: {{ legend: {{ display: false }} }},
      scales: {{ x: {{ title: {{ display: true, text: 'Importance' }} }} }}
    }}
  }});
}})();

// 3. Parameter exploration scatter (score vs each param, colored by score)
(() => {{
  if (!trialData.length) {{
    document.getElementById('paramChart').parentElement.innerHTML =
      '<p style="color:var(--muted);text-align:center;padding:40px">No completed trials</p>';
    return;
  }}
  const paramNames = Object.keys(trialData[0].params || {{}});
  const datasets = paramNames.map((p, idx) => ({{
    label: p,
    data: trialData.map(t => ({{ x: t.params[p], y: t.score }})),
    backgroundColor: `hsla(${{idx * 55}}, 70%, 55%, 0.6)`,
    pointRadius: 3, hidden: idx > 1,
  }}));
  new Chart(document.getElementById('paramChart'), {{
    type: 'scatter',
    data: {{ datasets }},
    options: {{
      responsive: true, maintainAspectRatio: false,
      plugins: {{ legend: {{ position: 'top', labels: {{ boxWidth: 10, font: {{ size: 11 }} }} }} }},
      scales: {{
        x: {{ title: {{ display: true, text: 'Parameter Value' }} }},
        y: {{ title: {{ display: true, text: 'Score' }} }}
      }}
    }}
  }});
}})();

// 4. Metrics over trials
(() => {{
  if (!trialData.length) {{
    document.getElementById('metricsChart').parentElement.innerHTML =
      '<p style="color:var(--muted);text-align:center;padding:40px">No completed trials</p>';
    return;
  }}
  const metricKeys = ['avg_rise_time', 'avg_settling_time', 'avg_overshoot', 'avg_vel_reversals', 'avg_ss_error'];
  const labels = trialData.map(t => t.number);
  const colors = ['#e94560', '#53d8fb', '#ffe66d', '#4ecdc4', '#ff6b9d'];
  const datasets = metricKeys.map((k, idx) => ({{
    label: k.replace('avg_', ''),
    data: trialData.map(t => t.metrics[k] || 0),
    borderColor: colors[idx % colors.length],
    borderWidth: 1.5, pointRadius: 1.5, tension: 0.15, fill: false,
    hidden: idx > 1,
  }}));
  new Chart(document.getElementById('metricsChart'), {{
    type: 'line',
    data: {{ labels, datasets }},
    options: {{
      responsive: true, maintainAspectRatio: false,
      plugins: {{ legend: {{ position: 'top', labels: {{ boxWidth: 10, font: {{ size: 11 }} }} }} }},
      scales: {{ y: {{ title: {{ display: true, text: 'Value' }} }} }}
    }}
  }});
}})();

// Best params table
(() => {{
  const container = document.getElementById('paramsTable');
  let html = '<table><tr><th>Parameter</th><th>Value</th><th>Firmware Command</th></tr>';
  for (const [k, v] of Object.entries(bestParams)) {{
    const cmd = `set ${{k}} ${{v.toPrecision(6)}}`;
    html += `<tr><td class="param-name">${{k}}</td><td>${{v.toFixed(6)}}</td><td><code>${{cmd}}</code></td></tr>`;
  }}
  html += '</table>';
  container.innerHTML = html;
}})();

function copyCommands() {{
  const cmds = Object.entries(bestParams).map(([k,v]) => `set ${{k}} ${{v.toPrecision(6)}}`).join('\\n');
  navigator.clipboard.writeText(cmds).then(() => {{
    const btn = document.querySelector('.copy-btn');
    btn.textContent = 'Copied!';
    setTimeout(() => btn.textContent = 'Copy All Commands', 1500);
  }});
}}

// Best metrics table
(() => {{
  const container = document.getElementById('metricsTable');
  const labels = {{
    'avg_rise_time': ['Rise Time', 's'], 'avg_settling_time': ['Settling Time', 's'],
    'avg_overshoot': ['Overshoot', 'deg'], 'avg_oscillations': ['Oscillations', ''],
    'avg_vel_reversals': ['Vel Reversals', ''],
    'avg_ss_error': ['Steady-State Error', 'deg'], 'avg_settle_accuracy': ['Accuracy', 'deg'],
    'avg_smoothness': ['Smoothness', ''],
  }};
  let html = '<table><tr><th>Metric</th><th>Value</th></tr>';
  for (const [k, v] of Object.entries(bestMetrics)) {{
    const [name, unit] = labels[k] || [k, ''];
    const cls = v < 0.5 ? 'good' : v < 2 ? 'warn' : 'bad';
    html += `<tr><td>${{name}}</td><td class="${{cls}}">${{v.toFixed(4)}} ${{unit}}</td></tr>`;
  }}
  html += '</table>';
  container.innerHTML = html;
}})();

// All trials table
(() => {{
  if (!trialData.length) {{
    document.getElementById('allTrialsTable').innerHTML =
      '<p style="color:var(--muted);padding:12px">No completed trials to display.</p>';
    return;
  }}
  const container = document.getElementById('allTrialsTable');
  const pNames = Object.keys(trialData[0].params || {{}});
  let html = '<table><tr><th>#</th><th>Score</th>';
  pNames.forEach(p => html += `<th>${{p}}</th>`);
  html += '</tr>';
  const bestScore = Math.min(...trialData.map(t => t.score));
  trialData.forEach(t => {{
    const cls = t.score === bestScore ? 'good' : '';
    html += `<tr><td>${{t.number}}</td><td class="${{cls}}">${{t.score.toFixed(3)}}</td>`;
    pNames.forEach(p => html += `<td>${{t.params[p]?.toFixed(4) || ''}}</td>`);
    html += '</tr>';
  }});
  html += '</table>';
  container.innerHTML = html;
}})();
</script>
</body>
</html>"""
