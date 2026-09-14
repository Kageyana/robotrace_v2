#!/usr/bin/env python3
"""距離カルマン融合の合成入力回帰テスト。"""

from __future__ import annotations

import math
import unittest

from robotrace_units import PULSE_METER, PULSE_MILLIMETER


DT = 0.001
SIGMA_ACCEL = 1.5
SIGMA_ENCODER = 0.08
BIAS_RANDOM_WALK = 0.05
INITIAL_BIAS_SIGMA = 0.5
MAX_ACCEL = 20.0
MAX_SPEED = 10.0
MAX_FUSED_DELTA_M = MAX_SPEED * DT + 0.5 * MAX_ACCEL * DT * DT
MAX_FUSED_DELTA_P = 535
INITIAL_SPEED_VARIANCE = SIGMA_ENCODER**2


def guarded_output_pulse(fused_pulse: float, raw_pulse: int) -> tuple[int, bool]:
    """control.cのfloat検証後パルス変換と生値フォールバックを再現する。"""
    if (not math.isfinite(fused_pulse) or
            fused_pulse < -MAX_FUSED_DELTA_P or fused_pulse > MAX_FUSED_DELTA_P):
        return max(-MAX_FUSED_DELTA_P, min(MAX_FUSED_DELTA_P, raw_pulse)), True
    return int(fused_pulse), False


class DistanceKalman:
    """firmware/Core/Src/distanceEstimator.cと同じ3状態モデル。"""

    def __init__(self, speed: float = 0.0) -> None:
        self.reset(speed)

    def reset(self, speed: float = 0.0) -> None:
        self.state = [0.0, max(-MAX_SPEED, min(MAX_SPEED, speed)), 0.0]
        self._reset_covariance()
        self.delta = 0.0
        self.rejects = 0
        self.fallbacks = 0
        self.invalids = 0
        self.max_abs_delta = 0.0
        self.fallback_active = False

    def _reset_covariance(self) -> None:
        self.cov = [[0.0, 0.0, 0.0],
                    [0.0, INITIAL_SPEED_VARIANCE, 0.0],
                    [0.0, 0.0, INITIAL_BIAS_SIGMA**2]]

    def _reset_velocity_covariance(self) -> None:
        for index in range(3):
            self.cov[1][index] = 0.0
            self.cov[index][1] = 0.0
        self.cov[1][1] = INITIAL_SPEED_VARIANCE

    def _record_delta(self) -> None:
        self.max_abs_delta = max(self.max_abs_delta, abs(self.delta))

    def _update_fallback(self, encoder_speed: float) -> None:
        previous = self.state[0] if math.isfinite(self.state[0]) else 0.0
        if not self.fallback_active:
            self.fallbacks += 1
        self.fallback_active = True
        speed = max(-MAX_SPEED, min(MAX_SPEED, encoder_speed)) if math.isfinite(encoder_speed) else 0.0
        self.state = [previous + speed * DT, speed, 0.0]
        self.delta = self.state[0] - previous
        self._reset_covariance()
        self._record_delta()

    def _update_invalid(self, encoder_speed: float) -> None:
        previous = self.state[0] if math.isfinite(self.state[0]) else 0.0
        self.invalids += 1
        speed = max(-MAX_SPEED, min(MAX_SPEED, encoder_speed)) if math.isfinite(encoder_speed) else 0.0
        self.state = [previous + speed * DT, speed, 0.0]
        self.delta = self.state[0] - previous
        self.fallback_active = False
        self._reset_covariance()
        self._record_delta()

    @staticmethod
    def _state_is_finite(state: list[float], cov: list[list[float]]) -> bool:
        return all(math.isfinite(value) for value in state) and all(
            math.isfinite(value) for row in cov for value in row)

    @staticmethod
    def _covariance_diagonal_is_valid(cov: list[list[float]]) -> bool:
        return all(math.isfinite(cov[index][index]) and cov[index][index] >= 0.0
                   for index in range(3))

    def update(self, encoder_speed: float, acceleration: float, valid: bool = True) -> None:
        if not valid:
            self._update_fallback(encoder_speed)
            return
        if not math.isfinite(encoder_speed) or not math.isfinite(acceleration):
            self._update_invalid(encoder_speed)
            return

        candidate_state = self.state[:]
        candidate_cov = [row[:] for row in self.cov]
        previous = candidate_state[0]
        if (not math.isfinite(previous) or
                not self._state_is_finite(candidate_state, candidate_cov) or
                not self._covariance_diagonal_is_valid(candidate_cov)):
            self._update_invalid(encoder_speed)
            return

        if self.fallback_active:
            candidate_state[1] = encoder_speed
            for index in range(3):
                candidate_cov[1][index] = 0.0
                candidate_cov[index][1] = 0.0
            candidate_cov[1][1] = INITIAL_SPEED_VARIANCE
            self.fallback_active = False

        acceleration = max(-MAX_ACCEL, min(MAX_ACCEL, acceleration))
        effective = acceleration - candidate_state[2]
        candidate_state[0] += candidate_state[1] * DT + 0.5 * effective * DT**2
        candidate_state[1] = max(-MAX_SPEED, min(MAX_SPEED, candidate_state[1] + effective * DT))

        transition = [[1.0, DT, -0.5 * DT**2], [0.0, 1.0, -DT], [0.0, 0.0, 1.0]]
        first = [[sum(transition[i][k] * candidate_cov[k][j] for k in range(3))
                  for j in range(3)] for i in range(3)]
        candidate_cov = [[sum(first[i][k] * transition[j][k] for k in range(3))
                     for j in range(3)] for i in range(3)]
        noise = [0.5 * DT**2, DT, 0.0]
        for i in range(3):
            for j in range(3):
                candidate_cov[i][j] += SIGMA_ACCEL**2 * noise[i] * noise[j]
        candidate_cov[2][2] += BIAS_RANDOM_WALK**2 * DT
        if (not self._state_is_finite(candidate_state, candidate_cov) or
                not self._covariance_diagonal_is_valid(candidate_cov)):
            self._update_invalid(encoder_speed)
            return

        innovation = encoder_speed - candidate_state[1]
        variance = candidate_cov[1][1] + SIGMA_ENCODER**2
        gate_variance = min(variance, INITIAL_SPEED_VARIANCE + SIGMA_ENCODER**2)
        if (abs(encoder_speed) > MAX_SPEED or
                not math.isfinite(gate_variance) or gate_variance <= 0.0 or
                abs(innovation) > 4.0 * math.sqrt(gate_variance)):
            self.rejects += 1
            self.state = candidate_state
            self.cov = candidate_cov
            self.delta = candidate_state[0] - previous
            if not math.isfinite(self.delta) or abs(self.delta) > MAX_FUSED_DELTA_M:
                self._update_invalid(encoder_speed)
                return
            self._reset_covariance()
            self._record_delta()
            return

        # 観測前共分散を完全に保存し、H=[0, 1, 0]のJoseph形式で更新する。
        prior = [row[:] for row in candidate_cov]
        gain = [prior[i][1] / variance for i in range(3)]
        for i in range(3):
            candidate_state[i] += gain[i] * innovation
        observation = [0.0, 1.0, 0.0]
        identity_minus_kh = [
            [(1.0 if i == j else 0.0) - gain[i] * observation[j]
             for j in range(3)] for i in range(3)]
        first_joseph = [[sum(identity_minus_kh[i][k] * prior[k][j]
                             for k in range(3))
                         for j in range(3)] for i in range(3)]
        candidate_cov = [[
            sum(first_joseph[i][k] * identity_minus_kh[j][k]
                for k in range(3)) + gain[i] * SIGMA_ENCODER**2 * gain[j]
            for j in range(3)] for i in range(3)]
        for i in range(3):
            for j in range(i + 1, 3):
                average = 0.5 * (candidate_cov[i][j] + candidate_cov[j][i])
                candidate_cov[i][j] = candidate_cov[j][i] = average
        candidate_state[1] = max(-MAX_SPEED, min(MAX_SPEED, candidate_state[1]))
        self.delta = candidate_state[0] - previous
        if (not self._state_is_finite(candidate_state, candidate_cov) or
                not self._covariance_diagonal_is_valid(candidate_cov) or
                not math.isfinite(self.delta) or abs(self.delta) > MAX_FUSED_DELTA_M):
            self._update_invalid(encoder_speed)
            return
        self.state = candidate_state
        self.cov = candidate_cov
        self._record_delta()

    def correction(self, distance_m: float) -> None:
        self.state[0] += distance_m


class DistanceEstimatorTests(unittest.TestCase):
    def test_encoder_conversion_round_trip_is_below_one_pulse(self) -> None:
        distance_m = 1.234567
        pulses = distance_m * PULSE_METER
        round_trip_m = round(pulses) / PULSE_METER
        self.assertLess(abs(round_trip_m - distance_m) * PULSE_METER, 1.0)
        self.assertAlmostEqual(PULSE_MILLIMETER * 1000.0, PULSE_METER, places=6)

    def test_constant_pulse_speed_matches_raw_encoder_distance(self) -> None:
        pulse_per_ms = 2.0 * PULSE_MILLIMETER
        encoder_speed_mps = pulse_per_ms / PULSE_MILLIMETER
        estimator = DistanceKalman(encoder_speed_mps)
        for _ in range(1000):
            estimator.update(encoder_speed_mps, 0.0)
        raw_distance_m = encoder_speed_mps * 1.0
        self.assertAlmostEqual(estimator.state[0], raw_distance_m, delta=0.01)

    def test_stop_and_constant_speed(self) -> None:
        stopped = DistanceKalman()
        for _ in range(1000):
            stopped.update(0.0, 0.0)
        self.assertAlmostEqual(stopped.state[0], 0.0, places=5)

        # 走行開始時のリセットは、開始時の生エンコーダ速度で速度状態を
        # 再同期するため、一定速度ケースも同じ初期条件で検証する。
        estimator = DistanceKalman(2.0)
        for _ in range(1000):
            estimator.update(2.0, 0.0)
        self.assertAlmostEqual(estimator.state[1], 2.0, delta=0.08)
        self.assertAlmostEqual(estimator.state[0], 2.0, delta=0.03)

    def test_acceleration_and_reverse(self) -> None:
        estimator = DistanceKalman()
        for index in range(1000):
            estimator.update(4.0 * (index + 1) * DT, 4.0)
        self.assertAlmostEqual(estimator.state[1], 4.0, delta=0.08)
        self.assertAlmostEqual(estimator.state[0], 2.0, delta=0.08)
        estimator.reset(-1.0)
        for _ in range(1000):
            estimator.update(-1.0, 0.0)
        self.assertAlmostEqual(estimator.state[0], -1.0, delta=0.03)
        self.assertLess(estimator.state[1], -0.9)

    def test_wheel_slip_is_rejected_without_bias_learning(self) -> None:
        estimator = DistanceKalman(2.0)
        for _ in range(1000):
            estimator.update(2.0, 0.0)
        distance_before_slip = estimator.state[0]
        bias_before_slip = estimator.state[2]
        for _ in range(1000):
            estimator.update(6.0, 0.0)
        distance_after_slip = estimator.state[0]
        self.assertEqual(estimator.rejects, 1000)
        self.assertAlmostEqual(estimator.state[2], bias_before_slip, places=6)
        self.assertAlmostEqual(distance_after_slip, distance_before_slip + 2.0, delta=0.01)
        for _ in range(1000):
            estimator.update(2.0, 0.0)
        self.assertAlmostEqual(estimator.state[0] - distance_after_slip, 2.0, delta=0.01)

    def test_single_abnormal_encoder_value_is_rejected(self) -> None:
        estimator = DistanceKalman(2.0)
        estimator.update(50.0, 0.0)
        self.assertEqual(estimator.rejects, 1)
        self.assertAlmostEqual(estimator.state[2], 0.0, places=6)
        self.assertAlmostEqual(estimator.state[0], 0.002, delta=0.0001)

    def test_invalid_state_or_covariance_falls_back_without_distance_jump(self) -> None:
        estimator = DistanceKalman(2.0)
        estimator.state[0] = 1.25
        estimator.cov[1][1] = -1.0
        estimator.update(2.0, 0.0)
        self.assertEqual(estimator.invalids, 1)
        self.assertAlmostEqual(estimator.state[0], 1.252, places=6)
        self.assertAlmostEqual(estimator.state[1], 2.0, places=6)
        self.assertAlmostEqual(estimator.state[2], 0.0, places=6)
        self.assertEqual(estimator.cov[1][1], INITIAL_SPEED_VARIANCE)

    def test_nonfinite_input_falls_back_without_committing_invalid_state(self) -> None:
        estimator = DistanceKalman(2.0)
        estimator.update(math.nan, 0.0, valid=True)
        estimator.update(2.0, math.inf, valid=True)
        self.assertEqual(estimator.invalids, 2)
        self.assertAlmostEqual(estimator.state[0], 0.002, places=6)
        self.assertAlmostEqual(estimator.state[1], 2.0, places=6)
        self.assertTrue(all(math.isfinite(value)
                            for row in estimator.cov for value in row))

    def test_forty_second_constant_speed_has_no_invalid_updates(self) -> None:
        estimator = DistanceKalman(2.0)
        for _ in range(40000):
            estimator.update(2.0, 0.0)
            self.assertLessEqual(abs(estimator.delta), MAX_FUSED_DELTA_M)
            self.assertLessEqual(abs(round(estimator.delta * PULSE_METER)), MAX_FUSED_DELTA_P)
        self.assertEqual(estimator.invalids, 0)
        self.assertLess(abs(estimator.state[0] - 80.0) / 80.0, 0.01)

    def test_maximum_acceleration_is_clamped_without_invalid_update(self) -> None:
        estimator = DistanceKalman()
        for _ in range(1000):
            estimator.update(10.0, 100.0)
            self.assertLessEqual(abs(estimator.delta), MAX_FUSED_DELTA_M)
        self.assertLessEqual(estimator.state[1], MAX_SPEED)
        self.assertTrue(all(math.isfinite(value)
                            for row in estimator.cov for value in row))

    def test_output_guard_checks_float_before_integer_conversion(self) -> None:
        cases = [
            (math.nan, 17, 17, True),
            (math.inf, 17, 17, True),
            (-math.inf, -17, -17, True),
            (-535.0, 17, -535, False),
            (535.0, 17, 535, False),
            (-536.0, -17, -17, True),
            (536.0, 17, 17, True),
            (2147483648.0, 700, 535, True),
            (-2147483648.0, -700, -535, True),
        ]
        for fused_pulse, raw_pulse, expected, guarded in cases:
            actual, actual_guarded = guarded_output_pulse(fused_pulse, raw_pulse)
            self.assertEqual(actual, expected)
            self.assertEqual(actual_guarded, guarded)

    def test_long_slip_keeps_imu_prediction_and_does_not_learn_bias(self) -> None:
        estimator = DistanceKalman(2.0)
        for _ in range(1000):
            estimator.update(2.0, 0.0)
        distance_before_slip = estimator.state[0]
        bias_before_slip = estimator.state[2]
        for _ in range(30000):
            estimator.update(6.0, 0.0)
        self.assertEqual(estimator.rejects, 30000)
        self.assertAlmostEqual(estimator.state[2], bias_before_slip, places=6)
        self.assertAlmostEqual(estimator.state[0], distance_before_slip + 60.0, delta=0.03)
        distance_before_recovery = estimator.state[0]
        estimator.update(2.0, 0.0)
        self.assertAlmostEqual(estimator.state[0] - distance_before_recovery, 0.002, places=6)

    def test_fused_delta_and_pulse_bounds_for_synthetic_inputs(self) -> None:
        estimator = DistanceKalman()
        inputs = [(0.0, 0.0), (10.0, 20.0), (-10.0, -20.0),
                  (2.0, 0.0), (6.0, 0.0), (-2.0, 0.0)]
        for encoder_speed, acceleration in inputs:
            estimator.update(encoder_speed, acceleration)
            self.assertLessEqual(abs(estimator.delta), MAX_FUSED_DELTA_M)
            self.assertLessEqual(abs(round(estimator.delta * PULSE_METER)), MAX_FUSED_DELTA_P)

    def test_invalid_imu_fallback_and_resynchronization(self) -> None:
        estimator = DistanceKalman()
        for _ in range(25):
            estimator.update(3.0, math.nan, valid=False)
        self.assertEqual(estimator.fallbacks, 1)
        self.assertAlmostEqual(estimator.state[0], 0.075, places=6)
        estimator.update(3.0, 0.0, valid=True)
        self.assertFalse(estimator.fallback_active)
        self.assertAlmostEqual(estimator.state[1], 3.0, delta=0.05)

    def test_marker_correction_preserves_next_delta_continuity(self) -> None:
        estimator = DistanceKalman(2.0)
        for _ in range(100):
            estimator.update(2.0, 0.0)
        before = estimator.state[0]
        estimator.correction(-0.1)
        estimator.update(2.0, 0.0)
        self.assertAlmostEqual(estimator.state[0], before - 0.1 + estimator.delta, delta=0.005)
        self.assertLess(abs(estimator.delta), 0.01)


if __name__ == "__main__":
    unittest.main()
