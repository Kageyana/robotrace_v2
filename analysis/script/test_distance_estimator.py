#!/usr/bin/env python3
"""距離カルマン融合の合成入力回帰テスト。"""

from __future__ import annotations

import math
import unittest


DT = 0.001
SIGMA_ACCEL = 1.5
SIGMA_ENCODER = 0.08
BIAS_RANDOM_WALK = 0.05
INITIAL_BIAS_SIGMA = 0.5
MAX_ACCEL = 20.0
MAX_SPEED = 10.0


class DistanceKalman:
    """firmware/Core/Src/distanceEstimator.cと同じ3状態モデル。"""

    def __init__(self, speed: float = 0.0) -> None:
        self.reset(speed)

    def reset(self, speed: float = 0.0) -> None:
        self.state = [0.0, max(-MAX_SPEED, min(MAX_SPEED, speed)), 0.0]
        self.cov = [[0.0, 0.0, 0.0],
                    [0.0, SIGMA_ENCODER**2, 0.0],
                    [0.0, 0.0, INITIAL_BIAS_SIGMA**2]]
        self.delta = 0.0
        self.rejects = 0
        self.fallbacks = 0
        self.fallback_active = False

    def update(self, encoder_speed: float, acceleration: float, valid: bool = True) -> None:
        if not valid or not math.isfinite(encoder_speed) or not math.isfinite(acceleration):
            previous = self.state[0] if math.isfinite(self.state[0]) else 0.0
            if not self.fallback_active:
                self.fallbacks += 1
            self.fallback_active = True
            speed = max(-MAX_SPEED, min(MAX_SPEED, encoder_speed)) if math.isfinite(encoder_speed) else 0.0
            self.state = [previous + speed * DT, speed, 0.0]
            self.delta = speed * DT
            self.cov = [[0.0, 0.0, 0.0],
                        [0.0, SIGMA_ENCODER**2, 0.0],
                        [0.0, 0.0, INITIAL_BIAS_SIGMA**2]]
            return

        previous = self.state[0]
        if self.fallback_active:
            self.state[1] = encoder_speed
            self.cov[0][1] = self.cov[1][0] = 0.0
            self.cov[1][1] = SIGMA_ENCODER**2
            self.cov[1][2] = self.cov[2][1] = 0.0
            self.fallback_active = False

        acceleration = max(-MAX_ACCEL, min(MAX_ACCEL, acceleration))
        effective = acceleration - self.state[2]
        self.state[0] += self.state[1] * DT + 0.5 * effective * DT**2
        self.state[1] = max(-MAX_SPEED, min(MAX_SPEED, self.state[1] + effective * DT))

        transition = [[1.0, DT, -0.5 * DT**2], [0.0, 1.0, -DT], [0.0, 0.0, 1.0]]
        first = [[sum(transition[i][k] * self.cov[k][j] for k in range(3))
                  for j in range(3)] for i in range(3)]
        self.cov = [[sum(first[i][k] * transition[j][k] for k in range(3))
                     for j in range(3)] for i in range(3)]
        noise = [0.5 * DT**2, DT, 0.0]
        for i in range(3):
            for j in range(3):
                self.cov[i][j] += SIGMA_ACCEL**2 * noise[i] * noise[j]
        self.cov[2][2] += BIAS_RANDOM_WALK**2 * DT

        innovation = encoder_speed - self.state[1]
        variance = self.cov[1][1] + SIGMA_ENCODER**2
        scale = 1.0
        normalized_squared = innovation**2 / variance
        if normalized_squared > 16.0:
            scale = min(100.0, normalized_squared / 16.0)
            self.rejects += 1
        variance = self.cov[1][1] + SIGMA_ENCODER**2 * scale
        gain = [self.cov[i][1] / variance for i in range(3)]
        prior = [row[:] for row in self.cov]
        for i in range(3):
            self.state[i] += gain[i] * innovation
            for j in range(3):
                self.cov[i][j] -= gain[i] * prior[1][j]
        self.state[1] = max(-MAX_SPEED, min(MAX_SPEED, self.state[1]))
        self.delta = self.state[0] - previous

    def correction(self, distance_m: float) -> None:
        self.state[0] += distance_m


class DistanceEstimatorTests(unittest.TestCase):
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
        estimator.reset()
        for _ in range(1000):
            estimator.update(-1.0, 0.0)
        self.assertAlmostEqual(estimator.state[0], -1.0, delta=0.03)
        self.assertLess(estimator.state[1], -0.9)

    def test_wheel_slip_downweights_encoder(self) -> None:
        estimator = DistanceKalman(2.0)
        for _ in range(500):
            estimator.update(2.0, 0.0)
        distance_before = estimator.state[0]
        for _ in range(100):
            estimator.update(6.0, 0.0)
        raw_distance = distance_before + 6.0 * 100 * DT
        self.assertGreater(estimator.rejects, 0)
        self.assertLess(estimator.state[0], raw_distance - 0.05)

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
