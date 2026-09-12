#!/usr/bin/env python3
"""Tests for BMI088 temperature-compensation CSV compatibility."""

from __future__ import annotations

import io
import unittest
from pathlib import Path
from unittest import mock

import fit_imu_temp_compensation as fit


class LoadDatasetTests(unittest.TestCase):
    def load_csv(self, content: str) -> fit.Dataset:
        with mock.patch.object(Path, "open", return_value=io.StringIO(content)):
            return fit.load_dataset(Path("imu_temp.csv"), max_std_dps=0.5)

    def test_new_four_column_csv_uses_gyro_standard_deviation(self) -> None:
        dataset = self.load_csv(
                "elapsed_ms,temp_C,gyroZMean_dps,gyroZStd_dps\n"
                "2000,20.0,0.10,0.20\n"
                "4000,21.0,0.11,0.60\n"
                "6000,22.0,0.12,0.30\n",
        )

        self.assertEqual(len(dataset.samples), 2)
        self.assertTrue(all(sample.encoder_movement is None for sample in dataset.samples))

    def test_legacy_five_column_csv_also_requires_zero_encoder_movement(self) -> None:
        dataset = self.load_csv(
                "elapsed_ms,temp_C,gyroZMean_dps,gyroZStd_dps,encoderMovement\n"
                "2000,20.0,0.10,0.20,0\n"
                "4000,21.0,0.11,0.20,2\n",
        )

        self.assertEqual(len(dataset.samples), 1)
        self.assertEqual(dataset.samples[0].encoder_movement, 0.0)

    def test_missing_required_column_is_rejected(self) -> None:
        with mock.patch.object(
            Path,
            "open",
            return_value=io.StringIO(
                "elapsed_ms,temp_C,gyroZMean_dps\n"
                "2000,20.0,0.10\n"
            ),
        ):
            with self.assertRaisesRegex(ValueError, "gyroZStd_dps"):
                fit.load_dataset(Path("imu_temp.csv"), max_std_dps=0.5)


if __name__ == "__main__":
    unittest.main()
