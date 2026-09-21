"""Schema 9経路はマーカーを座標補正に使用しない。"""

import unittest
from pathlib import Path

from analysis.script.path_log_recovery import CsvLog, build_route
from analysis.script.robotrace_units import SCHEMA9_MEASURED_PULSE_METER, pulse_meter_for_log


class HeadingRouteSchema9Tests(unittest.TestCase):
    def test_versioned_distance_scale(self):
        self.assertEqual(pulse_meter_for_log({"logSchemaVersion": "8"}), 53424.0)
        self.assertEqual(pulse_meter_for_log({"logSchemaVersion": "9",
                                             "encoderPulsePerMeter": "56687"}), 56687.0)
        self.assertEqual(pulse_meter_for_log({"logSchemaVersion": "9",
                                             "encoderPulsePerMeter": "55116"}), SCHEMA9_MEASURED_PULSE_METER)

    def make_source(self, marker: str = "0", pulse_meter: int = 55116) -> CsvLog:
        rows = [dict(x_fused_mm="0", y_fused_mm=str(y),
                     encTotalOptimal=str(round(y * pulse_meter / 1000)), courseMarker=marker)
                for y in range(10, 2010, 10)]
        return CsvLog(Path("synthetic.csv"), list(rows[0]), rows,
                      dict(logSchemaVersion="9", encoderPulsePerMeter=str(pulse_meter),
                           **{"headingCalibration.enabled": "1"},
                           closureValid="1", closureReason="0", optimalTrace="0",
                           emcStop="0", logExpectedRows=str(len(rows))))

    def test_marker_does_not_change_route_crc_or_goal(self):
        base = build_route(self.make_source("0"), 0, 0, expected_pulse_meter=55116)
        altered = build_route(self.make_source("2"), 0, 0, expected_pulse_meter=55116)
        self.assertEqual(base.geometry_crc32, altered.geometry_crc32)
        self.assertEqual([(p.x, p.y) for p in base.line],
                         [(p.x, p.y) for p in altered.line])
        self.assertIn((0, 2000), [(p.x, p.y) for p in base.line])
        self.assertEqual((base.drive[-1].x, base.drive[-1].y), (0, 1000))
        self.assertEqual(base.geometry_crc32, 175954615)

    def test_invalid_or_old_source_rejected_for_new_route(self):
        source = self.make_source()
        source.parameters["closureValid"] = "0"
        with self.assertRaisesRegex(ValueError, "closureValid"):
            build_route(source, 0, 0, expected_pulse_meter=55116)
        source.parameters["closureValid"] = "1"
        source.parameters["encoderPulsePerMeter"] = "53424"
        with self.assertRaisesRegex(ValueError, "換算"):
            build_route(source, 0, 0, expected_pulse_meter=55116)
        source.parameters["logSchemaVersion"] = "8"
        with self.assertRaises(ValueError):
            build_route(source, 0, 0)

    def test_previous_schema9_scale_is_historical_analysis_only(self):
        source = self.make_source(pulse_meter=56687)
        with self.assertRaisesRegex(ValueError, "換算"):
            build_route(source, 0, 0)
        self.assertGreater(build_route(source, 0, 0, expected_pulse_meter=56687).route_count, 0)
        with self.assertRaisesRegex(ValueError, "換算"):
            build_route(source, 0, 0, expected_pulse_meter=55116)

    def test_abnormal_or_missing_rows_rejected(self):
        source = self.make_source()
        source.parameters["emcStop"] = "1"
        with self.assertRaisesRegex(ValueError, "完走"):
            build_route(source, 0, 0, expected_pulse_meter=55116)
        source.parameters["emcStop"] = "0"
        source.parameters["logExpectedRows"] = str(len(source.rows) + 1)
        with self.assertRaisesRegex(ValueError, "行数"):
            build_route(source, 0, 0, expected_pulse_meter=55116)
        source.parameters["logExpectedRows"] = str(len(source.rows))
        source.rows[-1]["encTotalOptimal"] = "-1"
        with self.assertRaisesRegex(ValueError, "逆行"):
            build_route(source, 0, 0, expected_pulse_meter=55116)


if __name__ == "__main__":
    unittest.main()
