"""Schema 10のジャイロ経路と段階ゲートを検証する。"""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from analysis.script.path_log_recovery import CsvLog, build_route, recover_path_columns
from analysis.script.robotrace_units import CURRENT_PULSE_METER, pulse_meter_for_log


class HeadingRouteSchema10Tests(unittest.TestCase):
    def make_source(self, marker: str = "0") -> CsvLog:
        pulse_meter = int(CURRENT_PULSE_METER)
        rows = [dict(x="0", y=str(y), x_fused_mm="120", y_fused_mm=str(y),
                     encTotalOptimal=str(round(y * pulse_meter / 1000)),
                     courseMarker=marker)
                for y in range(10, 2010, 10)]
        return CsvLog(Path("synthetic_schema10.csv"), list(rows[0]), rows,
                      dict(logSchemaVersion="10", encoderPulsePerMeter=str(pulse_meter),
                           distanceScaleVerified="1", imuCalibrationValid="1",
                           imuCalibrationSamples="100", imuCalibrationReadErrors="0",
                           closureValid="1", closureReason="0", optimalTrace="0",
                           emcStop="0", logExpectedRows=str(len(rows))))

    def test_new_scale_and_gyro_xy_not_diagnostic_fused_xy(self):
        source = self.make_source()
        self.assertEqual(pulse_meter_for_log(source.parameters), CURRENT_PULSE_METER)
        self.assertEqual(CURRENT_PULSE_METER, 58019.0)
        route = build_route(source, 0, 0)
        self.assertIn((0, 2000), [(p.x, p.y) for p in route.line])
        self.assertEqual((route.drive[-1].x, route.drive[-1].y), (0, 1000))
        self.assertEqual(route.geometry_crc32, 175954615)

    def test_markers_and_diagnostic_pose_do_not_move_route(self):
        base = build_route(self.make_source("0"), 0, 0)
        changed = self.make_source("2")
        for row in changed.rows:
            row["x_fused_mm"] = "-300"
        changed.parameters["goalMarkerOnset_p"] = "103450"
        changed.parameters["goalMarkerXRaw_mm"] = "15"
        altered = build_route(changed, 0, 0)
        self.assertEqual(base.geometry_crc32, altered.geometry_crc32)

    def test_unverified_distance_and_bad_imu_rejected(self):
        for key, value in (("distanceScaleVerified", "0"),
                           ("imuCalibrationValid", "0"),
                           ("imuCalibrationSamples", "99"),
                           ("imuCalibrationReadErrors", "1")):
            with self.subTest(key=key):
                source = self.make_source()
                source.parameters[key] = value
                with self.assertRaisesRegex(ValueError, "IMU校正または距離検証"):
                    build_route(source, 0, 0)

    def test_old_schema_and_log_faults_rejected(self):
        source = self.make_source()
        source.parameters["logSchemaVersion"] = "9"
        with self.assertRaises(ValueError):
            build_route(source, 0, 0)
        source = self.make_source()
        source.parameters["closureValid"] = "0"
        with self.assertRaisesRegex(ValueError, "closureValid"):
            build_route(source, 0, 0)
        source = self.make_source()
        source.parameters["logExpectedRows"] = "201"
        with self.assertRaisesRegex(ValueError, "行数"):
            build_route(source, 0, 0)
        source = self.make_source()
        source.rows[-1]["encTotalOptimal"] = "-1"
        with self.assertRaisesRegex(ValueError, "逆行"):
            build_route(source, 0, 0)

    def test_version17_recovery_checks_crc(self):
        source = self.make_source()
        route = build_route(source, 0, 0)
        with TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source_path = directory / "1.csv"
            metadata = ",".join(f"{key}={value}" for key, value in source.parameters.items())
            fields = ",".join(source.fields)
            rows = "\n".join(",".join(row[field] for field in source.fields)
                             for row in source.rows)
            source_path.write_text(metadata + ",\n" + fields + "\n" + rows + "\n",
                                   encoding="utf-8")
            parameters = dict(logSchemaVersion="10", encoderPulsePerMeter="58019",
                              optimalTrace="3", emcStop="0", analysisSourceLog="1",
                              shortcutRequestedLevel="0", routeControllerVersion="17",
                              routePointCount=str(route.route_count),
                              routeGeometryCrc32=str(route.geometry_crc32),
                              shortcutLevel=str(route.applied_level),
                              shortcutBuildStatus=str(route.build_status),
                              shortcutCorridorCount=str(route.corridor_count),
                              shortcutReduction_mm=str(route.reduction_mm),
                              **{"routeShortcutSettings.maxLevel": "0"})
            secondary = CsvLog(directory / "2.csv", ["optimalIndex"],
                               [{"optimalIndex": "0"}], parameters)
            self.assertEqual(recover_path_columns(secondary).status, "restored")
            secondary.parameters["routeGeometryCrc32"] = "0"
            self.assertEqual(recover_path_columns(secondary).status, "missing")


if __name__ == "__main__":
    unittest.main()
