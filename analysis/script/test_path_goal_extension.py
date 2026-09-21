"""PATH系の停止点延長と旧バージョンの経路復元を検証する。"""

import math
import tempfile
import unittest
from pathlib import Path

from analysis.script.path_log_recovery import (
    RoutePoint,
    build_route,
    extend_to_origin,
    read_csv_log,
    recover_path_columns,
)


class PathGoalExtensionTests(unittest.TestCase):
    def test_goal_is_halfway_from_logged_endpoint_to_origin(self):
        for end_x, end_y in ((0, -454), (0, -1000), (300, 400), (0, -20)):
            with self.subTest(endpoint=(end_x, end_y)):
                line = [RoutePoint(0, 0), RoutePoint(end_x, end_y)]
                drive = [RoutePoint(0, 0), RoutePoint(end_x, end_y)]
                flags = [0, 0]
                extend_to_origin(line, drive, flags, 15)
                self.assertEqual((drive[-1].x, drive[-1].y), (end_x // 2, end_y // 2))
                self.assertEqual(len(drive) - 2, math.ceil(math.hypot(end_x, end_y) / 80))
                self.assertEqual([(p.x, p.y) for p in line], [(p.x, p.y) for p in drive])
                self.assertEqual(len(flags), len(drive))
                for before, after in zip(drive[1:-1], drive[2:]):
                    self.assertLessEqual(math.hypot(after.x - before.x, after.y - before.y), 41)

    def test_legacy_500mm_extension_is_kept(self):
        line = [RoutePoint(0, 0), RoutePoint(0, -454)]
        drive = [RoutePoint(0, 0), RoutePoint(0, -454)]
        extend_to_origin(line, drive, [0, 0], 14)
        self.assertEqual((drive[-1].x, drive[-1].y), (0, 46))
        self.assertEqual(len(drive), 15)

    def test_capacity_and_degenerate_endpoint_fail_safely(self):
        with self.assertRaisesRegex(ValueError, "終点が原点"):
            extend_to_origin([RoutePoint(0, 0)] * 2, [RoutePoint(0, 0)] * 2, [0, 0], 15)
        line = [RoutePoint(0, 0)] * 1490 + [RoutePoint(0, -2000)]
        drive = list(line)
        with self.assertRaisesRegex(ValueError, "点数上限"):
            extend_to_origin(line, drive, [0] * len(line), 15)

    def test_recovery_uses_recorded_route_version(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_path = root / "1.csv"
            source_path.write_text(
                "logSchemaVersion=8,closureValid=1,goalMarkerOnset_p=4000,"
                "goalMarkerYRaw_mm=-400,\n"
                "x_closed_mm,y_closed_mm,courseMarker,encTotalOptimal\n"
                + "".join(f"0,{-y},0,{y * 10}\n" for y in (10, 100, 200, 300, 400, 454)),
                encoding="utf-8",
            )
            source = read_csv_log(source_path)
            routes = {version: build_route(source, 0, 0, version) for version in (14, 15)}
            self.assertEqual(routes[14].drive[-1].y, 46)
            self.assertEqual(routes[15].drive[-1].y, -227)
            self.assertNotEqual(routes[14].geometry_crc32, routes[15].geometry_crc32)
            for version, route in routes.items():
                with self.subTest(version=version):
                    path = root / f"{version}.csv"
                    path.write_text(
                        "logSchemaVersion=8,optimalTrace=3,emcStop=0,analysisSourceLog=1,"
                        "routeSourceLog=1,shortcutRequestedLevel=0,"
                        f"routePointCount={route.route_count},"
                        f"routeGeometryCrc32={route.geometry_crc32},"
                        f"routeControllerVersion={version},"
                        "shortcutLevel=0,shortcutBuildStatus=0,"
                        "shortcutCorridorCount=0,shortcutReduction_mm=0,"
                        "routeShortcutSettings.maxLevel=0,\n"
                        "optimalIndex\n0\n",
                        encoding="utf-8",
                    )
                    result = recover_path_columns(read_csv_log(path))
                    self.assertEqual(result.status, "restored", result.reason)


if __name__ == "__main__":
    unittest.main()
