"""一次走行ログの区間平均・ゴール補間・閉路条件の合成検証。"""

import math
import tempfile
import unittest
from pathlib import Path

from analysis.script.path_log_recovery import CsvLog, build_route
from analysis.script.plot_primary_closed_xy import read_log


PULSE_PER_MM = 53.424


def reconstruct(samples, goal_p):
    """ファームウェアの区間平均角速度・中点方位による再構成を模擬する。"""
    points = [(0, 0.0, 0.0, 0.0)]
    heading = 0.0
    previous_p = 0
    previous_t = 0
    goal = None
    for t, pulse, avg_gyro in samples:
        dt = (t - previous_t) / 1000.0
        delta_p = pulse - previous_p
        assert 0 < t - previous_t <= 100 and delta_p >= 0
        delta_heading = avg_gyro * dt
        mid = math.radians(heading + delta_heading / 2)
        old_x, old_y = points[-1][1:3]
        x = old_x + delta_p / PULSE_PER_MM * math.sin(mid)
        y = old_y + delta_p / PULSE_PER_MM * math.cos(mid)
        if goal is None and previous_p <= goal_p <= pulse and delta_p:
            ratio = (goal_p - previous_p) / delta_p
            goal = (old_x + ratio * (x - old_x), old_y + ratio * (y - old_y),
                    goal_p / PULSE_PER_MM)
        points.append((pulse, x, y, pulse / PULSE_PER_MM))
        heading += delta_heading
        previous_p, previous_t = pulse, t
    if goal is None:
        return None
    gx, gy, gs = goal
    if abs(gx) > 250 or 1.5 * abs(gx) / gs > 0.02:
        return None
    corrected = []
    for pulse, x, y, s in points:
        progress = min(1.0, max(0.0, s / gs))
        blend = progress * progress * (3 - 2 * progress)
        corrected.append((pulse, x - gx * blend, y))
    return goal, corrected


class PrimaryClosedXYTests(unittest.TestCase):
    def test_variable_speed_and_gyro_with_irregular_log_spacing(self):
        pulses = 0
        gyro = []
        cumulative = []
        for t in range(1, 201):
            pulses += 35 + t % 9
            gyro.append(2.0 + 0.02 * t)
            cumulative.append(pulses)
        indices = [7, 18, 29, 39, 50, 63, 73, 85, 97, 109, 120, 131, 142, 153, 166, 177, 188, 199]
        samples = []
        previous = -1
        for index in indices:
            interval = gyro[previous + 1:index + 1]
            samples.append((index + 1, cumulative[index], sum(interval) / len(interval)))
            previous = index
        goal_p = (cumulative[153] + cumulative[154]) // 2
        result = reconstruct(samples, goal_p)
        self.assertIsNotNone(result)
        (goal_x, goal_y, _), corrected = result
        self.assertNotEqual(goal_x, 0.0)
        upper = next(i for i, point in enumerate(corrected) if point[0] >= goal_p)
        lower = upper - 1
        ratio = (goal_p - corrected[lower][0]) / (corrected[upper][0] - corrected[lower][0])
        interpolated_closed_x = corrected[lower][1] + ratio * (
            corrected[upper][1] - corrected[lower][1])
        self.assertLess(abs(interpolated_closed_x), 1.0)
        self.assertEqual(corrected[0], (0, 0.0, 0.0))
        self.assertNotEqual(corrected[-1][1], 0.0)  # 停止区間は原点へ強制しない
        self.assertTrue(math.isfinite(goal_y))

    def test_missing_or_excessive_goal_is_rejected(self):
        samples = [(10, 500, 0.0), (21, 1000, 0.0)]
        self.assertIsNone(reconstruct(samples, 2000))
        self.assertIsNone(reconstruct([(10, 500, 9000.0), (20, 1000, 9000.0)], 750))

    def test_route_includes_exact_marker_and_real_tail_endpoint(self):
        rows = [dict(x_closed_mm="0", y_closed_mm=str(y), courseMarker="0",
                     encTotalOptimal=str(round(y * PULSE_PER_MM)))
                for y in range(10, 2010, 10)]
        for version in ("7", "8"):
            with self.subTest(version=version):
                source = CsvLog(Path("synthetic.csv"), list(rows[0]), rows,
                                dict(logSchemaVersion=version, closureValid="1",
                                     goalMarkerOnset_p=str(round(1705 * PULSE_PER_MM)),
                                     goalMarkerYRaw_mm="1705.00"))
                route = build_route(source, 0, 0)
                self.assertIn((0, 1705), [(p.x, p.y) for p in route.line])
                self.assertIn((0, 2000), [(p.x, p.y) for p in route.line])
                self.assertEqual(route.line[0].x, route.line[0].y)
                source.parameters["closureValid"] = "0"
                with self.assertRaisesRegex(ValueError, "closureValid"):
                    build_route(source, 0, 0)

    def test_schema_8_closed_plot_keeps_separate_encoder_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "new.csv"
            path.write_text(
                "logSchemaVersion=8,closureValid=1,\n"
                "encTotalOptimal,encCurrentL,encCurrentR,x,y,x_closed_mm,y_closed_mm\n"
                "100,50,52,1,10,0,10\n", encoding="utf-8")
            metadata, rows = read_log(path)
            self.assertEqual(metadata["logSchemaVersion"], "8")
            self.assertEqual((rows[0]["encCurrentL"], rows[0]["encCurrentR"]), ("50", "52"))

    def test_old_csv_is_not_accepted_as_closed_plot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.csv"
            path.write_text("logSchemaVersion=6,\ncntlog,x,y\n10,0,10\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "スキーマ7"):
                read_log(path)


if __name__ == "__main__":
    unittest.main()
