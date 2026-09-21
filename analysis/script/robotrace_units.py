"""robotrace_v2とPC解析で共有する物理換算定数。"""

from __future__ import annotations


PULSE_METER = 53424.0  # Schema 8以前の履歴ログ用
PULSE_MILLIMETER = PULSE_METER / 1000.0
SCHEMA9_PULSE_METER = 56687.0
SCHEMA9_MEASURED_PULSE_METER = 55116.0  # スキーマ9の旧実測ログ
CURRENT_PULSE_METER = 58019.0  # 1m直進3回の実測 左58,092/右57,945 pulse/m の平均


def pulse_meter_for_log(metadata: dict[str, str]) -> float:
    """ログごとの保存時点の距離換算を返す。"""
    version = int(metadata.get("logSchemaVersion", "0"))
    if version >= 9:
        return float(metadata.get("encoderPulsePerMeter", SCHEMA9_PULSE_METER))
    return PULSE_METER
