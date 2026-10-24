"""Execute the production acquisition functions with a deterministic HAL/IRQ mock.

Only platform/UI/SD dependencies are excluded; acquisition and normalization
function bodies are read from the working tree, never copied into the test.
"""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SRC = ROOT / "robotrace_v2/Core/Src"
INC = ROOT / "robotrace_v2/Core/Inc"


def function(source, name):
    match = re.search(r"^(?:static )?(?:void|bool) " + name + r"\([^;]*?\)\s*\{", source, re.M)
    if not match:
        raise RuntimeError(f"Production function missing: {name}")
    start = match.start()
    pos = match.end()
    depth = 1
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    return source[start:pos] + "\n"


def main():
    line = (SRC / "lineSensor.c").read_text(encoding="utf-8-sig")
    marker = (SRC / "markerSensor.c").read_text(encoding="utf-8-sig")
    battery = (SRC / "battery.c").read_text(encoding="utf-8-sig")
    callbacks = (SRC / "main.c").read_text(encoding="utf-8-sig")
    line_h = (INC / "lineSensor.h").read_text(encoding="utf-8-sig")
    # Reuse the actual public constants; fail if the timer or averaging changes.
    constants = "\n".join(x for x in line_h.splitlines() if x.startswith("#define ")
                          and not x.startswith(("#define LINESENSOR_H_", "#define FILENAME")))
    assert "LS_AVERAGE_SAMPLES 2U" in constants
    assert "LS_ADC_SETTLE_US 120U" in constants
    assert "htim3.Init.Period = 3749;" in callbacks
    assert "htim3.Init.Prescaler = 11;" in callbacks
    assert "ADC_SAMPLETIME_15CYCLES" in callbacks and "ADC_SAMPLETIME_480CYCLES" in callbacks
    timer = (SRC / "timer.c").read_text(encoding="utf-8-sig")
    assert "monitorLineSensorUpdate1ms(!pathSteeringActive || pathState == PATH_STATE_LINE_FALLBACK || pathState == PATH_STATE_REJOIN)" in timer
    data = line[line.index("uint32_t lSensorInt"):line.index("/////////////////////////////////////////////////////////////////////")]
    calibration = line[line.index("uint16_t lSensorMax"):line.index("/////////////////////////////////////////////////////////////////////", line.index("uint16_t lSensorMax"))]
    names = ["skipLineSensorAcquisition", "lineSensorWindowValid", "isLineSensorStoredValueInRange",
             "validateLineSensorCalibrationValues", "isLineSensorCalibrationValid", "finishLineSensorCalibration",
             "powerLineSensors", "lineSensorPhaseChanged", "startLineSensorAcquisition",
             "completeLineSensorAcquisition", "failLineSensorAcquisition", "monitorLineSensorUpdate1ms",
             "getLineSensor", "normalizeLineSensor", "getAngleSensor"]
    imu_h = (INC / "IMU.h").read_text(encoding="utf-8-sig")
    constants += "\n" + next(x for x in imu_h.splitlines() if x.startswith("#define IMU_PI_F "))
    body = constants + "\n" + data + calibration + "\n".join(function(line, x) for x in names)
    body = next(x for x in line.splitlines() if x.strip() == "float angleSensor;") + "\n" + body
    body += function(marker, "getMarkerSensor") + function(marker, "powerMarkerSensors")
    body += function(battery, "getVref").replace("(__IO uint16_t*)0x1FFF7A2A", "&mockVrefCal")
    body += "\n".join(function(callbacks, x) for x in ["HAL_ADC_ConvCpltCallback",
                       "HAL_TIM_PWM_PulseFinishedCallback", "HAL_TIM_PeriodElapsedCallback",
                       "HAL_TIM_OC_DelayElapsedCallback", "HAL_ADC_ErrorCallback"])
    pid = (SRC / "PIDcontrol.c").read_text(encoding="utf-8-sig")
    pid_h = (INC / "PIDcontrol.h").read_text(encoding="utf-8-sig")
    body += "\n" + "\n".join(x for x in pid_h.splitlines() if x.startswith("#define TRACE_WEIGHT")) + "\n"
    body += "typedef struct { const char *name; int16_t kp, ki, kd; float Int; int16_t pwm; } pidParam;\n"
    body += "static pidParam lineTraceCtrl = {\"line\", 28, 1, 200, 0, 0};\n"
    body += function(pid, "motorControlTrace")
    # Exercise the same unchanged controller on the old reference input arrays.
    body += "static pidParam legacyCtrl = {\"line\", 28, 1, 200, 0, 0};\n"
    legacy_pid = function(pid, "motorControlTrace")
    for old, new in [("motorControlTrace", "legacyMotorControlTrace"), ("lineTraceCtrl", "legacyCtrl"),
                     ("lSensorCari", "legacyNorm"), ("lSensor", "legacyRaw")]:
        legacy_pid = re.sub(r"\b" + old + r"\b", new, legacy_pid)
    body += '#include "legacy_reference.inc"\n' + legacy_pid
    with tempfile.TemporaryDirectory(prefix="robotrace-adc-") as temp:
        generated = Path(temp) / "production.inc"
        generated.write_text(body, encoding="utf-8")
        binary = Path(temp) / "check.exe"
        for optimization in ["-O0", "-O2"]:
            subprocess.run(["gcc", "-std=c11", optimization, "-Wall", "-Wextra", "-Werror",
                            "-I", temp, "-I", str(HERE), str(HERE / "check.c"), "-lm", "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    print(json.dumps({"source_sha256": {x: hashlib.sha256((SRC / x).read_bytes()).hexdigest()
                                       for x in ["lineSensor.c", "markerSensor.c", "battery.c", "main.c", "timer.c"]}}, indent=2))


if __name__ == "__main__":
    main()
