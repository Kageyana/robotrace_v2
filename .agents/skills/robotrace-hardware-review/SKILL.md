---
name: robotrace-hardware-review
description: robotrace_v2の機体、KiCad回路、CubeMX ioc、センサー配置、配線、電源、ピン割り当て、機体寸法変更を確認するときに使う。ハード変更のコード反映確認も扱う。
---

# Robotrace Hardware Review

## Overview

Use this skill when checking machine or circuit information, reviewing hardware changes, or deciding what firmware parameters must change after hardware work.

## Source Order

Before inferring hardware behavior, inspect the relevant sources:

- `Machine/robotrace_v2 v86.step`
- `Circuit/robotrace_v2_main_v2/`
- `Circuit/robotrace_v2_Linesensor/`
- `Circuit/robotrace_v2_Sidemarker/`
- `Circuit/robotrace_v2_Extended_UI/`
- `robotrace_v2/robotrace_v2.ioc`
- related firmware source and headers

Do not infer pin assignments, sensor wiring, power rails, or connector meaning from variable names alone.

## Review Checklist

For machine or circuit changes, check at least:

- sensor count, ordering, and direction
- line sensor and marker sensor ADC channels
- GPIO assignments and LED timing
- motor left/right mapping
- PWM channels
- rotation direction
- encoder direction
- reduction ratio
- tire diameter
- encoder conversion factors
- IMU mounting direction, sign, and communication settings
- battery divider ratio and ADC conversion factors
- current sensor presence, direction, and scale
- buttons, display, and extended UI board connection
- microSD SPI, CS, power, and card-detect handling

## Reporting Rules

- Separate confirmed facts from inference.
- State which STEP, KiCad, PDF, `.ioc`, or source file was checked.
- Explicitly list real-machine confirmation items such as clearance, polarity, connector pinout, supply voltage, sensor direction, and mounting position.
- If code must be updated, identify the related definitions, log schema, parameter storage format, and `AGENTS.md` sections that must stay consistent.

## Firmware Consistency

When hardware data changes, verify consistency with:

- `AGENTS.md` hardware configuration and dimensions
- code parameters for tire diameter, tread width, encoder conversion, ADC scale, GPIO, timers, and DMA
- log schema fields if the meaning of data changes
- SD parameter files if stored values depend on hardware dimensions
