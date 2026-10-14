---
name: robotrace-hardware-review
description: robotrace_v2の機体、KiCad回路、CubeMX ioc、センサー配置、配線、電源、ピン割り当て、機体寸法変更を確認するときに使う。ハード変更のコード反映確認も扱う。
---

# Robotrace Hardware Review

## 実行前の再発防止

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。機体・IMU・ADCの設定や座標を確認するときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。

## Overview

Use this skill when checking machine or circuit information, reviewing hardware changes, or deciding what firmware parameters must change after hardware work.

## Source Order

Before inferring hardware behavior, inspect the relevant sources:

- `Machine/robotrace_v3.STEP` (present in the 2026-10-12 checkout; the former `robotrace_v2 v86.step` reference is missing, and agreement with the current physical robot is unverified)
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

## Geometry source preflight

Before opening a documented model, list actual model files with `rg --files Machine`. If a reference is missing, record the actual available source/version and keep its agreement with hardware unverified. Do not silently substitute it as hardware truth. Record source hashes for extracted coordinates.

For KiCad component lookup, inspect the library identifier and Reference as well as Value. Main-board BMI088 is U8 in library `robocon:BMI088`; its Value is `~`, so a Value-only BMI088 search is insufficient. Verify footprint rotation and board-to-body transformation before applying coordinates. In the lateral-candidate investigation, available CAD gives rear 31.675 mm/right 0.099 mm; code keeps Y=-0.03168 m/X=0. Physical raw X direction is still unverified.
