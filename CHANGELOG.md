# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
with the pre-1.0 convention that breaking changes increment the minor version.

---

## [0.4.2] — 2026-08-24

### Fixed

- I²C multi-byte buffer transfers now set the sub-address auto-increment bit
  (`SUB[7]`). `read_g` / `read_raw` previously burst-read `OUT_X_L` (0x28)
  without it, so the part returned dummy zeros and live traces were a flat
  line. SPI was already correct.

---

## [0.4.1] — 2026-08-24

### Fixed

- `CTRL_REG4`'s `ST` (self-test) field was mapped to bits `[3:2]` instead of
  the datasheet's `[2:1]` (Table 37/39). `set_self_test` / `set_st` therefore
  wrote to the reserved bit 3 (which must stay `0` per Table 37, footnote 1)
  and never touched the real `ST0` bit, so self-test mode selection did not
  match real silicon. `ST` is now correctly mapped to bits `[2:1]`; bit 3 is
  left unmodeled so it always reads/writes as `0`.

---

## [0.4.0] — 2026-08-11

### Added

- Self-test API: `set_self_test` / `self_test`, `evaluate_self_test`, and
  Table 4 limit constants (`SELF_TEST_MIN_LSB_2G` / `SELF_TEST_MAX_LSB_2G`).
- High-pass filter config: `HighPassConfig`, `set_high_pass_config` /
  `high_pass_config`, plus `set_reference` / `reference` /
  `reset_high_pass_filter`. Re-exports `Hpm` and `Hpcf`.
- Data-ready helpers: `data_status`, `data_ready`, `temperature_status`
  (`DataStatus` / `TemperatureStatus`).
- FIFO Stream-to-FIFO trigger pin (`FifoTriggerPin`) and `reset_fifo()`.
- Motion presets aligned with ST examples: `free_fall` / `wake_up` /
  6D & 4D orientation helpers, plus threshold constants.
- Orientation decode: `DeviceOrientation`, `DisplayOrientation`, and
  `MotionStatus::orientation()`.
- `MotionAxesConfig::xy_all()` for 4D portrait/landscape setups.

### Fixed

- `ActivityConfig.enable = false` now clears `ACT_THS` / `ACT_DUR` instead of
  leaving a non-zero threshold armed.

### Changed

- **Breaking:** `FifoConfig` gained a `trigger: FifoTriggerPin` field (default
  INT1 via `disabled()` / `enabled()` / `with_trigger()`).

---

## [0.3.1] — 2026-06-06

### Added

- Implemented `accelerometer::Accelerometer` for `Lis2de12`, including
  `accel_norm()` and `sample_rate()`.
- Added ODR-to-Hz mapping tests for `sample_rate()` behavior.

### Changed

- Updated CI workflows:
  - host-oriented `check` jobs now explicitly build for `x86_64-unknown-linux-gnu`
  - `no-std` "all" matrix entry now uses `async,defmt-03` features instead of
    enabling `std`

### Documentation

- Added README examples showing trait-based usage through
  `accelerometer::Accelerometer`, including generic helper integration.

---

## [0.3.0] — 2026-05-19

### Added

- **4-direction (4D) detection** — two new `MotionDetectionMode` variants:
  - `FourDirection` — 4D movement detection (X/Y axes only; Z ignored)
  - `FourDirectionPosition` — 4D position detection (X/Y axes only; Z ignored)

  Both variants set the `SIXD` bit in `INT_CFG` and the corresponding `D4D_INTx`
  bit in `CTRL_REG5`. Ported from `lis2de12_int1/2_pin_detect_4d_set` in the ST
  reference driver.

- **HP filter to outputs** (`set_high_pass_to_outputs` / `high_pass_to_outputs`,
  blocking and async) — controls the `FDS` bit in `CTRL_REG2`. When enabled,
  the accelerometer output registers and FIFO reflect the high-pass filtered
  signal rather than the raw signal.

- **SDO/SA0 pull-up control** (`set_sdo_pullup_connected` /
  `sdo_pullup_connected`, blocking and async) — connects or disconnects the
  internal SDO/SA0 pull-up resistor via `SDO_PU_DISC` in `CTRL_REG0`. Useful
  for reducing power draw or avoiding bus conflicts when the pin is driven
  externally. Pull-up is connected by default (hardware reset state).

### Changed

- **`HPM` enum variants renamed** for clarity (breaking change):
  - `NormalMode` → `NormalWithReset` (accumulator resets on `REFERENCE` register read)
  - `NormalMode2` → `Normal` (accumulator does not reset on reads)
  - `Autoreset` → `AutoresetOnInt` (filter resets automatically on interrupt)
  - Descriptions expanded to explain the accumulator-reset distinction between
    the two normal modes.

- **`HPCF` variant descriptions** now include the full ODR × cutoff-frequency
  lookup table from the ST reference driver, so users can select the right
  cutoff without consulting the datasheet separately.

- **`REFERENCE` register field** description now documents per-full-scale LSb
  weights (~16 @ ±2 g / ~31 @ ±4 g / ~63 @ ±8 g / ~127 @ ±16 g).

### Documentation

- **Threshold LSb weights** added to `MotionConfig::threshold`,
  `ActivityConfig::threshold`, and `ClickConfig::threshold`:
  - Motion / activity: 16 mg @ ±2 g | 32 mg @ ±4 g | 62 mg @ ±8 g | 186 mg @ ±16 g
  - Click: `1 LSb = full_scale / 128`

- **`ActivityConfig::duration`** now documents the hardware formula
  `(8 × value + 1) / ODR` with a worked example.

- **`LatchMode`** variants now explain what happens in each mode:
  - `NonLatched`: pin clears automatically after the latency window
  - `Latched`: pin stays high until the interrupt source register is read

- **`read_temperature` / `read_temperature_async`** (added in 0.2.2) now
  documents why this driver returns a relative delta rather than applying ST's
  implicit 25 °C power-on offset, and shows how to reproduce the ST convention
  when needed.

---

## [0.2.2] — prior release

- Corrected datasheet inconsistencies in register bit positions.
- Fixed `CLICK_SRC` register bit positions.
