# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
with the pre-1.0 convention that breaking changes increment the minor version.

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
