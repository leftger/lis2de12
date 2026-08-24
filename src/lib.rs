//! High-level LIS2DE12 accelerometer driver with synchronous and asynchronous FIFO helpers.
//!
//! This driver supports both I2C and SPI communication interfaces, with blocking and async modes.
//!
//! # Examples
//!
//! ## Blocking I2C
//!
//! ```no_run
//! use lis2de12::{Lis2de12, SlaveAddr};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! // Create driver with default configuration
//! let mut sensor = Lis2de12::new_i2c(i2c, SlaveAddr::Default)?;
//!
//! // Read acceleration in g (floating point)
//! let accel = sensor.read_g()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Blocking SPI
//!
//! ```no_run
//! use lis2de12::Lis2de12;
//! # use embedded_hal::spi::SpiDevice;
//! # fn example<SPI: SpiDevice>(spi: SPI) -> Result<(), lis2de12::Error<SPI::Error>>
//! # where SPI::Error: core::fmt::Debug {
//!
//! // Create driver with SPI interface
//! let mut sensor = Lis2de12::new_spi(spi)?;
//!
//! // Read acceleration in milli-g (integer)
//! let accel_mg = sensor.read_mg()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Async I2C
//!
//! ```ignore
//! use lis2de12::{Lis2de12Async, SlaveAddr};
//! # use embedded_hal_async::i2c::I2c;
//! # async fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! // Create async driver with I2C
//! let mut sensor = Lis2de12Async::new_i2c(i2c, SlaveAddr::Default).await?;
//!
//! // Read acceleration asynchronously
//! let accel = sensor.read_g().await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Async SPI
//!
//! ```ignore
//! use lis2de12::Lis2de12Async;
//! # use embedded_hal_async::spi::SpiDevice;
//! # async fn example<SPI: SpiDevice>(spi: SPI) -> Result<(), lis2de12::Error<SPI::Error>>
//! # where SPI::Error: core::fmt::Debug {
//!
//! // Create async driver with SPI
//! let mut sensor = Lis2de12Async::new_spi(spi).await?;
//!
//! // Read acceleration asynchronously
//! let accel = sensor.read_g().await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Custom Configuration
//!
//! ```no_run
//! use lis2de12::{Lis2de12, Lis2de12Config, SlaveAddr, Odr, Fs};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! let config = Lis2de12Config {
//!     odr: Odr::FourHundredHz,   // 400 Hz output data rate
//!     scale: Fs::PlusMinus8G,     // ±8g full scale
//!     ..Default::default()
//! };
//!
//! let mut sensor = Lis2de12::new_i2c_with_config(i2c, SlaveAddr::Default, config)?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Temperature Sensor
//!
//! ```no_run
//! use lis2de12::{Lis2de12, Lis2de12Config, SlaveAddr};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! // Enable temperature sensor in configuration
//! let config = Lis2de12Config {
//!     temperature_enable: true,
//!     ..Default::default()
//! };
//!
//! let mut sensor = Lis2de12::new_i2c_with_config(i2c, SlaveAddr::Default, config)?;
//!
//! // Or enable it after initialization
//! sensor.set_temperature_sensor(true)?;
//!
//! // Read temperature change in °C (relative to power-on reference)
//! // Positive = warmer, negative = cooler
//! let temp_delta = sensor.read_temperature()?;
//!
//! // Or read raw 16-bit left-justified value
//! let temp_raw = sensor.read_temperature_raw()?;
//! # Ok(())
//! # }
//! ```
//!
//! # SPI Mode
//!
//! The LIS2DE12 requires SPI Mode 0 or Mode 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1).
//! The driver uses the `embedded-hal` `SpiDevice` trait which handles chip select automatically.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(missing_docs)]
#![deny(warnings)]
#![allow(clippy::missing_errors_doc)]

use core::cmp;
use core::fmt::Debug;

pub use accelerometer::Error;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, ErrorKind, RawAccelerometer};
#[cfg(feature = "async")]
use device_driver::{AsyncBufferInterface, AsyncRegisterInterface};
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal_async;
use hal::i2c::I2c;
#[cfg(feature = "async")]
use hal_async::i2c::I2c as AsyncI2c;
#[cfg(feature = "async")]
use hal_async::spi::SpiDevice as AsyncSpiDevice;

#[allow(unsafe_code)]
#[allow(missing_docs)]
#[allow(clippy::doc_markdown, clippy::missing_errors_doc, clippy::identity_op)]
mod generated {
    device_driver::create_device!(
        device_name: Lis2de12Device,
        manifest: "src/lis2de12.yaml"
    );
}

pub use generated::{Fm, Fs, Hpcf, Hpm, Lis2de12Device, Odr, St, TempEn, field_sets};

/// Number of bytes per FIFO frame (X, Y, Z 16-bit samples).
pub const FIFO_FRAME_BYTES: usize = 6;
/// Maximum FIFO depth in frames.
pub const FIFO_CAPACITY: u8 = 32;
/// FIFO frame buffer alias.
pub type FifoFrame = [u8; FIFO_FRAME_BYTES];
/// Highest programmable FIFO watermark level.
pub const FIFO_WATERMARK_MAX: u8 = 31;

/// Minimum self-test output change at ±2 g (datasheet Table 4), in 8-bit `LSb`.
pub const SELF_TEST_MIN_LSB_2G: i16 = 4;
/// Maximum self-test output change at ±2 g (datasheet Table 4), in 8-bit `LSb`.
pub const SELF_TEST_MAX_LSB_2G: i16 = 90;

/// ST free-fall example threshold at ±2 g (`INT_THS` = 0x16 → ~350 mg).
pub const FREE_FALL_THRESHOLD_350MG_2G: u8 = 0x16;
/// ST free-fall example duration (`INT_DURATION` = 0x03, `LSb` = 1/ODR).
pub const FREE_FALL_DURATION_DEFAULT: u8 = 0x03;
/// ST wake-up example threshold at ±2 g (`INT_THS` = 0x10 → ~250 mg).
pub const WAKE_UP_THRESHOLD_250MG_2G: u8 = 0x10;
/// Typical 6D/4D orientation threshold at ±2 g (~512 mg ≈ 0.5 g).
pub const ORIENTATION_THRESHOLD_512MG_2G: u8 = 0x20;

/// High-level FIFO operating modes mirroring the `FM` register field.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FifoMode {
    /// FIFO disabled (register reads access the most recent sample).
    Bypass,
    /// FIFO mode (stops on full).
    Fifo,
    /// Stream mode (circular buffer).
    Stream,
    /// Stream-to-FIFO mode (stream until trigger then stop-on-full).
    StreamToFifo,
}

/// Interrupt pin used as the Stream-to-FIFO trigger (`TR` bit in `FIFO_CTRL_REG`).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum FifoTriggerPin {
    /// Trigger on INT1 (`TR` = 0, default).
    #[default]
    Int1,
    /// Trigger on INT2 (`TR` = 1).
    Int2,
}

impl From<FifoMode> for Fm {
    fn from(mode: FifoMode) -> Self {
        match mode {
            FifoMode::Bypass => Fm::Bypass,
            FifoMode::Fifo => Fm::Fifo,
            FifoMode::Stream => Fm::Stream,
            FifoMode::StreamToFifo => Fm::StreamToFifo,
        }
    }
}

impl From<Fm> for FifoMode {
    fn from(mode: Fm) -> Self {
        match mode {
            Fm::Bypass => FifoMode::Bypass,
            Fm::Fifo => FifoMode::Fifo,
            Fm::Stream => FifoMode::Stream,
            Fm::StreamToFifo => FifoMode::StreamToFifo,
        }
    }
}

// ============================================================================
// Interrupt Types
// ============================================================================

/// Interrupt pin polarity.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum InterruptPolarity {
    /// Interrupt pins are active high (default).
    #[default]
    ActiveHigh,
    /// Interrupt pins are active low.
    ActiveLow,
}

/// Latch mode for interrupt signals.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum LatchMode {
    /// Interrupt signal is not latched: the pin stays high only for the duration of the
    /// latency window, then clears automatically regardless of whether the source register
    /// has been read.
    #[default]
    NonLatched,
    /// Interrupt signal is latched: the pin stays high until the interrupt source register
    /// (`INT1_SRC`, `INT2_SRC`, or `CLICK_SRC`) is read, which also clears the flag.
    Latched,
}

/// Returns `(aoi, sixd, d4d)` register bits for a given motion detection mode.
fn mode_bits(mode: MotionDetectionMode) -> (bool, bool, bool) {
    match mode {
        MotionDetectionMode::OrCombination => (false, false, false),
        MotionDetectionMode::AndCombination => (true, false, false),
        MotionDetectionMode::SixDirection => (false, true, false),
        MotionDetectionMode::SixDirectionPosition => (true, true, false),
        MotionDetectionMode::FourDirection => (false, true, true),
        MotionDetectionMode::FourDirectionPosition => (true, true, true),
    }
}

/// Motion detection mode determining how axis events are combined.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum MotionDetectionMode {
    /// OR combination of enabled axis events.
    #[default]
    OrCombination,
    /// AND combination of enabled axis events.
    AndCombination,
    /// 6-direction movement detection (all axes).
    SixDirection,
    /// 6-direction position detection (all axes).
    SixDirectionPosition,
    /// 4-direction movement detection (X/Y only; Z ignored).
    /// Sets `D4D_INTx` in `CTRL_REG5` alongside the SIXD bit.
    FourDirection,
    /// 4-direction position detection (X/Y only; Z ignored).
    /// Sets `D4D_INTx` in `CTRL_REG5` alongside the SIXD bit.
    FourDirectionPosition,
}

/// Per-axis motion event enables for high/low thresholds.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct MotionAxesConfig {
    /// Enable X-axis high event detection.
    pub x_high: bool,
    /// Enable X-axis low event detection.
    pub x_low: bool,
    /// Enable Y-axis high event detection.
    pub y_high: bool,
    /// Enable Y-axis low event detection.
    pub y_low: bool,
    /// Enable Z-axis high event detection.
    pub z_high: bool,
    /// Enable Z-axis low event detection.
    pub z_low: bool,
}

impl MotionAxesConfig {
    /// No axes enabled.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            x_high: false,
            x_low: false,
            y_high: false,
            y_low: false,
            z_high: false,
            z_low: false,
        }
    }

    /// All high events enabled on all axes.
    #[must_use]
    pub const fn all_high() -> Self {
        Self {
            x_high: true,
            x_low: false,
            y_high: true,
            y_low: false,
            z_high: true,
            z_low: false,
        }
    }

    /// All low events enabled on all axes.
    #[must_use]
    pub const fn all_low() -> Self {
        Self {
            x_high: false,
            x_low: true,
            y_high: false,
            y_low: true,
            z_high: false,
            z_low: true,
        }
    }

    /// All events enabled on all axes (typical for 6D orientation).
    #[must_use]
    pub const fn all() -> Self {
        Self {
            x_high: true,
            x_low: true,
            y_high: true,
            y_low: true,
            z_high: true,
            z_low: true,
        }
    }

    /// X/Y high and low events only (typical for 4D portrait/landscape).
    #[must_use]
    pub const fn xy_all() -> Self {
        Self {
            x_high: true,
            x_low: true,
            y_high: true,
            y_low: true,
            z_high: false,
            z_low: false,
        }
    }

    /// Returns true if any axis event is enabled.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.x_high || self.x_low || self.y_high || self.y_low || self.z_high || self.z_low
    }
}

/// Motion detection configuration for interrupt generators (IA1/IA2).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MotionConfig {
    /// Enable motion detection.
    pub enable: bool,
    /// Motion detection mode (OR/AND/6D).
    pub mode: MotionDetectionMode,
    /// Per-axis event enables.
    pub axes: MotionAxesConfig,
    /// Threshold value (0-127). `LSb` weight per full-scale setting:
    /// 16 mg @ ±2 g | 32 mg @ ±4 g | 62 mg @ ±8 g | 186 mg @ ±16 g.
    pub threshold: u8,
    /// Duration value (0-127, `LSb` = 1/ODR).
    pub duration: u8,
    /// Latch mode for interrupt signal.
    pub latch: LatchMode,
    /// Enable high-pass filter for this interrupt.
    pub high_pass_filter: bool,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            enable: false,
            mode: MotionDetectionMode::OrCombination,
            axes: MotionAxesConfig::none(),
            threshold: 0,
            duration: 0,
            latch: LatchMode::NonLatched,
            high_pass_filter: false,
        }
    }
}

impl MotionConfig {
    /// Create a disabled motion configuration.
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            enable: false,
            mode: MotionDetectionMode::OrCombination,
            axes: MotionAxesConfig::none(),
            threshold: 0,
            duration: 0,
            latch: LatchMode::NonLatched,
            high_pass_filter: false,
        }
    }

    /// Set enable state.
    #[must_use]
    pub const fn with_enable(mut self, enable: bool) -> Self {
        self.enable = enable;
        self
    }

    /// Set motion detection mode.
    #[must_use]
    pub const fn with_mode(mut self, mode: MotionDetectionMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set axis configuration.
    #[must_use]
    pub const fn with_axes(mut self, axes: MotionAxesConfig) -> Self {
        self.axes = axes;
        self
    }

    /// Set threshold (0-127).
    #[must_use]
    pub const fn with_threshold(mut self, threshold: u8) -> Self {
        self.threshold = if threshold > 127 { 127 } else { threshold };
        self
    }

    /// Set duration (0-127).
    #[must_use]
    pub const fn with_duration(mut self, duration: u8) -> Self {
        self.duration = if duration > 127 { 127 } else { duration };
        self
    }

    /// Set latch mode.
    #[must_use]
    pub const fn with_latch(mut self, latch: LatchMode) -> Self {
        self.latch = latch;
        self
    }

    /// Set high-pass filter enable.
    #[must_use]
    pub const fn with_high_pass_filter(mut self, enable: bool) -> Self {
        self.high_pass_filter = enable;
        self
    }

    /// Free-fall detection: AND of X/Y/Z low events (`INT_CFG` = 0x95 pattern).
    ///
    /// Matches the ST example (threshold ≈ 350 mg / duration = 3 at ±2 g / 100 Hz).
    /// Route the matching IA source to an interrupt pin via [`InterruptConfig`].
    #[must_use]
    pub const fn free_fall(threshold: u8, duration: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::AndCombination,
            axes: MotionAxesConfig::all_low(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: if duration > 127 { 127 } else { duration },
            latch: LatchMode::Latched,
            high_pass_filter: false,
        }
    }

    /// Free-fall using ST's ±2 g example values (~350 mg, duration 3).
    #[must_use]
    pub const fn free_fall_default() -> Self {
        Self::free_fall(FREE_FALL_THRESHOLD_350MG_2G, FREE_FALL_DURATION_DEFAULT)
    }

    /// Wake-up / motion detection: OR of X/Y/Z high events on HP-filtered data.
    ///
    /// Matches the ST wake-up example (threshold ≈ 250 mg at ±2 g). After applying,
    /// call [`Lis2de12::reset_high_pass_filter`] once so the filter reference matches
    /// the current tilt.
    #[must_use]
    pub const fn wake_up(threshold: u8, duration: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::OrCombination,
            axes: MotionAxesConfig::all_high(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: if duration > 127 { 127 } else { duration },
            latch: LatchMode::Latched,
            high_pass_filter: true,
        }
    }

    /// Wake-up using ST's ±2 g example threshold (~250 mg) and duration 0.
    #[must_use]
    pub const fn wake_up_default() -> Self {
        Self::wake_up(WAKE_UP_THRESHOLD_250MG_2G, 0)
    }

    /// 6D position recognition (interrupt while orientation stays in a known zone).
    #[must_use]
    pub const fn orientation_6d_position(threshold: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::SixDirectionPosition,
            axes: MotionAxesConfig::all(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: 0,
            latch: LatchMode::Latched,
            high_pass_filter: false,
        }
    }

    /// 6D movement recognition (interrupt when entering a known zone).
    #[must_use]
    pub const fn orientation_6d_movement(threshold: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::SixDirection,
            axes: MotionAxesConfig::all(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: 0,
            latch: LatchMode::Latched,
            high_pass_filter: false,
        }
    }

    /// 4D position recognition for portrait/landscape (X/Y only; Z ignored).
    #[must_use]
    pub const fn orientation_4d_position(threshold: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::FourDirectionPosition,
            axes: MotionAxesConfig::xy_all(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: 0,
            latch: LatchMode::Latched,
            high_pass_filter: false,
        }
    }

    /// 4D movement recognition for portrait/landscape (X/Y only; Z ignored).
    #[must_use]
    pub const fn orientation_4d_movement(threshold: u8) -> Self {
        Self {
            enable: true,
            mode: MotionDetectionMode::FourDirection,
            axes: MotionAxesConfig::xy_all(),
            threshold: if threshold > 127 { 127 } else { threshold },
            duration: 0,
            latch: LatchMode::Latched,
            high_pass_filter: false,
        }
    }

    /// 6D position recognition with a typical ~0.5 g threshold at ±2 g.
    #[must_use]
    pub const fn orientation_6d_default() -> Self {
        Self::orientation_6d_position(ORIENTATION_THRESHOLD_512MG_2G)
    }

    /// 4D position recognition with a typical ~0.5 g threshold at ±2 g.
    #[must_use]
    pub const fn orientation_4d_default() -> Self {
        Self::orientation_4d_position(ORIENTATION_THRESHOLD_512MG_2G)
    }
}

/// Per-axis click event enables for single/double click.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct ClickAxesConfig {
    /// Enable X-axis single-click detection.
    pub x_single: bool,
    /// Enable X-axis double-click detection.
    pub x_double: bool,
    /// Enable Y-axis single-click detection.
    pub y_single: bool,
    /// Enable Y-axis double-click detection.
    pub y_double: bool,
    /// Enable Z-axis single-click detection.
    pub z_single: bool,
    /// Enable Z-axis double-click detection.
    pub z_double: bool,
}

impl ClickAxesConfig {
    /// No axes enabled.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            x_single: false,
            x_double: false,
            y_single: false,
            y_double: false,
            z_single: false,
            z_double: false,
        }
    }

    /// All single-click events enabled.
    #[must_use]
    pub const fn all_single() -> Self {
        Self {
            x_single: true,
            x_double: false,
            y_single: true,
            y_double: false,
            z_single: true,
            z_double: false,
        }
    }

    /// All double-click events enabled.
    #[must_use]
    pub const fn all_double() -> Self {
        Self {
            x_single: false,
            x_double: true,
            y_single: false,
            y_double: true,
            z_single: false,
            z_double: true,
        }
    }

    /// All click events enabled.
    #[must_use]
    pub const fn all() -> Self {
        Self {
            x_single: true,
            x_double: true,
            y_single: true,
            y_double: true,
            z_single: true,
            z_double: true,
        }
    }

    /// Returns true if any click event is enabled.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.x_single || self.x_double || self.y_single || self.y_double || self.z_single || self.z_double
    }
}

/// Click detection configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ClickConfig {
    /// Enable click detection.
    pub enable: bool,
    /// Per-axis click event enables.
    pub axes: ClickAxesConfig,
    /// Click threshold (0-127). 1 `LSb` = `full_scale` / 128, so:
    /// 15.6 mg @ ±2 g | 31.2 mg @ ±4 g | 62.5 mg @ ±8 g | 187.5 mg @ ±16 g.
    pub threshold: u8,
    /// Time limit for click detection (0-127, `LSb` = 1/ODR).
    pub time_limit: u8,
    /// Time latency for double-click detection (0-255, `LSb` = 1/ODR).
    pub time_latency: u8,
    /// Time window for double-click detection (0-255, `LSb` = 1/ODR).
    pub time_window: u8,
    /// Latch mode for click interrupt.
    pub latch: LatchMode,
    /// Enable high-pass filter for click detection.
    pub high_pass_filter: bool,
}

impl Default for ClickConfig {
    fn default() -> Self {
        Self {
            enable: false,
            axes: ClickAxesConfig::none(),
            threshold: 0,
            time_limit: 0,
            time_latency: 0,
            time_window: 0,
            latch: LatchMode::NonLatched,
            high_pass_filter: false,
        }
    }
}

impl ClickConfig {
    /// Create a disabled click configuration.
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            enable: false,
            axes: ClickAxesConfig::none(),
            threshold: 0,
            time_limit: 0,
            time_latency: 0,
            time_window: 0,
            latch: LatchMode::NonLatched,
            high_pass_filter: false,
        }
    }

    /// Set enable state.
    #[must_use]
    pub const fn with_enable(mut self, enable: bool) -> Self {
        self.enable = enable;
        self
    }

    /// Set axis configuration.
    #[must_use]
    pub const fn with_axes(mut self, axes: ClickAxesConfig) -> Self {
        self.axes = axes;
        self
    }

    /// Set threshold (0-127).
    #[must_use]
    pub const fn with_threshold(mut self, threshold: u8) -> Self {
        self.threshold = if threshold > 127 { 127 } else { threshold };
        self
    }

    /// Set time limit (0-127).
    #[must_use]
    pub const fn with_time_limit(mut self, time_limit: u8) -> Self {
        self.time_limit = if time_limit > 127 { 127 } else { time_limit };
        self
    }

    /// Set time latency (0-255).
    #[must_use]
    pub const fn with_time_latency(mut self, time_latency: u8) -> Self {
        self.time_latency = time_latency;
        self
    }

    /// Set time window (0-255).
    #[must_use]
    pub const fn with_time_window(mut self, time_window: u8) -> Self {
        self.time_window = time_window;
        self
    }

    /// Set latch mode.
    #[must_use]
    pub const fn with_latch(mut self, latch: LatchMode) -> Self {
        self.latch = latch;
        self
    }

    /// Set high-pass filter enable.
    #[must_use]
    pub const fn with_high_pass_filter(mut self, enable: bool) -> Self {
        self.high_pass_filter = enable;
        self
    }
}

/// Activity/inactivity detection configuration (sleep-to-wake).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct ActivityConfig {
    /// Enable activity detection.
    ///
    /// When `false`, `ACT_THS` and `ACT_DUR` are cleared (threshold 0 disarms sleep-to-wake).
    pub enable: bool,
    /// Activation threshold (0-127). `LSb` weight per full-scale setting:
    /// 16 mg @ ±2 g | 32 mg @ ±4 g | 62 mg @ ±8 g | 186 mg @ ±16 g.
    pub threshold: u8,
    /// Minimum inactivity duration before entering sleep mode (0-255).
    /// Actual time = (8 × value + 1) / ODR. At 100 Hz, value=99 → ~8 s.
    pub duration: u8,
}

impl ActivityConfig {
    /// Create a disabled activity configuration.
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            enable: false,
            threshold: 0,
            duration: 0,
        }
    }

    /// Set enable state.
    #[must_use]
    pub const fn with_enable(mut self, enable: bool) -> Self {
        self.enable = enable;
        self
    }

    /// Set threshold (0-127).
    #[must_use]
    pub const fn with_threshold(mut self, threshold: u8) -> Self {
        self.threshold = if threshold > 127 { 127 } else { threshold };
        self
    }

    /// Set duration (0-255).
    #[must_use]
    pub const fn with_duration(mut self, duration: u8) -> Self {
        self.duration = duration;
        self
    }
}

/// INT1 pin routing configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct Int1Routing {
    /// Route click interrupt to INT1.
    pub click: bool,
    /// Route IA1 (motion 1) interrupt to INT1.
    pub ia1: bool,
    /// Route IA2 (motion 2) interrupt to INT1.
    pub ia2: bool,
    /// Route data ready interrupt to INT1.
    pub data_ready: bool,
    /// Route FIFO watermark interrupt to INT1.
    pub fifo_watermark: bool,
    /// Route FIFO overrun interrupt to INT1.
    pub fifo_overrun: bool,
}

impl Int1Routing {
    /// No interrupts routed to INT1.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            click: false,
            ia1: false,
            ia2: false,
            data_ready: false,
            fifo_watermark: false,
            fifo_overrun: false,
        }
    }

    /// Returns true if any interrupt is routed to INT1.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.click || self.ia1 || self.ia2 || self.data_ready || self.fifo_watermark || self.fifo_overrun
    }
}

/// INT2 pin routing configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct Int2Routing {
    /// Route click interrupt to INT2.
    pub click: bool,
    /// Route IA1 (motion 1) interrupt to INT2.
    pub ia1: bool,
    /// Route IA2 (motion 2) interrupt to INT2.
    pub ia2: bool,
    /// Route boot interrupt to INT2.
    pub boot: bool,
    /// Route activity interrupt to INT2.
    pub activity: bool,
}

impl Int2Routing {
    /// No interrupts routed to INT2.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            click: false,
            ia1: false,
            ia2: false,
            boot: false,
            activity: false,
        }
    }

    /// Returns true if any interrupt is routed to INT2.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.click || self.ia1 || self.ia2 || self.boot || self.activity
    }
}

/// Combined interrupt routing and polarity configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct InterruptConfig {
    /// Interrupt pin polarity.
    pub polarity: InterruptPolarity,
    /// INT1 pin routing.
    pub int1: Int1Routing,
    /// INT2 pin routing.
    pub int2: Int2Routing,
}

impl InterruptConfig {
    /// Create a configuration with no interrupts routed.
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            polarity: InterruptPolarity::ActiveHigh,
            int1: Int1Routing::none(),
            int2: Int2Routing::none(),
        }
    }

    /// Set interrupt polarity.
    #[must_use]
    pub const fn with_polarity(mut self, polarity: InterruptPolarity) -> Self {
        self.polarity = polarity;
        self
    }

    /// Set INT1 routing.
    #[must_use]
    pub const fn with_int1(mut self, int1: Int1Routing) -> Self {
        self.int1 = int1;
        self
    }

    /// Set INT2 routing.
    #[must_use]
    pub const fn with_int2(mut self, int2: Int2Routing) -> Self {
        self.int2 = int2;
        self
    }
}

// ============================================================================
// Interrupt Status Types
// ============================================================================

/// Device face / axis direction decoded from a 6D/4D `INT_SRC` snapshot.
///
/// Values correspond to which axis event bit is set (`XH`/`XL`/…). Mapping to
/// portrait/landscape depends on how the sensor is mounted in the product.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum DeviceOrientation {
    /// No single known zone (idle, transition, or multiple bits set).
    #[default]
    Unknown,
    /// `XH` — acceleration above threshold on +X.
    PlusX,
    /// `XL` — acceleration above threshold on −X.
    MinusX,
    /// `YH` — acceleration above threshold on +Y.
    PlusY,
    /// `YL` — acceleration above threshold on −Y.
    MinusY,
    /// `ZH` — acceleration above threshold on +Z.
    PlusZ,
    /// `ZL` — acceleration above threshold on −Z.
    MinusZ,
}

impl DeviceOrientation {
    /// Returns `true` when a single known zone was decoded.
    #[must_use]
    pub const fn is_known(self) -> bool {
        !matches!(self, Self::Unknown)
    }

    /// Portrait/landscape helper for 4D X/Y events (Z faces map to [`Unknown`](Self::Unknown)).
    ///
    /// Convention: ±Y → portrait, ±X → landscape. Adjust in application code if the
    /// module is mounted differently.
    #[must_use]
    pub const fn display_orientation(self) -> Option<DisplayOrientation> {
        match self {
            Self::PlusY | Self::MinusY => Some(DisplayOrientation::Portrait),
            Self::PlusX | Self::MinusX => Some(DisplayOrientation::Landscape),
            Self::PlusZ | Self::MinusZ | Self::Unknown => None,
        }
    }
}

/// Coarse display orientation derived from 4D X/Y events.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DisplayOrientation {
    /// Gravity predominantly along ±Y.
    Portrait,
    /// Gravity predominantly along ±X.
    Landscape,
}

/// Motion detection status from `INT1_SRC` or `INT2_SRC` registers.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct MotionStatus {
    /// Interrupt active flag.
    pub active: bool,
    /// X-axis high event detected.
    pub x_high: bool,
    /// X-axis low event detected.
    pub x_low: bool,
    /// Y-axis high event detected.
    pub y_high: bool,
    /// Y-axis low event detected.
    pub y_low: bool,
    /// Z-axis high event detected.
    pub z_high: bool,
    /// Z-axis low event detected.
    pub z_low: bool,
}

impl MotionStatus {
    /// Returns true if the interrupt is active.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Returns true if any X-axis event was detected.
    #[must_use]
    pub const fn x_event(&self) -> bool {
        self.x_high || self.x_low
    }

    /// Returns true if any Y-axis event was detected.
    #[must_use]
    pub const fn y_event(&self) -> bool {
        self.y_high || self.y_low
    }

    /// Returns true if any Z-axis event was detected.
    #[must_use]
    pub const fn z_event(&self) -> bool {
        self.z_high || self.z_low
    }

    /// Returns true if any axis event was detected.
    #[must_use]
    pub const fn any_event(&self) -> bool {
        self.x_high || self.x_low || self.y_high || self.y_low || self.z_high || self.z_low
    }

    /// Decode 6D/4D orientation from the axis event bits.
    ///
    /// Returns [`DeviceOrientation::Unknown`] unless exactly one of the six
    /// direction bits is set (transition zones / multi-bit states are unknown).
    #[must_use]
    pub const fn orientation(self) -> DeviceOrientation {
        let mut found = DeviceOrientation::Unknown;
        let mut count = 0u8;

        if self.x_high {
            found = DeviceOrientation::PlusX;
            count += 1;
        }
        if self.x_low {
            found = DeviceOrientation::MinusX;
            count += 1;
        }
        if self.y_high {
            found = DeviceOrientation::PlusY;
            count += 1;
        }
        if self.y_low {
            found = DeviceOrientation::MinusY;
            count += 1;
        }
        if self.z_high {
            found = DeviceOrientation::PlusZ;
            count += 1;
        }
        if self.z_low {
            found = DeviceOrientation::MinusZ;
            count += 1;
        }

        if count == 1 { found } else { DeviceOrientation::Unknown }
    }
}

impl From<field_sets::Int1Src> for MotionStatus {
    fn from(raw: field_sets::Int1Src) -> Self {
        Self {
            active: raw.ia(),
            x_high: raw.xh(),
            x_low: raw.xl(),
            y_high: raw.yh(),
            y_low: raw.yl(),
            z_high: raw.zh(),
            z_low: raw.zl(),
        }
    }
}

impl From<field_sets::Int2Src> for MotionStatus {
    fn from(raw: field_sets::Int2Src) -> Self {
        Self {
            active: raw.ia(),
            x_high: raw.xh(),
            x_low: raw.xl(),
            y_high: raw.yh(),
            y_low: raw.yl(),
            z_high: raw.zh(),
            z_low: raw.zl(),
        }
    }
}

/// Click detection status from `CLICK_SRC` register.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct ClickStatus {
    /// Click interrupt active flag.
    pub active: bool,
    /// Double-click detected.
    pub double_click: bool,
    /// Single-click detected.
    pub single_click: bool,
    /// Click sign (true = negative direction).
    pub negative: bool,
    /// X-axis click event.
    pub x: bool,
    /// Y-axis click event.
    pub y: bool,
    /// Z-axis click event.
    pub z: bool,
}

impl ClickStatus {
    /// Returns true if the click interrupt is active.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Returns true if a double-click was detected.
    #[must_use]
    pub const fn is_double_click(&self) -> bool {
        self.double_click
    }

    /// Returns true if a single-click was detected.
    #[must_use]
    pub const fn is_single_click(&self) -> bool {
        self.single_click
    }

    /// Returns true if the click was in the negative direction.
    #[must_use]
    pub const fn is_negative(&self) -> bool {
        self.negative
    }

    /// Returns true if any click event was detected on any axis.
    #[must_use]
    pub const fn any_event(&self) -> bool {
        self.x || self.y || self.z
    }
}

impl From<field_sets::ClickSrc> for ClickStatus {
    fn from(raw: field_sets::ClickSrc) -> Self {
        Self {
            active: raw.ia(),
            double_click: raw.d_click(),
            single_click: raw.s_click(),
            negative: raw.sign(),
            x: raw.x(),
            y: raw.y(),
            z: raw.z(),
        }
    }
}

/// Combined interrupt sources status.
#[derive(Copy, Clone, Debug, Default)]
pub struct InterruptSources {
    /// Motion 1 (IA1) status.
    pub motion1: MotionStatus,
    /// Motion 2 (IA2) status.
    pub motion2: MotionStatus,
    /// Click detection status.
    pub click: ClickStatus,
}

impl InterruptSources {
    /// Returns true if any interrupt source is active.
    #[must_use]
    pub const fn any_active(&self) -> bool {
        self.motion1.active || self.motion2.active || self.click.active
    }
}

/// FIFO configuration options.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[must_use]
pub struct FifoConfig {
    /// Enable or disable the hardware FIFO.
    pub enable: bool,
    /// FIFO operating mode to apply when enabled.
    pub mode: FifoMode,
    /// Optional watermark threshold (0–31 frames).
    pub watermark: Option<u8>,
    /// Stream-to-FIFO trigger pin selection (`TR` bit).
    pub trigger: FifoTriggerPin,
}

impl FifoConfig {
    /// Disabled FIFO configuration helper.
    pub const fn disabled() -> Self {
        Self {
            enable: false,
            mode: FifoMode::Bypass,
            watermark: None,
            trigger: FifoTriggerPin::Int1,
        }
    }

    /// Enabled FIFO configuration helper with the given mode.
    pub const fn enabled(mode: FifoMode) -> Self {
        Self {
            enable: true,
            mode,
            watermark: None,
            trigger: FifoTriggerPin::Int1,
        }
    }

    /// Attach a watermark level to this configuration.
    pub const fn with_watermark(self, watermark: u8) -> Self {
        Self {
            watermark: Some(watermark),
            ..self
        }
    }

    /// Select the Stream-to-FIFO trigger pin.
    pub const fn with_trigger(self, trigger: FifoTriggerPin) -> Self {
        Self { trigger, ..self }
    }

    /// Watermark value written to the device.
    #[must_use]
    fn effective_threshold(self) -> u8 {
        if self.enable {
            match self.watermark {
                Some(level) if level <= FIFO_WATERMARK_MAX => level,
                Some(_) | None => FIFO_WATERMARK_MAX,
            }
        } else {
            0
        }
    }

    /// `TR` bit value for `FIFO_CTRL_REG`.
    #[must_use]
    const fn trigger_bit(self) -> bool {
        matches!(self.trigger, FifoTriggerPin::Int2)
    }
}

/// High-pass filter configuration (`CTRL_REG2` mode/cutoff/FDS bits).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct HighPassConfig {
    /// High-pass filter mode (`HPM`).
    pub mode: Hpm,
    /// Cutoff frequency selection (`HPCF`), as a fraction of ODR.
    pub cutoff: Hpcf,
    /// Route filtered data to output registers and FIFO (`FDS`).
    pub filtered_data_selection: bool,
}

impl Default for HighPassConfig {
    fn default() -> Self {
        Self {
            mode: Hpm::NormalWithReset,
            cutoff: Hpcf::TwoPercent,
            filtered_data_selection: false,
        }
    }
}

impl HighPassConfig {
    /// Default filter configuration (normal-with-reset, ~2% ODR cutoff, FDS off).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: Hpm::NormalWithReset,
            cutoff: Hpcf::TwoPercent,
            filtered_data_selection: false,
        }
    }

    /// Set filter mode.
    #[must_use]
    pub const fn with_mode(mut self, mode: Hpm) -> Self {
        self.mode = mode;
        self
    }

    /// Set cutoff selection.
    #[must_use]
    pub const fn with_cutoff(mut self, cutoff: Hpcf) -> Self {
        self.cutoff = cutoff;
        self
    }

    /// Enable or disable filtered-data selection (`FDS`).
    #[must_use]
    pub const fn with_filtered_data_selection(mut self, enable: bool) -> Self {
        self.filtered_data_selection = enable;
        self
    }
}

/// Acceleration data-ready / overrun status from `STATUS_REG`.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct DataStatus {
    /// New data available on X, Y, and Z.
    pub xyz_data_available: bool,
    /// X-axis new data available.
    pub x_data_available: bool,
    /// Y-axis new data available.
    pub y_data_available: bool,
    /// Z-axis new data available.
    pub z_data_available: bool,
    /// X/Y/Z overrun (new data overwrote unread data).
    pub xyz_overrun: bool,
    /// X-axis overrun.
    pub x_overrun: bool,
    /// Y-axis overrun.
    pub y_overrun: bool,
    /// Z-axis overrun.
    pub z_overrun: bool,
}

impl DataStatus {
    /// Returns `true` when a full XYZ sample is ready to read.
    #[must_use]
    pub const fn data_ready(self) -> bool {
        self.xyz_data_available
    }

    /// Returns `true` if any axis overrun flag is set.
    #[must_use]
    pub const fn any_overrun(self) -> bool {
        self.xyz_overrun || self.x_overrun || self.y_overrun || self.z_overrun
    }
}

impl From<field_sets::StatusReg> for DataStatus {
    fn from(raw: field_sets::StatusReg) -> Self {
        Self {
            xyz_data_available: raw.zyxda(),
            x_data_available: raw.xda(),
            y_data_available: raw.yda(),
            z_data_available: raw.zda(),
            xyz_overrun: raw.zyxor(),
            x_overrun: raw.xor(),
            y_overrun: raw.yor(),
            z_overrun: raw.zor(),
        }
    }
}

/// Temperature data-ready / overrun status from `STATUS_REG_AUX`.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct TemperatureStatus {
    /// New temperature data available.
    pub data_available: bool,
    /// Temperature data overrun.
    pub overrun: bool,
}

impl From<field_sets::StatusRegAux> for TemperatureStatus {
    fn from(raw: field_sets::StatusRegAux) -> Self {
        Self {
            data_available: raw.tda(),
            overrun: raw.tor(),
        }
    }
}

/// Result of comparing no-self-test and self-test acceleration samples.
#[derive(Copy, Clone, Debug)]
pub struct SelfTestResult {
    /// Absolute per-axis delta (`|ST − nost|`) in 8-bit `LSb`.
    pub delta: I16x3,
    /// X-axis delta within datasheet limits (scaled for current full-scale).
    pub x_ok: bool,
    /// Y-axis delta within datasheet limits (scaled for current full-scale).
    pub y_ok: bool,
    /// Z-axis delta within datasheet limits (scaled for current full-scale).
    pub z_ok: bool,
}

impl SelfTestResult {
    /// Returns `true` when all axes pass.
    #[must_use]
    pub const fn passed(self) -> bool {
        self.x_ok && self.y_ok && self.z_ok
    }
}

/// Evaluate self-test output change against datasheet Table 4 limits.
///
/// `nost` is a sample with self-test disabled; `st` is a sample after enabling
/// self-test (discard the first two samples after enabling, per the datasheet).
/// Limits are specified at ±2 g and scaled by sensitivity for other full-scale settings.
#[must_use]
pub fn evaluate_self_test(nost: I16x3, st: I16x3, scale: Fs) -> SelfTestResult {
    let delta = I16x3::new((st.x - nost.x).abs(), (st.y - nost.y).abs(), (st.z - nost.z).abs());
    let (min_lsb, max_lsb) = self_test_limits_lsb(scale);
    SelfTestResult {
        delta,
        x_ok: delta.x >= min_lsb && delta.x <= max_lsb,
        y_ok: delta.y >= min_lsb && delta.y <= max_lsb,
        z_ok: delta.z >= min_lsb && delta.z <= max_lsb,
    }
}

fn self_test_limits_lsb(scale: Fs) -> (i16, i16) {
    // Table 4 specifies limits at ±2 g. Scale by the sensitivity ratio so the
    // equivalent milli-g window is preserved at other full-scale settings.
    // Integer math keeps this usable on no_std targets without libm.
    match scale {
        Fs::PlusMinus2G => (SELF_TEST_MIN_LSB_2G, SELF_TEST_MAX_LSB_2G),
        Fs::PlusMinus4G => (2, 45),
        Fs::PlusMinus8G => (1, 22),
        Fs::PlusMinus16G => (1, 7),
    }
}

/// Sensor operating mode.
///
/// The LIS2DE12 is an 8-bit-only device: the `LPen` bit in `CTRL_REG1` must always be set to '1'
/// per the datasheet ("must be set to '1' for the correct operation of the device"). There is no
/// 10-bit normal mode. Both variants behave identically; `Normal` is kept only for API
/// compatibility and is deprecated.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OperatingMode {
    /// Deprecated — LIS2DE12 does not support 10-bit mode. Use `LowPower` instead.
    #[deprecated(note = "LIS2DE12 is always 8-bit; LPen must always be 1. Use LowPower instead.")]
    Normal,
    /// 8-bit low-power mode (the only valid mode for LIS2DE12).
    LowPower,
}

impl OperatingMode {
    const fn lpen() -> bool {
        // LPen must always be '1' per datasheet Table 29.
        true
    }
}

/// Axis enable mask.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct AxesEnable {
    /// Enable X-axis measurements.
    pub x: bool,
    /// Enable Y-axis measurements.
    pub y: bool,
    /// Enable Z-axis measurements.
    pub z: bool,
}

impl AxesEnable {
    /// Returns `true` if at least one axis is enabled.
    #[must_use]
    pub const fn any(self) -> bool {
        self.x || self.y || self.z
    }
}

impl Default for AxesEnable {
    fn default() -> Self {
        Self {
            x: true,
            y: true,
            z: true,
        }
    }
}

/// High-level configuration applied during initialisation or runtime updates.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Lis2de12Config {
    /// Output data rate.
    pub odr: Odr,
    /// Operating mode (resolution/power trade-off).
    pub mode: OperatingMode,
    /// Full-scale range.
    pub scale: Fs,
    /// Axis enable mask.
    pub axes: AxesEnable,
    /// Enable block data updates.
    pub block_data_update: bool,
    /// Enable temperature sensor output.
    pub temperature_enable: bool,
    /// FIFO configuration.
    pub fifo: FifoConfig,
}

impl Default for Lis2de12Config {
    fn default() -> Self {
        Self {
            odr: Odr::HundredHz,
            mode: OperatingMode::LowPower,
            scale: Fs::PlusMinus2G,
            axes: AxesEnable::default(),
            block_data_update: true,
            temperature_enable: false,
            fifo: FifoConfig::disabled(),
        }
    }
}

impl Lis2de12Config {
    fn sensitivity_g_per_lsb(self) -> f32 {
        // Exact values from datasheet Table 4 (mg/digit → g/digit).
        // ±16g does not follow the simple linear pattern; it uses 187.5 mg/digit.
        match self.scale {
            Fs::PlusMinus2G => 0.0156,
            Fs::PlusMinus4G => 0.0312,
            Fs::PlusMinus8G => 0.0625,
            Fs::PlusMinus16G => 0.1875,
        }
    }

    fn sensitivity_mg_per_lsb(self) -> f32 {
        self.sensitivity_g_per_lsb() * 1000.0
    }
}

fn odr_to_hz(odr: Odr) -> Option<f32> {
    match odr {
        Odr::PowerDown => None,
        Odr::OneHz => Some(1.0),
        Odr::TenHz => Some(10.0),
        Odr::TwentyFiveHz => Some(25.0),
        Odr::FiftyHz => Some(50.0),
        Odr::HundredHz => Some(100.0),
        Odr::TwoHundredHz => Some(200.0),
        Odr::FourHundredHz => Some(400.0),
        Odr::SixteenTwentyHz => Some(1620.0),
        Odr::FiveThousandHz => Some(5376.0),
    }
}

/// Parsed FIFO status register snapshot.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[must_use]
pub struct FifoStatus {
    /// Watermark flag.
    pub watermark: bool,
    /// Overrun flag.
    pub overrun: bool,
    /// FIFO empty flag.
    pub empty: bool,
    /// Number of unread frames (saturated to hardware capacity).
    pub frames: u8,
}

impl FifoStatus {
    /// Create a status snapshot from the raw register value.
    pub fn from_raw(raw: field_sets::FifoSrcReg) -> Self {
        let raw_frames = raw.fss();
        let frames = if raw.empty() {
            0
        } else if raw.ovrn_fifo() && raw_frames == 0 {
            FIFO_CAPACITY
        } else {
            raw_frames.min(FIFO_CAPACITY)
        };

        Self {
            watermark: raw.wtm(),
            overrun: raw.ovrn_fifo(),
            empty: raw.empty(),
            frames,
        }
    }

    /// Number of unread frames currently buffered.
    #[must_use]
    pub const fn len(self) -> u8 {
        self.frames
    }

    /// Returns `true` if the FIFO currently holds data.
    #[must_use]
    pub const fn has_data(self) -> bool {
        self.frames > 0
    }

    /// Remaining capacity before the FIFO becomes full.
    #[must_use]
    pub const fn remaining_capacity(self) -> u8 {
        FIFO_CAPACITY.saturating_sub(self.frames)
    }

    /// Returns `true` if the configured watermark has been reached.
    #[must_use]
    pub const fn is_watermark_triggered(self) -> bool {
        self.watermark
    }

    /// Returns `true` if unread data has been overwritten.
    #[must_use]
    pub const fn is_overrun(self) -> bool {
        self.overrun
    }

    /// Returns `true` if the FIFO is empty.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.empty
    }
}

impl From<field_sets::FifoSrcReg> for FifoStatus {
    fn from(raw: field_sets::FifoSrcReg) -> Self {
        Self::from_raw(raw)
    }
}

fn validate_config_common<E: Debug>(config: &Lis2de12Config) -> Result<(), Error<E>> {
    if !config.axes.any() {
        return Err(Error::new(ErrorKind::Device));
    }
    if config.fifo.enable
        && let Some(watermark) = config.fifo.watermark
        && watermark > FIFO_WATERMARK_MAX
    {
        return Err(Error::new(ErrorKind::Param));
    }
    Ok(())
}

/// Blocking I²C interface wrapper.
pub struct DeviceInterface<I2C> {
    /// Underlying I²C bus.
    pub i2c: I2C,
    /// Slave address.
    pub address: u8,
}

/// Asynchronous I²C interface wrapper.
#[cfg(feature = "async")]
pub struct DeviceInterfaceAsync<I2C> {
    /// Underlying async I²C bus.
    pub i2c: I2C,
    /// Slave address.
    pub address: u8,
}

/// Blocking SPI interface wrapper.
pub struct SpiInterface<SPI> {
    /// Underlying SPI bus (with CS management).
    pub spi: SPI,
}

/// Asynchronous SPI interface wrapper.
#[cfg(feature = "async")]
pub struct SpiInterfaceAsync<SPI> {
    /// Underlying async SPI bus (with CS management).
    pub spi: SPI,
}

/// I²C sub-address MSb. When set, the LIS2DE12 auto-increments the register
/// pointer so a burst read/write walks consecutive addresses (datasheet §5.1.1).
const I2C_AUTO_INCREMENT: u8 = 0x80;

impl<I2C> BufferInterfaceError for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
}

#[cfg(feature = "async")]
impl<I2C> BufferInterfaceError for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
}

impl<I2C> RegisterInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    fn write_register(&mut self, address: Self::AddressType, _size_bits: u32, data: &[u8]) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data)
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncRegisterInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end]).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data).await
    }
}

impl<I2C> BufferInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type AddressType = u8;

    fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        self.i2c
            .write_read(self.address, &[address | I2C_AUTO_INCREMENT], buf)?;
        Ok(buf.len())
    }

    fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address | I2C_AUTO_INCREMENT;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end])?;
        Ok(buf.len())
    }

    fn flush(&mut self, _address: Self::AddressType) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncBufferInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type AddressType = u8;

    async fn read(&mut self, address: Self::AddressType, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.i2c
            .write_read(self.address, &[address | I2C_AUTO_INCREMENT], buf)
            .await?;
        Ok(buf.len())
    }

    async fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address | I2C_AUTO_INCREMENT;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end]).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<SPI> BufferInterfaceError for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type Error = SPI::Error;
}

#[cfg(feature = "async")]
impl<SPI> BufferInterfaceError for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;
}

impl<SPI> RegisterInterface for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type Error = SPI::Error;
    type AddressType = u8;

    fn write_register(&mut self, address: Self::AddressType, _size_bits: u32, data: &[u8]) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address; // Write: bit 7 = 0
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.spi.write(&buf[..end])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr_byte = 0x80 | address; // Read: bit 7 = 1
        self.spi.transaction(&mut [
            hal::spi::Operation::Write(&[addr_byte]),
            hal::spi::Operation::Read(data),
        ])
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncRegisterInterface for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address; // Write: bit 7 = 0
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.spi.write(&buf[..end]).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr_byte = 0x80 | address; // Read: bit 7 = 1
        self.spi
            .transaction(&mut [
                hal_async::spi::Operation::Write(&[addr_byte]),
                hal_async::spi::Operation::Read(data),
            ])
            .await
    }
}

impl<SPI> BufferInterface for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type AddressType = u8;

    fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        // Multi-byte read: bit 7 = 1 (read), bit 6 = 1 (auto-increment)
        let addr_byte = 0xC0 | address;
        self.spi
            .transaction(&mut [hal::spi::Operation::Write(&[addr_byte]), hal::spi::Operation::Read(buf)])?;
        Ok(buf.len())
    }

    fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = 0x40 | address; // Write with auto-increment: bit 6 = 1
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.spi.write(&data[..end])?;
        Ok(buf.len())
    }

    fn flush(&mut self, _address: Self::AddressType) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncBufferInterface for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type AddressType = u8;

    async fn read(&mut self, address: Self::AddressType, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Multi-byte read: bit 7 = 1 (read), bit 6 = 1 (auto-increment)
        let addr_byte = 0xC0 | address;
        self.spi
            .transaction(&mut [
                hal_async::spi::Operation::Write(&[addr_byte]),
                hal_async::spi::Operation::Read(buf),
            ])
            .await?;
        Ok(buf.len())
    }

    async fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = 0x40 | address; // Write with auto-increment: bit 6 = 1
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.spi.write(&data[..end]).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Available LIS2DE12 I²C slave addresses (controlled by the SA0 pin).
///
/// These are 7-bit addresses as required by `embedded-hal` I²C traits. The datasheet Table 15
/// lists 8-bit SAD+R/W bytes (0x30/0x32 write, 0x31/0x33 read); strip the R/W bit to get the
/// 7-bit address used here.
pub enum SlaveAddr {
    /// SA0 pulled low (7-bit address `0x18`).
    Default,
    /// SA0 pulled high (7-bit address `0x19`).
    Alternative,
}

impl SlaveAddr {
    const fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => 0x18,
            SlaveAddr::Alternative => 0x19,
        }
    }
}

/// Blocking LIS2DE12 driver.
pub struct Lis2de12<IFACE> {
    device: Lis2de12Device<IFACE>,
    config: Lis2de12Config,
}

/// Type alias for I2C-based blocking driver.
pub type Lis2de12I2c<I2C> = Lis2de12<DeviceInterface<I2C>>;

/// Type alias for SPI-based blocking driver.
pub type Lis2de12Spi<SPI> = Lis2de12<SpiInterface<SPI>>;

// Generic implementation for all interface types
impl<IFACE> Lis2de12<IFACE>
where
    IFACE: RegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error>
        + BufferInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    /// Return the active configuration.
    pub const fn config(&self) -> Lis2de12Config {
        self.config
    }

    /// Update the sensor configuration.
    pub fn set_config(&mut self, config: Lis2de12Config) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        validate_config_common::<<IFACE as RegisterInterface>::Error>(&config)?;
        self.apply_config(config)?;
        self.config = config;
        Ok(())
    }

    /// Read raw acceleration sample triplet.
    pub fn read_raw(&mut self) -> Result<I16x3, Error<<IFACE as RegisterInterface>::Error>> {
        let frame = self.read_fifo_frame()?;
        Ok(decode_raw(frame, self.config.mode))
    }

    /// Read acceleration expressed in milli-g.
    pub fn read_mg(&mut self) -> Result<I16x3, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_mg(raw, self.config.sensitivity_mg_per_lsb()))
    }

    /// Read acceleration expressed in g (floating point).
    pub fn read_g(&mut self) -> Result<F32x3, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_g(raw, self.config.sensitivity_g_per_lsb()))
    }

    /// Read a single FIFO frame (XYZ sample) using blocking I²C transfers.
    pub fn read_fifo_frame(&mut self) -> Result<FifoFrame, Error<<IFACE as RegisterInterface>::Error>> {
        let mut buf: FifoFrame = [0; FIFO_FRAME_BYTES];
        let mut fifo = self.device.fifo_read_start();
        let mut offset = 0;
        while offset < buf.len() {
            let read = fifo.read(&mut buf[offset..]).map_err(Error::from)?;
            if read == 0 {
                return Err(Error::new(ErrorKind::Device));
            }
            offset += read;
        }
        Ok(buf)
    }

    /// Read multiple FIFO frames into the provided slice, returning the number retrieved.
    /// Reading stops early if the FIFO does not currently hold enough data.
    pub fn read_fifo_frames(
        &mut self,
        frames: &mut [FifoFrame],
    ) -> Result<usize, Error<<IFACE as RegisterInterface>::Error>> {
        if frames.is_empty() {
            return Ok(0);
        }

        let status = self.fifo_status()?;
        if status.is_empty() {
            return Ok(0);
        }

        let to_read = cmp::min(status.len() as usize, frames.len());
        for slot in &mut frames[..to_read] {
            *slot = self.read_fifo_frame()?;
        }
        Ok(to_read)
    }

    /// Drain all available FIFO frames, discarding their contents.
    pub fn drain_fifo(&mut self) -> Result<usize, Error<<IFACE as RegisterInterface>::Error>> {
        let mut drained = 0;
        loop {
            let status = self.fifo_status()?;
            if status.is_empty() {
                break;
            }
            self.read_fifo_frame()?;
            drained += 1;
        }
        Ok(drained)
    }

    /// Read the FIFO source register and return the parsed status snapshot.
    pub fn fifo_status(&mut self) -> Result<FifoStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let status = self.device.fifo_src_reg().read().map_err(Error::from)?;
        Ok(FifoStatus::from(status))
    }

    /// Enable or disable the on-die temperature sensor.
    pub fn set_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_temperature_sensor(enable)?;
        self.config.temperature_enable = enable;
        Ok(())
    }

    /// Read raw temperature sensor data as a 16-bit signed integer.
    /// The temperature sensor must be enabled via `set_temperature_sensor` or configuration.
    /// Returns the raw left-justified two's complement value.
    pub fn read_temperature_raw(&mut self) -> Result<i16, Error<<IFACE as RegisterInterface>::Error>> {
        let low = self.device.out_temp_l().read().map_err(Error::from)?.temp_l();
        let high = self.device.out_temp_h().read().map_err(Error::from)?.temp_h();
        Ok(i16::from_le_bytes([low, high]))
    }

    /// Read temperature change in degrees Celsius.
    /// The temperature sensor must be enabled via `set_temperature_sensor` or configuration.
    ///
    /// Returns the temperature **delta** relative to the device's internal reference point
    /// (established at power-on). The sensitivity is 1 digit/°C with 8-bit resolution
    /// (left-justified in a 16-bit register). Positive values indicate temperature increase,
    /// negative values indicate decrease.
    ///
    /// # Relative vs. absolute temperature
    ///
    /// The LIS2DE12 temperature sensor measures **change**, not absolute temperature. ST's
    /// reference C driver (`lis2de12_from_lsb_to_celsius`) adds a fixed 25 °C offset to the
    /// raw value, implicitly assuming the device powers on at room temperature. That assumption
    /// is unreliable in practice: if the device boots at a different ambient temperature the
    /// offset will be wrong by exactly that difference.
    ///
    /// This driver returns the raw delta so callers can apply their own reference. To approximate
    /// absolute temperature using the ST convention:
    ///
    /// ```ignore
    /// let abs_celsius = sensor.read_temperature()? + 25.0;
    /// ```
    ///
    /// For accurate absolute readings, record a known ambient temperature at boot and add it
    /// instead of the 25 °C constant.
    pub fn read_temperature(&mut self) -> Result<f32, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.read_temperature_raw()?;
        // Data is 8-bit resolution, left-justified in 16-bit register
        // Sensitivity: 1 digit/°C (see datasheet Table 5)
        // Convert from left-justified 16-bit to signed 8-bit value
        Ok(f32::from(raw >> 8))
    }

    /// Route HP-filtered data to the output registers and FIFO (FDS bit in `CTRL_REG2`).
    /// When enabled, accelerometer output values reflect the high-pass filtered signal
    /// rather than the raw signal.
    pub fn set_high_pass_to_outputs(&mut self, enable: bool) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_2()
            .modify(|reg: &mut field_sets::CtrlReg2| reg.set_fds(enable))
            .map_err(Error::from)
    }

    /// Return `true` if HP-filtered data is currently routed to the output registers.
    pub fn high_pass_to_outputs(&mut self) -> Result<bool, Error<<IFACE as RegisterInterface>::Error>> {
        Ok(self.device.ctrl_reg_2().read().map_err(Error::from)?.fds())
    }

    /// Connect (`true`) or disconnect (`false`) the internal SDO/SA0 pull-up resistor.
    /// The pull-up is connected by default. Disconnect it to reduce power draw or to
    /// avoid bus conflicts when the pin is driven externally.
    pub fn set_sdo_pullup_connected(
        &mut self,
        connected: bool,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_0()
            .modify(|reg: &mut field_sets::CtrlReg0| reg.set_sdo_pu_disc(!connected))
            .map_err(Error::from)
    }

    /// Return `true` if the internal SDO/SA0 pull-up resistor is connected.
    pub fn sdo_pullup_connected(&mut self) -> Result<bool, Error<<IFACE as RegisterInterface>::Error>> {
        Ok(!self.device.ctrl_reg_0().read().map_err(Error::from)?.sdo_pu_disc())
    }

    /// Issue a reboot command to reload memory content.
    pub fn reboot(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_5()
            .write(|reg: &mut field_sets::CtrlReg5| reg.set_boot(true))
            .map_err(Error::from)
    }

    // ========================================================================
    // Self-test
    // ========================================================================

    /// Enable or disable the electrostatic self-test (`ST` bits in `CTRL_REG4`).
    ///
    /// After enabling, discard the first two samples before using the reading for
    /// evaluation (datasheet Table 4 note 6). Compare against a no-self-test sample
    /// with [`evaluate_self_test`].
    pub fn set_self_test(&mut self, mode: St) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_4()
            .modify(|reg: &mut field_sets::CtrlReg4| reg.set_st(mode))
            .map_err(Error::from)
    }

    /// Read the current self-test mode.
    pub fn self_test(&mut self) -> Result<St, Error<<IFACE as RegisterInterface>::Error>> {
        Ok(self.device.ctrl_reg_4().read().map_err(Error::from)?.st())
    }

    // ========================================================================
    // High-pass filter / reference
    // ========================================================================

    /// Configure high-pass filter mode, cutoff, and filtered-data selection.
    ///
    /// Preserves the per-function HP enables (`HPCLICK` / `HP_IA1` / `HP_IA2`) already
    /// programmed via motion/click configuration.
    pub fn set_high_pass_config(
        &mut self,
        config: HighPassConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_2()
            .modify(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hpm(config.mode);
                reg.set_hpcf(config.cutoff);
                reg.set_fds(config.filtered_data_selection);
            })
            .map_err(Error::from)
    }

    /// Read the current high-pass filter configuration.
    pub fn high_pass_config(&mut self) -> Result<HighPassConfig, Error<<IFACE as RegisterInterface>::Error>> {
        let reg = self.device.ctrl_reg_2().read().map_err(Error::from)?;
        Ok(HighPassConfig {
            mode: reg.hpm(),
            cutoff: reg.hpcf(),
            filtered_data_selection: reg.fds(),
        })
    }

    /// Write the high-pass filter reference value (`REFERENCE` register).
    pub fn set_reference(&mut self, value: u8) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .reference()
            .write(|reg: &mut field_sets::Reference| reg.set_reference(value))
            .map_err(Error::from)
    }

    /// Read the high-pass filter reference value.
    pub fn reference(&mut self) -> Result<u8, Error<<IFACE as RegisterInterface>::Error>> {
        Ok(self.device.reference().read().map_err(Error::from)?.reference())
    }

    /// Reset the high-pass filter by reading the `REFERENCE` register.
    ///
    /// Effective when the filter mode is [`Hpm::NormalWithReset`].
    pub fn reset_high_pass_filter(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        let _ = self.device.reference().read().map_err(Error::from)?;
        Ok(())
    }

    // ========================================================================
    // Data / temperature status
    // ========================================================================

    /// Read acceleration data-ready and overrun flags from `STATUS_REG`.
    pub fn data_status(&mut self) -> Result<DataStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.device.status_reg().read().map_err(Error::from)?;
        Ok(DataStatus::from(raw))
    }

    /// Returns `true` when a new XYZ sample is available (`ZYXDA`).
    pub fn data_ready(&mut self) -> Result<bool, Error<<IFACE as RegisterInterface>::Error>> {
        Ok(self.data_status()?.data_ready())
    }

    /// Read temperature data-ready and overrun flags from `STATUS_REG_AUX`.
    pub fn temperature_status(&mut self) -> Result<TemperatureStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.device.status_reg_aux().read().map_err(Error::from)?;
        Ok(TemperatureStatus::from(raw))
    }

    // ========================================================================
    // FIFO helpers
    // ========================================================================

    /// Reset the FIFO by briefly switching to Bypass mode, then restoring the
    /// configured FIFO mode/watermark/trigger from [`Lis2de12Config`].
    ///
    /// Required after FIFO mode fills and stops collecting (datasheet §5.1.2).
    pub fn reset_fifo(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .fifo_ctrl_reg()
            .modify(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(Fm::Bypass);
            })
            .map_err(Error::from)?;

        let fifo = self.config.fifo;
        self.device
            .fifo_ctrl_reg()
            .write(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(fifo.mode.into());
                reg.set_tr(fifo.trigger_bit());
                reg.set_fth(fifo.effective_threshold());
            })
            .map_err(Error::from)
    }

    // ========================================================================
    // Motion Detection
    // ========================================================================

    /// Configure motion detection on interrupt generator 1 (IA1).
    pub fn set_motion1_config(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_motion_config_int1(config)
    }

    /// Configure motion detection on interrupt generator 2 (IA2).
    pub fn set_motion2_config(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_motion_config_int2(config)
    }

    /// Read motion detection status from interrupt generator 1 (IA1).
    pub fn motion1_status(&mut self) -> Result<MotionStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.device.int_1_src().read().map_err(Error::from)?;
        Ok(MotionStatus::from(raw))
    }

    /// Read motion detection status from interrupt generator 2 (IA2).
    pub fn motion2_status(&mut self) -> Result<MotionStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.device.int_2_src().read().map_err(Error::from)?;
        Ok(MotionStatus::from(raw))
    }

    // ========================================================================
    // Click Detection
    // ========================================================================

    /// Configure click detection.
    pub fn set_click_config(&mut self, config: ClickConfig) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_click_config(config)
    }

    /// Read click detection status.
    pub fn click_status(&mut self) -> Result<ClickStatus, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.device.click_src().read().map_err(Error::from)?;
        Ok(ClickStatus::from(raw))
    }

    // ========================================================================
    // Activity Detection
    // ========================================================================

    /// Configure activity/inactivity detection (sleep-to-wake).
    pub fn set_activity_config(
        &mut self,
        config: ActivityConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_activity_config(config)
    }

    // ========================================================================
    // Interrupt Routing
    // ========================================================================

    /// Configure interrupt pin routing and polarity.
    pub fn set_interrupt_config(
        &mut self,
        config: InterruptConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.write_interrupt_config(config)
    }

    /// Set interrupt pin polarity.
    pub fn set_interrupt_polarity(
        &mut self,
        polarity: InterruptPolarity,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_6()
            .modify(|reg: &mut field_sets::CtrlReg6| {
                reg.set_int_polarity(matches!(polarity, InterruptPolarity::ActiveLow));
            })
            .map_err(Error::from)
    }

    /// Read combined interrupt sources status.
    ///
    /// This reads all interrupt source registers in sequence.
    /// For latched interrupts, reading clears the interrupt flag.
    pub fn interrupt_sources(&mut self) -> Result<InterruptSources, Error<<IFACE as RegisterInterface>::Error>> {
        let motion1 = self.motion1_status()?;
        let motion2 = self.motion2_status()?;
        let click = self.click_status()?;
        Ok(InterruptSources {
            motion1,
            motion2,
            click,
        })
    }

    fn write_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .temp_cfg_reg()
            .write(|reg: &mut field_sets::TempCfgReg| {
                reg.set_temp_en(if enable { TempEn::TempEn } else { TempEn::TempDis });
            })
            .map_err(Error::from)
    }

    fn apply_config(&mut self, config: Lis2de12Config) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .ctrl_reg_1()
            .write(|reg: &mut field_sets::CtrlReg1| {
                reg.set_odr(config.odr);
                reg.set_lpen(OperatingMode::lpen());
                reg.set_xen(config.axes.x);
                reg.set_yen(config.axes.y);
                reg.set_zen(config.axes.z);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_4()
            .write(|reg: &mut field_sets::CtrlReg4| {
                reg.set_bdu(config.block_data_update);
                reg.set_fs(config.scale);
                reg.set_st(St::Normal);
                reg.set_sim(false);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .write(|reg: &mut field_sets::CtrlReg5| {
                reg.set_fifo_en(config.fifo.enable);
            })
            .map_err(Error::from)?;

        self.device
            .fifo_ctrl_reg()
            .write(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(config.fifo.mode.into());
                reg.set_tr(config.fifo.trigger_bit());
                reg.set_fth(config.fifo.effective_threshold());
            })
            .map_err(Error::from)?;

        self.write_temperature_sensor(config.temperature_enable)?;
        Ok(())
    }

    fn write_motion_config_int1(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        let (aoi, sixd, d4d) = mode_bits(config.mode);

        self.device
            .int_1_cfg()
            .write(|reg: &mut field_sets::Int1Cfg| {
                reg.set_aoi(aoi);
                reg.set_sixd(sixd);
                reg.set_xhie(config.axes.x_high && config.enable);
                reg.set_xlie(config.axes.x_low && config.enable);
                reg.set_yhie(config.axes.y_high && config.enable);
                reg.set_ylie(config.axes.y_low && config.enable);
                reg.set_zhie(config.axes.z_high && config.enable);
                reg.set_zlie(config.axes.z_low && config.enable);
            })
            .map_err(Error::from)?;

        self.device
            .int_1_ths()
            .write(|reg: &mut field_sets::Int1Ths| {
                reg.set_ths(config.threshold.min(127));
            })
            .map_err(Error::from)?;

        self.device
            .int_1_duration()
            .write(|reg: &mut field_sets::Int1Duration| {
                reg.set_d(config.duration.min(127));
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .modify(|reg: &mut field_sets::CtrlReg5| {
                reg.set_lir_int_1(matches!(config.latch, LatchMode::Latched));
                reg.set_d_4_d_int_1(d4d);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_2()
            .modify(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hp_ia_1(config.high_pass_filter);
            })
            .map_err(Error::from)?;

        Ok(())
    }

    fn write_motion_config_int2(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        let (aoi, sixd, d4d) = mode_bits(config.mode);

        self.device
            .int_2_cfg()
            .write(|reg: &mut field_sets::Int2Cfg| {
                reg.set_aoi(aoi);
                reg.set_sixd(sixd);
                reg.set_xhie(config.axes.x_high && config.enable);
                reg.set_xlie(config.axes.x_low && config.enable);
                reg.set_yhie(config.axes.y_high && config.enable);
                reg.set_ylie(config.axes.y_low && config.enable);
                reg.set_zhie(config.axes.z_high && config.enable);
                reg.set_zlie(config.axes.z_low && config.enable);
            })
            .map_err(Error::from)?;

        self.device
            .int_2_ths()
            .write(|reg: &mut field_sets::Int2Ths| {
                reg.set_ths(config.threshold.min(127));
            })
            .map_err(Error::from)?;

        self.device
            .int_2_duration()
            .write(|reg: &mut field_sets::Int2Duration| {
                reg.set_d(config.duration.min(127));
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .modify(|reg: &mut field_sets::CtrlReg5| {
                reg.set_lir_int_2(matches!(config.latch, LatchMode::Latched));
                reg.set_d_4_d_int_2(d4d);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_2()
            .modify(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hp_ia_2(config.high_pass_filter);
            })
            .map_err(Error::from)?;

        Ok(())
    }

    fn write_click_config(&mut self, config: ClickConfig) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        // Configure CLICK_CFG
        self.device
            .click_cfg()
            .write(|reg: &mut field_sets::ClickCfg| {
                reg.set_xs(config.axes.x_single && config.enable);
                reg.set_xd(config.axes.x_double && config.enable);
                reg.set_ys(config.axes.y_single && config.enable);
                reg.set_yd(config.axes.y_double && config.enable);
                reg.set_zs(config.axes.z_single && config.enable);
                reg.set_zd(config.axes.z_double && config.enable);
            })
            .map_err(Error::from)?;

        // Configure threshold and latch
        self.device
            .click_ths()
            .write(|reg: &mut field_sets::ClickThs| {
                reg.set_ths(config.threshold.min(127));
                reg.set_lir_click(matches!(config.latch, LatchMode::Latched));
            })
            .map_err(Error::from)?;

        // Configure time limit
        self.device
            .time_limit()
            .write(|reg: &mut field_sets::TimeLimit| {
                reg.set_tli(config.time_limit.min(127));
            })
            .map_err(Error::from)?;

        // Configure time latency
        self.device
            .time_latency()
            .write(|reg: &mut field_sets::TimeLatency| {
                reg.set_tla(config.time_latency);
            })
            .map_err(Error::from)?;

        // Configure time window
        self.device
            .time_window()
            .write(|reg: &mut field_sets::TimeWindow| {
                reg.set_tw(config.time_window);
            })
            .map_err(Error::from)?;

        // Configure high-pass filter in CTRL_REG2
        self.device
            .ctrl_reg_2()
            .modify(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hpclick(config.high_pass_filter);
            })
            .map_err(Error::from)?;

        Ok(())
    }

    fn write_activity_config(
        &mut self,
        config: ActivityConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        // Sleep-to-wake is armed by a non-zero ACT_THS; disable clears both registers.
        let (threshold, duration) = if config.enable {
            (config.threshold.min(127), config.duration)
        } else {
            (0, 0)
        };

        self.device
            .act_ths()
            .write(|reg: &mut field_sets::ActThs| {
                reg.set_acth(threshold);
            })
            .map_err(Error::from)?;

        self.device
            .act_dur()
            .write(|reg: &mut field_sets::ActDur| {
                reg.set_act_d(duration);
            })
            .map_err(Error::from)?;

        Ok(())
    }

    fn write_interrupt_config(
        &mut self,
        config: InterruptConfig,
    ) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        // Configure INT1 routing in CTRL_REG3
        self.device
            .ctrl_reg_3()
            .write(|reg: &mut field_sets::CtrlReg3| {
                reg.set_i_1_click(config.int1.click);
                reg.set_i_1_ia_1(config.int1.ia1);
                reg.set_i_1_ia_2(config.int1.ia2);
                reg.set_i_1_zyxda(config.int1.data_ready);
                reg.set_i_1_wtm(config.int1.fifo_watermark);
                reg.set_i_1_overrun(config.int1.fifo_overrun);
            })
            .map_err(Error::from)?;

        // Configure INT2 routing and polarity in CTRL_REG6
        self.device
            .ctrl_reg_6()
            .write(|reg: &mut field_sets::CtrlReg6| {
                reg.set_i_2_click(config.int2.click);
                reg.set_i_2_ia_1(config.int2.ia1);
                reg.set_i_2_ia_2(config.int2.ia2);
                reg.set_i_2_boot(config.int2.boot);
                reg.set_i_2_act(config.int2.activity);
                reg.set_int_polarity(matches!(config.polarity, InterruptPolarity::ActiveLow));
            })
            .map_err(Error::from)?;

        Ok(())
    }
}

// I2C-specific constructors and methods
impl<I2C> Lis2de12<DeviceInterface<I2C>>
where
    I2C: I2c,
    I2C::Error: Debug,
{
    /// Create a new I2C driver with the default configuration.
    pub fn new_i2c(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, addr, Lis2de12Config::default())
    }

    /// Create a new I2C driver with an explicit configuration.
    pub fn new_i2c_with_config(i2c: I2C, addr: SlaveAddr, config: Lis2de12Config) -> Result<Self, Error<I2C::Error>> {
        validate_config_common::<I2C::Error>(&config)?;

        let interface = DeviceInterface {
            i2c,
            address: addr.addr(),
        };
        let mut device = Lis2de12Device::new(interface);
        verify_device_id(&mut device)?;
        let mut this = Self { device, config };
        this.apply_config(config)?;
        Ok(this)
    }

    /// Create a new driver with the default configuration (backward compatible, uses I2C).
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c(i2c, addr)
    }

    /// Create a new driver with an explicit configuration (backward compatible, uses I2C).
    pub fn new_with_config(i2c: I2C, addr: SlaveAddr, config: Lis2de12Config) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, addr, config)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<DeviceInterface<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying I²C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }
}

// SPI-specific constructors and methods
impl<SPI> Lis2de12<SpiInterface<SPI>>
where
    SPI: hal::spi::SpiDevice,
    SPI::Error: Debug,
{
    /// Create a new SPI driver with the default configuration.
    pub fn new_spi(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        Self::new_spi_with_config(spi, Lis2de12Config::default())
    }

    /// Create a new SPI driver with an explicit configuration.
    pub fn new_spi_with_config(spi: SPI, config: Lis2de12Config) -> Result<Self, Error<SPI::Error>> {
        validate_config_common::<SPI::Error>(&config)?;

        let interface = SpiInterface { spi };
        let mut device = Lis2de12Device::new(interface);
        verify_device_id(&mut device)?;
        let mut this = Self { device, config };
        this.apply_config(config)?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<SpiInterface<SPI>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying SPI bus.
    pub fn destroy(self) -> SPI {
        self.device.interface.spi
    }
}

impl<IFACE> RawAccelerometer<I16x3> for Lis2de12<IFACE>
where
    IFACE: RegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error>
        + BufferInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    type Error = <IFACE as RegisterInterface>::Error;

    fn accel_raw(&mut self) -> Result<I16x3, Error<Self::Error>> {
        self.read_raw()
    }
}

impl<IFACE> Accelerometer for Lis2de12<IFACE>
where
    IFACE: RegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error>
        + BufferInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    type Error = <IFACE as RegisterInterface>::Error;

    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        self.read_g()
    }

    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        odr_to_hz(self.config.odr).ok_or_else(|| Error::new(ErrorKind::Mode))
    }
}

/// Asynchronous LIS2DE12 driver.
#[cfg(feature = "async")]
pub struct Lis2de12Async<IFACE> {
    device: Lis2de12Device<IFACE>,
    config: Lis2de12Config,
}

/// Type alias for I2C-based async driver.
#[cfg(feature = "async")]
pub type Lis2de12I2cAsync<I2C> = Lis2de12Async<DeviceInterfaceAsync<I2C>>;

/// Type alias for SPI-based async driver.
#[cfg(feature = "async")]
pub type Lis2de12SpiAsync<SPI> = Lis2de12Async<SpiInterfaceAsync<SPI>>;

#[cfg(feature = "async")]
// Generic implementation for all async interface types
impl<IFACE> Lis2de12Async<IFACE>
where
    IFACE: AsyncRegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error>
        + AsyncBufferInterface<AddressType = u8>,
    <IFACE as AsyncRegisterInterface>::Error: Debug,
{
    /// Return the active configuration.
    pub const fn config(&self) -> Lis2de12Config {
        self.config
    }

    /// Update the sensor configuration asynchronously.
    pub async fn set_config(
        &mut self,
        config: Lis2de12Config,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        validate_config_common::<<IFACE as AsyncRegisterInterface>::Error>(&config)?;
        self.apply_config(config).await?;
        self.config = config;
        Ok(())
    }

    /// Read raw acceleration sample triplet asynchronously.
    pub async fn read_raw(&mut self) -> Result<I16x3, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let frame = self.read_fifo_frame().await?;
        Ok(decode_raw(frame, self.config.mode))
    }

    /// Read acceleration expressed in milli-g asynchronously.
    pub async fn read_mg(&mut self) -> Result<I16x3, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_mg(raw, self.config.sensitivity_mg_per_lsb()))
    }

    /// Read acceleration expressed in g asynchronously.
    pub async fn read_g(&mut self) -> Result<F32x3, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_g(raw, self.config.sensitivity_g_per_lsb()))
    }

    /// Read a single FIFO frame asynchronously.
    pub async fn read_fifo_frame(&mut self) -> Result<FifoFrame, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let mut buf: FifoFrame = [0; FIFO_FRAME_BYTES];
        let mut fifo = self.device.fifo_read_start();
        let mut offset = 0;
        while offset < buf.len() {
            let read = fifo.read_async(&mut buf[offset..]).await.map_err(Error::from)?;
            if read == 0 {
                return Err(Error::new(ErrorKind::Device));
            }
            offset += read;
        }
        Ok(buf)
    }

    /// Read multiple FIFO frames asynchronously, returning the number retrieved.
    pub async fn read_fifo_frames(
        &mut self,
        frames: &mut [FifoFrame],
    ) -> Result<usize, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        if frames.is_empty() {
            return Ok(0);
        }

        let status = self.fifo_status().await?;
        if status.is_empty() {
            return Ok(0);
        }

        let to_read = cmp::min(status.len() as usize, frames.len());
        for slot in &mut frames[..to_read] {
            *slot = self.read_fifo_frame().await?;
        }
        Ok(to_read)
    }

    /// Drain all available FIFO frames asynchronously, discarding their contents.
    pub async fn drain_fifo(&mut self) -> Result<usize, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let mut drained = 0;
        loop {
            let status = self.fifo_status().await?;
            if status.is_empty() {
                break;
            }
            self.read_fifo_frame().await?;
            drained += 1;
        }
        Ok(drained)
    }

    /// Read the FIFO source register asynchronously and return the parsed status snapshot.
    pub async fn fifo_status(&mut self) -> Result<FifoStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let status = self.device.fifo_src_reg().read_async().await.map_err(Error::from)?;
        Ok(FifoStatus::from(status))
    }

    /// Enable or disable the on-die temperature sensor asynchronously.
    pub async fn set_temperature_sensor(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_temperature_sensor(enable).await?;
        self.config.temperature_enable = enable;
        Ok(())
    }

    /// Read raw temperature sensor data as a 16-bit signed integer asynchronously.
    /// The temperature sensor must be enabled via `set_temperature_sensor` or configuration.
    /// Returns the raw left-justified two's complement value.
    pub async fn read_temperature_raw(&mut self) -> Result<i16, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let low = self
            .device
            .out_temp_l()
            .read_async()
            .await
            .map_err(Error::from)?
            .temp_l() as u8;
        let high = self
            .device
            .out_temp_h()
            .read_async()
            .await
            .map_err(Error::from)?
            .temp_h() as u8;
        Ok(i16::from_le_bytes([low, high]))
    }

    /// Read temperature change in degrees Celsius asynchronously.
    /// The temperature sensor must be enabled via `set_temperature_sensor` or configuration.
    ///
    /// Returns the temperature **delta** relative to the device's internal reference point
    /// (established at power-on). The sensitivity is 1 digit/°C with 8-bit resolution
    /// (left-justified in a 16-bit register). Positive values indicate temperature increase,
    /// negative values indicate decrease.
    ///
    /// # Relative vs. absolute temperature
    ///
    /// The LIS2DE12 temperature sensor measures **change**, not absolute temperature. ST's
    /// reference C driver (`lis2de12_from_lsb_to_celsius`) adds a fixed 25 °C offset to the
    /// raw value, implicitly assuming the device powers on at room temperature. That assumption
    /// is unreliable in practice: if the device boots at a different ambient temperature the
    /// offset will be wrong by exactly that difference.
    ///
    /// This driver returns the raw delta so callers can apply their own reference. To approximate
    /// absolute temperature using the ST convention:
    ///
    /// ```ignore
    /// let abs_celsius = sensor.read_temperature().await? + 25.0;
    /// ```
    ///
    /// For accurate absolute readings, record a known ambient temperature at boot and add it
    /// instead of the 25 °C constant.
    pub async fn read_temperature(&mut self) -> Result<f32, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.read_temperature_raw().await?;
        // Data is 8-bit resolution, left-justified in 16-bit register
        // Sensitivity: 1 digit/°C (see datasheet Table 5)
        // Convert from left-justified 16-bit to signed 8-bit value
        Ok(f32::from(raw >> 8))
    }

    /// Route HP-filtered data to the output registers and FIFO asynchronously.
    pub async fn set_high_pass_to_outputs(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_2()
            .modify_async(|reg: &mut field_sets::CtrlReg2| reg.set_fds(enable))
            .await
            .map_err(Error::from)
    }

    /// Return `true` if HP-filtered data is currently routed to the output registers.
    pub async fn high_pass_to_outputs(&mut self) -> Result<bool, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        Ok(self.device.ctrl_reg_2().read_async().await.map_err(Error::from)?.fds())
    }

    /// Connect (`true`) or disconnect (`false`) the internal SDO/SA0 pull-up resistor asynchronously.
    pub async fn set_sdo_pullup_connected(
        &mut self,
        connected: bool,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_0()
            .modify_async(|reg: &mut field_sets::CtrlReg0| reg.set_sdo_pu_disc(!connected))
            .await
            .map_err(Error::from)
    }

    /// Return `true` if the internal SDO/SA0 pull-up resistor is connected.
    pub async fn sdo_pullup_connected(&mut self) -> Result<bool, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        Ok(!self
            .device
            .ctrl_reg_0()
            .read_async()
            .await
            .map_err(Error::from)?
            .sdo_pu_disc())
    }

    /// Issue a reboot command asynchronously.
    pub async fn reboot(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_5()
            .write_async(|reg: &mut field_sets::CtrlReg5| reg.set_boot(true))
            .await
            .map_err(Error::from)
    }

    // ========================================================================
    // Self-test (Async)
    // ========================================================================

    /// Enable or disable the electrostatic self-test asynchronously.
    pub async fn set_self_test(&mut self, mode: St) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_4()
            .modify_async(|reg: &mut field_sets::CtrlReg4| reg.set_st(mode))
            .await
            .map_err(Error::from)
    }

    /// Read the current self-test mode asynchronously.
    pub async fn self_test(&mut self) -> Result<St, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        Ok(self.device.ctrl_reg_4().read_async().await.map_err(Error::from)?.st())
    }

    // ========================================================================
    // High-pass filter / reference (Async)
    // ========================================================================

    /// Configure high-pass filter mode, cutoff, and filtered-data selection asynchronously.
    pub async fn set_high_pass_config(
        &mut self,
        config: HighPassConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_2()
            .modify_async(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hpm(config.mode);
                reg.set_hpcf(config.cutoff);
                reg.set_fds(config.filtered_data_selection);
            })
            .await
            .map_err(Error::from)
    }

    /// Read the current high-pass filter configuration asynchronously.
    pub async fn high_pass_config(
        &mut self,
    ) -> Result<HighPassConfig, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let reg = self.device.ctrl_reg_2().read_async().await.map_err(Error::from)?;
        Ok(HighPassConfig {
            mode: reg.hpm(),
            cutoff: reg.hpcf(),
            filtered_data_selection: reg.fds(),
        })
    }

    /// Write the high-pass filter reference value asynchronously.
    pub async fn set_reference(&mut self, value: u8) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .reference()
            .write_async(|reg: &mut field_sets::Reference| reg.set_reference(value))
            .await
            .map_err(Error::from)
    }

    /// Read the high-pass filter reference value asynchronously.
    pub async fn reference(&mut self) -> Result<u8, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        Ok(self
            .device
            .reference()
            .read_async()
            .await
            .map_err(Error::from)?
            .reference())
    }

    /// Reset the high-pass filter by reading the `REFERENCE` register asynchronously.
    pub async fn reset_high_pass_filter(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let _ = self.device.reference().read_async().await.map_err(Error::from)?;
        Ok(())
    }

    // ========================================================================
    // Data / temperature status (Async)
    // ========================================================================

    /// Read acceleration data-ready and overrun flags asynchronously.
    pub async fn data_status(&mut self) -> Result<DataStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.device.status_reg().read_async().await.map_err(Error::from)?;
        Ok(DataStatus::from(raw))
    }

    /// Returns `true` when a new XYZ sample is available asynchronously.
    pub async fn data_ready(&mut self) -> Result<bool, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        Ok(self.data_status().await?.data_ready())
    }

    /// Read temperature data-ready and overrun flags asynchronously.
    pub async fn temperature_status(
        &mut self,
    ) -> Result<TemperatureStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.device.status_reg_aux().read_async().await.map_err(Error::from)?;
        Ok(TemperatureStatus::from(raw))
    }

    // ========================================================================
    // FIFO helpers (Async)
    // ========================================================================

    /// Reset the FIFO asynchronously by switching through Bypass mode.
    pub async fn reset_fifo(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .fifo_ctrl_reg()
            .modify_async(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(Fm::Bypass);
            })
            .await
            .map_err(Error::from)?;

        let fifo = self.config.fifo;
        self.device
            .fifo_ctrl_reg()
            .write_async(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(fifo.mode.into());
                reg.set_tr(fifo.trigger_bit());
                reg.set_fth(fifo.effective_threshold());
            })
            .await
            .map_err(Error::from)
    }

    // ========================================================================
    // Motion Detection (Async)
    // ========================================================================

    /// Configure motion detection on interrupt generator 1 (IA1) asynchronously.
    pub async fn set_motion1_config(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_motion_config_int1(config).await
    }

    /// Configure motion detection on interrupt generator 2 (IA2) asynchronously.
    pub async fn set_motion2_config(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_motion_config_int2(config).await
    }

    /// Read motion detection status from interrupt generator 1 (IA1) asynchronously.
    pub async fn motion1_status(&mut self) -> Result<MotionStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.device.int_1_src().read_async().await.map_err(Error::from)?;
        Ok(MotionStatus::from(raw))
    }

    /// Read motion detection status from interrupt generator 2 (IA2) asynchronously.
    pub async fn motion2_status(&mut self) -> Result<MotionStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.device.int_2_src().read_async().await.map_err(Error::from)?;
        Ok(MotionStatus::from(raw))
    }

    // ========================================================================
    // Click Detection (Async)
    // ========================================================================

    /// Configure click detection asynchronously.
    pub async fn set_click_config(
        &mut self,
        config: ClickConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_click_config(config).await
    }

    /// Read click detection status asynchronously.
    pub async fn click_status(&mut self) -> Result<ClickStatus, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.device.click_src().read_async().await.map_err(Error::from)?;
        Ok(ClickStatus::from(raw))
    }

    // ========================================================================
    // Activity Detection (Async)
    // ========================================================================

    /// Configure activity/inactivity detection (sleep-to-wake) asynchronously.
    pub async fn set_activity_config(
        &mut self,
        config: ActivityConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_activity_config(config).await
    }

    // ========================================================================
    // Interrupt Routing (Async)
    // ========================================================================

    /// Configure interrupt pin routing and polarity asynchronously.
    pub async fn set_interrupt_config(
        &mut self,
        config: InterruptConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.write_interrupt_config(config).await
    }

    /// Set interrupt pin polarity asynchronously.
    pub async fn set_interrupt_polarity(
        &mut self,
        polarity: InterruptPolarity,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_6()
            .modify_async(|reg: &mut field_sets::CtrlReg6| {
                reg.set_int_polarity(matches!(polarity, InterruptPolarity::ActiveLow));
            })
            .await
            .map_err(Error::from)
    }

    /// Read combined interrupt sources status asynchronously.
    ///
    /// This reads all interrupt source registers in sequence.
    /// For latched interrupts, reading clears the interrupt flag.
    pub async fn interrupt_sources(
        &mut self,
    ) -> Result<InterruptSources, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let motion1 = self.motion1_status().await?;
        let motion2 = self.motion2_status().await?;
        let click = self.click_status().await?;
        Ok(InterruptSources {
            motion1,
            motion2,
            click,
        })
    }

    async fn write_temperature_sensor(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .temp_cfg_reg()
            .write_async(|reg: &mut field_sets::TempCfgReg| {
                reg.set_temp_en(if enable { TempEn::TempEn } else { TempEn::TempDis });
            })
            .await
            .map_err(Error::from)
    }

    async fn apply_config(
        &mut self,
        config: Lis2de12Config,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .ctrl_reg_1()
            .write_async(|reg: &mut field_sets::CtrlReg1| {
                reg.set_odr(config.odr);
                reg.set_lpen(OperatingMode::lpen());
                reg.set_xen(config.axes.x);
                reg.set_yen(config.axes.y);
                reg.set_zen(config.axes.z);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_4()
            .write_async(|reg: &mut field_sets::CtrlReg4| {
                reg.set_bdu(config.block_data_update);
                reg.set_fs(config.scale);
                reg.set_st(St::Normal);
                reg.set_sim(false);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .write_async(|reg: &mut field_sets::CtrlReg5| {
                reg.set_fifo_en(config.fifo.enable);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .fifo_ctrl_reg()
            .write_async(|reg: &mut field_sets::FifoCtrlReg| {
                reg.set_fm(config.fifo.mode.into());
                reg.set_tr(config.fifo.trigger_bit());
                reg.set_fth(config.fifo.effective_threshold());
            })
            .await
            .map_err(Error::from)?;

        self.write_temperature_sensor(config.temperature_enable).await?;
        Ok(())
    }

    async fn write_motion_config_int1(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let (aoi, sixd, d4d) = mode_bits(config.mode);

        self.device
            .int_1_cfg()
            .write_async(|reg: &mut field_sets::Int1Cfg| {
                reg.set_aoi(aoi);
                reg.set_sixd(sixd);
                reg.set_xhie(config.axes.x_high && config.enable);
                reg.set_xlie(config.axes.x_low && config.enable);
                reg.set_yhie(config.axes.y_high && config.enable);
                reg.set_ylie(config.axes.y_low && config.enable);
                reg.set_zhie(config.axes.z_high && config.enable);
                reg.set_zlie(config.axes.z_low && config.enable);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .int_1_ths()
            .write_async(|reg: &mut field_sets::Int1Ths| {
                reg.set_ths(config.threshold.min(127));
            })
            .await
            .map_err(Error::from)?;

        self.device
            .int_1_duration()
            .write_async(|reg: &mut field_sets::Int1Duration| {
                reg.set_d(config.duration.min(127));
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .modify_async(|reg: &mut field_sets::CtrlReg5| {
                reg.set_lir_int_1(matches!(config.latch, LatchMode::Latched));
                reg.set_d_4_d_int_1(d4d);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_2()
            .modify_async(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hp_ia_1(config.high_pass_filter);
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }

    async fn write_motion_config_int2(
        &mut self,
        config: MotionConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let (aoi, sixd, d4d) = mode_bits(config.mode);

        self.device
            .int_2_cfg()
            .write_async(|reg: &mut field_sets::Int2Cfg| {
                reg.set_aoi(aoi);
                reg.set_sixd(sixd);
                reg.set_xhie(config.axes.x_high && config.enable);
                reg.set_xlie(config.axes.x_low && config.enable);
                reg.set_yhie(config.axes.y_high && config.enable);
                reg.set_ylie(config.axes.y_low && config.enable);
                reg.set_zhie(config.axes.z_high && config.enable);
                reg.set_zlie(config.axes.z_low && config.enable);
            })
            .await
            .map_err(Error::from)?;

        // Configure threshold
        self.device
            .int_2_ths()
            .write_async(|reg: &mut field_sets::Int2Ths| {
                reg.set_ths(config.threshold.min(127));
            })
            .await
            .map_err(Error::from)?;

        // Configure duration
        self.device
            .int_2_duration()
            .write_async(|reg: &mut field_sets::Int2Duration| {
                reg.set_d(config.duration.min(127));
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .modify_async(|reg: &mut field_sets::CtrlReg5| {
                reg.set_lir_int_2(matches!(config.latch, LatchMode::Latched));
                reg.set_d_4_d_int_2(d4d);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_2()
            .modify_async(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hp_ia_2(config.high_pass_filter);
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }

    async fn write_click_config(
        &mut self,
        config: ClickConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        // Configure CLICK_CFG
        self.device
            .click_cfg()
            .write_async(|reg: &mut field_sets::ClickCfg| {
                reg.set_xs(config.axes.x_single && config.enable);
                reg.set_xd(config.axes.x_double && config.enable);
                reg.set_ys(config.axes.y_single && config.enable);
                reg.set_yd(config.axes.y_double && config.enable);
                reg.set_zs(config.axes.z_single && config.enable);
                reg.set_zd(config.axes.z_double && config.enable);
            })
            .await
            .map_err(Error::from)?;

        // Configure threshold and latch
        self.device
            .click_ths()
            .write_async(|reg: &mut field_sets::ClickThs| {
                reg.set_ths(config.threshold.min(127));
                reg.set_lir_click(matches!(config.latch, LatchMode::Latched));
            })
            .await
            .map_err(Error::from)?;

        // Configure time limit
        self.device
            .time_limit()
            .write_async(|reg: &mut field_sets::TimeLimit| {
                reg.set_tli(config.time_limit.min(127));
            })
            .await
            .map_err(Error::from)?;

        // Configure time latency
        self.device
            .time_latency()
            .write_async(|reg: &mut field_sets::TimeLatency| {
                reg.set_tla(config.time_latency);
            })
            .await
            .map_err(Error::from)?;

        // Configure time window
        self.device
            .time_window()
            .write_async(|reg: &mut field_sets::TimeWindow| {
                reg.set_tw(config.time_window);
            })
            .await
            .map_err(Error::from)?;

        // Configure high-pass filter in CTRL_REG2
        self.device
            .ctrl_reg_2()
            .modify_async(|reg: &mut field_sets::CtrlReg2| {
                reg.set_hpclick(config.high_pass_filter);
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }

    async fn write_activity_config(
        &mut self,
        config: ActivityConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        // Sleep-to-wake is armed by a non-zero ACT_THS; disable clears both registers.
        let (threshold, duration) = if config.enable {
            (config.threshold.min(127), config.duration)
        } else {
            (0, 0)
        };

        self.device
            .act_ths()
            .write_async(|reg: &mut field_sets::ActThs| {
                reg.set_acth(threshold);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .act_dur()
            .write_async(|reg: &mut field_sets::ActDur| {
                reg.set_act_d(duration);
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }

    async fn write_interrupt_config(
        &mut self,
        config: InterruptConfig,
    ) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        // Configure INT1 routing in CTRL_REG3
        self.device
            .ctrl_reg_3()
            .write_async(|reg: &mut field_sets::CtrlReg3| {
                reg.set_i_1_click(config.int1.click);
                reg.set_i_1_ia_1(config.int1.ia1);
                reg.set_i_1_ia_2(config.int1.ia2);
                reg.set_i_1_zyxda(config.int1.data_ready);
                reg.set_i_1_wtm(config.int1.fifo_watermark);
                reg.set_i_1_overrun(config.int1.fifo_overrun);
            })
            .await
            .map_err(Error::from)?;

        // Configure INT2 routing and polarity in CTRL_REG6
        self.device
            .ctrl_reg_6()
            .write_async(|reg: &mut field_sets::CtrlReg6| {
                reg.set_i_2_click(config.int2.click);
                reg.set_i_2_ia_1(config.int2.ia1);
                reg.set_i_2_ia_2(config.int2.ia2);
                reg.set_i_2_boot(config.int2.boot);
                reg.set_i_2_act(config.int2.activity);
                reg.set_int_polarity(matches!(config.polarity, InterruptPolarity::ActiveLow));
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }
}

// I2C-specific async constructors and methods
#[cfg(feature = "async")]
impl<I2C> Lis2de12Async<DeviceInterfaceAsync<I2C>>
where
    I2C: AsyncI2c,
    I2C::Error: Debug,
{
    /// Create a new async I2C driver with the default configuration.
    pub async fn new_i2c(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, addr, Lis2de12Config::default()).await
    }

    /// Create a new async I2C driver with an explicit configuration.
    pub async fn new_i2c_with_config(
        i2c: I2C,
        addr: SlaveAddr,
        config: Lis2de12Config,
    ) -> Result<Self, Error<I2C::Error>> {
        validate_config_common::<I2C::Error>(&config)?;

        let interface = DeviceInterfaceAsync {
            i2c,
            address: addr.addr(),
        };
        let mut device = Lis2de12Device::new(interface);
        verify_device_id_async(&mut device).await?;
        let mut this = Self { device, config };
        this.apply_config(config).await?;
        Ok(this)
    }

    /// Create a new async I2C driver with the default configuration (backward compatible).
    pub async fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c(i2c, addr).await
    }

    /// Create a new driver with an explicit configuration (backward compatible, uses I2C).
    pub async fn new_with_config(i2c: I2C, addr: SlaveAddr, config: Lis2de12Config) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, addr, config).await
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<DeviceInterfaceAsync<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying asynchronous I²C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }
}

// Async SPI-specific constructors and methods
#[cfg(feature = "async")]
impl<SPI> Lis2de12Async<SpiInterfaceAsync<SPI>>
where
    SPI: AsyncSpiDevice,
    SPI::Error: Debug,
{
    /// Create a new asynchronous SPI driver with the default configuration.
    pub async fn new_spi(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        Self::new_spi_with_config(spi, Lis2de12Config::default()).await
    }

    /// Create a new asynchronous SPI driver with an explicit configuration.
    pub async fn new_spi_with_config(spi: SPI, config: Lis2de12Config) -> Result<Self, Error<SPI::Error>> {
        validate_config_common::<SPI::Error>(&config)?;

        let interface = SpiInterfaceAsync { spi };
        let mut device = Lis2de12Device::new(interface);
        verify_device_id_async(&mut device).await?;
        let mut this = Self { device, config };
        this.apply_config(config).await?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<SpiInterfaceAsync<SPI>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying async SPI bus.
    pub fn destroy(self) -> SPI {
        self.device.interface.spi
    }
}

fn verify_device_id<IFACE>(device: &mut Lis2de12Device<IFACE>) -> Result<(), Error<<IFACE as RegisterInterface>::Error>>
where
    IFACE: RegisterInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    let who = device.who_am_i().read().map_err(Error::from)?;
    if who != field_sets::WhoAmI::new() {
        return Err(Error::new(ErrorKind::Device));
    }
    Ok(())
}

#[cfg(feature = "async")]
async fn verify_device_id_async<IFACE>(
    device: &mut Lis2de12Device<IFACE>,
) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>>
where
    IFACE: AsyncRegisterInterface<AddressType = u8>,
    <IFACE as AsyncRegisterInterface>::Error: Debug,
{
    let who = device.who_am_i().read_async().await.map_err(Error::from)?;
    if who != field_sets::WhoAmI::new() {
        return Err(Error::new(ErrorKind::Device));
    }
    Ok(())
}

fn decode_raw(bytes: FifoFrame, _mode: OperatingMode) -> I16x3 {
    // FIFO frame layout: [FIFO_READ_START(0x00), OUT_X_H, reserved, OUT_Y_H, reserved, OUT_Z_H].
    // The LIS2DE12 is always 8-bit; signed axis data sits in bytes[1], [3], [5].
    // Construct i16 from [0, H_byte] then arithmetic-right-shift by 8 to sign-extend.
    let x = i16::from_le_bytes([bytes[0], bytes[1]]);
    let y = i16::from_le_bytes([bytes[2], bytes[3]]);
    let z = i16::from_le_bytes([bytes[4], bytes[5]]);
    I16x3::new(x >> 8, y >> 8, z >> 8)
}

fn scale_to_mg(raw: I16x3, mg_per_lsb: f32) -> I16x3 {
    #[allow(clippy::cast_possible_truncation)]
    fn round_to_i16(value: f32) -> i16 {
        if value >= 0.0 {
            (value + 0.5) as i16
        } else {
            (value - 0.5) as i16
        }
    }

    I16x3::new(
        round_to_i16(f32::from(raw.x) * mg_per_lsb),
        round_to_i16(f32::from(raw.y) * mg_per_lsb),
        round_to_i16(f32::from(raw.z) * mg_per_lsb),
    )
}

fn scale_to_g(raw: I16x3, g_per_lsb: f32) -> F32x3 {
    F32x3::new(
        f32::from(raw.x) * g_per_lsb,
        f32::from(raw.y) * g_per_lsb,
        f32::from(raw.z) * g_per_lsb,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_config_requires_enabled_axis() {
        let mut config = Lis2de12Config::default();
        config.axes = AxesEnable {
            x: false,
            y: false,
            z: false,
        };

        let err = validate_config_common::<()>(&config).unwrap_err();
        assert_eq!(err.kind(), ErrorKind::Device);
    }

    #[test]
    fn validate_config_enforces_watermark_limit() {
        let mut config = Lis2de12Config::default();
        config.fifo = FifoConfig::enabled(FifoMode::Fifo).with_watermark(FIFO_WATERMARK_MAX + 1);

        let err = validate_config_common::<()>(&config).unwrap_err();
        assert_eq!(err.kind(), ErrorKind::Param);
    }

    #[test]
    fn validate_config_accepts_valid_fifo_settings() {
        let mut config = Lis2de12Config::default();
        config.fifo = FifoConfig::enabled(FifoMode::Stream).with_watermark(FIFO_WATERMARK_MAX);

        assert!(validate_config_common::<()>(&config).is_ok());
    }

    #[test]
    fn fifo_status_parses_flags_and_length() {
        let mut raw = field_sets::FifoSrcReg::new();
        raw.set_wtm(true);
        raw.set_ovrn_fifo(false);
        raw.set_empty(false);
        raw.set_fss(5);

        let status = FifoStatus::from_raw(raw);
        assert!(status.is_watermark_triggered());
        assert!(!status.is_overrun());
        assert!(!status.is_empty());
        assert_eq!(status.len(), 5);
        assert_eq!(status.remaining_capacity(), (FIFO_CAPACITY as u8).saturating_sub(5));
    }

    #[test]
    fn fifo_status_reports_full_when_overrun() {
        let mut raw = field_sets::FifoSrcReg::new();
        raw.set_wtm(true);
        raw.set_ovrn_fifo(true);
        raw.set_empty(false);
        raw.set_fss(0);

        let status = FifoStatus::from_raw(raw);
        assert!(status.is_overrun());
        assert_eq!(status.len(), FIFO_CAPACITY as u8);
    }

    #[test]
    fn decode_raw_extracts_h_byte_with_sign_extension() {
        // FIFO frame: [pad, OUT_X_H, pad, OUT_Y_H, pad, OUT_Z_H]
        // Positive values: H-byte in odd positions, even positions are placeholders.
        let frame: FifoFrame = [0x00, 0x01, 0x00, 0x02, 0x00, 0x03];
        let decoded = decode_raw(frame, OperatingMode::LowPower);
        assert_eq!(decoded.x, 1);
        assert_eq!(decoded.y, 2);
        assert_eq!(decoded.z, 3);

        // Negative value: 0xFF in the H-byte position should decode to -1.
        let frame_neg: FifoFrame = [0x00, 0xFF, 0x00, 0xFE, 0x00, 0x80];
        let decoded_neg = decode_raw(frame_neg, OperatingMode::LowPower);
        assert_eq!(decoded_neg.x, -1);
        assert_eq!(decoded_neg.y, -2);
        assert_eq!(decoded_neg.z, -128);
    }

    #[test]
    fn scale_to_mg_rounds_toward_nearest() {
        let raw = I16x3::new(1, -1, 2);
        let scaled = scale_to_mg(raw, 1.5);

        assert_eq!(scaled.x, 2);
        assert_eq!(scaled.y, -2);
        assert_eq!(scaled.z, 3);
    }

    #[test]
    fn scale_to_g_scales_components() {
        let raw = I16x3::new(2, -4, 0);
        let scaled = scale_to_g(raw, 0.5);

        assert!((scaled.x - 1.0).abs() <= f32::EPSILON);
        assert!((scaled.y + 2.0).abs() <= f32::EPSILON);
        assert!(scaled.z.abs() <= f32::EPSILON);
    }

    #[test]
    fn odr_to_hz_maps_supported_data_rates() {
        assert_eq!(odr_to_hz(Odr::OneHz), Some(1.0));
        assert_eq!(odr_to_hz(Odr::TenHz), Some(10.0));
        assert_eq!(odr_to_hz(Odr::TwentyFiveHz), Some(25.0));
        assert_eq!(odr_to_hz(Odr::FiftyHz), Some(50.0));
        assert_eq!(odr_to_hz(Odr::HundredHz), Some(100.0));
        assert_eq!(odr_to_hz(Odr::TwoHundredHz), Some(200.0));
        assert_eq!(odr_to_hz(Odr::FourHundredHz), Some(400.0));
        assert_eq!(odr_to_hz(Odr::SixteenTwentyHz), Some(1620.0));
        assert_eq!(odr_to_hz(Odr::FiveThousandHz), Some(5376.0));
    }

    #[test]
    fn odr_to_hz_returns_none_in_power_down() {
        assert_eq!(odr_to_hz(Odr::PowerDown), None);
    }

    #[test]
    fn fifo_config_effective_threshold_zero_when_disabled() {
        let config = FifoConfig {
            enable: false,
            mode: FifoMode::Bypass,
            watermark: Some(12),
            trigger: FifoTriggerPin::Int1,
        };

        assert_eq!(config.effective_threshold(), 0);
    }

    #[test]
    fn fifo_config_effective_threshold_clamps_to_max() {
        let config = FifoConfig {
            enable: true,
            mode: FifoMode::Stream,
            watermark: Some(40),
            trigger: FifoTriggerPin::Int1,
        };

        assert_eq!(config.effective_threshold(), FIFO_WATERMARK_MAX);
    }

    #[test]
    fn fifo_config_with_trigger_selects_int2() {
        let config = FifoConfig::enabled(FifoMode::StreamToFifo).with_trigger(FifoTriggerPin::Int2);
        assert!(config.trigger_bit());
        assert!(matches!(config.trigger, FifoTriggerPin::Int2));
    }

    #[test]
    fn evaluate_self_test_accepts_in_range_delta_at_2g() {
        let nost = I16x3::new(0, 0, 0);
        let st = I16x3::new(20, -25, 40);
        let result = evaluate_self_test(nost, st, Fs::PlusMinus2G);
        assert!(result.passed());
        assert_eq!(result.delta.x, 20);
        assert_eq!(result.delta.y, 25);
        assert_eq!(result.delta.z, 40);
    }

    #[test]
    fn evaluate_self_test_rejects_out_of_range_delta() {
        let nost = I16x3::new(0, 0, 0);
        let st = I16x3::new(1, 20, 20); // X below min
        let result = evaluate_self_test(nost, st, Fs::PlusMinus2G);
        assert!(!result.passed());
        assert!(!result.x_ok);
        assert!(result.y_ok);
        assert!(result.z_ok);
    }

    #[test]
    fn self_test_limits_scale_with_full_scale() {
        let (min_2g, max_2g) = self_test_limits_lsb(Fs::PlusMinus2G);
        assert_eq!(min_2g, SELF_TEST_MIN_LSB_2G);
        assert_eq!(max_2g, SELF_TEST_MAX_LSB_2G);

        let (min_4g, max_4g) = self_test_limits_lsb(Fs::PlusMinus4G);
        // Sensitivity doubles → LSb limits halve
        assert_eq!(min_4g, 2);
        assert_eq!(max_4g, 45);
    }

    #[test]
    fn high_pass_config_builder() {
        let config = HighPassConfig::new()
            .with_mode(Hpm::AutoresetOnInt)
            .with_cutoff(Hpcf::HalfPercent)
            .with_filtered_data_selection(true);
        assert!(matches!(config.mode, Hpm::AutoresetOnInt));
        assert!(matches!(config.cutoff, Hpcf::HalfPercent));
        assert!(config.filtered_data_selection);
    }

    #[test]
    fn data_status_helpers() {
        let status = DataStatus {
            xyz_data_available: true,
            x_data_available: true,
            y_data_available: true,
            z_data_available: true,
            xyz_overrun: false,
            x_overrun: true,
            y_overrun: false,
            z_overrun: false,
        };
        assert!(status.data_ready());
        assert!(status.any_overrun());
    }

    #[test]
    fn fifo_status_reports_empty_flag() {
        let mut raw = field_sets::FifoSrcReg::new();
        raw.set_wtm(false);
        raw.set_ovrn_fifo(false);
        raw.set_empty(true);
        raw.set_fss(7);

        let status = FifoStatus::from_raw(raw);
        assert!(status.is_empty());
        assert_eq!(status.len(), 0);
        assert_eq!(status.remaining_capacity(), FIFO_CAPACITY as u8);
    }

    // ========================================================================
    // Interrupt Configuration Tests
    // ========================================================================

    #[test]
    fn motion_axes_config_all_high_enables_only_high() {
        let axes = MotionAxesConfig::all_high();
        assert!(axes.x_high);
        assert!(!axes.x_low);
        assert!(axes.y_high);
        assert!(!axes.y_low);
        assert!(axes.z_high);
        assert!(!axes.z_low);
        assert!(axes.any());
    }

    #[test]
    fn motion_axes_config_none_disables_all() {
        let axes = MotionAxesConfig::none();
        assert!(!axes.x_high);
        assert!(!axes.x_low);
        assert!(!axes.y_high);
        assert!(!axes.y_low);
        assert!(!axes.z_high);
        assert!(!axes.z_low);
        assert!(!axes.any());
    }

    #[test]
    fn motion_config_builder_methods() {
        let config = MotionConfig::disabled()
            .with_enable(true)
            .with_threshold(200) // Should be clamped to 127
            .with_duration(64)
            .with_latch(LatchMode::Latched)
            .with_mode(MotionDetectionMode::AndCombination)
            .with_axes(MotionAxesConfig::all_high())
            .with_high_pass_filter(true);

        assert!(config.enable);
        assert_eq!(config.threshold, 127); // Clamped
        assert_eq!(config.duration, 64);
        assert!(matches!(config.latch, LatchMode::Latched));
        assert!(matches!(config.mode, MotionDetectionMode::AndCombination));
        assert!(config.axes.x_high);
        assert!(config.high_pass_filter);
    }

    #[test]
    fn click_axes_config_all_single_enables_only_single() {
        let axes = ClickAxesConfig::all_single();
        assert!(axes.x_single);
        assert!(!axes.x_double);
        assert!(axes.y_single);
        assert!(!axes.y_double);
        assert!(axes.z_single);
        assert!(!axes.z_double);
        assert!(axes.any());
    }

    #[test]
    fn click_config_builder_methods() {
        let config = ClickConfig::disabled()
            .with_enable(true)
            .with_threshold(50)
            .with_time_limit(10)
            .with_time_latency(20)
            .with_time_window(100)
            .with_latch(LatchMode::Latched)
            .with_axes(ClickAxesConfig::all_double());

        assert!(config.enable);
        assert_eq!(config.threshold, 50);
        assert_eq!(config.time_limit, 10);
        assert_eq!(config.time_latency, 20);
        assert_eq!(config.time_window, 100);
        assert!(matches!(config.latch, LatchMode::Latched));
        assert!(config.axes.x_double);
    }

    #[test]
    fn activity_config_threshold_clamps() {
        let config = ActivityConfig::disabled().with_threshold(255); // Should be clamped to 127

        assert_eq!(config.threshold, 127);
    }

    #[test]
    fn int1_routing_none_disables_all() {
        let routing = Int1Routing::none();
        assert!(!routing.click);
        assert!(!routing.ia1);
        assert!(!routing.ia2);
        assert!(!routing.data_ready);
        assert!(!routing.fifo_watermark);
        assert!(!routing.fifo_overrun);
        assert!(!routing.any());
    }

    #[test]
    fn int2_routing_any_detects_enabled() {
        let mut routing = Int2Routing::none();
        assert!(!routing.any());
        routing.activity = true;
        assert!(routing.any());
    }

    #[test]
    fn motion_status_event_detection() {
        let status = MotionStatus {
            active: true,
            x_high: true,
            x_low: false,
            y_high: false,
            y_low: true,
            z_high: false,
            z_low: false,
        };

        assert!(status.is_active());
        assert!(status.x_event());
        assert!(status.y_event());
        assert!(!status.z_event());
        assert!(status.any_event());
    }

    #[test]
    fn motion_presets_match_st_patterns() {
        let ff = MotionConfig::free_fall_default();
        assert!(ff.enable);
        assert!(matches!(ff.mode, MotionDetectionMode::AndCombination));
        assert!(ff.axes.x_low && ff.axes.y_low && ff.axes.z_low);
        assert!(!ff.axes.x_high && !ff.axes.y_high && !ff.axes.z_high);
        assert_eq!(ff.threshold, FREE_FALL_THRESHOLD_350MG_2G);
        assert_eq!(ff.duration, FREE_FALL_DURATION_DEFAULT);
        assert!(!ff.high_pass_filter);
        assert!(matches!(ff.latch, LatchMode::Latched));

        let wake = MotionConfig::wake_up_default();
        assert!(matches!(wake.mode, MotionDetectionMode::OrCombination));
        assert!(wake.axes.x_high && wake.axes.y_high && wake.axes.z_high);
        assert!(!wake.axes.x_low);
        assert!(wake.high_pass_filter);
        assert_eq!(wake.threshold, WAKE_UP_THRESHOLD_250MG_2G);

        let orient6 = MotionConfig::orientation_6d_default();
        assert!(matches!(orient6.mode, MotionDetectionMode::SixDirectionPosition));
        assert!(orient6.axes.any());
        assert!(orient6.axes.z_high);

        let orient4 = MotionConfig::orientation_4d_default();
        assert!(matches!(orient4.mode, MotionDetectionMode::FourDirectionPosition));
        assert!(orient4.axes.x_high && orient4.axes.y_low);
        assert!(!orient4.axes.z_high && !orient4.axes.z_low);
    }

    #[test]
    fn motion_status_orientation_decodes_single_face() {
        let zh = MotionStatus {
            active: true,
            z_high: true,
            ..MotionStatus::default()
        };
        assert!(matches!(zh.orientation(), DeviceOrientation::PlusZ));
        assert!(zh.orientation().is_known());
        assert!(zh.orientation().display_orientation().is_none());

        let xl = MotionStatus {
            active: true,
            x_low: true,
            ..MotionStatus::default()
        };
        assert!(matches!(xl.orientation(), DeviceOrientation::MinusX));
        assert!(matches!(
            xl.orientation().display_orientation(),
            Some(DisplayOrientation::Landscape)
        ));

        let ambiguous = MotionStatus {
            active: true,
            x_high: true,
            y_high: true,
            ..MotionStatus::default()
        };
        assert!(matches!(ambiguous.orientation(), DeviceOrientation::Unknown));
    }

    #[test]
    fn click_status_detection() {
        let status = ClickStatus {
            active: true,
            double_click: false,
            single_click: true,
            negative: true,
            x: false,
            y: false,
            z: true,
        };

        assert!(status.is_active());
        assert!(!status.is_double_click());
        assert!(status.is_single_click());
        assert!(status.is_negative());
        assert!(status.any_event());
    }

    #[test]
    fn interrupt_sources_any_active() {
        let sources = InterruptSources {
            motion1: MotionStatus {
                active: false,
                ..Default::default()
            },
            motion2: MotionStatus {
                active: true,
                ..Default::default()
            },
            click: ClickStatus::default(),
        };

        assert!(sources.any_active());
    }

    #[test]
    fn interrupt_config_builder() {
        let config = InterruptConfig::disabled()
            .with_polarity(InterruptPolarity::ActiveLow)
            .with_int1(Int1Routing {
                ia1: true,
                ..Default::default()
            })
            .with_int2(Int2Routing {
                activity: true,
                ..Default::default()
            });

        assert!(matches!(config.polarity, InterruptPolarity::ActiveLow));
        assert!(config.int1.ia1);
        assert!(config.int2.activity);
    }
}
