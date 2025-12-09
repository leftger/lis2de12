//! High-level LIS2DE12 accelerometer driver with synchronous and asynchronous FIFO helpers.

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Error, ErrorKind, RawAccelerometer};
use core::{cmp, fmt::Debug};
#[cfg(feature = "async")]
use device_driver::{AsyncBufferInterface, AsyncRegisterInterface};
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal_async;
use hal::i2c::I2c;
#[cfg(feature = "async")]
use hal_async::i2c::I2c as AsyncI2c;

#[allow(unsafe_code)]
#[allow(missing_docs)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/lis2de12_device.rs"));
}

pub use generated::{Fm, Fs, Lis2de12Device, Odr, St, TempEn, field_sets};

/// Number of bytes per FIFO frame (X, Y, Z 16-bit samples).
pub const FIFO_FRAME_BYTES: usize = 6;
/// Maximum FIFO depth in frames.
pub const FIFO_CAPACITY: usize = 32;
/// FIFO frame buffer alias.
pub type FifoFrame = [u8; FIFO_FRAME_BYTES];
/// Highest programmable FIFO watermark level.
pub const FIFO_WATERMARK_MAX: u8 = 31;

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

/// FIFO configuration options.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FifoConfig {
    /// Enable or disable the hardware FIFO.
    pub enable: bool,
    /// FIFO operating mode to apply when enabled.
    pub mode: FifoMode,
    /// Optional watermark threshold (0–31 frames).
    pub watermark: Option<u8>,
}

impl FifoConfig {
    /// Disabled FIFO configuration helper.
    pub const fn disabled() -> Self {
        Self {
            enable: false,
            mode: FifoMode::Bypass,
            watermark: None,
        }
    }

    /// Enabled FIFO configuration helper with the given mode.
    pub const fn enabled(mode: FifoMode) -> Self {
        Self {
            enable: true,
            mode,
            watermark: None,
        }
    }

    /// Attach a watermark level to this configuration.
    pub const fn with_watermark(self, watermark: u8) -> Self {
        Self {
            watermark: Some(watermark),
            ..self
        }
    }

    /// Watermark value written to the device.
    fn effective_threshold(self) -> u8 {
        if !self.enable {
            0
        } else {
            match self.watermark {
                Some(level) if level <= FIFO_WATERMARK_MAX => level,
                Some(_) | None => FIFO_WATERMARK_MAX,
            }
        }
    }
}

/// Sensor operating mode controlling resolution and power consumption.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OperatingMode {
    /// Normal mode (10-bit resolution).
    Normal,
    /// Low-power mode (8-bit resolution).
    LowPower,
}

impl OperatingMode {
    const fn resolution_bits(self) -> u8 {
        match self {
            OperatingMode::Normal => 10,
            OperatingMode::LowPower => 8,
        }
    }

    const fn lpen(self) -> bool {
        matches!(self, OperatingMode::LowPower)
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
            mode: OperatingMode::Normal,
            scale: Fs::PlusMinus2G,
            axes: AxesEnable::default(),
            block_data_update: true,
            temperature_enable: false,
            fifo: FifoConfig::disabled(),
        }
    }
}

impl Lis2de12Config {
    fn resolution_bits(self) -> u8 {
        self.mode.resolution_bits()
    }

    fn sensitivity_g_per_lsb(self) -> f32 {
        let full_scale_g = match self.scale {
            Fs::PlusMinus2G => 2.0,
            Fs::PlusMinus4G => 4.0,
            Fs::PlusMinus8G => 8.0,
            Fs::PlusMinus16G => 16.0,
        };
        let denom = 1u32 << (self.resolution_bits() - 1);
        full_scale_g / denom as f32
    }

    fn sensitivity_mg_per_lsb(self) -> f32 {
        self.sensitivity_g_per_lsb() * 1000.0
    }
}

/// Parsed FIFO status register snapshot.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
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
            FIFO_CAPACITY as u8
        } else {
            raw_frames.min(FIFO_CAPACITY as u8)
        };

        Self {
            watermark: raw.wtm(),
            overrun: raw.ovrn_fifo(),
            empty: raw.empty(),
            frames,
        }
    }

    /// Number of unread frames currently buffered.
    pub const fn len(self) -> u8 {
        self.frames
    }

    /// Returns `true` if the FIFO currently holds data.
    pub const fn has_data(self) -> bool {
        self.frames > 0
    }

    /// Remaining capacity before the FIFO becomes full.
    pub const fn remaining_capacity(self) -> u8 {
        (FIFO_CAPACITY as u8).saturating_sub(self.frames)
    }

    /// Returns `true` if the configured watermark has been reached.
    pub const fn is_watermark_triggered(self) -> bool {
        self.watermark
    }

    /// Returns `true` if unread data has been overwritten.
    pub const fn is_overrun(self) -> bool {
        self.overrun
    }

    /// Returns `true` if the FIFO is empty.
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
    if config.fifo.enable {
        if let Some(watermark) = config.fifo.watermark {
            if watermark > FIFO_WATERMARK_MAX {
                return Err(Error::new(ErrorKind::Param));
            }
        }
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

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        buf[1..1 + data.len()].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..1 + data.len()])
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
        buf[1..1 + data.len()].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..1 + data.len()]).await
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
        self.i2c.write_read(self.address, &[address], buf)?;
        Ok(buf.len())
    }

    fn write(
        &mut self,
        address: Self::AddressType,
        buf: &[u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        data[1..1 + buf.len()].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..1 + buf.len()])?;
        Ok(buf.len())
    }

    fn flush(
        &mut self,
        _address: Self::AddressType,
    ) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncBufferInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type AddressType = u8;

    async fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, Self::Error> {
        self.i2c.write_read(self.address, &[address], buf).await?;
        Ok(buf.len())
    }

    async fn write(
        &mut self,
        address: Self::AddressType,
        buf: &[u8],
    ) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        data[1..1 + buf.len()].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..1 + buf.len()]).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Available LIS2DE12 I²C slave addresses (controlled by the SA0 pin).
pub enum SlaveAddr {
    /// SA0 pulled low (`0x31`).
    Default,
    /// SA0 pulled high (`0x33`).
    Alternative,
}

impl SlaveAddr {
    const fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => 0x31,
            SlaveAddr::Alternative => 0x33,
        }
    }
}

/// Blocking LIS2DE12 driver.
pub struct Lis2de12<I2C> {
    device: Lis2de12Device<DeviceInterface<I2C>>,
    config: Lis2de12Config,
}

impl<I2C> Lis2de12<I2C>
where
    I2C: I2c,
    I2C::Error: Debug,
{
    /// Create a new driver with the default configuration.
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_with_config(i2c, addr, Lis2de12Config::default())
    }

    /// Create a new driver with an explicit configuration.
    pub fn new_with_config(
        i2c: I2C,
        addr: SlaveAddr,
        config: Lis2de12Config,
    ) -> Result<Self, Error<I2C::Error>> {
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

    /// Return the active configuration.
    pub const fn config(&self) -> Lis2de12Config {
        self.config
    }

    /// Update the sensor configuration.
    pub fn set_config(&mut self, config: Lis2de12Config) -> Result<(), Error<I2C::Error>> {
        validate_config_common::<I2C::Error>(&config)?;
        self.apply_config(config)?;
        self.config = config;
        Ok(())
    }

    /// Read raw acceleration sample triplet.
    pub fn read_raw(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let frame = self.read_fifo_frame()?;
        Ok(decode_raw(&frame, self.config.mode))
    }

    /// Read acceleration expressed in milli-g.
    pub fn read_mg(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_mg(raw, self.config.sensitivity_mg_per_lsb()))
    }

    /// Read acceleration expressed in g (floating point).
    pub fn read_g(&mut self) -> Result<F32x3, Error<I2C::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_g(raw, self.config.sensitivity_g_per_lsb()))
    }

    /// Read a single FIFO frame (XYZ sample) using blocking I²C transfers.
    pub fn read_fifo_frame(&mut self) -> Result<FifoFrame, Error<I2C::Error>> {
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
    ) -> Result<usize, Error<I2C::Error>> {
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
    pub fn drain_fifo(&mut self) -> Result<usize, Error<I2C::Error>> {
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
    pub fn fifo_status(&mut self) -> Result<FifoStatus, Error<I2C::Error>> {
        let status = self.device.fifo_src_reg().read().map_err(Error::from)?;
        Ok(FifoStatus::from(status))
    }

    /// Enable or disable the on-die temperature sensor.
    pub fn set_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.write_temperature_sensor(enable)?;
        self.config.temperature_enable = enable;
        Ok(())
    }

    /// Issue a reboot command to reload memory content.
    pub fn reboot(&mut self) -> Result<(), Error<I2C::Error>> {
        self.device
            .ctrl_reg_5()
            .write(|reg| reg.set_boot(true))
            .map_err(Error::from)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<DeviceInterface<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying I²C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }

    fn write_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.device
            .temp_cfg_reg()
            .write(|reg| {
                reg.set_temp_en(if enable {
                    TempEn::TempEn
                } else {
                    TempEn::TempDis
                });
            })
            .map_err(Error::from)
    }

    fn apply_config(&mut self, config: Lis2de12Config) -> Result<(), Error<I2C::Error>> {
        self.device
            .ctrl_reg_1()
            .write(|reg| {
                reg.set_odr(config.odr);
                reg.set_lpen(config.mode.lpen());
                reg.set_xen(config.axes.x);
                reg.set_yen(config.axes.y);
                reg.set_zen(config.axes.z);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_4()
            .write(|reg| {
                reg.set_bdu(config.block_data_update);
                reg.set_fs(config.scale);
                reg.set_st(St::Normal);
                reg.set_sim(false);
            })
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .write(|reg| {
                reg.set_fifo_en(config.fifo.enable);
            })
            .map_err(Error::from)?;

        self.device
            .fifo_ctrl_reg()
            .write(|reg| {
                reg.set_fm(config.fifo.mode.into());
                reg.set_tr(false);
                reg.set_fth(config.fifo.effective_threshold());
            })
            .map_err(Error::from)?;

        self.write_temperature_sensor(config.temperature_enable)?;
        Ok(())
    }
}

impl<I2C> RawAccelerometer<I16x3> for Lis2de12<I2C>
where
    I2C: I2c,
{
    type Error = I2C::Error;

    fn accel_raw(&mut self) -> Result<I16x3, Error<Self::Error>> {
        self.read_raw()
    }
}

/// Asynchronous LIS2DE12 driver.
#[cfg(feature = "async")]
pub struct Lis2de12Async<I2C> {
    device: Lis2de12Device<DeviceInterfaceAsync<I2C>>,
    config: Lis2de12Config,
}

#[cfg(feature = "async")]
impl<I2C> Lis2de12Async<I2C>
where
    I2C: AsyncI2c,
    I2C::Error: Debug,
{
    /// Create a new async driver with the default configuration.
    pub async fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        Self::new_with_config(i2c, addr, Lis2de12Config::default()).await
    }

    /// Create a new async driver with an explicit configuration.
    pub async fn new_with_config(
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

    /// Return the active configuration.
    pub const fn config(&self) -> Lis2de12Config {
        self.config
    }

    /// Update the sensor configuration asynchronously.
    pub async fn set_config(&mut self, config: Lis2de12Config) -> Result<(), Error<I2C::Error>> {
        validate_config_common::<I2C::Error>(&config)?;
        self.apply_config(config).await?;
        self.config = config;
        Ok(())
    }

    /// Read raw acceleration sample triplet asynchronously.
    pub async fn read_raw(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let frame = self.read_fifo_frame().await?;
        Ok(decode_raw(&frame, self.config.mode))
    }

    /// Read acceleration expressed in milli-g asynchronously.
    pub async fn read_mg(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_mg(raw, self.config.sensitivity_mg_per_lsb()))
    }

    /// Read acceleration expressed in g asynchronously.
    pub async fn read_g(&mut self) -> Result<F32x3, Error<I2C::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_g(raw, self.config.sensitivity_g_per_lsb()))
    }

    /// Read a single FIFO frame asynchronously.
    pub async fn read_fifo_frame(&mut self) -> Result<FifoFrame, Error<I2C::Error>> {
        let mut buf: FifoFrame = [0; FIFO_FRAME_BYTES];
        let mut fifo = self.device.fifo_read_start();
        let mut offset = 0;
        while offset < buf.len() {
            let read = fifo
                .read_async(&mut buf[offset..])
                .await
                .map_err(Error::from)?;
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
    ) -> Result<usize, Error<I2C::Error>> {
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
    pub async fn drain_fifo(&mut self) -> Result<usize, Error<I2C::Error>> {
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
    pub async fn fifo_status(&mut self) -> Result<FifoStatus, Error<I2C::Error>> {
        let status = self
            .device
            .fifo_src_reg()
            .read_async()
            .await
            .map_err(Error::from)?;
        Ok(FifoStatus::from(status))
    }

    /// Enable or disable the on-die temperature sensor asynchronously.
    pub async fn set_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.write_temperature_sensor(enable).await?;
        self.config.temperature_enable = enable;
        Ok(())
    }

    /// Issue a reboot command asynchronously.
    pub async fn reboot(&mut self) -> Result<(), Error<I2C::Error>> {
        self.device
            .ctrl_reg_5()
            .write_async(|reg| reg.set_boot(true))
            .await
            .map_err(Error::from)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2de12Device<DeviceInterfaceAsync<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying asynchronous I²C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }

    async fn write_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.device
            .temp_cfg_reg()
            .write_async(|reg| {
                reg.set_temp_en(if enable {
                    TempEn::TempEn
                } else {
                    TempEn::TempDis
                });
            })
            .await
            .map_err(Error::from)
    }

    async fn apply_config(&mut self, config: Lis2de12Config) -> Result<(), Error<I2C::Error>> {
        self.device
            .ctrl_reg_1()
            .write_async(|reg| {
                reg.set_odr(config.odr);
                reg.set_lpen(config.mode.lpen());
                reg.set_xen(config.axes.x);
                reg.set_yen(config.axes.y);
                reg.set_zen(config.axes.z);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_4()
            .write_async(|reg| {
                reg.set_bdu(config.block_data_update);
                reg.set_fs(config.scale);
                reg.set_st(St::Normal);
                reg.set_sim(false);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .ctrl_reg_5()
            .write_async(|reg| {
                reg.set_fifo_en(config.fifo.enable);
            })
            .await
            .map_err(Error::from)?;

        self.device
            .fifo_ctrl_reg()
            .write_async(|reg| {
                reg.set_fm(config.fifo.mode.into());
                reg.set_tr(false);
                reg.set_fth(config.fifo.effective_threshold());
            })
            .await
            .map_err(Error::from)?;

        self.write_temperature_sensor(config.temperature_enable)
            .await?;
        Ok(())
    }
}

fn verify_device_id<I2C>(
    device: &mut Lis2de12Device<DeviceInterface<I2C>>,
) -> Result<(), Error<I2C::Error>>
where
    I2C: I2c,
{
    let who = device.who_am_i().read().map_err(Error::from)?;
    if who != field_sets::WhoAmI::new() {
        return Err(Error::new(ErrorKind::Device));
    }
    Ok(())
}

#[cfg(feature = "async")]
async fn verify_device_id_async<I2C>(
    device: &mut Lis2de12Device<DeviceInterfaceAsync<I2C>>,
) -> Result<(), Error<I2C::Error>>
where
    I2C: AsyncI2c,
{
    let who = device.who_am_i().read_async().await.map_err(Error::from)?;
    if who != field_sets::WhoAmI::new() {
        return Err(Error::new(ErrorKind::Device));
    }
    Ok(())
}

fn decode_raw(bytes: &FifoFrame, mode: OperatingMode) -> I16x3 {
    let x = i16::from_le_bytes([bytes[0], bytes[1]]);
    let y = i16::from_le_bytes([bytes[2], bytes[3]]);
    let z = i16::from_le_bytes([bytes[4], bytes[5]]);
    let shift = match mode {
        OperatingMode::Normal => 6,
        OperatingMode::LowPower => 8,
    };
    I16x3::new(x >> shift, y >> shift, z >> shift)
}

fn scale_to_mg(raw: I16x3, mg_per_lsb: f32) -> I16x3 {
    fn round_to_i16(value: f32) -> i16 {
        if value >= 0.0 {
            (value + 0.5) as i16
        } else {
            (value - 0.5) as i16
        }
    }

    I16x3::new(
        round_to_i16(raw.x as f32 * mg_per_lsb),
        round_to_i16(raw.y as f32 * mg_per_lsb),
        round_to_i16(raw.z as f32 * mg_per_lsb),
    )
}

fn scale_to_g(raw: I16x3, g_per_lsb: f32) -> F32x3 {
    F32x3::new(
        raw.x as f32 * g_per_lsb,
        raw.y as f32 * g_per_lsb,
        raw.z as f32 * g_per_lsb,
    )
}
