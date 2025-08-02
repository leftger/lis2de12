//! Platform-agnostic LIS2DE12 accelerometer driver generated from `lis2de12.yaml`.
//! Implements the [`Accelerometer`] and [`RawAccelerometer`] traits from the `accelerometer` crate.

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Error, ErrorKind, RawAccelerometer};
use device_driver::create_device;
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
use hal::i2c::I2c;

// RegisterInterface & BufferInterface implementations for blocking I²C

create_device!(
    device_name: Lis2de12Device,
    manifest: "src/lis2de12.yaml"
);

/// I²C bus interface for LIS2DE12.
pub struct DeviceInterface<I2C> {
    /// Underlying I²C bus
    pub i2c: I2C,
    /// I²C slave address
    pub address: u8,
}

impl<I2C> BufferInterfaceError for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
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
        // Write [address, data...]
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        buf[1..=data.len()].copy_from_slice(data);
        self.i2c.write(address, &buf[..=data.len()])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(address, &[address], data)
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
        self.i2c.write_read(address, &[address], buf)?;
        Ok(buf.len())
    }

    fn write(
        &mut self,
        address: Self::AddressType,
        buf: &[u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        data[1..=buf.len()].copy_from_slice(buf);
        self.i2c.write(address, &data[..=buf.len()])?;
        Ok(buf.len())
    }

    fn flush(
        &mut self,
        _address: Self::AddressType,
    ) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}
// ...existing code...
// Implement RawAccelerometer for Lis2de12<I2C> so accel_raw is available
impl<I2C> RawAccelerometer<I16x3> for Lis2de12<I2C>
where
    I2C: I2c,
    I2C::Error: core::fmt::Debug,
{
    type Error = Error<Error<Error<Error<Error<Error<I2C::Error>>>>>>;

    fn accel_raw(&mut self) -> Result<I16x3, Self::Error> {
        let mut buf = [0u8; 6];
        self.device
            .interface
            .read_register(0x28, 8, &mut buf)
            .map_err(|e| {
                Error::from(Error::from(Error::from(Error::from(Error::from(
                    Error::from(e),
                )))))
            })?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok(I16x3::new(x, y, z))
    }
}

/// Possible I²C slave addresses for the LIS2DE12.
pub enum SlaveAddr {
    /// SA0 = 0 -> `0x31`
    Default,
    /// SA0 = 1 -> `0x33`
    Alternative,
}

impl SlaveAddr {
    fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => 0x31,
            SlaveAddr::Alternative => 0x33,
        }
    }
}

/// High-level LIS2DE12 driver.
pub struct Lis2de12<I2C> {
    /// Underlying generated device implementation.
    device: Lis2de12Device<DeviceInterface<I2C>>,
}

impl<I2C> Lis2de12<I2C>
where
    I2C: I2c,
{
    /// Create a new driver from an I²C bus instance and selected address.
    ///
    /// Returns an error if the WHO_AM_I register does not match the expected device ID.
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        let interface = DeviceInterface {
            i2c,
            address: addr.addr(),
        };
        let mut device = Lis2de12Device::new(interface);
        // Verify device ID (WHO_AM_I = 0x33)
        let mut who_buf = [0u8; 1];
        device
            .interface
            .read_register(0x0F, 8, &mut who_buf)
            .map_err(Error::from)?;
        if who_buf[0] != 0x33 {
            ErrorKind::Device.err()?;
        }
        Ok(Self { device })
    }

    /// Consume the driver and return the inner I²C bus.
    pub fn destroy(self) -> I2C {
        // Access the underlying interface to extract the bus
        let Lis2de12Device { interface, .. } = self.device;
        interface.i2c
    }

    /// Read a generic register
    pub fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.device
            .interface
            .read_register(reg, 8, buf)
            .map_err(Error::from)
    }

    /// Write a generic register
    pub fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.device
            .interface
            .write_register(reg, 8, data)
            .map_err(Error::from)
    }

    /// Get device ID (WHO_AM_I)
    pub fn device_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.read_reg(0x0F, &mut buf)?;
        Ok(buf[0])
    }

    /// Get raw temperature value
    pub fn temperature_raw(&mut self) -> Result<i16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_reg(0x0C, &mut buf)?;
        Ok(((buf[1] as i16) << 8) | (buf[0] as i16))
    }

    /// Set output data rate
    pub fn set_data_rate(&mut self, odr: u8) -> Result<(), Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.read_reg(0x20, &mut buf)?;
        buf[0] = (buf[0] & !0xF0) | ((odr & 0x0F) << 4);
        self.write_reg(0x20, &buf)
    }

    /// Get output data rate
    pub fn get_data_rate(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.read_reg(0x20, &mut buf)?;
        Ok((buf[0] >> 4) & 0x0F)
    }

    // Add more methods for other register operations as needed...
}

// ...existing code...

// ...existing code...

use accelerometer::Accelerometer;
#[cfg(feature = "out_f32")]
use accelerometer::vector::F32x3;

impl<I2C: I2c> Accelerometer for Lis2de12<I2C> {
    type Error = Error<I2C::Error>;

    /// Get normalized ±g reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        let acc_raw = self.accel_raw().map_err(Error::from)?;
        // Example conversion: scale raw value to g assuming 16-bit full scale ±2g
        // You may want to adjust this based on your sensor configuration
        const SCALE: f32 = 2.0 / 32768.0;
        Ok(F32x3::new(
            acc_raw.x as f32 * SCALE,
            acc_raw.y as f32 * SCALE,
            acc_raw.z as f32 * SCALE,
        ))
    }

    /// Get sample rate of accelerometer in Hz
    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        let mut buf = [0u8; 1];
        self.read_reg(0x20, &mut buf)?;
        let odr = (buf[0] >> 4) & 0x0F;
        let rate = match odr {
            0x0 => 0.0, // Power-down
            0x1 => 1.0,
            0x2 => 10.0,
            0x3 => 25.0,
            0x4 => 50.0,
            0x5 => 100.0,
            0x6 => 200.0,
            0x7 => 400.0,
            0x8 => 1620.0, // High rate
            0x9 => 5376.0, // High rate
            _ => 0.0,
        };
        Ok(rate)
    }
}
