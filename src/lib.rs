//! Platform-agnostic LIS2DE12 accelerometer driver generated from `lis2de12.yaml` during the build process.
//! Provides register accessors and implements the [`RawAccelerometer`] trait from the `accelerometer` crate.

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

use accelerometer::vector::I16x3;
use accelerometer::{Error, ErrorKind, RawAccelerometer};
#[cfg(feature = "async")]
use device_driver::{AsyncBufferInterface, AsyncRegisterInterface};
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal_async;
use hal::i2c::I2c;
#[cfg(feature = "async")]
use hal_async::i2c::I2c as AsyncI2c;

// RegisterInterface & BufferInterface implementations for blocking I²C

#[allow(unsafe_code)]
#[allow(missing_docs)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/lis2de12_device.rs"));
}

pub use generated::*;

/// I²C bus interface for LIS2DE12.
pub struct DeviceInterface<I2C> {
    /// Underlying I²C bus
    pub i2c: I2C,
    /// I²C slave address
    pub address: u8,
}

/// Asynchronous I²C bus interface for LIS2DE12.
#[cfg(feature = "async")]
pub struct DeviceInterfaceAsync<I2C> {
    /// Underlying asynchronous I²C bus
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
        // Write [address, data...]
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
}

/// Asynchronous LIS2DE12 driver built on an async I²C bus.
#[cfg(feature = "async")]
pub struct Lis2de12Async<I2C> {
    /// Underlying generated device implementation.
    device: Lis2de12Device<DeviceInterfaceAsync<I2C>>,
}

#[cfg(feature = "async")]
impl<I2C> Lis2de12Async<I2C>
where
    I2C: AsyncI2c,
{
    /// Create a new driver from an asynchronous I²C bus instance and selected address.
    pub async fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        let interface = DeviceInterfaceAsync {
            i2c,
            address: addr.addr(),
        };
        let mut device = Lis2de12Device::new(interface);
        let mut who_buf = [0u8; 1];
        device
            .interface
            .read_register(0x0F, 8, &mut who_buf)
            .await
            .map_err(Error::from)?;
        if who_buf[0] != 0x33 {
            ErrorKind::Device.err()?;
        }
        Ok(Self { device })
    }

    /// Consume the driver and return the inner asynchronous I²C bus.
    pub fn destroy(self) -> I2C {
        let Lis2de12Device { interface, .. } = self.device;
        interface.i2c
    }

    /// Read raw acceleration data asynchronously.
    pub async fn accel_raw(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.device
            .interface
            .read_register(0x28, 8, &mut buf)
            .await
            .map_err(Error::from)?;
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);
        Ok(I16x3::new(x, y, z))
    }

    /// Access the generated device API for advanced register operations.
    pub fn device(&mut self) -> &mut Lis2de12Device<DeviceInterfaceAsync<I2C>> {
        &mut self.device
    }
}

impl<I2C> RawAccelerometer<I16x3> for Lis2de12<I2C>
where
    I2C: I2c,
{
    type Error = I2C::Error;

    fn accel_raw(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.device
            .interface
            .read_register(0x28, 8, &mut buf)
            .map_err(Error::from)?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok(I16x3::new(x, y, z))
    }
}
