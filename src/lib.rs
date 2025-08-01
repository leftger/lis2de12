

//! Platform-agnostic LIS2DE12 accelerometer driver generated from `lis2de12.yaml`.
//! Implements the [`Accelerometer`] and [`RawAccelerometer`] traits from the `accelerometer` crate.

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

use device_driver::create_device;
use embedded_hal as hal;
use hal::i2c::I2c;
use accelerometer::{Accelerometer, Error, ErrorKind, RawAccelerometer};
use accelerometer::vector::{I16x3, F32x3};
use device_driver::{RegisterInterface, BufferInterfaceError, BufferInterface};

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
    type Error = I2C::Error;
    type AddressType = u8;

    fn read(&mut self, address: Self::AddressType, buf: &mut [u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        self.i2c.write_read(address, &[address], buf)?;
        Ok(buf.len())
    }

    fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        data[1..=buf.len()].copy_from_slice(buf);
        self.i2c.write(address, &data[..=buf.len()])?;
        Ok(buf.len())
    }

    fn flush(&mut self, _address: Self::AddressType) -> Result<(), <Self as RegisterInterface>::Error> {
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
        let interface = DeviceInterface { i2c, address: addr.addr() };
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

impl<I2C> RawAccelerometer<I16x3> for Lis2de12<I2C>
where
    I2C: I2c,
{
    type Error = Error<I2C::Error>;

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

impl<I2C: I2c> RawAccelerometer<I16x3> for Lis2de12<I2C> {
    type Error = I2C::Error;

    /// Get acceleration reading from the accelerometer
    fn accel_raw(&mut self) -> Result<I16x3, Error<Self::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_L, &mut buf)?;

        Ok(I16x3::new(
            (u16(buf[0]) + (u16(buf[1]) << 8)) as i16,
            (u16(buf[2]) + (u16(buf[3]) << 8)) as i16,
            (u16(buf[4]) + (u16(buf[5]) << 8)) as i16,
        ))
    }
}

#[cfg(feature = "out_f32")]
impl<I2C: I2c> Accelerometer for Lis2de12<I2C> {
    type Error = I2C::Error;

    /// Get normalized ±g reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        let acc_raw: I16x3 = self.accel_raw()?;

        Ok(F32x3::new(
            self.fs.convert_out_i16tof32(acc_raw.x),
            self.fs.convert_out_i16tof32(acc_raw.y),
            self.fs.convert_out_i16tof32(acc_raw.z),
        ))
    }

    /// Get sample rate of accelerometer in Hz
    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        let creg1 = self.read_reg(Register::CTRL_REG1)?;
        let rate = match FromPrimitive::from_u8(creg1 >> 4) {
            Some(Odr::PowerDown) => 0.0,
            Some(Odr::Hz1) => 1.0,
            Some(Odr::Hz10) => 10.0,
            Some(Odr::Hz25) => 25.0,
            Some(Odr::Hz50) => 50.0,
            Some(Odr::Hz100) => 100.0,
            Some(Odr::Hz200) => 200.0,
            Some(Odr::Hz400) => 400.0,
            Some(Odr::HighRate0) => 1620.0,
            Some(Odr::HighRate1) => {
                if creg1 & LPen == 0 {
                    1344.0
                } else {
                    5376.0
                }
            }
            None => 0.0,
        };
        Ok(rate)
    }
}