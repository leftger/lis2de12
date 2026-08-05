//! Integration tests for the LIS2DE12 driver using mock I²C and SPI buses.
#![allow(missing_docs)]

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use accelerometer::ErrorKind;
use lis2de12::{
    ActivityConfig, AxesEnable, ClickAxesConfig, ClickConfig, FIFO_CAPACITY, FIFO_FRAME_BYTES, FifoConfig, FifoMode,
    Fs, Int1Routing, Int2Routing, InterruptConfig, InterruptPolarity, LatchMode, Lis2de12, Lis2de12Config,
    MotionAxesConfig, MotionConfig, MotionDetectionMode, Odr, SlaveAddr,
};

// ============================================================================
// Register map / mock state
// ============================================================================

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL_REG0: u8 = 0x1E;
const REG_TEMP_CFG: u8 = 0x1F;
const REG_CTRL_REG1: u8 = 0x20;
const REG_CTRL_REG2: u8 = 0x21;
const REG_CTRL_REG3: u8 = 0x22;
const REG_CTRL_REG4: u8 = 0x23;
const REG_CTRL_REG5: u8 = 0x24;
const REG_CTRL_REG6: u8 = 0x25;
const REG_FIFO_READ: u8 = 0x28;
const REG_FIFO_CTRL: u8 = 0x2E;
const REG_FIFO_SRC: u8 = 0x2F;
const REG_INT1_CFG: u8 = 0x30;
const REG_INT1_SRC: u8 = 0x31;
const REG_INT1_THS: u8 = 0x32;
const REG_INT1_DUR: u8 = 0x33;
const REG_CLICK_CFG: u8 = 0x38;
const REG_CLICK_SRC: u8 = 0x39;
const REG_CLICK_THS: u8 = 0x3A;
const REG_ACT_THS: u8 = 0x3E;
const REG_ACT_DUR: u8 = 0x3F;
const REG_OUT_TEMP_L: u8 = 0x0C;
const REG_OUT_TEMP_H: u8 = 0x0D;

const WHO_AM_I_VALUE: u8 = 0x33;

#[derive(Debug)]
struct DummyError;

impl embedded_hal::i2c::Error for DummyError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal::spi::Error for DummyError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

struct MockState {
    registers: HashMap<u8, u8>,
    fifo_data: VecDeque<u8>,
    fifo_frames: u8,
    watermark: u8,
}

impl MockState {
    fn new() -> Self {
        let mut registers = HashMap::new();
        registers.insert(REG_WHO_AM_I, WHO_AM_I_VALUE);
        registers.insert(REG_CTRL_REG0, 0x10);
        registers.insert(REG_TEMP_CFG, 0x00);
        registers.insert(REG_CTRL_REG1, 0x07);
        registers.insert(REG_CTRL_REG2, 0x00);
        registers.insert(REG_CTRL_REG3, 0x00);
        registers.insert(REG_CTRL_REG4, 0x00);
        registers.insert(REG_CTRL_REG5, 0x00);
        registers.insert(REG_CTRL_REG6, 0x00);
        registers.insert(REG_FIFO_CTRL, 0x00);
        registers.insert(REG_FIFO_SRC, 0x20); // EMPTY
        registers.insert(REG_INT1_CFG, 0x00);
        registers.insert(REG_INT1_SRC, 0x00);
        registers.insert(REG_INT1_THS, 0x00);
        registers.insert(REG_INT1_DUR, 0x00);
        registers.insert(REG_CLICK_CFG, 0x00);
        registers.insert(REG_CLICK_SRC, 0x00);
        registers.insert(REG_CLICK_THS, 0x00);
        registers.insert(REG_ACT_THS, 0x00);
        registers.insert(REG_ACT_DUR, 0x00);
        registers.insert(REG_OUT_TEMP_L, 0x00);
        registers.insert(REG_OUT_TEMP_H, 0x00);

        Self {
            registers,
            fifo_data: VecDeque::new(),
            fifo_frames: 0,
            watermark: 0,
        }
    }

    fn update_fifo_src(&mut self) {
        let empty = self.fifo_frames == 0;
        let overrun = self.fifo_frames >= FIFO_CAPACITY;
        let fss = if overrun { 0 } else { self.fifo_frames.min(31) };
        let wtm = self.watermark > 0 && self.fifo_frames >= self.watermark;

        let mut value = fss & 0x1F;
        if empty {
            value |= 1 << 5;
        }
        if overrun {
            value |= 1 << 6;
        }
        if wtm {
            value |= 1 << 7;
        }
        self.registers.insert(REG_FIFO_SRC, value);
    }

    fn write_bytes(&mut self, start: u8, data: &[u8]) {
        for (i, &value) in data.iter().enumerate() {
            let addr = start.wrapping_add(i as u8);
            self.registers.insert(addr, value);
            if addr == REG_FIFO_CTRL {
                self.watermark = value & 0x1F;
                self.update_fifo_src();
            }
        }
    }

    fn read_bytes(&mut self, start: u8, buf: &mut [u8]) {
        if start == REG_FIFO_READ {
            for byte in buf.iter_mut() {
                *byte = self.fifo_data.pop_front().unwrap_or(0);
            }
            let frames_read = (buf.len() / FIFO_FRAME_BYTES) as u8;
            if frames_read > 0 {
                self.fifo_frames = self.fifo_frames.saturating_sub(frames_read);
            }
            self.update_fifo_src();
            return;
        }

        for (i, byte) in buf.iter_mut().enumerate() {
            let addr = start.wrapping_add(i as u8);
            *byte = self.registers.get(&addr).copied().unwrap_or(0);
        }
    }

    /// Inject one XYZ sample as a 6-byte FIFO frame `[pad, Xh, pad, Yh, pad, Zh]`.
    fn inject_fifo_sample(&mut self, x: i8, y: i8, z: i8) {
        self.fifo_data.push_back(0x00);
        self.fifo_data.push_back(x as u8);
        self.fifo_data.push_back(0x00);
        self.fifo_data.push_back(y as u8);
        self.fifo_data.push_back(0x00);
        self.fifo_data.push_back(z as u8);
        if self.fifo_frames < FIFO_CAPACITY {
            self.fifo_frames += 1;
        }
        self.update_fifo_src();
    }

    fn set_temperature_raw(&mut self, raw: i16) {
        let bytes = raw.to_le_bytes();
        self.registers.insert(REG_OUT_TEMP_L, bytes[0]);
        self.registers.insert(REG_OUT_TEMP_H, bytes[1]);
    }
}

// ============================================================================
// Dummy I²C
// ============================================================================

#[derive(Clone)]
struct DummyI2c {
    state: Rc<RefCell<MockState>>,
}

impl DummyI2c {
    fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(MockState::new())),
        }
    }

    fn with_who_am_i(who: u8) -> Self {
        let i2c = Self::new();
        i2c.state.borrow_mut().registers.insert(REG_WHO_AM_I, who);
        i2c
    }

    fn state(&self) -> Rc<RefCell<MockState>> {
        Rc::clone(&self.state)
    }
}

impl embedded_hal::i2c::ErrorType for DummyI2c {
    type Error = DummyError;
}

impl embedded_hal::i2c::I2c for DummyI2c {
    fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Support write_read via transaction if used.
        let mut pending_addr: Option<u8> = None;
        for op in operations {
            match op {
                embedded_hal::i2c::Operation::Write(write) => {
                    if write.is_empty() {
                        continue;
                    }
                    if write.len() == 1 {
                        pending_addr = Some(write[0]);
                    } else {
                        self.state.borrow_mut().write_bytes(write[0], &write[1..]);
                        pending_addr = None;
                    }
                }
                embedded_hal::i2c::Operation::Read(read) => {
                    let addr = pending_addr.take().unwrap_or(0);
                    self.state.borrow_mut().read_bytes(addr, read);
                }
            }
        }
        Ok(())
    }

    fn write(&mut self, _address: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }
        self.state.borrow_mut().write_bytes(data[0], &data[1..]);
        Ok(())
    }

    fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        buffer.fill(0);
        Ok(())
    }

    fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        if write.is_empty() {
            return Ok(());
        }
        self.state.borrow_mut().read_bytes(write[0], read);
        Ok(())
    }
}

// ============================================================================
// Dummy SPI
// ============================================================================

#[derive(Clone)]
struct DummySpi {
    state: Rc<RefCell<MockState>>,
}

impl DummySpi {
    fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(MockState::new())),
        }
    }

    fn with_who_am_i(who: u8) -> Self {
        let spi = Self::new();
        spi.state.borrow_mut().registers.insert(REG_WHO_AM_I, who);
        spi
    }

    fn state(&self) -> Rc<RefCell<MockState>> {
        Rc::clone(&self.state)
    }
}

impl embedded_hal::spi::ErrorType for DummySpi {
    type Error = DummyError;
}

impl embedded_hal::spi::SpiDevice for DummySpi {
    fn transaction(&mut self, operations: &mut [embedded_hal::spi::Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut pending_addr: Option<u8> = None;
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Write(write) => {
                    if write.is_empty() {
                        continue;
                    }
                    let cmd = write[0];
                    if write.len() == 1 {
                        // Address-only write preceding a Read in transaction.
                        pending_addr = Some(cmd);
                    } else if cmd & 0x80 == 0 {
                        // Register / buffer write (bit7=0); bit6 may be auto-inc.
                        let addr = cmd & 0x3F;
                        self.state.borrow_mut().write_bytes(addr, &write[1..]);
                        pending_addr = None;
                    } else {
                        pending_addr = Some(cmd);
                    }
                }
                embedded_hal::spi::Operation::Read(read) => {
                    let cmd = pending_addr.take().unwrap_or(0);
                    let addr = cmd & 0x3F;
                    self.state.borrow_mut().read_bytes(addr, read);
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    if write.is_empty() {
                        continue;
                    }
                    let addr = write[0] & 0x3F;
                    if write[0] & 0x80 != 0 {
                        self.state.borrow_mut().read_bytes(addr, read);
                    } else if write.len() > 1 {
                        self.state.borrow_mut().write_bytes(addr, &write[1..]);
                    }
                }
                embedded_hal::spi::Operation::TransferInPlace(buf) => {
                    if buf.is_empty() {
                        continue;
                    }
                    let cmd = buf[0];
                    let addr = cmd & 0x3F;
                    if cmd & 0x80 != 0 {
                        let mut data = vec![0u8; buf.len().saturating_sub(1)];
                        self.state.borrow_mut().read_bytes(addr, &mut data);
                        if !data.is_empty() {
                            buf[1..].copy_from_slice(&data);
                        }
                    }
                }
                embedded_hal::spi::Operation::DelayNs(_) => {}
            }
        }
        Ok(())
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        if buf.is_empty() {
            return Ok(());
        }
        let addr = buf[0] & 0x3F;
        self.state.borrow_mut().write_bytes(addr, &buf[1..]);
        Ok(())
    }
}

fn create_i2c_driver() -> (Lis2de12<lis2de12::DeviceInterface<DummyI2c>>, Rc<RefCell<MockState>>) {
    let i2c = DummyI2c::new();
    let state = i2c.state();
    let driver = Lis2de12::new_i2c(i2c, SlaveAddr::Default).expect("init should succeed");
    (driver, state)
}

fn create_spi_driver() -> (Lis2de12<lis2de12::SpiInterface<DummySpi>>, Rc<RefCell<MockState>>) {
    let spi = DummySpi::new();
    let state = spi.state();
    let driver = Lis2de12::new_spi(spi).expect("SPI init should succeed");
    (driver, state)
}

// ============================================================================
// Init / WHO_AM_I
// ============================================================================

#[test]
fn i2c_init_verifies_who_am_i() {
    let (driver, state) = create_i2c_driver();
    assert_eq!(
        state.borrow().registers.get(&REG_WHO_AM_I).copied(),
        Some(WHO_AM_I_VALUE)
    );
    // Default config applied: ODR=100Hz, LPEN, XYZ enable → 0x5F
    assert_eq!(state.borrow().registers.get(&REG_CTRL_REG1).copied(), Some(0x5F));
    let _ = driver;
}

#[test]
fn i2c_init_rejects_wrong_who_am_i() {
    let i2c = DummyI2c::with_who_am_i(0x00);
    match Lis2de12::new_i2c(i2c, SlaveAddr::Default) {
        Err(err) => assert_eq!(err.kind(), ErrorKind::Device),
        Ok(_) => panic!("expected WHO_AM_I mismatch"),
    }
}

#[test]
fn i2c_alternative_address_init() {
    let i2c = DummyI2c::new();
    let driver = Lis2de12::new_i2c(i2c, SlaveAddr::Alternative).unwrap();
    assert_eq!(driver.destroy().state.borrow().registers[&REG_WHO_AM_I], WHO_AM_I_VALUE);
}

#[test]
fn spi_init_verifies_who_am_i() {
    let (driver, state) = create_spi_driver();
    assert_eq!(
        state.borrow().registers.get(&REG_WHO_AM_I).copied(),
        Some(WHO_AM_I_VALUE)
    );
    assert_eq!(state.borrow().registers.get(&REG_CTRL_REG1).copied(), Some(0x5F));
    let _ = driver;
}

#[test]
fn spi_init_rejects_wrong_who_am_i() {
    let spi = DummySpi::with_who_am_i(0xFF);
    match Lis2de12::new_spi(spi) {
        Err(err) => assert_eq!(err.kind(), ErrorKind::Device),
        Ok(_) => panic!("expected WHO_AM_I mismatch"),
    }
}

// ============================================================================
// Configuration readback
// ============================================================================

#[test]
fn set_config_writes_ctrl_and_fifo_registers() {
    let (mut driver, state) = create_i2c_driver();

    let config = Lis2de12Config {
        odr: Odr::FourHundredHz,
        scale: Fs::PlusMinus8G,
        axes: AxesEnable {
            x: true,
            y: false,
            z: true,
        },
        block_data_update: true,
        temperature_enable: true,
        fifo: FifoConfig::enabled(FifoMode::Stream).with_watermark(10),
        ..Default::default()
    };
    driver.set_config(config).unwrap();

    let regs = state.borrow();
    // ODR=0b0111, LPEN=1, ZEN=1, YEN=0, XEN=1 → 0b0111_1101 = 0x7D
    assert_eq!(regs.registers.get(&REG_CTRL_REG1).copied(), Some(0x7D));
    // BDU=1, FS=±8g (0b10) → 0b1000_0000 | 0b0010_0000 = 0xA0
    assert_eq!(regs.registers.get(&REG_CTRL_REG4).copied(), Some(0xA0));
    // FIFO_EN bit6
    assert_eq!(regs.registers.get(&REG_CTRL_REG5).copied().unwrap() & 0x40, 0x40);
    // FM=Stream (0b10 << 6) | FTH=10 → 0x80 | 0x0A = 0x8A
    assert_eq!(regs.registers.get(&REG_FIFO_CTRL).copied(), Some(0x8A));
    // TEMP_EN = 0b11 << 6 = 0xC0
    assert_eq!(regs.registers.get(&REG_TEMP_CFG).copied(), Some(0xC0));
}

#[test]
fn config_accessor_returns_active_settings() {
    let (mut driver, _) = create_i2c_driver();
    let config = Lis2de12Config {
        odr: Odr::FiftyHz,
        scale: Fs::PlusMinus4G,
        ..Default::default()
    };
    driver.set_config(config).unwrap();
    assert_eq!(driver.config().odr, Odr::FiftyHz);
    assert_eq!(driver.config().scale, Fs::PlusMinus4G);
}

// ============================================================================
// Acceleration / FIFO reads
// ============================================================================

#[test]
fn read_raw_decodes_fifo_frame() {
    let (mut driver, state) = create_i2c_driver();
    state.borrow_mut().inject_fifo_sample(10, -20, 30);

    let raw = driver.read_raw().unwrap();
    assert_eq!(raw.x, 10);
    assert_eq!(raw.y, -20);
    assert_eq!(raw.z, 30);
}

#[test]
fn read_mg_applies_default_sensitivity() {
    let (mut driver, state) = create_i2c_driver();
    // ±2g → 15.6 mg/LSB; raw=1 → ~16 mg after rounding
    state.borrow_mut().inject_fifo_sample(1, 0, 0);

    let mg = driver.read_mg().unwrap();
    assert_eq!(mg.x, 16);
    assert_eq!(mg.y, 0);
    assert_eq!(mg.z, 0);
}

#[test]
fn read_g_applies_default_sensitivity() {
    let (mut driver, state) = create_i2c_driver();
    state.borrow_mut().inject_fifo_sample(2, -4, 0);

    let g = driver.read_g().unwrap();
    assert!((g.x - 0.0312).abs() < 1e-4);
    assert!((g.y + 0.0624).abs() < 1e-4);
    assert!(g.z.abs() < 1e-6);
}

#[test]
fn fifo_status_reflects_injected_frames() {
    let (mut driver, state) = create_i2c_driver();
    {
        let mut s = state.borrow_mut();
        s.watermark = 2;
        s.inject_fifo_sample(1, 2, 3);
        s.inject_fifo_sample(4, 5, 6);
        s.inject_fifo_sample(7, 8, 9);
    }

    let status = driver.fifo_status().unwrap();
    assert!(!status.is_empty());
    assert_eq!(status.len(), 3);
    assert!(status.is_watermark_triggered());
}

#[test]
fn read_fifo_frames_and_drain() {
    let (mut driver, state) = create_i2c_driver();
    {
        let mut s = state.borrow_mut();
        s.inject_fifo_sample(1, 2, 3);
        s.inject_fifo_sample(4, 5, 6);
    }

    let mut frames = [[0u8; FIFO_FRAME_BYTES]; 4];
    let n = driver.read_fifo_frames(&mut frames).unwrap();
    assert_eq!(n, 2);
    assert_eq!(frames[0][1], 1);
    assert_eq!(frames[1][1], 4);

    // FIFO should now be empty
    let status = driver.fifo_status().unwrap();
    assert!(status.is_empty());

    // Re-inject and drain
    state.borrow_mut().inject_fifo_sample(9, 8, 7);
    state.borrow_mut().inject_fifo_sample(6, 5, 4);
    let drained = driver.drain_fifo().unwrap();
    assert_eq!(drained, 2);
    assert!(driver.fifo_status().unwrap().is_empty());
}

#[test]
fn spi_read_raw_works() {
    let (mut driver, state) = create_spi_driver();
    state.borrow_mut().inject_fifo_sample(-1, 2, -3);

    let raw = driver.read_raw().unwrap();
    assert_eq!(raw.x, -1);
    assert_eq!(raw.y, 2);
    assert_eq!(raw.z, -3);
}

// ============================================================================
// Temperature / misc
// ============================================================================

#[test]
fn temperature_enable_and_read() {
    let (mut driver, state) = create_i2c_driver();
    driver.set_temperature_sensor(true).unwrap();
    assert_eq!(state.borrow().registers.get(&REG_TEMP_CFG).copied(), Some(0xC0));

    // Left-justified: +5 °C → 0x0500
    state.borrow_mut().set_temperature_raw(0x0500);
    assert_eq!(driver.read_temperature_raw().unwrap(), 0x0500);
    assert!((driver.read_temperature().unwrap() - 5.0).abs() < f32::EPSILON);

    state.borrow_mut().set_temperature_raw(-0x0300);
    assert!((driver.read_temperature().unwrap() + 3.0).abs() < f32::EPSILON);
}

#[test]
fn sdo_pullup_and_high_pass_roundtrip() {
    let (mut driver, state) = create_i2c_driver();

    assert!(driver.sdo_pullup_connected().unwrap());
    driver.set_sdo_pullup_connected(false).unwrap();
    assert!(!driver.sdo_pullup_connected().unwrap());
    assert_eq!(
        state.borrow().registers.get(&REG_CTRL_REG0).copied().unwrap() & 0x80,
        0x80
    );

    assert!(!driver.high_pass_to_outputs().unwrap());
    driver.set_high_pass_to_outputs(true).unwrap();
    assert!(driver.high_pass_to_outputs().unwrap());
    // FDS is bit 3 of CTRL_REG2
    assert_eq!(
        state.borrow().registers.get(&REG_CTRL_REG2).copied().unwrap() & 0x08,
        0x08
    );
}

#[test]
fn reboot_sets_boot_bit() {
    let (mut driver, state) = create_i2c_driver();
    driver.reboot().unwrap();
    assert_eq!(
        state.borrow().registers.get(&REG_CTRL_REG5).copied().unwrap() & 0x80,
        0x80
    );
}

// ============================================================================
// Motion / click / interrupts
// ============================================================================

#[test]
fn motion1_config_writes_expected_registers() {
    let (mut driver, state) = create_i2c_driver();

    let config = MotionConfig::disabled()
        .with_enable(true)
        .with_threshold(40)
        .with_duration(5)
        .with_latch(LatchMode::Latched)
        .with_mode(MotionDetectionMode::AndCombination)
        .with_axes(MotionAxesConfig::all_high())
        .with_high_pass_filter(true);

    driver.set_motion1_config(config).unwrap();

    let regs = state.borrow();
    // AOI=1, XHIE/YHIE/ZHIE → 0b1000_0000 | 0b0010_1010 = 0xAA
    assert_eq!(regs.registers.get(&REG_INT1_CFG).copied(), Some(0xAA));
    assert_eq!(regs.registers.get(&REG_INT1_THS).copied(), Some(40));
    assert_eq!(regs.registers.get(&REG_INT1_DUR).copied(), Some(5));
    // LIR_INT1 bit5
    assert_eq!(regs.registers.get(&REG_CTRL_REG5).copied().unwrap() & 0x20, 0x20);
    // HP_IA1 bit0
    assert_eq!(regs.registers.get(&REG_CTRL_REG2).copied().unwrap() & 0x01, 0x01);
}

#[test]
fn motion1_status_parses_injected_source() {
    let (mut driver, state) = create_i2c_driver();
    // IA + XH = bits 6 and 1 → 0x42
    state.borrow_mut().registers.insert(REG_INT1_SRC, 0x42);

    let status = driver.motion1_status().unwrap();
    assert!(status.is_active());
    assert!(status.x_high);
    assert!(!status.y_high);
    assert!(status.x_event());
}

#[test]
fn click_config_and_status() {
    let (mut driver, state) = create_i2c_driver();

    let config = ClickConfig::disabled()
        .with_enable(true)
        .with_threshold(30)
        .with_time_limit(8)
        .with_time_latency(16)
        .with_time_window(32)
        .with_latch(LatchMode::Latched)
        .with_axes(ClickAxesConfig::all_single());

    driver.set_click_config(config).unwrap();

    {
        let regs = state.borrow();
        // XS/YS/ZS = bits 0,2,4 → 0x15
        assert_eq!(regs.registers.get(&REG_CLICK_CFG).copied(), Some(0x15));
        // LIR_Click bit7 | THS=30 → 0x80 | 30 = 0x9E
        assert_eq!(regs.registers.get(&REG_CLICK_THS).copied(), Some(0x9E));
    }

    // IA + SClick + Z = bits 6,4,2 → 0x54
    state.borrow_mut().registers.insert(REG_CLICK_SRC, 0x54);
    let status = driver.click_status().unwrap();
    assert!(status.is_active());
    assert!(status.is_single_click());
    assert!(status.z);
}

#[test]
fn activity_and_interrupt_routing() {
    let (mut driver, state) = create_i2c_driver();

    driver
        .set_activity_config(
            ActivityConfig::disabled()
                .with_enable(true)
                .with_threshold(25)
                .with_duration(10),
        )
        .unwrap();
    assert_eq!(state.borrow().registers.get(&REG_ACT_THS).copied(), Some(25));
    assert_eq!(state.borrow().registers.get(&REG_ACT_DUR).copied(), Some(10));

    let irq = InterruptConfig::disabled()
        .with_polarity(InterruptPolarity::ActiveLow)
        .with_int1(Int1Routing {
            ia1: true,
            data_ready: true,
            ..Default::default()
        })
        .with_int2(Int2Routing {
            activity: true,
            ..Default::default()
        });
    driver.set_interrupt_config(irq).unwrap();

    let regs = state.borrow();
    // I1_IA1 bit6 | I1_ZYXDA bit4 → 0x50
    assert_eq!(regs.registers.get(&REG_CTRL_REG3).copied(), Some(0x50));
    // I2_ACT bit3 | INT_POLARITY bit1 → 0x0A
    assert_eq!(regs.registers.get(&REG_CTRL_REG6).copied(), Some(0x0A));
}

#[test]
fn interrupt_sources_aggregates_status() {
    let (mut driver, state) = create_i2c_driver();
    {
        let mut s = state.borrow_mut();
        s.registers.insert(REG_INT1_SRC, 0x40); // IA
        s.registers.insert(0x35, 0x00); // INT2_SRC inactive
        s.registers.insert(REG_CLICK_SRC, 0x40); // click IA
    }

    let sources = driver.interrupt_sources().unwrap();
    assert!(sources.motion1.is_active());
    assert!(!sources.motion2.is_active());
    assert!(sources.click.is_active());
    assert!(sources.any_active());
}

// ============================================================================
// End-to-end workflow
// ============================================================================

#[test]
fn typical_app_workflow() {
    let i2c = DummyI2c::new();
    let state = i2c.state();

    let config = Lis2de12Config {
        odr: Odr::HundredHz,
        scale: Fs::PlusMinus2G,
        fifo: FifoConfig::enabled(FifoMode::Fifo).with_watermark(1),
        ..Default::default()
    };
    let mut driver = Lis2de12::new_i2c_with_config(i2c, SlaveAddr::Default, config).unwrap();

    assert_eq!(state.borrow().registers.get(&REG_CTRL_REG1).copied(), Some(0x5F));
    assert_eq!(
        state.borrow().registers.get(&REG_FIFO_CTRL).copied().unwrap() & 0xC0,
        0x40 // FIFO mode
    );

    state.borrow_mut().inject_fifo_sample(0, 0, 64); // ~1g on Z at ±2g (64*15.6≈998mg)
    let mg = driver.read_mg().unwrap();
    assert_eq!(mg.x, 0);
    assert_eq!(mg.y, 0);
    assert!((mg.z - 998).abs() <= 2);

    driver.set_temperature_sensor(true).unwrap();
    state.borrow_mut().set_temperature_raw(0x0200);
    assert!((driver.read_temperature().unwrap() - 2.0).abs() < f32::EPSILON);

    let _bus = driver.destroy();
}

// ============================================================================
// Async smoke
// ============================================================================

#[cfg(feature = "async")]
mod async_tests {
    use futures::executor::block_on;
    use lis2de12::Lis2de12Async;

    use super::*;

    impl embedded_hal_async::i2c::I2c for DummyI2c {
        async fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal_async::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            let mut pending_addr: Option<u8> = None;
            for op in operations.iter_mut() {
                match op {
                    embedded_hal_async::i2c::Operation::Write(write) => {
                        if write.is_empty() {
                            continue;
                        }
                        if write.len() == 1 {
                            pending_addr = Some(write[0]);
                        } else {
                            self.state.borrow_mut().write_bytes(write[0], &write[1..]);
                            pending_addr = None;
                        }
                    }
                    embedded_hal_async::i2c::Operation::Read(read) => {
                        let addr = pending_addr.take().unwrap_or(0);
                        self.state.borrow_mut().read_bytes(addr, read);
                    }
                }
            }
            Ok(())
        }

        async fn write(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error> {
            embedded_hal::i2c::I2c::write(self, address, data)
        }

        async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            embedded_hal::i2c::I2c::read(self, address, buffer)
        }

        async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            embedded_hal::i2c::I2c::write_read(self, address, write, read)
        }
    }

    #[test]
    fn async_i2c_init_and_read() {
        block_on(async {
            let i2c = DummyI2c::new();
            let state = i2c.state();
            let mut driver = Lis2de12Async::new_i2c(i2c, SlaveAddr::Default)
                .await
                .expect("async init");

            assert_eq!(state.borrow().registers.get(&REG_CTRL_REG1).copied(), Some(0x5F));

            state.borrow_mut().inject_fifo_sample(3, 4, 5);
            let raw = driver.read_raw().await.unwrap();
            assert_eq!(raw.x, 3);
            assert_eq!(raw.y, 4);
            assert_eq!(raw.z, 5);
        });
    }
}
