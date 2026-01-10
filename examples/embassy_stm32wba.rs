#![no_std]
#![no_main]
#![allow(missing_docs)]

use defmt::{Debug2Format, info, panic, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::Speed;
use embassy_stm32::i2c::{self, Error as I2cError, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use lis2de12::{AxesEnable, FifoConfig, FifoMode, Fs, Lis2de12Async, Lis2de12Config, Odr, OperatingMode, SlaveAddr};
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(feature = "embassy"))]
compile_error!("Enable the `embassy` feature to build this example.");

#[cfg(not(feature = "defmt-03"))]
compile_error!("Enable the `defmt-03` feature to build this example.");

#[cfg(not(target_os = "none"))]
compile_error!("Build this example for a bare-metal target (e.g. thumbv8m.main-none-eabihf).");

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
});

type SensorError = lis2de12::Error<I2cError>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();

    {
        use embassy_stm32::rcc::*;

        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,  // PLLM = 1  → HSI / 1 = 16 MHz
            mul: PllMul::MUL25,       // PLLN = 25 → 16 MHz * 25 = 400 MHz VCO
            divr: Some(PllDiv::DIV4), // PLLR = 4  → 100 MHz (Sysclk)
            divq: Some(PllDiv::DIV8), // PLLQ = 8  → 50 MHz
            divp: None,
            frac: Some(0),
        });

        config.rcc.hse = Some(Hse {
            prescaler: HsePrescaler::DIV1,
        });

        config.rcc.voltage_scale = VoltageScale::RANGE1;
        config.rcc.mux.otghssel = mux::Otghssel::HSE;
        config.rcc.mux.sai1sel = mux::Sai1sel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb7_pre = APBPrescaler::DIV1;
        config.rcc.ahb5_pre = AHB5Prescaler::DIV4;
    }

    let p = embassy_stm32::init(config);

    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.gpio_speed = Speed::VeryHigh;
    i2c_cfg.scl_pullup = false;
    i2c_cfg.sda_pullup = false;

    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, i2c_cfg);

    let mut driver = match Lis2de12Async::new(i2c, SlaveAddr::Default).await {
        Ok(driver) => driver,
        Err(err) => panic!("Failed to initialise LIS2DE12: {:?}", Debug2Format(&err)),
    };

    let mut sensor_cfg = Lis2de12Config::default();
    sensor_cfg.odr = Odr::HundredHz;
    sensor_cfg.scale = Fs::PlusMinus4G;
    sensor_cfg.mode = OperatingMode::Normal;
    sensor_cfg.axes = AxesEnable::default();
    sensor_cfg.fifo = FifoConfig::enabled(FifoMode::Stream).with_watermark(6);

    if let Err(err) = driver.set_config(sensor_cfg).await {
        panic!("Failed to configure LIS2DE12: {:?}", Debug2Format(&err));
    }

    if let Err(err) = driver.drain_fifo().await {
        log_error("drain_fifo", err);
    }

    info!(
        "[lis2de12] sensor ready (ODR={:?}, scale={:?})",
        Debug2Format(&sensor_cfg.odr),
        Debug2Format(&sensor_cfg.scale),
    );

    loop {
        match driver.read_mg().await {
            Ok(mg) => {
                info!("[lis2de12] accel [mg]: x={} y={} z={}", mg.x, mg.y, mg.z);
            }
            Err(err) => {
                log_error("read_mg", err);
                Timer::after_millis(50).await;
                continue;
            }
        }

        match driver.fifo_status().await {
            Ok(status) => {
                info!(
                    "[lis2de12] fifo frames={} watermark={} overrun={} empty={}",
                    status.len(),
                    status.is_watermark_triggered(),
                    status.is_overrun(),
                    status.is_empty()
                );
            }
            Err(err) => log_error("fifo_status", err),
        }

        Timer::after_millis(100).await;
    }
}

fn log_error(label: &str, err: SensorError) {
    warn!("[lis2de12] {} failed: {:?}", label, Debug2Format(&err));
}
