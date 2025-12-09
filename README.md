# `lis2de12`

Platform-agnostic Rust driver for the ST LIS2DE12 ultra-low-power 3-axis accelerometer. The crate targets `no_std` environments, generates its register API at build time from a YAML manifest, and supports both blocking and asynchronous I²C transports.

---

## Features

- High-level blocking driver implementing `accelerometer::RawAccelerometer`
- Optional async driver (`Lis2de12Async`)
- Auto-generated register-level API via [`device-driver`](https://crates.io/crates/device-driver)
- `no_std` compatible with optional `defmt` logging integration

---

## Getting Started

Add the crate to your `Cargo.toml`:

```/dev/null/Cargo.toml#L1-12
[dependencies]
lis2de12 = "0.1"
embedded-hal = "1.0"

# Optional features:
# lis2de12 = { version = "0.1", features = ["async"] }
# lis2de12 = { version = "0.1", features = ["defmt-03"] }
```

---

## Blocking usage

```/dev/null/examples/blocking.rs#L1-30
use embedded_hal::i2c::blocking::I2c; // trait alias provided by your HAL
use lis2de12::{Lis2de12, SlaveAddr};

fn init_driver<I2C>(bus: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let mut lis = Lis2de12::new(bus, SlaveAddr::Default)?;
    let accel = lis.accel_raw()?;
    defmt::info!("Raw accel: {:?}", accel);
    Ok(())
}
```

---

## Configuring the sensor

Use `Lis2de12Config` to bring the device out of power-down with your preferred output data rate, operating mode, scale, axis enables, and temperature sensing:

```/dev/null/examples/config.rs#L1-27
use embedded_hal::i2c::blocking::I2c;
use lis2de12::{
    AxesEnable, Fs, Lis2de12, Lis2de12Config, OperatingMode, Odr, SlaveAddr,
};

fn init_with_config<I2C>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let config = Lis2de12Config {
        odr: Odr::FiftyHz,
        mode: OperatingMode::Normal,
        scale: Fs::PlusMinus4G,
        axes: AxesEnable { x: true, y: true, z: true },
        block_data_update: true,
        temperature_enable: true,
    };

    let mut lis = Lis2de12::new_with_config(i2c, SlaveAddr::Default, config)?;
    defmt::info!("Sensor configured with 50 Hz output");
    Ok(())
}
```

---

## Asynchronous usage (Embassy, RTIC, etc.)

Enable the `async` feature and depend on `embedded-hal-async`:

```/dev/null/examples/async.rs#L1-38
use embedded_hal_async::i2c::I2c;
use lis2de12::{Lis2de12Async, SlaveAddr};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let i2c_bus = acquire_async_i2c().await;
    let mut lis = Lis2de12Async::new(i2c_bus, SlaveAddr::Default)
        .await
        .expect("failed to init LIS2DE12");

    let accel = lis.accel_raw()
        .await
        .expect("failed to read acceleration");

    embassy_time::Delay.delay_ms(10).await;
    defmt::info!("Raw accel: {:?}", accel);
}
```

---

## Reading scaled samples

Both blocking and async drivers provide `read_raw`, `read_mg`, and `read_g` helpers to obtain measurements in the format that best suits your application:

```/dev/null/examples/scaled.rs#L1-24
use embedded_hal::i2c::blocking::I2c;
use lis2de12::{Lis2de12, SlaveAddr};

fn read_scaled<I2C>(bus: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let mut lis = Lis2de12::new(bus, SlaveAddr::Default)?;
    let raw = lis.read_raw()?;
    let mg = lis.read_mg()?;
    let g = lis.read_g()?;
    defmt::info!("raw={:?}, mg={:?}, g={:?}", raw, mg, g);
    Ok(())
}
```

The async variant exposes the same API surface, returning futures for each method.

---

## Register-level access

For advanced configuration you can interact with the generated register API directly:

```/dev/null/examples/registers.rs#L1-28
use lis2de12::{Lis2de12, Odr, SlaveAddr};

fn configure_normal_mode<I2C>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
{
    let mut lis = Lis2de12::new(i2c, SlaveAddr::Default)?;

    lis
        .device()
        .ctrl_reg_1()
        .write(|reg| {
            reg.set_odr(Odr::HundredHz);
            reg.set_lpen(false); // normal mode (low-power disabled)
            reg.set_xen(true);
            reg.set_yen(true);
            reg.set_zen(true);
        })?;
    Ok(())
}
```

---

## Feature flags

| Feature      | Default | Description                                             |
|--------------|---------|---------------------------------------------------------|
| `async`      | ✗       | Enables the async driver (pulls in `embedded-hal-async`) |
| `defmt-03`   | ✗       | Enables `defmt` logging support compatible with `device-driver` |

---

## Build-time code generation

During `cargo build`, the `build.rs` script reads `src/lis2de12.yaml`, generates the register accessor API, and writes it into `$OUT_DIR`. End users do not need to run any manual code-generation steps.

---

## Development

```/dev/null/CONTRIBUTING.md#L1-9
# Suggested commands
cargo fmt --all
cargo clippy --all-targets --all-features
cargo test --all-features
cargo publish --dry-run
```

CI workflows verify formatting, lints, and `no_std` builds.

---

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.

---

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the project by you shall be dual licensed as above (MIT OR Apache-2.0), without any additional terms or conditions.

---

## Links

- 📦 Crate: <https://crates.io/crates/lis2de12>
- 📚 Docs: <https://docs.rs/lis2de12>
- 🗺️ Repository: <https://github.com/leftger/lis2de12>
