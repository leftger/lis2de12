# `lis2de12`

Platform-agnostic Rust driver for the ST LIS2DE12 ultra-low-power 3-axis accelerometer. The crate targets `no_std` environments, generates its register API at build time from a YAML manifest, and supports both blocking and asynchronous I²C and SPI transports.

---

## Features

- **I²C and SPI support** - Use either interface with the same high-level API
- High-level blocking driver implementing `accelerometer::RawAccelerometer`
- Optional async driver (`Lis2de12Async`) for both I²C and SPI
- Auto-generated register-level API via [`device-driver`](https://crates.io/crates/device-driver)
- `no_std` compatible with optional `defmt` logging integration

---

## Getting Started

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
lis2de12 = "0.1.2"
embedded-hal = "1.0"

# Optional features:
# lis2de12 = { version = "0.1.2", features = ["async"] }
# lis2de12 = { version = "0.1.2", features = ["defmt-03"] }
```

---

## Blocking I²C usage

```rust
use embedded_hal::i2c::I2c;
use lis2de12::{Lis2de12, SlaveAddr};

fn init_driver<I2C>(bus: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    // Create driver using I²C interface
    let mut lis = Lis2de12::new_i2c(bus, SlaveAddr::default())?;
    let accel = lis.read_g()?;
    defmt::info!("Acceleration: {:?} g", accel);
    Ok(())
}
```

---

## Blocking SPI usage

```rust
use embedded_hal::spi::SpiDevice;
use lis2de12::Lis2de12;

fn init_driver_spi<SPI>(spi: SPI) -> Result<(), lis2de12::Error<SPI::Error>>
where
    SPI: SpiDevice,
{
    // Create driver using SPI interface
    let mut lis = Lis2de12::new_spi(spi)?;
    let accel = lis.read_g()?;
    defmt::info!("Acceleration: {:?} g", accel);
    Ok(())
}
```

**Note:** The LIS2DE12 requires SPI Mode 0 or Mode 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1). The driver uses the `SpiDevice` trait which handles chip select automatically.

---

## Configuring the sensor

Use `Lis2de12Config` to bring the device out of power-down with your preferred output data rate, operating mode, scale, axis enables, and temperature sensing:

```rust
use embedded_hal::i2c::I2c;
use lis2de12::{
    AxesEnable, Fs, Lis2de12, Lis2de12Config, OperatingMode, Odr, SlaveAddr,
};

fn init_with_config<I2C>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let config = Lis2de12Config {
        odr: Odr::Hz50,
        mode: OperatingMode::Normal,
        scale: Fs::G4,
        axes: AxesEnable { x: true, y: true, z: true },
        block_data_update: true,
        temperature_enable: true,
        ..Default::default()
    };

    let mut lis = Lis2de12::new_i2c_with_config(i2c, SlaveAddr::default(), config)?;
    // Or for SPI: Lis2de12::new_spi_with_config(spi, config)?
    defmt::info!("Sensor configured with 50 Hz output");
    Ok(())
}
```

---

## Asynchronous usage (Embassy, RTIC, etc.)

Enable the `async` feature and depend on `embedded-hal-async`:

```rust
use embedded_hal_async::i2c::I2c;
use lis2de12::{Lis2de12Async, SlaveAddr};

async fn init_async<I2C>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    // Async I²C
    let mut lis = Lis2de12Async::new_i2c(i2c, SlaveAddr::default()).await?;
    // Or async SPI: Lis2de12Async::new_spi(spi).await?

    let accel = lis.read_g().await?;
    defmt::info!("Acceleration: {:?} g", accel);
    Ok(())
}
```

---

## Reading scaled samples

Both blocking and async drivers provide `read_raw`, `read_mg`, and `read_g` helpers to obtain measurements in the format that best suits your application:

```rust
use embedded_hal::i2c::I2c;
use lis2de12::{Lis2de12, SlaveAddr};

fn read_scaled<I2C>(bus: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let mut lis = Lis2de12::new_i2c(bus, SlaveAddr::default())?;
    let raw = lis.read_raw()?;  // Raw ADC values
    let mg = lis.read_mg()?;    // milli-g (integer)
    let g = lis.read_g()?;      // g (floating point)
    defmt::info!("raw={:?}, mg={:?}, g={:?}", raw, mg, g);
    Ok(())
}
```

The async variant exposes the same API surface, returning futures for each method.

---

## FIFO configuration and burst reads

Configure the hardware FIFO to collect XYZ frames and drain them in bursts when the watermark is reached:

```rust
use embedded_hal::i2c::I2c;
use lis2de12::{
    FifoConfig, FifoMode, Lis2de12, Lis2de12Config, SlaveAddr,
};

fn read_fifo<I2C>(bus: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: I2c,
{
    let mut config = Lis2de12Config::default();
    config.fifo = FifoConfig::enabled(FifoMode::Stream).with_watermark(8);

    let mut lis = Lis2de12::new_i2c_with_config(bus, SlaveAddr::default(), config)?;
    let status = lis.fifo_status()?;
    if status.has_data() {
        let mut frames = [[0u8; lis2de12::FIFO_FRAME_BYTES]; 4];
        let read = lis.read_fifo_frames(&mut frames)?;
        defmt::info!("drained {} frames", read);
    }
    Ok(())
}
```

Async users get identical functionality via `Lis2de12Async::read_fifo_frame`, `read_fifo_frames`, and `drain_fifo`.

---

## Register-level access

For advanced configuration you can interact with the generated register API directly:

```rust
use lis2de12::{Lis2de12, Odr, SlaveAddr};

fn configure_normal_mode<I2C>(i2c: I2C) -> Result<(), lis2de12::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
{
    let mut lis = Lis2de12::new_i2c(i2c, SlaveAddr::default())?;

    lis
        .device()
        .ctrl_reg_1()
        .write(|reg| {
            reg.set_odr(Odr::Hz100);
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

```bash
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
