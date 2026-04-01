# Sensirion I2C SCD4x Driver

[![crates.io](https://img.shields.io/crates/v/scd4x.svg)](https://crates.io/crates/scd4x)
[![docs.rs](https://docs.rs/scd4x/badge.svg)](https://docs.rs/scd4x)
[![License](https://img.shields.io/crates/l/scd4x)](https://crates.io/crates/scd4x)
[![no_std](https://img.shields.io/badge/target-no__std-blue)](https://crates.io/crates/scd4x)

A platform-agnostic `no_std` Rust driver for the [Sensirion SCD4x](https://sensirion.com/products/catalog/SCD41/) CO2 sensor family (SCD40/SCD41), built on [embedded-hal](https://docs.rs/embedded-hal/) traits. Based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).

## Sensirion SCD4x

The SCD4x is a miniature CO2, temperature, and relative humidity sensor using photoacoustic NDIR sensing.

Further information: [Datasheet SCD4x](https://sensirion.com/media/documents/48C4B7FB/67FE0194/CD_DS_SCD4x_Datasheet_D1.pdf)

## Features

- Full SCD4x command set (periodic & single-shot measurements, calibration, configuration)
- SCD41-specific commands behind the `scd41` feature flag (single-shot, power down/wake up)
- Async support via the `embedded-hal-async` feature
- Optional `thiserror` integration for `std` environments
- Optional `defmt` support for embedded logging
- Blocking and non-blocking measurement modes

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
scd4x = "0.5"

# For SCD41-specific features (single-shot, power management):
# scd4x = { version = "0.5", features = ["scd41"] }

# For async support:
# scd4x = { version = "0.5", features = ["embedded-hal-async"] }
```

### Examples

See examples for [ESP32-C3](examples/esp32c3.rs) and [Linux](examples/linux.rs).

```bash
cargo run --example linux
```

Run the built-in CLI utility to read serial number and measurements:
```bash
cargo run --features="util"
```

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.