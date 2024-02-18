# Sensirion I2C SCD4x Driver

This library provides an embedded `no_std` driver for the [Sensirion SCD4x series](https://www.sensirion.com/de/umweltsensoren/evaluationskit-sek-environmental-sensing/evaluationskit-sek-scd41/). This driver was built using [embedded-hal](https://docs.rs/embedded-hal/) traits. The implementaion are based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).

## Sensirion SCD4x

The SCD4x is a miniature carbon dioxide sensor. It also measure temperature and relative humidity.

Further information: [Datasheet CO2 Sensor SCD4x](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD4x_Datasheet.pdf)

## Usage

Run scd4x-util to read the serial number and the measurement output.
```bash
cargo run --features="util"
```

See an example using `linux-embedded-hal` in `examples/linux.rs`.
```bash
cargo run --example linux
```

## Development Status

The driver is in an early development state. It allows you to:
- Get the serial number.
- Read the measurement output.

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