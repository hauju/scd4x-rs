//! This library provides an embedded `no_std` driver for the [Sensirion SCD4x series](https://www.sensirion.com/de/umweltsensoren/evaluationskit-sek-environmental-sensing/evaluationskit-sek-scd41/).
//! This driver was built using [embedded-hal](https://docs.rs/embedded-hal/) traits.
//! The implementaion are based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).
//!

#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

pub mod commands;
pub mod error;
pub mod scd4x;
pub mod types;
