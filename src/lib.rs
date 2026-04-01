//! A platform-agnostic `no_std` Rust driver for the [Sensirion SCD4x](https://sensirion.com/products/catalog/SCD41/) CO2 sensor family (SCD40/SCD41),
//! built on [`embedded-hal`](https://docs.rs/embedded-hal/) traits.
//! Based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).
//!
//! ## `embedded-hal-async` Support
//!
//! This crate has optional support for the [`embedded-hal-async`] crate. The
//! [`Scd4xAsync`] type provides a driver for a SCD4x sensor which uses
//! [`embedded-hal-async`]'s asynchronous versions of the `I2c` and `DelayNs`
//! traits, rather than the blocking versions from [`embedded-hal`].
//!
//! The [`embedded-hal-async`] support is feature flagged, so that users who
//! don't need the asynchronous versions of these traits don't have to depend on
//! `embedded-hal-async`. To use it, enable the `embedded-hal-async` feature
//! flag in your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! scd4x = { version = "0.5", features = ["embedded-hal-async"] }
//! ```

#![deny(unsafe_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]

mod scd4x;
pub use crate::scd4x::Scd4x;

#[cfg(feature = "embedded-hal-async")]
pub use crate::scd4x::Scd4xAsync;

mod error;
pub use error::Error;

pub mod commands;

pub mod types;
