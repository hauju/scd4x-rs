//! This library provides an embedded `no_std` driver for the [Sensirion SCD4x series](https://www.sensirion.com/de/umweltsensoren/evaluationskit-sek-environmental-sensing/evaluationskit-sek-scd41/).
//! This driver was built using [`embedded-hal``](https://docs.rs/embedded-hal/) traits.
//! The implementaion are based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).
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
//! scd4x = { version = "0.3", features = ["embedded-hal-async"] }
//! ```

#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

/// Log functions for internal use, use `crate::log::*`
mod log {
    // Default to logging with log
    #[cfg(not(feature = "defmt"))]
    pub use log::{debug, error, info, trace, warn};

    // Replace with defmt if enabled
    #[cfg(feature = "defmt")]
    pub use defmt::{debug, error, info, trace, warn};
}

mod scd4x;
pub use crate::scd4x::Scd4x;

#[cfg(feature = "embedded-hal-async")]
pub use crate::scd4x::Scd4xAsync;

mod error;
pub use error::Error;

pub mod commands;

pub mod types;
