//! This library provides an embedded `no_std` driver for the [Sensirion SCD4x series](https://www.sensirion.com/de/umweltsensoren/evaluationskit-sek-environmental-sensing/evaluationskit-sek-scd41/).
//! This driver was built using [embedded-hal](https://docs.rs/embedded-hal/) traits.
//! The implementaion are based on [embedded-i2c-scd4x](https://github.com/Sensirion/embedded-i2c-scd4x) and [sgpc3-rs](https://github.com/mjaakkol/sgpc3-rs).
//!

#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

/// Log functions for internal use, use `crate::log::*`
mod log {
    // Default to logging with log
    #[cfg(not(feature = "defmt"))]
    pub use log::{trace, debug, info, warn, error};

    // Replace with defmt if enabled
    #[cfg(feature = "defmt")]
    pub use defmt::{trace, debug, info, warn, error};
}


mod scd4x;
pub use crate::scd4x::Scd4x;

mod error;
pub use error::Error;

pub mod commands;

pub mod types;
