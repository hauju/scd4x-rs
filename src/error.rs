use embedded_hal as hal;
use sensirion_i2c::i2c;

/// SCD4X errors
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
pub enum Error<E> {
    #[cfg_attr(feature = "thiserror", error("I2C: {0}"))]
    /// I²C bus error
    I2c(E),
    #[cfg_attr(feature = "thiserror", error("CRC"))]
    /// CRC checksum validation failed
    Crc,
    #[cfg_attr(feature = "thiserror", error("Self Test"))]
    /// Self-test measure failure
    SelfTest,
    #[cfg_attr(feature = "thiserror", error("Not Allowed"))]
    /// Not allowed when periodic measurement is running
    NotAllowed,
    #[cfg_attr(feature = "thiserror", error("Internal"))]
    /// Internal fail
    Internal,
}

#[cfg(not(feature = "thiserror"))]
impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C: {e:?}"),
            Error::Crc => write!(f, "CRC"),
            Error::SelfTest => write!(f, "Self Test"),
            Error::NotAllowed => write!(f, "Not Allowed"),
            Error::Internal => write!(f, "Internal"),
        }
    }
}

impl<I2C> From<i2c::Error<I2C>> for Error<I2C::Error>
where
    I2C: hal::i2c::ErrorType,
{
    fn from(err: i2c::Error<I2C>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}
