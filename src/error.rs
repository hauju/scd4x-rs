use embedded_hal as hal;
use hal::i2c::I2c;
use sensirion_i2c::i2c;

/// SCD4X errors
#[derive(Debug, PartialEq)]
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

impl<E, I2C> From<i2c::Error<I2C>> for Error<E>
where
    I2C: I2c<Error = E>,
{
    fn from(err: i2c::Error<I2C>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}
