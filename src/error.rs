use embedded_hal as hal;
use hal::blocking::i2c::{Read, Write};
use sensirion_i2c::i2c;

/// Scd4x errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
    /// Self-test measure failure
    SelfTest,
    /// Not allowed when periodic measurement is running
    NotAllowed,
    /// Internal fail
    Internal
}

impl<E, I2cWrite, I2cRead> From<i2c::Error<I2cWrite, I2cRead>> for Error<E>
where
    I2cWrite: Write<Error = E>,
    I2cRead: Read<Error = E>,
{
    fn from(err: i2c::Error<I2cWrite, I2cRead>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}
