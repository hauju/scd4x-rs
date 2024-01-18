
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
    Internal
}
