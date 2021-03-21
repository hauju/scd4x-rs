#[derive(Debug, Copy, Clone)]
pub enum Command {
    /// Reading out the serial number.
    GetSerial,
    /// Wake up sensor from sleep mode to idle mode.
    WakeUp,
    /// Reinitializes the sensor by reloading user settings from EEPROM.
    Reinit,
    /// Start periodic measurement
    StartPeriodicMeasurement,
    /// Stop periodic measurement and return to idle mode.
    StopPeriodicMeasurement,
    /// Read sensor output.
    ReadMeasurement,
}

impl Command {
    /// Command and the requested delay in ms
    pub fn as_tuple(self) -> (u16, u32) {
        match self {
            Command::GetSerial => (0x3682, 1),
            Command::WakeUp => (0x36F6, 20),
            Command::Reinit => (0x3646, 20),
            Command::StartPeriodicMeasurement => (0x21B1, 1),
            Command::StopPeriodicMeasurement => (0x3F86, 500),
            Command::ReadMeasurement => (0xEC05, 1),
        }
    }
}
