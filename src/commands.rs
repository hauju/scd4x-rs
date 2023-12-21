#[derive(Debug, Copy, Clone)]
/// List of SCD4x sensor commands.
pub enum Command {
    /// Start periodic measurement, signal update interval is 5 seconds.
    StartPeriodicMeasurement,
    /// Read sensor output. The measurement data can only be read out
    /// once per signal update interval as the buffer is emptied upon read-out.
    ReadMeasurement,
    /// Stop periodic measurement and return to idle mode for sensor configuration or to safe energy.
    StopPeriodicMeasurement,
    /// The temperature offset has no influence on the SCD4x CO2 accuracy. Setting the
    /// temperature offset of the SCD4x inside the customer device correctly allows
    /// the user to leverage the RH and T output signal.
    SetTemperatureOffset,
    /// Get current temperature offset.
    GetTemperatureOffset,
    /// Set the height above sea level. Should be done while the sensor is in idle mode.
    SetSensorAltitude,
    /// Get the height above sea level.
    GetSensorAltitude,
    /// Set ambient pressure.
    SetAmbientPressure,
    /// Perform forced recalibration
    PerformForcedRecalibration,
    /// Set the current state (enabled / disabled) of the automatic self-calibration.
    /// By default, ASC is enabled.
    SetAutomaticSelfCalibrationEnabled,
    /// Get the current state of the automatic self-calibration.
    GetAutomaticSelfCalibrationEnabled,
    /// Start low power periodic measurement, signal update interval is approximately 30 seconds.
    StartLowPowerPeriodicMeasurement,
    /// Is data ready for read-out?
    GetDataReadyStatus,
    /// Save to non-volatile memory configuration settings
    /// such as the temperature offset, sensor altitude and the ASC enabled/disabled parameter.
    PersistSettings,
    /// Reading out the serial number can be used to identify the chip and to verify the presence of the sensor.
    GetSerialNumber,
    /// This feature can be used as an end-of-line test to check sensor functionality
    /// and the customer power supply to the sensor.  
    PerformSelfTest,
    /// Resets all configuration settings stored in the EEPROM and erases the
    /// FRC and ASC algorithm history.
    PerformFactoryReset,
    /// Reinitializes the sensor by reloading user settings from EEPROM.
    Reinit,
    /// Allows on-demand measurements
    #[cfg(feature = "scd41")]
    MeasureSingleShot,
    /// Allows on-demand, non-blocking measurements
    #[cfg(feature = "scd41")]
    MeasureSingleShotNonBlocking,
    /// On-demand measurement of CO2 concentration, relative humidity and temperature.
    #[cfg(feature = "scd41")]
    MeasureSingleShotRhtOnly,
    /// Put the sensor from idle to sleep mode to reduce current consumption.
    #[cfg(feature = "scd41")]
    PowerDown,
    /// Wake up sensor from sleep mode to idle mode.
    #[cfg(feature = "scd41")]
    WakeUp,
}

impl Command {
    // Command, execution time ms, possibility to execute during measurements.
    pub fn as_tuple(self) -> (u16, u32, bool) {
        match self {
            Self::StartPeriodicMeasurement => (0x21B1, 1, false),
            Self::ReadMeasurement => (0xEC05, 1, true),
            Self::StopPeriodicMeasurement => (0x3F86, 500, true),
            Self::SetTemperatureOffset => (0x241D, 1, false),
            Self::GetTemperatureOffset => (0x2318, 1, false),
            Self::SetSensorAltitude => (0x2427, 1, false),
            Self::GetSensorAltitude => (0x2322, 1, false),
            Self::SetAmbientPressure => (0xE000, 1, true),
            Self::PerformForcedRecalibration => (0x362F, 400, false),
            Self::SetAutomaticSelfCalibrationEnabled => (0x2416, 1, false),
            Self::GetAutomaticSelfCalibrationEnabled => (0x2313, 1, false),
            Self::StartLowPowerPeriodicMeasurement => (0x21AC, 0, false),
            Self::GetDataReadyStatus => (0xE4B8, 1, true),
            Self::PersistSettings => (0x3615, 800, false),
            Self::GetSerialNumber => (0x3682, 1, false),
            Self::PerformSelfTest => (0x3639, 10_000, false),
            Self::PerformFactoryReset => (0x3632, 1200, false),
            Self::Reinit => (0x3646, 20, false),
            #[cfg(feature = "scd41")]
            Self::MeasureSingleShot => (0x219D, 5000, false),
            #[cfg(feature = "scd41")]
            Self::MeasureSingleShotNonBlocking => (0x219D, 1, false),
            #[cfg(feature = "scd41")]
            Self::MeasureSingleShotRhtOnly => (0x2196, 50, false),
            #[cfg(feature = "scd41")]
            Self::PowerDown => (0x36E0, 1, false),
            #[cfg(feature = "scd41")]
            Self::WakeUp => (0x36F6, 20, false),
        }
    }
}
