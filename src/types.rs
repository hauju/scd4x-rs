//! Types for the SCD4x sensor.

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorData {
    pub co2: u16,
    pub temperature: f32,
    pub humidity: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSensorData {
    pub co2: u16,
    pub temperature: u16,
    pub humidity: u16,
}
