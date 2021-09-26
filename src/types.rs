//! Types for the SCD4x sensor.

pub struct SensorData {
    pub co2: u16,
    pub temperature: f32,
    pub humidity: f32,
}

pub struct RawSensorData {
    pub co2: u16,
    pub temperature: u16,
    pub humidity: u16,
}