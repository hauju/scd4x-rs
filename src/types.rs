//! Types for the SCD4x sensor.

pub struct SensorData {
    pub co2: f32,
    pub temperature: f32,
    pub humidity: f32,
}
