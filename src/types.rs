//! Types for the SCD4x sensor.

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(not(feature = "fixed"))]
pub struct SensorData {
    pub co2: u16,
    pub temperature: f32,
    pub humidity: f32,
}

#[cfg(feature = "fixed")]
use fixed::types::{I16F16, U16F16};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(feature = "fixed")]
pub struct SensorData {
    pub co2: u16,
    pub temperature: I16F16,
    pub humidity: U16F16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSensorData {
    pub co2: u16,
    pub temperature: u16,
    pub humidity: u16,
}
