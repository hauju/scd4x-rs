use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::commands::Command;
use crate::error::Error;
use crate::types::{RawSensorData, SensorData};
use sensirion_i2c::{crc8, i2c};

const SCD4X_I2C_ADDRESS: u8 = 0x62;

/// SCD4X sensor instance. Use related methods to take measurements.
#[derive(Debug, Default)]
pub struct Scd4x<I2C, D> {
    i2c: I2C,
    delay: D,
    is_running: bool,
}

impl<I2C, D, E> Scd4x<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Scd4x {
            i2c,
            delay,
            is_running: false,
        }
    }

    /// Start periodic measurement, signal update interval is 5 seconds.
    /// This command is only available in idle mode.
    pub fn start_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StartPeriodicMeasurement)?;
        self.is_running = true;
        Ok(())
    }

    /// Stop periodic measurement and return to idle mode for sensor configuration or to safe energy.
    /// This command is only available in measurement mode.
    pub fn stop_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StopPeriodicMeasurement)?;
        self.is_running = false;
        Ok(())
    }

    /// Read raw sensor data
    pub fn sensor_output(&mut self) -> Result<RawSensorData, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        let co2 = u16::from_be_bytes([buf[0], buf[1]]);
        let temperature = u16::from_be_bytes([buf[3], buf[4]]);
        let humidity = u16::from_be_bytes([buf[6], buf[7]]);

        Ok(RawSensorData {
            co2,
            temperature,
            humidity,
        })
    }

    /// Read converted sensor data
    pub fn measurement(&mut self) -> Result<SensorData, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        // buf[2], buf[5], buf[8] is CRC bytes and not used
        let co2 = u16::from_be_bytes([buf[0], buf[1]]);
        let temperature = u16::from_be_bytes([buf[3], buf[4]]);
        let humidity = u16::from_be_bytes([buf[6], buf[7]]);

        Ok(SensorData {
            co2,
            temperature: temperature as f32 * 175_f32 / 65536_f32 - 45_f32,
            humidity: humidity as f32 * 100_f32 / 65536_f32,
        })
    }

    /// Get sensor temperature offset
    pub fn temperature_offset(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetTemperatureOffset, &mut buf)?;

        let raw_offset = u16::from_be_bytes([buf[0], buf[1]]);
        let offset = raw_offset as f32 * 175.0 / 65536.0;
        Ok(offset)
    }

    /// Set sensor temperature offset
    pub fn set_temperature_offset(&mut self, offset: f32) -> Result<(), Error<E>> {
        let t_offset = (offset * 65536.0 / 175.0) as i16;
        self.write_command_with_data(Command::SetTemperatureOffset, t_offset as u16)?;
        Ok(())
    }

    /// Get sensor altitude in meters above sea level.
    pub fn altitude(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetSensorAltitude, &mut buf)?;
        let altitude = u16::from_be_bytes([buf[0], buf[1]]);
        Ok(altitude)
    }

    /// Set sensor altitude in meters above sea level.
    pub fn set_altitude(&mut self, altitude: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetSensorAltitude, altitude)?;
        Ok(())
    }

    /// Set ambient pressure to enable continious pressure compensation
    pub fn set_ambient_pressure(&mut self, pressure_hpa: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetAmbientPressure, pressure_hpa)?;
        Ok(())
    }

    /// Perform forced recalibration
    pub fn forced_recalibration(&mut self, target_co2_concentration: u16) -> Result<u16, Error<E>> {
        let frc_correction = self.delayed_read_cmd_with_data(
            Command::PerformForcedRecalibration,
            target_co2_concentration,
        )?;
        if frc_correction == u16::MAX {
            return Err(Error::Internal);
        }
        match frc_correction.checked_sub(0x8000) {
            Some(concentration) => Ok(concentration),
            None => Err(Error::Internal),
        }
    }

    /// Get the status of automatic self-calibration
    pub fn automatic_self_calibration(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetAutomaticSelfCalibrationEnabled, &mut buf)?;
        let status = u16::from_be_bytes([buf[0], buf[1]]) != 0;
        Ok(status)
    }

    /// Enable or disable automatic self-calibration
    pub fn set_automatic_self_calibration(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetAutomaticSelfCalibrationEnabled, enabled as u16)?;
        Ok(())
    }

    /// Start low power periodic measurements
    pub fn start_low_power_periodic_measurements(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StartLowPowerPeriodicMeasurement)?;
        Ok(())
    }

    /// Check whether new measurement data is available for read-out.
    pub fn data_ready_status(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetDataReadyStatus, &mut buf)?;
        let status = u16::from_be_bytes([buf[0], buf[1]]);

        // 7FF is the last 11 bytes. If they are all zeroes, then data isn't ready.
        let ready = (status & 0x7FF) != 0;
        Ok(ready)
    }

    /// Save settings to non-volatile memory
    pub fn persist_settings(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PersistSettings)?;
        Ok(())
    }

    /// Get 48-bit serial number
    pub fn serial_number(&mut self) -> Result<u64, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::GetSerialNumber, &mut buf)?;
        let serial = u64::from(buf[0]) << 40
            | u64::from(buf[1]) << 32
            | u64::from(buf[3]) << 24
            | u64::from(buf[4]) << 16
            | u64::from(buf[6]) << 8
            | u64::from(buf[7]);

        Ok(serial)
    }

    ///  End-of-line test to confirm sensor functionality.
    pub fn self_test_is_ok(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::PerformSelfTest, &mut buf)?;

        let status = u16::from_be_bytes([buf[0], buf[1]]) == 0;
        Ok(status)
    }

    /// Initiates the reset of all configurations stored in the EEPROM and erases the FRC and ASC algorithm history.
    pub fn factory_reset(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PerformFactoryReset)?;
        Ok(())
    }

    /// The reinit command reinitializes the sensor by reloading user settings from EEPROM.
    pub fn reinit(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::Reinit)?;
        Ok(())
    }

    /// On-demand measurement of CO₂ concentration, relative humidity and temperature.
    /// The sensor output is read with the measurement method.
    /// Takes around 5 seconds to complete
    #[cfg(feature = "scd41")]
    pub fn measure_single_shot(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShot)?;
        Ok(())
    }

    /// On-demand measurement of CO₂ concentration, relative humidity and temperature.
    /// The sensor output is read with the measurement method.
    /// Completes immediately, but the measurement can only be read after 5 seconds.
    #[cfg(feature = "scd41")]
    pub fn measure_single_shot_non_blocking(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShotNonBlocking)?;
        Ok(())
    }

    /// On-demand measurement of relative humidity and temperature only.
    #[cfg(feature = "scd41")]
    pub fn measure_single_shot_rht(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShotRhtOnly)?;
        Ok(())
    }

    /// Put the sensor from idle to sleep mode to reduce current consumption.
    #[cfg(feature = "scd41")]
    pub fn power_down(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PowerDown)?;
        Ok(())
    }

    /// Wake up sensor from sleep mode to idle mode.
    #[cfg(feature = "scd41")]
    pub fn wake_up(&mut self) {
        // Sensor does not acknowledge the wake-up call, error is ignored
        self.write_command(Command::WakeUp).ok();
    }

    /// Command for reading values from the sensor
    fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd)?;
        i2c::read_words_with_crc(&mut self.i2c, SCD4X_I2C_ADDRESS, data)?;
        Ok(())
    }

    /// Send command with parameter, takes response
    fn delayed_read_cmd_with_data(&mut self, cmd: Command, data: u16) -> Result<u16, Error<E>> {
        self.write_command_with_data(cmd, data)?;
        let mut buf = [0; 3];
        i2c::read_words_with_crc(&mut self.i2c, SCD4X_I2C_ADDRESS, &mut buf)?;

        Ok(u16::from_be_bytes([buf[0], buf[1]]))
    }

    /// Writes commands without additional arguments.
    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay, allowed_if_running) = cmd.as_tuple();
        if !allowed_if_running && self.is_running {
            return Err(Error::NotAllowed);
        }
        i2c::write_command_u16(&mut self.i2c, SCD4X_I2C_ADDRESS, command).map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }

    /// Sets sensor internal parameter
    fn write_command_with_data(&mut self, cmd: Command, data: u16) -> Result<(), Error<E>> {
        let (command, delay, allowed_if_running) = cmd.as_tuple();
        if !allowed_if_running && self.is_running {
            return Err(Error::NotAllowed);
        }
        let c = command.to_be_bytes();
        let d = data.to_be_bytes();

        let mut buf = [0; 5];
        buf[0..2].copy_from_slice(&c);
        buf[2..4].copy_from_slice(&d);
        buf[4] = crc8::calculate(&d);

        self.i2c
            .write(SCD4X_I2C_ADDRESS, &buf)
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    /// Test the get_serial_number function
    #[test]
    fn test_get_serial_number() {
        // Arrange
        let (cmd, _, _) = Command::GetSerialNumber.as_tuple();
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(
                SCD4X_I2C_ADDRESS,
                vec![0xbe, 0xef, 0x92, 0xbe, 0xef, 0x92, 0xbe, 0xef, 0x92],
            ),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);
        // Act
        let serial = sensor.serial_number().unwrap();
        // Assert
        assert_eq!(serial, 0xbeefbeefbeef);
    }

    /// Test the measurement function
    #[test]
    fn test_measurement() {
        // Arrange
        let (cmd, _, _) = Command::ReadMeasurement.as_tuple();
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(
                SCD4X_I2C_ADDRESS,
                vec![0x03, 0xE8, 0xD4, 0x62, 0x03, 0x5E, 0x80, 0x00, 0xA2],
            ),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);
        // Act
        let data = sensor.measurement().unwrap();
        // Assert
        assert_eq!(data.co2, 1000_u16);
        assert_eq!(data.temperature, 22.000198_f32);
        assert_eq!(data.humidity, 50_f32);
    }
}
