use super::*;
use async_hal::{delay::DelayNs, i2c::I2c};
use embedded_hal_async as async_hal;
use sensirion_i2c::i2c_async;

/// Asynchronous SCD4X sensor instance, for use with [`embedded_hal_async`].
///
/// Use related methods to take measurements.
#[derive(Debug, Default)]
pub struct Scd4xAsync<I2C, D> {
    i2c: I2C,
    delay: D,
    is_running: bool,
}

impl<I2C, D, E> Scd4xAsync<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            i2c,
            delay,
            is_running: false,
        }
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Start periodic measurement, signal update interval is 5 seconds.
    /// This command is only available in idle mode.
    pub async fn start_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StartPeriodicMeasurement)
            .await?;
        self.is_running = true;
        Ok(())
    }

    /// Stop periodic measurement and return to idle mode for sensor configuration or to safe energy.
    /// This command is only available in measurement mode.
    pub async fn stop_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StopPeriodicMeasurement).await?;
        self.is_running = false;
        Ok(())
    }

    /// Read raw sensor data
    pub async fn sensor_output(&mut self) -> Result<RawSensorData, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)
            .await?;

        Ok(RawSensorData::from_bytes(buf))
    }

    /// Read converted sensor data
    pub async fn measurement(&mut self) -> Result<SensorData, Error<E>> {
        let raw = self.sensor_output().await?;
        Ok(SensorData::from_raw(raw))
    }

    /// Get sensor temperature offset
    pub async fn temperature_offset(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetTemperatureOffset, &mut buf)
            .await?;

        Ok(temp_offset_from_bytes(buf))
    }

    /// Set sensor temperature offset
    pub async fn set_temperature_offset(&mut self, offset: f32) -> Result<(), Error<E>> {
        let t_offset = (offset * 65536.0 / 175.0) as i16;
        self.write_command_with_data(Command::SetTemperatureOffset, t_offset as u16)
            .await?;
        Ok(())
    }

    /// Get sensor altitude in meters above sea level.
    pub async fn altitude(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetSensorAltitude, &mut buf)
            .await?;
        let altitude = u16::from_be_bytes([buf[0], buf[1]]);
        Ok(altitude)
    }

    /// Set sensor altitude in meters above sea level.
    pub async fn set_altitude(&mut self, altitude: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetSensorAltitude, altitude)
            .await?;
        Ok(())
    }

    /// Set ambient pressure to enable continious pressure compensation
    pub async fn set_ambient_pressure(&mut self, pressure_hpa: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetAmbientPressure, pressure_hpa)
            .await?;
        Ok(())
    }

    /// Perform forced recalibration
    pub async fn forced_recalibration(
        &mut self,
        target_co2_concentration: u16,
    ) -> Result<u16, Error<E>> {
        let frc_correction = self
            .delayed_read_cmd_with_data(
                Command::PerformForcedRecalibration,
                target_co2_concentration,
            )
            .await?;
        check_frc_correction(frc_correction)
    }

    /// Get the status of automatic self-calibration
    pub async fn automatic_self_calibration(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetAutomaticSelfCalibrationEnabled, &mut buf)
            .await?;
        let status = u16::from_be_bytes([buf[0], buf[1]]) != 0;
        Ok(status)
    }

    /// Enable or disable automatic self-calibration
    pub async fn set_automatic_self_calibration(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetAutomaticSelfCalibrationEnabled, enabled as u16)
            .await?;
        Ok(())
    }

    /// Start low power periodic measurements
    pub async fn start_low_power_periodic_measurements(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StartLowPowerPeriodicMeasurement)
            .await?;
        Ok(())
    }

    /// Check whether new measurement data is available for read-out.
    pub async fn data_ready_status(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetDataReadyStatus, &mut buf)
            .await?;
        let status = u16::from_be_bytes([buf[0], buf[1]]);

        // 7FF is the last 11 bytes. If they are all zeroes, then data isn't ready.
        let ready = (status & 0x7FF) != 0;
        Ok(ready)
    }

    /// Save settings to non-volatile memory
    pub async fn persist_settings(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PersistSettings).await?;
        Ok(())
    }

    /// Get 48-bit serial number
    pub async fn serial_number(&mut self) -> Result<u64, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::GetSerialNumber, &mut buf)
            .await?;

        Ok(serial_number_from_bytes(buf))
    }

    ///  End-of-line test to confirm sensor functionality.
    pub async fn self_test_is_ok(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::PerformSelfTest, &mut buf)
            .await?;

        let status = u16::from_be_bytes([buf[0], buf[1]]) == 0;
        Ok(status)
    }

    /// Initiates the reset of all configurations stored in the EEPROM and erases the FRC and ASC algorithm history.
    pub async fn factory_reset(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PerformFactoryReset).await?;
        Ok(())
    }

    /// The reinit command reinitializes the sensor by reloading user settings from EEPROM.
    pub async fn reinit(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::Reinit).await?;
        Ok(())
    }

    /// On-demand measurement of CO₂ concentration, relative humidity and temperature.
    /// The sensor output is read with the measurement method.
    /// Takes around 5 seconds to complete
    #[cfg(feature = "scd41")]
    pub async fn measure_single_shot(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShot).await?;
        Ok(())
    }

    /// On-demand measurement of CO₂ concentration, relative humidity and temperature.
    /// The sensor output is read with the measurement method.
    /// Completes immediately, but the measurement can only be read after 5 seconds.
    #[cfg(feature = "scd41")]
    pub async fn measure_single_shot_non_blocking(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShotNonBlocking)
            .await?;
        Ok(())
    }

    /// On-demand measurement of relative humidity and temperature only.
    #[cfg(feature = "scd41")]
    pub async fn measure_single_shot_rht(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::MeasureSingleShotRhtOnly)
            .await?;
        Ok(())
    }

    /// Put the sensor from idle to sleep mode to reduce current consumption.
    #[cfg(feature = "scd41")]
    pub async fn power_down(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::PowerDown).await?;
        Ok(())
    }

    /// Wake up sensor from sleep mode to idle mode.
    #[cfg(feature = "scd41")]
    pub async fn wake_up(&mut self) {
        // Sensor does not acknowledge the wake-up call, error is ignored
        self.write_command(Command::WakeUp).await.ok();
    }

    /// Command for reading values from the sensor
    async fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd).await?;
        i2c_async::read_words_with_crc(&mut self.i2c, SCD4X_I2C_ADDRESS, data).await?;
        Ok(())
    }

    /// Send command with parameter, takes response
    async fn delayed_read_cmd_with_data(
        &mut self,
        cmd: Command,
        data: u16,
    ) -> Result<u16, Error<E>> {
        self.write_command_with_data(cmd, data).await?;
        let mut buf = [0; 3];
        i2c_async::read_words_with_crc(&mut self.i2c, SCD4X_I2C_ADDRESS, &mut buf).await?;

        Ok(u16::from_be_bytes([buf[0], buf[1]]))
    }

    /// Writes commands without additional arguments.
    async fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay, allowed_if_running) = cmd.as_tuple();
        if !allowed_if_running && self.is_running {
            return Err(Error::NotAllowed);
        }
        i2c_async::write_command_u16(&mut self.i2c, SCD4X_I2C_ADDRESS, command)
            .await
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay).await;
        Ok(())
    }

    /// Sets sensor internal parameter
    async fn write_command_with_data(&mut self, cmd: Command, data: u16) -> Result<(), Error<E>> {
        let (command, delay, allowed_if_running) = cmd.as_tuple();
        if !allowed_if_running && self.is_running {
            return Err(Error::NotAllowed);
        }

        let buf = encode_cmd_with_data(command, data);

        self.i2c
            .write(SCD4X_I2C_ADDRESS, &buf)
            .await
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay).await;
        Ok(())
    }
}
