use embedded_hal as hal;
use hal::delay::DelayNs;
use hal::i2c::I2c;

use crate::commands::Command;
use crate::error::Error;
use crate::types::{RawSensorData, SensorData};
use sensirion_i2c::{crc8, i2c};

const SCD4X_I2C_ADDRESS: u8 = 0x62;

#[cfg(feature = "embedded-hal-async")]
mod async_impl;
#[cfg(feature = "embedded-hal-async")]
pub use async_impl::Scd4xAsync;

/// SCD4X sensor instance. Use related methods to take measurements.
#[derive(Debug, Default)]
pub struct Scd4x<I2C, D> {
    i2c: I2C,
    delay: D,
    is_running: bool,
}

impl<I2C, D, E> Scd4x<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Scd4x {
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

        Ok(RawSensorData::from_bytes(buf))
    }

    /// Read converted sensor data
    pub fn measurement(&mut self) -> Result<SensorData, Error<E>> {
        let raw = self.sensor_output()?;
        Ok(SensorData::from_raw(raw))
    }

    /// Get sensor temperature offset
    pub fn temperature_offset(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetTemperatureOffset, &mut buf)?;

        Ok(temp_offset_from_bytes(buf))
    }

    /// Set sensor temperature offset
    pub fn set_temperature_offset(&mut self, offset: f32) -> Result<(), Error<E>> {
        let t_offset = temp_offset_to_u16(offset);
        self.write_command_with_data(Command::SetTemperatureOffset, t_offset)?;
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

    /// Set ambient pressure to enable continuous pressure compensation
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
        check_frc_correction(frc_correction)
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

    /// Get the current background CO2 level the sensor is configured to expect. Factory default is 400 ppm.
    pub fn automatic_self_calibration_target(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0; 3];
        self.delayed_read_cmd(Command::GetAutomaticSelfCalibrationTarget, &mut buf)?;
        let ppm = u16::from_be_bytes([buf[0], buf[1]]);
        Ok(ppm)
    }

    /// Set the background CO2 level the sensor should expect to measure
    pub fn set_automatic_self_calibration_target(&mut self, ppm: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(Command::SetAutomaticSelfCalibrationTarget, ppm)?;
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

        Ok(serial_number_from_bytes(buf))
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
        let buf = encode_cmd_with_data(command, data);

        self.i2c
            .write(SCD4X_I2C_ADDRESS, &buf)
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }
}

impl RawSensorData {
    fn from_bytes(buf: [u8; 9]) -> Self {
        // buf[2], buf[5], buf[8] is CRC bytes and not used
        let co2 = u16::from_be_bytes([buf[0], buf[1]]);
        let temperature = u16::from_be_bytes([buf[3], buf[4]]);
        let humidity = u16::from_be_bytes([buf[6], buf[7]]);
        Self {
            co2,
            temperature,
            humidity,
        }
    }
}

impl SensorData {
    fn from_raw(raw: RawSensorData) -> Self {
        let RawSensorData {
            co2,
            temperature,
            humidity,
        } = raw;
        SensorData {
            co2,
            temperature: (temperature as f32 * 175_f32) / (u16::MAX as f32) - 45_f32,
            humidity: (humidity as f32 * 100_f32) / (u16::MAX as f32),
        }
    }
}

fn serial_number_from_bytes(buf: [u8; 9]) -> u64 {
    (u64::from(buf[0]) << 40)
        | (u64::from(buf[1]) << 32)
        | (u64::from(buf[3]) << 24)
        | (u64::from(buf[4]) << 16)
        | (u64::from(buf[6]) << 8)
        | u64::from(buf[7])
}

fn encode_cmd_with_data(command: u16, data: u16) -> [u8; 5] {
    let c = command.to_be_bytes();
    let d = data.to_be_bytes();

    let mut buf = [0; 5];
    buf[0..2].copy_from_slice(&c);
    buf[2..4].copy_from_slice(&d);
    buf[4] = crc8::calculate(&d);
    buf
}

fn temp_offset_from_bytes(buf: [u8; 3]) -> f32 {
    let raw_offset = u16::from_be_bytes([buf[0], buf[1]]);
    (raw_offset as f32 * 175_f32) / (u16::MAX as f32)
}

fn temp_offset_to_u16(offset: f32) -> u16 {
    (((offset * (u16::MAX as f32)) / 175_f32) as i32) as u16
}

fn check_frc_correction<E>(frc_correction: u16) -> Result<u16, Error<E>> {
    if frc_correction == u16::MAX {
        return Err(Error::Internal);
    }
    match frc_correction.checked_sub(0x8000) {
        Some(concentration) => Ok(concentration),
        None => Err(Error::Internal),
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1 as hal;

    use self::hal::delay::NoopDelay as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    /// Build a 3-byte response word: [msb, lsb, crc]
    fn word(msb: u8, lsb: u8) -> [u8; 3] {
        [msb, lsb, crc8::calculate(&[msb, lsb])]
    }

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

        let mut mock = sensor.destroy();
        mock.done();
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
        assert_eq!(data.temperature, 22.00122_f32);
        assert_eq!(data.humidity, 50.000763_f32);

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test temperature offset round-trip conversion
    #[test]
    fn test_temp_offset_round_trip() {
        let offsets = [0.0_f32, 1.0, 5.0, 10.5, 0.1];
        for original in offsets {
            let encoded = temp_offset_to_u16(original);
            let buf = [
                (encoded >> 8) as u8,
                encoded as u8,
                0, // CRC placeholder (not checked here)
            ];
            let decoded = temp_offset_from_bytes(buf);
            assert!(
                (original - decoded).abs() < 0.01,
                "Round-trip failed for {original}: got {decoded}"
            );
        }
    }

    /// Test that idle-only commands fail when periodic measurement is running
    #[test]
    fn test_not_allowed_when_running() {
        let (start_cmd, _, _) = Command::StartPeriodicMeasurement.as_tuple();
        let expectations = [Transaction::write(
            SCD4X_I2C_ADDRESS,
            start_cmd.to_be_bytes().to_vec(),
        )];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        sensor.start_periodic_measurement().unwrap();

        // These commands are not allowed during periodic measurement
        assert_eq!(sensor.serial_number(), Err(Error::NotAllowed));
        assert_eq!(sensor.temperature_offset(), Err(Error::NotAllowed));
        assert_eq!(sensor.self_test_is_ok(), Err(Error::NotAllowed));
        assert_eq!(sensor.factory_reset(), Err(Error::NotAllowed));
        assert_eq!(sensor.reinit(), Err(Error::NotAllowed));
        assert_eq!(sensor.persist_settings(), Err(Error::NotAllowed));

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test that commands allowed during measurement still work
    #[test]
    fn test_allowed_when_running() {
        let (start_cmd, _, _) = Command::StartPeriodicMeasurement.as_tuple();
        let (read_cmd, _, _) = Command::ReadMeasurement.as_tuple();
        let (ready_cmd, _, _) = Command::GetDataReadyStatus.as_tuple();
        let ready_word = word(0x80, 0x01);
        let co2_word = word(0x01, 0xF4);
        let temp_word = word(0x62, 0x03);
        let hum_word = word(0x80, 0x00);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, start_cmd.to_be_bytes().to_vec()),
            // data_ready_status
            Transaction::write(SCD4X_I2C_ADDRESS, ready_cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, ready_word.to_vec()),
            // measurement
            Transaction::write(SCD4X_I2C_ADDRESS, read_cmd.to_be_bytes().to_vec()),
            Transaction::read(
                SCD4X_I2C_ADDRESS,
                [co2_word, temp_word, hum_word].concat(),
            ),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        sensor.start_periodic_measurement().unwrap();
        assert!(sensor.data_ready_status().unwrap());
        let data = sensor.measurement().unwrap();
        assert_eq!(data.co2, 500);

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test data_ready_status returns false when lower 11 bits are all zero
    #[test]
    fn test_data_not_ready() {
        let (cmd, _, _) = Command::GetDataReadyStatus.as_tuple();
        // 0xF800 => upper bits set, lower 11 bits zero => not ready
        let w = word(0xF8, 0x00);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(!sensor.data_ready_status().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test data_ready_status returns true when any lower 11 bits are set
    #[test]
    fn test_data_ready() {
        let (cmd, _, _) = Command::GetDataReadyStatus.as_tuple();
        let w = word(0x00, 0x01);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(sensor.data_ready_status().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test FRC correction with valid response
    #[test]
    fn test_frc_correction_valid() {
        // 0x8000 + 50 = 0x8032 means a correction of 50 ppm
        let result = check_frc_correction::<()>(0x8032);
        assert_eq!(result, Ok(50));
    }

    /// Test FRC correction returns zero correction
    #[test]
    fn test_frc_correction_zero() {
        let result = check_frc_correction::<()>(0x8000);
        assert_eq!(result, Ok(0));
    }

    /// Test FRC correction returns error on 0xFFFF (failed)
    #[test]
    fn test_frc_correction_failed() {
        let result = check_frc_correction::<()>(0xFFFF);
        assert_eq!(result, Err(Error::Internal));
    }

    /// Test FRC correction returns error on value below 0x8000
    #[test]
    fn test_frc_correction_underflow() {
        let result = check_frc_correction::<()>(0x7FFF);
        assert_eq!(result, Err(Error::Internal));
    }

    /// Test self-test passes (returns 0x0000)
    #[test]
    fn test_self_test_pass() {
        let (cmd, _, _) = Command::PerformSelfTest.as_tuple();
        let w = word(0x00, 0x00);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(sensor.self_test_is_ok().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test self-test fails (returns non-zero)
    #[test]
    fn test_self_test_fail() {
        let (cmd, _, _) = Command::PerformSelfTest.as_tuple();
        let w = word(0x00, 0x01);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(!sensor.self_test_is_ok().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test stop_periodic_measurement clears is_running flag
    #[test]
    fn test_stop_clears_running() {
        let (start_cmd, _, _) = Command::StartPeriodicMeasurement.as_tuple();
        let (stop_cmd, _, _) = Command::StopPeriodicMeasurement.as_tuple();
        let (serial_cmd, _, _) = Command::GetSerialNumber.as_tuple();
        let w1 = word(0x00, 0x01);
        let w2 = word(0x00, 0x02);
        let w3 = word(0x00, 0x03);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, start_cmd.to_be_bytes().to_vec()),
            Transaction::write(SCD4X_I2C_ADDRESS, stop_cmd.to_be_bytes().to_vec()),
            Transaction::write(SCD4X_I2C_ADDRESS, serial_cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, [w1, w2, w3].concat()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        sensor.start_periodic_measurement().unwrap();
        // serial_number should fail while running
        assert_eq!(sensor.serial_number(), Err(Error::NotAllowed));
        // after stopping, it should work again
        sensor.stop_periodic_measurement().unwrap();
        assert!(sensor.serial_number().is_ok());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test altitude get
    #[test]
    fn test_get_altitude() {
        let (cmd, _, _) = Command::GetSensorAltitude.as_tuple();
        // 0x0258 = 600 meters
        let w = word(0x02, 0x58);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert_eq!(sensor.altitude().unwrap(), 600);

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test automatic self-calibration status
    #[test]
    fn test_asc_enabled() {
        let (cmd, _, _) = Command::GetAutomaticSelfCalibrationEnabled.as_tuple();
        let w = word(0x00, 0x01);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(sensor.automatic_self_calibration().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }

    /// Test automatic self-calibration disabled
    #[test]
    fn test_asc_disabled() {
        let (cmd, _, _) = Command::GetAutomaticSelfCalibrationEnabled.as_tuple();
        let w = word(0x00, 0x00);
        let expectations = [
            Transaction::write(SCD4X_I2C_ADDRESS, cmd.to_be_bytes().to_vec()),
            Transaction::read(SCD4X_I2C_ADDRESS, w.to_vec()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Scd4x::new(mock, DelayMock);

        assert!(!sensor.automatic_self_calibration().unwrap());

        let mut mock = sensor.destroy();
        mock.done();
    }
}
