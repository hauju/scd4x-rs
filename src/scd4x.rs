use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::commands::Command;
use crate::error::Error;
use crate::types::{RawSensorData, SensorData};
use sensirion_i2c::{crc8, i2c};

const SCD4X_I2C_ADDRESS: u8 = 0x62;

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

    /// Read and convert sensor data
    pub fn measurement(&mut self) -> Result<SensorData, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        // buf[2], buf[5], buf[8] is CRC bytes and not used
        let co2 = u16::from_be_bytes([buf[0], buf[1]]);
        let temperature = u16::from_be_bytes([buf[3], buf[4]]);
        let humidity = u16::from_be_bytes([buf[6], buf[7]]);

        Ok(SensorData {
            co2: co2,
            temperature: ((((21875 * temperature) >> 13) - 45000) as f32) / 1000.0,
            humidity: (((12500 * humidity) >> 13) as f32) / 1000.0,
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
        let t_offset = (offset *65536.0 / 175.0) as i16;
        self.write_parameter(Command::SetTemperatureOffset, t_offset as u16)?;
        Ok(())
    }

    pub fn reinit(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::Reinit)?;
        Ok(())
    }

    pub fn get_serial_number(&mut self) -> Result<u64, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::GetSerial, &mut buf)?;
        //let serial = u16::from_be_bytes([serial[0], serial[1]]);
        let serial = u64::from(buf[0]) << 40
            | u64::from(buf[1]) << 32
            | u64::from(buf[3]) << 24
            | u64::from(buf[4]) << 16
            | u64::from(buf[6]) << 8
            | u64::from(buf[7]);

        //let serial = u64::from_be_bytes([0x00, 0x00, buf[0], buf[1], buf[3], buf[4], buf[6], buf[7]]);
        Ok(serial)
    }

    /// Wake up sensor from sleep mode to idle mode.
    /// Only available in sleep mode.
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

    /// Writes commands without additional arguments.
    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay, allowed_if_running) = cmd.as_tuple();
        if !allowed_if_running && self.is_running {
            return Err(Error::NotAllowed);
        }
        i2c::write_command(&mut self.i2c, SCD4X_I2C_ADDRESS, command).map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }

    /// Sets sensor internal parameter
    fn write_parameter(&mut self, cmd: Command, data: u16) -> Result<(), Error<E>> {
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

        self.i2c.write(SCD4X_I2C_ADDRESS, buf).map_err(Error::I2c)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    /// Test the get_serial_number function
    #[test]
    fn test_get_serial_number() {
        // Arrange
        let (cmd, _) = Command::GetSerial.as_tuple();
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
        let serial = sensor.get_serial_number().unwrap();
        // Assert
        assert_eq!(serial, 0xbeefbeefbeef);
    }
}
