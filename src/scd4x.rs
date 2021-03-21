use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::commands::Command;
use crate::error::Error;
use crate::types::SensorData;
use sensirion_i2c::i2c;

const SCD4X_I2C_ADDRESS: u8 = 0x62;

#[derive(Debug, Default)]
pub struct Scd4x<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> Scd4x<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Scd4x { i2c, delay }
    }

    pub fn wake_up(&mut self) {
        // Sensor does not acknowledge the wake-up call, error is ignored
        self.write_command(Command::WakeUp).ok();
    }

    pub fn start_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StartPeriodicMeasurement)?;
        Ok(())
    }

    pub fn stop_periodic_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::StopPeriodicMeasurement)?;
        Ok(())
    }

    pub fn reinit(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::Reinit)?;
        Ok(())
    }

    pub fn read_measurement_linux(&mut self) -> Result<Option<SensorData>, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        let co2 = f32::from(u16::from_be_bytes([buf[0], buf[1]]));
        let temperature = f32::from(u16::from_be_bytes([buf[3], buf[4]]));
        let humidity = f32::from(u16::from_be_bytes([buf[6], buf[7]]));

        Ok(Some(SensorData {
            co2: co2,
            temperature: temperature * 175.0 / 65536.0 - 45.0,
            humidity: humidity * 100.0 / 65536.0,
        }))
    }

    pub fn read_measurement_raw(&mut self) -> Result<Option<SensorData>, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        Ok(Some(SensorData {
            co2: f32::from(u16::from_be_bytes([buf[0], buf[1]])),
            temperature: f32::from(u16::from_be_bytes([buf[3], buf[4]])),
            humidity: f32::from(u16::from_be_bytes([buf[6], buf[7]])),
        }))
    }

    pub fn read_measurement(&mut self) -> Result<Option<SensorData>, Error<E>> {
        let mut buf = [0; 9];
        self.delayed_read_cmd(Command::ReadMeasurement, &mut buf)?;

        let co2 = f32::from(u16::from_be_bytes([buf[0], buf[1]]));
        //let temperature = f32::from(u16::from_be_bytes([ buf[3],  buf[4]]));
        let temperature = i32::from(u16::from_be_bytes([buf[3], buf[4]]));
        let humidity = i32::from(u16::from_be_bytes([buf[6], buf[7]]));

        Ok(Some(SensorData {
            co2: co2,
            temperature: (((21875 * temperature) >> 13) - 45000) as f32,
            humidity: ((12500 * humidity) >> 13) as f32,
        }))
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

    /// Command for reading values from the sensor
    fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd)?;
        i2c::read_words_with_crc(&mut self.i2c, SCD4X_I2C_ADDRESS, data)?;
        Ok(())
    }

    /// Writes commands without additional arguments.
    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay) = cmd.as_tuple();
        i2c::write_command(&mut self.i2c, SCD4X_I2C_ADDRESS, command).map_err(Error::I2c)?;
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
