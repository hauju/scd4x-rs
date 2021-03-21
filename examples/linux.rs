use embedded_hal::blocking::delay::DelayMs;
use hal::{Delay, I2cdev};
use linux_embedded_hal as hal;

use scd4x::scd4x::Scd4x;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sensor = Scd4x::new(dev, Delay);

    sensor.wake_up();
    sensor.stop_periodic_measurement().unwrap();
    sensor.reinit().unwrap();

    let serial = sensor.get_serial_number().unwrap();
    println!("serial: {:#04x}", serial);

    sensor.start_periodic_measurement().unwrap();
    println!("Waiting for first measurement... (5 sec)");
    loop {
        hal::Delay.delay_ms(5000u16);

        let opt = sensor.read_measurement_raw().unwrap();
        match opt {
            Some(data) => println!(
                "CO2: {0}, Temperature: {1} m°C, Humidity: {2} mRH",
                data.co2, data.temperature, data.humidity
            ),
            _ => println!("No data!"),
        };
    }
}
