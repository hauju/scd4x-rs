use embedded_hal::delay::DelayNs;
use hal::{Delay, I2cdev};
use linux_embedded_hal as hal;

use scd4x::Scd4x;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sensor = Scd4x::new(dev, Delay);

    #[cfg(feature = "scd41")]
    sensor.wake_up();
    sensor.stop_periodic_measurement().unwrap();
    sensor.reinit().unwrap();

    let serial = sensor.serial_number().unwrap();
    println!("serial: {serial:#04x}");

    sensor.start_periodic_measurement().unwrap();
    println!("Waiting for first measurement... (5 sec)");
    loop {
        hal::Delay.delay_ms(5000u32);

        let data = sensor.measurement().unwrap();

        println!(
            "CO2: {0}, Temperature: {1:#.2} °C, Humidity: {2:#.2} RH",
            data.co2, data.temperature, data.humidity
        );
    }
}
