use embedded_hal::delay::DelayNs;
use hal::{Delay, I2CError, I2cdev};
use linux_embedded_hal as hal;

use log::{debug, error, info};

use humantime::Duration as HumanDuration;
use simplelog::{LevelFilter, TermLogger};
use structopt::StructOpt;

use scd4x::{Error, Scd4x};

#[derive(StructOpt)]
#[structopt(name = "scd4x-util")]
/// A Command Line Interface (CLI) for interacting with a sensiron SCD4x environmental sensor via Linux I2C
pub struct Options {
    /// Specify the i2c interface to use to connect to the scd30 device
    #[structopt(
        short = "d",
        long = "i2c",
        default_value = "/dev/i2c-1",
        env = "SCD4x_I2C"
    )]
    i2c: String,

    /// Delay between sensor poll operations
    #[structopt(long = "poll-delay", default_value = "100ms")]
    pub poll_delay: HumanDuration,

    /// Number of allowed I2C errors (per measurement attempt) prior to exiting
    #[structopt(long = "allowed-errors", default_value = "3")]
    pub allowed_errors: usize,

    /// Enable verbose logging
    #[structopt(long = "log-level", default_value = "info")]
    level: LevelFilter,
}

fn main() -> Result<(), Error<I2CError>> {
    // Load options
    let opts = Options::from_args();

    // Setup logging
    TermLogger::init(
        opts.level,
        simplelog::Config::default(),
        simplelog::TerminalMode::Mixed,
        simplelog::ColorChoice::Auto,
    )
    .unwrap();

    debug!("Connecting to I2C device");
    let i2c = match I2cdev::new(&opts.i2c) {
        Ok(v) => v,
        Err(e) => {
            error!("Error opening I2C device '{}': {:?}", &opts.i2c, e);
            std::process::exit(-1);
        }
    };

    debug!("Connecting to sensor");
    let mut sensor = Scd4x::new(i2c, Delay);

    debug!("Initalising sensor");

    #[cfg(feature = "scd41")]
    sensor.wake_up();
    sensor.stop_periodic_measurement()?;
    sensor.reinit()?;

    let serial = sensor.serial_number()?;
    info!("Serial: {serial:#04x}");

    debug!("Starting periodic measurement");
    sensor.start_periodic_measurement()?;

    debug!("Waiting for first measurement... (5 sec)");

    loop {
        hal::Delay.delay_ms(5000u32);

        debug!("Waiting for data ready");
        loop {
            match sensor.data_ready_status() {
                Ok(true) => break,
                Ok(false) => std::thread::sleep(*opts.poll_delay),
                Err(e) => {
                    error!("Failed to poll for data ready: {e:?}");
                    std::process::exit(-2);
                }
            }
        }

        debug!("Reading sensor data");
        let data = match sensor.measurement() {
            Ok(v) => v,
            Err(e) => {
                error!("Failed to read measurement: {e:?}");
                std::process::exit(-3);
            }
        };

        println!(
            "CO2: {0}, Temperature: {1:#.2} °C, Humidity: {2:#.2} RH",
            data.co2, data.temperature, data.humidity
        );
    }
}
