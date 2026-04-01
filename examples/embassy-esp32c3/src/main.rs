//! SCD4x example for ESP32-C3 using Embassy async runtime.
//!
//! Reads CO2, temperature, and humidity from the sensor via async I2C.
//!
//! # Hardware
//! - Board: ESP32-C3 (e.g. ESP32-C3-DevKitM-1)
//! - SCD4x sensor connected via I2C: GPIO6 = SDA, GPIO7 = SCL
//! - Target: riscv32imc-unknown-none-elf
//!
//! # Flash & monitor
//! ```bash
//! cargo run --release
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use scd4x::Scd4xAsync;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("SCD4x ESP32-C3 example starting...");

    let config = esp_hal::Config::default();
    let peripherals = esp_hal::init(config);

    // Heap allocator
    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    // Initialize embassy time driver
    esp_hal_embassy::init(timg0.timer0);

    // Initialize I2C for SCD4x sensor (GPIO6 = SDA, GPIO7 = SCL)
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let mut scd4x = Scd4xAsync::new(i2c, embassy_time::Delay);

    // Stop any ongoing measurement, reinit, then start periodic measurement
    let _ = scd4x.stop_periodic_measurement().await;
    Timer::after_millis(500).await;
    scd4x.reinit().await.unwrap();
    Timer::after_millis(20).await;

    let serial = scd4x.serial_number().await.unwrap();
    esp_println::println!("SCD4x serial: {}", serial);

    scd4x.start_periodic_measurement().await.unwrap();
    esp_println::println!("Waiting for first measurement... (5 sec)");

    // Main loop: read sensor every 5 seconds
    loop {
        Timer::after(Duration::from_secs(5)).await;

        match scd4x.data_ready_status().await {
            Ok(true) => match scd4x.measurement().await {
                Ok(data) => {
                    esp_println::println!(
                        "CO2: {} ppm, Temperature: {:.1} \u{00b0}C, Humidity: {:.1} %RH",
                        data.co2, data.temperature, data.humidity
                    );
                }
                Err(e) => {
                    esp_println::println!("SCD4x measurement error: {:?}", e);
                }
            },
            Ok(false) => {
                esp_println::println!("Data not ready yet");
            }
            Err(e) => {
                esp_println::println!("SCD4x I2C error: {:?}", e);
            }
        }
    }
}
