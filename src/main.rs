#![no_std]
#![no_main]

extern crate embedded_hal as hal;

use hal::prelude::*;

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::prelude::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::ledc::{config::TimerConfig, Channel, Timer};
use esp_idf_hal::delay::FreeRtos;

use log::*;

#[no_mangle]
fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("Am RISC-Y");
    
    let peripherals = Peripherals::take().unwrap();
    let config = TimerConfig::default().frequency(25.kHz().into());
    let timer = Timer::new(peripherals.ledc.timer0, &config).unwrap();
    let mut channel = Channel::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio4).unwrap();
    _ = channel.set_duty(channel.get_max_duty());
    _ = channel.enable();
    loop {
        _ = channel.disable();
        FreeRtos.delay_ms(100u16);
        _ = channel.enable();
        FreeRtos.delay_ms(2000u16);
        info!("tick")
    }
}
