#![no_std]
#![no_main]

extern crate embedded_hal as hal;

use core::borrow::Borrow;

use hal::digital::v2::InputPin;
use hal::prelude::*;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{self, Unknown, ADCPin};
use esp_idf_hal::gpio::{OutputPin, Pins, Pull};
use esp_idf_hal::ledc::{config::TimerConfig, Channel, HwChannel, HwTimer, Timer};
use esp_idf_hal::peripherals::{self, Peripherals};
use esp_idf_hal::{prelude::*, adc};
use esp_idf_sys::{
    self, esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH,
    esp_set_deep_sleep_wake_stub, GPIO_MODE_DEF_OUTPUT, gpio_hold_dis,
};

use log::*;

const TICK_INTERVAL_MS: u16 = 100u16; // milliseconds
// Sequences
const BLINKY_DUTY_SEQUENCE: &[u32] = &[5, 4, 3, 2, 1, 2, 3, 4, 5, 6, 10, 9, 8];
const BLINKY_SHUTDOWN_DUTY_SEQUENCE: &[u32] = &[1, 2, 3, 4, 5, 6, 7, 7, 5, 4, 10];

const SLEEPY_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 3, 5, 8, 13, 21, 33];
const STATUS_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 2, 3, 3, 3, 5, 5, 5, 5, 5, 3, 3, 3, 2, 2, 1, 1];
const STATUS_SHUTDOWN_SEQUENCE: &[u32] = &[1, 1, 2, 3, 5, 8, 13, 21, 33];

const SLEEP_WAKEUP_PIN_MASK: u64 = 1 << 1;

// Pins
const SLEEPY_LED: i32 = 7;
const STATUS_LED: i32 = 10;

#[no_mangle]
fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Clear up deep sleep pin state. FIXME: Handle this better.
    // Set up wake stub.
    unsafe {
        esp_set_deep_sleep_wake_stub(Some(wake_stub));
        // Clear deep sleep holds.
        esp_idf_sys::gpio_deep_sleep_hold_dis();
        gpio_hold_dis(SLEEPY_LED); // Disable held state when resuming from deep sleep.
    }

    info!("Setting up soil check");
    let peripherals = Peripherals::take().unwrap();
    let config = TimerConfig::default().frequency(25.kHz().into());

    // Two LEDS, blinky and sleepy.
    let blinky_timer = Timer::new(peripherals.ledc.timer0, &config).unwrap();
    let sleepy_timer = Timer::new(peripherals.ledc.timer1, &config).unwrap();
    let status_timer = Timer::new(peripherals.ledc.timer2, &config).unwrap();

    // Set up LEDs.
    let mut blinky = Channel::new(
        peripherals.ledc.channel0,
        &blinky_timer,
        peripherals.pins.gpio0,
    )
    .unwrap();
    let mut sleepy = Channel::new(
        peripherals.ledc.channel1,
        &sleepy_timer,
        peripherals.pins.gpio7,
    )
    .unwrap();
    let mut status = Channel::new(
        peripherals.ledc.channel2,
        &status_timer,
        peripherals.pins.gpio10,
    ).unwrap();

    // Set up buttons for the LEDs.
    let mut led_button = peripherals.pins.gpio1.into_input().unwrap();
    led_button.set_pull_down().unwrap();
    let mut sleep_button = peripherals.pins.gpio6.into_input().unwrap();
    sleep_button.set_pull_up().unwrap();

    // Moisture sensor.
    let mut moisture_sensor = peripherals.pins.gpio3.into_analog_atten_0db().unwrap();
    let mut powered_adc = adc::PoweredAdc::new(peripherals.adc1, adc::config::Config::new().calibration(true)).unwrap();

    unsafe {
        esp_idf_sys::esp_deep_sleep_enable_gpio_wakeup(
            SLEEP_WAKEUP_PIN_MASK,
            esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH,
        );
    }

    loop {
        if sleep_button.is_low().unwrap() {
            perform_duty_cycle(&mut blinky, BLINKY_SHUTDOWN_DUTY_SEQUENCE, 50);
            perform_duty_cycle(&mut sleepy, SLEEPY_DUTY_SEQUENCE, 50);
            perform_duty_cycle(&mut status, STATUS_DUTY_SEQUENCE, 50);
            unsafe {
                info!("sleeping");
                esp_idf_sys::gpio_reset_pin(SLEEPY_LED);
                esp_idf_sys::gpio_set_direction(SLEEPY_LED, GPIO_MODE_DEF_OUTPUT);
                FreeRtos.delay_us(500u16);
                esp_idf_sys::gpio_output_set(1 << SLEEPY_LED, 1 << STATUS_LED, 0,1 << STATUS_LED);
                esp_idf_sys::gpio_hold_en(SLEEPY_LED);
                esp_idf_sys::gpio_deep_sleep_hold_en();
                FreeRtos.delay_ms(1000u16);
                esp_idf_sys::esp_deep_sleep_start();
            }
        }
        if led_button.is_high().unwrap() {
            info!("led button pressed");
            _ = status.disable();
            perform_duty_cycle(&mut blinky, BLINKY_DUTY_SEQUENCE, 50);
            // read moisture.
            info!("moisture: {}", powered_adc.read(&mut moisture_sensor).unwrap());
        } else {
        // The main action.
            perform_duty_cycle(&mut status, STATUS_DUTY_SEQUENCE, 30);
            FreeRtos.delay_ms(TICK_INTERVAL_MS);
        }
    }
}


fn perform_duty_cycle<C, H, T, P>(
    channel: &mut Channel<C, H, T, P>,
    sequence: &[u32],
    delay_ms: u16,
) -> ()
where
    C: HwChannel,
    H: HwTimer,
    T: Borrow<Timer<H>>,
    P: OutputPin,
{
    _ = channel.enable();
    for divisor in sequence.iter() {
        _ = channel.set_duty(channel.get_max_duty() / divisor);
        FreeRtos.delay_ms(delay_ms);
    }
}

extern "C" fn wake_stub() -> () {
    unsafe {
        //esp_idf_sys::gpio_set_level(SLEEPY_LED, 0);
        info!("waking up...");
    }
}
