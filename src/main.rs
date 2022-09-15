#![no_std]
#![no_main]

extern crate embedded_hal as hal;

#[macro_use]
extern crate alloc;

use core::any;
use core::borrow::Borrow;
use core::fmt::Debug;
use core::ops::Range;

use anyhow;

use embedded_graphics::mono_font::{ascii::FONT_5X8, MonoTextStyle};
use hal::digital::v2::{InputPin, OutputPin};
use hal::prelude::*;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::Pull;
use esp_idf_hal::i2c;
use esp_idf_hal::ledc::{config::TimerConfig, Channel, HwChannel, HwTimer, Timer};
use esp_idf_hal::peripherals::{self, Peripherals};
use esp_idf_hal::{adc, prelude::*};
use esp_idf_sys::{
    self, esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH, gpio_hold_dis,
    GPIO_MODE_DEF_OUTPUT,
};

use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::*;

use embedded_graphics::geometry::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Triangle},
    text::Text,
};

use log::*;

const TICK_INTERVAL_MS: u16 = 100u16; // milliseconds

// Sequences
const BLINKY_DUTY_SEQUENCE: &[u32] = &[5, 4, 3, 2, 1, 2, 3, 4, 5, 6, 10, 9, 8];
const BLINKY_SHUTDOWN_DUTY_SEQUENCE: &[u32] = &[1, 2, 3, 4, 5, 6, 7, 7, 5, 4, 10];
const SLEEPY_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 3, 5, 8, 13, 21, 33];
const STATUS_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 2, 3, 3, 3, 5, 5, 5, 5, 5, 3, 3, 3, 2, 2, 1, 1];
const STATUS_SHUTDOWN_SEQUENCE: &[u32] = &[1, 1, 2, 3, 5, 8, 13, 21, 33];

//
const SLEEP_WAKEUP_PIN_MASK: u64 = 1 << 1;

// Pins
const SLEEPY_LED: i32 = 7;
const STATUS_LED: i32 = 0;

// Ranges
const HARD_TAP_WATER_RANGE: Range<u32> = 0u32..999u32;
const DAMP_SOIL_RANGE: Range<u32> = 1000u32..1100u32;
const MED_DAMP_SOIL_RANGE: Range<u32> = 1101u32..1300u32;
const DRY_SOIL_RANGE: Range<u32> = 1201u32..1599u32;
const DRY_AIR_RANGE: Range<u32> = 1600u32..2400u32;

#[derive(Debug)]
enum MoistureState {
    TapWater(u16),
    DampSoil(u16),
    MediumDampSoil(u16),
    DrySoil(u16),
    DryAir(u16),
    Unknown(u16),
}

#[derive(Debug)]
enum WavelengthState {
    Optimal(u16),
    TooDark(u16),
    TooBright(u16),
}

impl MoistureState {
    fn get(reading: u16) -> Self {
        if HARD_TAP_WATER_RANGE.contains(&(reading as u32)) {
            MoistureState::TapWater(reading)
        } else if DAMP_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::DampSoil(reading)
        } else if MED_DAMP_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::MediumDampSoil(reading)
        } else if DRY_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::DrySoil(reading)
        } else if DRY_AIR_RANGE.contains(&(reading as u32)) {
            MoistureState::DryAir(reading)
        } else {
            MoistureState::Unknown(reading)
        }
    }
}

impl WavelengthState {
    fn get(reading: u16) -> Self {
        Self::Optimal(reading)
    }
}

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
        // esp_set_deep_sleep_wake_stub(Some(wake_stub));
        // Clear deep sleep holds.
        esp_idf_sys::gpio_deep_sleep_hold_dis();
        esp_idf_sys::esp_deep_sleep_enable_gpio_wakeup(
            SLEEP_WAKEUP_PIN_MASK,
            esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH,
        );
        gpio_hold_dis(SLEEPY_LED); // Disable held state when resuming from deep sleep.
    }

    info!("Setting up soil check");
    let peripherals = Peripherals::take().unwrap();
    let config = TimerConfig::default().frequency(400.kHz().into());

    // Two LEDS, blinky and sleepy.
    let blinky_timer = Timer::new(peripherals.ledc.timer0, &config).unwrap();
    let sleepy_timer = Timer::new(peripherals.ledc.timer1, &config).unwrap();
    let status_timer = Timer::new(peripherals.ledc.timer2, &config).unwrap();

    // Set up LEDs.
    let mut action_led = Channel::new(
        peripherals.ledc.channel0,
        &blinky_timer,
        peripherals.pins.gpio10,
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
        peripherals.pins.gpio0,
    )
    .unwrap();

    // Set up buttons for the LEDs.
    let mut action_button = peripherals.pins.gpio8.into_input().unwrap();
    action_button.set_pull_down().unwrap();
    let mut sleep_button = peripherals.pins.gpio6.into_input().unwrap();
    sleep_button.set_pull_up().unwrap();

    // Moisture sensor.
    let mut powered_adc = adc::PoweredAdc::new(
        peripherals.adc1,
        adc::config::Config::new().calibration(true),
    )
    .unwrap();
    let mut moisture_sensor = peripherals.pins.gpio2.into_analog_atten_11db().unwrap();
    let mut wavelength_sensor = peripherals.pins.gpio1.into_analog_atten_11db().unwrap();

    // I2C
    let scl = peripherals.pins.gpio4;
    let sda = peripherals.pins.gpio5;
    let mut display_power = peripherals.pins.gpio3.into_output().unwrap();
    display_power
        .set_drive_strength(esp_idf_hal::gpio::DriveStrength::I40mA)
        .unwrap();
    display_power.set_high().unwrap();
    info!("set display on");
    let config = <i2c::config::MasterConfig>::default().baudrate(400.kHz().into());
    let display_handle = ssd1306::I2CDisplayInterface::new(
        i2c::Master::<i2c::I2C0, _, _>::new(peripherals.i2c0, i2c::MasterPins { sda, scl }, config)
            .unwrap(),
    );
    let mut display = ssd1306::Ssd1306::new(
        display_handle,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    // Display clear.
    display
        .init()
        .map_err(|e| info!("Display error: {:?}", e))
        .unwrap();
    FreeRtos.delay_ms(30u32);

    display.clear();
    draw_welcome_text(&mut display).unwrap();
    display
        .flush()
        .map_err(|e| info!("Flush error: {:?}", e))
        .unwrap();
    info!("initialized display. should be on now");

    loop {
        if sleep_button.is_low().unwrap() {
            display.clear();
            draw_display_text(&mut display, "Sleeping... Bye.").unwrap();
            display
                .flush()
                .map_err(|e| info!("Flush error: {:?}", e))
                .unwrap();
            perform_duty_cycle(&mut action_led, BLINKY_SHUTDOWN_DUTY_SEQUENCE, 50);
            perform_duty_cycle(&mut sleepy, SLEEPY_DUTY_SEQUENCE, 50);
            perform_duty_cycle(&mut status, STATUS_DUTY_SEQUENCE, 50);
            unsafe {
                info!("sleeping");
                _ = status.disable(); // we shouldn't need this.
                _ = action_led.disable();
                esp_idf_sys::gpio_reset_pin(SLEEPY_LED);
                esp_idf_sys::gpio_set_direction(SLEEPY_LED, GPIO_MODE_DEF_OUTPUT);
                FreeRtos.delay_us(500u16);
                esp_idf_sys::gpio_output_set(1 << SLEEPY_LED, 1 << 0xf, 0, 0);
                esp_idf_sys::gpio_hold_en(SLEEPY_LED);
                esp_idf_sys::gpio_deep_sleep_hold_en();
                esp_idf_sys::esp_deep_sleep_start();
            }
        }
        if action_button.is_high().unwrap() {
            info!("led button pressed");
            _ = status.disable();
            display.clear();
            // Wait for a bit.
            let wavelength =
                WavelengthState::get(powered_adc.read(&mut wavelength_sensor).unwrap());
            FreeRtos.delay_ms(100u32);
            let moisture = MoistureState::get(powered_adc.read(&mut moisture_sensor).unwrap());
            FreeRtos.delay_ms(100u32);
            info!("moisture: {:#?}", moisture);
            draw_display_text(
                &mut display,
                format!(
                    "Dirtplug reading\n\nearth: {:#?}\nlight: {:#?}\n{}\n{}",
                    moisture,
                    wavelength,
                    get_moisture_status_message(&moisture),
                    get_wavelength_status_message(&wavelength)
                )
                .as_str(),
            )
            .unwrap();
            display
                .flush()
                .map_err(|e| info!("Flush error: {:?}", e))
                .unwrap();
            perform_duty_cycle(&mut action_led, BLINKY_DUTY_SEQUENCE, 50);
        } else {
            // The main action.
            _ = action_led.disable();
            perform_duty_cycle(&mut status, STATUS_DUTY_SEQUENCE, 10);
            display.clear();
            draw_welcome_text(&mut display).unwrap();
            display
                .flush()
                .map_err(|e| info!("Flush error: {:?}", e))
                .unwrap();
            info!("initialized display. should be on now");
        }
        FreeRtos.delay_ms(TICK_INTERVAL_MS);
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
    P: esp_idf_hal::gpio::OutputPin,
{
    _ = channel.enable();
    for divisor in sequence.iter() {
        _ = channel.set_duty(channel.get_max_duty() / divisor);
        FreeRtos.delay_ms(delay_ms);
    }
}

// #[no_mangle]
// extern "C" fn wake_stub() -> () {
//     ()
// }

#[allow(dead_code)]
fn draw_display_text<D>(display: &mut D, text: &str) -> Result<(), D::Error>
where
    D: DrawTarget + Dimensions,
    D::Color: From<Rgb565>,
{
    display.clear(Rgb565::BLACK.into())?;
    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::BLUE.into())
                .stroke_color(Rgb565::YELLOW.into())
                .stroke_width(1)
                .build(),
        )
        .draw(display)?;

    Text::new(
        text,
        // oint::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
        Point::new(10, 10),
        MonoTextStyle::new(&FONT_5X8, Rgb565::WHITE.into()),
    )
    .draw(display)?;
    info!("LED rendering done");
    Ok(())
}

fn draw_welcome_text<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget + Dimensions,
    <D as embedded_graphics::draw_target::DrawTarget>::Error: core::fmt::Debug,
    D::Color: From<Rgb565>,
{
    let yoffset = 10;
    let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::YELLOW.into(), 1);
    // Draw a triangle.
    Triangle::new(
        Point::new(16, 16 + yoffset),
        Point::new(16 + 16, 16 + yoffset),
        Point::new(16 + 8, yoffset),
    )
    .into_styled(thin_stroke)
    .draw(display)
    .unwrap();

    Text::new(
        "Ready da ngotha.\nPress button to measure.",
        Point::new(10, 40),
        MonoTextStyle::new(&FONT_5X8, Rgb565::WHITE.into()),
    )
    .draw(display)?;

    Ok(())
}

fn get_moisture_status_message(status: &MoistureState) -> &'static str {
    match *status {
        MoistureState::DampSoil(_) => ":D",
        MoistureState::MediumDampSoil(_) => ":)",
        MoistureState::DrySoil(_) => ":(",
        MoistureState::DryAir(_) => "X|",
        MoistureState::TapWater(_) => ":|",
        MoistureState::Unknown(_) => ":?",
    }
}

fn get_wavelength_status_message(status: &WavelengthState) -> &'static str {
    match *status {
        WavelengthState::Optimal(_) => "O",
        WavelengthState::TooBright(_) => "B",
        WavelengthState::TooDark(_) => "D",
    }
}
