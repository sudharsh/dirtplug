// MIT License
//
// Copyright (c) 2022 - Sudharshan S <sudharsh@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#![no_std]
#![no_main]

#[macro_use]
extern crate alloc;

extern crate embedded_hal as hal;

use core::borrow::Borrow;
use core::default::Default;
use core::fmt;
use core::ops::Range;

use hal::digital::v2::{InputPin, OutputPin};
use hal::prelude::*;

use esp_idf_hal::adc::{Atten11dB, PoweredAdc, ADC1};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::config::MasterConfig;
use esp_idf_hal::i2c::*;
use esp_idf_hal::ledc::{
    config::TimerConfig, Channel, HwChannel, HwTimer, Peripheral, Timer, CHANNEL0, CHANNEL1,
    CHANNEL2, TIMER0, TIMER1, TIMER2,
};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::{adc, prelude::*};
use esp_idf_sys::{
    self, esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH as GPIO_HIGH, gpio_hold_dis,
    GPIO_MODE_DEF_OUTPUT,
};

use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::*;

use embedded_graphics::geometry::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    primitives::{PrimitiveStyle, Triangle},
    text::Text,
};
use embedded_graphics::mono_font::{ascii::FONT_5X8, MonoTextStyle};

use log::*;

// Tick interval between each iteration in the simple main loop.
const TICK_INTERVAL_MS: u16 = 100u16;

// Named LED Duty Sequences
const SLEEP_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 3, 5, 8, 13, 21, 33];
const STATUS_DUTY_SEQUENCE: &[u32] = &[1, 1, 2, 2, 3, 3, 3, 5, 5, 5, 5, 5, 3, 3, 3, 2, 2, 1, 1];

// Pins
const SLEEP_LED: i32 = 0x7;
const SLEEP_WAKEUP_PIN_MASK: u64 = 1 << 0x3;

// AOUT ranges from the sensor.
// Requires calibration.
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
    Unplugged(u16),
    Unknown(u16),
}

impl fmt::Display for MoistureState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            MoistureState::DampSoil(_) => write!(f, ":D"),
            MoistureState::MediumDampSoil(_) => write!(f, ":)"),
            MoistureState::DrySoil(_) => write!(f, ":("),
            MoistureState::Unplugged(_) => write!(f, "X|"),
            MoistureState::TapWater(_) => write!(f, ":|"),
            MoistureState::Unknown(_) => write!(f, ":?"),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
enum WavelengthState {
    Optimal(u16),
    TooDark(u16), // FIXME: better states
    TooBright(u16),
}

impl fmt::Display for WavelengthState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            WavelengthState::Optimal(_) => write!(f, "O"),
            WavelengthState::TooBright(_) => write!(f, "B"),
            WavelengthState::TooDark(_) => write!(f, "D"),
        }
    }
}

impl From<u16> for WavelengthState {
    fn from(reading: u16) -> Self {
        Self::Optimal(reading)
        // FIXME: Rest.
    }
}

/// Lights for the device.
struct Lights {
    action_led: Channel<CHANNEL0, TIMER0, Timer<TIMER0>, Gpio10<Unknown>>,
    sleepy_led: Channel<CHANNEL1, TIMER1, Timer<TIMER1>, Gpio7<Unknown>>,
    status_led: Channel<CHANNEL2, TIMER2, Timer<TIMER2>, Gpio0<Unknown>>,
}

impl Lights {
    fn initialize_from_ledc(
        ledc: Peripheral,
        action_pin: Gpio10<Unknown>,
        sleep_pin: Gpio7<Unknown>,
        status_pin: Gpio0<Unknown>,
    ) -> Self {
        let config = TimerConfig::default().frequency(25.kHz().into());
        // Three LEDS, action, sleepy and status.
        let action_timer = Timer::new(ledc.timer0, &config).unwrap();
        let sleepy_timer = Timer::new(ledc.timer1, &config).unwrap();
        let status_timer = Timer::new(ledc.timer2, &config).unwrap();
        Lights {
            action_led: Channel::new(ledc.channel0, action_timer, action_pin).unwrap(),
            sleepy_led: Channel::new(ledc.channel1, sleepy_timer, sleep_pin).unwrap(),
            status_led: Channel::new(ledc.channel2, status_timer, status_pin).unwrap(),
        }
    }
}

/// A bag of sensors.
struct Sensors {
    powered_adc: PoweredAdc<ADC1>,
    uvb: Gpio1<Atten11dB<ADC1>>,
    moisture: Gpio2<Atten11dB<ADC1>>,
}

impl Sensors {
    fn initialize_from_pins(
        adc1: ADC1,
        wavelength: Gpio1<Unknown>,
        moisture: Gpio2<Unknown>,
    ) -> Sensors {
        let adc1 =
            adc::PoweredAdc::new(adc1, adc::config::Config::new().calibration(true)).unwrap();
        Sensors {
            powered_adc: adc1,
            moisture: moisture.into_analog_atten_11db().unwrap(),
            uvb: wavelength.into_analog_atten_11db().unwrap(),
        }
    }

    fn read_moisture(&mut self) -> MoistureState {
        self.powered_adc.read(&mut self.moisture).unwrap().into()
    }

    fn read_wavelength(&mut self) -> WavelengthState {
        self.powered_adc.read(&mut self.uvb).unwrap().into()
    }
}

/// A bag of buttons
struct Buttons {
    action: Gpio3<Input>,
    sleep: Gpio6<Input>,
}

impl Buttons {
    /// Initialize buttons from the set of pins.
    fn initialize_from_pins(action: Gpio3<Unknown>, sleep: Gpio6<Unknown>) -> Buttons {
        let mut action = action.into_input().unwrap();
        let mut sleep = sleep.into_input().unwrap();
        action
            .set_pull_down()
            .map_err(|e| info!("error setting pulldown for action: {:?}", e))
            .unwrap();
        sleep
            .set_pull_up()
            .map_err(|e| info!("error setting pullup for sleep: {:?}", e))
            .unwrap();
        Buttons { action, sleep }
    }

    fn is_sleep_active(&self) -> bool {
        self.sleep.is_low().unwrap()
    }

    fn is_action_active(&self) -> bool {
        self.action.is_high().unwrap()
    }
}

struct MainDisplay {
    pub power: Gpio19<Output>,
    handle: ssd1306::Ssd1306<
        I2CInterface<Master<I2C0, Gpio5<Unknown>, Gpio4<Unknown>>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
}

impl MainDisplay {
    /// Returns an unitialized `MaindDisplay`.
    fn initialize_from_pins(
        i2c: I2C0,
        scl: Gpio4<Unknown>,
        sda: Gpio5<Unknown>,
        power: Gpio19<Unknown>,
    ) -> MainDisplay {
        let handle = ssd1306::I2CDisplayInterface::new(
            Master::<I2C0, _, _>::new(
                i2c,
                MasterPins { sda, scl },
                MasterConfig::default().baudrate(400.kHz().into()),
            )
            .unwrap(),
        );
        let mut m = MainDisplay {
            power: power.into_output().expect("couldn't get hold of gpio19"),
            handle: ssd1306::Ssd1306::new(
                handle,
                ssd1306::size::DisplaySize128x64,
                ssd1306::rotation::DisplayRotation::Rotate0,
            )
            .into_buffered_graphics_mode(),
        };
        m.power.set_drive_strength(DriveStrength::I40mA).unwrap();
        m.power.set_high().unwrap();
        FreeRtos.delay_ms(100u32);
        m.handle
            .init()
            .map_err(|e| info!("display init error: {:?}", e))
            .unwrap();
        m.flush();
        m
    }

    fn clear(&mut self) {
        self.handle.clear();
    }

    fn flush(&mut self) {
        self.handle
            .flush()
            .map_err(|e| info!("flush error: {:?}", e))
            .unwrap();
    }

    fn draw_text(&mut self, text: &str) -> Result<(), ()> {
        Text::new(
            text,
            Point::new(10, 10),
            MonoTextStyle::new(&FONT_5X8, Rgb565::WHITE.into()),
        )
        .draw(&mut self.handle)
        .unwrap();
        Ok(())
    }

    fn draw_ready(&mut self) -> Result<(), ()> {
        let yoffset = 10;
        let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::YELLOW.into(), 1);
        // Draw a triangle.
        Triangle::new(
            Point::new(16, 16 + yoffset),
            Point::new(16 + 16, 16 + yoffset),
            Point::new(16 + 8, yoffset),
        )
        .into_styled(thin_stroke)
        .draw(&mut self.handle)
        .unwrap();

        Text::new(
            "Ready da ngotha.\nPress `action` button to measure.\n<3",
            Point::new(10, 40),
            MonoTextStyle::new(&FONT_5X8, Rgb565::WHITE.into()),
        )
        .draw(&mut self.handle)
        .unwrap();
        self.flush();

        Ok(())
    }
}

struct Device {
    display: MainDisplay,
    status_lights: Lights,
    buttons: Buttons,
    sensors: Sensors,
}

impl Device {
    fn init_from_peripherals(peripherals: Peripherals) -> Self {
        Device {
            status_lights: Lights::initialize_from_ledc(
                peripherals.ledc,
                peripherals.pins.gpio10,
                peripherals.pins.gpio7,
                peripherals.pins.gpio0,
            ),
            buttons: Buttons::initialize_from_pins(peripherals.pins.gpio3, peripherals.pins.gpio6),
            sensors: Sensors::initialize_from_pins(
                peripherals.adc1,
                peripherals.pins.gpio1,
                peripherals.pins.gpio2,
            ),
            display: MainDisplay::initialize_from_pins(
                peripherals.i2c0,
                peripherals.pins.gpio4,
                peripherals.pins.gpio5,
                peripherals.pins.gpio19,
            ),
        }
    }

    fn moisture(&mut self) -> MoistureState {
        self.sensors.read_moisture()
    }

    fn wavelength(&mut self) -> WavelengthState {
        self.sensors.read_wavelength()
    }
}

impl From<u16> for MoistureState {
    fn from(reading: u16) -> Self {
        if HARD_TAP_WATER_RANGE.contains(&(reading as u32)) {
            MoistureState::TapWater(reading)
        } else if DAMP_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::DampSoil(reading)
        } else if MED_DAMP_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::MediumDampSoil(reading)
        } else if DRY_SOIL_RANGE.contains(&(reading as u32)) {
            MoistureState::DrySoil(reading)
        } else if DRY_AIR_RANGE.contains(&(reading as u32)) {
            MoistureState::Unplugged(reading)
        } else {
            MoistureState::Unknown(reading)
        }
    }
}

impl MoistureState {
    fn duty_cycle(&self) -> &[u32] {
        match *self {
            MoistureState::TapWater(_) => &[2, 1, 2, 1, 1, 1],
            MoistureState::DampSoil(_) => &[3, 3, 3, 3, 2, 1, 1, 3, 3, 3, 3, 1, 1, 2],
            MoistureState::MediumDampSoil(_) => &[4, 4, 2, 2, 2, 2, 2],
            MoistureState::DrySoil(_) => &[5, 5, 5, 5, 4, 4, 4, 3, 3],
            MoistureState::Unknown(_) => &[50],
            MoistureState::Unplugged(_) => &[10, 8, 8, 8, 6, 6, 66, 666, 6666],
        }
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
        esp_idf_sys::esp_deep_sleep_enable_gpio_wakeup(SLEEP_WAKEUP_PIN_MASK, GPIO_HIGH);
        gpio_hold_dis(SLEEP_LED); // Disable held state when resuming from deep sleep.
    }

    info!("setting up soil check");
    let peripherals = Peripherals::take().unwrap();
    let mut device = Device::init_from_peripherals(peripherals);
    FreeRtos.delay_ms(30u32);
    device.display.clear();
    device.display.draw_ready().unwrap();
    info!("initialized display");
    info!("delaying a bit");
    FreeRtos.delay_ms(100u32);
    // main loop with a tick.
    loop {
        if device.buttons.is_sleep_active() {
            device.display.clear();
            device.display.flush();
            device.display.draw_text("sleeping... bye.\n:O").unwrap();
            device.display.flush();
            perform_duty_cycle(
                &mut device.status_lights.action_led,
                SLEEP_DUTY_SEQUENCE,
                50,
            );
            perform_duty_cycle(
                &mut device.status_lights.sleepy_led,
                SLEEP_DUTY_SEQUENCE,
                50,
            );
            perform_duty_cycle(
                &mut device.status_lights.status_led,
                SLEEP_DUTY_SEQUENCE,
                50,
            );
            device.display.clear();
            device.display.flush();
            unsafe {
                info!("sleeping");
                _ = device.status_lights.status_led.disable(); // we shouldn't need this.
                _ = device.status_lights.action_led.disable();
                esp_idf_sys::gpio_reset_pin(SLEEP_LED);
                esp_idf_sys::gpio_set_direction(SLEEP_LED, GPIO_MODE_DEF_OUTPUT);
                FreeRtos.delay_us(500u16);
                esp_idf_sys::gpio_output_set(1 << SLEEP_LED, 1 << 0xf, 0, 1 << 19);
                esp_idf_sys::gpio_hold_en(SLEEP_LED);
                esp_idf_sys::gpio_deep_sleep_hold_en();
                esp_idf_sys::esp_deep_sleep_start();
            }
        }
        if device.buttons.is_action_active() {
            device.display.clear();
            info!("led button pressed");
            _ = device.status_lights.status_led.disable();
            // Wait for a bit.
            FreeRtos.delay_ms(100u32);
            let wavelength: WavelengthState = device.wavelength();
            FreeRtos.delay_ms(100u32);
            let moisture: MoistureState = device.moisture();
            FreeRtos.delay_ms(100u32);
            info!("moisture: {:#?}", moisture);
            device
                .display
                .draw_text(
                    format!(
                        "dirtplug reading\n\nmoisture: {:#?} - {}\nwavelength: {:#?} - {}",
                        moisture, &moisture, wavelength, &wavelength
                    )
                    .as_str(),
                )
                .unwrap();
            device.display.flush();
            perform_duty_cycle(
                &mut device.status_lights.action_led,
                moisture.duty_cycle(),
                50,
            );
            device.display.clear();
        } else {
            // The main action.
            perform_duty_cycle(
                &mut device.status_lights.status_led,
                STATUS_DUTY_SEQUENCE,
                10,
            );
            device.display.draw_ready().unwrap();
            _ = device.status_lights.action_led.disable();
            FreeRtos.delay_ms(TICK_INTERVAL_MS);
        }
    }
}

fn perform_duty_cycle<C, H, T, P>(
    channel: &mut Channel<C, H, T, P>,
    sequence: &[u32],
    delay_ms: u16,
) where
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
