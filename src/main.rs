#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    gpio::{GpioPin, Io},
    peripherals::{Peripherals, ADC1},
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};
use esp_println::println;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const THRESHOLD_FOR_WHITE_DIAL_MV_HIGH: u16 = 3000;
const THRESHOLD_FOR_WHITE_DIAL_MV_LOW: u16 = 500;

#[embassy_executor::task]
async fn compute_speed(analog_pin: GpioPin<3>, adc: ADC1) {
    // Create ADC instances
    type AdcCal = esp_hal::analog::adc::AdcCalBasic<esp_hal::peripherals::ADC1>;
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(analog_pin, Attenuation::Attenuation11dB);
    let mut adc1: Adc<'_, _> = Adc::new(adc, adc1_config);

    // Local variabels to manage states and durations
    let mut is_in_white_dial: bool = false;
    let mut prev_dial_state: bool = false;
    let mut last_time_change_s = Instant::now();
    let mut stop_is_sent: bool = false;

    // Initilaize first position
    let raw_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
    if raw_value < THRESHOLD_FOR_WHITE_DIAL_MV_LOW {
        is_in_white_dial = true;
        prev_dial_state = true;
    }

    loop {
        let raw_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();

        if raw_value > THRESHOLD_FOR_WHITE_DIAL_MV_HIGH {
            is_in_white_dial = false;
        }
        if raw_value < THRESHOLD_FOR_WHITE_DIAL_MV_LOW {
            is_in_white_dial = true;
        }

        if is_in_white_dial != prev_dial_state {
            //Update sate and compute speed
            stop_is_sent = false;
            prev_dial_state = is_in_white_dial;
            let current_time_s = Instant::now();

            // One dial represents 1,3cm
            let delta_t_us = (current_time_s.as_micros() - last_time_change_s.as_micros()) as f32;
            // Compute speed in cm/s on a float
            let speed = 1.30 * 10_000_000.0 / delta_t_us;
            // Compute speed in km/h on a u32
            let speed_km_h = (((speed * 3600.0) as u32) / 100_000) as u32;

            // Update last time change
            last_time_change_s = current_time_s;
            println!("SPEED_IS {:?};\r", speed_km_h);
        }

        // If wheel is not mooving since more than x seconds send a 0kms speed
        if (Instant::now().as_secs() - last_time_change_s.as_secs()) > 2 && !stop_is_sent {
            stop_is_sent = true;
            println!("SPEED_IS: 0;\r");
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    spawner
        .spawn(compute_speed(io.pins.gpio3, peripherals.ADC1))
        .ok();

    loop {
        // Nothing to do for the moment
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
