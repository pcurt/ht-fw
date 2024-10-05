#![no_std]
#![no_main]
mod cli;
use core::cell::RefCell;

use cli::{get_force, set_speed};
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{GpioPin, Input, Io, Pull},
    ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
    uart::Uart,
};

static COUNTER: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let dial_io = io.pins.gpio4;
    let dial: Input<'_, GpioPin<4>> = Input::new(dial_io, Pull::None);
    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let force = io.pins.gpio0;

    let mut forcec = Ledc::new(peripherals.LEDC, &clocks);

    forcec.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = forcec.get_timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 16.kHz(),
        })
        .unwrap();

    let mut channel0 = forcec.get_channel(channel::Number::Channel0, force);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    spawner.spawn(detect_dial(dial)).unwrap();

    // Start the CLI task
    let serial0 = Uart::new(peripherals.UART0, &clocks, io.pins.gpio21, io.pins.gpio20).unwrap();
    spawner.spawn(cli::cli_run(serial0)).ok();

    let mut prev_cnt: u64 = 0;
    let mut last_time_change_s = Instant::now();

    loop {
        let cnt = critical_section::with(|cs| *COUNTER.borrow_ref(cs));

        if cnt != prev_cnt {
            // we are mooving
            prev_cnt = cnt;
            last_time_change_s = Instant::now();
        }
        // If wheel is not mooving since more than 800ms send a 0kms speed
        if (Instant::now().as_millis() - last_time_change_s.as_millis()) > 800 {
            set_speed(0);
        }

        // Update the force
        let _ = channel0.set_duty(100 - (get_force()));
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn detect_dial(mut dial: Input<'static, GpioPin<4>>) {
    let mut last_time_change_s = Instant::now();
    let mut prev_cnt: u64 = 0;
    let mut cnt: u64 = 0;
    loop {
        dial.wait_for_any_edge().await;

        cnt = cnt + 1;
        if (cnt - prev_cnt) > 100 {
            // Update the speed
            let current_time_s = Instant::now();
            let delta_t_us = current_time_s.as_micros() - last_time_change_s.as_micros();
            let speed_raw = 10_000_000 / delta_t_us;
            last_time_change_s = current_time_s;
            prev_cnt = cnt;
            // drop the wrong sensor measures
            if 2_000_000 > delta_t_us {
                critical_section::with(|cs| {
                    *COUNTER.borrow_ref_mut(cs) += 1;
                });
                set_speed(speed_raw as u32);
            }
        }
    }
}
