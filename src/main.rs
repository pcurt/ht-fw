#![no_std]
#![no_main]
mod cli;

use cli::{get_force, set_speed};
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
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
    let dial: Input<'_, GpioPin<4>> = Input::new(dial_io, Pull::Up);
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

    loop {
        // Update the force
        let _ = channel0.set_duty(100 - (get_force()));
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn detect_dial(mut dial: Input<'static, GpioPin<4>>) {
    let mut zero_count = 0;
    let mut last_non_zero_speed = 0;
    let timeout_duration = Duration::from_secs(1); // 5 second timeout

    loop {
        // Wait for the dial to go high (rising edge) with a timeout
        match with_timeout(timeout_duration, dial.wait_for_high()).await {
            Ok(_) => {
                let start = Instant::now();
                // Wait for the dial to go low (falling edge) with a timeout
                match with_timeout(timeout_duration, dial.wait_for_low()).await {
                    Ok(_) => {
                        let duration = start.elapsed();
                        // Debounce: Ignore any pulses shorter than 1000 microseconds
                        if duration.as_micros() >= 1000 {
                            let speed: f32 = 29700.0 / duration.as_micros() as f32;
                            let scaled_speed = (speed * 1000.0) as u32;

                            if speed == 0.000 {
                                zero_count += 1;
                                if zero_count >= 5 {
                                    set_speed(0);
                                    last_non_zero_speed = 0;
                                } else {
                                    set_speed(last_non_zero_speed);
                                }
                            } else {
                                zero_count = 0;
                                last_non_zero_speed = scaled_speed;
                                set_speed(scaled_speed);
                            }
                        }
                    }
                    Err(_) => {
                        // Timeout occurred while waiting for low
                        set_speed(0);
                        last_non_zero_speed = 0;
                    }
                }
            }
            Err(_) => {
                // Timeout occurred while waiting for high
                set_speed(0);
                last_non_zero_speed = 0;
            }
        }
    }
}
