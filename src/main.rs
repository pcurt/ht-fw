#![no_std]
#![no_main]
mod cli;
use core::cell::RefCell;

use cli::set_speed;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{GpioPin, Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    uart::Uart,
};

static COUNTER: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

struct MovingAverage {
    samples: [u32; 5],
    index: usize,
    count: usize,
    sum: u32,
}

impl MovingAverage {
    fn new() -> Self {
        MovingAverage {
            samples: [0; 5],
            index: 0,
            count: 0,
            sum: 0,
        }
    }

    fn add_sample(&mut self, sample: u32) -> u32 {
        if self.count == 5 {
            self.sum -= self.samples[self.index];
        } else {
            self.count += 1;
        }

        self.samples[self.index] = sample;
        self.sum += sample;

        self.index = (self.index + 1) % 5;
        self.sum / self.count as u32
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
    let dial_io = io.pins.gpio4;
    let dial: Input<'_, GpioPin<4>> = Input::new(dial_io, Pull::None);
    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    spawner.spawn(detect_dial(dial)).unwrap();

    // Start the CLI task
    let serial0 = Uart::new(peripherals.UART0, &clocks, io.pins.gpio21, io.pins.gpio20).unwrap();
    spawner.spawn(cli::cli_run(serial0)).ok();

    let mut prev_cnt: u64 = 0;
    let mut last_time_change_s = Instant::now();
    let mut avg_speed = MovingAverage::new();

    loop {
        let cnt = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        if cnt != prev_cnt {
            // we are mooving
            let distance: u64 = cnt - prev_cnt;
            let current_time_s = Instant::now();
            let delta_t_us = current_time_s.as_micros() - last_time_change_s.as_micros();

            last_time_change_s = current_time_s;
            prev_cnt = cnt;
            let speed_raw = distance * 10_000_000 / delta_t_us;

            // Compute the relative speed in 0.1 km/h, coefficient has been set using the home trainer
            let speed_km_h: u32 = (speed_raw / 10) as u32;
            // Above 70kmh it must be an error on the sensor drop it
            let mean = avg_speed.add_sample(speed_km_h); // Average on last 5 samples
            set_speed(mean);
        }

        // If wheel is not mooving since more than 2 seconds send a 0kms speed
        if (Instant::now().as_secs() - last_time_change_s.as_secs()) > 2 {
            set_speed(0);
        }

        Timer::after(Duration::from_millis(20)).await;
    }
}

#[embassy_executor::task]
async fn detect_dial(mut dial: Input<'static, GpioPin<4>>) {
    loop {
        dial.wait_for_any_edge().await;
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) += 1;
        });
    }
}
