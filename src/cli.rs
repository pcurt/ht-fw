use core::convert::Infallible;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use embassy_time::{Duration, Timer};
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use esp_hal::peripherals::UART0;
use esp_hal::uart::{Uart, UartTx};
use esp_hal::Blocking;
use esp_println::println;

// SPEED in 0,1 km/h
static SPEED: AtomicU32 = AtomicU32::new(0);
// FORCE
static FORCE: AtomicU8 = AtomicU8::new(0);

// Fonction setter pour modifier la variable
pub fn set_speed(new_sped: u32) {
    SPEED.store(new_sped, Ordering::Relaxed);
}

// Fonction setter pour modifier la variable
pub fn get_force() -> u8 {
    FORCE.load(Ordering::Relaxed)
}

#[derive(Command)]
enum Base<'a> {
    /// Speed cli command
    Speed {
        #[command(subcommand)]
        command: SpeedCommand,
    },

    /// Force cli command
    Force {
        #[command(subcommand)]
        command: ForceCommand<'a>,
    },
}

#[derive(Debug, Command)]
enum SpeedCommand {
    /// Get current speed value
    Get,
}

#[derive(Debug, Command)]
enum ForceCommand<'a> {
    /// Get force value
    Get,

    /// Set force value
    Set { value: &'a str },
}

/// Wrapper around usart so we can impl embedded_io::Write
/// which is required for cli
struct Writer(UartTx<'static, esp_hal::peripherals::UART0, Blocking>);

impl embedded_io::ErrorType for Writer {
    type Error = Infallible;
}

impl embedded_io::Write for Writer {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let size = self.0.write(buf).unwrap();
        Ok(size)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn handle_speed_command(command: SpeedCommand) -> Result<(), Infallible> {
    match command {
        SpeedCommand::Get => {
            let shared_speed = SPEED.load(Ordering::Relaxed);
            let speed_km_h: f32 = shared_speed as f32 / 10.0;
            println!("speed {}", speed_km_h);
        }
    }
    Ok(())
}

fn handle_force_command(command: ForceCommand) -> Result<(), Infallible> {
    match command {
        ForceCommand::Get => {
            let shared_force = FORCE.load(Ordering::Relaxed);
            let speed_km_float: f32 = shared_force as f32 / 100.0;
            println!("force {}", speed_km_float);
        }
        ForceCommand::Set { value } => {
            let resultat: Result<f32, _> = value.parse();
            match resultat {
                // Force is a float between 0 and 20
                // Multiply it by 100 to have an integer
                Ok(read_force) => {
                    let mut duty: u8 = read_force as u8 * 5;
                    if duty > 100 {
                        duty = 100;
                    }
                    FORCE.store(duty, Ordering::Relaxed);
                }
                Err(e) => println!("Not a float : {}", e),
            }
        }
    }
    Ok(())
}

#[embassy_executor::task]
pub async fn cli_run(uart: Uart<'static, UART0, Blocking>) {
    let (tx, mut rx) = uart.split();
    let writer = Writer(tx);

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 64] = [0; 64];
        static mut HISTORY_BUFFER: [u8; 64] = [0; 64];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };
    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build()
        .ok()
        .unwrap();

    loop {
        // Read next byte
        let result = rx.read_byte();
        if let Ok(value) = result {
            let _ = cli.process_byte::<Base, _>(
                value,
                &mut Base::processor(|_cli, command| {
                    let _ = match command {
                        Base::Speed { command } => handle_speed_command(command),
                        Base::Force { command } => handle_force_command(command),
                    };
                    Ok(())
                }),
            );
        }
        // Wait 10ms between each byte read, bytes are buffered by peripheral
        Timer::after(Duration::from_millis(10)).await;
    }
}
