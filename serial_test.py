import logging
import time

import serial

logging.basicConfig(
    format="%(asctime)s %(levelname)-8s %(message)s",
    level=logging.INFO,
    datefmt="%Y-%m-%d %H:%M:%S",
)

uart_port = "/dev/tty.usbserial-140"
baud_rate = 115200
timeout = 1

ser = serial.Serial(uart_port, baud_rate, timeout=timeout)


def send_command(command):
    ser.write((command + "\r\n").encode())
    time.sleep(0.1)

    # Read and log all incoming serial data
    read_serial_data()


def read_serial_data():
    """Read and log all incoming lines from the serial buffer."""
    while ser.in_waiting > 0:  # Check if there is data waiting
        line = ser.readline().decode().strip()
        logging.info(f"Incoming serial data: {line}")


i = 0
force_value = 0
command = f"force set 5"
send_command(command)
while True:
    i += 1
    logging.info(f"====== LOOP {i} ======")

    command = "speed get"
    send_command(command)

    time.sleep(0.05)

    command = f"force set {force_value}"
    send_command(command)

    force_value += 0.1

    time.sleep(0.25)

    # Ensure any remaining serial data is read
    read_serial_data()

ser.close()
