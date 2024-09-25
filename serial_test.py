import serial
import time

uart_port = '/dev/tty.usbserial-140'
baud_rate = 115200
timeout = 1

ser = serial.Serial(uart_port, baud_rate, timeout=timeout)

def send_command(command):
    ser.write((command + '\r\n').encode())
    time.sleep(0.1)

    echo = ser.readline().decode().strip()
    response = ser.readline().decode().strip()

    return echo, response


while True:
    command = "speed get"
    echo, response = send_command(command)

    print(f"Commande sent (echo) : {echo}")
    print(f"Response : {response}")
    
    time.sleep(0.2)
	
    command = "force set 10.0"
    echo, response = send_command(command)
    
    time.sleep(0.2)

ser.close()

