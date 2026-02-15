from gpiozero import DigitalOutputDevice
import serial
import time
import sys

RE_DE_PIN = 17
PORT = "/dev/serial0"
BAUDRATE = 9600

# GPIO: RE+DE
re_de = DigitalOutputDevice(RE_DE_PIN)
re_de.off()  # приём по умолчанию

# UART
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=1
)

def send_rs485(data: bytes):
    re_de.on()          # передача
    time.sleep(0.001)
    ser.write(data)
    ser.flush()
    time.sleep(0.001)
    re_de.off()         # обратно в приём

# --- ожидание команды ---
print("Введите 'start' и нажмите Enter")

while True:
    cmd = sys.stdin.buffer.readline().strip()
    if cmd == b"start":
        send_rs485(b"Test")
        print("Отправлено: Test")