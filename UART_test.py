import serial
import RPi.GPIO as GPIO

RS485_DIR_PIN = 17          # GPIO17, управляет DE+RE
GPIO.output(RS485_DIR_PIN, GPIO.LOW)


def set_receive_mode():
    # DE/RE = 0 -> прием
    GPIO.output(RS485_DIR_PIN, GPIO.LOW)
    # можно без задержки, но оставим
    time.sleep(DIR_SWITCH_DELAY)
    
ser = serial.Serial(
    port='/dev/serial0',   # UART порт
    baudrate=2000000,
    timeout=1
)

print("Waiting for data...")

while True:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RS485_DIR_PIN, GPIO.OUT, initial=GPIO.LOW)  # сразу прием
    
    set_receive_mode()
    
    if ser.in_waiting > 0:
        data = ser.read(1)           # читаем 1 байт
        char = data.decode('utf-8', errors='ignore')
        print(f"Received: {char}")
    set_receive_mode()
