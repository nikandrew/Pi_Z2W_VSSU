import serial

RS485_DIR_PIN = 17          # GPIO17, управляет DE+RE
GPIO.output(RS485_DIR_PIN, GPIO.LOW)

ser = serial.Serial(
    port='/dev/serial0',   # UART порт
    baudrate=115200,
    timeout=1
)

print("Waiting for data...")

while True:
    if ser.in_waiting > 0:
        data = ser.read(1)           # читаем 1 байт
        char = data.decode('utf-8', errors='ignore')
        print(f"Received: {char}")