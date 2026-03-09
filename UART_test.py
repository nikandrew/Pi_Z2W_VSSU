import serial

ser = serial.Serial(
    port="/dev/serial0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=None
)

print("Жду байт на /dev/serial0 (9600)...")

while True:
    data = ser.read(1)
    print("Получено:", data, data.hex())