import time
import serial
import RPi.GPIO as GPIO

# Настройки
UART_PORT = "/dev/serial0"  # чаще всего так на Pi
BAUDRATE = 1000000       # 1000000 бод
RS485_DIR_PIN = 17          # GPIO17, управляет DE+RE

# Время на переключение направления (подберите при необходимости)
DIR_SWITCH_DELAY = 0.001    # 1 мс

def set_receive_mode():
    # DE/RE = 0 -> прием
    GPIO.output(RS485_DIR_PIN, GPIO.LOW)
    # можно без задержки, но оставим
    time.sleep(DIR_SWITCH_DELAY)

def set_transmit_mode():
    # DE/RE = 1 -> передача
    GPIO.output(RS485_DIR_PIN, GPIO.HIGH)
    time.sleep(DIR_SWITCH_DELAY)

def main():
    # Настройка GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RS485_DIR_PIN, GPIO.OUT, initial=GPIO.LOW)  # сразу прием
    set_receive_mode()

    # Открываем UART
    ser = serial.Serial(
        port=UART_PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=None   # ждать бесконечно
    )

    print("RS485 сервер запущен. Жду один байт...")

    try:
        while True:
            # Режим прием
            set_receive_mode()

            # Ждем 1 байт
            data = ser.read(1)   # блокирующий вызов, ждёт ровно один байт

            if not data:
                continue  # на всякий случай

            print(f"Получен байт: {data.hex()}")

            # Подготовка к передаче
            set_transmit_mode()

            # Отправляем 'ok'
            reply = b"ok"
            ser.write(reply)
            ser.flush()  # дождаться отправки

            # Возвращаемся в режим приема
            set_receive_mode()

    except KeyboardInterrupt:
        print("Остановка по Ctrl+C")
    finally:
        ser.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
