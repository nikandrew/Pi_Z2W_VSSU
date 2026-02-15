#!/usr/bin/env python3
"""
RS485_test.py

Raspberry Pi Zero 2W: ожидание сообщения "ST" по RS485 (UART 2 Мбит/с),
ответ "Recording_started". RE и DE трансивера объединены на GPIO17.
Ожидание ввода реализовано через select().
"""

import select
import sys

try:
    import serial
except ImportError:
    print("Установите pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None
    print(
        "Предупреждение: RPi.GPIO не найден. DE/RE (GPIO17) не будет управляться.",
        file=sys.stderr,
    )

# ---------------------- Настройки ----------------------

UART_PORT = "/dev/serial0"
UART_BAUDRATE = 2_000_000
RS485_DE_RE_GPIO = 17

EXPECTED_MSG = b"ST"
REPLY_MSG = b"Recording_started"

SELECT_TIMEOUT = 1.0  # таймаут select в секундах (для периодической проверки выхода)


def rs485_init() -> None:
    """GPIO17 в режим приёма (LOW)."""
    if GPIO is None:
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RS485_DE_RE_GPIO, GPIO.OUT, initial=GPIO.LOW)


def rs485_receive() -> None:
    """Режим приёма (DE/RE = LOW)."""
    if GPIO is not None:
        GPIO.output(RS485_DE_RE_GPIO, GPIO.LOW)


def rs485_transmit() -> None:
    """Режим передачи (DE/RE = HIGH)."""
    if GPIO is not None:
        GPIO.output(RS485_DE_RE_GPIO, GPIO.HIGH)


def rs485_cleanup() -> None:
    if GPIO is not None:
        try:
            GPIO.cleanup(RS485_DE_RE_GPIO)
        except Exception:
            pass


def main() -> int:
    rs485_init()
    rs485_receive()

    try:
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=UART_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )
    except serial.SerialException as e:
        print(f"Ошибка открытия порта {UART_PORT}: {e}", file=sys.stderr)
        rs485_cleanup()
        return 1

    fd = ser.fileno()
    buffer = bytearray()

    print(f"Порт: {UART_PORT}, {UART_BAUDRATE} бод. DE/RE → GPIO{RS485_DE_RE_GPIO}")
    print(f"Ожидание сообщения {EXPECTED_MSG!r}. Ответ: {REPLY_MSG!r}")
    print("Выход: Ctrl+C")

    try:
        while True:
            readable, _, _ = select.select([fd], [], [], SELECT_TIMEOUT)
            if not readable:
                continue

            try:
                data = ser.read(4096)
            except serial.SerialException as e:
                print(f"Ошибка чтения: {e}", file=sys.stderr)
                break

            if not data:
                continue

            buffer.extend(data)
            if len(buffer) > 2048:
                buffer = buffer[-1024:]

            if EXPECTED_MSG in buffer:
                idx = buffer.find(EXPECTED_MSG)
                del buffer[: idx + len(EXPECTED_MSG)]

                rs485_transmit()
                try:
                    ser.write(REPLY_MSG)
                    ser.flush()
                finally:
                    rs485_receive()
                print(f"Получено {EXPECTED_MSG!r} → отправлено {REPLY_MSG!r}")
            else:
                try:
                    text = data.decode("utf-8", errors="replace")
                except Exception:
                    text = str(bytes(data))
                print(f"Получено не ST. Содержимое: {text!r}")

    except KeyboardInterrupt:
        print("\nОстановка по Ctrl+C.")
    finally:
        ser.close()
        rs485_cleanup()
        print("Порт закрыт.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
