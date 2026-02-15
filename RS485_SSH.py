#!/usr/bin/env python3
"""
Ожидание команды "start" по RS485 (последовательный порт).
При получении "start" — ответ "Test" в тот же RS485.
RE и DE на GPIO17.
Чтение по таймауту (select на Linux часто не срабатывает на serial).
"""

import sys
import time

from gpiozero import DigitalOutputDevice
import serial

RE_DE_PIN = 17
PORT = "/dev/serial0"
BAUDRATE = 2_000_000

EXPECTED_CMD = b"start"
REPLY_MSG = b"Test"
READ_TIMEOUT = 0.05  # с — опрос порта; при данных они сразу читаются

# Вывод сырых байт в hex для отладки (осциллограф vs программа)
DEBUG_HEX = True

# GPIO: RE+DE
re_de = DigitalOutputDevice(RE_DE_PIN)
re_de.off()  # приём по умолчанию

# UART: короткий timeout, чтобы не блокироваться и не зависеть от select()
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=READ_TIMEOUT,
)


def send_rs485(data: bytes) -> None:
    re_de.on()
    time.sleep(0.001)
    ser.write(data)
    ser.flush()
    time.sleep(0.001)
    re_de.off()


def main() -> int:
    buffer = bytearray()

    print(f"Порт: {PORT}, {BAUDRATE} бод. Ожидание команды {EXPECTED_CMD!r} по RS485.")
    print("Если данных нет — проверьте: 1) скорость передатчика = 2 Мбит/с  2) порт (ttyAMA0/serial0).")
    if DEBUG_HEX:
        print("DEBUG: вывод сырых байт (hex) включён.")
    print("Выход: Ctrl+C")

    try:
        while True:
            try:
                data = ser.read(4096)
            except serial.SerialException as e:
                print(f"Ошибка чтения: {e}", file=sys.stderr)
                return 1

            if not data:
                continue

            try:
                text = data.decode("utf-8", errors="replace")
            except Exception:
                text = str(bytes(data))
            print(f"Получено по RS485: {text!r}")
            if DEBUG_HEX:
                print(f"  hex: {data.hex()!r}")

            buffer.extend(data)
            if len(buffer) > 2048:
                buffer = buffer[-1024:]

            if EXPECTED_CMD in buffer:
                idx = buffer.find(EXPECTED_CMD)
                del buffer[: idx + len(EXPECTED_CMD)]
                send_rs485(REPLY_MSG)
                print(f"Получено {EXPECTED_CMD!r} → отправлено {REPLY_MSG!r}")

    except KeyboardInterrupt:
        print("\nОстановка по Ctrl+C.")
    finally:
        ser.close()
        re_de.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
