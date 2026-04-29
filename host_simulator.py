#!/usr/bin/env python3
"""
Простой симулятор хоста для тестирования системы на Pi.
Отправляет команду 0x00 0x01 и ждет ответа по RS-485.

Использование (на другом компьютере):
  python3 host_simulator.py --port /dev/ttyUSB0 --baudrate 115200
"""

import argparse
import binascii
import sys
import time
from pathlib import Path
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Установите: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)


EXPECTED_REPLY = b"recording_complete"
DEFAULT_COMMAND = b"\x00\x01"


def crc32_iso_hdlc(data: bytes) -> int:
    """CRC-32/ISO-HDLC: poly 0x04C11DB7, init/xorout 0xffffffff, reflected."""
    return binascii.crc32(data) & 0xFFFFFFFF


def append_crc32_iso_hdlc(data: bytes) -> bytes:
    return data + crc32_iso_hdlc(data).to_bytes(4, "little")


def wait_us(delay_us: float) -> None:
    """Короткая busy-wait задержка для межбайтовых пауз."""
    if delay_us <= 0:
        return
    end_time = time.perf_counter() + delay_us / 1_000_000
    while time.perf_counter() < end_time:
        pass


def print_ports() -> None:
    """Печатает доступные последовательные порты."""
    ports = list(list_ports.comports())
    if not ports:
        print("COM/serial порты не найдены")
        return

    print("Доступные COM/serial порты:")
    for port in ports:
        print(f"  {port.device}: {port.description} [{port.hwid}]")


def send_command(
    port: str,
    baudrate: int,
    command: bytes,
    timeout: float = 35,
    expected_reply: bytes = EXPECTED_REPLY,
    open_delay: float = 0.2,
    post_write_delay: float = 0.05,
    inter_byte_delay_us: float = 50,
    rts: Optional[bool] = None,
    dtr: Optional[bool] = None,
) -> bool:
    """
    Отправляет команду и ждет ответа.
    """
    print(f"\n{'=' * 50}")
    print(f"Отправка команды на {port} ({baudrate} бод)")
    print(f"{'=' * 50}")
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        print(f"Порт открыт: {ser.name}")
        print(f"Параметры: {ser.baudrate} 8N1, timeout={ser.timeout}")

        if rts is not None:
            ser.rts = rts
            print(f"RTS установлен в {ser.rts}")
        if dtr is not None:
            ser.dtr = dtr
            print(f"DTR установлен в {ser.dtr}")

        if open_delay > 0:
            time.sleep(open_delay)

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        frame = append_crc32_iso_hdlc(command)
        print(f"\n[SEND] Отправка payload: {command}")
        print(f"       CRC-32/ISO-HDLC: 0x{crc32_iso_hdlc(command):08x}")
        print(f"       Кадр: {frame.hex()}")
        bytes_written = 0
        for byte in frame:
            bytes_written += ser.write(bytes([byte]))
            ser.flush()
            wait_us(inter_byte_delay_us)
        if post_write_delay > 0:
            time.sleep(post_write_delay)
        print(f"       Записано байт в COM-порт: {bytes_written}")
        print("       Команда отправлена. Если main.py сработал, ответ придет после записи видео.")
        
        # Ожидание ответа
        print(f"\n[WAIT] Ожидание ответа (таймаут {timeout} сек)...")
        start_time = time.time()
        response = b""
        
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"[RECV] Получено: {chunk}")
                if expected_reply and expected_reply in response:
                    break
            time.sleep(0.01)
        
        if expected_reply and expected_reply in response:
            print(f"\n✓ Полный ответ: {response}")
            print(f"  Размер: {len(response)} байт")
            print(f"  Hex: {response.hex()}")
            return True
        elif response:
            print(f"\n! Получен ответ, но без ожидаемого {expected_reply!r}: {response}")
            print(f"  Размер: {len(response)} байт")
            print(f"  Hex: {response.hex()}")
            return False
        else:
            print("\n✗ Ответ не получен (проверьте подключение RS-485)")
            return False
    
    except serial.SerialException as e:
        print(f"\n✗ Ошибка RS-485: {e}")
        return False
    finally:
        try:
            ser.close()
        except:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Симулятор хоста для тестирования RS-485",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры:
  # На Linux/Mac
  python3 host_simulator.py --port /dev/ttyUSB0
  
  # На Windows
  python3 host_simulator.py --port COM3
  
  # С конкретной скоростью
  python3 host_simulator.py --port /dev/ttyUSB0 --baudrate 115200
"""
    )
    
    parser.add_argument(
        "--port",
        default="COM5",
        help="Порт RS-485 (по умолчанию COM5)"
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Скорость (по умолчанию 115200)"
    )
    parser.add_argument(
        "--command-hex",
        default=DEFAULT_COMMAND.hex(),
        help="Команда для отправки в hex (по умолчанию 0001)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=35,
        help="Таймаут ответа в секундах (по умолчанию 35)"
    )
    parser.add_argument(
        "--expect",
        default=EXPECTED_REPLY.decode(),
        help="Ожидаемый ответ (по умолчанию 'recording_complete')"
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Повторять команду циклически (Ctrl+C для выхода)"
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="Показать доступные COM/serial порты и выйти"
    )
    parser.add_argument(
        "--open-delay",
        type=float,
        default=0.2,
        help="Пауза после открытия порта перед отправкой, сек (по умолчанию 0.2)"
    )
    parser.add_argument(
        "--post-write-delay",
        type=float,
        default=0.05,
        help="Пауза после write/flush перед чтением, сек (по умолчанию 0.05)"
    )
    parser.add_argument(
        "--inter-byte-delay-us",
        type=float,
        default=50,
        help="Задержка между отправкой байтов, мкс (по умолчанию 50)"
    )
    parser.add_argument(
        "--rts",
        choices=["on", "off"],
        help="Принудительно установить RTS; полезно для некоторых USB-RS485 адаптеров"
    )
    parser.add_argument(
        "--dtr",
        choices=["on", "off"],
        help="Принудительно установить DTR; полезно для некоторых USB-RS485 адаптеров"
    )
    
    args = parser.parse_args()

    if args.list_ports:
        print_ports()
        return 0
    
    try:
        command = bytes.fromhex(args.command_hex)
    except ValueError:
        print(f"Некорректный hex для --command-hex: {args.command_hex!r}", file=sys.stderr)
        return 1
    rts = None if args.rts is None else args.rts == "on"
    dtr = None if args.dtr is None else args.dtr == "on"
    
    print(f"\n{'=' * 50}")
    print("СИМУЛЯТОР ХОСТА RS-485")
    print(f"{'=' * 50}")
    print(f"Порт: {args.port}")
    print(f"Скорость: {args.baudrate} бод")
    print(f"Команда: {command}")
    expected_reply = args.expect.encode() if args.expect else b""
    print(f"Таймаут: {args.timeout} сек")
    print(f"Ожидаемый ответ: {expected_reply!r}")
    print(f"CRC-32/ISO-HDLC: little-endian append")
    print(f"Задержка между байтами: {args.inter_byte_delay_us} мкс")
    print(f"RTS: {'не менять' if rts is None else rts}")
    print(f"DTR: {'не менять' if dtr is None else dtr}")
    
    if args.loop:
        print("\nРежим циклической отправки (Ctrl+C для выхода)")
        iteration = 1
        try:
            while True:
                print(f"\n\n[ИТЕРАЦИЯ {iteration}]")
                success = send_command(
                    args.port,
                    args.baudrate,
                    command,
                    args.timeout,
                    expected_reply,
                    args.open_delay,
                    args.post_write_delay,
                    args.inter_byte_delay_us,
                    rts,
                    dtr,
                )
                iteration += 1
                time.sleep(2)  # Пауза между итерациями
        except KeyboardInterrupt:
            print("\n\nОстановлено пользователем")
            return 0
    else:
        success = send_command(
            args.port,
            args.baudrate,
            command,
            args.timeout,
            expected_reply,
            args.open_delay,
            args.post_write_delay,
            args.inter_byte_delay_us,
            rts,
            dtr,
        )
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
