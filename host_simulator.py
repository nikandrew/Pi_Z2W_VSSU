#!/usr/bin/env python3
"""
Простой симулятор хоста для тестирования системы на Pi.
Отправляет команду 'start' и ждет ответа по RS-485.

Использование (на другом компьютере):
  python3 host_simulator.py --port /dev/ttyUSB0 --baudrate 2000000
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Установите: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)


EXPECTED_REPLY = b"recording_complete"


def send_command(
    port: str,
    baudrate: int,
    command: bytes,
    timeout: float = 90,
    expected_reply: bytes = EXPECTED_REPLY,
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
        
        # Отправка команды
        print(f"\n[SEND] Отправка: {command}")
        ser.write(command)
        ser.flush()
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
  python3 host_simulator.py --port /dev/ttyUSB0 --baudrate 2000000
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
        default=2_000_000,
        help="Скорость (по умолчанию 2000000)"
    )
    parser.add_argument(
        "--command",
        default="start",
        help="Команда для отправки (по умолчанию 'start')"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=90,
        help="Таймаут ответа в секундах (по умолчанию 90, т.к. main.py пишет видео 60 секунд)"
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
    
    args = parser.parse_args()
    
    command = args.command.encode() if isinstance(args.command, str) else args.command
    
    print(f"\n{'=' * 50}")
    print("СИМУЛЯТОР ХОСТА RS-485")
    print(f"{'=' * 50}")
    print(f"Порт: {args.port}")
    print(f"Скорость: {args.baudrate} бод")
    print(f"Команда: {command}")
    expected_reply = args.expect.encode() if args.expect else b""
    print(f"Таймаут: {args.timeout} сек")
    print(f"Ожидаемый ответ: {expected_reply!r}")
    
    if args.loop:
        print("\nРежим циклической отправки (Ctrl+C для выхода)")
        iteration = 1
        try:
            while True:
                print(f"\n\n[ИТЕРАЦИЯ {iteration}]")
                success = send_command(args.port, args.baudrate, command, args.timeout, expected_reply)
                iteration += 1
                time.sleep(2)  # Пауза между итерациями
        except KeyboardInterrupt:
            print("\n\nОстановлено пользователем")
            return 0
    else:
        success = send_command(args.port, args.baudrate, command, args.timeout, expected_reply)
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
