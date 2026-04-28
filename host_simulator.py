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


def send_command(port: str, baudrate: int, command: bytes, timeout: float = 5) -> bool:
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
        
        # Ожидание ответа
        print(f"\n[WAIT] Ожидание ответа (таймаут {timeout} сек)...")
        start_time = time.time()
        response = b""
        
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"[RECV] Получено: {chunk}")
            time.sleep(0.01)
        
        if response:
            print(f"\n✓ Полный ответ: {response}")
            print(f"  Размер: {len(response)} байт")
            print(f"  Hex: {response.hex()}")
            return True
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
        default=5,
        help="Таймаут ответа в секундах (по умолчанию 5)"
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
    print(f"Таймаут: {args.timeout} сек")
    
    if args.loop:
        print("\nРежим циклической отправки (Ctrl+C для выхода)")
        iteration = 1
        try:
            while True:
                print(f"\n\n[ИТЕРАЦИЯ {iteration}]")
                success = send_command(args.port, args.baudrate, command, args.timeout)
                iteration += 1
                time.sleep(2)  # Пауза между итерациями
        except KeyboardInterrupt:
            print("\n\nОстановлено пользователем")
            return 0
    else:
        success = send_command(args.port, args.baudrate, command, args.timeout)
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
