#!/usr/bin/env python3
"""
Тестирование компонентов системы.
Запустить на Raspberry Pi: python3 test_components.py
"""

import sys
import subprocess
import time


def test_rpicam() -> bool:
    """Проверяет доступность rpicam-vid."""
    print("\n[TEST] rpicam-vid...")
    try:
        result = subprocess.run(
            ["rpicam-vid", "--version"],
            capture_output=True,
            timeout=5
        )
        if result.returncode == 0:
            print("  ✓ rpicam-vid доступен")
            return True
        else:
            print("  ✗ rpicam-vid вернул ошибку")
            return False
    except FileNotFoundError:
        print("  ✗ rpicam-vid не найден (установите: sudo apt install rpicam-apps)")
        return False
    except Exception as e:
        print(f"  ✗ Ошибка: {e}")
        return False


def test_pyserial() -> bool:
    """Проверяет pyserial."""
    print("\n[TEST] pyserial...")
    try:
        import serial
        print(f"  ✓ pyserial версия: {serial.__version__}")
        return True
    except ImportError:
        print("  ✗ pyserial не установлен (pip install pyserial)")
        return False


def test_gpiozero() -> bool:
    """Проверяет gpiozero."""
    print("\n[TEST] gpiozero...")
    try:
        from gpiozero import DigitalOutputDevice
        print("  ✓ gpiozero доступен")
        return True
    except ImportError:
        print("  ✗ gpiozero не установлен (pip install gpiozero)")
        return False


def test_uart_port() -> bool:
    """Проверяет наличие UART порта."""
    print("\n[TEST] UART порт...")
    import os
    
    port = None
    for p in ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]:
        if os.path.exists(p):
            port = p
            print(f"  ✓ Найден порт: {p}")
            return True
    
    if not port:
        print("  ✗ UART порт не найден")
        print("     Проверьте: raspi-config → Interface Options → Serial Port")
        return False


def test_gpio17() -> bool:
    """Проверяет GPIO17."""
    print("\n[TEST] GPIO17...")
    try:
        from gpiozero import DigitalOutputDevice
        device = DigitalOutputDevice(17)
        device.off()
        device.close()
        print("  ✓ GPIO17 работает")
        return True
    except Exception as e:
        print(f"  ✗ Ошибка GPIO17: {e}")
        print("     Убедитесь, что запускаете скрипт с правами суперпользователя: sudo python3 test_components.py")
        return False


def test_camera() -> bool:
    """Пробует захватить кадр с камеры."""
    print("\n[TEST] Камера (захват одного кадра)...")
    try:
        result = subprocess.run(
            ["rpicam-vid", "-t", "100", "-o", "/tmp/test.h264"],
            capture_output=True,
            timeout=5
        )
        
        import os
        if os.path.exists("/tmp/test.h264"):
            size = os.path.getsize("/tmp/test.h264")
            print(f"  ✓ Камера работает ({size} байт записано)")
            os.remove("/tmp/test.h264")
            return True
        else:
            print("  ✗ Файл не создан")
            return False
    except Exception as e:
        print(f"  ✗ Ошибка: {e}")
        return False


def main() -> int:
    """Главная функция тестирования."""
    print("=" * 50)
    print("Тестирование компонентов системы")
    print("=" * 50)
    
    tests = [
        ("rpicam-vid", test_rpicam),
        ("pyserial", test_pyserial),
        ("gpiozero", test_gpiozero),
        ("UART порт", test_uart_port),
        ("GPIO17", test_gpio17),
        ("Камера", test_camera),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            passed = test_func()
            results.append((name, passed))
        except Exception as e:
            print(f"  ✗ Необработанная ошибка: {e}")
            results.append((name, False))
    
    # Итоги
    print("\n" + "=" * 50)
    print("ИТОГИ")
    print("=" * 50)
    
    passed = sum(1 for _, p in results if p)
    total = len(results)
    
    for name, passed_test in results:
        status = "✓" if passed_test else "✗"
        print(f"  {status} {name}")
    
    print(f"\nВсего: {passed}/{total} тестов пройдено")
    
    if passed == total:
        print("\n✓ Система готова к работе!")
        return 0
    else:
        print(f"\n✗ Требуется устранить {total - passed} проблем")
        return 1


if __name__ == "__main__":
    sys.exit(main())
