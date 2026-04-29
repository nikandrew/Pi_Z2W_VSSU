#!/usr/bin/env python3
"""
Простой тестер RS-485 для отладки.
Генерирует команды и проверяет ответы.
"""

import asyncio
import sys
import time

try:
    import serial
except ImportError:
    print("Установите: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    print("Установите: sudo apt install -y python3-gpiozero", file=sys.stderr)
    sys.exit(1)


UART_PORT = "/dev/serial0"
UART_BAUDRATE = 115200
GPIO_PIN = 17

# Команды
START_CMD = b"\x00\x01"
SUCCESS_REPLY = b"recording_complete"


def test_gpio() -> None:
    """Тестирует GPIO переключение."""
    print("\n[GPIO TEST]")
    try:
        gpio = DigitalOutputDevice(GPIO_PIN)
        
        print(f"  GPIO{GPIO_PIN} OFF (режим приема)...")
        gpio.off()
        time.sleep(0.5)
        
        print(f"  GPIO{GPIO_PIN} ON (режим передачи)...")
        gpio.on()
        time.sleep(0.5)
        
        print(f"  GPIO{GPIO_PIN} OFF (режим приема)...")
        gpio.off()
        
        gpio.close()
        print("  ✓ GPIO работает корректно")
    except Exception as e:
        print(f"  ✗ Ошибка GPIO: {e}")
        print("  Если ошибка содержит 'busy', остановите main.py/test_components.py/другие RS485_*.py или перезагрузите Pi")
        print("  Если ошибка про права, запустите: sudo python3 test_rs485.py")
        sys.exit(1)


def test_uart_loopback() -> None:
    """Тестирует UART loopback (требует перемычки RX-TX на плате)."""
    print("\n[UART LOOPBACK TEST]")
    try:
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=UART_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        test_data = b"TEST_LOOPBACK"
        print(f"  Отправка: {test_data}")
        ser.write(test_data)
        
        time.sleep(0.1)
        received = ser.read(len(test_data))
        
        if received == test_data:
            print(f"  ✓ Получено: {received}")
        else:
            print(f"  ! Получено другие данные: {received}")
            print("  (Это ожидаемо, если нет loopback перемычки)")
        
        ser.close()
    except Exception as e:
        print(f"  ✗ Ошибка UART: {e}")
        sys.exit(1)


def test_uart_send_receive() -> None:
    """Интерактивный тест отправки/приема."""
    print("\n[UART INTERACTIVE TEST]")
    print("  Отправка команды 0x00 0x01 и ожидание ответа...")
    
    try:
        gpio = DigitalOutputDevice(GPIO_PIN)
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=UART_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
        
        # Режим приема
        gpio.off()
        time.sleep(0.1)
        
        # Режим передачи
        print(f"  1. Переключение в режим передачи (GPIO{GPIO_PIN}=HIGH)...")
        gpio.on()
        time.sleep(0.001)
        
        # Отправка
        print(f"  2. Отправка: {START_CMD}")
        ser.write(START_CMD)
        ser.flush()
        
        time.sleep(0.1)
        
        # Режим приема
        print(f"  3. Переключение в режим приема (GPIO{GPIO_PIN}=LOW)...")
        gpio.off()
        time.sleep(0.001)
        
        # Чтение ответа
        print(f"  4. Ожидание ответа (таймаут 25 сек)...")
        start_time = time.time()
        response = b""
        
        while time.time() - start_time < 25:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                print(f"     Получено: {chunk}")
            time.sleep(0.01)
        
        if not response:
            print("  ! Ответ не получен (проверьте подключение RS-485)")
        else:
            print(f"  ✓ Полный ответ: {response}")
        
        ser.close()
        gpio.close()
    except Exception as e:
        print(f"  ✗ Ошибка: {e}")
        sys.exit(1)


def main() -> int:
    print("=" * 50)
    print("Тестирование RS-485")
    print("=" * 50)
    print(f"Порт: {UART_PORT}")
    print(f"Скорость: {UART_BAUDRATE} бод")
    print(f"GPIO: {GPIO_PIN}")
    
    try:
        test_gpio()
        test_uart_loopback()
        test_uart_send_receive()
        
        print("\n" + "=" * 50)
        print("✓ Все тесты завершены")
        print("=" * 50)
        return 0
    except KeyboardInterrupt:
        print("\n\nОстановлено пользователем")
        return 1


if __name__ == "__main__":
    sys.exit(main())
