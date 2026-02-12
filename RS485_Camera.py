#!/usr/bin/env python3
"""
RS485_Camera.py

Raspberry Pi Zero 2W:
- Слушает UART (через RS‑485 преобразователь) на скорости 2 Мбит/с.
- По команде "start" записывает 1‑минутное видео с камеры в H.264.

Требования:
- Raspberry Pi OS (64‑bit) с поддержкой libcamera.
- Включена камера (raspi-config → Interface Options → Camera).
- Настроенный UART (обычно /dev/serial0 или /dev/ttyS0).
- Установлен пакет libcamera:
    sudo apt update
    sudo apt install -y libcamera-apps
- Установлен pyserial:
    pip install pyserial
"""

import datetime
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial  # type: ignore
except ImportError:
    print("Не найден модуль 'serial' (pyserial). Установите: pip install pyserial", file=sys.stderr)
    sys.exit(1)

try:
    import RPi.GPIO as GPIO  # type: ignore[import-not-found]
except ImportError:
    GPIO = None  # type: ignore[assignment]
    print(
        "Предупреждение: не найден модуль 'RPi.GPIO'. "
        "Управление выводом DE/RE (GPIO17) будет недоступно.",
        file=sys.stderr,
    )


# ---------------------- НАСТРОЙКИ ----------------------

# Последовательный порт (проверьте, какой у вас активен: /dev/serial0 или /dev/ttyS0)
UART_PORT = "/dev/serial0"

# Скорость порта 2 Мбит/с
UART_BAUDRATE = 2_000_000

# Команда запуска записи видео
START_COMMAND = b"start"  # ожидаем байтовую последовательность "start"

# Длительность видео в миллисекундах (1 минута = 60000 мс)
VIDEO_DURATION_MS = 60_000

# Папка для сохранения видео
OUTPUT_DIR = Path("./videos")

# Имя файла: video_YYYYMMDD_HHMMSS.h264
FILENAME_TEMPLATE = "video_{timestamp}.h264"

# Путь к утилите libcamera-vid
LIBCAMERA_VID_CMD = "libcamera-vid"

# GPIO‑вывод для DE/RE RS‑485‑трансивера
# DE и /RE у вас объединены и посажены на GPIO17 (BCM‑нумерация).
# Для типичного MAX485: лог.0 → приём (DE=0, /RE=0), лог.1 → передача (DE=1, /RE=1).
RS485_DE_RE_GPIO = 17


# ---------------------- ФУНКЦИИ ДЛЯ ВИДЕО ----------------------

def ensure_output_dir() -> None:
    """Создает папку для видео, если её нет."""
    try:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    except Exception as exc:  # noqa: BLE001
        print(f"Ошибка создания папки для видео '{OUTPUT_DIR}': {exc}", file=sys.stderr)
        sys.exit(1)


def build_output_filename() -> Path:
    """Генерирует имя файла с текущей датой/временем."""
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    filename = FILENAME_TEMPLATE.format(timestamp=timestamp)
    return OUTPUT_DIR / filename


def record_video_h264(duration_ms: int) -> Path:
    """
    Записывает видео с камеры в H.264 с помощью libcamera-vid.

    Возвращает путь к созданному файлу.
    """
    ensure_output_dir()
    output_path = build_output_filename()

    # Команда libcamera-vid:
    # -t <мс>  : длительность записи
    # -o <файл>: путь к выходному файлу
    # --codec h264 (по умолчанию для h264, можно явно указать)
    cmd = [
        LIBCAMERA_VID_CMD,
        "-t",
        str(duration_ms),
        "-o",
        str(output_path),
        "--codec",
        "h264",
        "--inline",  # полезно для возможности перемотки/просмотра
    ]

    print(f"Запуск камеры, запись в файл: {output_path}")
    print("Команда:", " ".join(cmd))

    try:
        # subprocess.run блокирует текущий поток до завершения записи
        result = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError:
        print(
            "Не найдена утилита 'libcamera-vid'. Установите пакет 'libcamera-apps':\n"
            "  sudo apt update\n"
            "  sudo apt install -y libcamera-apps",
            file=sys.stderr,
        )
        raise

    if result.returncode != 0:
        print("Ошибка при выполнении libcamera-vid:", file=sys.stderr)
        print("STDOUT:", result.stdout, file=sys.stderr)
        print("STDERR:", result.stderr, file=sys.stderr)
        raise RuntimeError(f"libcamera-vid завершилась с кодом {result.returncode}")

    print(f"Видео успешно записано: {output_path}")
    return output_path


# ---------------------- RS‑485 GPIO DE/RE ----------------------

def rs485_gpio_available() -> bool:
    """Проверяет, доступен ли модуль RPi.GPIO."""
    return GPIO is not None


def rs485_init_receive_mode() -> None:
    """
    Инициализирует GPIO17 и переводит RS‑485‑трансивер в режим приёма.

    Предполагаем, что DE и /RE объединены на один вывод:
    - уровень 0: приём (DE=0, /RE=0);
    - уровень 1: передача (DE=1, /RE=1).
    """
    if not rs485_gpio_available():
        return

    # Используем BCM‑нумерацию выводов, чтобы GPIO17 означал именно BCM17.
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RS485_DE_RE_GPIO, GPIO.OUT, initial=GPIO.LOW)
    print(f"RS‑485 DE/RE (GPIO{RS485_DE_RE_GPIO}) установлен в режим приёма (LOW).")


def rs485_set_receive() -> None:
    """Переводит RS‑485‑трансивер в режим приёма (GPIO17 = LOW)."""
    if not rs485_gpio_available():
        return

    GPIO.output(RS485_DE_RE_GPIO, GPIO.LOW)


def rs485_set_transmit() -> None:
    """Переводит RS‑485‑трансивер в режим передачи (GPIO17 = HIGH). Не используется, но оставлен на будущее."""
    if not rs485_gpio_available():
        return

    GPIO.output(RS485_DE_RE_GPIO, GPIO.HIGH)


def rs485_cleanup() -> None:
    """Освобождает ресурсы GPIO при завершении работы."""
    if not rs485_gpio_available():
        return

    try:
        GPIO.cleanup(RS485_DE_RE_GPIO)
    except Exception:  # noqa: BLE001
        pass


# ---------------------- СЛУШАТЕЛЬ UART ----------------------

class UartCameraController:
    """
    Слушает UART и по команде START_COMMAND запускает запись видео.
    """

    def __init__(self, port: str, baudrate: int) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self.ser: serial.Serial | None = None
        self._stop_flag = False
        self._recording_lock = threading.Lock()

    def open(self) -> None:
        """Открывает последовательный порт."""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # небольшой таймаут, чтобы можно было проверять stop_flag
            )
        except serial.SerialException as exc:  # type: ignore[attr-defined]
            print(f"Не удалось открыть порт {self.port_name}: {exc}", file=sys.stderr)
            sys.exit(1)

        print(f"UART открыт: {self.port_name} @ {self.baudrate} бод")

    def close(self) -> None:
        """Закрывает последовательный порт."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
        self.ser = None

    def stop(self) -> None:
        """Останавливает главный цикл."""
        self._stop_flag = True

    def _handle_start_command(self) -> None:
        """
        Обрабатывает команду START: запускает запись видео.

        Используем lock, чтобы не запустить несколько записей одновременно,
        если команда придет повторно во время записи.
        """
        if not self._recording_lock.acquire(blocking=False):
            print("Команда 'start' получена, но запись уже идет. Игнорирую.")
            return

        try:
            print("Команда 'start' получена. Начинаю запись видео на 60 секунд.")
            try:
                record_video_h264(VIDEO_DURATION_MS)
            except Exception as exc:  # noqa: BLE001
                print(f"Ошибка записи видео: {exc}", file=sys.stderr)
        finally:
            self._recording_lock.release()

    def run(self) -> None:
        """
        Главный цикл: читает данные из UART и ищет команду START_COMMAND.

        Для простоты ищем последовательность байтов "start" в потоке.
        """
        if self.ser is None:
            raise RuntimeError("Порт не открыт. Вызовите open() перед run().")

        buffer = bytearray()
        print(f"Ожидание команды {START_COMMAND!r} по UART...")

        while not self._stop_flag:
            try:
                data = self.ser.read(1024)
            except serial.SerialException as exc:  # type: ignore[attr-defined]
                print(f"Ошибка чтения из UART: {exc}", file=sys.stderr)
                break

            if not data:
                # Нет новых данных, подождём чуть-чуть
                time.sleep(0.01)
                continue

            buffer.extend(data)

            # Ограничим размер буфера, чтобы не рос бесконечно
            if len(buffer) > 1024:
                buffer = buffer[-1024:]

            # Проверяем, есть ли в буфере команда START_COMMAND
            if START_COMMAND in buffer:
                # Очищаем буфер до и включая команду, чтобы не срабатывать дважды
                index = buffer.find(START_COMMAND)
                del buffer[: index + len(START_COMMAND)]

                # Обрабатываем команду
                self._handle_start_command()

        print("Главный цикл остановлен.")


def main() -> int:
    print("RS485_Camera контроллер запущен.")
    print(f"Порт: {UART_PORT}, скорость: {UART_BAUDRATE} бод")
    print(f"Команда запуска записи: {START_COMMAND!r}")
    print(f"DE/RE RS‑485 трансивера привязан к GPIO{RS485_DE_RE_GPIO} (BCM).")

    # Инициализируем GPIO для RS‑485 и сразу ставим в режим приёма
    rs485_init_receive_mode()

    controller = UartCameraController(UART_PORT, UART_BAUDRATE)
    controller.open()

    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nОстановка по Ctrl+C.")
    finally:
        controller.stop()
        controller.close()
        rs485_cleanup()
        print("Порт закрыт. Завершение работы.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

