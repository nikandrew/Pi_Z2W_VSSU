#!/usr/bin/env python3
"""
Полная система для Raspberry Pi Zero 2W:
- Слушает RS-485 на скорости 2 Мбит/с
- При команде 'start' записывает видео 1 минуту в H.264
- Разбивает видео на части по 50 МБ
- Отправляет сообщение об успехе по RS-485

Требования:
  sudo apt update && sudo apt upgrade -y
  sudo apt install -y rpicam-apps python3-pip
  sudo apt install -y python3-serial python3-gpiozero
"""

import asyncio
import datetime
import logging
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError:
    print("Установите: sudo apt install -y python3-serial", file=sys.stderr)
    sys.exit(1)

try:
    from gpiozero import DigitalOutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Предупреждение: gpiozero не найден. GPIO17 не будет работать.", file=sys.stderr)


# ==================== КОНФИГУРАЦИЯ ====================

UART_PORT = "/dev/serial0"
UART_BAUDRATE = 2_000_000

RS485_GPIO_PIN = 17  # DE и RE объединены на GPIO17
RS485_RX_MODE = False  # LOW
RS485_TX_MODE = True  # HIGH

# Команды
START_CMD = b"start"
SUCCESS_REPLY = b"recording_complete"

# Видео
VIDEO_DURATION_MS = 60_000  # 1 минута
VIDEO_OUTPUT_DIR = Path("./videos")
CHUNK_SIZE_MB = 50
CHUNK_SIZE_BYTES = CHUNK_SIZE_MB * 1024 * 1024

# Логирование
LOG_FILE = Path("./logs/camera_system.log")
LOG_FILE.parent.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


# ==================== GPIO УПРАВЛЕНИЕ ====================

class RS485Manager:
    """Управление RS-485 трансивером через GPIO17."""
    
    def __init__(self):
        self.gpio = None
        self.enabled = False
        self._init_gpio()
    
    def _init_gpio(self) -> None:
        """Инициализирует GPIO для RS-485."""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO недоступен, RS-485 будет работать в режиме приема")
            return
        
        try:
            self.gpio = DigitalOutputDevice(RS485_GPIO_PIN)
            self.set_rx_mode()
            self.enabled = True
            logger.info(f"GPIO{RS485_GPIO_PIN} инициализирован (режим приема)")
        except Exception as e:
            logger.error(f"Ошибка инициализации GPIO: {e}")
    
    def set_rx_mode(self) -> None:
        """Переводит в режим приема (DE/RE = LOW)."""
        if self.gpio:
            self.gpio.off()
    
    def set_tx_mode(self) -> None:
        """Переводит в режим передачи (DE/RE = HIGH)."""
        if self.gpio:
            self.gpio.on()
    
    def cleanup(self) -> None:
        """Освобождает GPIO."""
        if self.gpio:
            try:
                self.gpio.off()
                self.gpio.close()
            except Exception as e:
                logger.error(f"Ошибка при очистке GPIO: {e}")


# ==================== RS-485 КОММУНИКАЦИЯ ====================

class RS485Handler:
    """Асинхронный обработчик RS-485."""
    
    def __init__(self, port: str, baudrate: int, gpio_manager: RS485Manager):
        self.port = port
        self.baudrate = baudrate
        self.gpio_manager = gpio_manager
        self.ser = None
        self.buffer = bytearray()
        self.running = False
    
    async def connect(self) -> bool:
        """Открывает последовательный порт."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            logger.info(f"Подключено к {self.port} ({self.baudrate} бод)")
            return True
        except serial.SerialException as e:
            logger.error(f"Ошибка подключения к {self.port}: {e}")
            return False
    
    async def listen(self) -> Optional[bytes]:
        """
        Асинхронное слушание RS-485.
        Возвращает полученную команду, если она завершена (содержит START_CMD).
        """
        if not self.ser:
            return None
        
        try:
            # Неблокирующее чтение
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(data)
                
                # Пытаемся найти команду в буфере
                if START_CMD in self.buffer:
                    idx = self.buffer.find(START_CMD)
                    self.buffer = self.buffer[idx + len(START_CMD):]
                    logger.info(f"Получена команда: {START_CMD}")
                    return START_CMD
        except serial.SerialException as e:
            logger.error(f"Ошибка чтения RS-485: {e}")
        
        return None
    
    async def send_reply(self, message: bytes) -> bool:
        """Отправляет ответное сообщение по RS-485."""
        if not self.ser:
            return False
        
        try:
            self.gpio_manager.set_tx_mode()
            await asyncio.sleep(0.001)  # Стабилизация
            
            self.ser.write(message)
            self.ser.flush()
            
            await asyncio.sleep(0.001)
            self.gpio_manager.set_rx_mode()
            
            logger.info(f"Отправлено по RS-485: {message}")
            return True
        except serial.SerialException as e:
            logger.error(f"Ошибка отправки RS-485: {e}")
            return False
        finally:
            self.gpio_manager.set_rx_mode()
    
    def disconnect(self) -> None:
        """Закрывает последовательный порт."""
        if self.ser:
            try:
                self.ser.close()
            except Exception as e:
                logger.error(f"Ошибка закрытия порта: {e}")


# ==================== ЗАПИСЬ ВИДЕО ====================

def ensure_output_dir() -> None:
    """Создает директорию для видео."""
    try:
        VIDEO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        logger.error(f"Ошибка создания директории {VIDEO_OUTPUT_DIR}: {e}")
        raise


def build_output_filename() -> Path:
    """Генерирует имя файла видео."""
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    return VIDEO_OUTPUT_DIR / f"video_{timestamp}.h264"


async def record_video(duration_ms: int) -> Optional[Path]:
    """
    Записывает видео с камеры с помощью rpicam-vid.
    Запуск в отдельном процессе через asyncio.
    """
    ensure_output_dir()
    output_path = build_output_filename()
    
    cmd = [
        "rpicam-vid",
        "-t", str(duration_ms),
        "-o", str(output_path),
        "--codec", "h264",
        "--inline"
    ]
    
    logger.info(f"Запуск записи видео: {output_path}")
    logger.debug(f"Команда: {' '.join(cmd)}")
    
    try:
        # Запускаем rpicam-vid в отдельном процессе
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        # Ждем завершения
        stdout, stderr = await proc.communicate()
        
        if proc.returncode != 0:
            logger.error(f"rpicam-vid ошибка (код {proc.returncode})")
            logger.error(f"STDERR: {stderr.decode()}")
            return None
        
        if output_path.exists():
            file_size_mb = output_path.stat().st_size / (1024 * 1024)
            logger.info(f"Видео записано: {output_path} ({file_size_mb:.1f} МБ)")
            return output_path
        else:
            logger.error(f"Файл видео не создан: {output_path}")
            return None
    
    except FileNotFoundError:
        logger.error(
            "rpicam-vid не найден. Установите: sudo apt install -y rpicam-apps"
        )
        return None
    except Exception as e:
        logger.error(f"Ошибка при записи видео: {e}")
        return None


# ==================== РАЗДЕЛЕНИЕ ВИДЕО ====================

async def split_video_into_chunks(video_path: Path) -> list[Path]:
    """
    Разбивает H.264 видео на части по CHUNK_SIZE_BYTES.
    Использует ffmpeg для простоты и надежности.
    
    Возвращает список путей к созданным файлам.
    """
    if not video_path.exists():
        logger.error(f"Видео файл не найден: {video_path}")
        return []
    
    file_size = video_path.stat().st_size
    
    # Если файл меньше лимита, разделение не требуется
    if file_size <= CHUNK_SIZE_BYTES:
        logger.info(f"Видео уже меньше {CHUNK_SIZE_MB} МБ, разделение не требуется")
        return [video_path]
    
    logger.info(f"Разделение видео {file_size / (1024*1024):.1f} МБ на части по {CHUNK_SIZE_MB} МБ")
    
    # Используем ffmpeg для разделения на основе размера
    output_pattern = video_path.parent / f"{video_path.stem}_part_%d.h264"
    
    cmd = [
        "ffmpeg",
        "-i", str(video_path),
        "-c", "copy",  # копируем без перекодирования
        "-f", "segment",
        "-segment_size", str(CHUNK_SIZE_BYTES),
        str(output_pattern)
    ]
    
    logger.debug(f"Команда ffmpeg: {' '.join(cmd)}")
    
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await proc.communicate()
        
        if proc.returncode != 0:
            logger.error(f"ffmpeg ошибка: {stderr.decode()}")
            return []
        
        # Проверяем созданные части
        chunks = sorted(video_path.parent.glob(f"{video_path.stem}_part_*.h264"))
        if chunks:
            logger.info(f"Создано {len(chunks)} частей видео")
            for i, chunk in enumerate(chunks, 1):
                size_mb = chunk.stat().st_size / (1024 * 1024)
                logger.info(f"  Часть {i}: {chunk.name} ({size_mb:.1f} МБ)")
            return chunks
        else:
            logger.error("ffmpeg не создал части, возвращаю исходный файл")
            return [video_path]
    
    except FileNotFoundError:
        logger.warning(
            "ffmpeg не найден, будет использовано простое разделение "
            "(установите: sudo apt install -y ffmpeg для лучшего результата)"
        )
        return await split_video_simple(video_path)
    except Exception as e:
        logger.error(f"Ошибка разделения видео: {e}")
        return [video_path]


async def split_video_simple(video_path: Path) -> list[Path]:
    """
    Простое разделение файла на части по размеру (без перекодирования).
    Менее надежно для видео, но работает без ffmpeg.
    """
    try:
        file_size = video_path.stat().st_size
        num_chunks = (file_size + CHUNK_SIZE_BYTES - 1) // CHUNK_SIZE_BYTES
        
        chunks = []
        with open(video_path, "rb") as f:
            for i in range(num_chunks):
                chunk_path = video_path.parent / f"{video_path.stem}_part_{i}.h264"
                chunk_data = f.read(CHUNK_SIZE_BYTES)
                
                with open(chunk_path, "wb") as out:
                    out.write(chunk_data)
                
                chunks.append(chunk_path)
                logger.info(f"  Часть {i + 1}: {chunk_path.name}")
        
        return chunks
    except Exception as e:
        logger.error(f"Ошибка простого разделения: {e}")
        return [video_path]


# ==================== ГЛАВНЫЙ КОНТРОЛЛЕР ====================

class CameraSystem:
    """Основной контроллер всей системы."""
    
    def __init__(self):
        self.gpio_manager = RS485Manager()
        self.rs485 = RS485Handler(UART_PORT, UART_BAUDRATE, self.gpio_manager)
        self.running = True
    
    async def main_loop(self) -> None:
        """Главный цикл слушания и обработки."""
        logger.info("=" * 60)
        logger.info("Система камеры запущена")
        logger.info("=" * 60)
        
        # Подключаемся к RS-485
        if not await self.rs485.connect():
            logger.error("Не удалось подключиться к RS-485")
            return
        
        try:
            while self.running:
                # Слушаем RS-485
                command = await self.rs485.listen()
                
                if command == START_CMD:
                    await self.handle_start_command()
                
                # Небольшая задержка для снижения нагрузки на CPU
                await asyncio.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.info("Остановка по Ctrl+C")
        except Exception as e:
            logger.error(f"Критическая ошибка в main_loop: {e}")
        finally:
            self.shutdown()
    
    async def handle_start_command(self) -> None:
        """Обработчик команды start."""
        logger.info("Обработка команды START")
        
        try:
            # 1. Записываем видео
            video_path = await record_video(VIDEO_DURATION_MS)
            if not video_path:
                logger.error("Не удалось записать видео")
                return
            
            # 2. Разбиваем видео на части
            chunks = await split_video_into_chunks(video_path)
            if not chunks:
                logger.error("Не удалось разделить видео")
                return
            
            logger.info(f"Обработка завершена успешно ({len(chunks)} файлов)")
            
            # 3. Отправляем сообщение об успехе
            await self.rs485.send_reply(SUCCESS_REPLY)
        
        except Exception as e:
            logger.error(f"Ошибка при обработке START: {e}")
    
    def shutdown(self) -> None:
        """Корректное завершение работы."""
        logger.info("Завершение работы системы...")
        self.running = False
        self.rs485.disconnect()
        self.gpio_manager.cleanup()
        logger.info("Система остановлена")


# ==================== ТОЧКА ВХОДА ====================

async def main() -> int:
    """Точка входа приложения."""
    system = CameraSystem()
    
    try:
        await system.main_loop()
        return 0
    except Exception as e:
        logger.error(f"Необработанное исключение: {e}", exc_info=True)
        return 1
    finally:
        system.shutdown()


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
