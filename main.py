#!/usr/bin/env python3
"""
Полная система для Raspberry Pi Zero 2W:
- Слушает RS-485 на скорости 1500000 бод
- При команде 0x00 0x01 записывает видео 10 секунд в H.264
- Разбивает видео на части по 50 МБ
- Отправляет сообщение об успехе по RS-485

Требования:
  sudo apt update && sudo apt upgrade -y
  sudo apt install -y rpicam-apps python3-pip
  sudo apt install -y python3-serial python3-gpiozero
"""

import asyncio
import binascii
import datetime
import json
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
UART_BAUDRATE = 1500000

RS485_GPIO_PIN = 17  # DE и RE объединены на GPIO17
RS485_RX_MODE = False  # LOW
RS485_TX_MODE = True  # HIGH
RS485_USE_GPIO = os.environ.get("RS485_USE_GPIO", "1").lower() not in ("0", "false", "no", "off")

# Команды
START_CMD = b"\x00\x01"
CRC_SIZE_BYTES = 4
SUCCESS_REPLY = b"recording_complete"
FILE_FRAME_MAGIC = b"VSSU"
FILE_FRAME_META = 1
FILE_FRAME_CHUNK = 2
FILE_FRAME_END = 3
FILE_TRANSFER_CHUNK_SIZE = 1024

# Видео
VIDEO_DURATION_MS = 10_000  # 10 секунд
VIDEO_FRAMERATE = 30
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


def crc32_iso_hdlc(data: bytes) -> int:
    """CRC-32/ISO-HDLC: poly 0x04C11DB7, init/xorout 0xffffffff, reflected."""
    return binascii.crc32(data) & 0xFFFFFFFF


def append_crc32_iso_hdlc(data: bytes) -> bytes:
    return data + crc32_iso_hdlc(data).to_bytes(CRC_SIZE_BYTES, "little")


def verify_crc32_iso_hdlc(frame: bytes) -> bool:
    if len(frame) < CRC_SIZE_BYTES:
        return False
    payload = frame[:-CRC_SIZE_BYTES]
    received_crc = int.from_bytes(frame[-CRC_SIZE_BYTES:], "little")
    return received_crc == crc32_iso_hdlc(payload)


def make_file_frame(frame_type: int, payload: bytes) -> bytes:
    return (
        FILE_FRAME_MAGIC
        + bytes([frame_type])
        + len(payload).to_bytes(4, "little")
        + payload
        + crc32_iso_hdlc(payload).to_bytes(CRC_SIZE_BYTES, "little")
    )


def file_crc32_iso_hdlc(path: Path) -> int:
    crc = 0
    with open(path, "rb") as file:
        while True:
            chunk = file.read(64 * 1024)
            if not chunk:
                break
            crc = binascii.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


# ==================== GPIO УПРАВЛЕНИЕ ====================

class RS485Manager:
    """Управление RS-485 трансивером через GPIO17."""
    
    def __init__(self):
        self.gpio = None
        self.enabled = False
        self._init_gpio()
    
    def _init_gpio(self) -> None:
        """Инициализирует GPIO для RS-485."""
        if not RS485_USE_GPIO:
            logger.warning(
                "Управление GPIO17 отключено через RS485_USE_GPIO=0. "
                "Прием будет работать, ответ возможен только с auto-direction RS-485 адаптером."
            )
            return

        if not GPIO_AVAILABLE:
            logger.warning("GPIO недоступен, RS-485 будет работать в режиме приема")
            return
        
        try:
            self.gpio = DigitalOutputDevice(RS485_GPIO_PIN)
            self.set_rx_mode()
            self.enabled = True
            logger.info(f"GPIO{RS485_GPIO_PIN} инициализирован (режим приема)")
        except Exception as e:
            self.gpio = None
            self.enabled = False
            logger.warning(f"GPIO{RS485_GPIO_PIN} недоступен: {e}")
            logger.warning(
                "Если видите 'GPIO busy', остановите другие скрипты, которые используют GPIO17 "
                "(main.py, test_rs485.py, test_components.py, RS485_*.py), либо перезагрузите Pi. "
                "Для проверки только приема можно запустить: RS485_USE_GPIO=0 python3 main.py"
            )
    
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
            serial_kwargs = {
                "port": self.port,
                "baudrate": self.baudrate,
                "bytesize": serial.EIGHTBITS,
                "parity": serial.PARITY_NONE,
                "stopbits": serial.STOPBITS_ONE,
                "timeout": 0.1,
                "xonxoff": False,
                "rtscts": False,
                "dsrdtr": False,
            }
            if sys.platform.startswith("linux"):
                serial_kwargs["exclusive"] = True
            self.ser = serial.Serial(**serial_kwargs)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
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
                logger.info(f"RS-485 RX: {data!r} hex={data.hex()} buffer={self.buffer.hex()}")
                if self.buffer.endswith(b"\x01") and START_CMD not in self.buffer:
                    logger.warning(
                        "Получен 0x01 без ведущего 0x00. Вероятно, первый байт теряется "
                        "на USB-RS485 адаптере или при переключении направления."
                    )
                
                # Ищем кадр: 00 01 + CRC-32/ISO-HDLC (4 байта, little-endian)
                while True:
                    idx = self.buffer.find(START_CMD)
                    if idx < 0:
                        if len(self.buffer) > len(START_CMD) + CRC_SIZE_BYTES:
                            self.buffer = self.buffer[-(len(START_CMD) + CRC_SIZE_BYTES - 1):]
                        break

                    frame_len = len(START_CMD) + CRC_SIZE_BYTES
                    if len(self.buffer) < idx + frame_len:
                        if idx > 0:
                            del self.buffer[:idx]
                        break

                    frame = bytes(self.buffer[idx:idx + frame_len])
                    tail = self.buffer[idx + frame_len:]
                    if verify_crc32_iso_hdlc(frame):
                        logger.info(
                            f"Команда запуска найдена и CRC корректен: idx={idx}, "
                            f"frame={frame.hex()}, tail={bytes(tail).hex()}"
                        )
                        self.buffer.clear()
                        return START_CMD

                    received_crc = int.from_bytes(frame[-CRC_SIZE_BYTES:], "little")
                    expected_crc = crc32_iso_hdlc(frame[:-CRC_SIZE_BYTES])
                    logger.warning(
                        f"Команда 00 01 найдена, но CRC неверен: frame={frame.hex()}, "
                        f"received=0x{received_crc:08x}, expected=0x{expected_crc:08x}"
                    )
                    del self.buffer[:idx + 1]
        except serial.SerialException as e:
            logger.error(f"Ошибка чтения RS-485: {e}")
        
        return None
    
    async def send_bytes(self, payload: bytes, log_payload: bool = True) -> bool:
        """Отправляет байты по RS-485 с переключением направления."""
        if not self.ser:
            return False
        if not self.gpio_manager.enabled:
            logger.warning(
                "GPIO управления DE/RE недоступен; отправляю ответ без переключения направления. "
                "Это сработает только с auto-direction RS-485 адаптером."
            )
        
        try:
            self.gpio_manager.set_tx_mode()
            await asyncio.sleep(0.001)  # Стабилизация
            
            self.ser.write(payload)
            self.ser.flush()
            
            await asyncio.sleep(0.001)
            self.gpio_manager.set_rx_mode()
            
            if log_payload:
                logger.info(f"Отправлено по RS-485: {payload}")
            return True
        except serial.SerialException as e:
            logger.error(f"Ошибка отправки RS-485: {e}")
            return False
        finally:
            self.gpio_manager.set_rx_mode()

    async def send_reply(self, message: bytes) -> bool:
        """Отправляет ответное сообщение по RS-485."""
        return await self.send_bytes(message)

    async def send_file(self, file_path: Path) -> bool:
        """Отправляет записанный файл фреймами VSSU с CRC-32/ISO-HDLC."""
        if not file_path.exists():
            logger.error(f"Файл для отправки не найден: {file_path}")
            return False

        file_size = file_path.stat().st_size
        file_crc = file_crc32_iso_hdlc(file_path)
        metadata = {
            "name": file_path.name,
            "size": file_size,
            "crc32": file_crc,
        }
        metadata_payload = json.dumps(metadata, separators=(",", ":")).encode("utf-8")

        logger.info(
            f"Начало передачи файла по RS-485: {file_path.name}, "
            f"{file_size} bytes, crc=0x{file_crc:08x}"
        )

        if not await self.send_bytes(make_file_frame(FILE_FRAME_META, metadata_payload), log_payload=False):
            return False

        sent = 0
        chunk_index = 0
        with open(file_path, "rb") as file:
            while True:
                chunk = file.read(FILE_TRANSFER_CHUNK_SIZE)
                if not chunk:
                    break

                if not await self.send_bytes(make_file_frame(FILE_FRAME_CHUNK, chunk), log_payload=False):
                    return False

                sent += len(chunk)
                chunk_index += 1
                if chunk_index == 1 or chunk_index % 50 == 0 or sent == file_size:
                    percent = sent * 100 / file_size if file_size else 100
                    logger.info(f"Передача файла: {sent}/{file_size} bytes ({percent:.1f}%)")
                await asyncio.sleep(0.003)

        if not await self.send_bytes(make_file_frame(FILE_FRAME_END, b""), log_payload=False):
            return False

        logger.info(f"Файл отправлен по RS-485: {file_path.name}")
        return True
    
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


def build_timestamped_filename(video_path: Path) -> Path:
    """Генерирует имя файла видео с наложенным таймкодом."""
    return video_path.with_name(f"{video_path.stem}_timestamped{video_path.suffix}")


async def add_video_timestamp(video_path: Path) -> Optional[Path]:
    """Добавляет в кадр секунды и миллисекунды от начала съемки."""
    timestamped_path = build_timestamped_filename(video_path)
    drawtext_filter = (
        "drawtext="
        "fontcolor=white:"
        "fontsize=36:"
        "box=1:"
        "boxcolor=black@0.55:"
        "boxborderw=8:"
        "x=20:"
        "y=20:"
        "text='%{pts\\:hms}'"
    )

    cmd = [
        "ffmpeg",
        "-y",
        "-r", str(VIDEO_FRAMERATE),
        "-i", str(video_path),
        "-vf", drawtext_filter,
        "-c:v", "libx264",
        "-preset", "veryfast",
        "-pix_fmt", "yuv420p",
        "-f", "h264",
        str(timestamped_path),
    ]

    logger.info(f"Добавление таймкода в видео: {timestamped_path}")
    logger.debug(f"Команда ffmpeg: {' '.join(cmd)}")

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await proc.communicate()

        if proc.returncode != 0:
            logger.error(f"ffmpeg не смог добавить таймкод (код {proc.returncode})")
            logger.error(f"STDERR: {stderr.decode(errors='replace')}")
            return None

        if not timestamped_path.exists() or timestamped_path.stat().st_size == 0:
            logger.error(f"Файл с таймкодом не создан: {timestamped_path}")
            return None

        logger.info(f"Видео с таймкодом создано: {timestamped_path}")
        return timestamped_path

    except FileNotFoundError:
        logger.error("ffmpeg не найден, невозможно добавить секунды и миллисекунды в видео")
        return None
    except Exception as e:
        logger.error(f"Ошибка при добавлении таймкода в видео: {e}")
        return None


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
        "--framerate", str(VIDEO_FRAMERATE),
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
            timestamped_path = await add_video_timestamp(output_path)
            if not timestamped_path:
                logger.error("Видео записано, но таймкод добавить не удалось")
                return None
            return timestamped_path
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
                    logger.info("Команда запуска подтверждена, начинаю обработку")
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
        """Обработчик команды запуска записи."""
        logger.info("Обработка команды START")
        
        try:
            # 1. Записываем видео
            logger.info(f"Запуск записи на {VIDEO_DURATION_MS} мс")
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
            
            # 3. Отправляем сообщение об успехе и сам записанный файл
            if await self.rs485.send_reply(SUCCESS_REPLY):
                await self.rs485.send_file(video_path)
        
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
