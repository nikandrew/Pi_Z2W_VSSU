# Конфигурация системы записи видео

## Основные параметры

```python
# UART
UART_PORT = "/dev/serial0"           # Порт UART (проверьте: /dev/serial0 или /dev/ttyAMA0)
UART_BAUDRATE = 115200            # 115200 бод

# RS-485
RS485_GPIO_PIN = 17                  # GPIO17 для DE и RE (объединены)

# Видео
VIDEO_DURATION_MS = 10_000           # 10 минута
VIDEO_OUTPUT_DIR = "./videos"        # Папка для видео
CHUNK_SIZE_MB = 50                   # Размер части файла

# Команды
START_CMD = b"\x00\x01"              # Payload команды запуска
CRC_SIZE_BYTES = 4                   # CRC-32/ISO-HDLC, little-endian
SUCCESS_REPLY = b"recording_complete" # Ответ об успехе
```

## Использование

### 1. Подготовка Raspberry Pi Zero 2W

```bash
# На самой Pi:
sudo apt update && sudo apt upgrade -y
sudo apt install -y rpicam-apps python3-pip ffmpeg
sudo apt install -y python3-serial python3-gpiozero

# Включить камеру:
sudo raspi-config
# Interface Options → Camera → Enable → Reboot
```

### 2. Проверка компонентов

```bash
python3 test_components.py
```

### 3. Запуск системы

```bash
# Обычный запуск:
python3 main.py

# С отладкой (видеть все логи):
python3 main.py 2>&1 | tee system.log
```

### 4. Тестирование RS-485

```bash
# Тестер RS-485 (требуется sudo):
sudo python3 test_rs485.py
```

## Логирование

Логи сохраняются в `logs/camera_system.log` и выводятся в консоль.

### Уровни логирования:
- `INFO` - основная информация (записано видео, разделено на части)
- `DEBUG` - подробная отладочная информация
- `ERROR` - ошибки

##架构

```
┌─────────────────────────────────────────┐
│   asyncio Event Loop                    │
├─────────────────────────────────────────┤
│  RS485Handler    CameraSystem           │
│  ├─ listen()     ├─ record_video()      │
│  └─ send_reply() └─ split_video()       │
├─────────────────────────────────────────┤
│  GPIO Manager (GPIO17)                  │
└─────────────────────────────────────────┘
```

## Возможные проблемы

### 1. `rpicam-vid` не найден
```bash
sudo apt install -y rpicam-apps
```

### 2. Ошибка при открытии UART порта
```bash
# Проверьте доступный порт:
ls -la /dev/serial* /dev/ttyAMA*
# Или в конфиге:
sudo raspi-config # Interface Options → Serial Port
```

### 3. GPIO17 не работает
```bash
# Запустите с sudo:
sudo python3 main.py
```

### 4. Видео не записывается
```bash
# Проверьте камеру:
rpicam-vid -t 1000 -o test.h264
# Должен создаться файл test.h264
```

### 5. Медленное разделение видео
Используется ffmpeg. Для больших файлов это может занять время.
Если ffmpeg не установлен, используется простое разделение (менее надежное).

```bash
sudo apt install -y ffmpeg
```

## Автозапуск при включении Pi

Создайте systemd сервис `/etc/systemd/system/camera.service`:

```ini
[Unit]
Description=Camera Recording System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/CODE/Pi_Z2W_VSSU
ExecStart=/usr/bin/python3 /home/pi/CODE/Pi_Z2W_VSSU/main.py
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Затем:
```bash
sudo systemctl enable camera.service
sudo systemctl start camera.service
sudo systemctl status camera.service
```

## Безопасность

- Убедитесь, что UART на 115200 бод стабилен (качество кабеля, экранирование)
- RS-485 требует правильного завершения (терминирующие резисторы 120 Ω)
- GPIO17 управляется в отдельном потоке (asyncio), поэтому безопасен

## Производительность

На Pi Zero 2W система занимает примерно:
- CPU: 40-60% (во время записи видео)
- RAM: ~80 МБ
- Запись 10 секунд видео: примерно 2-4 МБ
- Разделение на части: ~30 сек/видео (с ffmpeg)
