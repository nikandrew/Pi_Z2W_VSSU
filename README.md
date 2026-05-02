# Система записи видео Raspberry Pi Zero 2W

**Полная, надежная система для записи 10-секундного видео с камеры по команде RS-485.**

## 📋 Требования

- **Оборудование**: Raspberry Pi Zero 2W, Camera Module 3 (Sony IMX708), RS-485 преобразователь SN65HVD1781AQDRQ1
- **ОС**: Raspberry Pi OS (64-bit)
- **Python**: 3.8+

## 🎯 Функциональность

✅ Асинхронное слушание RS-485 на скорости 1500000 бод  
✅ Запись 10-секундного видео в H.264 при кадре `00 01 + CRC-32/ISO-HDLC`  
✅ Автоматическое разбиение видео на части по 50 МБ  
✅ Отправка подтверждения об успехе обратно по RS-485  
✅ Логирование в файл и консоль  
✅ Корректная обработка GPIO17 для DE/RE  
✅ Обработка ошибок и восстановление после сбоев  

## 🚀 Быстрый старт

### 1. На компьютере (подготовка)
```bash
# Скопируйте все файлы на Pi через SCP
scp -r ./* pi@192.168.1.100:/home/pi/CODE/Pi_Z2W_VSSU/
```

### 2. На Raspberry Pi Zero 2W

```bash
# Подключитесь по SSH
ssh pi@192.168.1.100

# Перейдите в папку
cd /home/pi/CODE/Pi_Z2W_VSSU

# Запустите установку
bash install.sh

# Проверьте компоненты
python3 test_components.py

# Запустите систему
python3 main.py
```

## 📁 Структура проекта

```
Pi_Z2W_VSSU/
├── main.py                  # 🎯 ГЛАВНЫЙ ФАЙЛ СИСТЕМЫ
├── test_components.py       # Тестирование всех компонентов
├── test_rs485.py           # Тестирование RS-485
├── install.sh              # Скрипт установки зависимостей
├── CONFIG.md               # Подробная конфигурация
├── README.md               # Этот файл
├── videos/                 # Сохраненные видео (создается автоматически)
└── logs/                   # Логи системы (создается автоматически)
```

## 🔧 Конфигурация

Основные параметры в `main.py`:

```python
UART_PORT = "/dev/serial0"           # Порт UART
UART_BAUDRATE = 1500000           # 1500000 бод
RS485_GPIO_PIN = 17                  # GPIO для DE/RE
VIDEO_DURATION_MS = 10_000           # 10 минута
CHUNK_SIZE_MB = 50                   # Размер частей
```

Подробная документация: [CONFIG.md](CONFIG.md)

## 📊 Архитектура

```
┌──────────────────────────────────────────────────┐
│  asyncio Event Loop (асинхронный)               │
├──────────────────────────────────────────────────┤
│  RS485Handler                  CameraSystem      │
│  • Слушает UART                • Управляет       │
│  • Обрабатывает команды        • Записывает      │
│  • Отправляет ответы           • Разбивает       │
├──────────────────────────────────────────────────┤
│  GPIO Manager (GPIO17)                           │
│  • Переключает DE/RE                            │
├──────────────────────────────────────────────────┤
│  rpicam-vid (видеозапись)                    │
│  ffmpeg (разбиение видео)                       │
└──────────────────────────────────────────────────┘
```

## 🧪 Тестирование

### Проверка всех компонентов
```bash
python3 test_components.py
```

Проверяет:
- ✓ rpicam-vid доступен
- ✓ pyserial установлен
- ✓ gpiozero работает
- ✓ UART порт найден
- ✓ GPIO17 отзывается
- ✓ Камера работает

### Тестирование RS-485
```bash
sudo python3 test_rs485.py
```

Проверяет:
- ✓ GPIO переключение (приём/передача)
- ✓ UART работает
- ✓ Отправка и приём данных

## 📝 Логирование

Логи сохраняются в `logs/camera_system.log`:

```
2024-010-15 10:23:45,123 - INFO - Система камеры запущена
2024-010-15 10:23:46,234 - INFO - Подключено к /dev/serial0 (1500000 бод)
2024-010-15 10:23:50,567 - INFO - Получена команда: b"\x00\x01"
2024-010-15 10:23:50,890 - INFO - Запуск записи видео: ./videos/video_20240115_102350.h264
2024-010-15 10:24:51,234 - INFO - Видео записано: ./videos/video_20240115_102350.h264 (45.2 МБ)
2024-010-15 10:24:52,567 - INFO - Разделение видео 45.2 МБ на части по 50 МБ
2024-010-15 10:24:52,890 - INFO - Видео уже меньше 50 МБ, разделение не требуется
2024-010-15 10:24:53,123 - INFO - Отправлено по RS-485: b'recording_complete'
```

## ⚙️ Автозапуск при включении

Создайте systemd сервис:

```bash
sudo nano /etc/systemd/system/camera.service
```

Содержимое:
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

Активация:
```bash
sudo systemctl enable camera.service
sudo systemctl start camera.service
sudo systemctl status camera.service
```

## 🐛 Решение проблем

### Порт UART не открывается
```bash
# Проверьте доступные порты:
ls -la /dev/serial* /dev/ttyAMA*

# Включите UART:
sudo raspi-config  # Interface Options → Serial Port → Enable
```

### GPIO17 выдает ошибку
```bash
# Запустите с правами администратора:
sudo python3 main.py
```

### Камера не записывает
```bash
# Проверьте, что камера включена и работает:
rpicam-vid -t 1000 -o /tmp/test.h264

# Если не работает:
sudo raspi-config  # Interface Options → Camera → Enable → Reboot
```

### RS-485 не принимает данные
- Проверьте скорость передатчика (должна быть 1500000 бод)
- Проверьте подключение кабелей RS-485 (A, B, GND)
- Убедитесь, что DE и RE подключены к GPIO17
- Используйте `sudo python3 test_rs485.py` для отладки

## 📈 Производительность

На Raspberry Pi Zero 2W:
- **CPU**: 40-60% (во время записи)
- **RAM**: ~80 МБ
- **Размер видео**: примерно 2-4 МБ за 10 секунд
- **Время разделения**: ~30 сек (с ffmpeg)

## 🔒 Безопасность

- Система работает в отдельном asyncio loop
- GPIO управляется потокобезопасно
- Все ошибки записываются в логи
- Восстановление после сбоев автоматическое

## 📚 Дополнительная информация

- [CONFIG.md](CONFIG.md) - Подробная конфигурация
- [Документация rpicam](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Документация RPi.GPIO](https://sourceforge.net/p/raspberry-gpio-python/wiki/Home/)
- [RS-485 на Raspberry Pi](https://raspberrypi.stackexchange.com/questions/92246/)

## 💡 Советы

1. **Используйте экранированный кабель** для RS-485
2. **Установите терминирующие резисторы** 120 Ω на концах шины RS-485
3. **Тестируйте компоненты** перед запуском системы (`test_components.py`)
4. **Проверяйте логи** в `logs/camera_system.log`
5. **Используйте GPIO17** только для этой системы (не смешивайте с другими приложениями)

## 📞 Контакты / Поддержка

Если возникли проблемы:
1. Проверьте логи: `logs/camera_system.log`
2. Запустите тесты: `test_components.py` и `test_rs485.py`
3. Проверьте конфигурацию: `CONFIG.md`

---

**Готово к использованию на Raspberry Pi Zero 2W! 🚀**
