#!/bin/bash
# Быстрый старт на Raspberry Pi Zero 2W

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}\"
echo -e \"${BLUE}║  Система записи видео на Raspberry Pi Zero 2W        ║${NC}\""
echo -e \"${BLUE}╚════════════════════════════════════════════════════════╝${NC}\"

# Проверка, запущено ли с sudo
if [ \"$EUID\" -eq 0 ]; then
  echo -e \"${YELLOW}⚠  Запуск с sudo (некоторые операции требуют его)${NC}\"
else
  echo -e \"${YELLOW}⚠  Рекомендуется запустить с sudo для GPIO${NC}\"
fi

echo \"\"
echo -e \"${BLUE}[ЭТАП 1/5]${NC} Обновление системы...\"
sudo apt update
sudo apt upgrade -y

echo \"\"
echo -e \"${BLUE}[ЭТАП 2/5]${NC} Установка зависимостей...\"
sudo apt install -y \
    python3-libcamera \
    python3-picamera2 \
    python3-pip \
    python3-venv \
    rpicam-apps \
    ffmpeg \
    git \
    libcamera0 \
    libcamera-tools

echo \"\"
echo -e \"${BLUE}[ЭТАП 3/5]${NC} Установка Python пакетов...\"
pip install --upgrade pip --break-system-packages
pip install --break-system-packages \
    pyserial \
    gpiozero

echo \"\"
echo -e \"${BLUE}[ЭТАП 4/5]${NC} Активация камеры...\"
if grep -q \"camera_auto_detect=1\" /boot/firmware/config.txt; then
    echo -e \"${GREEN}✓${NC} Камера уже активирована\"
else
    echo -e \"${YELLOW}⚠  Камера может быть отключена${NC}\"
    echo -e \"    Активируйте: ${BLUE}sudo raspi-config${NC}\"
    echo -e \"    → Interface Options → Camera → Enable\"
    echo -e \"    → Finish → Reboot\"
fi

echo \"\"
echo -e \"${BLUE}[ЭТАП 5/5]${NC} Проверка UART...\"
if [ -e /dev/serial0 ]; then
    echo -e \"${GREEN}✓${NC} Найден /dev/serial0\"
elif [ -e /dev/ttyAMA0 ]; then
    echo -e \"${GREEN}✓${NC} Найден /dev/ttyAMA0 (обновите UART_PORT в main.py)\"
else
    echo -e \"${YELLOW}⚠  UART не найден${NC}\"
    echo -e \"    Активируйте: ${BLUE}sudo raspi-config${NC}\"
    echo -e \"    → Interface Options → Serial Port → Enable\"
fi

echo \"\"
echo -e \"${GREEN}╔════════════════════════════════════════════════════════╗${NC}\"
echo -e \"${GREEN}║  ✓ Установка завершена!                              ║${NC}\"
echo -e \"${GREEN}╚════════════════════════════════════════════════════════╝${NC}\"

echo \"\"
echo -e \"${BLUE}Следующие шаги:${NC}\"
echo -e \"  1. Перезагрузитесь (если включили камеру): ${YELLOW}sudo reboot${NC}\"
echo -e \"  2. Проверьте компоненты: ${YELLOW}python3 checklist.py${NC}\"
echo -e \"  3. Запустите систему: ${YELLOW}python3 main.py${NC}\"
echo \"\"
echo -e \"${BLUE}Опционально:${NC}\"
echo -e \"  • Тестирование: ${YELLOW}python3 test_components.py${NC}\"
echo -e \"  • RS-485 тест: ${YELLOW}sudo python3 test_rs485.py${NC}\"
echo -e \"  • Автозапуск: см. ${YELLOW}CONFIG.md${NC}\"
echo \"\"
