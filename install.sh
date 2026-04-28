#!/bin/bash
# Скрипт установки системы записи видео на Raspberry Pi Zero 2W

set -e  # Выход при ошибке

echo "=========================================="
echo "Установка системы записи видео на Pi"
echo "=========================================="

# 1. Обновление системы
echo ""
echo "[1/5] Обновление пакетов системы..."
sudo apt update
sudo apt upgrade -y

# 2. Установка зависимостей
echo ""
echo "[2/5] Установка зависимостей..."
sudo apt install -y \
    libcamera-apps \
    python3-pip \
    ffmpeg \
    git

# 3. Активация камеры
echo ""
echo "[3/5] Проверка камеры..."
if grep -q "camera_auto_detect=1" /boot/firmware/config.txt; then
    echo "  ✓ Камера уже включена в конфиге"
else
    echo "  ! Камера может быть отключена"
    echo "  ! Запустите: sudo raspi-config → Interface Options → Camera"
    echo "  ! И перезагрузитесь"
fi

# 4. Установка Python пакетов
echo ""
echo "[4/5] Установка Python зависимостей..."
pip install --upgrade pip
pip install \
    pyserial \
    gpiozero

# 5. Проверка UART
echo ""
echo "[5/5] Проверка UART..."
if [ -e /dev/serial0 ]; then
    echo "  ✓ /dev/serial0 найден"
elif [ -e /dev/ttyAMA0 ]; then
    echo "  ✓ /dev/ttyAMA0 найден (используйте этот порт)"
else
    echo "  ! UART не найден, проверьте расширенные опции в raspi-config"
fi

echo ""
echo "=========================================="
echo "✓ Установка завершена!"
echo "=========================================="
echo ""
echo "Следующие шаги:"
echo "1. Убедитесь, что камера включена в raspi-config"
echo "2. Перезагрузитесь: sudo reboot"
echo "3. Запустите систему: python3 main.py"
echo ""
echo "Проверка:"
echo "  libcamera-vid --version"
echo "  python3 -c 'import serial; print(\"OK\")'"
echo "  gpio readall | grep 17  (должна быть строка с GPIO17)"
