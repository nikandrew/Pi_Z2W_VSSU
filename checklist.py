#!/usr/bin/env python3
"""
Чек-лист и проверка готовности системы к запуску.
Запустите перед первым использованием: python3 checklist.py
"""

import subprocess
import sys
from pathlib import Path


class ChecklistItem:
    def __init__(self, name: str, description: str, critical: bool = False):
        self.name = name
        self.description = description
        self.critical = critical
        self.passed = False
        self.message = ""
    
    def check(self) -> None:
        """Переопределяется в подклассах."""
        pass
    
    def status_icon(self) -> str:
        if self.passed:
            return "✓"
        elif self.critical:
            return "✗"
        else:
            return "⚠"
    
    def print_result(self) -> None:
        status = self.status_icon()
        print(f"  {status} {self.name}")
        if self.message:
            print(f"     {self.message}")


class FileCheckItem(ChecklistItem):
    def __init__(self, filepath: str, critical: bool = False):
        super().__init__(f"Файл: {filepath}", "Проверка наличия файла", critical)
        self.filepath = Path(filepath)
    
    def check(self) -> None:
        if self.filepath.exists():
            self.passed = True
            self.message = f"✓ Найден: {self.filepath.absolute()}"
        else:
            self.message = f"✗ Не найден: {self.filepath}"


class DirectoryCheckItem(ChecklistItem):
    def __init__(self, dirpath: str, create: bool = False):
        super().__init__(f"Папка: {dirpath}", "Проверка наличия папки", critical=False)
        self.dirpath = Path(dirpath)
        self.create = create
    
    def check(self) -> None:
        if self.dirpath.exists():
            self.passed = True
            self.message = f"Существует"
        else:
            if self.create:
                self.dirpath.mkdir(parents=True, exist_ok=True)
                self.passed = True
                self.message = f"Создана автоматически"
            else:
                self.message = f"Требуется создать"


class CommandCheckItem(ChecklistItem):
    def __init__(self, command: str, description: str = "", critical: bool = False):
        super().__init__(
            f"Команда: {command.split()[0]}",
            description or f"Проверка доступности {command}",
            critical
        )
        self.command = command
    
    def check(self) -> None:
        try:
            result = subprocess.run(
                self.command.split(),
                capture_output=True,
                timeout=5
            )
            if result.returncode == 0:
                self.passed = True
                self.message = "Доступна"
            else:
                self.message = f"Доступна, но ошибка (код {result.returncode})"
        except FileNotFoundError:
            self.message = f"Не найдена (установите: apt install {self.command.split()[0]})"
        except Exception as e:
            self.message = f"Ошибка: {e}"


class PythonPackageCheckItem(ChecklistItem):
    def __init__(
        self,
        package: str,
        import_name: str = None,
        install_hint: str = None,
        critical: bool = False,
    ):
        super().__init__(
            f"Пакет: {package}",
            f"Проверка установки Python пакета",
            critical
        )
        self.package = package
        self.import_name = import_name or package.replace("-", "_")
        self.install_hint = install_hint or f"sudo apt install -y python3-{package}"
    
    def check(self) -> None:
        try:
            __import__(self.import_name)
            self.passed = True
            self.message = "Установлен"
        except ImportError:
            self.message = f"Не установлен ({self.install_hint})"


def main() -> int:
    print("\n" + "=" * 60)
    print("ЧЕК-ЛИСТ ГОТОВНОСТИ СИСТЕМЫ")
    print("=" * 60)
    
    items = [
        # Критические файлы
        FileCheckItem("main.py", critical=True),
        FileCheckItem("test_components.py", critical=False),
        FileCheckItem("test_rs485.py", critical=False),
        
        # Папки (создаем при необходимости)
        DirectoryCheckItem("videos", create=True),
        DirectoryCheckItem("logs", create=True),
        
        # Команды
        CommandCheckItem("rpicam-vid --version", "rpicam (запись видео)", critical=True),
        CommandCheckItem("ffmpeg -version", "ffmpeg (разбиение видео)", critical=False),
        CommandCheckItem("python3 --version", "Python 3", critical=True),
        
        # Python пакеты
        PythonPackageCheckItem(
            "pyserial",
            import_name="serial",
            install_hint="sudo apt install -y python3-serial",
            critical=True,
        ),
        PythonPackageCheckItem(
            "gpiozero",
            install_hint="sudo apt install -y python3-gpiozero",
            critical=True,
        ),
    ]
    
    # Проверяем все
    passed_count = 0
    critical_failed = 0
    
    for item in items:
        item.check()
        item.print_result()
        
        if item.passed:
            passed_count += 1
        elif item.critical:
            critical_failed += 1
    
    # Итоги
    print("\n" + "=" * 60)
    print("ИТОГИ")
    print("=" * 60)
    
    print(f"\n✓ Проверено: {len(items)}")
    print(f"✓ Пройдено: {passed_count}")
    print(f"✗ Ошибок: {len(items) - passed_count}")
    
    if critical_failed > 0:
        print(f"\n⚠ КРИТИЧЕСКИЕ ОШИБКИ: {critical_failed}")
        print("\nДействия:")
        print("1. Выполните install.sh: bash install.sh")
        print("2. Перезагрузитесь: sudo reboot")
        print("3. Запустите этот чек-лист еще раз")
        return 1
    
    if passed_count == len(items):
        print("\n✓ ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ!")
        print("\nВы готовы запустить систему:")
        print("  python3 main.py")
        return 0
    else:
        print("\n⚠ Некоторые опциональные компоненты отсутствуют")
        print("Система может работать, но рекомендуется установить их")
        print("для полной функциональности.")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nОстановлено пользователем")
        sys.exit(1)
