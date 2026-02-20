# Magnetic Monitor

Консольный монитор для чтения данных датчика магнитного компаса.  
Читает пакеты DORIENT по RS-232, отображает ориентацию в реальном времени и опционально пишет лог в CSV.


## Требования

- Python 3.10+
- Подключение: RS-232, 9600 8N1 (заводские настройки)


## Выводимые данные

- Курс, крен, дифферент в градусах
- Сырые данные магнетометров (приведенные и неприведенные в горизонт)
- Частота приёма пакетов (Гц) и средний интервал (мс)
- Счётчики принятых пакетов и ошибок


## Установка
```bash
#1. Клонирование репозитория
git clone https://github.com/rodnaxel/gmessage.git

#2. Установка зависимостей
uv sync --frozen

# Скачивание uv
# [windows] powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex
# [linux] curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Использование
```bash
# Только монитор
uv run sensor_monitor.py COM3

# С логированием в файл
uv run sensor_monitor.py COM3 --log session.csv

# Указать директорию — файл создастся автоматически с timestamp
uv run sensor_monitor.py /dev/ttyUSB0 --log ./logs/

# Все параметры
uv run sensor_monitor.py COM3 --baud 19200 --log data.csv --window 30
```

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `port` | — | COM-порт (`COM3`, `/dev/ttyUSB0`) |
| `-b`, `--baud` | `9600` | Скорость соединения |
| `-l`, `--log` | — | Путь к CSV-файлу лога |
| `-w`, `--window` | `20` | Размер окна для расчёта частоты |


## Сборка исполняемого файла с помощью PyInstaller

```bash
uv run python -m PyInstaller --windowed sensor_monitor.py --name sensor_monitor
```



