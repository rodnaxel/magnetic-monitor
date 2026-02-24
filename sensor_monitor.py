import struct
import sys
import time
import csv
import os
from collections import deque
from datetime import datetime
import argparse

from rich import print
from rich.console import Console, Group
from rich.table import Table
from rich.text import Text
from rich.live import Live

import serial
from serial.tools.list_ports import comports


# Константы для протокола DORIENT
HEADER = bytes([0x0D, 0x0A, 0x7E])
DORIENT_ID = 0x70
DORIENT_DATA_LEN = 18

# Количество пакетов для усреднения
FREQ_WINDOW = 20


class FrequencyEstimator:
    """Класс для оценки частоты поступления пакетов
    на основе скользящего окна последних временных меток."""

    def __init__(self, window: int = FREQ_WINDOW):
        self.timestamps = deque(maxlen=window)

    def tick(self) -> None:
        self.timestamps.append(time.monotonic())

    def hz(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        span = self.timestamps[-1] - self.timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / span

    def interval_ms(self) -> float:
        hz = self.hz()
        return (1000.0 / hz) if hz > 0 else 0.0


class DataLogger:
    """Класс сохранения данных в файл CSV.
    Принимает словарь с данными, дополняет его временной меткой и сохраняет в файл."""

    COLUMNS = ["timestamp", "heading", "pitch", "roll", "CH", "BH", "ZH", "C", "B", "Z"]

    def __init__(self, path: str):
        self.path = path
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.COLUMNS)
        self._writer.writeheader()
        self._file.flush()
        self.count = 0

    def write(self, parsed: dict) -> None:
        row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "heading": round(parsed["azimuth_deg"], 1),
            "pitch": round(parsed["pitch_deg"], 2),
            "roll": round(parsed["roll_deg"], 2),
            "CH": round(parsed["acc_right"], 2),
            "BH": round(parsed["acc_fwd"], 2),
            "ZH": round(parsed["acc_up"], 2),
            "C": round(parsed["mag_right"], 2),
            "B": round(parsed["mag_fwd"], 2),
            "Z": round(parsed["mag_up"], 2),
        }
        self._writer.writerow(row)
        self._file.flush()
        self.count += 1

    def close(self) -> None:
        self._file.close()


class Measurement:
    """Класс для хранения и обработки текущих измерений, 
    а также для отслеживания максимальных и минимальных значений каждого параметра."""
    KEYS = (
        "azimuth_deg", "pitch_deg", "roll_deg",
        "acc_right", "acc_fwd", "acc_up",
        "mag_right", "mag_fwd", "mag_up",
    )
    WINDOW = 20

    def __init__(self):
        self._values = {k: deque([0.0], maxlen=self.WINDOW) for k in self.KEYS}
        self._max = dict.fromkeys(self.KEYS, 0.0)
        self._min = dict.fromkeys(self.KEYS, 0.0)

    def max_value(self, key: str) -> float:
        return self._max[key]

    def min_value(self, key: str) -> float:
        return self._min[key]

    def average_value(self, key: str) -> float:
        vals = self._values[key]
        return sum(vals) / len(vals) if vals else 0.0

    def update(self, new_values: dict) -> None:
        for key, value in new_values.items():
            self._values[key].append(value)

            if value > self._max[key]:
                self._max[key] = value
            elif value < self._min[key]:
                self._min[key] = value

    def reset(self):
        """Сброс всех накопленных данных."""
        for k in self.KEYS:
            self._values[k].clear()
            self._values[k].append(0.0)
            self._max[k] = 0.0
            self._min[k] = 0.0
                

def kang_to_degrees(raw: int, signed: bool = False) -> float:
    """Преобразует 16-битное значение из датчика в градусы."""
    if signed:
        if raw > 32767:
            raw -= 65536
        return raw * 360.0 / 65536.0
    else:
        return (raw & 0xFFFF) * 360.0 / 65536.0


def int16_to_utesla(value: int) -> float:
    return value / 75.0


def parse_dorient(data: bytes, verify_enable: bool = False) -> dict:
    (
        roll_raw,
        pitch_raw,
        az_raw,
        acc_right,
        acc_fwd,
        acc_up,
        mag_right,
        mag_fwd,
        mag_up,
    ) = struct.unpack_from("<hhhhhhhhh", data)

    return {
        "roll_deg": kang_to_degrees(roll_raw, signed=True),
        "pitch_deg": kang_to_degrees(pitch_raw, signed=True),
        "azimuth_deg": kang_to_degrees(az_raw, signed=False),
        "acc_right": int16_to_utesla(acc_right),
        "acc_fwd": int16_to_utesla(acc_fwd),
        "acc_up": int16_to_utesla(acc_up),
        "mag_right": int16_to_utesla(mag_right),
        "mag_fwd": int16_to_utesla(mag_fwd),
        "mag_up": int16_to_utesla(mag_up),
    }


def verify_checksum(packet: bytes) -> bool:
    """Проверяет контрольную сумму пакета."""
    return (sum(packet[:-1]) & 0xFF) == packet[-1]


def fake_find_and_read_packet(ser: serial.Serial, verify_enable: bool = False) -> bytes:
    """Фейковая функция для тестирования интерфейса без реального устройства."""
    time.sleep(0.1)  # имитируем задержку между пакетами
    data = struct.pack(
        "<hhhhhhhhh",
        1000,  # roll_raw
        -500,  # pitch_raw
        20000,  # az_raw
        3750,  # acc_right (50 uT)
        -2250,  # acc_fwd (-30 uT)
        1500,  # acc_up (20 uT)
        7500,  # mag_right (100 uT)
        -6000,  # mag_fwd (-80 uT)
        4500,  # mag_up (60 uT)
    )
    pkt_id = DORIENT_ID
    count = DORIENT_DATA_LEN
    packet = HEADER + bytes([pkt_id, count]) + data
    checksum = sum(packet) & 0xFF
    return packet + bytes([checksum])


def find_and_read_packet(
    ser: serial.Serial, verify_enable: bool = False
) -> bytes | None:
    """Считывает данные из последовательного порта, ищет заголовок и возвращает полный пакет DORIENT."""
    buf = bytearray()

    # Ищем заголовок 0x0D 0x0A 0x7E
    while True:
        byte = ser.read(1)
        if not byte:
            return None
        buf += byte
        if len(buf) >= 3 and buf[-3:] == HEADER:
            break

    # После заголовка должны идти ID пакета и длина данных
    pkt_id_b = ser.read(1)
    count_b = ser.read(1)
    if not pkt_id_b or not count_b:
        return None

    # Извлекаем ID и длину данных
    pkt_id = pkt_id_b[0]
    count = count_b[0]

    # Читаем оставшуюся часть пакета (данные + контрольная сумма)
    remainder = ser.read(count + 1)
    if len(remainder) < count + 1:
        return None

    full_packet = HEADER + bytes([pkt_id, count]) + remainder

    if pkt_id != DORIENT_ID:
        return None

    if count != DORIENT_DATA_LEN:
        print(f"[Warning] : {count} (ожидаем {DORIENT_DATA_LEN})")
        return None

    # Проверка контрольной суммы отключена, так как некоторые устройства могут её не использовать или считать неправильно.
    if verify_enable and not verify_checksum(full_packet):
        print("[WARN] Ошибка контрольной суммы (пакет проигнорирован)")
        return None

    return full_packet


def show_available_ports() -> None:
    console = Console()
    
    if not (serial_ports := list(comports())):
        console.print("No serial ports found.", style="bold red")
        return
    
    table = Table(title="Available Serial Ports")
    table.add_column("Port", style="cyan", width=10, no_wrap=True)
    table.add_column("Description", width=30, style="magenta")
    table.add_column("HWID", width=25, style="green")
    table.add_column("Manufacturer", width=20, style="yellow")

    for port in serial_ports:
        table.add_row(
            port.device, port.description, port.hwid, port.manufacturer or "N/A"
        )
        
    console.print(table)


def create_table(
    d: dict,
    meas: Measurement,
    freq: FrequencyEstimator,
    logger: DataLogger | None,
    total: int,
    errors: int,
) -> Table:
    table = Table(title="Magnetic sensor data monitor")

    table.add_column("Parameter", justify="left", style="cyan", width=12)
    table.add_column("Value", justify="center", width=12)
    table.add_column("Max", justify="center", width=12)
    table.add_column("Min", justify="center", width=12) 
    table.add_column("Unit", justify="center", style="dim", width=5)

    table.add_row("Heading", f"{d['azimuth_deg']:>8.2f}", f"{meas.max_value('azimuth_deg'):>8.2f}", f"{meas.min_value('azimuth_deg'):>8.2f}", "deg")
    table.add_row("Pitch", f"{d['pitch_deg']:>+8.2f}", f"{meas.max_value('pitch_deg'):>8.2f}", f"{meas.min_value('pitch_deg'):>8.2f}", "deg")
    table.add_row("Roll", f"{d['roll_deg']:>+8.2f}", f"{meas.max_value('roll_deg'):>8.2f}", f"{meas.min_value('roll_deg'):>8.2f}", "deg")
    table.add_section()
    table.add_row("Mag C (H)", f"{d['acc_right']:>+8.2f}", f"{meas.max_value('acc_right'):>8.2f}", f"{meas.min_value('acc_right'):>8.2f}", "uT")
    table.add_row("Mag B (H)", f"{d['acc_fwd']:>+8.2f}", f"{meas.max_value('acc_fwd'):>8.2f}", f"{meas.min_value('acc_fwd'):>8.2f}", "uT")
    table.add_row("Mag Z (H)", f"{d['acc_up']:>+8.2f}", f"{meas.max_value('acc_up'):>8.2f}", f"{meas.min_value('acc_up'):>8.2f}", "uT")
    table.add_section()
    table.add_row("Mag C", f"{d['mag_right']:>+8.2f}", f"{meas.max_value('mag_right'):>8.2f}", f"{meas.min_value('mag_right'):>8.2f}", "uT")
    table.add_row("Mag B", f"{d['mag_fwd']:>+8.2f}", f"{meas.max_value('mag_fwd'):>8.2f}", f"{meas.min_value('mag_fwd'):>8.2f}", "uT")
    table.add_row("Mag Z", f"{d['mag_up']:>+8.2f}", f"{meas.max_value('mag_up'):>8.2f}", f"{meas.min_value('mag_up'):>8.2f}", "uT")
    table.add_section()
    table.add_row("Freq", f"{freq.hz():>8.2f}", "Hz")
    table.add_row("Interval", f"{freq.interval_ms():>8.1f}", "ms")
    table.add_section()
    table.add_row("Packets", f"{total:<10d}", "")
    table.add_row("Errors", f"{errors:<6d}", "")

    return table


def create_log_line(logger: DataLogger | None) -> Text:
    if logger:
        text = Text()
        text.append("  Log: ", style="dim")
        text.append(logger.path, style="cyan")
        text.append(f"  ({logger.count} rows)", style="dim")
    else:
        text = Text("  Log: disabled", style="dim")
    return text


def create_renderable(
    d: dict,
    meas: Measurement,
    freq: FrequencyEstimator,
    logger: DataLogger | None,
    total: int,
    errors: int,
) -> Group:
    table = create_table(d, meas, freq, logger, total, errors)
    log_line = create_log_line(logger)
    return Group(table, log_line)


def get_args() -> argparse.Namespace:
    """Парсит аргументы командной строки и возвращает их в виде объекта."""
    parser = argparse.ArgumentParser(
        description="Magnetic sensor data monitor with console output and optional CSV logging",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("port", help="Serial port  (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument(
        "-l",
        "--log",
        metavar="FILE",
        help="CSV log file path  (logging disabled if omitted)",
    )
    parser.add_argument(
        "-w",
        "--window",
        type=int,
        default=FREQ_WINDOW,
        help="Sliding window size for frequency estimation",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Enable checksum verification (may cause packet drops if device doesn't use correct checksums)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output",
    )
    return parser.parse_args()


def main():
    args = get_args()

    # Если указанный путь для логирования является директорией, 
    # создаем файл с уникальным именем внутри этой директории
    log_path = args.log
    if log_path and os.path.isdir(log_path):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_path, f"magsensor_{ts}.csv")
        print("[INFO] Log path is a directory, will create log file:", log_path)

    # Открываем последовательный порт
    # если не удается открыть, выводим ошибку и список доступных портов и выходим
    print(f"Opening {args.port} at {args.baud} baud …")
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"[red bold][ERROR][/] Cannot open port: {e}")
        show_available_ports()
        sys.exit(1)

    # Инициализируем логгер, если указано
    logger = None
    if log_path:
        logger = DataLogger(log_path)
        print(f"Logging to: {log_path}")

    # Инициализируем оценщик частоты и счетчики пакетов и ошибок
    freq = FrequencyEstimator(window=args.window)
    total = 0
    errors = 0

    # Основной цикл чтения данных и обновления интерфейса
    print("Listening for DORIENT packets … (Ctrl+C to quit)\n")

    if args.debug:
        print("[red bold][DEBUG MODE ENABLED] Using fake data generator instead of serial input.\n")
        find_and_read_packet_fn = fake_find_and_read_packet
    else:
        find_and_read_packet_fn = find_and_read_packet


    meas = Measurement()
    
    try:
        with Live(console=Console(), refresh_per_second=10, screen=False) as live:
            while True:
                
                packet = find_and_read_packet_fn(ser=ser, verify_enable=args.verify)
                
                if packet is None:
                    errors += 1
                    continue

                total += 1
                freq.tick()

                data = packet[5:-1]

                parsed = parse_dorient(data, verify_enable=args.verify)
                
                meas.update(parsed)
                
                if logger:
                    logger.write(parsed)

                live.update(create_renderable(parsed, meas, freq, logger, total, errors))

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        if logger:
            logger.close()
            print(f"Log saved → {logger.path}  ({logger.count} rows)")


if __name__ == "__main__":
    main()
