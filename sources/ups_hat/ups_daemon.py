#!/usr/bin/env python3
import os
import smbus
import time
import subprocess
import signal
from pathlib import Path

ADDR = 0x2d
LOW_VOL = 3150  # mV
STATUS_FILE = "/tmp/ups_status"
IMAGE_DIR = Path(os.environ.get("QUADRF_UPS_IMAGES", "/usr/share/quadrf/ups/images"))

def get_i2c_bus():
    try:
        return smbus.SMBus(1)
    except Exception:
        return None

def send_notification(title, message):
    subprocess.Popen(['notify-send', '-u', 'critical', '-i', 'battery-caution', title, message])

def format_time(minutes):
    # 0xFFFF (65535) indicates the IC is still calculating the baseline
    if minutes == 65535 or minutes == 0:
        return "--h--m"
    h = minutes // 60
    m = minutes % 60
    return f"{h}h{m:02d}m"

def main():
    bus = get_i2c_bus()
    warning_triggered = False
    shutdown_counter = 60

    while True:
        try:
            if not bus:
                bus = get_i2c_bus()
                if not bus:
                    raise IOError("Bus not initialized")

            # Read Battery Voltage, Current, Percentage, and Time Estimates
            data = bus.read_i2c_block_data(ADDR, 0x20, 0x0C)
            
            vol_mv = data[0] | data[1] << 8
            
            # Handle two's complement for current
            cur_raw = data[2] | data[3] << 8
            cur_ma = cur_raw - 0xFFFF if cur_raw > 0x7FFF else cur_raw
            
            pct = data[4] | data[5] << 8
            
            # Extract time metrics
            time_empty_min = data[8] | data[9] << 8
            time_full_min = data[10] | data[11] << 8
            
            v = vol_mv / 1000.0
            is_charging = cur_ma > 0
            
            if is_charging:
                status_char = "?"
                time_str = format_time(time_full_min)
            else:
                status_char = "??"
                time_str = format_time(time_empty_min)
            
            # Read Cell Voltages for safety shutdown
            data_cells = bus.read_i2c_block_data(ADDR, 0x30, 0x08)
            cells = [
                data_cells[0] | data_cells[1] << 8,
                data_cells[2] | data_cells[3] << 8,
                data_cells[4] | data_cells[5] << 8,
                data_cells[6] | data_cells[7] << 8
            ]

            charge_math = 1 if is_charging else 0
            img_index = int(pct / 10 + charge_math * 11)
            icon_path = IMAGE_DIR / f"battery.{img_index}.png"

            # tint2 expects line 1 to be the icon path, and line 2 to be the text
            with open(STATUS_FILE, "w") as f:
                f.write(f"{icon_path}\n{pct}% {v:.1f}V ({time_str})\n")
            # Auto-shutdown logic
            low_cell = any(cell < LOW_VOL for cell in cells)
            if low_cell and not is_charging:
                if not warning_triggered:
                    send_notification("Battery Critical", "System will shutdown in 60 seconds unless plugged in.")
                    warning_triggered = True
                
                shutdown_counter -= 2 # Decrement by loop sleep interval
                if shutdown_counter <= 0:
                    os.system("i2cset -y 1 0x2d 0x01 0x55")
                    os.system("sudo poweroff")
            else:
                warning_triggered = False
                shutdown_counter = 60

        except Exception:
            # If the HAT is not detected or I2C fails, write an empty string.
            # tint2 will read this and hide the panel element entirely.
            try:
                with open(STATUS_FILE, "w") as f:
                    f.write("")
            except Exception:
                pass

        time.sleep(2)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()