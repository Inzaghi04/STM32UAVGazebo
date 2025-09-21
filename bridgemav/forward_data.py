#!/usr/bin/env python3
import serial
import time
from pymavlink import mavutil

# Cấu hình cổng serial cho STM32
SERIAL_PORT = '/dev/ttyUSB0'   # Cổng serial STM32
SERIAL_BAUD = 115200           # Baud STM32

# Cấu hình MAVLink cho PX4 SITL
MAVLINK_URI = 'udp:127.0.0.1:14550'  # PX4 SITL listen

# Giải mã custom_mode theo định dạng PX4
def decode_px4_mode(base_mode, custom_mode):
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode  = (custom_mode >> 24) & 0xFF

    mode_dict = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        4: "AUTO",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
        9: "SIMPLE"
    }

    mode_name = mode_dict.get(main_mode, "UNKNOWN")
    return f"{mode_name} (main={main_mode}, sub={sub_mode})"

def main():
    # Kết nối serial STM32
    try:
        serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        print(f"[OK] Connected to STM32 on {SERIAL_PORT} at {SERIAL_BAUD} baud")
    except serial.SerialException as e:
        print(f"[ERR] Failed to connect STM32: {e}")
        return

    # Kết nối MAVLink đến PX4
    try:
        master = mavutil.mavlink_connection(MAVLINK_URI)
        master.wait_heartbeat()
        print(f"[OK] Connected to PX4. Heartbeat from sys={master.target_system}, comp={master.target_component}")
    except Exception as e:
        print(f"[ERR] Failed to connect PX4: {e}")
        serial_port.close()
        return

    print("[INFO] Forwarding MAVLink data from STM32 → PX4 and monitoring modes...")

    # Parser MAVLink để decode gói từ STM32
    parser = mavutil.mavlink.MAVLink(None)

    # Biến lưu mode trước đó
    last_set_mode = None
    last_px4_mode = None

    while True:
        try:
            # 1. Đọc dữ liệu từ STM32
            if serial_port.in_waiting > 0:
                mav_bytes = serial_port.read(serial_port.in_waiting)
                # Forward sang PX4 SITL
                master.write(mav_bytes)

                # Parse dữ liệu MAVLink từ STM32
                for b in mav_bytes:
                    msg = parser.parse_char(bytes([b]))
                    if msg and msg.get_type() == "SET_MODE":
                        current_mode = (msg.base_mode, msg.custom_mode)
                        if current_mode != last_set_mode:
                            print(f"[STM32->PX4][SET_MODE] target_sys={msg.target_system}, "
                                  f"base_mode={msg.base_mode}, custom_mode={msg.custom_mode}")
                            last_set_mode = current_mode

            # 2. Đọc dữ liệu PX4 trả về
            msg_from_px4 = master.recv_match(blocking=False)
            if msg_from_px4 and msg_from_px4.get_type() == "HEARTBEAT":
                current_mode = (msg_from_px4.base_mode, msg_from_px4.custom_mode)
                if current_mode != last_px4_mode:
                    mode = decode_px4_mode(msg_from_px4.base_mode, msg_from_px4.custom_mode)
                    print(f"[PX4 HEARTBEAT] base_mode={msg_from_px4.base_mode}, "
                          f"custom_mode={msg_from_px4.custom_mode}, mode={mode}")
                    last_px4_mode = current_mode

        except Exception as e:
            print(f"[ERR] {e}")
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] Stopped by user")
