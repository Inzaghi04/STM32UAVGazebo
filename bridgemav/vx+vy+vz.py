#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import cv2
import numpy as np
import math
import time
from ultralytics import YOLO
from pymavlink import mavutil

# =========================
# PID CLASS
# =========================
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error):
        now = time.time()
        dt = now - self.prev_time if self.prev_time else 0.01
        self.prev_time = now

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        self.integral = max(min(self.integral, 100.0), -100.0)  # Giới hạn tích phân
        # Output
        return (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

# =========================
# YOLO + MAVLink INIT
# =========================
model = YOLO("yolov5s.pt")

# MAVLink connection to PX4 SITL
mav = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
mav.wait_heartbeat()
print(f"[OK] Connected to PX4 (sys={mav.target_system}, comp={mav.target_component})")
target_sysid = mav.target_system
target_compid = mav.target_component

# Set OFFBOARD mode
def set_offboard_mode():
    custom_mode = 6 << 16  # PX4_MAIN_MODE_OFFBOARD
    mav.mav.set_mode_send(
        target_sysid,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode
    )
    print("[MAV] OFFBOARD mode enabled")

# Send velocity setpoint
def send_velocity_target(vx, vz, yaw_rate):
    """
    vx: tiến/lùi (m/s)
    vz: lên/xuống (m/s)
    yaw_rate: tốc độ quay yaw (rad/s)
    """
    mav.mav.set_position_target_local_ned_send(
        0,
        target_sysid, target_compid,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,  # velocity + yaw_rate
        0, 0, 0,
        vx, 0.0, vz,          # vy cố định = 0.0
        0, 0, 0,
        0, yaw_rate
    )

# =========================
# GStreamer INIT
# =========================
Gst.init(None)
pipeline_str = (
    "udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=appsink sync=false"
)
pipeline = Gst.parse_launch(pipeline_str)
appsink = pipeline.get_by_name("appsink")
appsink.set_property("emit-signals", False)
appsink.set_property("max-buffers", 1)
appsink.set_property("drop", True)

# =========================
# MAIN
# =========================
def main():
    # PID cho vx (điều khiển tiến/lùi dựa trên chiều cao bbox)
    pid_vx = PID(Kp=0.002, Ki=0.0008, Kd=0.008)
    # PID cho vz (điều khiển độ cao để giữ đối tượng ở hàng dưới cùng)
    pid_vz = PID(Kp=0.01, Ki=0.0008, Kd=0.002)
    # PID cho yaw (điều khiển góc quay để căn giữa đối tượng theo trục X)
    pid_yaw = PID(Kp=0.003, Ki=0.0001, Kd=0.0005)

    desired_height = 50.0  # Chiều cao mong muốn của bbox (px)

    pipeline.set_state(Gst.State.PLAYING)
    set_offboard_mode()
    print("[INFO] Started video + PID vx, vz, and yaw control loop")

    try:
        while True:
            # Lấy frame từ camera
            sample = appsink.emit("try-pull-sample", Gst.SECOND)
            if not sample:
                continue

            buf = sample.get_buffer()
            caps = sample.get_caps()
            w = caps.get_structure(0).get_value("width")
            h = caps.get_structure(0).get_value("height")

            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                continue
            frame = np.frombuffer(map_info.data, np.uint8).reshape((h, w, 3))
            buf.unmap(map_info)

            # YOLO detect
            results = model(frame, verbose=False)
            detected_object_pos = None
            detected_height = None

            for r in results:
                for box in r.boxes:
                    if int(box.cls) == 0:  # class 'person'
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        detected_object_pos = (cx, cy)
                        detected_height = float(y2 - y1)  # Chiều cao bbox
                        break
                if detected_object_pos and detected_height:
                    break

            # Annotate
            annotated = results[0].plot()
            center_camera = (w // 2, h // 2)
            cv2.circle(annotated, center_camera, 6, (255, 0, 0), -1)  # Camera center

            # Vẽ lưới 3x3
            grid_color = (255, 255, 255)  # Màu trắng cho lưới
            thickness = 2  # Độ dày đường lưới
            grid_w = w // 3  # Chiều rộng mỗi ô
            grid_h = h // 3  # Chiều cao mỗi ô

            # Vẽ 2 đường dọc
            cv2.line(annotated, (grid_w, 0), (grid_w, h), grid_color, thickness)
            cv2.line(annotated, (2 * grid_w, 0), (2 * grid_w, h), grid_color, thickness)

            # Vẽ 2 đường ngang
            cv2.line(annotated, (0, grid_h), (w, grid_h), grid_color, thickness)
            cv2.line(annotated, (0, 2 * grid_h), (w, 2 * grid_h), grid_color, thickness)

            if detected_object_pos and detected_height:
                # Vẽ tâm đối tượng
                cv2.circle(annotated, detected_object_pos, 6, (0, 255, 0), -1)

                # Vẽ đoạn thẳng nối 2 tâm
                cv2.line(annotated, center_camera, detected_object_pos, (0, 0, 255), 2)

                # Tính độ dài đoạn thẳng
                dx = detected_object_pos[0] - center_camera[0]
                dy = detected_object_pos[1] - center_camera[1]
                distance = math.sqrt(dx * dx + dy * dy)
                distance = round(distance, 1)

                # Vị trí để hiển thị khoảng cách
                mid_point = ((center_camera[0] + detected_object_pos[0]) // 2,
                             (center_camera[1] + detected_object_pos[1]) // 2)
                cv2.putText(annotated, f"{distance:.1f}px", mid_point,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Vẽ chiều cao bbox
                cv2.putText(annotated, f"Height: {detected_height:.1f}px", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                # Xác định ô vuông chứa tâm đối tượng
                grid_x = detected_object_pos[0] // grid_w  # Cột (0, 1, 2)
                grid_y = detected_object_pos[1] // grid_h  # Hàng (0, 1, 2)
                grid_pos = f"Grid: ({grid_x}, {grid_y})"
                cv2.putText(annotated, grid_pos, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # ===== PID điều khiển vx dựa trên chiều cao bbox =====
                error_height = desired_height - detected_height
                if abs(error_height) <= 5.0:
                    vx = 0.0
                else:
                    vx = pid_vx.update(error_height)
                vx = max(min(vx, 1.0), -1.0)  # limit to [-1.0, 1.0] m/s

                # ===== PID điều khiển vz để giữ đối tượng ở hàng dưới cùng =====
                desired_y = (2 * grid_h + h) / 2  # Điểm giữa của hàng dưới cùng
                error_y = detected_object_pos[1] - desired_y  # Lỗi theo y
                if abs(error_y) <= 10.0:
                    vz = 0.0
                else:
                    vz = pid_vz.update(error_y)
                vz = max(min(vz, 1.0), -1.0)  # limit to [-1.0, 1.0] m/s

                # ===== PID điều khiển yaw để căn giữa đối tượng theo trục X =====
                error_x = detected_object_pos[0] - center_camera[0]
                if abs(error_x) <= 5:
                    yaw_rate = 0.0
                else:
                    yaw_rate = pid_yaw.update(error_x)
                yaw_rate = max(min(yaw_rate, 0.5), -0.5)  # rad/s

                # Gửi MAVLink velocity
                send_velocity_target(vx, vz, yaw_rate)
                print(f"[PID] height_error={error_height:.2f}, desired_height={desired_height:.1f}, actual_height={detected_height:.1f}, vx={vx:.2f}, "
                      f"error_y={error_y:.2f}, cy={detected_object_pos[1]:.1f}, desired_y={desired_y:.1f}, vz={vz:.2f}, "
                      f"error_x={error_x:.2f}, yaw_rate={yaw_rate:.3f}, grid=({grid_x}, {grid_y})")

            else:
                # Không detect thấy người -> hover
                send_velocity_target(0.0, 0.0, 0.0)

            # Hiển thị video
            cv2.imshow("YOLOv5 - PID vx, vz, and Yaw Control", annotated)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("[EXIT] Stopped by user")
    finally:
        pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()