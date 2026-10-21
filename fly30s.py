#!/usr/bin/env python3
"""Fly a ~30s square circuit in PX4 SITL, record video + telemetry, then land."""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# Telemetry logger
# ---------------------------------------------------------------------------
class TelemetryLogger:
    """Ghi LOCAL_POSITION_NED / ATTITUDE / GLOBAL_POSITION_INT / GPS_RAW_INT ra CSV.

    Mỗi dòng: t_wall, t_boot, msg_type, + các field quan trọng.
    Dùng để map frame video <-> vị trí drone.
    """

    FIELDS = [
        't_wall', 't_boot', 'msg',
        # LOCAL_POSITION_NED
        'x', 'y', 'z', 'vx', 'vy', 'vz',
        # ATTITUDE
        'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed',
        # GLOBAL_POSITION_INT
        'lat', 'lon', 'alt', 'relative_alt', 'hdg',
        # GPS_RAW_INT
        'fix_type', 'satellites_visible',
    ]

    def __init__(self, master: mavutil.mavfile, out_dir: Path) -> None:
        self.master = master
        self.path = out_dir / 'telemetry.csv'
        self._fp = open(self.path, 'w', newline='')
        self._writer = csv.DictWriter(self._fp, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self._fp.flush()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._t_start = time.time()
        print(f'[tel] logging to {self.path}')

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._fp.close()
        print('[tel] stopped')

    def _row(self, msg_type: str, **kw) -> None:
        row = {k: '' for k in self.FIELDS}
        row['t_wall'] = f'{time.time():.6f}'
        row['t_boot'] = f'{time.time() - self._t_start:.6f}'
        row['msg'] = msg_type
        for k, v in kw.items():
            row[k] = v
        self._writer.writerow(row)

    def _loop(self) -> None:
        while not self._stop.is_set():
            msg = self.master.recv_match(
                type=['LOCAL_POSITION_NED', 'ATTITUDE',
                      'GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
                blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'LOCAL_POSITION_NED':
                self._row(t, x=msg.x, y=msg.y, z=msg.z,
                          vx=msg.vx, vy=msg.vy, vz=msg.vz)
            elif t == 'ATTITUDE':
                self._row(t, roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw,
                          rollspeed=msg.rollspeed, pitchspeed=msg.pitchspeed,
                          yawspeed=msg.yawspeed)
            elif t == 'GLOBAL_POSITION_INT':
                self._row(t, lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                          alt=msg.alt / 1000.0,
                          relative_alt=msg.relative_alt / 1000.0,
                          hdg=msg.hdg / 100.0)
            elif t == 'GPS_RAW_INT':
                self._row(t, fix_type=msg.fix_type,
                          satellites_visible=msg.satellites_visible)
            if self._fp.tell() > 8192:  # flush theo lô nhỏ
                self._fp.flush()


# ---------------------------------------------------------------------------
# Video recorder
# ---------------------------------------------------------------------------
class VideoRecorder:
    """Ghi video từ Gazebo camera topic bằng ffmpeg (Gazebo Classic hoặc Garden).

    Cần biết tên topic camera của bạn. Ví dụ:
      - Gazebo Classic: gz topic -l | grep camera
      - Gazebo (ignition/gz-sim): gz topic -l | grep camera
    """

    def __init__(self, out_path: Path, topic: str, tool: str = 'gz',
                 fps: int = 30, width: int = 1280, height: int = 720,
                 duration_s: float | None = None) -> None:
        self.out_path = out_path
        self.topic = topic
        self.tool = tool
        self.fps = fps
        self.width = width
        self.height = height
        self.duration_s = duration_s
        self.proc: subprocess.Popen | None = None

    def start(self) -> bool:
        if shutil.which('ffmpeg') is None:
            print('[vid] WARN: ffmpeg không có trong PATH -> bỏ qua ghi video')
            return False

        # 1) Lấy raw image stream từ Gazebo qua 'gz topic -e ...'
        #    Đây là cách đơn giản nhất, không cần ROS.
        cmd_topic = [self.tool, 'topic', '-e', '-t', self.topic]
        try:
            reader = subprocess.Popen(
                cmd_topic, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            print(f"[vid] WARN: không tìm thấy lệnh '{self.tool}' -> bỏ qua video")
            return False

        # 2) Pipe sang ffmpeg -> mp4 (image2pipe)
        #    LƯU Ý: 'gz topic -e' in ra text protobuf, KHÔNG phải raw frame.
        #    Cách này chỉ hoạt động nếu bạn có script chuyển text->raw.
        #    => Khuyến nghị dùng ROS2 (xem ghi chú bên dưới).
        print('[vid] WARN: ghi trực tiếp từ gz topic cần parser ảnh. '
              'Xem hướng dẫn ROS2 trong ghi chú.')
        reader.terminate()
        return False


# ---------------------------------------------------------------------------
# (Phương án khuyến nghị) Ghi video qua ROS2 image topic
# ---------------------------------------------------------------------------
def start_ros2_video_recorder(out_path: Path, image_topic: str) -> subprocess.Popen | None:
    """Ghi video từ một ROS2 sensor_msgs/Image topic bằng ros2 + ffmpeg.

    Yêu cầu:
      - ROS2 đã source (Foxy/Humble).
      - Gói 'image_transport' + 'cv_bridge' hoặc dùng node có sẵn:
          ros2 run image_view video_recorder --ros-args -r image:=<topic>
      - Hoặc dùng 'ros2 bag record <topic>' rồi convert sau (an toàn nhất).
    """
    if shutil.which('ros2') is None:
        print('[vid] WARN: không có ros2 trong PATH')
        return None

    # Cách đơn giản nhất: ghi rosbag, convert sang mp4 sau khi bay xong.
    bag_dir = out_path.with_suffix('')
    cmd = ['ros2', 'bag', 'record', '-o', str(bag_dir), image_topic]
    print(f'[vid] ros2 bag record -> {bag_dir}')
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


# ---------------------------------------------------------------------------
# MAVLink helpers (giữ nguyên từ code cũ)
# ---------------------------------------------------------------------------
def connect(timeout_s: float = 15.0) -> mavutil.mavfile:
    master = mavutil.mavlink_connection(
        'udpin:0.0.0.0:14540',
        source_system=255,
        source_component=190,
    )
    print('[fly] waiting for heartbeat on udpin:14540 ...')
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None:
            print(f'[fly] heartbeat from sys={master.target_system} comp={master.target_component}')
            return master
    raise TimeoutError('No MAVLink heartbeat (is PX4 SITL running?)')


def set_mode(master, mode_name: str) -> None:
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise RuntimeError(f'Mode {mode_name} not in mapping: {sorted(mode_map)}')
    base_mode, custom_mode, custom_sub_mode = mode_map[mode_name]
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(base_mode), float(custom_mode), float(custom_sub_mode),
        0, 0, 0, 0,
    )
    print(f'[fly] set mode {mode_name} ({base_mode},{custom_mode},{custom_sub_mode})')


def send_gcs_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def wait_command_ack(master, command: int, timeout_s: float = 3.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return msg.result
        text = master.recv_match(type='STATUSTEXT', blocking=False)
        if text:
            print(f'[fly] {text.text}')
    return None


def arm(master, arming: bool = True) -> None:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0 if arming else 0.0, 0, 0, 0, 0, 0, 0,
    )
    print('[fly] arm' if arming else '[fly] disarm')
    result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    if arming and result not in (None, mavutil.mavlink.MAV_RESULT_ACCEPTED):
        print(f'[fly] WARN: arm rejected (result={result}).')


def land(master) -> None:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, float('nan'), float('nan'), float('nan'), 0,
    )
    print('[fly] land')


def send_position(master, x, y, z_up, yaw) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        x, y, -z_up,
        0, 0, 0,
        0, 0, 0,
        yaw, 0,
    )


def get_local_pose(master, timeout_s: float = 2.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if msg:
            return msg.x, msg.y, -msg.z
    return None


def hold_offboard_stream(master, x, y, z_up, yaw, seconds) -> None:
    t_end = time.time() + seconds
    while time.time() < t_end:
        send_position(master, x, y, z_up, yaw)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description='PX4 SITL fly + record demo')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--alt', type=float, default=5.0)
    parser.add_argument('--radius', type=float, default=6.0)
    parser.add_argument('--pattern', choices=('square', 'circle'), default='square')
    parser.add_argument('--out', type=str, default='runs',
                        help='Thư mục gốc chứa output (mặc định: runs/)')
    parser.add_argument('--video-topic', type=str, default=None,
                        help='ROS2 image topic để ghi, vd: /camera/image_raw')
    parser.add_argument('--no-video', action='store_true',
                        help='Không ghi video')
    args = parser.parse_args()

    # --- Thư mục output có timestamp ---
    run_dir = Path(args.out) / datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f'[run] output dir: {run_dir.resolve()}')

    # --- Kết nối PX4 ---
    master = connect()

    # --- Yêu cầu stream dữ liệu ---
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 200000,
        0, 0, 0, 0, 0)

    for _ in range(40):
        send_gcs_heartbeat(master)
        master.recv_match(blocking=False)
        time.sleep(0.02)

    # --- Telemetry logger (chạy song song) ---
    tel = TelemetryLogger(master, run_dir)
    tel.start()

    # --- Video recorder ---
    vid_proc: subprocess.Popen | None = None
    if not args.no_video and args.video_topic:
        vid_proc = start_ros2_video_recorder(run_dir / 'video', args.video_topic)

    # --- Lấy home pose ---
    home = get_local_pose(master, timeout_s=5.0)
    if home is None:
        print('[fly] warn: no LOCAL_POSITION_NED yet; assuming origin')
        home = (0.0, 0.0, 0.0)
    hx, hy, hz = home
    print(f'[fly] local pose NED-up ≈ ({hx:.2f}, {hy:.2f}, {hz:.2f})')

    cruise_z = max(args.alt, hz + 1.0)
    yaw0 = 0.0

    # --- Seed setpoint trước khi vào OFFBOARD ---
    for _ in range(20):
        send_gcs_heartbeat(master)
        send_position(master, hx, hy, cruise_z, yaw0)
        time.sleep(0.05)

    set_mode(master, 'OFFBOARD')
    time.sleep(0.3)
    arm(master, True)

    # --- Climb ---
    print('[fly] climbing ...')
    climb_end = time.time() + 8.0
    last_report = 0.0
    while time.time() < climb_end:
        send_gcs_heartbeat(master)
        send_position(master, hx, hy, cruise_z, yaw0)
        text = master.recv_match(type='STATUSTEXT', blocking=False)
        if text:
            print(f'[fly] {text.text}')
        now = time.time()
        if now - last_report >= 1.0:
            pose = get_local_pose(master, timeout_s=0.5)
            if pose:
                print(f'[fly] alt={pose[2]:.2f} m (target {cruise_z:.1f} m)')
            last_report = now
        time.sleep(0.05)

    # --- Bay theo pattern ---
    print(f'[fly] flying {args.pattern} for {args.duration:.0f}s @ {cruise_z:.1f} m')
    t0 = time.time()
    r = args.radius
    while True:
        elapsed = time.time() - t0
        if elapsed >= args.duration:
            break
        if args.pattern == 'square':
            leg = int((elapsed / args.duration) * 4) % 4
            corners = [
                (hx + r, hy + r),
                (hx + r, hy - r),
                (hx - r, hy - r),
                (hx - r, hy + r),
            ]
            tx, ty = corners[leg]
            yaw = math.atan2(ty - hy, tx - hx)
        else:
            ang = 2.0 * math.pi * (elapsed / args.duration)
            tx = hx + r * math.cos(ang)
            ty = hy + r * math.sin(ang)
            yaw = ang + math.pi / 2.0
        send_position(master, tx, ty, cruise_z, yaw)
        time.sleep(0.05)

    # --- Về home + land ---
    print('[fly] returning / landing ...')
    hold_offboard_stream(master, hx, hy, cruise_z, yaw0, 3.0)
    set_mode(master, 'LAND')
    land(master)

    t_land = time.time()
    while time.time() - t_land < 25.0:
        pose = get_local_pose(master, timeout_s=1.0)
        if pose and pose[2] < 0.4:
            print('[fly] on ground')
            break
        time.sleep(0.5)

    arm(master, False)
    print('[fly] done')

    # --- Dọn dẹp ---
    if vid_proc is not None:
        print('[vid] stopping ros2 bag ...')
        vid_proc.send_signal(signal.SIGINT)
        try:
            vid_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            vid_proc.kill()
    tel.stop()

    print(f'[run] outputs saved to: {run_dir.resolve()}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\n[fly] interrupted')
        raise SystemExit(130)
    except Exception as exc:  # noqa: BLE001
        print(f'[fly] ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)