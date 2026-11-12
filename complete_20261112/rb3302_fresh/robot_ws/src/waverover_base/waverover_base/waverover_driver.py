#!/usr/bin/env python3

import json
import math
import threading
import time
import subprocess
import atexit

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Float32, Float32MultiArray, Int32MultiArray
from tf2_ros import TransformBroadcaster

import serial


class DisplayServiceGuard:
    def __init__(self, service_name: str, logger):
        self.service_name = service_name
        self.logger = logger
        self.stopped_by_us = False

    def stop(self):
        try:
            result = subprocess.run(
                ["sudo", "/bin/systemctl", "stop", self.service_name],
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
            if result.returncode == 0:
                self.stopped_by_us = True
                self.logger.info(f"Stopped {self.service_name}")
            else:
                self.logger.warning(f"Could not stop {self.service_name}: {result.stderr.strip()}")
        except Exception as exc:
            self.logger.warning(f"Failed to stop {self.service_name}: {exc}")

    def start(self):
        if not self.stopped_by_us:
            return
        try:
            result = subprocess.run(
                ["sudo", "/bin/systemctl", "start", self.service_name],
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
            if result.returncode == 0:
                self.logger.info(f"Restarted {self.service_name}")
                self.stopped_by_us = False
            else:
                self.logger.warning(f"Could not restart {self.service_name}: {result.stderr.strip()}")
        except Exception as exc:
            self.logger.warning(f"Failed to restart {self.service_name}: {exc}")


class SerialTransport:
    def __init__(self, port: str, baud: int, timeout: float, logger):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.logger = logger
        self.ser = None
        self.io_lock = threading.RLock()
        self.rx_buffer = bytearray()

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.logger.info(f"Opened serial port {self.port} at {self.baud} baud")
            return True
        except serial.SerialException as exc:
            self.logger.error(f"Failed to open serial port {self.port}: {exc}")
            return False

    def close(self):
        with self.io_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def write_json(self, cmd: dict):
        if not self.ser or not self.ser.is_open:
            return
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        with self.io_lock:
            try:
                self.ser.write(line.encode("utf-8"))
                self.ser.flush()
            except serial.SerialException as exc:
                self.logger.warning(f"Serial write error: {exc}")

    def send_cmd(self, cmd: dict) -> dict:
        if not self.ser or not self.ser.is_open:
            return {}
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        with self.io_lock:
            try:
                self.ser.write(line.encode("utf-8"))
                self.ser.flush()
                deadline = time.time() + self.timeout
                while time.time() < deadline:
                    raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if raw.startswith("{"):
                        try:
                            return json.loads(raw)
                        except json.JSONDecodeError:
                            continue
            except serial.SerialException as exc:
                self.logger.warning(f"Serial send error: {exc}")
        return {}

    def read_json_line(self) -> dict:
        if not self.ser or not self.ser.is_open:
            return {}
        with self.io_lock:
            try:
                # 只读已经收到的字节，不在持锁时等下一行或剩余半包。
                available = self.ser.in_waiting
                if available:
                    self.rx_buffer.extend(self.ser.read(available))
                if b'\n' not in self.rx_buffer:
                    return {}
                raw, _, remaining = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(remaining)
            except serial.SerialException:
                return {}
        try:
            data = json.loads(raw)
            return data if isinstance(data, dict) else {}
        except (ValueError, UnicodeError):
            return {}


class WaveRoverDriver(Node):
    def __init__(self):
        super().__init__("waverover_driver")

        self.declare_parameter("serial_port", "/dev/waverover")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("max_linear_vel", 1.0)
        self.declare_parameter("max_angular_vel", 2.0)
        self.declare_parameter("turn_wheel_gain", 0.50)
        self.declare_parameter("turn_wheel_limit", 0.25)
        self.declare_parameter("turn_wheel_min", 0.12)
        self.declare_parameter("cmd_vel_timeout", 0.8)
        self.declare_parameter("feedback_timeout", 1.0)
        self.declare_parameter("heartbeat_rate", 20.0)
        self.declare_parameter("load_calib_on_start", True)
        self.declare_parameter("rezero_imu_on_start", False)
        self.declare_parameter("heartbeat_delay_ms", 3000)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("odom_topic", "odom/raw")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # Explicit protocol units; raw yaw is not fused by the current EKF.
        self.declare_parameter("yaw_units", "radians")
        self.declare_parameter("linear_velocity_stddev", 0.03)
        self.declare_parameter("angular_velocity_stddev", 0.10)
        self.declare_parameter("feedback_rate", 100.0)
        self.declare_parameter("feedback_interval_ms", 10)
        self.declare_parameter("battery_rate", 1.0)
        self.declare_parameter("display_update_rate", 1.0)

        self.declare_parameter("battery_full_voltage", 12.6)
        self.declare_parameter("battery_empty_voltage", 9.0)
        self.declare_parameter("low_battery_percentage", 20.0)
        self.declare_parameter("very_low_battery_percentage", 10.0)

        self.declare_parameter("show_battery_percentage", True)
        self.declare_parameter("display_line_num", 4)
        self.declare_parameter("very_low_flash_rate", 2.0)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.serial_timeout = self.get_parameter("serial_timeout").value
        self.max_linear_vel = self.get_parameter("max_linear_vel").value
        self.max_angular_vel = self.get_parameter("max_angular_vel").value
        self.turn_wheel_gain = self.get_parameter("turn_wheel_gain").value
        self.turn_wheel_limit = self.get_parameter("turn_wheel_limit").value
        self.turn_wheel_min = self.get_parameter("turn_wheel_min").value
        if not (math.isfinite(self.turn_wheel_min) and 0 < self.turn_wheel_min <= self.turn_wheel_limit
                and math.isfinite(self.turn_wheel_gain) and self.turn_wheel_gain > 0
                and math.isfinite(self.turn_wheel_limit) and 0 < self.turn_wheel_limit <= .5):
            raise ValueError("Invalid direct wheel turn gain/limit")
        self.direct_turn_active = False
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.feedback_timeout = self.get_parameter("feedback_timeout").value
        self.heartbeat_rate = self.get_parameter("heartbeat_rate").value
        self.load_calib_on_start = self.get_parameter("load_calib_on_start").value
        self.rezero_imu_on_start = self.get_parameter("rezero_imu_on_start").value
        self.heartbeat_delay_ms = self.get_parameter("heartbeat_delay_ms").value
        self.publish_odom = self.get_parameter("publish_odom").value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.yaw_units = self.get_parameter("yaw_units").value
        if self.yaw_units not in ("radians", "degrees"):
            raise ValueError("yaw_units must be radians or degrees")
        self.linear_variance = float(self.get_parameter("linear_velocity_stddev").value)**2
        self.angular_variance = float(self.get_parameter("angular_velocity_stddev").value)**2
        if self.linear_variance <= 0 or self.angular_variance <= 0:
            raise ValueError("Odometry measurement variance must be positive")
        self.feedback_sample = None
        self.previous_sample = None
        self.feedback_sequence = 0
        self.published_sequence = 0
        self.last_motion_log = 0.
        self.feedback_rate = self.get_parameter("feedback_rate").value
        self.feedback_interval_ms = self.get_parameter("feedback_interval_ms").value
        self.battery_rate = self.get_parameter("battery_rate").value
        self.display_update_rate = self.get_parameter("display_update_rate").value

        self.battery_full_voltage = self.get_parameter("battery_full_voltage").value
        self.battery_empty_voltage = self.get_parameter("battery_empty_voltage").value
        self.low_battery_percentage = self.get_parameter("low_battery_percentage").value
        self.very_low_battery_percentage = self.get_parameter("very_low_battery_percentage").value

        self.show_battery_percentage = self.get_parameter("show_battery_percentage").value
        self.display_line_num = self.get_parameter("display_line_num").value
        self.very_low_flash_rate = self.get_parameter("very_low_flash_rate").value

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_feedback_time = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        self.feedback_lock = threading.Lock()
        self.latest_json_feedback = "{}"
        self.dist_L = 0.0
        self.dist_R = 0.0
        self.dist_centre = 0.0
        self.yaw_odom = 0.0
        self.yaw_imu = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.voltage = 0.0
        self.battery_percentage = 0.0
        self.enc_A = 0
        self.enc_B = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.running = True
        self.display_flash_on = True
        self.display_update_count = 0

        self.display_service = DisplayServiceGuard("waverover-init.service", self.get_logger())
        self.display_service.stop()
        atexit.register(self.display_service.start)
        time.sleep(0.5)

        self.transport = SerialTransport(self.serial_port, self.baud_rate, self.serial_timeout, self.get_logger())
        if not self.transport.open():
            self.display_service.start()
            raise SystemExit(1)

        self._initialize_robot()

        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        self.json_feedback_pub = self.create_publisher(String, "waverover/json_feedback", 10)
        self.raw_feedback_pub = self.create_publisher(Float32MultiArray, "waverover/raw_feedback", 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, "encoders", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self.battery_percentage_pub = self.create_publisher(Float32, "battery_percentage", 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / self.heartbeat_rate, self._heartbeat_callback)
        self.create_timer(1.0 / self.feedback_rate, self._feedback_publish_callback)
        self.create_timer(1.0 / self.battery_rate, self._battery_publish_callback)
        self.create_timer(1.0 / self.display_update_rate, self._battery_display_callback)

        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self._send_display_text("BAT --%", force=True)
        self.get_logger().info("WAVE ROVER driver started")

    def _initialize_robot(self):
        self.get_logger().info("Initializing WAVE ROVER base")
        self.transport.send_cmd({"T": 900, "main": 1, "module": 0})
        time.sleep(0.2)

        if self.load_calib_on_start:
            self.transport.send_cmd({"T": 161})
            time.sleep(0.2)

        if self.rezero_imu_on_start:
            self.get_logger().info("Re-zeroing IMU")
            time.sleep(2.0)
            self.transport.send_cmd({"T": 127})
            time.sleep(0.2)

        self.transport.send_cmd({"T": 151})
        time.sleep(0.2)
        self.transport.send_cmd({"T": 136, "cmd": self.heartbeat_delay_ms})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 142, "cmd": int(self.feedback_interval_ms)})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 131, "cmd": 1})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 143, "cmd": 0})
        time.sleep(0.1)
        self._send_velocity(0.0, 0.0)

    def _cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()
        linear,angular = msg.linear.x,msg.angular.z
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or time.monotonic()-self.last_feedback_time > self.feedback_timeout):
            linear,angular = 0.0,0.0
        linear = max(-self.max_linear_vel,min(self.max_linear_vel,linear))
        angular = max(-self.max_angular_vel,min(self.max_angular_vel,angular))
        self.current_linear = linear
        self.current_angular = angular
        self._send_velocity(linear, angular)

    def _heartbeat_callback(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if age > self.cmd_vel_timeout or time.monotonic()-self.last_feedback_time > self.feedback_timeout:
            if self.current_linear != 0.0 or self.current_angular != 0.0:
                self.current_linear = 0.0
                self.current_angular = 0.0
                self.get_logger().warning("Command or base feedback timeout, stopping robot")
            self._send_velocity(0.0, 0.0)
            return
        self._send_velocity(self.current_linear, self.current_angular)

    def _send_velocity(self, linear: float, angular: float):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear, angular = 0.0, 0.0
        if abs(linear) < 1e-6 and abs(angular) >= 1e-6:
            # 原地转：左右轮反向。L/R 是底盘指令值，不是实测角速度。
            if not self.direct_turn_active:
                self.transport.write_json({"T": 13, "X": 0.0, "Z": 0.0})
            # 避免小指令带不动轮子；左右两侧始终等幅反向。
            magnitude = max(self.turn_wheel_min,
                            min(self.turn_wheel_limit, abs(angular)*self.turn_wheel_gain))
            wheel = math.copysign(magnitude, angular)
            self.transport.write_json({"T": 1, "L": round(-wheel, 4), "R": round(wheel, 4)})
            self.direct_turn_active = True
            return
        # 切回直走或停车时，先清除左右轮指令；超时和退出也走这里。
        if self.direct_turn_active or (linear == 0.0 and angular == 0.0):
            self.transport.write_json({"T": 1, "L": 0.0, "R": 0.0})
        self.direct_turn_active = False
        self.transport.write_json({"T": 13, "X": round(linear, 4), "Z": round(angular, 4)})

    def _serial_reader_loop(self):
        while self.running:
            try:
                data = self.transport.read_json_line()
                if data:
                    self._process_feedback(data)
                else:
                    time.sleep(0.002)
            except Exception as exc:
                self.get_logger().warning(f"Serial read loop error: {exc}")
                time.sleep(0.5)

    def _process_feedback(self, data: dict):
        with self.feedback_lock:
            self.latest_json_feedback = json.dumps(data, separators=(",", ":"))
            t_code = data.get("T", 0)
            if t_code == 1001:
                # A partial/status packet must not refresh the odometry watchdog.
                if not all(k in data and math.isfinite(float(data[k]))
                           for k in ("dist_centre", "yaw_odom")):
                    return
                self.dist_L = float(data.get("dist_L", self.dist_L))
                self.dist_R = float(data.get("dist_R", self.dist_R))
                self.dist_centre = float(data.get("dist_centre", self.dist_centre))
                self.yaw_odom = float(data.get("yaw_odom", self.yaw_odom))
                self.yaw_imu = float(data.get("yaw_imu", self.yaw_imu))
                self.enc_A = int(data.get("encA", self.enc_A))
                self.enc_B = int(data.get("encB", self.enc_B))
                self.voltage = float(data.get("voltage", self.voltage))
                self.battery_percentage = self._estimate_battery_percentage(self.voltage)
                self.last_feedback_time = time.monotonic()
                self.feedback_sequence += 1
                self.feedback_sample = (self.last_feedback_time, self.get_clock().now().to_msg(),
                                        self.dist_centre, self._yaw_radians(self.yaw_odom),
                                        self.dist_L, self.dist_R, self.enc_A, self.enc_B)
            elif t_code == 1002:
                self.yaw_imu = float(data.get("yaw_imu", self.yaw_imu))
                self.imu_roll = float(data.get("r", self.imu_roll))
                self.imu_pitch = float(data.get("p", self.imu_pitch))

    def _estimate_battery_percentage(self, voltage: float) -> float:
        if voltage <= 0.01:
            return 0.0
        span = self.battery_full_voltage - self.battery_empty_voltage
        if span <= 0.01:
            return 0.0
        pct = (voltage - self.battery_empty_voltage) / span * 100.0
        return max(0.0, min(100.0, pct))

    def _feedback_publish_callback(self):
        with self.feedback_lock:
            json_text = self.latest_json_feedback
            raw_values = [
                float(self.dist_L), float(self.dist_R), float(self.dist_centre),
                float(self.yaw_odom), float(self.yaw_imu),
                float(self.imu_roll), float(self.imu_pitch),
                float(self.voltage), float(self.battery_percentage),
            ]
            encoders = [int(self.enc_A), int(self.enc_B)]

        json_msg = String()
        json_msg.data = json_text
        self.json_feedback_pub.publish(json_msg)

        raw_msg = Float32MultiArray()
        raw_msg.data = raw_values
        self.raw_feedback_pub.publish(raw_msg)

        encoder_msg = Int32MultiArray()
        encoder_msg.data = encoders
        self.encoder_pub.publish(encoder_msg)

        if self.publish_odom:
            self._publish_odom()

    def _yaw_radians(self, yaw_value: float) -> float:
        return math.radians(yaw_value) if self.yaw_units == "degrees" else yaw_value

    def _publish_odom(self):
        with self.feedback_lock:
            if self.feedback_sample is None or self.feedback_sequence == self.published_sequence:
                return
            sample = self.feedback_sample
            self.published_sequence = self.feedback_sequence
        received, stamp, distance, yaw, left, right, enc_a, enc_b = sample
        previous = self.previous_sample
        self.previous_sample = sample
        if previous is None or received-previous[0] > self.feedback_timeout:
            return
        dt = received-previous[0]
        if dt <= 0:
            return
        delta = distance-previous[2]
        angle = math.atan2(math.sin(yaw-previous[3]), math.cos(yaw-previous[3]))
        linear_velocity, angular_velocity = delta/dt, angle/dt
        mid_yaw = previous[3]+angle/2
        self.odom_x += delta*math.cos(mid_yaw)
        self.odom_y += delta*math.sin(mid_yaw)

        msg = Odometry()
        msg.header.stamp = stamp  # Receipt of this measurement, not a later timer tick.
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.odom_x, self.odom_y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity
        # Initial uncertainty estimates, not a calibration result. EKF uses vx only.
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 1.0
            msg.twist.covariance[index] = 1.0
        msg.twist.covariance[0] = self.linear_variance
        msg.twist.covariance[35] = self.angular_variance
        self.odom_pub.publish(msg)
        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x, transform.transform.translation.y = self.odom_x, self.odom_y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        # One readable diagnostic line per second; measurement publication is not throttled.
        if received-self.last_motion_log >= 1.0 and (self.current_linear != 0 or delta != 0):
            self.last_motion_log = received
            self.get_logger().info(
                f'底盘方向: cmd={self.current_linear:+.3f} vx={linear_velocity:+.3f} '
                f'dcentre={delta:+.5f} dL={left-previous[4]:+.5f} dR={right-previous[5]:+.5f} '
                f'encA={enc_a-previous[6]:+d} encB={enc_b-previous[7]:+d}')

    def _battery_publish_callback(self):
        with self.feedback_lock:
            voltage = float(self.voltage)
            percent = float(self.battery_percentage)

        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = voltage
        battery_msg.percentage = percent / 100.0
        battery_msg.present = True
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(battery_msg)

        pct_msg = Float32()
        pct_msg.data = percent
        self.battery_percentage_pub.publish(pct_msg)

    def _battery_display_callback(self):
        if not self.show_battery_percentage:
            return

        with self.feedback_lock:
            percent = float(self.battery_percentage)
            voltage = float(self.voltage)

        if voltage <= 0.01:
            self._send_display_text("BAT --%", force=True)
            return

        pct_int = int(round(percent))

        if percent <= self.very_low_battery_percentage:
            self.display_update_count += 1
            self.display_flash_on = not self.display_flash_on
            text = f"CRIT {pct_int}%" if self.display_flash_on else ""
            self._send_display_text(text, force=True)
            return

        if percent <= self.low_battery_percentage:
            self._send_display_text(f"LOW {pct_int}%", force=True)
            return

        self._send_display_text(f"BAT {pct_int}%", force=True)

    def _send_display_text(self, text: str, force: bool = False):
        # Always force for display reliability because other firmware/UI tasks may repaint the line.
        self.transport.write_json({"T": 3, "lineNum": int(self.display_line_num), "Text": str(text)})

    def destroy_node(self):
        self.get_logger().info("Stopping WAVE ROVER")
        try:
            self.running = False
            self._send_velocity(0.0, 0.0)
            time.sleep(0.1)
            self.transport.write_json({"T": 131, "cmd": 0})
            time.sleep(0.1)
            self.transport.close()
        finally:
            self.display_service.start()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaveRoverDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
