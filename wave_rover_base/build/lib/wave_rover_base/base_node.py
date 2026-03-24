import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial


def quat_from_yaw(yaw: float):
    half = yaw * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    msg = Imu().orientation
    msg.x = 0.0
    msg.y = 0.0
    msg.z = qz
    msg.w = qw
    return msg


class WaveRoverBase(Node):
    def __init__(self):
        super().__init__("wave_rover_base")

        # Serial
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)

        # cmd_vel mapping
        self.declare_parameter("wheel_base", 0.20)      # meters
        self.declare_parameter("max_wheel_cmd", 0.5)   # normalized max abs(L/R)
        self.declare_parameter("cmd_timeout", 0.5)      # seconds

        # Deadband / minimum usable command tuning
        self.declare_parameter("min_linear_cmd", 0.12)
        self.declare_parameter("min_angular_cmd", 0.35)
        self.declare_parameter("cmd_zero_epsilon", 0.01)

        # Rover protocol
        self.declare_parameter("telemetry_request_hz", 10.0)
        self.declare_parameter("use_rover_yaw_imu", True)
        self.declare_parameter("imu_frame_id", "imu_link")

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.max_wheel_cmd = float(self.get_parameter("max_wheel_cmd").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.min_linear_cmd = float(self.get_parameter("min_linear_cmd").value)
        self.min_angular_cmd = float(self.get_parameter("min_angular_cmd").value)
        self.cmd_zero_epsilon = float(self.get_parameter("cmd_zero_epsilon").value)

        self.telemetry_hz = float(self.get_parameter("telemetry_request_hz").value)
        self.use_rover_imu = bool(self.get_parameter("use_rover_yaw_imu").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)

        self.last_cmd_time = 0.0
        self.last_l = 0.0
        self.last_r = 0.0

        # Open serial (prevent ESP32 reset behavior)
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05, dsrdtr=None)
        try:
            self.ser.setRTS(False)
            self.ser.setDTR(False)
        except Exception:
            pass
        time.sleep(0.5)

        self.get_logger().info(f"Connected to Wave Rover on {self.port} @ {self.baud}")

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.imu_pub = None
        if self.use_rover_imu:
            self.imu_pub = self.create_publisher(Imu, "imu", 10)

        # Reader thread for telemetry + responses
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Telemetry request timer
        self.tele_timer = self.create_timer(1.0 / max(self.telemetry_hz, 1.0), self.request_telemetry)

        # Safety stop timer
        self.stop_timer = self.create_timer(0.1, self.safety_stop)

    def write_json(self, obj: dict):
        line = (json.dumps(obj) + "\n").encode()
        self.ser.write(line)

    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        min_linear = self.min_linear_cmd
        min_angular = self.min_angular_cmd
        zero_eps = self.cmd_zero_epsilon

        # Ignore tiny noise near zero
        if abs(v) < zero_eps:
            v = 0.0
        elif 0.0 < v < min_linear:
            v = min_linear
        elif -min_linear < v < 0.0:
            v = -min_linear

        if abs(w) < zero_eps:
            w = 0.0
        elif 0.0 < w < min_angular:
            w = min_angular
        elif -min_angular < w < 0.0:
            w = -min_angular

        # Ensure pure rotation gets enough power
        if v == 0.0 and w != 0.0:
            if abs(w) < self.min_angular_cmd:
                w = math.copysign(self.min_angular_cmd, w)

        # Differential drive mapping -> normalized wheel command
        turn_gain = 0.3  # tuned for your rover

        l = v + (turn_gain * w)
        r = v - (turn_gain * w)

        # Clamp to max_wheel_cmd
        l = max(-self.max_wheel_cmd, min(self.max_wheel_cmd, l))
        r = max(-self.max_wheel_cmd, min(self.max_wheel_cmd, r))

        self.last_l, self.last_r = l, r
        self.last_cmd_time = time.time()

        # T=1 speed control
        self.get_logger().info(f"SEND L={l:.2f}, R={r:.2f}")
        self.write_json({"T": 1, "L": l, "R": r})

    def safety_stop(self):
        if self.last_cmd_time == 0.0:
            return
        if (time.time() - self.last_cmd_time) > self.cmd_timeout:
            if abs(self.last_l) > 1e-6 or abs(self.last_r) > 1e-6:
                self.last_l = 0.0
                self.last_r = 0.0
                self.write_json({"T": 1, "L": 0.0, "R": 0.0})

    def request_telemetry(self):
        self.write_json({"T": 130})

    def _rx_loop(self):
        buf = b""
        while not self._stop:
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            s = line.decode(errors="ignore")
                            obj = json.loads(s)
                            self.handle_rx(obj)
                        except Exception:
                            pass
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def handle_rx(self, obj: dict):
        if not isinstance(obj, dict):
            return
        if obj.get("T") == 1001 and self.imu_pub is not None:
            y_deg = obj.get("y", None)
            if y_deg is None:
                return
            yaw = math.radians(float(y_deg))
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.imu_frame_id
            msg.orientation = quat_from_yaw(yaw)
            msg.orientation_covariance = [
                1e6, 0.0, 0.0,
                0.0, 1e6, 0.0,
                0.0, 0.0, 0.05,
            ]
            self.imu_pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WaveRoverBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
