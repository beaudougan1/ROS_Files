#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from adafruit_extended_bus import ExtendedI2C
import adafruit_bno055


def quat_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class BNO055Imu(Node):
    def __init__(self):
        super().__init__("bno055_imu")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x28)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)

        bus = int(self.get_parameter("i2c_bus").value)
        addr = int(self.get_parameter("i2c_address").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        rate = float(self.get_parameter("rate_hz").value)

        i2c = ExtendedI2C(bus)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=addr)

        # Prefer fused orientation mode
        try:
            self.sensor.mode = adafruit_bno055.NDOF_MODE
        except Exception as e:
            self.get_logger().warn(f"Could not set NDOF mode: {e}")

        self.pub = self.create_publisher(Imu, "imu", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Publishing /imu at {rate:.1f} Hz (bus={bus}, addr=0x{addr:02X}, frame={self.frame_id})")

    def tick(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation from Euler (heading, roll, pitch) degrees
        e = self.sensor.euler
        if e is not None and None not in e:
            heading, roll_deg, pitch_deg = e
            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(heading)
            msg.orientation = quat_from_euler(roll, pitch, yaw)
            msg.orientation_covariance[0] = 0.02
            msg.orientation_covariance[4] = 0.02
            msg.orientation_covariance[8] = 0.05
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0

        # Gyro rad/s
        g = self.sensor.gyro
        if g is not None and None not in g:
            msg.angular_velocity.x = float(g[0])
            msg.angular_velocity.y = float(g[1])
            msg.angular_velocity.z = float(g[2])
            msg.angular_velocity_covariance[0] = 0.02
            msg.angular_velocity_covariance[4] = 0.02
            msg.angular_velocity_covariance[8] = 0.02
        else:
            msg.angular_velocity_covariance[0] = -1.0

        # Linear acceleration m/s^2 (gravity removed)
        a = self.sensor.linear_acceleration
        if a is not None and None not in a:
            msg.linear_acceleration.x = float(a[0])
            msg.linear_acceleration.y = float(a[1])
            msg.linear_acceleration.z = float(a[2])
            msg.linear_acceleration_covariance[0] = 0.1
            msg.linear_acceleration_covariance[4] = 0.1
            msg.linear_acceleration_covariance[8] = 0.1
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BNO055Imu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
