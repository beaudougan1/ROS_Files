#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class CmdVelOdom(Node):
    def __init__(self):
        super().__init__("cmd_vel_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("topic_cmd_vel", "/cmd_vel")
        self.declare_parameter("topic_odom", "/odom_raw")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rate_hz", 50.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.topic_cmd_vel = self.get_parameter("topic_cmd_vel").value
        self.topic_odom = self.get_parameter("topic_odom").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.vx = 0.0
        self.wz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = time.time()

        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, self.topic_cmd_vel, self.on_cmd, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f"Publishing {self.topic_odom} and TF {self.odom_frame}->{self.base_frame} from {self.topic_cmd_vel}"
        )

    def on_cmd(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)

    def tick(self):
        now = time.time()
        dt = now - self.last_t
        if dt <= 0.0:
            return
        self.last_t = now

        # integrate pose
        self.yaw += self.wz * dt
        c = math.cos(self.yaw); s = math.sin(self.yaw)
        self.x += (self.vx * c) * dt
        self.y += (self.vx * s) * dt

        stamp = self.get_clock().now().to_msg()
        q = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_br.sendTransform(t)

def main():
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
