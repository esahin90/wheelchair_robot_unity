#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PoseToOdomTF(Node):
    def __init__(self):
        super().__init__("pose_to_odom_tf")

        self.declare_parameter("pose_topic", "/unity_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_pose = None
        self.last_time = None

        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(
            f"Listening to {pose_topic}, publishing {odom_topic} and TF {self.odom_frame} -> {self.base_frame}"
        )

    def pose_callback(self, msg: PoseStamped):
        # Build Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = msg.pose

        # Simple linear velocity estimate
        if self.last_pose is not None and self.last_time is not None:
            dt = (msg.header.stamp.sec - self.last_time.sec) + \
                 (msg.header.stamp.nanosec - self.last_time.nanosec) * 1e-9
            if dt > 1e-4:
                dx = msg.pose.position.x - self.last_pose.position.x
                dy = msg.pose.position.y - self.last_pose.position.y
                odom.twist.twist.linear.x = dx / dt
                odom.twist.twist.linear.y = dy / dt

        self.last_pose = msg.pose
        self.last_time = msg.header.stamp

        self.odom_pub.publish(odom)

        # Broadcast TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
