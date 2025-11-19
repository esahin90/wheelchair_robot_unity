#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class UnityGoalBridge(Node):
    def __init__(self):
        super().__init__('unity_goal_bridge')

        self.declare_parameter("goal_topic", "/unity/goal_pose")
        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self.nav = BasicNavigator()
        self.sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )

        self.get_logger().info(
            "UnityGoalBridge ready. Waiting for goals..."
        )

    def goal_callback(self, msg: PoseStamped):
        self.nav.waitUntilNav2Active()
        self.nav.goToPose(msg)

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
        else:
            self.get_logger().warn(f"Navigation result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = UnityGoalBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
