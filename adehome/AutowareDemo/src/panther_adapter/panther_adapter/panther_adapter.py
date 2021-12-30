#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lgsvl_msgs.msg import VehicleControlData
from geometry_msgs.msg import Twist
from autoware_auto_vehicle_msgs.msg import GearReport


class PantherAdapter(Node):
    def __init__(self):
        super().__init__("panther_adapter")

        self.sub_bool = self.create_subscription(
            VehicleControlData, "/lgsvl/vehicle_control_cmd", self.callback_controll, 1)

        self.pub_boxes = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_gear_report = self.create_publisher(GearReport, "/panther/gear_report", 1)

        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Panther adapter initialized")

    def callback_controll(self, msg: VehicleControlData):
        twist = Twist()

        # Velocity x and angular capped to 0.2

        twist.linear.x = min(msg.acceleration_pct, 0.2)
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = min(msg.target_wheel_angle, 0.2)

        # self.pub_boxes.publish(twist)
        self.get_logger().info("will be publishing lin.x = {}, ang.z = {}".format(twist.linear.x, twist.angular.z))

    def timer_callback(self):
        greport = GearReport()

        greport.stamp.sec = 0
        greport.stamp.nanosec = 0
        greport.report = 2

        self.pub_gear_report.publish(greport)


def main(args=None):
    rclpy.init(args=args)
    node = PantherAdapter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
subs to:
    - /lgsvl/vehicle_control_cmd
pub to
    - /cmd_vel
"""