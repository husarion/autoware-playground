#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lgsvl_msgs.msg import VehicleControlData
from geometry_msgs.msg import Twist
import math 
from autoware_auto_vehicle_msgs.msg import GearReport


class PantherAdapter(Node):
    def __init__(self):
        super().__init__("panther_adapter")

        self.sub_bool = self.create_subscription(
            VehicleControlData, "/lgsvl/vehicle_control_cmd", self.callback_controll, 1)

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_gear_report = self.create_publisher(GearReport, "/panther/gear_report", 1)

        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Panther adapter initialized")

        self.prev_vel_lin = 0.0

        self.declare_parameter('acc_reduce_val', 0.1)
        self.declare_parameter('time_step', 24.0)
        self.declare_parameter('speed_limit', 0.5)
        self.declare_parameter('brake_scale', 3.0)
        self.declare_parameter('wheelbase_m', 0.46)

        self.acc_reduce_val = self.get_parameter('acc_reduce_val')
        self.time_step = self.get_parameter('time_step')
        self.speed_limit = self.get_parameter('speed_limit')
        self.brake_scale = self.get_parameter('brake_scale')
        self.wheelbase_m = self.get_parameter('wheelbase_m')

    
    def limit_value(self, val, lower, upper):

        if val > upper:
            return upper
        elif val < lower:
            return  lower
        else:
            return val


    def callback_controll(self, msg: VehicleControlData):
        twist = Twist()

        # Calculations based on pages 18 - 20, angular velocity: (10), in:
        # https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf


        vel = ((msg.acceleration_pct - self.acc_reduce_val) * self.time_step) + self.prev_vel_lin
        vel = self.limit_value(vel, 0.0, self.speed_limit)

        if msg.braking_pct > 0.0:
            vel = vel / self.brake_scale
        
        # self.get_logger().info("Acc = {}, prev_vel = {}, vel = {}".format(msg.acceleration_pct, self.prev_vel_lin, vel))
        self.prev_vel_lin = vel

        vel_ang = vel * math.tan(msg.target_wheel_angle) / self.wheelbase_m
        self.get_logger().info("\nanggle = {}\n angualr vel = {}".format(msg.target_wheel_angle, vel_ang))

        # Velocity x and angular capped to 0.2

        twist.linear.x = vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vel_ang

        self.pub_cmd_vel.publish(twist)


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


if __name__ == "__main__":
    try:
        main()
    except rclpy.ROSInterruptException:
        rclpy.shutdown()
