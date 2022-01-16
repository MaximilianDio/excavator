#!/usr/bin/env python3

# ros client library
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# messages
from sensor_msgs.msg import Joy
from excavator_interfaces.msg import ExcavatorPistonVel, ExcavatorAngleVel


class UIType(Enum):
    PISTON = (0,)
    ANGLE = 1


class UI2ExcavatorPistonVel(Node):
    def __init__(self):
        # initialize node
        self.node_name_ = "excavator_ui_to_excavator_joint_vel"
        super().__init__(self.node_name_)

        # quality of service (QoS)
        qos_profile = QoSProfile(depth=10)

        # subscriber
        self.user_input_sub_ = self.create_subscription(
            Joy, "/joy", self.callback_get_joy, qos_profile
        )

        # TODO get ui_type from parameter
        self.ui_type_ = UIType.PISTON

        if self.ui_type_ == UIType.PISTON:
            self.excavator_joint_vel_pub_ = self.create_publisher(
                ExcavatorPistonVel, "/excavator_piston_vel", qos_profile
            )
        elif self.ui_type_ == UIType.ANGLE:
            self.excavator_joint_vel_pub_ = self.create_publisher(
                ExcavatorAngleVel, "/excavator_angle_vel", qos_profile
            )

        # initialization complete
        self.get_logger().info(self.node_name_ + " has been started!")

    def callback_get_joy(self, msg):
        """map user input (Joystick) to excavator joint velocities

        Args:
            msg (Joy): [description]
        """
        vel_rate = 0.5

        if self.ui_type_ == UIType.PISTON:
            exc_joint_vel_msg = ExcavatorPistonVel()
            exc_joint_vel_msg.slew_angle = vel_rate * msg.axes[2]
            exc_joint_vel_msg.boom_actuator_piston = vel_rate * msg.axes[3]
            exc_joint_vel_msg.arm_actuator_piston = -vel_rate * msg.axes[1]
            exc_joint_vel_msg.bucket_actuator_piston = vel_rate * msg.axes[0]

        elif self.ui_type_ == UIType.ANGLE:
            exc_joint_vel_msg = ExcavatorAngleVel()
            exc_joint_vel_msg.slew_angle = vel_rate * msg.axes[2]
            exc_joint_vel_msg.boom_angle = vel_rate * msg.axes[3]
            exc_joint_vel_msg.arm_angle = -vel_rate * msg.axes[1]
            exc_joint_vel_msg.bucket_angle = vel_rate * msg.axes[0]

        self.excavator_joint_vel_pub_.publish(exc_joint_vel_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = UI2ExcavatorPistonVel()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
