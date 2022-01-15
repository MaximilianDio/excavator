#!/usr/bin/env python3

# ros client library
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# messages
from sensor_msgs.msg import Joy
from excavator_interfaces.msg import ExcavatorJointVel


class UI2ExcavatorJointVel(Node):
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

        self.excavator_joint_vel_pub_ = self.create_publisher(
            ExcavatorJointVel, "/excavator_joint_vel", qos_profile
        )

        # initialization complete
        self.get_logger().info(self.node_name_ + " has been started!")

    def callback_get_joy(self, msg):
        """map user input (Joystick) to excavator joint velocities

        Args:
            msg (Joy): [description]
        """
        vel_rate = 0.5

        exc_joint_vel_msg = ExcavatorJointVel()
        exc_joint_vel_msg.slew_axis = vel_rate * msg.axes[2]
        exc_joint_vel_msg.boom_actuator_piston = vel_rate * msg.axes[3]
        exc_joint_vel_msg.arm_actuator_piston = -vel_rate * msg.axes[1]
        exc_joint_vel_msg.bucket_actuator_piston = vel_rate * msg.axes[0]

        self.excavator_joint_vel_pub_.publish(exc_joint_vel_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = UI2ExcavatorJointVel()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
