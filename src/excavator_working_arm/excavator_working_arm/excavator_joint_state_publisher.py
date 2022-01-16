#!/usr/bin/env python3

# ros client library
import enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# external submodules
from math import inf

# local submodules
try:
    from .submodules import excavator_kinematics as ek
    from .submodules.excavator_kinematics import DynJointState
except:
    # if in debug mode
    import submodules.excavator_kinematics as ek
    from submodules.excavator_kinematics import DynJointState


# messages
from sensor_msgs.msg import JointState as JSmsg
from excavator_interfaces.msg import ExcavatorPistonVel, ExcavatorAngleVel

# services


class RedundantJointStates:
    def __init__(self):
        # todo get the values from urdf file
        # NOTE have to be exactly the same as defined in urdf file!
        self.states = {
            "joint_slew_axis": DynJointState(-inf, inf),
            "joint_boom": DynJointState(-inf, inf),
            "joint_boom_actuator_cylinder": DynJointState(-inf, inf),
            "joint_boom_actuator_piston": DynJointState(-0.65, 0.6),
            "joint_arm": DynJointState(-inf, inf),
            "joint_arm_actuator_cylinder": DynJointState(-inf, inf),
            "joint_arm_actuator_piston": DynJointState(-0.85, 0.4),
            "joint_bucket_actuator_piston": DynJointState(-0.85, 0.4),
            "joint_bucket_actuator_cylinder": DynJointState(-inf, inf),
            "joint_bucket_link1": DynJointState(-inf, inf),
            "joint_bucket_link2": DynJointState(-inf, inf),
            "joint_bucket": DynJointState(-inf, inf),
        }


# Main Node
class ExcavatorJointStatePublisher(Node):
    def __init__(self):
        # initialize node
        self.node_name_ = "excavator_joint_state_publisher"
        super().__init__(self.node_name_)

        # quality of service (QoS)
        qos_profile = QoSProfile(depth=10)

        self.red_js_ = RedundantJointStates()

        self.minimal_js_ = {
            "joint_slew_axis": DynJointState(-inf, inf),
            "joint_boom_actuator_piston": DynJointState(-0.65, 0.6),
            "joint_arm_actuator_piston": DynJointState(-0.85, 0.4),
            "joint_bucket_actuator_piston": DynJointState(-0.85, 0.4),
        }

        # subscriber
        self.update_joint_vel_sub_ = self.create_subscription(
            ExcavatorPistonVel,
            "/excavator_piston_vel",
            self.callback_update_join_vel,
            qos_profile,
        )

        # publisher
        self.js_pub_ = self.create_publisher(JSmsg, "/joint_states", qos_profile)

        # timer
        self.timer_rate_ = 1 / 100
        self.timer_ = self.create_timer(self.timer_rate_, self.publish_joint_state)

        # initialization complete
        self.get_logger().info(self.node_name_ + " has been started!")

    def callback_update_join_vel(self, msg: ExcavatorPistonVel):

        self.minimal_js_["joint_slew_axis"].dx_ = msg.slew_angle
        self.minimal_js_["joint_boom_actuator_piston"].dx_ = msg.boom_actuator_piston
        self.minimal_js_["joint_arm_actuator_piston"].dx_ = msg.arm_actuator_piston
        self.minimal_js_[
            "joint_bucket_actuator_piston"
        ].dx_ = msg.bucket_actuator_piston

    def publish_joint_state(self):

        # publish joint_state
        js = JSmsg()

        now = self.get_clock().now()
        js.header.stamp = now.to_msg()
        js.name = list(self.red_js_.states.keys())
        js.position = []

        # integrate state and clip
        for joint_name, state in self.minimal_js_.items():
            # update state
            self.minimal_js_[joint_name].integrate(self.timer_rate_)
            clipped = self.minimal_js_[joint_name].clip()

            if clipped:
                info_msg = "maximum joint state for " + joint_name + " was reached!"
                # self.get_logger().info(info_msg)

        # calc redundant states
        self.calc_piston_to_joint_states()

        # append values
        for state in self.red_js_.states.values():
            js.position.append(state.x_)

        # publish joint state message
        self.js_pub_.publish(js)

    def calc_piston_to_joint_states(self):
        """calculates remaining working arm joint states based on piston positions

        Raises:
            ValueError: if singularity is reached
        """

        try:
            ## piston positions to other joint positions
            # BOOM forward kinematics

            self.red_js_.states["joint_boom_actuator_piston"] = self.minimal_js_[
                "joint_boom_actuator_piston"
            ]
            [
                self.red_js_.states["joint_boom"].x_,
                self.red_js_.states["joint_boom_actuator_cylinder"].x_,
            ] = ek.piston_to_rotation_joints_boom(
                self.minimal_js_["joint_boom_actuator_piston"].x_
            )
            # ARM forward kinematics
            self.red_js_.states["joint_arm_actuator_piston"] = self.minimal_js_[
                "joint_arm_actuator_piston"
            ]
            [
                self.red_js_.states["joint_arm"].x_,
                self.red_js_.states["joint_arm_actuator_cylinder"].x_,
            ] = ek.piston_to_rotation_joints_arm(
                self.minimal_js_["joint_arm_actuator_piston"].x_
            )
            # BUCKET forward kinematics
            self.red_js_.states["joint_bucket_actuator_piston"] = self.minimal_js_[
                "joint_bucket_actuator_piston"
            ]
            [
                self.red_js_.states["joint_bucket_actuator_cylinder"].x_,
                self.red_js_.states["joint_bucket_link1"].x_,
                self.red_js_.states["joint_bucket_link2"].x_,
                self.red_js_.states["joint_bucket"].x_,
            ] = ek.piston_to_rotation_joints_bucket(
                self.minimal_js_["joint_bucket_actuator_piston"].x_
            )

        except ValueError as err:
            err_msg = "Singularity reached when calculating joint states!"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ExcavatorJointStatePublisher()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
