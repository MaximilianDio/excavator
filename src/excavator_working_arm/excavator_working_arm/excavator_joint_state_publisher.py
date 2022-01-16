#!/usr/bin/env python3

# ros client library
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# external submodules
from math import inf

# local submodules
try:
    from .submodules.excavator_kinematics import (
        piston_to_rotation_joints_arm,
        piston_to_rotation_joints_boom,
        piston_to_rotation_joints_bucket,
    )
except:
    # if in debug mode
    from submodules.excavator_kinematics import (
        piston_to_rotation_joints_arm,
        piston_to_rotation_joints_boom,
        piston_to_rotation_joints_bucket,
    )


# messages
from sensor_msgs.msg import JointState
from excavator_interfaces.msg import ExcavatorJointVel

# services


def clip_joint_state(joint_state, min_joint_state, max_joint_state):
    """clips joint state if limit exceeded

    Args:
        joint_state (float):
        min_joint_state (float):
        max_joint_state (float):

    Returns:
        float, bool: new_joint_state, is_clipped
    """
    if joint_state < min_joint_state:
        return min_joint_state, True
    elif joint_state > max_joint_state:
        return max_joint_state, True
    else:
        return joint_state, False


# Main Node
class ExcavatorJointStatePublisher(Node):
    def __init__(self):
        # initialize node
        self.node_name_ = "excavator_joint_state_publisher"
        super().__init__(self.node_name_)

        # quality of service (QoS)
        qos_profile = QoSProfile(depth=10)

        # todo get the values from urdf file
        self.min_minimal_joint_value = {
            "joint_slew_axis": -inf,
            "joint_boom_actuator_piston": -0.65,
            "joint_arm_actuator_piston": -0.85,
            "joint_bucket_actuator_piston": -0.85,
        }
        self.max_minimal_joint_value = {
            "joint_slew_axis": inf,
            "joint_boom_actuator_piston": 0.6,
            "joint_arm_actuator_piston": 0.4,
            "joint_bucket_actuator_piston": 0.4,
        }
        # initial state
        self.minimal_joint_states = {
            "joint_slew_axis": 0.0,
            "joint_boom_actuator_piston": 0.0,
            "joint_arm_actuator_piston": -0.0,
            "joint_bucket_actuator_piston": 0.0,
        }
        self.minimal_joint_vel = {
            "joint_slew_axis": 0.0,
            "joint_boom_actuator_piston": 0.00,
            "joint_arm_actuator_piston": 0.00,
            "joint_bucket_actuator_piston": 0.0,
        }

        # subscriber
        self.update_joint_vel_sub_ = self.create_subscription(
            ExcavatorJointVel,
            "/excavator_joint_vel",
            self.callback_update_join_vel,
            qos_profile,
        )

        # publisher
        self.joint_state_pub_ = self.create_publisher(
            JointState, "/joint_states", qos_profile
        )

        # timer
        self.timer_rate_ = 1 / 100
        self.timer_ = self.create_timer(self.timer_rate_, self.publish_joint_state)

        # initial publish
        self.publish_joint_state()

        # initialization complete
        self.get_logger().info(self.node_name_ + " has been started!")

    def callback_update_join_vel(self, msg):

        self.minimal_joint_vel["joint_slew_axis"] = msg.slew_axis
        self.minimal_joint_vel["joint_boom_actuator_piston"] = msg.boom_actuator_piston
        self.minimal_joint_vel["joint_arm_actuator_piston"] = msg.arm_actuator_piston
        self.minimal_joint_vel[
            "joint_bucket_actuator_piston"
        ] = msg.bucket_actuator_piston

    def publish_joint_state(self):

        for joint, joint_state in self.minimal_joint_states.items():
            # update state
            joint_state = joint_state + self.minimal_joint_vel[joint] * self.timer_rate_
            # limit joint state
            [self.minimal_joint_states[joint], clipped] = clip_joint_state(
                joint_state,
                self.min_minimal_joint_value[joint],
                self.max_minimal_joint_value[joint],
            )
            if clipped:
                # self.get_logger().warn(
                #     "maximum joint state for "
                #     + joint
                #     + " was reached - clipping piston state!"
                # )
                pass

        # calc redundant states
        redundant_joint_states = self.calc_joint_states()

        # publish joint_state
        joint_state = JointState()

        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = list(redundant_joint_states.keys())
        joint_state.position = list(redundant_joint_states.values())

        self.joint_state_pub_.publish(joint_state)

    def calc_joint_states(self):
        geometry_scale = 1 / 0.025

        # NOTE have to be exactly the same as defined in urdf file!
        redundant_joint_states = {
            "joint_slew_axis": self.minimal_joint_states["joint_slew_axis"],
            "joint_boom": 0.0,
            "joint_boom_actuator_cylinder": 0.0,
            "joint_boom_actuator_piston": self.minimal_joint_states[
                "joint_boom_actuator_piston"
            ],
            "joint_arm": 0.0,
            "joint_arm_actuator_cylinder": 0.0,
            "joint_arm_actuator_piston": self.minimal_joint_states[
                "joint_arm_actuator_piston"
            ],
            "joint_bucket_actuator_piston": self.minimal_joint_states[
                "joint_bucket_actuator_piston"
            ],
            "joint_bucket_actuator_cylinder": 0.0,
            "joint_bucket_link1": 0.0,
            "joint_bucket_link2": 0.0,
            "joint_bucket": 0.0,
        }

        try:
            ## BOOM forward kinematics
            [
                redundant_joint_states["joint_boom"],
                redundant_joint_states["joint_boom_actuator_cylinder"],
            ] = piston_to_rotation_joints_boom(
                redundant_joint_states["joint_boom_actuator_piston"]
            )

            ## ARM forward kinematics
            [
                redundant_joint_states["joint_arm"],
                redundant_joint_states["joint_arm_actuator_cylinder"],
            ] = piston_to_rotation_joints_arm(
                redundant_joint_states["joint_arm_actuator_piston"]
            )

            ## BUCKET forward kinematics
            [
                redundant_joint_states["joint_bucket_actuator_cylinder"],
                redundant_joint_states["joint_bucket_link1"],
                redundant_joint_states["joint_bucket_link2"],
                redundant_joint_states["joint_bucket"],
            ] = piston_to_rotation_joints_bucket(
                redundant_joint_states["joint_bucket_actuator_piston"]
            )

        except ValueError as err:
            self.get_logger().error(
                "Singularity reached when calculating joint states!"
            )
            raise ValueError("Singularity reached when calculating joint states!")

        return redundant_joint_states


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
