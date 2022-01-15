#!/usr/bin/env python3

# ros client library
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# external submodules
from math import acos, sqrt, cos, pi, inf
from argparse import ArgumentError

# local submodules
try:
    pass
except:
    # if in debug mode
    pass

# messages
from sensor_msgs.msg import JointState
from excavator_interfaces.msg import ExcavatorJointVel

# services


def triangle_length_2_angle(a, b, c):
    """calculates the angle opposite of length c provided all lengths of triangle based on cosine rule

    Args:
        a (float): 1. length of triangle
        b (float): 2. length of triangle
        c (float): length of triangle opposite desired angle

    Raises:
        ArgumentError: if lengths are not positive

    Returns:
        [float]: angle opposite of length c
    """
    if a * b > 0:
        gamma = acos((pow(c, 2) - pow(a, 2) - pow(b, 2)) / (-2 * a * b))
    else:
        raise ArgumentError("Lengths of triangle must be strictly positive!")

    return gamma


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


def calc_parallel_kinematics(epsilon):
    """calculates angles of parallel kinematic problem:

    Args:
        epsilon (float): angle of upper link

    Returns:
        [beta, delta]: angle (joint state) of links
    """

    l_bucket2 = 36  # constant
    l_bucket3 = 30.98  # constant
    l_bucket4 = 25.639  # constant
    l_bucket5 = 26.344  # constant
    xi = 0.4305008189  # constant

    alpha = pi / 2 - xi + epsilon  # inner angle

    s_tmp = sqrt(
        pow(l_bucket2, 2) + pow(l_bucket4, 2) - 2 * l_bucket2 * l_bucket4 * cos(alpha)
    )

    beta1 = triangle_length_2_angle(l_bucket4, s_tmp, l_bucket2)
    beta2 = triangle_length_2_angle(l_bucket5, s_tmp, l_bucket3)

    delta1 = triangle_length_2_angle(l_bucket2, s_tmp, l_bucket4)
    delta2 = triangle_length_2_angle(l_bucket3, s_tmp, l_bucket5)

    beta = beta1 + beta2
    delta = delta1 + delta2

    return beta, delta


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

            l_boom1 = 104.858  # constant
            l_boom2 = 49.709  # constant
            s_boom_0 = 98.779  # idle length of boom cylinder

            # initial rotational angels of idle position
            alpha_boom_0 = triangle_length_2_angle(l_boom2, s_boom_0, l_boom1)
            beta_boom_0 = triangle_length_2_angle(l_boom1, l_boom2, s_boom_0)

            # new piston position
            s_boom = (
                redundant_joint_states["joint_boom_actuator_piston"] * geometry_scale
                + s_boom_0
            )
            # rotational angles in new position
            redundant_joint_states["joint_boom"] = (
                triangle_length_2_angle(l_boom1, l_boom2, s_boom) - beta_boom_0
            )
            redundant_joint_states[
                "joint_boom_actuator_cylinder"
            ] = alpha_boom_0 - triangle_length_2_angle(l_boom2, s_boom, l_boom1)

            ## ARM forward kinematics

            l_arm1 = 112.156  # constant
            l_arm2 = 41.031  # constant
            s_arm_0 = 130.89  # idle length of boom cylinder

            # initial rotational angels of idle position
            alpha_arm_0 = triangle_length_2_angle(l_arm1, s_arm_0, l_arm2)
            beta_arm_0 = triangle_length_2_angle(l_arm1, l_arm2, s_arm_0)

            # new piston position

            s_arm = (
                redundant_joint_states["joint_arm_actuator_piston"] * geometry_scale
                + s_arm_0
            )
            # rotational angles in new position
            redundant_joint_states["joint_arm"] = beta_arm_0 - triangle_length_2_angle(
                l_arm1, l_arm2, s_arm
            )
            redundant_joint_states["joint_arm_actuator_cylinder"] = (
                triangle_length_2_angle(l_arm1, s_arm, l_arm2) - alpha_arm_0
            )

            ## BUCKET forward kinematics

            l_bucket1 = 104.039  # constant
            l_bucket2 = 36  # constant
            l_bucket3 = 30.98  # constant
            l_bucket4 = 25.639  # constant
            l_bucket5 = 26.344  # constant
            xi = 0.4305008189  # constant

            s_bucket_0 = 120.459  # idle length of boom cylinder

            # initial rotational angels of idle position
            alpha_bucket_0 = triangle_length_2_angle(l_bucket1, s_bucket_0, l_bucket2)
            beta_bucket_0 = triangle_length_2_angle(l_bucket1, l_bucket2, s_bucket_0)

            # new piston position
            s_bucket = (
                redundant_joint_states["joint_bucket_actuator_piston"] * geometry_scale
                + s_bucket_0
            )
            # rotational angles in new position
            redundant_joint_states[
                "joint_bucket_link1"
            ] = beta_bucket_0 - triangle_length_2_angle(l_bucket1, l_bucket2, s_bucket)
            redundant_joint_states["joint_bucket_actuator_cylinder"] = (
                triangle_length_2_angle(l_bucket1, s_bucket, l_bucket2) - alpha_bucket_0
            )

            # parallel kinematics
            epsilon = redundant_joint_states["joint_bucket_link1"]

            [beta_0, delta_0] = calc_parallel_kinematics(epsilon=0.0)
            [beta, delta] = calc_parallel_kinematics(epsilon=epsilon)

            redundant_joint_states["joint_bucket_link2"] = delta - delta_0

            redundant_joint_states["joint_bucket"] = beta_0 - beta

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
