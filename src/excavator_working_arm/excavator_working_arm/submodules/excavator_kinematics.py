from math import acos, sqrt, cos, pi
from argparse import ArgumentError

## geometric constants of excavator
geometry_scale = 1 / 0.025
# Boom
l_boom1 = 104.858  # constant
l_boom2 = 49.709  # constant
s_boom_0 = 98.779  # idle length of boom cylinder
# Arm
l_arm1 = 112.156  # constant
l_arm2 = 41.031  # constant
s_arm_0 = 130.89  # idle length of boom cylinder
# Bucket
s_bucket_0 = 120.459  # idle length of boom cylinder
l_bucket1 = 104.039  # constant
l_bucket2 = 36  # constant
l_bucket3 = 30.98  # constant
l_bucket4 = 25.639  # constant
l_bucket5 = 26.344  # constant
xi = 0.4305008189  # constant


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


def piston_to_rotation_joints_boom(joint_boom_actuator_piston):
    s_boom = joint_boom_actuator_piston * geometry_scale + s_boom_0

    # initial rotational angels of idle position
    alpha_boom_0 = triangle_length_2_angle(l_boom2, s_boom_0, l_boom1)
    beta_boom_0 = triangle_length_2_angle(l_boom1, l_boom2, s_boom_0)
    # rotational angles in new position
    joint_boom = triangle_length_2_angle(l_boom1, l_boom2, s_boom) - beta_boom_0
    joint_boom_actuator_cylinder = alpha_boom_0 - triangle_length_2_angle(
        l_boom2, s_boom, l_boom1
    )

    return joint_boom, joint_boom_actuator_cylinder


def piston_to_rotation_joints_arm(joint_arm_actuator_piston):
    s_arm = joint_arm_actuator_piston * geometry_scale + s_arm_0

    # initial rotational angels of idle position
    alpha_arm_0 = triangle_length_2_angle(l_arm1, s_arm_0, l_arm2)
    beta_arm_0 = triangle_length_2_angle(l_arm1, l_arm2, s_arm_0)

    # rotational angles in new position
    joint_arm = beta_arm_0 - triangle_length_2_angle(l_arm1, l_arm2, s_arm)
    joint_arm_actuator_cylinder = (
        triangle_length_2_angle(l_arm1, s_arm, l_arm2) - alpha_arm_0
    )

    return joint_arm, joint_arm_actuator_cylinder


def calc_parallel_kinematics_bucket(epsilon):
    """calculates angles of parallel kinematic problem:

    Args:
        epsilon (float): angle of upper link

    Returns:
        [beta, delta]: angle (joint state) of links
    """

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


def piston_to_rotation_joints_bucket(joint_bucket_actuator_piston):

    s_bucket = joint_bucket_actuator_piston * geometry_scale + s_bucket_0

    # initial rotational angels of idle position
    alpha_bucket_0 = triangle_length_2_angle(l_bucket1, s_bucket_0, l_bucket2)
    beta_bucket_0 = triangle_length_2_angle(l_bucket1, l_bucket2, s_bucket_0)

    # rotational angles in new position
    joint_bucket_link1 = beta_bucket_0 - triangle_length_2_angle(
        l_bucket1, l_bucket2, s_bucket
    )
    joint_bucket_actuator_cylinder = (
        triangle_length_2_angle(l_bucket1, s_bucket, l_bucket2) - alpha_bucket_0
    )

    [beta_0_bucket, delta_0_bucket] = calc_parallel_kinematics_bucket(epsilon=0.0)
    [beta_bucket, delta_bucket] = calc_parallel_kinematics_bucket(
        epsilon=joint_bucket_link1
    )

    joint_bucket_link2 = delta_bucket - delta_0_bucket
    joint_bucket = beta_0_bucket - beta_bucket

    return [
        joint_bucket_actuator_cylinder,
        joint_bucket_link1,
        joint_bucket_link2,
        joint_bucket,
    ]
