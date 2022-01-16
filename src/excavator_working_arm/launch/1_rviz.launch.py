import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # rviz
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen")

    ld.add_action(rviz_node)

    # excavator
    package_dir = get_package_share_directory("excavator_working_arm")
    urdf = os.path.join(package_dir, "excavator_working_arm.urdf")

    # extract robot description from uniform robot descripton file (urdf) file
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    # joint publisher
    joint_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="excavator",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
        arguments=[urdf],
    )
    ld.add_action(joint_publisher_node)

    # joystick node
    joint_publisher_node = Node(
        package="joy",
        executable="joy_node",
        name="user_input",
    )
    ld.add_action(joint_publisher_node)

    # map ui to joint_vel
    map_ui_to_joint_vel_node = Node(
        package="excavator_working_arm",
        executable="excavator_joint_ui",
        name="excavator_map_ui",
    )

    ld.add_action(map_ui_to_joint_vel_node)

    # joint state publisher (kinematic loop relations)
    joint_state_publisher_node = Node(
        package="excavator_working_arm",
        executable="excavator_joint_state_publisher",
        name="excavator_state_publisher",
    )

    ld.add_action(joint_state_publisher_node)

    # GUI to alter joint states
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="JSP_gui",
    #     output="screen",
    #     parameters=[{"robot_description": robot_desc}],
    #     arguments=[urdf],
    # )

    # ld.add_action(joint_state_publisher_gui_node)

    return ld
