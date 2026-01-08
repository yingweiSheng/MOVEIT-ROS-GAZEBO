import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import  OnProcessStart
from launch.actions import TimerAction
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("er7robot", package_name="er7robot_two_arm_moveit_config").to_moveit_configs()

    # Get parameters for the Servo node
    camera_servo_yaml = load_yaml("realtime_servo_er7robot","config/camera_simulated_config.yaml")
    camera_servo_params = {"camera_servo_config":camera_servo_yaml}
    laser_servo_yaml = load_yaml("realtime_servo_er7robot","config/laser_simulated_config.yaml")
    laser_servo_params = {"laser_servo_config":laser_servo_yaml}

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    camera_servo_node = TimerAction(
        period = 4.0,
        actions=[
            Node(
                package="realtime_servo_er7robot",
                executable="camera_servo_demo",
                namespace="camera_servo_demo",
                output="screen",
                parameters=[
                    camera_servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            )
        ]
    )

    laser_servo_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="realtime_servo_er7robot",
                executable="laser_servo_demo",
                namespace="laser_servo_demo",
                output="screen",
                parameters=[
                    laser_servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
    )

        ]
    )
    # Need to set delay
    dual_commander_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="realtime_servo_er7robot",
                executable="dual_arm_servo_commander",
                namespace="dual_commander_node",
                output="screen",
                parameters=
                [
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                ],
            )
        ]
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("realtime_servo_er7robot")
        + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("er7robot_two_arm_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="both",
    )

    load_controllers = [
        ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner",
                controller,
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        )
        for controller in [
            "joint_state_broadcaster",
            "camera_arm_group_controller",
            "laser_arm_group_controller",
        ]
    ]

    controller_start = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=load_controllers,
        )
    )


    return LaunchDescription(
        [rviz_node, static_tf, camera_servo_node,laser_servo_node,dual_commander_node,
         ros2_control_node,robot_state_publisher,controller_start]
    )
