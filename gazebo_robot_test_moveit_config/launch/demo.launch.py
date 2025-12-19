from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("six_arm", package_name="gazebo_robot_test_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
