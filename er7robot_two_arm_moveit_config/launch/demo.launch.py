from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("er7robot", package_name="er7robot_two_arm_moveit_config").to_moveit_configs()
    params = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
    ]    
    ld = generate_demo_launch(moveit_config)  #自动生成启动的节点的launch描述，并且包含启动顺序
  
    return ld
    # return generate_demo_launch(moveit_config)