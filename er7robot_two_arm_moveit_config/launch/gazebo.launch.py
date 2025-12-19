import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler,SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit

import xacro
import re

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'er7robot'
    package_name = 'er7robot_arm_description'
    urdf_name = "test.urdf.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    install_dir = os.path.dirname(pkg_share)
    
    set_model_path_cmd = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=install_dir
    )
    set_rmw_cmd = SetEnvironmentVariable(name = "RMW_IMPLEMENTATION",value="rmw_cyclonesdds_cpp")
    set_localhost_cmd = SetEnvironmentVariable(name = "ROS_LOCALHOST_ONLY",value = "1")
    # 因为 urdf文件中有一句 $(find robot_moveit_config) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}
    
    # Start Gazebo server
    #ExecuteProcess 
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 启动了robot_state_publisher节点
    # 
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'], 
        output='screen'
    )


    # 关节状态广播器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # # 轨迹控制器
    #Equal to "ros2 control load_controller --set-state active my_arm_group_controller" in Terminator
    load_camera_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'camera_arm_group_controller'],
        output='screen'
    )

    load_laser_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'laser_arm_group_controller'],
        output='screen'
    )

    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller？
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_camera_joint_trajectory_controller],
            )
    )
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_camera_joint_trajectory_controller,
                on_exit=[load_laser_joint_trajectory_controller],
            )
    )

    ld = LaunchDescription()
    ld.add_action(set_rmw_cmd)
    ld.add_action(set_localhost_cmd)
    ld.add_action(set_model_path_cmd)
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(close_evt3)

    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)


    return ld
