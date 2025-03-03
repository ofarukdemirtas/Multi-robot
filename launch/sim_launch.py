from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_suru')
    model_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'worlds')
    
    # Resource paths for Ignition Fortress
    ign_res_path = [
        model_path,
        '/usr/share/ignition/gazebo6',
        os.path.join(model_path, 'x3'),
        os.path.join(model_path, 'X1'),
        os.path.join(model_path, 'ground_plane')
    ]
    
    return LaunchDescription([
        # Environment setup for Ignition Fortress
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join(ign_res_path)
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value='/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins'
        ),

        # Launch Ignition Gazebo with empty world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', os.path.join(world_path, 'empty.sdf')],
            output='screen'
        ),

        # Spawn Ground Plane
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'ground_plane',
                        '-file', os.path.join(model_path, 'ground_plane', 'model.sdf'),
                        '-x', '0', '-y', '0', '-z', '0'
                    ],
                    output='screen'
                )
            ]
        ),

        # Spawn X1 UGV
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'x1',
                        '-file', os.path.join(model_path, 'X1', 'model.sdf'),
                        '-x', '0.0', '-y', '0.0', '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),

        # Spawn X3 UAV
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'X3',
                        '-file', os.path.join(model_path, 'x3', 'model.sdf'),
                        '-x', '0.0', '-y', '0.0', '-z', '3.0'
                    ],
                    output='screen'
                )
            ]
        ),

        # ROS-Ignition Bridge (changed to ros_gz_bridge for Humble)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # UGV Topics
                '/model/x1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/x1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                
                # UAV Topics 
                '/X3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/X3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/X3/command/motor_speed@std_msgs/msg/Float64MultiArray@gz.msgs.Double_V'
            ],
            output='screen'
        ),

        # Control Node
        Node(
            package='my_suru',
            executable='pattern.py',
            name='pattern_control',
            output='screen'
        )
    ])