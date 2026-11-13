import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """Same MoveIt configuration as positronic_moveit_config/move_group.launch.py."""
    return (
        MoveItConfigsBuilder('positronic', package_name='positronic_moveit_config')
        .robot_description(
            file_path=os.path.join(get_package_share_directory('positronic_description'),
                                    'urdf', 'positronic_gazebo.urdf.xacro')
        )
        .robot_description_semantic(file_path='config/positronic.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = build_moveit_config()

    mission_node = Node(
        package='positronic_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )
    return [mission_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                                description='true when the clock comes from Gazebo'),
        OpaqueFunction(function=launch_setup),
    ])
