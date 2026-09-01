from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_share = get_package_share_directory("mecharm_moveit_config")
    description_file = str(
        Path(get_package_share_directory("mycobot_description"))
        / "urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
    )
    rviz_config = str(Path(moveit_share) / "config/moveit.rviz")
    config = (
        MoveItConfigsBuilder("mecharm_270_pi", package_name="mecharm_moveit_config")
        .robot_description(file_path=description_file)
        .robot_description_semantic(file_path="config/mecharm_270_pi.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    params = config.to_dict()
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", "base",
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[config.robot_description],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[config.robot_description,
                        config.robot_description_semantic,
                        config.robot_description_kinematics],
        ),
    ])