from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    run_task = LaunchConfiguration("run_task")
    project_root = LaunchConfiguration("project_root")
    moveit_share = get_package_share_directory("mecharm_moveit_config")
    task_share = get_package_share_directory("mecharm_moveit_demo")
    description_share = get_package_share_directory("mycobot_description")

    description_file = str(
        Path(description_share) / "urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
    )
    moveit_config = (
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
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("run_task", default_value="false"),
        DeclareLaunchArgument("project_root", default_value="/mnt/e/机器人集成小组项目/实验二"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(moveit_share) / "launch/move_group.launch.py")),
            launch_arguments={"use_rviz": use_rviz}.items(),
        ),
        Node(
            package="mecharm_pick_place",
            executable="isaac_trajectory_controller",
            name="isaac_trajectory_controller",
            output="screen",
            parameters=[[project_root, "/config/simulation.yaml"]],
        ),
        Node(
            package="mecharm_moveit_demo",
            executable="fixed_pick_place_moveit",
            name="fixed_pick_place",
            output="screen",
            condition=IfCondition(run_task),
            parameters=[
                moveit_config.to_dict(),
                str(Path(task_share) / "config/fixed_pick_place.yaml"),
            ],
        ),
    ])
