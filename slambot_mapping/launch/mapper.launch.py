import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    slambot_description_dir = get_package_share_directory("slambot_description")

    # Your existing args and nodes
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            slambot_description_dir, "urdf", "slambot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Now include the c1.launch.py from another package (replace 'some_package' & path)
    sllidar_ros2 = get_package_share_directory("sllidar_ros2")
    lidar_launch_path = os.path.join(sllidar_ros2, "launch", "sllidar_c1_launch.py")

    # joystick controller
    controller = get_package_share_directory("slambot_controller") 
    controller_launch_path = os.path.join(controller, "launch", "controller.launch.py")


    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_path)
    )

    # slam-toolbox node 
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "scan_topic": "/scan",               
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "publish_map": True,
            "resolution": 0.05,
        }]
    )


    return LaunchDescription(
        [
            model_arg,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            lidar_launch,
            controller_launch, 
            slam_toolbox_node, 
        ]
    )
