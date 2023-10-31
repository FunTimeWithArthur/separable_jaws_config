import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "using_left_arm",
            default_value="true",
            description="Arguement to determine wether to use left arm.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_right_arm",
            default_value="false",
            description="Arguement to determine wether to use right arm.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_fake_hardware",
            default_value="true",
            description="Arguement to determine wether to use fake hardware."
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_db",
            default_value="false",
            description="Arguement to determine wether to use database."
        )
    )

    # initialize arguments
    # it dosen't work though, can't directly covert LaunchConfiguration into str,
    # one possible way: https://answers.ros.org/question/382028/ros2-string-repr-of-parameter-within-launch-file/
    using_left_arm = LaunchConfiguration("using_left_arm")
    using_right_arm = LaunchConfiguration("using_right_arm")
    using_fake_hardware = LaunchConfiguration("using_fake_hardware")
    using_db = LaunchConfiguration("using_db")

    # variables
    # using this stupid way since os.path.join does not support FindPackageShare
    robot_description_package = get_package_share_directory("separable_jaws_description")
    robot_bringup_package = get_package_share_directory("separable_jaws_bringup")
    robot_config_package = get_package_share_directory("separable_jaws_config")
    # using this stupid way since MoveItConfigsBuilder does not support PathJoinSubstitution
    robot_description_file = os.path.join(
        robot_description_package,
        "urdf",
        "separable_jaws.xacro",
    )
    robot_description_semantic = os.path.join(
        robot_config_package,
        "config",
        "separable_jaws.srdf",
    )
    moveit_controllers_file = os.path.join(
        robot_config_package,
        "config",
        "moveit_controllers.yaml",
    )
    ros2_controllers_file = os.path.join(
        robot_bringup_package,
        "config",
        "separable_jaws_controllers.yaml",
    )
    rviz_config_file = os.path.join(
        robot_config_package,
        "config",
        "moveit.rviz",
    )
    moveit_config = (
        MoveItConfigsBuilder("separable_jaws", package_name="separable_jaws_config")
        .robot_description(
            file_path=robot_description_file,
            mappings={
                "using_left_arm": "true",
                "using_right_arm": "false",
                "using_fake_hardware": "true",
                "using_gazebo": "false",
            },
        )
        .robot_description_semantic(file_path=robot_description_semantic)
        .trajectory_execution(file_path=moveit_controllers_file)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # declare nodes
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="both",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    left_arm_controller_spawner = Node(
        condition=IfCondition(using_left_arm),
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    right_arm_controller_spawner = Node(
        condition=IfCondition(using_right_arm),
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    mongodb_server_node = Node(
        condition=IfCondition(using_db),
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        output="both",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
    )
    
    nodes = [
        static_tf_node,
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        mongodb_server_node,
    ]

    return LaunchDescription(arguments + nodes)
