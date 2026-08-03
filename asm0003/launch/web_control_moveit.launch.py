import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_dir = get_package_share_directory('asm0003')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'asm0003.urdf')
    srdf_path = os.path.join(pkg_dir, 'config', 'asm0003.srdf')
    kinematics_path = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    ompl_path = os.path.join(pkg_dir, 'config', 'ompl_planning.yaml')
    controllers_path = os.path.join(pkg_dir, 'config', 'moveit_controllers.yaml')
    joint_limits_path = os.path.join(pkg_dir, 'config', 'joint_limits.yaml')
    merger_script = os.path.join(pkg_dir, 'scripts', 'joint_state_merger.py')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    with open(srdf_path, 'r', encoding='utf-8') as f:
        robot_description_semantic = f.read()

    kinematics = load_yaml(kinematics_path)
    ompl_config = load_yaml(ompl_path)
    controllers = load_yaml(controllers_path)
    joint_limits = load_yaml(joint_limits_path)

    ompl_pipeline = dict(ompl_config)
    ompl_pipeline['planning_plugin'] = 'ompl_interface/OMPLPlanner'

    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics,
        'robot_description_planning': joint_limits,
        'planning_pipelines': {'pipeline_names': ['ompl']},
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
        'moveit_simple_controller_manager': controllers.get('moveit_simple_controller_manager', {}),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.2,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.01,
            'execution_duration_monitoring': False,
        },
        'plan_execution': {
            'record_trajectory_state_frequency': 10.0,
        },
        'scene_graph': {},
        'octomap': {},
    }

    input_topics = LaunchConfiguration('input_topics')
    joint_state_topic = LaunchConfiguration('joint_state_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topics',
            default_value='/master2/joint_states,/master0/joint_states',
        ),
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/joint_states',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '1.5708', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'base_link'],
        ),

        ExecuteProcess(
            cmd=['python3', merger_script,
                 '--ros-args',
                 '-p', ['input_topics:=', input_topics],
                 '-p', 'output_topic:=/joint_states',
                 '-p', 'publish_rate_hz:=100.0'],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
            }],
            remappings=[
                ('/joint_states', joint_state_topic),
                ('joint_states', joint_state_topic),
                ('/robot_description', '/robot_description_local'),
                ('robot_description', '/robot_description_local'),
            ],
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group_backend',
            output='screen',
            parameters=[
                moveit_config,
                {
                    'publish_robot_description': True,
                    'publish_robot_description_semantic': True,
                },
            ],
            remappings=[
                ('/joint_states', joint_state_topic),
                ('joint_states', joint_state_topic),
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])
