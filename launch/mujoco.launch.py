import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# --------------------------
# Internal: minimal generic override
# --------------------------
def generate_temp_config(config_path, package_name, kv_pairs):
    """
    Load <package_name>/<config_path>, apply overrides from kv_pairs,
    and write to /tmp/<package_name>/temp_controllers.yaml. Returns the path.
    kv_pairs: list of (dotted_key, raw_value_str)
    """
    pkg_dir = get_package_share_directory(package_name)
    src_path = os.path.join(pkg_dir, config_path)
    dst_path = os.path.join('/tmp', package_name, 'temp_controllers.yaml')
    os.makedirs(os.path.dirname(dst_path), exist_ok=True)

    with open(src_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    for dotted_key, raw_val in kv_pairs:
        parts = [p for p in dotted_key.split('.') if p]
        if len(parts) < 2:
            raise ValueError(
                f"Key '{dotted_key}' is incomplete; expected '<ns>.[ros__parameters.]foo.bar'"
            )
        # Auto-insert ros__parameters right after namespace if omitted
        if parts[1] != 'ros__parameters':
            parts.insert(1, 'ros__parameters')

        try:
            val = yaml.safe_load(raw_val)
        except Exception:
            val = raw_val

        cur = cfg
        for k in parts[:-1]:
            if not isinstance(cur.get(k), dict):
                cur[k] = {}
            cur = cur[k]
        cur[parts[-1]] = val

    with open(dst_path, 'w') as f:
        yaml.dump(cfg, f, sort_keys=False)
        print(f"[launch] Temp controllers.yaml written to {dst_path}")

    return dst_path


# --------------------------
# ROS nodes / launch wiring
# --------------------------
def control_spawner(names, inactive=False):
    args = list(names)
    args += ['--param-file', LaunchConfiguration('controllers_yaml')]
    if inactive:
        args.append('--inactive')
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=args,
        output='screen'
    )


def setup_controllers(context):
    organization_value = LaunchConfiguration('organization').perform(context)
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    policy_path_value = LaunchConfiguration('policy_path').perform(context)
    start_step_value = LaunchConfiguration('start_step').perform(context)
    ext_pos_corr = LaunchConfiguration('ext_pos_corr').perform(context)

    kv_pairs = []
    if policy_path_value:
        abs_path = os.path.abspath(os.path.expanduser(os.path.expandvars(policy_path_value)))
        kv_pairs.append(('motion_tracking_controller.policy.path', abs_path))
    if start_step_value:
        kv_pairs.append(('motion_tracking_controller.motion.start_step', start_step_value))
    if ext_pos_corr.lower() in ["true", "1", "yes"]:
        kv_pairs.append(('state_estimator.estimation.contact.height_sensor_noise', 1e10))
        kv_pairs.append(('state_estimator.estimation.position.topic', "/mid360"))

    controllers_config_path = f'config/{organization_value}/{robot_type_value}/sim.yaml'
    temp_controllers_config_path = generate_temp_config(
        controllers_config_path,
        'humanoid_controller',
        kv_pairs
    )

    set_controllers_yaml = SetLaunchConfiguration(
        name='controllers_yaml',
        value=temp_controllers_config_path
    )

    active_list = ["state_estimator", "standby_controller"]
    inactive_list = ["damping_controller", "passive_controller", "stand_loop_controller", "motion_tracking_controller", "walking_controller"]

    active_spawner = control_spawner(active_list)
    inactive_spawner = control_spawner(inactive_list, inactive=True)

    return [set_controllers_yaml, active_spawner, inactive_spawner]


def generate_launch_description():
    organization_arg = DeclareLaunchArgument(name='organization', default_value='unitree', choices=['unitree', 'agibot', 'engineai'], description='Name of the organization')
    robot_type_arg = DeclareLaunchArgument(name='robot_type', default_value='g1', choices=['g1', 'x2', 'pm01'], description='Name of the robot type')
    policy_path_arg = DeclareLaunchArgument(name='policy_path', default_value='', description='Absolute or ~-expanded path for motion_tracking_controller.policy.path')
    start_step_arg = DeclareLaunchArgument(name='start_step', default_value='0', description='Integer start step for motion_tracking_controller.motion.start_step')
    ext_pos_corr_arg = DeclareLaunchArgument(name='ext_pos_corr', default_value='false', description='Enable external position correction')

    urdf_file = PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", LaunchConfiguration('organization'), LaunchConfiguration('robot_type'), "robot.xacro"])
    robot_description_content = ParameterValue(Command([FindExecutable(name='xacro'), " ", urdf_file, " ", "simulation:=", "mujoco"]), value_type=str)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 1.0,
            'use_sim_time': True
        }],
    )

    mjcf_relative_path = ParameterValue(PythonExpression(["'mjcf/", LaunchConfiguration('organization'), "/", LaunchConfiguration('robot_type'), ".xml'"]), value_type=str)
    mujoco_simulator = Node(
        package='mujoco_sim_ros2',
        executable='mujoco_sim',
        parameters=[
            {"model_package": "robot_description",
             "model_file": mjcf_relative_path,
             "physics_plugins": ["mujoco_ros2_control::MujocoRos2ControlPlugin"],
             'robot_description': robot_description_content,
             "use_sim_time": True
             },
            LaunchConfiguration('controllers_yaml'),
        ],
        output='screen')

    wandb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/wandb.launch.py"]),
        launch_arguments={
            "wandb_path": LaunchConfiguration("wandb_path")
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('policy_path'), "' == ''"])
        )
    )

    controllers_opaque_func = OpaqueFunction(function=setup_controllers)

    joy_config_file = PathJoinSubstitution([FindPackageShare("humanoid_controller"), "config", LaunchConfiguration('organization'), LaunchConfiguration('robot_type'), "joy.yaml"])
    joy_node = Node(package='joy_linux',  executable='joy_linux_node', output='both')
    joy_teleop = Node(package='joy_teleop', executable='joy_teleop', parameters=[joy_config_file])

    return LaunchDescription([
        organization_arg,
        robot_type_arg,
        policy_path_arg,
        start_step_arg,
        ext_pos_corr_arg,
        wandb,
        controllers_opaque_func,
        mujoco_simulator,
        node_robot_state_publisher,
        joy_node,
        joy_teleop
    ])
