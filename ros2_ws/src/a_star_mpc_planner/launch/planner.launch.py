"""A*+MPC planner bring-up for the Navigation / DLIO stack.

Spawns
------
  a_star_node          2.5D Gaussian-grid A* on /local_voxel_map/obstacles, pose
                       from /dlio/odom_node/odom; publishes /a_star/path.
  mpc_node             CasADi/IPOPT nonlinear MPC tracker; publishes a velocity
                       command as a Twist on /mpc/cmd_vel.
  cmd_vel_to_amo       (optional, bridge:=true) g1_sim_bridge node that forwards
                       /mpc/cmd_vel as {vx,vy,yaw} JSON to the AMO WebSocket
                       server (:8766). The AMO policy is not a ROS 2 process, so
                       this is how velocity reaches the gait.

Run (inside the ROS 2 / localization container, after DLIO + g1_local_map are up):

    ros2 launch a_star_mpc_planner planner.launch.py
    ros2 launch a_star_mpc_planner planner.launch.py bridge:=false   # planner only
    ros2 launch a_star_mpc_planner planner.launch.py amo_host:=127.0.0.1 amo_port:=8766

The whole stack runs on ROS_DOMAIN_ID=42 to match real_localization.launch.py and
isolate it from the ROS 2 Jazzy host (see docs/perception/DLIO_G1_MID360_TUNING.md and the
QoS/transport notes in docs/perception/LOCAL_VOXEL_MAP.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('a_star_mpc_planner'),
        'config', 'planner_params_default.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bridge = LaunchConfiguration('bridge')
    amo_host = LaunchConfiguration('amo_host')
    amo_port = LaunchConfiguration('amo_port')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    rviz = LaunchConfiguration('rviz')
    gait = LaunchConfiguration('gait')        # 'amo' | 'unitree' | 'sonic'
    net_if = LaunchConfiguration('net_if')    # robot NIC for the Unitree native gait
    sonic_host = LaunchConfiguration('sonic_host')
    sonic_port = LaunchConfiguration('sonic_port')
    hold_arms = LaunchConfiguration('hold_arms')  # pin SONIC upper body (arms still)

    common = [params_file, {'use_sim_time': use_sim_time}]

    # Which gait consumes /mpc/cmd_vel — run exactly one (all drive the motors).
    amo_bridge_on = IfCondition(PythonExpression(
        ["'", bridge, "' == 'true' and '", gait, "' == 'amo'"]))
    unitree_bridge_on = IfCondition(PythonExpression(
        ["'", bridge, "' == 'true' and '", gait, "' == 'unitree'"]))
    sonic_bridge_on = IfCondition(PythonExpression(
        ["'", bridge, "' == 'true' and '", gait, "' == 'sonic'"]))

    a_star_node = Node(
        package='a_star_mpc_planner',
        executable='a_star_node',
        name='a_star_node',
        output='screen',
        parameters=common,
    )

    mpc_node = Node(
        package='a_star_mpc_planner',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=common,
    )

    # Global planner (long-horizon routing layer). Publishes /global_path that the
    # local A* follows as a carrot. Toggle with global_planner:=false.
    global_planner_node = Node(
        package='a_star_mpc_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('global_planner')),
        parameters=common,
    )

    # Forward the MPC's Twist to the AMO WebSocket gait. cmd_vel_topic points at
    # /mpc/cmd_vel (the same bridge also serves teleop on /cmd_vel — run only one
    # source at a time). Caps are set at/above the MPC velocity envelope so they
    # never clip a valid MPC command; AMO applies its own internal limits.
    cmd_vel_to_amo = Node(
        package='g1_sim_bridge',
        executable='cmd_vel_to_amo_node',
        name='cmd_vel_to_amo',
        output='screen',
        condition=amo_bridge_on,
        parameters=[{
            'cmd_vel_topic': '/mpc/cmd_vel',
            'amo_host': amo_host,
            'amo_port': amo_port,
            'rate_hz': 20.0,
            'max_forward_vel': 0.5,
            # Lateral cap — kept in step with mpc_vy_max (0.12) so the bridge
            # never clips a vy the MPCC actually planned. Low value DISCOURAGES
            # lateral motion (small dodges only). Set 0.0 to forbid it entirely.
            'max_lateral_vel': 0.12,
            'max_yaw_rate': 0.8,
            # Fail-safe: zero the gait command if the MPC stops publishing.
            'cmd_timeout_sec': 0.5,
            'use_sim_time': use_sim_time,
        }],
    )

    # Alternative gait: forward /mpc/cmd_vel to the Unitree NATIVE (factory)
    # walking policy via the Unitree SDK LocoClient (gait:=unitree). Mutually
    # exclusive with the AMO bridge above. Reads the same /estop + watchdog
    # contract; smooths the velocity command (the high-level analog of AMO joint
    # filtering). The robot must be brought to walking control first — set
    # auto_bring_up:=true here, or run `ros2 run g1_sim_bridge unitree_gait_test`.
    cmd_vel_to_unitree_loco = Node(
        package='g1_sim_bridge',
        executable='cmd_vel_to_unitree_loco_node',
        name='cmd_vel_to_unitree_loco',
        output='screen',
        condition=unitree_bridge_on,
        parameters=[{
            'cmd_vel_topic': '/mpc/cmd_vel',
            'net_if': net_if,
            'unitree_domain_id': 0,        # Unitree DDS domain (robot), != ROS domain
            'rate_hz': 20.0,
            'max_forward_vel': 0.5,
            'max_lateral_vel': 0.12,
            'max_yaw_rate': 0.8,
            'cmd_timeout_sec': 0.5,
            'auto_bring_up': False,        # SAFETY: bring up manually by default
            'use_sim_time': use_sim_time,
        }],
    )

    # Alternative gait: forward /mpc/cmd_vel to the SONIC whole-body policy
    # (gait:=sonic). The SONIC deploy controller is not a ROS 2 process — it SUBs
    # a ZMQ PUB socket (:5556) and drives the robot over DDS — so this bridge is
    # the ZMQ counterpart of the AMO/Unitree bridges and is mutually exclusive
    # with them. Unlike those, it also reads /dlio/odom_node/odom: the MPC's Twist
    # is body-frame but SONIC steers with WORLD-frame direction vectors, so the
    # node anchors `facing` on measured yaw (closed loop, no heading drift).
    cmd_vel_to_sonic = Node(
        package='g1_sim_bridge',
        executable='cmd_vel_to_sonic_node',
        name='cmd_vel_to_sonic',
        output='screen',
        condition=sonic_bridge_on,
        parameters=[{
            'cmd_vel_topic': '/mpc/cmd_vel',
            'odom_topic': '/dlio/odom_node/odom',
            # The default '*' (ZMQ bind-all wildcard) is not valid YAML, so
            # launch_ros can't infer the type — force string typing explicitly.
            'zmq_host': ParameterValue(sonic_host, value_type=str),
            'zmq_port': sonic_port,
            'rate_hz': 30.0,
            'max_forward_vel': 0.5,
            'max_lateral_vel': 0.12,
            'max_yaw_rate': 0.8,
            'cmd_timeout_sec': 0.5,
            # SONIC realises ~0.85x commanded m/s; feed-forward correction.
            'speed_gain': 1.18,
            # `facing` must lead measured heading or the policy never turns.
            'facing_lookahead_sec': 0.4,
            # Pin the 17-DOF upper body to the neutral standing pose so the arms
            # stay still instead of swinging with the policy's generated gait.
            'hold_arms': ParameterValue(hold_arms, value_type=bool),
            'use_sim_time': use_sim_time,
        }],
    )

    # Optional RViz, OFF by default. real_localization.launch.py already opens
    # the shared g1_dlio.rviz (which now includes the planner displays), so leave
    # this false when running the full stack to avoid two RViz windows. Set
    # rviz:=true only when running the planner STANDALONE for visualization.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_planner',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('g1_bringup'), 'rviz', 'g1_dlio.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML parameter file for a_star_node + mpc_node.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (true in sim, false on the real robot).'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Run the gait bridge that forwards /mpc/cmd_vel to the '
                        'selected gait (see gait:=). false = planner only.'),
        DeclareLaunchArgument(
            'gait', default_value='amo',
            description="Which gait consumes /mpc/cmd_vel: 'amo' (RoboJuDo joint "
                        "policy over WebSocket :8766), 'unitree' (native factory "
                        "gait via the Unitree SDK LocoClient), or 'sonic' (SONIC "
                        "whole-body policy over ZMQ :5556). Run only one."),
        DeclareLaunchArgument(
            'net_if',
            default_value=EnvironmentVariable('UNITREE_NET_IFACE', default_value='eth0'),
            description='Robot network interface for the Unitree native gait '
                        '(gait:=unitree). Defaults to $UNITREE_NET_IFACE.'),
        DeclareLaunchArgument(
            'sonic_host', default_value='*',
            description='Bind address of the SONIC ZMQ PUB socket (gait:=sonic). '
                        "'*' binds all interfaces; the SONIC deploy controller "
                        'SUBs it.'),
        DeclareLaunchArgument(
            'sonic_port', default_value='5556',
            description='Port of the SONIC ZMQ PUB socket (gait:=sonic).'),
        DeclareLaunchArgument(
            'hold_arms', default_value='false',
            description='Pin the SONIC upper body to the neutral standing pose so '
                        'the arms stay still instead of swinging with the policy '
                        'gait (gait:=sonic). true = arms held.'),
        DeclareLaunchArgument(
            'global_planner', default_value='true',
            description='Run the global planner layer (long-horizon /global_path '
                        'the local A* follows as a carrot). Set false for '
                        'local-only planning.'),
        DeclareLaunchArgument(
            'amo_host', default_value='127.0.0.1',
            description='Host of the AMO WebSocket server (amo_inference, :8766).'),
        DeclareLaunchArgument(
            'amo_port', default_value='8766',
            description='Port of the AMO WebSocket server.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='42',
            description='DDS domain, matching real_localization.launch.py, to '
                        'isolate the stack from the ROS 2 Jazzy host.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz (g1_bringup g1_dlio.rviz) for standalone '
                        'planner viz. Leave false when real_localization already '
                        'runs RViz, to avoid two windows.'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        a_star_node,
        mpc_node,
        global_planner_node,
        cmd_vel_to_amo,
        cmd_vel_to_unitree_loco,
        cmd_vel_to_sonic,
        rviz_node,
    ])
