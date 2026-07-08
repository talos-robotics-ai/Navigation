"""Static-map localization bring-up (OPTIONAL — pairs with use_static_map:=true).

Serves a pre-built 2D map and localizes the robot in it, so the global planner
can route over the static map in the `map` frame:

    pointcloud_to_laserscan → flatten the MID-360 3D cloud into a 2D /scan for AMCL
    map_server              → publish the pre-built OccupancyGrid on /map (latched)
    amcl                    → localize against /map → map→odom TF + /amcl_pose
    lifecycle_manager       → activate map_server + amcl (Nav2 lifecycle nodes)

Build the map first with scripts/dlio_map_to_occupancy.py, then:

    ros2 launch a_star_mpc_planner static_map_localization.launch.py map:=/abs/path/map.yaml
    # planner:  ros2 launch a_star_mpc_planner planner.launch.py  (with use_static_map:=true)
    # in RViz, set the initial pose (2D Pose Estimate) so AMCL converges.

Needs: ros-humble-nav2-map-server, ros-humble-nav2-amcl,
ros-humble-nav2-lifecycle-manager, ros-humble-pointcloud-to-laserscan.
Tune scan_min_height/scan_max_height (metres, in base_link) to a wall-height slice.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_min_h = LaunchConfiguration('scan_min_height')
    scan_max_h = LaunchConfiguration('scan_max_height')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the map .yaml file.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/dlio/odom_node/pointcloud/deskewed',
            description='PointCloud2 flattened into /scan for AMCL.'),
        DeclareLaunchArgument('scan_min_height', default_value='-0.5'),
        DeclareLaunchArgument('scan_max_height', default_value='1.5'),

        # 3D cloud -> 2D /scan (AMCL is a 2D localizer; the MID-360 is 3D).
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen',
            remappings=[('cloud_in', cloud_topic), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': scan_min_h,
                'max_height': scan_max_h,
                'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.0087,
                'scan_time': 0.1, 'range_min': 0.3, 'range_max': 30.0,
                'use_inf': True,
            }],
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'scan_topic': '/scan',
                'set_initial_pose': True,   # start at the map origin; refine with RViz 2D Pose Estimate
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])
