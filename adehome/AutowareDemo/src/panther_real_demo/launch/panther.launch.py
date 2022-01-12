# Copyright 2020, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    panther_real_demo_pkg_prefix = get_package_share_directory('panther_real_demo')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')

    ## CUSTOM PARAMS
    pure_pursuit_param_file = os.path.join(
        panther_real_demo_pkg_prefix, 'custom_params/pure_pursuit.param.yaml')
    pc_filter_transform_param_file = os.path.join(
        panther_real_demo_pkg_prefix, 'custom_params/os64_filter_transform.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        panther_real_demo_pkg_prefix, 'custom_params/vehicle_characteristics.param.yaml')
    map_publisher_param_file = os.path.join(
        panther_real_demo_pkg_prefix, 'custom_params/map_publisher.param.yaml')
    panther_adapter_param_file = os.path.join(
        panther_real_demo_pkg_prefix, 'custom_params/panther_adapter.param.yaml')
    map_pcd_file = os.path.join(
        panther_real_demo_pkg_prefix, 'data/custom_map.pcd')
    map_yaml_file = os.path.join(
        panther_real_demo_pkg_prefix, 'data/custom_map.yaml')

    lgsvl_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/lgsvl_interface.param.yaml')
    ndt_localizer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/ndt_localizer_sim.param.yaml')


    urdf_path = os.path.join(panther_real_demo_pkg_prefix, 'urdf/panther.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    # Arguments

    panther_adapter_param = DeclareLaunchArgument(
        'panther_adapter_param_file',
        default_value=panther_adapter_param_file,
        description='Path to patnher adapter params'
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    pure_pursuit_controller_param = DeclareLaunchArgument(
        "pure_pursuit_param_file",
        default_value=pure_pursuit_param_file,
        description="Path to config file to Pure Pursuit Controller",
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    # Nodes

    panther_adapter = Node(
        package='panther_adapter',
        executable='panther_adapter',
        name='panther_adapter',
        output='screen',
        parameters=[LaunchConfiguration('panther_adapter_param_file')]
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        name='lgsvl_interface_node',
        output='screen',
        parameters=[
            LaunchConfiguration('lgsvl_interface_param_file'),
            {"lgsvl.publish_tf": True}
        #   {"use_nav_odometry_topic": False}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/odometry/filtered"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )
    filter_transform_os64 = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_os64',
        namespace='os_cloud_node',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points")]
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[
            LaunchConfiguration('map_publisher_param_file'),
            {"map_pcd_file": map_pcd_file},
            {"map_yaml_file": map_yaml_file}]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/os_cloud_node/points_fused_downsampled"),
            ("observation_republish", "/os_cloud_node/points_fused_viz"),
        ]
    )
    pure_pursuit_controller = Node(
        package="pure_pursuit_nodes",
        executable="pure_pursuit_node_exe",
        namespace="control",
        name="pure_pursuit_node",
        output="screen",
        parameters=[
            LaunchConfiguration("pure_pursuit_param_file"), 
            LaunchConfiguration('vehicle_characteristics_param_file')],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/vehicle_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
            ("tf", "/tf"),
        ],
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([panther_real_demo_pkg_prefix, '/launch/avp_core.launch.py']),
        launch_arguments={}.items()
    )

    vis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([panther_real_demo_pkg_prefix, '/launch/autoware_auto_visualization.launch.py']),
        launch_arguments={}.items()
    )

    point_type_adapter_pkg_prefix = get_package_share_directory(
        'point_type_adapter')

    adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_type_adapter_pkg_prefix,
                         'launch/point_type_adapter.launch.py'))
    )

    return LaunchDescription([
        lgsvl_interface_param,
        map_publisher_param,
        ndt_localizer_param,
        pure_pursuit_controller_param,
        pc_filter_transform_param,
        vehicle_characteristics_param,
        panther_adapter_param,
        urdf_publisher,
        lgsvl_interface,
        map_publisher,
        ndt_localizer,
        pure_pursuit_controller,
        filter_transform_os64,
        core_launch,
        adapter_launch,
        panther_adapter,
        vis_launch,
    ])
