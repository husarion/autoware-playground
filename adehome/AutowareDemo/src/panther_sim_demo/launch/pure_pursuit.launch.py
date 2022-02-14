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
    panther_sim_demo_pkg_prefix = get_package_share_directory('panther_sim_demo')


    pure_pursuit_param_file = os.path.join(
        panther_sim_demo_pkg_prefix, 'custom_params/pure_pursuit.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        panther_sim_demo_pkg_prefix, 'custom_params/vehicle_characteristics.param.yaml')
   
    pure_pursuit_controller_param = DeclareLaunchArgument(
        "pure_pursuit_param_file",
        default_value=pure_pursuit_param_file,
        description="Path to config file to Pure Pursuit Controller",
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    # Nodes

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

    return LaunchDescription([
        vehicle_characteristics_param,
        pure_pursuit_controller_param,
        pure_pursuit_controller,
    ])
