# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_type_str,
        robot_type_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    return [robot_state_publisher]


def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='true',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot.')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, franka_hand])

    # Gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ik_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_impedance_with_ik_example_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # MoveIt2 move_group for IK service (kinematics-only URDF, no ros2_control)
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    moveit_robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', franka_xacro_file,
                ' ros2_control:=false hand:=true robot_type:=fr3 robot_ip:=dont_need'
            ]),
            value_type=str
        )
    }

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    moveit_robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
                     ' hand:=true']),
            value_type=str
        )
    }

    kinematics_yaml_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'), 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_robot_description,
            moveit_robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # Barrier obstacle in robot workspace
    barrier_sdf = (
        '<sdf version="1.8">'
        '<model name="barrier">'
        '<static>true</static>'
        '<link name="link">'
        '<collision name="collision">'
        '<geometry><box><size>0.3 0.3 0.3</size></box></geometry>'
        '</collision>'
        '<visual name="visual">'
        '<geometry><box><size>0.3 0.3 0.3</size></box></geometry>'
        '<material>'
        '<ambient>0.2 0.6 0.9 1</ambient>'
        '<diffuse>0.2 0.6 0.9 1</diffuse>'
        '</material>'
        '</visual>'
        '<sensor name="contact_sensor" type="contact">'
        '<always_on>true</always_on>'
        '<update_rate>100</update_rate>'
        '<contact>'
        '<collision>collision</collision>'
        '</contact>'
        '</sensor>'
        '</link>'
        '</model>'
        '</sdf>'
    )

    # Bridge the Gz contact sensor topic into ROS 2
    # Topic pattern: /world/{world}/model/barrier/link/link/sensor/contact_sensor/contact
    barrier_contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/barrier/link/link/sensor/contact_sensor/contact'
            '@ros_gz_interfaces/msg/Contacts'
            '[gz.msgs.Contacts'
        ],
        remappings=[(
            '/world/empty/model/barrier/link/link/sensor/contact_sensor/contact',
            '/barrier_contact'
        )],
        output='screen',
    )
    spawn_barrier = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'barrier', '-string', barrier_sdf,
                   '-x', '0.6', '-y', '0.0', '-z', '0.4'],
        output='screen',
    )

    # Falcon hardware node
    falcon_node = Node(
        package='falcon_ros2',
        executable='falcon_node',
        name='falcon_node',
        output='screen',
    )

    # Falcon-Franka bridge
    falcon_bridge = Node(
        package='franka_falcon_haptic_control',
        executable='falcon_franka_bridge',
        name='falcon_franka_bridge',
        output='screen',
        parameters=[{
            'franka_center_x': 0.5,
            'franka_center_y': 0.0,
            'franka_center_z': 0.4,
            'scale_x': 5.0,
            'scale_y': 5.0,
            'scale_z': 5.0,
            'loop_rate_hz': 100.0,
            # Falcon rests at z=0.125 when homed; subtract so Franka target stays at center when at rest
            'falcon_rest_z': 0.125,
            # FR3 default EE orientation measured from TF (fr3_link0 -> fr3_link8)
            # RPY ~ (180deg, 0deg, -45deg), matches default joint config
            'ee_orientation_w': 0.001,
            'ee_orientation_x': 0.924,
            'ee_orientation_y': -0.383,
            'ee_orientation_z': -0.002,
        }],
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        robot_type_launch_argument,
        namespace_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        spawn,
        move_group,
        falcon_node,
        falcon_bridge,
        barrier_contact_bridge,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster, spawn_barrier],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ik_controller, load_gripper_controller],
            )
        ),
    ])
