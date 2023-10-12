# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

import autosail_gz.launch
from autosail_gz.model import Model


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    controller_pkg = LaunchConfiguration('controller_pkg').perform(context)
    sensors_s= LaunchConfiguration('sensor_source').perform(context)
    launch_processes = []

    models = []
    if config_file != '':
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
      m = Model('sailboat', 'sailboat', [-532, 162, 1, 0, 0, 1])
      if robot_urdf and robot_urdf != '':
          m.set_urdf(robot_urdf)
      models.append(m)

    launch_processes.extend(autosail_gz.launch.simulation(world_name, headless))
    launch_processes.extend(autosail_gz.launch.spawn(sim_mode, world_name, models, robot, controller, controller_pkg,sensors_s))

    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(autosail_gz.launch.competition_bridges(world_name))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Worlds name'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the model. '),
        DeclareLaunchArgument(
            'controller',
            default_value='my_controller_node',
            description='Controller assigned to the model'), 
        DeclareLaunchArgument(
            'controller_pkg',
            default_value='my_sailboat_controller',
            description='Controller package'),
        DeclareLaunchArgument(
            'sensor_source',
            default_value='Ros',
            description='Sensor source of information: "Ros","Gazebo","uController".'
                        'uController: This will activate the virtual serial port inside the nodes to receive the information'
                        'Gazebo: This will activate the gazebo sensor decoder node,'
                        'Ros: Since is the default interface for the proyect it would not activate new nodes '),
        OpaqueFunction(function=launch),
    ])