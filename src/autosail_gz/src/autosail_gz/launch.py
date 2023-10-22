# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, EmitEvent
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import autosail_gz.bridges

import os


def simulation(world_name, headless=False):
    gz_args = ['-v 4', '-r']
    if headless:
        gz_args.append('-s')
    gz_args.append(f'{world_name}.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    # Register handler for shutting down ros launch when gazebo process exits
    # monitor_sim.py will run until it can not find the gazebo process.
    p = os.path.join(get_package_share_directory('autosail_ros'), 'launch',
                     'monitor_sim.py')
    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', p],
        name='monitor_sim',
        output='screen',
    )
    # Once monitor_sim.py exits, a process exit event is triggered which causes the
    # handler to emit a Shutdown event
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim_proc,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Simulation ended'))
            ]
        )
    )

    return [gz_sim, monitor_sim_proc, sim_exit_event_handler]


def competition_bridges(world_name):
    bridges = [
        autosail_gz.bridges.clock(),
    ]
    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))
    return nodes


def spawn(sim_mode, world_name, models, robot=None,controller='my_controller_node',controller_pkg='my_sailboat_controller',sensor_source='Ros'):
    if type(models) != list:
        models = [models]

    launch_processes = []
    for model in models:
        if robot and model.model_name != robot:
            continue
        silent_var=model.spawn_args()
        # print(silent_var)
        # Script to insert model in running simulation
        # if sim_mode == 'full' or sim_mode == 'sim':
        #     gz_spawn_entity = Node(
        #         package='ros_gz_sim',
        #         executable='create',
        #         output='screen',
        #         arguments=model.spawn_args() ### Here is where the *.sdf model file is generated!!!!
        #     )
        #     launch_processes.append(gz_spawn_entity)

        if sim_mode == 'full' or sim_mode == 'bridge':
            bridges, nodes, custom_launches = model.bridges(world_name)
            payload = model.payload_bridges(world_name)# Here is where the ROS/Gazebo bridges are generated
            payload_bridges = payload[0]
            payload_nodes = payload[1]
            payload_launches = payload[2]

            bridges.extend(payload_bridges)
            nodes.extend(payload_nodes)

            #GZ/ROS bridges creator
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges],
            ))
            if sensor_source=='uController' or sensor_source=='HILT':
                # Microcontroller sensors decode
                nodes.append(Node(
                    package='my_sailboat_controller',
                    executable='decode_node',
                    output='screen',
                ))
            elif sensor_source=='Gazebo' or sensor_source=='HILT':
                # GAZEBO sensors decoder
                nodes.append(Node(
                    package='my_sailboat_controller',
                    output='screen',
                    executable='gz_decode_node',
                ))
            else: 
                pass
            # tf broadcaster
            nodes.append(Node(
                package='autosail_ros',
                executable='pose_tf_broadcaster',
                output='screen',
            ))

            # Controller
            nodes.append(Node(
                package=controller_pkg,
                executable=controller,
                output='screen',
            ))
            group_action = GroupAction([
                PushRosNamespace('autosail'), #this action configures a common namespace for all the nodes created 
                *nodes
            ])
            if sim_mode == 'sim':
            #     # handler = RegisterEventHandler(
            #     #     event_handler=OnProcessExit(
            #     #         target_action=gz_spawn_entity,
            #     #         on_exit=[group_action],
            #     #     )
            #     # )
            #     # launch_processes.append(handler)
                pass
            else:
                launch_processes.append(group_action)
            
             # GAZEBO GUI
            # gz_spawn_entity =Node(
            #     package='GUI',
            #     executable='GUI_node',
            #     output='screen',
            # )
            # launch_processes.append(gz_spawn_entity)

            

            launch_processes.extend(payload_launches)
            launch_processes.extend(custom_launches)

    return launch_processes
