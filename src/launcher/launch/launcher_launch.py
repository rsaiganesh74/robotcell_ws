#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution

def generate_launch_description():
    wms_server = ExecuteProcess(
        cmd=[[
            'python3 ~/robotcell_ws/src/http_server_client/http_server_client/wms_server.py'
        ]],
        shell=True
    )
    return LaunchDescription([
        Node(
            package='barcode_scanner',
            executable='scanner',
            name='scanner',
            output="screen"
        ),
        Node(
            package='door_handle',
            executable='door_handle',
            name='door',
            output="screen"
        ),
        Node(
            package='estop',
            executable='estop_node',
            name='estop',
            output="screen"
        ),
        Node(
            package='stack_light',
            executable='stack_light_node',
            name='stack_light',
            output="screen"
        ),
        Node(
            package='robot_server',
            executable='robot_server',
            name='robot_server',
            output="screen"
        ),
        Node(
            package='http_server_client',
            executable='robot_server',
            name='request_receiver',
            output="screen"
        ),
        Node(
            package='hmi',
            executable='hmi_node',
            name='hmi',
            output="screen"
        ),
        wms_server,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])