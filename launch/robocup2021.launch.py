#!/usr/bin/env python3

# Copyright (c) 2019, Toyota Motor Corporation
# Copyright (c) 2021, MID Academic Promotions, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name = LaunchConfiguration('robot_name', default=EnvironmentVariable('ROBOT_NAME', default_value='hsrb'))
    camera_controller = LaunchConfiguration('camera_controller', default='false')
    seed = LaunchConfiguration('seed', default='1')

    # Common parameters for object detectors
    def create_object_detector(name, box_name, box_size, box_pose, object_names, 
                             object_axes=None, target_axes=None, allow_degree=None, both_direction=None):
        params = {
            'box_name': box_name,
            'box_size': box_size,
            'box_pose': box_pose,
            'object_names': object_names
        }
        if object_axes is not None:
            params.update({
                'object_axes': object_axes,
                'target_axes': target_axes,
                'allow_degree': allow_degree,
                'both_direction': both_direction
            })
        
        return Node(
            package='tmc_gazebo_task_evaluators',
            executable='object_in_box_detector',
            name=name,
            output='screen',
            parameters=[params]
        )

    # Create launch description
    ld = LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='hsrb'),
        DeclareLaunchArgument('camera_controller', default_value='false'),
        DeclareLaunchArgument('seed', default_value='1'),

        # Drawer Left detectors
        create_object_detector(
            'any_in_drawerleft_detector',
            'trofast_1::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_*']
        ),
        create_object_detector(
            'shapeitems_in_drawerleft_detector',
            'trofast_1::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_shapeitem_*']
        ),

        # Drawer Top detectors
        create_object_detector(
            'any_in_drawertop_detector',
            'trofast_2::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_*']
        ),
        create_object_detector(
            'tools_in_drawertop_detector',
            'trofast_2::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_tool_*']
        ),

        # Drawer Bottom detectors
        create_object_detector(
            'any_in_drawerbottom_detector',
            'trofast_3::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_*']
        ),
        create_object_detector(
            'tools_in_drawerbottom_detector',
            'trofast_3::link',
            [0.35, 0.21, 0.25],
            [0, 0, 0.125],
            ['task1_tool_*']
        ),

        # Drawer Front detector
        create_object_detector(
            'drawer_in_drawerfront_detector',
            'wrc_stair_like_drawer::link',
            [0.5, 1, 1.5],
            [0.5, 0, 0],
            ['trofast_*']
        ),

        # Container A detectors
        create_object_detector(
            'any_in_containera_detector',
            'wrc_container_b::link',
            [0.18, 0.26, 0.11],
            [0, 0, 0.055],
            ['task1_*']
        ),
        create_object_detector(
            'kitchenitems_in_containera_detector',
            'wrc_container_b::link',
            [0.18, 0.26, 0.11],
            [0, 0, 0.055],
            ['task1_kitchenitem_*']
        ),

        # Container B detectors
        create_object_detector(
            'any_in_containerb_detector',
            'wrc_container_a::link',
            [0.12, 0.12, 0.2],
            [0, 0, 0.1],
            ['task1_*']
        ),
        create_object_detector(
            'largemarker_in_containerb_detector',
            'wrc_container_a::link',
            [0.12, 0.12, 0.2],
            [0, 0, 0.1],
            ['task1_tool_ycb_040_large_marker_*'],
            [0, 1, 0],
            [0, 0, -1],
            45,
            0
        ),
        create_object_detector(
            'fork_in_containerb_detector',
            'wrc_container_a::link',
            [0.12, 0.12, 0.2],
            [0, 0, 0.1],
            ['task1_kitchenitem_ycb_030_fork_*'],
            [1, 0, 0],
            [0, 0, 1],
            45,
            0
        ),
        create_object_detector(
            'spoon_in_containerb_detector',
            'wrc_container_a::link',
            [0.12, 0.12, 0.2],
            [0, 0, 0.1],
            ['task1_kitchenitem_ycb_031_spoon_*'],
            [1, 0, 0],
            [0, 0, 1],
            45,
            0
        ),

        # Tray A detectors
        create_object_detector(
            'any_in_traya_detector',
            'wrc_tray_1::link',
            [0.285, 0.37, 0.2],
            [0, 0, 0.1],
            ['task1_*']
        ),
        create_object_detector(
            'foods_in_traya_detector',
            'wrc_tray_1::link',
            [0.285, 0.37, 0.2],
            [0, 0, 0.1],
            ['task1_food_*']
        ),

        # Tray B detectors
        create_object_detector(
            'any_in_trayb_detector',
            'wrc_tray_2::link',
            [0.285, 0.37, 0.2],
            [0, 0, 0.1],
            ['task1_*']
        ),
        create_object_detector(
            'foods_in_trayb_detector',
            'wrc_tray_2::link',
            [0.285, 0.37, 0.2],
            [0, 0, 0.1],
            ['task1_food_*']
        ),

        # Bin A detectors
        create_object_detector(
            'any_in_bina_detector',
            'wrc_bin_green::link',
            [0.33, 0.38, 0.33],
            [0, 0, 0.165],
            ['task1_*']
        ),
        create_object_detector(
            'taskitems_in_bina_detector',
            'wrc_bin_green::link',
            [0.33, 0.38, 0.33],
            [0, 0, 0.165],
            ['task1_taskitem_*']
        ),

        # Bin B detectors
        create_object_detector(
            'any_in_binb_detector',
            'wrc_bin_black::link',
            [0.33, 0.38, 0.33],
            [0, 0, 0.165],
            ['task1_*']
        ),
        create_object_detector(
            'taskitems_in_binb_detector',
            'wrc_bin_black::link',
            [0.33, 0.38, 0.33],
            [0, 0, 0.165],
            ['task1_taskitem_*']
        ),

        # Room 2 detector
        create_object_detector(
            'hsrb_in_room2_detector',
            'wrc_frame::link',
            [3, 4, 3],
            [1.5, 0, 1],
            [robot_name]
        ),

        # Human Left Front detector
        create_object_detector(
            'hsrb_in_humanleftfront_detector',
            'person_standing::link',
            [1.2, 1.6, 3],
            [0, -0.5, 1],
            [robot_name]
        ),

        # Human Right Front detector
        create_object_detector(
            'hsrb_in_humanrightfront_detector',
            'person_standing_0::link',
            [1.2, 1.6, 3],
            [0, -0.5, 1],
            [robot_name]
        ),

        # Undesired contact detector
        Node(
            package='tmc_gazebo_task_evaluators',
            executable='undesired_contact_detector',
            name='undesired_contact_detector',
            output='screen',
            parameters=[{
                'target_model_name': robot_name,
                'except_model_names': ['wrc_ground_plane', 'wrc_tray_*', 'wrc_container_*', 'trofast_*', 'task1_*', 'wrc_bookshelf', 'task2_*']
            }]
        ),

        # WRS score counter
        Node(
            package='tmc_gazebo_task_evaluators',
            executable='wrs_score_counter',
            name='wrs_score_counter',
            output='screen',
            parameters=[{
                'seed': seed
            }]
        ),
    ])

    return ld
