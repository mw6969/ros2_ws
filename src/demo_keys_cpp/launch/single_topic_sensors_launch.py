# Copyright 2025 Proyectos y Sistemas de Mantenimiento SL (eProsima).
# Copyright 2025 Open Source Robotics Foundation, Inc.
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
from launch_ros.actions import Node

def generate_launch_description():

  sensor_nodes = []

  for id in range(1, 11):

    sensor_node = Node(
      package="demo_keys_cpp",
      executable="single_topic_sensor",
      parameters=[
                {"id": id}
            ])

    sensor_nodes.append(sensor_node)

  return LaunchDescription(sensor_nodes)
