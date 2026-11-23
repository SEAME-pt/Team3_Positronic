# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Publish the wrist camera of the real arm, on the Jetson, as Gazebo does.

Starts wrist_camera.py in the namespace wrist_camera, so the topics are the ones the Gazebo
bridge of gazebo.launch.py gives:

  /wrist_camera/image_raw              sensor_msgs/Image (bgr8)
  /wrist_camera/camera_info            sensor_msgs/CameraInfo
  /wrist_camera/image_raw/compressed   jpeg, for a laptop over Wi-Fi

The frame_id is camera_link, the camera link of positronic_real.urdf.xacro: the driver's
robot_state_publisher (driver.launch.py) publishes it under joint 6. The camera does not need
the driver and the driver does not need the camera: they start separately.

Launch arguments:
  device:=/dev/video0       the camera (v4l2-ctl --list-devices)
  width:=640 height:=480    size asked of the camera, before the rotation
  fps:=30                   frames per second asked of the camera
  rotation:=ccw90           none | cw90 | ccw90 | 180. ccw90 makes the image upright, gripper
                            at the bottom (#97); the published image is then height x width
  frame_id:=camera_link     header.frame_id of image and camera_info
  calibration_file:=<share>/config/wrist_camera.yaml   camera_calibration result (ost.yaml)
  jpeg_quality:=80          of image_raw/compressed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# The namespace of the Gazebo camera topics (gazebo.launch.py bridge). Not an argument: a
# different one breaks the RViz and module 2 configs this launch exists to share.
NAMESPACE = 'wrist_camera'


def generate_launch_description():
    """Return the camera launch: the arguments and the camera node."""
    arguments = [
        DeclareLaunchArgument('device', default_value='/dev/video0',
                              description='The camera device.'),
        DeclareLaunchArgument('width', default_value='640',
                              description='Width asked of the camera, before the rotation.'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Height asked of the camera, before the rotation.'),
        DeclareLaunchArgument('fps', default_value='30',
                              description='Frames per second asked of the camera.'),
        DeclareLaunchArgument('rotation', default_value='ccw90',
                              description='none, cw90, ccw90 or 180. ccw90: upright (#97).'),
        DeclareLaunchArgument('frame_id', default_value='camera_link',
                              description='Camera link of positronic_real.urdf.xacro.'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('positronic_bringup'), 'config', 'wrist_camera.yaml']),
            description='camera_calibration yaml; missing: camera_info has the size only.'),
        DeclareLaunchArgument('jpeg_quality', default_value='80',
                              description='JPEG quality of image_raw/compressed, 1 to 100.'),
    ]

    def integer(name):
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    camera = Node(
        package='positronic_bringup',
        executable='wrist_camera.py',
        name='wrist_camera',
        namespace=NAMESPACE,
        output='both',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'width': integer('width'),
            'height': integer('height'),
            'fps': integer('fps'),
            'rotation': LaunchConfiguration('rotation'),
            'frame_id': LaunchConfiguration('frame_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'jpeg_quality': integer('jpeg_quality'),
        }])

    return LaunchDescription(arguments + [camera])
