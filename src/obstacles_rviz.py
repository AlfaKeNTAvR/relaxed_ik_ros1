#! /usr/bin/env python

import rospy
import rospkg
import ctypes
import os
import yaml
import transformations
import sys

from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool

# === Get ROS parameter for arm side ===
rospy.init_node('rviz_viewer', anonymous=True)
arm_side = rospy.get_param('~arm_side', 'right_arm')  # Default to right_arm

# Set color based on arm
if arm_side == 'left_arm':
    color = {'r': 0.0, 'g': 1.0, 'b': 0.0}  # Green
elif arm_side == 'right_arm':
    color = {'r': 1.0, 'g': 0.0, 'b': 0.0}  # Red
else:
    rospy.logerr("Invalid arm_side parameter! Use 'left_arm' or 'right_arm'.")
    sys.exit(1)

# === Load paths and settings ===
path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')

env_settings_file_path = os.path.join(
    path_to_src, 'relaxed_ik_core/config', f'{arm_side}_settings.yaml'
)
with open(env_settings_file_path, 'r') as env_settings_file:
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

if 'loaded_robot' in env_settings:
    info_file_name = env_settings['loaded_robot']['name']
else:
    raise NameError('Please define the loaded robot!')

info_file_path = f'{path_to_src}/relaxed_ik_core/config/info_files/{info_file_name}'
with open(info_file_path, 'r') as info_file:
    info = yaml.load(info_file, Loader=yaml.FullLoader)

# === Setup URDF in RVIZ ===
urdf_file_path = f'{path_to_src}/relaxed_ik_core/config/urdfs/{info["urdf_file_name"]}'
with open(urdf_file_path, 'r') as urdf_file:
    urdf_string = urdf_file.read()

rospy.set_param(f'{arm_side}/robot_description', urdf_string)

os.chdir(f'{path_to_src}/relaxed_ik_core')
lib = ctypes.cdll.LoadLibrary(
    f'{path_to_src}/relaxed_ik_core/target/debug/librelaxed_ik_lib.so'
)

static_marker_topic = f'/{arm_side}/static_marker_array'
dynamic_marker_topic = f'/{arm_side}/dynamic_marker_array'

static_marker_array_pub = rospy.Publisher(
    static_marker_topic, MarkerArray, queue_size=10
)
dynamic_marker_array_pub = rospy.Publisher(
    dynamic_marker_topic, MarkerArray, queue_size=1
)

static_obstacle_markers = []
dynamic_obstacle_markers = []

for cuboid in env_settings['obstacles']['cuboids']:
    marker = Marker()
    marker.ns = cuboid['name']
    marker.header.frame_id = f'{arm_side}/base_link'
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = cuboid['scale'][0]
    marker.scale.y = cuboid['scale'][1]
    marker.scale.z = cuboid['scale'][2]
    marker.color.a = 1.0
    marker.color.r = color['r']
    marker.color.g = color['g']
    marker.color.b = color['b']

    marker.pose.position.x = cuboid['translation'][0]
    marker.pose.position.y = cuboid['translation'][1]
    marker.pose.position.z = cuboid['translation'][2]

    rotation_quaternion = transformations.quaternion_from_euler(
        cuboid['rotation'][0],
        cuboid['rotation'][1],
        cuboid['rotation'][2],
    )

    marker.pose.orientation.w = rotation_quaternion[0]
    marker.pose.orientation.x = rotation_quaternion[1]
    marker.pose.orientation.y = rotation_quaternion[2]
    marker.pose.orientation.z = rotation_quaternion[3]

    if cuboid['animation'] == 'static':
        static_obstacle_markers.append(marker)
    elif cuboid['animation'] == 'interactive':
        dynamic_obstacle_markers.append(marker)

static_marker_array = MarkerArray()
static_marker_array.markers = static_obstacle_markers

dynamic_marker_array = MarkerArray()
dynamic_marker_array.markers = dynamic_obstacle_markers


def move_obstacle_handler(request):
    global dynamic_marker_array

    position = [0.5, 0.0, 0.5] if request.data else [0.5, 0.0, 0.25]
    orientation = [1.0, 0.0, 0.0, 0.0]

    print(position)

    lib.update_dynamic_obstacle('test', position, orientation)

    m = dynamic_marker_array.markers[0]
    m.pose.position.x, m.pose.position.y, m.pose.position.z = position
    m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = orientation

    return True, ''


rospy.Service('move_obstacle', SetBool, move_obstacle_handler)

# === Main publish loop ===
while not rospy.is_shutdown():
    static_marker_array_pub.publish(static_marker_array)
    dynamic_marker_array_pub.publish(dynamic_marker_array)
    rospy.rostime.wallsleep(1.0)
