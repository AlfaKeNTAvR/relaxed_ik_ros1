#! /usr/bin/env python

import csv
import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import utils
import transformations as T
import yaml

from relaxed_ik_ros1.msg import EEPoseGoals
from relaxed_ik_ros1.srv import activate_ik
from robot import Robot
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from timeit import default_timer as timer
from urdf_load import urdf_load
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

from kortex_driver.msg import JointAngles, JointAngle

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
env_settings_file_path = path_to_src + '/relaxed_ik_core/config/settings.yaml'

os.chdir(path_to_src + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(path_to_src + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
lib.solve.restype = Opt

def marker_feedback_cb(msg):
    pos_arr = (ctypes.c_double * 3)()
    quat_arr = (ctypes.c_double * 4)()
    pos_arr[0] = msg.pose.position.x
    pos_arr[1] = msg.pose.position.y
    pos_arr[2] = msg.pose.position.z
    quat_arr[0] = msg.pose.orientation.x
    quat_arr[1] = msg.pose.orientation.y
    quat_arr[2] = msg.pose.orientation.z
    quat_arr[3] = msg.pose.orientation.w
    # Call the rust callback function
    lib.dynamic_obstacle_cb(msg.marker_name, pos_arr, quat_arr)

def marker_update_cb(msg):
    # update dynamic collision obstacles in relaxed IK
    for pose_stamped in msg.poses:
        pos_arr = (ctypes.c_double * 3)()
        quat_arr = (ctypes.c_double * 4)()
        pos_arr[0] = pose_stamped.pose.position.x
        pos_arr[1] = pose_stamped.pose.position.y
        pos_arr[2] = pose_stamped.pose.position.z
        quat_arr[0] = pose_stamped.pose.orientation.x
        quat_arr[1] = pose_stamped.pose.orientation.y
        quat_arr[2] = pose_stamped.pose.orientation.z
        quat_arr[3] = pose_stamped.pose.orientation.w
        # Call the rust callback function
        lib.dynamic_obstacle_cb(pose_stamped.name, pos_arr, quat_arr)

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

# Turn relaxed IK calculation ON and OFF
def activate_ik_handler(req):
    global activateIk

    activateIk = req.command

    return True

def main(args=None):
    global activateIk

    rospy.init_node('relaxed_ik')

    # Flags
    activateIk = True

    # Load the infomation
    env_settings_file = open(env_settings_file_path, 'r')
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

    if 'loaded_robot' in env_settings:
        robot_info = env_settings['loaded_robot']
    else:
        raise NameError('Please define the relevant information of the robot!')

    info_file_name = robot_info['name']
    robot_name = info_file_name.split('_')[0]
    objective_mode = robot_info['objective_mode']
    print("\nRelaxedIK initialized!\nRobot: {}\nObjective mode: {}\n".format(robot_name, objective_mode))

    # Publishers
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=10)

    # Subscribers
    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)

    # Services
    activate_ik_srv = rospy.Service('/relaxed_ik/activate_ik', activate_ik, activate_ik_handler)

    cur_time = 0.0
    delta_time = 0.01
    step = 1 / 30.0

    global eepg

    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

    while eepg == None: continue

    rate = rospy.Rate(3000)
    speed_list = []
    while not rospy.is_shutdown():
        cur_time_msg = Float64()
        cur_time_msg.data = cur_time
        time_pub.publish(cur_time_msg)
        cur_time += delta_time * step

        pose_goals = eepg.ee_poses
        header = eepg.header
        pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
        quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

        for i in range(len(pose_goals)):
            p = pose_goals[i]
            pos_arr[3*i] = p.position.x
            pos_arr[3*i+1] = p.position.y
            pos_arr[3*i+2] = p.position.z

            quat_arr[4*i] = p.orientation.x
            quat_arr[4*i+1] = p.orientation.y
            quat_arr[4*i+2] = p.orientation.z
            quat_arr[4*i+3] = p.orientation.w

        start = timer()
        xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
        end = timer()
        speed = 1.0 / (end - start)
        #print("Speed: {}".format(speed))
        speed_list.append(speed)

        ja = JointAngles()
        ja_str = "["

        for i in range(xopt.length):
            joint_angle = JointAngle()
            joint_angle.joint_identifier = i
            joint_angle.value = (xopt.data[i])
            ja.joint_angles.append(joint_angle)
            ja_str += str(xopt.data[i])

            if i == xopt.length - 1:
                ja_str += "]"
            else:
                ja_str += ", "

        if activateIk == True:
            angles_pub.publish(ja)

        rate.sleep()

    print("Average speed: {} HZ".format(numpy.mean(speed_list)))
    print("Min speed: {} HZ".format(numpy.min(speed_list)))
    print("Max speed: {} HZ".format(numpy.max(speed_list)))

if __name__ == '__main__':
    main()
