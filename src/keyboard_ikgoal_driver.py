#!/usr/bin/python

import numpy as np
import readchar
import rospy
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
import kinova_positional_control.srv as posctrl_srv
from positional_control import utils


class KeyboardControl():

    def __init__(self):

        self.pos_stride = 0.015
        self.rot_stride = 0.055

        self.position_r = [0, 0, 0]
        self.rotation_r = [1, 0, 0, 0]

        self.position_l = [0, 0, 0]
        self.rotation_l = [1, 0, 0, 0]

        #Publishers
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=5,
        )
        self.quit_pub = rospy.Publisher(
            '/relaxed_ik/quit',
            Bool,
            queue_size=5,
        )

        # Service
        self.pid_vel_limit_srv = rospy.ServiceProxy(
            'pid_vel_limit',
            posctrl_srv.pid_vel_limit,
        )

        # Subscribers
        rospy.Subscriber(
            '/pid/motion_finished',
            Bool,
            self.pid_motion_finished_callback,
        )

    def pid_motion_finished_callback(self, data):
        """Callback function for the '/pid/motion_finished' topic that updates the
        'motionFinished' flag variable.
        """

        self.motionFinished = data.data

    def wait_motion_finished(self):
        """Block code execution until the 'motionFinished' flag is set or the ROS node
        is shutdown.
        """

        # Allow motion to start
        rospy.sleep(1)

        # Block code execution
        while not self.motionFinished and not rospy.is_shutdown():
            pass

    def initialization(self):
        """Initializes the system by homing the robotic arm, with velocity limit
        set to 20%, to a pre-specified initial configuration (starting_config in
        kortex_info.yaml) with relaxedIK.
        """

        # Set 20% velocity
        self.pid_vel_limit_srv(0.2)

        # Homing position
        print("\nHoming has started...\n")

        # Let the node initialized
        rospy.sleep(2)

        # Home using relaxedIK
        utils.relaxedik_publish([0, 0, 0], [1, 0, 0, 0])

        # Block until the motion is finished
        self.wait_motion_finished()

        print("\nHoming has finished.\n")

        self.pid_vel_limit_srv(1.0)

        print("\nSystem is ready.\n")

    def main(self):

        print("Pos R: {}, Pos L: {}".format(self.position_r, self.position_l))

        key = readchar.readkey()
        if key == 'w':
            self.position_r[0] += self.pos_stride
        elif key == 'x':
            self.position_r[0] -= self.pos_stride
        elif key == 'a':
            self.position_r[1] += self.pos_stride
        elif key == 'd':
            self.position_r[1] -= self.pos_stride
        elif key == 'q':
            self.position_r[2] += self.pos_stride
        elif key == 'z':
            self.position_r[2] -= self.pos_stride
        elif key == '1':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[0] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '2':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[0] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '3':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '4':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '5':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[2] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '6':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[2] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )

        elif key == 'i':
            self.position_l[0] += self.pos_stride  # left
        elif key == 'm':
            self.position_l[0] -= self.pos_stride
        elif key == 'j':
            self.position_l[1] += self.pos_stride
        elif key == 'l':
            self.position_l[1] -= self.pos_stride
        elif key == 'u':
            self.position_l[2] += self.pos_stride
        elif key == 'n':
            self.position_l[2] -= self.pos_stride
        elif key == '=':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[0] += self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '-':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[0] -= self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '0':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[1] += self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '9':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[1] -= self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '8':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[2] += self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == '7':
            euler = list(T.euler_from_quaternion(self.rotation_l))
            euler[2] -= self.rot_stride
            self.rotation_l = T.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )
        elif key == 'q':
            q = Bool()
            q.data = True
            self.quit_pub.publish(q)
        elif key == 'c':
            rospy.signal_shutdown()

        target_pos = np.array(
            [self.position_r[0], self.position_r[1], self.position_r[2]]
        )
        target_orientation = np.array(
            [
                self.rotation_r[0], self.rotation_r[1], self.rotation_r[2],
                self.rotation_r[3]
            ]
        )
        utils.relaxedik_publish(target_pos, target_orientation)

        q = Bool()
        q.data = False
        self.quit_pub.publish(q)


if __name__ == '__main__':

    rospy.init_node('keyboard_ikgoal_driver')
    system = KeyboardControl()

    system.initialization()

    while not rospy.is_shutdown():
        system.main()
