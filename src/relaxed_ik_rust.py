#!/usr/bin/env python
"""

"""

import ctypes
import os
import rospkg
import rospy
import yaml

from relaxed_ik_ros1.msg import (EEPoseGoals)
from kortex_driver.msg import (
    JointAngles,
    JointAngle,
)


class RelaxedIK:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        update_rate=750,
    ):
        """
        
        """

        class Opt(ctypes.Structure):
            _fields_ = [
                ('data', ctypes.POINTER(ctypes.c_double)),
                ('length', ctypes.c_int)
            ]

        # # Private constants:
        self.__PATH_TO_SRC = rospkg.RosPack().get_path('relaxed_ik_ros1')
        self.__ENV_SETTINGS_FILE_PATH = (
            f'{self.__PATH_TO_SRC}/relaxed_ik_core/config/settings.yaml'
        )
        os.chdir(f'{self.__PATH_TO_SRC}/relaxed_ik_core')
        self.__LIB = (
            ctypes.cdll.LoadLibrary(
                f'{self.__PATH_TO_SRC}/relaxed_ik_core/target/debug/librelaxed_ik_lib.so'
            )
        )
        self.__LIB.solve.restype = Opt

        env_settings_file = open(self.__ENV_SETTINGS_FILE_PATH, 'r')
        env_settings = yaml.load(
            env_settings_file,
            Loader=yaml.FullLoader,
        )

        if 'loaded_robot' in env_settings:
            robot_info = env_settings['loaded_robot']
        else:
            raise NameError(
                'Please define the relevant information of the robot!'
            )

        info_file_name = robot_info['name']

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.OBJECTIVE_MODE = robot_info['objective_mode']
        self.RATE = rospy.Rate(update_rate)

        # # Private variables:
        self.__ee_pose_goals = EEPoseGoals()

        # # Public variables:
        self.is_initialized = False

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__joint_angles_solutions = rospy.Publisher(
            'relaxed_ik/joint_angle_solutions',
            JointAngles,
            queue_size=1,
        )


        # # Topic subscriber:
        rospy.Subscriber(
            'relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            self.__ee_pose_goals_callback,
        )

        print(
            f'\n/{self.ROBOT_NAME}/relaxed_ik: waiting for the first goal pose...\n'
        )

    # # Service handlers:

    # # Topic callbacks:
    def __ee_pose_goals_callback(self, message):
        """

        """

        if not self.is_initialized:
            self.is_initialized = True

            print(
                f'\n/{self.ROBOT_NAME}/relaxed_ik: first goal pose was received!\n'
            )

            print(f'\n/{self.ROBOT_NAME}/relaxed_ik: ready.\n')

        self.__ee_pose_goals = message

    # # Private methods:

    # # Public methods:
    def main_loop(self):
        """
        
        """

        if not self.is_initialized:
            return

        ee_pose_goals = self.__ee_pose_goals.ee_poses
        xopt = None

        positions = (ctypes.c_double * (3 * len(ee_pose_goals)))()
        quaternions = (ctypes.c_double * (4 * len(ee_pose_goals)))()

        for i in range(len(ee_pose_goals)):
            p = ee_pose_goals[i]
            positions[3 * i] = p.position.x
            positions[3 * i + 1] = p.position.y
            positions[3 * i + 2] = p.position.z

            quaternions[4 * i] = p.orientation.x
            quaternions[4 * i + 1] = p.orientation.y
            quaternions[4 * i + 2] = p.orientation.z
            quaternions[4 * i + 3] = p.orientation.w

            # With each solve function call it returns the next trajectory point
            # in a form of joint angles solution.
            xopt = self.__LIB.solve(
                positions,
                len(positions),
                quaternions,
                len(quaternions),
            )

        joint_angles = JointAngles()

        if xopt:
            for i in range(xopt.length):
                joint_angle = JointAngle()
                joint_angle.joint_identifier = i
                joint_angle.value = xopt.data[i]
                joint_angles.joint_angles.append(joint_angle)

            self.__joint_angles_solutions.publish(joint_angles)

        self.RATE.sleep()

    def node_shutdown(self):
        """
        
        """

        print(f'\n/{self.ROBOT_NAME}/relaxed_ik: node is shutting down...')

        print(f'\n/{self.ROBOT_NAME}/relaxed_ik: nNode has shut down.')


def main():
    """
    
    """

    rospy.init_node('relaxed_ik')

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    print('realxed_ik', kinova_name)

    relaxed_ik_solver = RelaxedIK(
        robot_name=kinova_name,
        update_rate=1000,
    )

    rospy.on_shutdown(relaxed_ik_solver.node_shutdown)

    while not rospy.is_shutdown():
        relaxed_ik_solver.main_loop()


if __name__ == '__main__':
    main()