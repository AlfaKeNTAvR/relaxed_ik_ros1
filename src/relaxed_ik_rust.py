#!/usr/bin/env python
"""Implements relaxed_ik rust module for positional control.

TODO: Add detailed description.

Author (s):
    1. Lorena Genua (lorenagenua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    
"""

import ctypes
import os
import rospkg
import rospy
import tf2_ros
import transformations
import numpy as np

from std_msgs.msg import (Bool)
from std_srvs.srv import (Empty)

from geometry_msgs.msg import (
    TransformStamped,
    Pose,
)

from relaxed_ik_ros1.msg import (EEPoseGoals)
from kortex_driver.msg import (
    JointAngles,
    JointAngle,
)
from sensor_msgs.msg import (JointState)


class RelaxedIK:
    """
    
    """

    def __init__(
        self,
        robot_name,
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
        os.chdir(f'{self.__PATH_TO_SRC}/relaxed_ik_core')
        self.__LIB = (
            ctypes.cdll.LoadLibrary(
                f'{self.__PATH_TO_SRC}/relaxed_ik_core/target/debug/librelaxed_ik_lib.so'
            )
        )
        self.__LIB.solve.restype = Opt

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # # Private variables:
        self.__ee_pose_goals = EEPoseGoals()
        self.__current_joint_positions = None

        self.__base_link_to_tool_frame = None

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {}
        self.__dependency_status_topics = {}

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status['kortex_driver'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).

        self.__dependency_status_topics['kortex_driver'] = rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback/joint_state',
            JointState,
            self.__joint_positions_callback,
        )

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/relaxed_ik/reset',
            Empty,
            self.__reset_handler,
        )

        # # Service subscriber:

        # # Topic publisher:
        self.__joint_angles_solutions = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/joint_angle_solutions',
            JointAngles,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            self.__ee_pose_goals_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback/joint_state',
            JointState,
            self.__joint_positions_callback,
        )

        # # TF broadcaster:

        # # TF listener:
        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buffer)

    # # Service handlers:
    def __reset_handler(self, request):
        """

        """

        self.__reset(self.__current_joint_positions)

        return []

    # # Topic callbacks:
    def __ee_pose_goals_callback(self, message: EEPoseGoals):
        """

        """

        self.__ee_pose_goals = message

    def __joint_positions_callback(self, msg):
        """
        
        """

        if not self.__dependency_status['kortex_driver']:
            self.__dependency_status['kortex_driver'] = True

        self.__current_joint_positions = msg.position[0:7]

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.ROBOT_NAME}/relaxed_ik: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}/relaxed_ik:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # Extract tool_frame translation to base_link:
        try:
            self.__base_link_to_tool_frame = self.__tf_buffer.lookup_transform(
                f'{self.ROBOT_NAME}/base_link',
                f'{self.ROBOT_NAME}/tool_frame',
                rospy.Time(),
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

        # NOTE: Add more initialization criterea if needed.
        if (
            self.__dependency_initialized
            and self.__base_link_to_tool_frame != None
        ):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/relaxed_ik: ready.\033[0m',
                )

                self.__is_initialized = True
                self.__reset(self.__current_joint_positions)

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __reset(self, joint_state):
        """Resets Relaxed IK origin to Kinova current joint positions.

        """

        js_arr = (ctypes.c_double * len(joint_state))()

        for i in range(len(joint_state)):
            js_arr[i] = joint_state[i]

        self.__LIB.reset(js_arr, len(js_arr))

        self.__initialize_ee_pose_goals()

    def __initialize_ee_pose_goals(self):
        """
        
        """

        pose_message = Pose()
        pose_message.position.x = (
            self.__base_link_to_tool_frame.transform.translation.x
        )
        pose_message.position.y = (
            self.__base_link_to_tool_frame.transform.translation.y
        )
        pose_message.position.z = (
            self.__base_link_to_tool_frame.transform.translation.z
        )

        pose_message.orientation.w = (
            self.__base_link_to_tool_frame.transform.rotation.w
        )
        pose_message.orientation.x = (
            self.__base_link_to_tool_frame.transform.rotation.x
        )
        pose_message.orientation.y = (
            self.__base_link_to_tool_frame.transform.rotation.y
        )
        pose_message.orientation.z = (
            self.__base_link_to_tool_frame.transform.rotation.z
        )

        self.__ee_pose_goals = EEPoseGoals()
        self.__ee_pose_goals.ee_poses.append(pose_message)
        self.__ee_pose_goals.ee_poses.append(pose_message)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
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

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/relaxed_ik: node is shutting down...',
        )

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/relaxed_ik: node has shut down.',
        )


def main():
    """
    
    """

    rospy.init_node(
        'relaxed_ik',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    solver_rate = rospy.get_param(
        param_name=f'{rospy.get_name()}/solver_rate',
        default=1000,
    )

    relaxed_ik_solver = RelaxedIK(robot_name=kinova_name,)

    rospy.on_shutdown(relaxed_ik_solver.node_shutdown)
    node_rate = rospy.Rate(solver_rate)

    while not rospy.is_shutdown():
        relaxed_ik_solver.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
