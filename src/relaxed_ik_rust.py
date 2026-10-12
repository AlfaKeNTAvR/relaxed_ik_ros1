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
        self.__LIB.initialize_relaxed_ik.argtypes = [
            ctypes.c_char_p, ctypes.c_int
        ]
        self.__LIB.initialize_relaxed_ik.restype = None

        settings_file = rospy.get_param(
            param_name=f'{rospy.get_name()}/settings_file',
            default='settings.yaml'
        )

        settings_file_bytes = settings_file.encode('utf-8')

        self.__LIB.initialize_relaxed_ik(
            ctypes.c_char_p(settings_file_bytes),
            1  # mode=1 for RelaxedIK
        )
        self.__LIB.solve.restype = Opt

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # Per joint speed ceiling used to rate limit the published solution.
        # Mirrors velocity_limits in
        # relaxed_ik_core/config/info_files/kortex_info.yaml, which is the
        # source of truth for the solver itself.
        self.JOINT_SPEED_LIMITS = (
            1.3963,
            1.3963,
            1.3963,
            1.3963,
            1.2218,
            1.2218,
            1.2218,
        )

        # A stalled main loop must not bank up an unbounded step allowance, so
        # the timestep used for rate limiting is capped no matter how long the
        # previous iteration actually took.
        self.MAXIMUM_TIMESTEP = 0.05

        # # Private variables:
        self.__ee_pose_goals = EEPoseGoals()
        self.__current_joint_positions = None

        # Previous published solution and its stamp, used as the origin that
        # the next solution is rate limited against.
        self.__last_solution = None
        self.__last_solution_stamp = None

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
        self.__target_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()

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

        # Normalize target quaternion:
        ee_pose_goals_quaternion = np.array(
            [
                message.ee_poses[0].orientation.w,
                message.ee_poses[0].orientation.x,
                message.ee_poses[0].orientation.y,
                message.ee_poses[0].orientation.z,
            ]
        )
        ee_pose_goals_quaternion = transformations.unit_vector(
            ee_pose_goals_quaternion
        )
        ee_pose_goals = message
        ee_pose_goals.ee_poses[0].orientation.w = ee_pose_goals_quaternion[0]
        ee_pose_goals.ee_poses[0].orientation.x = ee_pose_goals_quaternion[1]
        ee_pose_goals.ee_poses[0].orientation.y = ee_pose_goals_quaternion[2]
        ee_pose_goals.ee_poses[0].orientation.z = ee_pose_goals_quaternion[3]

        self.__ee_pose_goals = ee_pose_goals

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
            transform = self.__tf_buffer.lookup_transform(
                f'{self.ROBOT_NAME}/base_link',
                f'{self.ROBOT_NAME}/tool_frame',
                rospy.Time(),
            )

            # Protection against kinova/tool_frame not fully loaded initial
            # position (fully extended up).
            if (
                abs(transform.transform.translation.x) > 0.005
                and abs(transform.transform.translation.y - 0.025) > 0.005
                and abs(transform.transform.translation.z - 1.307) > 0.005
            ):
                self.__base_link_to_tool_frame = transform

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

        # The solver is now seeded at the measured configuration, so the rate
        # limiter origin has to move there too. Leaving a stale origin behind
        # would throttle the first solves after a reset for no reason.
        self.__last_solution = list(joint_state)
        self.__last_solution_stamp = rospy.Time.now()

        self.__initialize_ee_pose_goals()

        rospy.logwarn(
            (f'/{self.ROBOT_NAME}/relaxed_ik: '
             f'relaxed_ik was reset.')
        )

    def __rate_limit_solution(self, solution):
        """Caps how fast the published joint solution is allowed to move.

        relaxed_ik integrates from its own previous output, so it steps
        discontinuously whenever that output is projected onto the joint
        bounds. That happens on the first solve after a reset if the arm is
        parked outside the configured limits, and downstream those steps
        become large PID setpoint jumps.

        The whole joint step is scaled by a single factor rather than each
        joint being clipped on its own, so the direction of travel in joint
        space is preserved and only the speed along it changes.

        """

        now = rospy.Time.now()

        if (
            self.__last_solution is None
            or len(self.__last_solution) != len(solution)
        ):
            self.__last_solution = list(solution)
            self.__last_solution_stamp = now

            return list(solution)

        timestep = (now - self.__last_solution_stamp).to_sec()
        timestep = max(0.0, min(timestep, self.MAXIMUM_TIMESTEP))

        scale = 1.0

        for joint_index, value in enumerate(solution):
            maximum_delta = self.JOINT_SPEED_LIMITS[joint_index] * timestep
            delta = abs(value - self.__last_solution[joint_index])

            if delta > maximum_delta:
                scale = min(scale, maximum_delta / delta)

        limited_solution = [
            self.__last_solution[joint_index]
            + (value - self.__last_solution[joint_index]) * scale
            for joint_index, value in enumerate(solution)
        ]

        if scale < 1.0:
            rospy.logwarn_throttle(
                5,
                (
                    f'/{self.ROBOT_NAME}/relaxed_ik: '
                    f'solution rate limited to {round(scale, 3)} of the '
                    f'requested step.'
                ),
            )

        self.__last_solution = limited_solution
        self.__last_solution_stamp = now

        return limited_solution

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

    def __broadcast_target_pose(self):
        """
        
        """

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = (f'/{self.ROBOT_NAME}/base_link')
        transform_stamped.child_frame_id = (
            f'/{self.ROBOT_NAME}/relaxed_ik_target'
        )

        transform_stamped.transform.translation.x = (
            self.__ee_pose_goals.ee_poses[0].position.x
        )
        transform_stamped.transform.translation.y = (
            self.__ee_pose_goals.ee_poses[0].position.y
        )
        transform_stamped.transform.translation.z = (
            self.__ee_pose_goals.ee_poses[0].position.z
        )

        transform_stamped.transform.rotation.x = (
            self.__ee_pose_goals.ee_poses[0].orientation.x
        )
        transform_stamped.transform.rotation.y = (
            self.__ee_pose_goals.ee_poses[0].orientation.y
        )
        transform_stamped.transform.rotation.z = (
            self.__ee_pose_goals.ee_poses[0].orientation.z
        )
        transform_stamped.transform.rotation.w = (
            self.__ee_pose_goals.ee_poses[0].orientation.w
        )

        self.__target_pose_broadcaster.sendTransform(transform_stamped)

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
            solution = self.__rate_limit_solution(
                [xopt.data[i] for i in range(xopt.length)]
            )

            for i in range(len(solution)):
                joint_angle = JointAngle()
                joint_angle.joint_identifier = i
                joint_angle.value = solution[i]
                joint_angles.joint_angles.append(joint_angle)

            self.__joint_angles_solutions.publish(joint_angles)

        self.__broadcast_target_pose()

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
