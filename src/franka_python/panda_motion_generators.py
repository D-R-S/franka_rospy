#!/usr/bin/env python

import copy
from math import *
from tf.transformations import compose_matrix, quaternion_from_euler, quaternion_from_matrix, inverse_matrix, euler_from_quaternion
#from sensor_msgs.msg import JointState
from gripper_controller import *
from panda_initialization_functions import *
#from geometry_msgs.msg import WrenchStamped


def joint_pose_motion(controllers=None, joint_pose=None):
    if joint_pose is None:
        joint_pose = [1.5534, 0.0, 0.0, -1.4835, 0.0, 1.8675, 0.7853]
    if controllers is None:
        controllers = All_Controllers()
    controllers.group.set_joint_value_target(joint_pose)
    plan1 = controllers.group.plan()
    if plan1.joint_trajectory.joint_names != []:
        plan1 = trajectory_speed_up(plan1, 0.5)
        controllers.group.execute(plan1, wait=False)
        panda_wait_for_stop()

def carthesian_pose_motion(controllers=None):
    if not controllers:
        controllers = All_Controllers()
    pose_x = 0.5
    pose_y = 0.0
    pose_z = 0.15
    TCP_wrt_world_matrix = compose_matrix(angles=[radians(180), radians(0), radians(0)],
                                                      translate=[pose_x, pose_y, pose_z])

    q = quaternion_from_matrix(TCP_wrt_world_matrix)
    touch_pose = Pose()
    touch_pose.orientation.x = q[0]
    touch_pose.orientation.y = q[1]
    touch_pose.orientation.z = q[2]
    touch_pose.orientation.w = q[3]
    touch_pose.position.x = pose_x
    touch_pose.position.y = pose_y
    touch_pose.position.z = pose_z

    controllers.group.set_pose_target(touch_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)
    # print ee pose and joint pose
    ee = controllers.group.get_end_effector_link()
    print('ee link:', ee)
    pose = controllers.group.get_current_pose()
    print(pose)

if __name__ == "__main__":
    controllers = All_Controllers()
    joint_pose_motion(controllers)
    carthesian_pose_motion(controllers)
    real_gripper_position(0.07)
