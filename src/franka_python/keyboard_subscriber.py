#!/usr/bin/env python

import threading
from sensor_msgs.msg import JointState
from panda_initialization_functions import *

def keyboard_pose_subscriber():
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_planning_time(4.0)
    try:
        while True:
            pose = rospy.wait_for_message("/keyboard_pose", PoseStamped)
            if pose.pose.position.x == 100:
                break
            group.set_pose_target(pose)
            plan = group.plan()
            plan = trajectory_speed_up(plan, 4) # was 4
            group.execute(plan, wait=False)
            panda_wait_for_stop()
    except:
        print("keyboard_subscriber: Finished subscribing keyboard pose commands")


def keyboard_joints_subscriber():
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_planning_time(4.0)
    try:
        while True:
            panda_joints = rospy.wait_for_message("/keyboard_joints", JointState)
            print('gotit')
            if panda_joints.position[2:] == (9, 9, 9, 9, 9, 9, 9):
                break
            group.set_joint_value_target(panda_joints.position) #read only arm pos
            plan = group.plan()
            plan = trajectory_speed_up(plan, 4) # was 4
            group.execute(plan, wait=False)
            panda_wait_for_stop()
    except:
        print("keyboard_subscriber: Finished subscribing keyboard joints commands")



def keyboard_subscribers():
    # Start subscribers as threads to run in the background

    keyboard_pose_subscriber_thread = threading.Thread(target=keyboard_pose_subscriber)
    keyboard_pose_subscriber_thread.start()

    keyboard_joints_subscriber_thread = threading.Thread(target=keyboard_joints_subscriber)
    keyboard_joints_subscriber_thread.start()


def keyboard_subscribers_with_initialization():
    # Start subscribers as threads to run in the background.

    initialize_free_environment()

    keyboard_pose_subscriber_thread = threading.Thread(target=keyboard_pose_subscriber)
    keyboard_pose_subscriber_thread.start()

    keyboard_joints_subscriber_thread = threading.Thread(target=keyboard_joints_subscriber)
    keyboard_joints_subscriber_thread.start()


if __name__ == "__main__":
    keyboard_subscribers_with_initialization()

