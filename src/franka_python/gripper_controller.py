#!/usr/bin/env python
import rospy
import time
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import RobotTrajectory
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal


def gripper_wait_for_stop():
    # Wait for the gripper to Stop
    time.sleep(0.1)
    controller_state = rospy.wait_for_message("/EffortJointInterface_trajectory_gripper/state", JointTrajectoryControllerState)
    while controller_state.desired.velocities!=(0,0):
        controller_state = rospy.wait_for_message("/EffortJointInterface_trajectory_gripper/state", JointTrajectoryControllerState)
        #time.sleep(0.1)
    return

def gripper_speed_up(trajectory, speed=1.0):
    new_trajectory = RobotTrajectory()
    new_trajectory.joint_trajectory = trajectory.joint_trajectory
    n_points = len(trajectory.joint_trajectory.points)
    for i in range(n_points):
        new_trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start / speed
        velocities = (new_trajectory.joint_trajectory.points[i].velocities[0] * speed, new_trajectory.joint_trajectory.points[i].velocities[1] * speed)
        accelerations = (new_trajectory.joint_trajectory.points[i].accelerations[0] * speed, new_trajectory.joint_trajectory.points[i].accelerations[1] * speed)
        new_trajectory.joint_trajectory.points[i].velocities = velocities
        new_trajectory.joint_trajectory.points[i].accelerations = accelerations
    return new_trajectory

def real_gripper_position(object_width=0.08):
    #print '123'
    action_address = '/franka_gripper/move'
    gripper_move_client = actionlib.SimpleActionClient(action_address, MoveAction)
    #print('Waiting for ArmPoseAction server...')
    gripper_move_client.wait_for_server()
    #print('ArmPoseAction Server Connected')
    goal = MoveGoal()
    goal.width = object_width
    goal.speed = 0.4
    gripper_move_client.send_goal(goal)
    #gripper_move_client.wait_for_result()

def real_gripper_close_on_object(object_width=0.0, force=40):
    action_address = '/franka_gripper/grasp'
    gripper_grasp_client = actionlib.SimpleActionClient(action_address, GraspAction)
    gripper_grasp_client.wait_for_server()
    goal = GraspGoal()
    goal.width = object_width
    goal.epsilon.inner = 0.009
    goal.epsilon.outer = 0.009
    goal.speed = 0.1
    goal.force = force
    gripper_grasp_client.send_goal(goal)
    gripper_grasp_client.wait_for_result()


if __name__ == "__main__":
    real_gripper_position()






