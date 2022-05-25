import sys
import rospy
import moveit_commander
import numpy as np
import time
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import RobotTrajectory


class All_Controllers(object):
    # initialize all real and simulation controllers to avoid initialization at every function separately
    def __init__(self):
        # type: () -> object
        # sim arm manipulation controller
        initialize_free_environment()
        #initialize_constrained_environment()
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_planning_time(15)  # was 0.3
        self.group.set_end_effector_link('panda_hand')
        self.sim_gripper = moveit_commander.MoveGroupCommander("hand")
        try:
            self.real_gripper = None

        except:
            print('real gripper is not connected')
            self.real_gripper = None

def trajectory_speed_up(trajectory, speed=1.0):
    # This function takes a moveit planned trajectory and returns a new trajectory which is 'speed' times faster.
    # 10 is the maximum speed up value without printing the "PATH_TOLERANCE_VIOLATED" warning from the move_group node (in the terminal).
    new_trajectory = RobotTrajectory()
    new_trajectory.joint_trajectory = trajectory.joint_trajectory
    n_points = len(trajectory.joint_trajectory.points)

    for i in range(n_points):
        new_trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start / speed
        velocities = (new_trajectory.joint_trajectory.points[i].velocities[0] * speed, new_trajectory.joint_trajectory.points[i].velocities[1] * speed,
                      new_trajectory.joint_trajectory.points[i].velocities[2] * speed, new_trajectory.joint_trajectory.points[i].velocities[3] * speed,
                      new_trajectory.joint_trajectory.points[i].velocities[4] * speed, new_trajectory.joint_trajectory.points[i].velocities[5] * speed,
                      new_trajectory.joint_trajectory.points[i].velocities[6] * speed)
        accelerations = (new_trajectory.joint_trajectory.points[i].accelerations[0] * speed, new_trajectory.joint_trajectory.points[i].accelerations[1] * speed,
                         new_trajectory.joint_trajectory.points[i].accelerations[2] * speed, new_trajectory.joint_trajectory.points[i].accelerations[3] * speed,
                         new_trajectory.joint_trajectory.points[i].accelerations[4] * speed, new_trajectory.joint_trajectory.points[i].accelerations[5] * speed,
                         new_trajectory.joint_trajectory.points[i].accelerations[6] * speed)
        new_trajectory.joint_trajectory.points[i].velocities = velocities
        new_trajectory.joint_trajectory.points[i].accelerations = accelerations

    return new_trajectory

def linear_trajectory_check(trajectory):
    # This function makes sure that the whole trajectory is in a linear movement
    points = range(len(trajectory.joint_trajectory.points)-1)
    for i in points[::-1]:
        for j in range(7):
            if abs(trajectory.joint_trajectory.points[i+1].positions[j] - trajectory.joint_trajectory.points[i].positions[j]) > 1:
                print("Found a difference of " + str(abs(trajectory.joint_trajectory.points[i+1].positions[j] - trajectory.joint_trajectory.points[i].positions[j])) +
                      " between points " + str(i) + " and " + str(i+1) + " in panda_joint" + str(j+1))
                print('The trajectory is not in a linear movement')
                return False
    return True


def panda_wait_for_stop(real=True):
    # This function makes python wait until the controller finishes the current trajectory.
    # The difference from using 'wait=True' inside the 'execute' function is
    # that the thread 'gripper_adjust_joints' can work while python is waiting for the trajectory to finish.
    if real:
        time.sleep(0.1)
        controller_state = rospy.wait_for_message("/position_joint_trajectory_controller/state", JointTrajectoryControllerState)
        while controller_state.desired.velocities!=(0,0,0,0,0,0,0):
            controller_state = rospy.wait_for_message("/position_joint_trajectory_controller/state", JointTrajectoryControllerState)
            #print(controller_state.actual.velocities)
            time.sleep(0.1)
        return
    else:
        time.sleep(0.1)
        controller_state = rospy.wait_for_message("/PositionJointInterface_trajectory_controller/state", JointTrajectoryControllerState)
        while controller_state.desired.velocities!=(0,0,0,0,0,0,0):
            controller_state = rospy.wait_for_message("/PositionJointInterface_trajectory_controller/state", JointTrajectoryControllerState)
            #print(controller_state.actual.velocities)
            time.sleep(0.1)
        return

def initialize_constrained_environment():
    # Initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True, log_level=rospy.WARN) # Use this line if you don't want to print INFO messages
    # rospy.init_node('move_group_python_interface', anonymous=True) # Use this line if you want to print INFO messages
    # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Setting the limitations of the environment (surface and imaginary walls)
    if scene.get_known_object_names()==[]:
        print("Adding limitations to the environment")

        rospy.sleep(0.3)
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        # Right side of the table
        p.pose.position.x = 0.3775
        p.pose.position.y = 0.491
        p.pose.position.z = 0.00
        scene.add_box("right_surface", p, (0.435, 1.32, 0.00))
        # Left side of the table
        p.pose.position.x = -0.3865
        p.pose.position.y = 0.491
        p.pose.position.z = 0.00
        scene.add_box("left_surface", p, (0.435, 1.32, 0.000))
        # Front side of the table
        p.pose.position.x = -0.0045
        p.pose.position.y = 0.655
        p.pose.position.z = 0.00
        scene.add_box("front_surface", p, (0.329, 0.99, 0.00))
        # Rear side of the table
        p.pose.position.x = -0.0045
        p.pose.position.y = -0.29
        p.pose.position.z = 0.00
        scene.add_box("rear_surface", p, (1.20, 0.24, 0.00))
        if False: # Change to True to add imaginary walls
            # Right side of imaginary wall
            p.pose.position.x = 1.045
            p.pose.position.y = 0.35
            p.pose.position.z = 0.25
            scene.add_box("right_wall", p, (0.01, 2.84, 2.5))
            # Left side of imaginary wall
            p.pose.position.x = -0.755
            p.pose.position.y = 0.35
            p.pose.position.z = 0.25
            scene.add_box("left_wall", p, (0.01, 2.84, 2.5))
            # Front side of imaginary wall
            p.pose.position.x = 0.145
            p.pose.position.y = 1.775
            p.pose.position.z = 0.25
            scene.add_box("front_wall", p, (1.79, 0.01, 2.5))
            # Rear side of imaginary wall
            p.pose.position.x = 0.145
            p.pose.position.y = -0.55
            p.pose.position.z = 0.25
            scene.add_box("rear_wall", p, (1.79, 0.01, 2.5))
            while np.shape(scene.get_known_object_names()) != (8,):
                rospy.sleep(0.1)
        # else:
        #     while np.shape(scene.get_known_object_names())!=(4,):
        #         rospy.sleep(0.1)
        #
def initialize_free_environment():
    rospy.init_node('move_group_python_interface', anonymous=True, log_level=rospy.WARN) # Use this line if you don't want to print INFO messages



def check_pose_validation(pose):
    # This function checks that the target pose for the gripper is not outside of the environment's borders
    valid = True
    # z direction: high limit - maximum reach, low limit - avoiding the surface
    if pose.position.z > 1.307 or pose.position.z < 0.01: # TODO if used this results in not beeing able to go closer to table (old 0.079) -> check usage
        valid = False
    # y direction: high limit - avoiding the left "imaginary wall", low limit - maximum reach
    elif pose.position.y > 1 or pose.position.y < -1:
        valid = False
    # x direction: high limit - maximum reach, low limit - maximum reach
    elif pose.position.x > 0.97 or pose.position.x < -0.67:
        valid = False
    if valid == False:
        print("Target pose is not valid.")
    return valid
