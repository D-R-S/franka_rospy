from panda_motion_generators import *


def grasp_lift_drop_test(controllers=None):
    if not controllers:
        controllers = All_Controllers()
    # Start at "Ready" pos w\ open gripper
    panda_joints = rospy.wait_for_message("/joint_states", JointState) # todo make sim order of states = real order of states
    arm_joints = np.around(panda_joints.position[0:-2], 3)
    if arm_joints[3] != -0.07:  # if not in home -> goto home
        print('Going to Ready Position')
        rdy_pose_joints = [1.55340, 0.00000, 0.00000, -1.48350, 0.00000, 1.86750, 0.78530]
        controllers.group.set_joint_value_target(rdy_pose_joints)
        plan = controllers.group.plan()
        if plan.joint_trajectory.joint_names != []:
            plan = trajectory_speed_up(plan, 0.5)
            controllers.group.execute(plan, wait=False)
            panda_wait_for_stop(True)
    hand_joints = np.around(panda_joints.position[-2:], 3)
    if hand_joints[0] != 0.04:  # if hand not open -> open hand
        print 'opening hand'
        real_gripper_position(0.08)

    # Get the robot to the image position (above the objects)
    panda_wait_for_stop(True)
    pose_x = 0.0
    pose_y = 0.5
    pose_z = 0.53 + 0.01
    TCP_wrt_world_matrix = compose_matrix(angles=[radians(180), radians(0), radians(0)],                                                      translate=[pose_x, pose_y, pose_z])
    image_pose = Pose()
    q = quaternion_from_matrix(TCP_wrt_world_matrix)
    image_pose.orientation.x = q[0]
    image_pose.orientation.y = q[1]
    image_pose.orientation.z = q[2]
    image_pose.orientation.w = q[3]
    image_pose.position.x = TCP_wrt_world_matrix[0, 3]
    image_pose.position.y = TCP_wrt_world_matrix[1, 3]
    image_pose.position.z = TCP_wrt_world_matrix[2, 3]

    controllers.group.set_pose_target(image_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)

    image_pose.position.z -= 0.49
    controllers.group.set_pose_target(image_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)

    a = 0
    waypoints = []
    image_pose.position.z -= 0.02  # grasping height
    waypoints.append(copy.deepcopy(image_pose))
    while a < 10:
        (plan, fraction) = controllers.group.compute_cartesian_path(waypoints=waypoints, eef_step=0.0001, jump_threshold=0)
        if fraction != 0 and plan.joint_trajectory.joint_names != [] and np.sum(np.abs(plan.joint_trajectory.points[-2].velocities)) > 0:
            if linear_trajectory_check(plan):  # Makes sure that the whole trajectory is in a linear movement
                plan = trajectory_speed_up(plan, speed=0.5)
                controllers.group.execute(plan, wait=False)
                panda_wait_for_stop(True)
                a += 20
                print 'executed lin. down trajectory'
        a += 1
        print a

    real_gripper_close_on_object(0.03115, force=40) # shaft1

    a = 0
    waypoints = []
    image_pose.position.z += 0.05
    waypoints.append(copy.deepcopy(image_pose))
    while a < 10:
        (plan, fraction) = controllers.group.compute_cartesian_path(waypoints=waypoints, eef_step=0.0001, jump_threshold=0)
        if fraction != 0 and plan.joint_trajectory.joint_names != [] and np.sum(np.abs(plan.joint_trajectory.points[-2].velocities)) > 0:
            if linear_trajectory_check(plan):  # Makes sure that the whole trajectory is in a linear movement
                plan = trajectory_speed_up(plan, speed=0.5)
                controllers.group.execute(plan, wait=False)
                panda_wait_for_stop(True)
                a += 20
                print 'executed lin. up trajectory'
        a += 1
        print a

    image_pose.position.z += 0.4
    controllers.group.set_pose_target(image_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)

    doit = True
    if doit:
        TCP_wrt_world_matrix = compose_matrix(angles=[radians(-90), radians(0), radians(92)],
                                                          translate=[pose_x, pose_y, pose_z])

        q = quaternion_from_matrix(TCP_wrt_world_matrix)
        image_pose.orientation.x = q[0]
        image_pose.orientation.y = q[1]
        image_pose.orientation.z = q[2]
        image_pose.orientation.w = q[3]

    image_pose.position.z -= 0.34
    controllers.group.set_pose_target(image_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)

def touch_floor():
    # find out robo - table contact w/o franka-reflex
    print 'about to trick the shit out of ya'
    controllers = None
    if not controllers:
        controllers = All_Controllers()

    # Get the robot to touch the floor
    panda_wait_for_stop(True)
    pose_x = 0.5
    pose_y = 0.00  # @ 0.3 z= 0.023  @ 0.4 z = 0.024

    pose_z = 0.01 + 0.03 + 0.001  # - 0.065 # tip tcp, table, spare

    TCP_wrt_world_matrix = compose_matrix(angles=[radians(180), radians(0), radians(0)],
                                                      translate=[pose_x, pose_y, pose_z])
    touch_pose = Pose()
    q = quaternion_from_matrix(TCP_wrt_world_matrix)
    touch_pose.orientation.x = q[0]
    touch_pose.orientation.y = q[1]
    touch_pose.orientation.z = q[2]
    touch_pose.orientation.w = q[3]
    touch_pose.position.x = TCP_wrt_world_matrix[0, 3]
    touch_pose.position.y = TCP_wrt_world_matrix[1, 3]
    touch_pose.position.z = TCP_wrt_world_matrix[2, 3]

    controllers.group.set_pose_target(touch_pose)
    plan = controllers.group.plan()
    if plan.joint_trajectory.joint_names != []:
        plan = trajectory_speed_up(plan, 0.5)
        controllers.group.execute(plan, wait=False)
        panda_wait_for_stop(True)
    pose = controllers.group.get_current_pose().pose
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qq = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    aa = euler_from_quaternion(qq)
    aa = [degrees(aa[0]),degrees(aa[1]),degrees(aa[2])]
    print 'EE link ', controllers.group.get_end_effector_link()
    print "EE Position: ", [round(x,5), round(y,5), round(z,5)], " EE Orientation:", [round(aa[0],5), round(aa[1],5), round(aa[2],5)] # where the tcp is
    joint_state = rospy.wait_for_message("/joint_states", JointState)
    jj = [round(degrees(joint_state.position[0]),5),round(degrees(joint_state.position[1]),5),round(degrees(joint_state.position[2]),5),
          round(degrees(joint_state.position[3]),5),round(degrees(joint_state.position[4]),5),round(degrees(joint_state.position[5]),5),
          round(degrees(joint_state.position[6]),5)]
    print "Joint States: ", jj
    print 'JS in rad', joint_state.position

if __name__ == "__main__":
    controllers = All_Controllers()
    touch_floor()
    #grasp_lift_drop_test()
