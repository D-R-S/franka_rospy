#!/usr/bin/env python
import select, termios, tty
from math import radians, degrees, pi
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from gripper_controller import *
from panda_initialization_functions import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# def float_round(num, places = 0, direction = floor):
#     return direction(num * (10**places)) / float(10**places)

msg = """
Move the robot using the keyboard!
-------------------------------------------------
Changing Cartesian Position:        Position Step Size:
+x: a          -x: y                increase 0.01m: f
+y: s          -y: x                decrease 0.01m: v
+z: d          -z: c

Changing Cartesian Orientation:     Orientation Step Size:
+roll: g       -roll: b             increase 5 degrees: k
+pitch: h      -pitch: n            decrease 5 degrees: ,
+yaw: j        -yaw: m

Changing Joint Position:            Orientation Step Size:
+joint_1: 1    -joint_1: q          increase 5 degrees: 8
+joint_2: 2    -joint_2: w          decrease 5 degrees: i
+joint_3: 3    -joint_3: e
+joint_4: 4    -joint_4: r
+joint_5: 5    -joint_5: t
+joint_6: 6    -joint_6: z
+joint_7: 7    -joint_7: u

Gripper Control:                    Gripping Position 1: p
open gripper: +                     Gripping Position 2: o
close gripper: #                    Gripping Position 3: 9
gripper pos: -                      Back to Home Position: 0
                                    Print Pose and JointState: l
CTRL-C to quit
"""

positionBindings = {
    'a': (1, 0, 0, 0, 0, 0),
    'y': (-1, 0, 0, 0, 0, 0),
    's': (0, 1, 0, 0, 0, 0),
    'x': (0, -1, 0, 0, 0, 0),
    'd': (0, 0, 1, 0, 0, 0),
    'c': (0, 0, -1, 0, 0, 0),
    'g': (0, 0, 0, 1, 0, 0),
    'b': (0, 0, 0, -1, 0, 0),
    'h': (0, 0, 0, 0, 1, 0),
    'n': (0, 0, 0, 0, -1, 0),
    'j': (0, 0, 0, 0, 0, 1),
    'm': (0, 0, 0, 0, 0, -1),
}

jointsBindings = {
    '1': (1, 0, 0, 0, 0, 0, 0),
    'q': (-1, 0, 0, 0, 0, 0, 0),
    '2': (0, 1, 0, 0, 0, 0, 0),
    'w': (0, -1, 0, 0, 0, 0, 0),
    '3': (0, 0, 1, 0, 0, 0, 0),
    'e': (0, 0, -1, 0, 0, 0, 0),
    '4': (0, 0, 0, 1, 0, 0, 0),
    'r': (0, 0, 0, -1, 0, 0, 0),
    '5': (0, 0, 0, 0, 1, 0, 0),
    't': (0, 0, 0, 0, -1, 0, 0),
    '6': (0, 0, 0, 0, 0, 1, 0),
    'z': (0, 0, 0, 0, 0, -1, 0),
    '7': (0, 0, 0, 0, 0, 0, 1),
    'u': (0, 0, 0, 0, 0, 0, -1),
}

stepSizeBindings = {
    'f': (0.01, 0, 0),
    'v': (-0.01, 0, 0),
    'k': (0, radians(5), 0),
    ',': (0, radians(-5), 0),
    '8': (0, 0, radians(5)),
    'i': (0, 0, radians(-5)),
}

gripperCommandBindings = {
'+': ("open"),
'#': ("close"),
'-': ("position"),
}

definedPositionBinding = {
'0': ('0'),
'p': ('p'),
'o': ('o'),
'9': ('9'),
'l': ('l'),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_current_step_sizes(pos_step_size, angle_step_size, joint_step_size):
    print "Currently Step Sizes:    Position %s m    Orientation %s degrees    Joint %s degrees " % (pos_step_size, int(round(degrees(angle_step_size))),  int(round(degrees(joint_step_size))))


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    controllers = All_Controllers()
    pose_publisher = rospy.Publisher('keyboard_pose', PoseStamped, queue_size=10)
    joints_publisher = rospy.Publisher('keyboard_joints', JointState, queue_size=10)
    gripper_publisher = rospy.Publisher('keyboard_gripper_control', Bool, queue_size=1) #original (usage with gripper_controller.py)
    gripper_publisher2 = rospy.Publisher('keyboard_gripper_control2', JointState, queue_size=10) #my trials to control the gripper like the arm...
    # rospy.init_node('keyboard_publisher', anonymous=True)

    pos_step_size = 0.05
    angle_step_size = radians(15)
    joint_step_size = radians(15)
    status = 0

    pose = controllers.group.get_current_pose()
    print(pose)
    panda_joints = rospy.wait_for_message("/joint_states", JointState)
    print(panda_joints)
    try:
        print msg
        print_current_step_sizes(pos_step_size, angle_step_size, joint_step_size)
        while True:
            key = getKey()
            if key in positionBindings.keys():
                pose = controllers.group.get_current_pose()
                # Adding the position change in xyz without crossing the limits of each axis.
                pose.pose.position.x = max(-0.855, min(0.855, (pose.pose.position.x + positionBindings[key][0] * pos_step_size)))
                pose.pose.position.y = max(-0.855, min(0.855, (pose.pose.position.y + positionBindings[key][1] * pos_step_size)))
                pose.pose.position.z = max(-0.36, min(1.19, (pose.pose.position.z + positionBindings[key][2] * pos_step_size)))
                euler_angles = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
                roll = euler_angles[0]
                pitch = euler_angles[1]
                yaw = euler_angles[2]
                roll += positionBindings[key][3] * angle_step_size
                pitch += positionBindings[key][4] * angle_step_size
                yaw += positionBindings[key][5] * angle_step_size
                pose_quaternion = quaternion_from_euler(roll, pitch, yaw)
                pose.pose.orientation.x = pose_quaternion[0]
                pose.pose.orientation.y = pose_quaternion[1]
                pose.pose.orientation.z = pose_quaternion[2]
                pose.pose.orientation.w = pose_quaternion[3]

                controllers.group.set_pose_target(pose)
                plan1 = controllers.group.plan()
                print(plan1.joint_trajectory.joint_names)
                if plan1.joint_trajectory.joint_names != []:
                    pose_publisher.publish(pose)
                else:
                    if (status == 14):
                        print msg
                    status = (status + 1) % 15
                    print("keyboard_publisher: Panda Arms' plan is not valid1")


            elif key in jointsBindings.keys():
                panda_joints = rospy.wait_for_message("/joint_states", JointState)
                # Adding the angle change in all joints without crossing the limits of each joint.
                finger_joints1 = panda_joints.position[0] + 0.01
                finger_joints2 = finger_joints1 #panda_joints.position[1] - 0.01  not working because finger joint 2 is set to mimic finger joint 1 in hand.xacro
                panda_joints1 = max((-166 * pi / 180), min((166 * pi / 180), (panda_joints.position[2] + jointsBindings[key][0] * joint_step_size)))
                panda_joints2 = max((-101 * pi / 180), min((101 * pi / 180), (panda_joints.position[3] + jointsBindings[key][1] * joint_step_size)))
                panda_joints3 = max((-166 * pi / 180), min((166 * pi / 180), (panda_joints.position[4] + jointsBindings[key][2] * joint_step_size)))
                panda_joints4 = max((-176 * pi / 180), min((-4 * pi / 180), (panda_joints.position[5] + jointsBindings[key][3] * joint_step_size)))
                panda_joints5 = max((-166 * pi / 180), min((166 * pi / 180), (panda_joints.position[6] + jointsBindings[key][4] * joint_step_size)))
                panda_joints6 = max((-1 * pi / 180), min((215 * pi / 180), (panda_joints.position[7] + jointsBindings[key][5] * joint_step_size)))
                panda_joints7 = max((-166 * pi / 180), min((166 * pi / 180), (panda_joints.position[8] + jointsBindings[key][6] * joint_step_size)))
                joint_target = (finger_joints1, finger_joints2, panda_joints1, panda_joints2, panda_joints3, panda_joints4, panda_joints5, panda_joints6, panda_joints7)
                controllers.group.set_joint_value_target(joint_target[2:])
                plan1 = controllers.group.plan()
                if plan1.joint_trajectory.joint_names != []:
                    panda_joints.position = (panda_joints1, panda_joints2, panda_joints3, panda_joints4, panda_joints5, panda_joints6, panda_joints7) #finger_joints1, finger_joints2,
                    joints_publisher.publish(panda_joints)
                else:
                    if (status == 14):
                        print msg
                    status = (status + 1) % 15
                    print("keyboard_publisher: Panda Arms' plan is not valid2")


            elif key in stepSizeBindings.keys():
                if pos_step_size > 0.01:
                    pos_step_size = max(pos_step_size + stepSizeBindings[key][0], 0.01)
                elif pos_step_size > 0.001:
                    pos_step_size = max(pos_step_size + stepSizeBindings[key][0]/10, 0.001)
                else:
                    pos_step_size = max(pos_step_size + stepSizeBindings[key][0] / 100, 0.0001)
                if angle_step_size > radians(5):
                    angle_step_size = max(angle_step_size + stepSizeBindings[key][1], radians(5))
                else:
                    angle_step_size = max(angle_step_size + stepSizeBindings[key][1]/10, radians(5)/10)
                if joint_step_size > radians(5):
                    joint_step_size = max(joint_step_size + stepSizeBindings[key][2], radians(5))
                else:
                    joint_step_size = max(joint_step_size + stepSizeBindings[key][2]/10, radians(5)/10)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
                print_current_step_sizes(pos_step_size, angle_step_size, joint_step_size)

            elif key in gripperCommandBindings.keys():
                if gripperCommandBindings[key]=="close":
                    real_gripper_position(0.0)
                elif gripperCommandBindings[key]=="open":
                    real_gripper_position(0.08)

            elif key in definedPositionBinding.keys():

                if key=='0': # Home by joint control
                    panda_joints = rospy.wait_for_message("/joint_states", JointState)
                    panda_joints.position = (0.0, 0.0, 0.0, -0.1, 0.0, 3.1416, 0.0)
                    joints_publisher.publish(panda_joints)

                elif key=='9': # Home by position control
                    q = quaternion_from_euler(ai=radians(0.0354106), aj=radians(5.7239232), ak=radians(-134.5823471))
                    print(q)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    pose.pose.position.x = -0.03816
                    pose.pose.position.y = 0.00034
                    pose.pose.position.z = 1.15456# 0.18
                    pose_publisher.publish(pose)

                elif key == 'o':  # some test position
                    q = quaternion_from_euler(ai=radians(-90), aj=radians(0), ak=radians(-180))
                    pose.pose.orientation.x = 0.707800813341 # q[0]
                    pose.pose.orientation.y = 0.705529079392 # q[1]
                    pose.pose.orientation.z = -0.0262860127754 # q[2]
                    pose.pose.orientation.w = 0.0235748234061 # q[3]
                    pose.pose.position.x =  0.00263965839473
                    pose.pose.position.y = 0.768259834575
                    pose.pose.position.z = 0.375867027687
                    pose_publisher.publish(pose)

                elif key == 'p': # Position to take an image of the whole objects area
                    pose.pose.orientation.x = 0.707800813341 # q[0]
                    pose.pose.orientation.y = 0.705529079392 # q[1]
                    pose.pose.orientation.z = -0.0262860127754 # q[2]
                    pose.pose.orientation.w = 0.0235748234061 # q[3]
                    pose.pose.position.x =  0.00263965839473
                    pose.pose.position.y = 0.768259834575
                    pose.pose.position.z = 0.375867027687
                    pose_publisher.publish(pose)
                elif key == 'l':
                    pose = controllers.group.get_current_pose()
                    print(pose)
                    panda_joint_states = rospy.wait_for_message("/joint_states", JointState)
                    print(panda_joint_states)
                    ee = controllers.group.get_end_effector_link()
                    print('ee link:',ee)

            else:
                # After CTRL+C is pressed - shutdown the publisher and subscriber nodes.
                if key=='\x03':
                    pose.pose.position.x = 100
                    pose_publisher.publish(pose)
                    panda_joints.position = (9, 9, 9, 9, 9, 9, 9)
                    joints_publisher.publish(panda_joints)
                    time.sleep(0.3)
                    break
    except:
        print "Finished publishing keyboard commands"

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

