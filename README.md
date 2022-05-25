# Python interface and modified franka_ros (for linux)
Control the franka panda cobot and its original gripper via python (2.7) scripts that call ros' moveit. Offers gripper pose and grasping objects as well as arm carthesian and joint pose control.

## Install on your linux real time pc (connected to franka controller):
1. install ros melodic: apt install ros-melodic-desktop-full
2. install pip and cmake: apt install python-pip cmake
3. install ros control pkgs: apt install ros-melodic-ros-control ros-melodic-moveit ros-melodic-catkin ros-melodic*controller*
4. pip install ros python pkgs: pip install ros_numpy
5. download or pull the franka_rospy repository (catkin workspace)
6. cd to catkin workspace (franka_rospy) and catkin_make (fix errors)
7. find in franka_rospy/src/franka_python the basic scripts (e.g., panda_motion_generators.py)
8. source franka_rospy/devel/setup.bash && export ROS_MASTER_URI=http://your.ip.add.ress:11311 && export ROS_IP=your.ip.add.ress'
9. (optional) repeat on further computers (1-8) and send commands via network to RT computer  


## Based on original Franka ROS
See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.
All packages of franka_ros are licensed under the Apache 2.0 license
