# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_robot", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)
# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)
# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())
# Sometimes for debugging it is useful to print the entire state of the robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

### POSE GOAL ###
# x = -183.9      #mm
# y = 714.04      #mm
# z = 216.86      #mm
# yaw = 12.71     #degrees about Z (A)
# pitch = -29.81  #degrees about Y (B)
# roll = -89.64   #degrees about X (C)

# pose_goal = geometry_msgs.msg.Pose()

# pose_goal.position.x = x / 1000
# pose_goal.position.y = y / 1000
# pose_goal.position.z = z / 1000

# rot = Rotation.from_euler('zyx', [yaw, pitch, roll], degrees = True)
# rot_quat = rot.as_quat()

# pose_goal.orientation.x = rot_quat[0]
# pose_goal.orientation.y = rot_quat[1]
# pose_goal.orientation.z = rot_quat[2]
# pose_goal.orientation.w = rot_quat[3]

# move_group.set_pose_target(pose_goal)
# # `go()` returns a boolean indicating whether the planning and execution was successful.
# success = move_group.go(wait=True)
### END POSE GOAL ###

### JOINT GOAL ###
a1 = -33.47     #degrees
a2 = 29.72      #degrees
a3 = -115.22    #degrees
a4 = -4.07      #degrees
a5 = -46.87     #degrees
a6 = -33.93     #degrees

joint_goal = move_group.get_current_joint_values()

joint_goal[0] = a1 * math.pi / 180
joint_goal[1] = a2 * math.pi / 180
joint_goal[2] = a3 * math.pi / 180
joint_goal[3] = a4 * math.pi / 180
joint_goal[4] = a5 * math.pi / 180
joint_goal[5] = a6 * math.pi / 180

move_group.go(joint_goal)
### END JOINT GOAL ###


# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()