#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
import time
import subprocess
from std_msgs.msg import Bool
from math import pi
from moveit_msgs.msg import Constraints,JointConstraint


#########################   Node Initialization     #########################

rospy.init_node('home_pose',anonymous=True)

#########################   Moveit Initialization   #########################

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.robot.RobotCommander()
move_group = moveit_commander.MoveGroupCommander('manipulator')
gripper = moveit_commander.MoveGroupCommander('gripper')

move_group.set_planning_time(5)
move_group.allow_replanning(True)

move_group.set_pose_reference_frame('base_link')

move_group.set_goal_position_tolerance(0.005)
move_group.set_goal_orientation_tolerance(0.1)

move_group.set_planner_id("APSkConfigDefault") # perfect
move_group.set_end_effector_link('tool0')
end_effector_link = move_group.get_end_effector_link()

display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

home=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]

#########################   Functions Implementation  #########################

def grip_close():

    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'close'", shell=True,start_new_session=True)
    time.sleep(1)

    process.terminate()
    process.wait()

#*********************************#

def grip_open():

    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'open'", shell=True,start_new_session=True)
    time.sleep(1)

    process.terminate()
    process.wait()

#*********************************#

def move_rotational(joint_goal) : 
  
  move_group.go(joint_goal, wait=True)

####################### Constrains #######################
def Set_constraints():
    
    joints_con = Constraints()
    joints_con.name = 'goal_constraints'

    elbow_con = JointConstraint()
    elbow_con.joint_name = 'elbow_joint'
    elbow_con.position = home[2]
    elbow_con.tolerance_above = 1
    elbow_con.tolerance_below = 1
    elbow_con.weight = 1.0

    joints_con.joint_constraints.append(elbow_con)

    wrist3_con = JointConstraint()
    wrist3_con.joint_name = "wrist_3_joint"
    wrist3_con.position   =  home[5]
    wrist3_con.tolerance_above = 0.2
    wrist3_con.tolerance_below = 0.2
    wrist3_con.weight = 1.0

    joints_con.joint_constraints.append(wrist3_con)

    base_con = JointConstraint()
    base_con.joint_name = "shoulder_pan_joint"
    base_con.position   =  home[0]
    base_con.tolerance_above = 1.8
    base_con.tolerance_below = 1.8
    base_con.weight = 0.8

    joints_con.joint_constraints.append(base_con)

    move_group.set_path_constraints(joints_con)


if __name__ == '__main__':
    try:

        print("######## Objective 10 is started #######")
        Set_constraints()
        move_rotational(home)
        start_pose = move_group.get_current_pose().pose
        
        grip_open()
        rospy.sleep(4) 


    except rospy.ROSInterruptException:
        pass


