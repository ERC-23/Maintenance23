#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
import time
import subprocess
from std_msgs.msg import Bool
from math import pi


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


if __name__ == '__main__':
    try:

        print("######## Objective 10 is started #######")

        init_pos=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
        move_rotational(init_pos)
        start_pose = move_group.get_current_pose().pose
        
        grip_open()
        rospy.sleep(4) 


    except rospy.ROSInterruptException:
        pass


