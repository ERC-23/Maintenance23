#!/usr/bin/env python3

import csv
import os
import rospy 
import moveit_commander
import sys
from math import pi 
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints,JointConstraint
import subprocess
import time


####################### Initialization #######################
rospy.init_node("panel",anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot=moveit_commander.robot.RobotCommander()

#planning group 
moveit_group = moveit_commander.MoveGroupCommander("manipulator")
hand = moveit_commander.MoveGroupCommander("gripper")

moveit_group.set_pose_reference_frame("base_link")
moveit_group.set_end_effector_link("tool0")

end_effector_link = moveit_group.get_end_effector_link()

moveit_group.set_goal_position_tolerance(0.005)
moveit_group.allow_replanning(True)

####################### Home Pose #######################
Home_pose = [0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
start_pose = moveit_group.get_current_pose().pose

####################### Constrains #######################
def Set_constraints():
    
    joints_con = Constraints()
    joints_con.name = 'goal_constraints'

    elbow_con = JointConstraint()
    elbow_con.joint_name = 'elbow_joint'
    elbow_con.position = Home_pose[2]
    elbow_con.tolerance_above = 1
    elbow_con.tolerance_below = 1
    elbow_con.weight = 1.0

    joints_con.joint_constraints.append(elbow_con)

    wrist3_con = JointConstraint()
    wrist3_con.joint_name = "wrist_3_joint"
    wrist3_con.position   =  Home_pose[5]
    wrist3_con.tolerance_above = 0.2
    wrist3_con.tolerance_below = 0.2
    wrist3_con.weight = 1.0

    joints_con.joint_constraints.append(wrist3_con)

    base_con = JointConstraint()
    base_con.joint_name = "shoulder_pan_joint"
    base_con.position   =  Home_pose[0]
    base_con.tolerance_above = 1.8
    base_con.tolerance_below = 1.8
    base_con.weight = 0.8

    joints_con.joint_constraints.append(base_con)

    moveit_group.set_path_constraints(joints_con)
 
def close_gripper():

    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'close'", shell=True,start_new_session=True)
    time.sleep(5)
    process.terminate()
    process.wait()

def open_gripper():
    
    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'open'", shell=True,start_new_session=True)
    time.sleep(5)
    process.terminate()
    process.wait()

def Go_home_pose():

    moveit_group.go(Home_pose,wait=True)
    rospy.sleep(5)

def get_pos(x):
            with open('filex.csv','r') as csv_pos_file: 
                csv_reader = csv.reader(csv_pos_file)
                y=[] 
                pos=[]
                for line in csv_reader : #read file
                    for word in line :   
                        y.append (word) #move elements to list y
                i=0
                for key in y : 
                    i += 1
                    if key == f"ID = {x}":  #search in list for ID n
                        y[i]= y[i].split()
                        pos.append(y[i][1]) #add position to new list pos
                        pos.append(y[i][3])
                        pos.append(y[i][5])
                        break        
                return pos

def press_switch(pose):

    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x = float(pose[0]) - 0.25
    target.pose.position.y = float(pose[1]) 
    target.pose.position.z = float(pose[2])  - (0.02 + 0.03)

    target.pose.orientation.x = start_pose.orientation.x
    target.pose.orientation.y = start_pose.orientation.y
    target.pose.orientation.z = start_pose.orientation.z
    target.pose.orientation.w = start_pose.orientation.w

    moveit_group.set_start_state(robot.get_current_state())  

    print("###### Reaching ",current_ID," ######")     
    moveit_group.set_pose_target(target,end_effector_link)
    success = moveit_group.go(wait = True)
    time.sleep(5)

    print("###### Close gripper ######")     
    close_gripper()
    time.sleep(2)
    
    print("###### Move forward ######")     
    target.pose.position.x += 0.015
    moveit_group.set_pose_target(target,end_effector_link)
    success = moveit_group.go(wait = True)
    time.sleep(5)
          
    print("###### Open gripper ######")     
    open_gripper()
    time.sleep(2)

if __name__ == '__main__':
    
    try:

        os.chdir(os.path.dirname(os.path.abspath(__file__)))

        n = rospy.get_param('~tag')
        rospy.loginfo(n)
        Switch_arr = n.split(',')
        Switch_arr = [int(i) for i in Switch_arr]

        Set_constraints()
        
        for j in range(len(Switch_arr)) :
        
            current_ID = Switch_arr[j]
            current_switch_pose = get_pos(current_ID)
            
            press_switch(current_switch_pose)
            
            print("###### Return Home Pose ######")
            Go_home_pose()     
            

    except rospy.ROSInterruptException:
        pass

