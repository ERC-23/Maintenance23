#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg as msg
from moveit_msgs.msg import Constraints,JointConstraint
from math import pi
from csv_read import get_pos

rospy.init_node("moveit_press_node", anonymous = True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.robot.RobotCommander()
moveit_gp = moveit_commander.MoveGroupCommander("manipulator")
hand = moveit_commander.MoveGroupCommander("gripper")

moveit_gp.set_pose_reference_frame("base_link")
moveit_gp.set_end_effector_link("tool0")
moveit_gp.set_goal_position_tolerance(0.005)
moveit_gp.set_planning_time(5)
moveit_gp.allow_replanning(True)
pub = rospy.Publisher("/move_group/display_planned_path",msg.DisplayTrajectory,queue_size=20)


home = [0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]

def go_delay(pos):
    moveit_gp.go(pos, wait=True)
    rospy.sleep(1)
        
def Traverse ():
    print ("#############starting###########")

    pos1= [0.10007249103361762, -2.094304532481178, 1.789424605603017, 0.34880856674207017, 1.5707935270862619, -1.570854054406988]
    go_delay(pos1)

    pos1= [-0.09992750896638239, -2.4943045324811783, 1.789424605603017, 0.34880856674207017, 1.5707935270862619, -1.570854054406988]
    go_delay(pos1)
    
    pos1= [-0.09992750896638239, -2.4943045324811783, 2.1894246056030173, 0.34880856674207017, 1.670793527086262, -1.570854054406988]
    go_delay(pos1)

    pos1=[-0.5999613598024223, -2.294563788670477, 1.7207320214606074, 0.6502522780386224, 1.670787729708466, -1.570661938820356]
    go_delay(pos1)

    pos1= [0.5999356919918236, -1.59441668791816, 1.889397396376634, 0.5487174191027042, 1.5708080449667223, -1.5708207972587571]
    go_delay(pos1)

    pos1= [-0.4999741404008722, -1.6944622357838153, 1.8893902132858753, 0.6487277358514777, 1.6707369736010054, -1.5707083676273177]
    go_delay(pos1)
    
    pos1= [0.40006882077927997, -2.394547891831385, 1.5207773353134089, 0.6501786989873253, 1.670682515120844, -1.570795821464654]
    go_delay(pos1)
    
    print ("############finisihing###########")
#[-0.9000643080081763, -1.7944166879181602, 1.6893973963766338, 0.44871741910270424, 1.5708080449667223, -1.5708207972587571]

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

    moveit_gp.set_path_constraints(joints_con)
 
if __name__ == "__main__":
    try:
        Set_constraints()
        moveit_gp.go(home, wait=True)
        Traverse()
        moveit_gp.go(home, wait=True)
        print("Task 1 Finsished")
        
    except rospy.ROSInterruptException:
        pass

