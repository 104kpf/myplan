#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
from math import atan2, acos, asin, sqrt, sin, cos, pi

'''Variable for end-effector'''
EefState = 0

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_heitght = 0.025   #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#You may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0,15) as lab4 shown

'''Robot arm geometry'''
l0 = 0.06;l1 = 0.082;l2 = 0.132;l3 = 0.1664;l4 = 0.048;d4 = 0.004

'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    group_names = robot.get_group_names()
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles

  def go_to_joint_state(self):

    move_group = self.move_group
    joint_angles = self.joint_angles

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = joint_angles[0]
    joint_goal[1] = joint_angles[1]
    joint_goal[2] = joint_angles[2]
    joint_goal[3] = joint_angles[3]

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print "current pose:"
    print current_pose.position 
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x,y,z): 
  global l0,l1,l2,l3,l4,d4
  '''
  Write your code here!
  x,y,z is in world frame (the same as link0 frame)
  The end-effector should parallel to the ground.
  '''
  joint_angle=[0.0, 0.0, 0.0, 0.0]
  return joint_angle


def pub_EefState_to_arm():
    '''
    Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
    '''
    global pub_EefState,rate
    pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
    rate = rospy.Rate(100) # 100hz

def hanoi_tower():
  pass

def main():
  global pub_EefState, EefState
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print "ctrl + z to close"
    while not rospy.is_shutdown():
  
        try:

          '''
          Modify this into a list of [x, y, z, 1 or 0]

          x_input=float(raw_input("x:  "))
          y_input=float(raw_input("y:  "))
          z_input=float(raw_input("z:  "))
          1 for end-effector on;0 for off
          
          
          path_object.joint_angles = Your_IK(x,y,z)
          path_object.go_to_joint_state() #path will automatically be published by moveit
          EefState = 0
          pub_EefState.publish(EefState)  #publish end-effector state


          '''
          
           

        except:
          '''Go back to home if weird input'''
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

