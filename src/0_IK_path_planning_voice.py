#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import json
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
command = None

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
    # print "============ Planning frame: %s" % planning_frame

    # move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])

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
    print ("current pose:")
    # print (current_pose.position) 
    print ("x: %.5f" %current_pose.position.x)
    print ("y: %.5f" %current_pose.position.y)
    print ("z: %.5f" %current_pose.position.z)

    current_rpy = self.move_group.get_current_rpy('link5')
    print ("rol: %.5f" %current_rpy[0])
    print ("pit: %.5f" %current_rpy[1])
    print ("yaw: %.5f" %current_rpy[2])
    print ("")
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x,y,z,q): 
  import math
  import numpy as np
  
  link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040]
  q1 = math.atan2(y,x)
  
  X = math.sqrt(x**2 + y**2) - math.cos(pi/2)*link[5] - math.sin(pi/2)*link[4]
  Y = z - link[0] - link[1] + math.sin(pi/2)*link[5] - math.cos(pi/2)*link[4]
  # recursive to find q2 q3
  Ts = 0.01
  Xd = np.array([[X],
          [Y]])

  K = np.array([[1, 0],
        [0, 1]])

  q = np.array([[-1.57], [1.57]])


  #print('qdot',qdot)
  #print('q',q)

  finished = False
  iter = 0

  while not finished:
          iter += 1
          Kinematic = np.array([0.1664 * np.sin(q[0] + q[1]) + 0.1320 * np.sin(q[0]),0.1664 * np.cos(q[0] + q[1]) + 0.1320 * np.cos(q[0])])
        
          e = Xd-Kinematic
          et = np.transpose(e)

          if (np.dot(et,e) <= 1e-6).all():
              finished = True
          else:
              qa = q[0,0]
              qb = q[1,0]
              J = np.array([[0.1664 * np.cos(qa + qb) + 0.1320 * np.cos(qa), 0.1664 * np.cos(qa + qb)], 
                            [-0.1664 * np.sin(qa + qb) - 0.1320 * np.sin(qa), -0.1664 * np.sin(qa + qb)]])
          Jt = np.transpose(J)
          qdot = Jt.dot(K).dot(e)*0.01
          q += qdot

  q2 = q[0,0]
  q3 = q[1,0]
  q4 = pi/2 - q2 - q3
  

  # joint_angle = your_IK_solution
  joint_angle=[q1, q2, q3, q4]
  return joint_angle

def callback(data):
    global command
    rospy.loginfo("Received: %s", data.data)
    
    try:
        command = json.loads(data.data)
        
        
    except json.JSONDecodeError as e:
        rospy.logerr("Error decoding JSON: %s", e)
    except KeyError as e:
        rospy.logerr("Missing key in JSON data: %s", e)

def whichpoint(point):
  global x_input, y_input, z_input
  if point=='a':
    x_input=0.25
    y_input=0.15
    z_input=0.05
  elif point =='b':
    x_input=0.25
    y_input=0.00
    z_input=0.05
  else:
    x_input=0.25
    y_input=-0.15
    z_input=0.05



def main():
  global command, x_input, y_input, z_input
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print("ctrl + z to close")
    rospy.Subscriber("gpt_reply_to_user", String, callback)
    print("wating for input")
    while not rospy.is_shutdown():
      
      if command is not None:
        try:
          point = command["point"]
          whichpoint(point)
          
          q_input = pi/2


          path_object.joint_angles = Your_IK(x_input,y_input,z_input,q_input)
          '''
          You just need to solve IK of a point, path planning will automatically be taken.  
          '''
          path_object.go_to_joint_state()
          command = None

        except:
          '''go back to home if weird input'''
          path_object.go_to_joint_state()
          path_object.joint_angles = [0,-pi/2,pi/2,0]

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


