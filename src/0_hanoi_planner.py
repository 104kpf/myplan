#!/usr/bin/env python
import rospkg 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
from math import atan2, acos, asin, sqrt, sin, cos, pi
import copy
import json
import time
import math
import numpy as np

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

'''Hanoi tower mesh file path'''
rospack = rospkg.RosPack()
FILE_PATH = rospack.get_path('myplan')+ "/mesh"
MESH_FILE_PATH = [FILE_PATH +"/tower1.stl",FILE_PATH +"/tower2.stl",FILE_PATH +"/tower3.stl"]



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
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # move_group.set_planner_id("RRT")
    eef_link = 'link5'
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # EefState_publisher =  rospy.Publisher('/SetEndEffector', Bool, queue_size=10)                                        
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame


    #move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404]) #default
    move_group.set_workspace([-0.23, -0.24, 0.0, 0.275, 0.24, 0.38])


    group_names = robot.get_group_names()
    # Misc variables
    
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    # self.EefState_publisher = EefState_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
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
    start = time.time()
    move_group.go(joint_goal, wait=True)
    print("planning cost", time.time() - start)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print("current pose:" , current_pose.position) 
    return all_close(joint_goal, current_joints, 0.01)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self, box_name , box_pose, size_tuple):  
    '''
    Description: 
        1. Add a box to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
        3. Google scene.add_box for more details
    '''
    scene = self.scene
    scene.add_box(box_name, box_pose, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, box_name, link_name):
    '''
    Description:
        1. Make sure the box has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. Google scene.attach_box for more details
    '''
    scene = self.scene
    scene.attach_box(link_name, box_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, box_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
        3. Google scene.detach_box for more details
    '''
    return # TODO
    scene = self.scene
    scene.remove_attached_object(link_name, name=box_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, box_name):
    '''
    Description: 
        Remove a box from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def add_mesh(self, mesh_name, mesh_pose, file_path, size_tuple): 
    '''
    Description: 
        1. Add a mesh to rviz, Moveit_planner will think of which as an obstacle.
        2. An example is shown in the main function below.
    '''
    return # TODO
    scene = self.scene
    mesh_pose.pose.orientation.w = 0.7071081
    mesh_pose.pose.orientation.x = 0.7071081
    mesh_pose.pose.orientation.y = 0
    mesh_pose.pose.orientation.z = 0
    #deal with orientation-definition difference btw .stl and robot_urdf
    scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Make sure the mesh has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. An example is shown in the main function below.
    '''
    return # TODO
    scene = self.scene
    scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
    '''
    return # TODO
    scene = self.scene
    scene.remove_attached_object(link_name, name=mesh_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_mesh(self, mesh_name):
    '''
    Description: 
        Remove a mesh from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(mesh_name)
    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_world_object(self):
    '''
    Description: 
        Remove all objects from rviz.
    '''
    #remove all if no name specified
    self.scene.remove_world_object()
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  
  
  
  '''
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
    return all_close(joint_goal, current_joints, 0.01)'''

def pub_EefState_to_arm():
    '''
    Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
    '''
    global pub_EefState,rate
    pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
    rate = rospy.Rate(100) # 100hz

def Your_IK(x,y,z,q): 
  start_time = time.time()
  
  #link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040]
  link = [0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0050] #new
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
          qdot = Jt.dot(K).dot(e)
          q += qdot

  q2 = q[0,0]
  q3 = q[1,0]
  q4 = pi/2 - q2 - q3
  # joint_angle = your_IK_solution
  joint_angle=[q1, q2, q3, q4]
  time_cost = time.time() - start_time
  print("IK cost ", time_cost, "sec to compute")
  return joint_angle

def placetower(pos):
  global tower  
  if pos =='a': 
    #tower = {'A': ['tower3', 'tower2', 'tower1'], 'B': [], 'C': []}  
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object
    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.0             # Specify z of the mesh
    pathPlanObject.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.015             # Specify z of the mesh
    pathPlanObject.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.03            # Specify z of the mesh
    pathPlanObject.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))
  elif pos =='b':
    #tower = {'A': [], 'B': ['tower3', 'tower2', 'tower1'], 'C': []}  
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object
    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.0             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.0             # Specify z of the mesh
    pathPlanObject.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.0             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.015             # Specify z of the mesh
    pathPlanObject.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.0             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.03            # Specify z of the mesh
    pathPlanObject.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))
  elif pos =='c':
    #tower = {'A': [], 'B': [], 'C': ['tower3', 'tower2', 'tower1']} 
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object
    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = -0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.0             # Specify z of the mesh
    pathPlanObject.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = -0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.015             # Specify z of the mesh
    pathPlanObject.add_mesh('tower2', mesh_pose, MESH_FILE_PATH[1], (.00095,.00095,.00095))

    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.25            # Specify x of the mesh
    mesh_pose.pose.position.y = -0.15             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.03            # Specify z of the mesh
    pathPlanObject.add_mesh('tower3', mesh_pose, MESH_FILE_PATH[2], (.00095,.00095,.00095))

def move_disc(pathPlanObject, from_pos, to_pos, tower, positions):
    # Get the positions for from_pos and to_pos
    from_position = positions[from_pos]
    to_position = positions[to_pos]
    #from_z_height = 0.06 - (0.0175 * (3 - len(tower[from_pos])))
    #to_z_height = (0.025 * (1+len(tower[to_pos])*0.0175/0.025))
    from_z_height = 0.074 - (0.016 * (3 - len(tower[from_pos])))
    to_z_height = (0.045 * (1+len(tower[to_pos])*0.018/0.045))
    
    disc = tower[from_pos].pop()
    pathPlanObject.joint_angles = Your_IK(from_position[0], from_position[1], from_z_height,pi/2)
    pathPlanObject.go_to_joint_state()
    EefState = 1
    pub_EefState.publish(EefState)
    #print("Moving disc", disc, "from", from_pos, "to", to_pos,'tnumf',len(tower[from_pos]),'tnumt',len(tower[to_pos]))
    #pathPlanObject.attach_mesh(disc, 'link5')
    # publish eff state to arm
    

    # Move the robot arm to the 'to' position
    pathPlanObject.joint_angles = Your_IK(to_position[0], to_position[1], to_z_height,pi/2)
    pathPlanObject.go_to_joint_state()
    #pathPlanObject.detach_mesh(disc, 'link5')
    # publish eff state to arm
    EefState = 0
    pub_EefState.publish(EefState)
    tower[to_pos].append(disc)

       
def hanoi(n, from_pos, to_pos, aux_pos, move_func, pathPlanObject, tower, positions):
    if n == 1:
        move_func(pathPlanObject, from_pos, to_pos, tower, positions)
        #print('disc', n, 'from', from_pos, 'to', to_pos)
    else:
        hanoi(n-1, from_pos, aux_pos, to_pos, move_func, pathPlanObject, tower, positions)
        move_func(pathPlanObject, from_pos, to_pos, tower, positions)
        #print('disc', n, 'from', from_pos, 'to', to_pos)
        hanoi(n-1, aux_pos, to_pos, from_pos, move_func, pathPlanObject, tower, positions)

def placeobs():
    pathPlanObject = MoveGroupPythonIntefaceTutorial()
    
    box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    box_pose.pose.position.x = 0.25             # Specify x of the box
    box_pose.pose.position.y = 0.075             # Specify y of the box
    box_pose.pose.position.z = 0.05        # Specify z of the box
    pathPlanObject.add_box('box_1', box_pose, (0.05, 0.005, 0.1)) #Specify box name, box pose, size in xyz
    box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    box_pose.pose.position.x = 0.25             # Specify x of the box
    box_pose.pose.position.y = -0.075             # Specify y of the box
    box_pose.pose.position.z = 0.08         # Specify z of the box
    pathPlanObject.add_box('box_2', box_pose, (0.1, 0.015, 0.16)) #Specify box name, box pose, size in xyz
    box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    box_pose.pose.position.x = 0.1             # Specify x of the box
    box_pose.pose.position.y = 0.0             # Specify y of the box
    box_pose.pose.position.z = -0.01        # Specify z of the box
    pathPlanObject.add_box('box_3', box_pose, (0.61, 0.46, 0.01)) #Specify box name, box pose, size in xyz
    # box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    # box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    # box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    # box_pose.pose.position.x = 0.31             # Specify x of the box
    # box_pose.pose.position.y = 0.0             # Specify y of the box
    # box_pose.pose.position.z = 0.09       # Specify z of the box
    # pathPlanObject.add_box('box_4', box_pose, (0.005, 0.5, 0.18)) #Specify box name, box pose, size in xyz
    
    

def main():
  global pub_EefState, EefState, goal_input
  tower = {
        'a': [], 
        'b': [], 
        'c': []
    }
  positions = {
        'a': [0.260, 0.153, 0.065],
        'b': [0.259, -0.004, 0.065],
        'c': [0.255, -0.159, 0.065]
    }
  '''positions = {
        'a': [0.256, 0.153, 0.065],
        'b': [0.254, 0.0, 0.065],
        'c': [0.254, -0.154, 0.065]
    }'''
  try:
    
    
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object
    pub_EefState_to_arm()
    
    pathPlanObject.joint_angles = [0,-pi/2,pi/2,0]
    pathPlanObject.go_to_joint_state()
    print "ctrl + z to close"
    raw_input('Remove all object from rviz')
    pathPlanObject.remove_world_object()
    
    
    while not rospy.is_shutdown():
        try:
          placeobs()
          start_input = raw_input("enter start pos (a, b, or c):  ").strip().lower()
          placetower(start_input)
          tower = {
            'a': [], 
            'b': [], 
            'c': []
          }
          tower[start_input] = ['tower1', 'tower2', 'tower3']
          goal_input = raw_input("enter goal pos (a, b, or c):  ").strip().lower()
          aux = next(pos for pos in 'abc' if pos not in [start_input, goal_input])

          start_time = time.time()
          hanoi(3, start_input, goal_input, aux, move_disc, pathPlanObject, tower, positions)
          pathPlanObject.joint_angles = [0,-pi/2,pi/2,0]
          pathPlanObject.go_to_joint_state()
          EefState = 0

          print("total time elapse: ", time.time() - start_time)
          pub_EefState.publish(EefState)  #publish end-effector state

        except: 
          '''Go back to home if weird input'''
          pathPlanObject.joint_angles = [0,-pi/2,pi/2,0]
          pathPlanObject.go_to_joint_state()

  
  
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

