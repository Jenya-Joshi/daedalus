#!/usr/bin/env python
from __future__ import print_function
from re import X

import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
import numpy as np;

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetKinematicsPose

class cubeTracker:

  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.target=False

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.0,-1.05,0.357,0.703]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)
    self.grip_pos = rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, self.grip_where)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
    self.IKpose = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)

    self.aimLoop = 0

    ## EXAMPLE POSES - Start by moving the robot to some example positions, and use the gripper

    print("Grabby hands commence!")
    # Send the robot to "zero"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,0.0,0.0,0.0]
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up

    # Open the gripper
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# 0.01 represents open
    self.setGripper(str(),self.gripperRequest,1.0)

    rospy.sleep(1) # Wait for the gripper to open

    # Close the gripper
    # self.gripperRequest=JointPosition()
    # self.gripperRequest.joint_name=["gripper"]  
    # self.gripperRequest.position=[-0.01]# -0.01 represents closed
    # self.setGripper(str(),self.gripperRequest,1.0)

    # rospy.sleep(1) # Wait for the gripper to close

    # Send the robot "home"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,-1.05,0.357,1.3]
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up

    pose = KinematicsPose()
    pose.pose.position.x = 0.1
    pose.pose.position.y = 0.2
    self.IKpose(str(), "gripper", pose, 4)

    rospy.sleep(1)

    self.order = str(input("Enter cube stack order (first letter only e.g. RYB): "))
    self.cubeNumber = 0
    self.colourPicker()

    # As the last movement called was the arm, we dont update the request again below, but it would be necessary if switching between the arm and the gripper.

  def move(self,pose,time=0.5):
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position= pose
    self.setPose(str(),self.jointRequest,time)
    rospy.sleep(time)

  def moveKinematic(self,x,y,z,time):
    pose = KinematicsPose()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    response = self.IKpose(str(), "gripper", pose, time)
    rospy.sleep(time)
    if not response.is_planned:
      currentPose = [self.kinematics.position.x,self.kinematics.position.y,self.kinematics.position.z]
      difference = np.subtract([x,y,z],currentPose)
      target = currentPose + difference/2
      self.moveKinematic(target[0],target[1],target[2],time)
      self.moveKinematic(x,y,z,time)

  def moveGripper(self,state=1):
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[state*0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(1)

  def scan(self, inc = 6, time = 5):
    self.aimLoop = 0
    ground_search = np.linspace([0.443,-0.979, 0.453, 1.8],[-1.5,-0.979, 0.453, 1.8], inc)
    high_search = np.linspace([-1.5, -0.73,-0.215,1.8],[0.443, -0.73,-0.215,1.8], inc)
    search = np.concatenate((ground_search, high_search), axis=0)
    print(search)
    for i in search:
      print(i)
      self.move(i)
      if self.isTarget == True:
        break


  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position


  def grip_where(self,data):
    self.kinematics=data.pose 

  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False



  # Using the data from all the subscribers, call the robot's services to move the end effector
  def aimCamera(self):
    if self.readyToMove==True: # If the robot state is not moving
      # print(self.target)
      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.1):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)

      if (abs(self.targetX-0.5)>0.1):
        self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)

      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast   
      ground_cam2block = np.sqrt((self.target.distance/100)**2 - (self.kinematics.position.z)**2)
      blockX = ground_cam2block*np.cos(self.jointPose[0]) + self.kinematics.position.x
      blockY = ground_cam2block*np.sin(self.jointPose[0]) + self.kinematics.position.y
      print(blockX)
      print(blockY)
      # print(self.target.distance)

      self.aimLoop += 1

      if self.aimLoop == 6:
        self.moveGripper(1)
        self.moveKinematic(blockX*1.05,blockY*1.05,0.02,4)
        self.moveGripper(-1)
        self.moveKinematic(blockX*1.05,blockY*1.05,0.1,1)
        self.moveKinematic(0,-0.15-(0.01*self.cubeNumber),0.05+(self.cubeNumber*0.05),1)
        self.moveKinematic(0,-0.15-(0.01*self.cubeNumber),self.cubeNumber*0.05,1)
        self.moveGripper(1)
        self.cubeNumber += 1
        self.colourPicker()
        self.moveKinematic(0,-0.1,0.02+(self.cubeNumber*0.06),1)
        self.aimLoop = 0
  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    # Get the red cubes
    for c in range(len(data.cubes)):
      if (data.cubes[c].cube_colour==self.colour):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)

      
    
    # Find the biggest red cube
    if (len(area))>0:
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.target=data.cubes[index_max]
      self.isTarget=True
      # print("TARGET ACQUIRED")
    else: # If you dont find a target, report the centre of the image to keep the camera still
      self.targetX=0.5
      self.targetY=0.5
      self.isTarget=False
      # print("TARGET NEUTRALISED")

  def colourPicker(self):
    try:
      if self.order[self.cubeNumber].lower() == "r":
        self.colour = "red"
      elif self.order[self.cubeNumber].lower() == "y":
        self.colour = "yellow"
      elif self.order[self.cubeNumber].lower() == "b":
        self.colour = "blue"
    except:
      print("done")
      

  def loop(self):
    if self.isTarget == False:
      print("SCANNING FOR DROMAOSAURID")
      self.scan()
    else:
      # print("DROMAOSAURID SPOTTED")
      self.aimCamera()

# Main 
def main(args):

  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  try:
    #rospy.spin()
    while not rospy.is_shutdown() or ic.cubeNumber <= len(ic.order):
      ic.loop()
    if ic.cubeNumber > len(ic.order):
      print("Job done :)")
  except KeyboardInterrupt:
    print("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)