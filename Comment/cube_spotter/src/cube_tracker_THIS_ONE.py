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
from open_manipulator_msgs.srv import SetKinematicsPose #We imported Set KinematicsPose to subscribe to the goal_path_position_only service to plan the IK route of the gripper in taskspace

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
    self.grip_pos = rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, self.grip_where) # This subscribes to the kinematics pose of the gripper so we know its position.

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
    self.IKpose = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose) # Subscribes to the service that plans a route through task space from one position to another 

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
    self.jointRequest.position=[0.0,-1.05,0.357,1.3]  #We lowered joint 4's angle in this position so it didn't see the wall as a giant blue cube God.
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up

    pose = KinematicsPose()
    pose.pose.position.x = 0.1
    pose.pose.position.y = 0.2
    self.IKpose(str(), "gripper", pose, 4)

    rospy.sleep(1)

    self.order = str(input("Enter cube stack order (first letter only e.g. RYB): ")) # with this the user can input how many blocks to move and in which order.
    while len([i for i in self.order if i.lower() not in "rby"]) > 0:
      print("Unknown colour detected")
      self.order = str(input("Enter cube stack order (first letter only e.g. RYB): "))

    self.mode = str(input("Pyramid or stack? (P/S): ")) #This gives the user the option between placing the cubes as a stack or a pyramid.
  
    if self.mode.lower() == "s":
      self.stackPosX = float(input("Enter stack x position (in meters): "))
      self.stackPosY = float(input("Enter stack y position (in meters): "))
      while ((np.sqrt(self.stackPosX**2 + self.stackPosY**2) > 0.20) or (self.stackPosX < 0) or (self.stackPosY > 0.02)):
        print("Position out of bounds")
        self.stackPosX = float(input("Enter stack x position (in meters): "))
        self.stackPosY = float(input("Enter stack y position (in meters): "))
                               
    self.cubeNumber = 0 #To move from one cube placement to the next, once a cube is placed, the cube number increments so the next cube in self.order is scanned for, at the start its set to 0.
    self.colourPicker() #This function is where the colour of the next cube is selected

    # As the last movement called was the arm, we dont update the request again below, but it would be necessary if switching between the arm and the gripper.

  def move(self,pose,time=0.5):  #This function sets the movement of the robot
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=pose
    self.setPose(str(),self.jointRequest,time)
    rospy.sleep(time)

  def moveKinematic(self,x,y,z,time,recursCount=0):  #This function controls all movement of the robot, taking in desired position it wants to move to, and the time wished to move it there as parametres.
    pose = KinematicsPose() #The current position of the gripper is set to 'pose'.
    pose.pose.position.x = x #The desired x position of the gripper in taskspace is set
    pose.pose.position.y = y #as are the y and z position
    pose.pose.position.z = z
    response = self.IKpose(str(), "gripper", pose, time) #The Inverse Kinematics of the robot is found between gripper current pose and the desired position
    rospy.sleep(time) #To prevent controller freezing the robot must sleep for the duration it moves
    if not response.is_planned: #This is a recursive function for if the IK soloution fails
      recursCount += 1
      currentPose = [self.kinematics.position.x,self.kinematics.position.y,self.kinematics.position.z] #If the full path planning fails, the current position is taken
      difference = np.subtract([x,y,z],currentPose) #And the path from current postion to desired is found and halved so the robot can try to move halfway to desired point 
      target = currentPose + difference/2
      if recursCount < 5: #The count prevents the robot getting stuck trying to find a soloution, it only tries 5 times.
        self.moveKinematic(target[0],target[1],target[2],time,recursCount)
      else:
        self.move([0.0,-1.05,0.357,1.3])
      self.moveKinematic(x,y,z,time) #Once halfway to desired point it tries to find an IK soloution from the new position

  def moveGripper(self,state=1): #This function is called to move the gripper, its default state is open
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[state*0.01]# -0.01 represents closed, if state changes to -1, this calc opens the gripper
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(1)

  def scan(self, inc = 6, time = 5): #This function scans the reachable workspace for blocks
    self.aimLoop = 0
    ground_search = np.linspace([0.443,-0.979, 0.453, 1.8],[-1.5,-0.979, 0.453, 1.8], inc) # range of lower search positions
    high_search = np.linspace([-1.5, -0.73,-0.215,1.8],[0.443, -0.73,-0.215,1.8], inc) # range of higher search positions
    search = np.concatenate((ground_search, high_search), axis=0) # array of all search positions
    # print(search)
    for i in search: # move through each search position
      # print(i)
      self.move(i)
      if self.isTarget == True: # if block is seen, stop scanning
        break


  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position

  # get the grippers cartesean coordinates
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
      ground_cam2block = np.sqrt((self.target.distance/100)**2 - (self.kinematics.position.z)**2) # use pythag to calculate the distance from the block to the ground beneath the claw
      blockX = ground_cam2block*np.cos(self.jointPose[0]) + self.kinematics.position.x # use trig to calculate the x coordinate for the block
      blockY = ground_cam2block*np.sin(self.jointPose[0]) + self.kinematics.position.y # use trig to calculate the y coordinate for the block
      # print(blockX)
      # print(blockY)
      # print(self.target.distance)

      self.aimLoop += 1 

      if self.aimLoop == 6: # run aimCamera 6 times before grabbing the robot to make sure the block is centred
        # Grab the cube
        self.moveGripper(1)
        self.moveKinematic(blockX*1.05,blockY*1.05,0.02,4)
        self.moveGripper(-1)
        self.moveKinematic(blockX*1.05,blockY*1.05,0.1,1)
        if self.mode.lower() == "p":
          self.pyramid()
        elif self.mode.lower() == "s":
          self.stack()
        self.aimLoop = 0

  # Place cubes in a pyramid
  def pyramid(self):
    if self.cubeNumber == 0: # first cube
      self.moveKinematic(0, -0.2, 0.02, 2)
      self.moveGripper(1)
      self.cubeNumber += 1
      self.colourPicker()
      self.moveKinematic(0, -0.15, 0.1, 1)
    elif self.cubeNumber == 1: # second cube
      self.moveKinematic(0, -0.17, 0.1, 2)
      self.moveKinematic(0, -0.17, 0.02, 1)
      self.moveGripper(1)
      self.cubeNumber += 1
      self.colourPicker()
      self.moveKinematic(0, -0.1, 0.1, 1)
    elif self.cubeNumber == 2: # third cube
      self.moveKinematic(0, -0.175, 0.12, 2)
      self.moveKinematic(0, -0.175, 0.08, 1)
      self.moveGripper(1)
      self.cubeNumber += 1
      self.colourPicker()
      self.moveKinematic(0, -0.1, 0.14, 1)

  # place cubes in a stack
  def stack(self):
    self.moveKinematic(self.stackPosX,self.stackPosY,0.05+(self.cubeNumber*0.05),1)
    self.moveKinematic(self.stackPosX,self.stackPosY,0.02+(self.cubeNumber*0.05),1)
    self.moveGripper(1)
    self.cubeNumber += 1
    self.colourPicker()
    self.moveKinematic(self.stackPosX,self.stackPosY,0.05+(self.cubeNumber*0.05),1)
    

  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    # Get the red cubes
    for c in range(len(data.cubes)):
      if (data.cubes[c].cube_colour==self.colour) and (data.cubes[c].distance < 25):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
      if (data.cubes[c].distance >= 25):
        print(data.cubes[c].cube_colour + " cube spotted out of reach")

      
    
    # Find the biggest red cube
    if (len(area))>0:
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.target=data.cubes[index_max]
      self.isTarget=True # raises flag to indicate that a cube has been seen
      # print("TARGET ACQUIRED")
    else: # If you dont find a target, report the centre of the image to keep the camera still
      self.targetX=0.5
      self.targetY=0.5
      self.isTarget=False # lowers flag to indicate no cubes seen
      # print("TARGET NEUTRALISED")

  # Changes colour cube that the camera looks for
  def colourPicker(self):
    try:
      if self.order[self.cubeNumber].lower() == "r":
        self.colour = "red"
      elif self.order[self.cubeNumber].lower() == "y":
        self.colour = "yellow"
      elif self.order[self.cubeNumber].lower() == "b":
        self.colour = "blue"
      print(self.colour)
    except: # if an overflow error occurs, it indicates that the program is complete and ends the program
      print("job done :)")
      # self.party()
      exit()
      print("hasnt ended")

  # Does the worm
  def party(self):
    for i in range(0,4):
      self.move([0,0.342,-0.775,0.396])
      self.move([0.0,1.0,-1.5,-0.097])
      self.move([0,0.718,-0.054,-0.773])
      self.move([0,-0.468,0.621,-0.104])
    self.move([0.443,-0.979, 0.453, 1.8])

  # chooses between scanning or targeting
  def loop(self):
    if self.isTarget == False:
      print("SCANNING FOR DROMAOSAURID")
      self.scan()
    else:
      print("DROMAOSAURID SPOTTED")
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
