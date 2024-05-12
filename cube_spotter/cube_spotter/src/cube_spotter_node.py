#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
import cv2 # Melodic might be cv3
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np;

class cube:
  def __init__(self):
    self.centreX=0.0
    self.centreY=0.0  
    self.area=0.0

# Function for finding boxes and drawing on the output image
def mask2box(mask,colour,canvas,minArea):

  # convert image into contours
  contours, hierachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  cubeList=dict()

  # Remove small areas
  minRect = []
  for i, c in enumerate(contours):

      if cv2.contourArea(c) > minArea:
        minRect.append(cv2.minAreaRect(c))

    
  for i, c in enumerate(minRect):
    tempCube=cube()  
    box = cv2.boxPoints(minRect[i])
    centre=minRect[i][0]
    size=minRect[i][1]
    tempCube.centreX=centre[0]
    tempCube.centreY=centre[1]
    tempCube.area=size[0]*size[1]
    cubeList[i]=tempCube
    box = np.intp(box) #np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)
    cv2.drawContours(canvas, [box], 0, colour)

  return canvas, cubeList

# Image erosion (remove edge of shapes and small blobs)
def erode(image,erosion_size):

    erosion_shape = cv2.MORPH_ELLIPSE
    
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                       (erosion_size, erosion_size))
    
    eroded = cv2.erode(image, element)

    return eroded

# Image dilation (make blobs bigger)
def dilate(image, dilation_size):

    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilation_size + 1, 2 * dilation_size + 1),
                                       (dilation_size, dilation_size))
    dilated = cv2.dilate(image, element)

    return dilated


class cubeSpotter:

  def __init__(self):

    # Define the colour ranges used to detect the blocks
    # Looking at the hsv image can be used to tune the values
    # When looking at the colours in an image, the colour channels are always labelled as RGB
    # OpenCv uses BGR image format, so when passing through colours the channels are actually mapped as:
    # B = Hue
    # G = Sat
    # R = Val

    # Yellow - H=30
    self.hsvYellowLow=(20.0000, 100.0000, 150.0000)
    self.hsvYellowHigh=(30.0000, 255.0000, 255.0000)

    # Blue
    self.hsvBlueLow=(95.0000, 150.0000, 50.0000)
    self.hsvBlueHigh=(110,255,255)

    # Red - wraps around 0, but the red blocks are mostly in the 0-10 range
    self.hsvRedLow1=(0.0000, 100.0000, 100.0000)
    self.hsvRedHigh1=(15,255,255)
    self.hsvRedLow2=(177.0000, 100.0000, 100.0000)
    self.hsvRedHigh2=(180,255,255)

    # Use the openCV bridge
    self.bridge = CvBridge()

    # Create a subscriber
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    # Create a publisher
    self.cube_pub = rospy.Publisher('cubes',cubeArray,queue_size=10)




  # Cubes are detected in every frame that is sent by the camera.
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Using cv_image as the image
    # Convert to HSV colorspace
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Create a mask for each colour being tracked
    # mask = cv2.inRange(hsv_image, darkerColour, brighterColour)

    maskYellow = cv2.inRange(hsv_image, self.hsvYellowLow, self.hsvYellowHigh)
    maskBlue = cv2.inRange(hsv_image, self.hsvBlueLow, self.hsvBlueHigh)

    # Red is the difficult case, as it wraps around the zero point

    maskRed1 = cv2.inRange(hsv_image, self.hsvRedLow1, self.hsvRedHigh1)
    maskRed2 = cv2.inRange(hsv_image, self.hsvRedLow2, self.hsvRedHigh2)
    maskRed = maskRed1+maskRed2

    # Erode then Dilate the image to remove small elements
    erodedMaskRed=erode(maskRed, 11)
    erodedMaskBlue=erode(maskBlue, 11)
    erodedMaskYellow=erode(maskYellow, 11)    
    
    dilatedMaskRed=dilate(erodedMaskRed, 11)
    dilatedMaskBlue=dilate(erodedMaskBlue, 11)
    dilatedMaskYellow=dilate(erodedMaskYellow, 11)


    # Minimum area of objects to find in pixels
    minArea=1000

    # Draw onto the original image
    canvas=cv_image

    # Find the objects in each mask - colours are BGR - draw on the "canvas"
    canvas,cubeListRed = mask2box(dilatedMaskRed,(0,0,255),canvas,minArea)
    canvas,cubeListBlue = mask2box(dilatedMaskBlue,(255,0,0),canvas,minArea)
    canvas,cubeListYellow = mask2box(dilatedMaskYellow,(0,255,255),canvas,minArea)


    cv2.imshow("Detected Objects", cv_image)
    cv2.waitKey(3) # This redraws the window

    # Get the size of the image to normalise the output
    (rows,cols,channels) = cv_image.shape

    # Return the cube data
    returnCubeArray=cubeArray()
    
    for c in range(len(cubeListRed)):
      tempCube=cubeData()
      tempCube.cube_colour='red'
      tempCube.area=cubeListRed[c].area
      tempCube.normalisedCoordinateX=cubeListRed[c].centreX/cols
      tempCube.normalisedCoordinateY=cubeListRed[c].centreY/rows
      returnCubeArray.cubes.append(tempCube)

    for c in range(len(cubeListBlue)):
      tempCube=cubeData()
      tempCube.cube_colour='blue'
      tempCube.area=cubeListBlue[c].area
      tempCube.normalisedCoordinateX=cubeListBlue[c].centreX/cols
      tempCube.normalisedCoordinateY=cubeListBlue[c].centreY/rows
      returnCubeArray.cubes.append(tempCube)

    for c in range(len(cubeListYellow)):
      tempCube=cubeData()
      tempCube.cube_colour='yellow'
      tempCube.area=cubeListYellow[c].area
      tempCube.normalisedCoordinateX=cubeListYellow[c].centreX/cols
      tempCube.normalisedCoordinateY=cubeListYellow[c].centreY/rows
      returnCubeArray.cubes.append(tempCube)   

    try:
      self.cube_pub.publish(returnCubeArray)
    except CvBridgeError as e:
      print(e)



def main(args):
  ic = cubeSpotter()
  rospy.init_node('cube_spotter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
