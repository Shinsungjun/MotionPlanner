#!/usr/bin/env python

import rospy
import cv2

import numpy as np

#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

class Map(object):
  def __init__(self):
  #rospySubscribe to the map topic, the second is the data type, the third is the callback function
  # Pass the subscribed data to the callback function, which is the mapmsg variable
  #If a topic comes, call the callback function directly
    self.map_sub = rospy.Subscriber("/map",OccupancyGrid, self.callback)
    print("get map~")
    #The output below is the address, not the data
    print(self.map_sub)

#The definition of the callback function, passed mapmsg
  def callback(self,mapmsg):
    try:
      print("into callback")
      #Mainly want to get data, here is stored map information
      map = mapmsg.data
      # Below is the tuple type
      print(type(map))
      #Change to numpy format that can draw pictures
      map = np.array(map)
      #The following output is (368466,), obviously can not draw
      print(map.shape)
      #Need to reshape, factor the above numbers online, then calculate the two largest factors
      #So it's probably like this:
      map = map.reshape((2248,3000)) #height , width
      print(map)
      #You can see that most of the values ​​are -1, so you need to regularize the values
      row,col = map.shape
      print(row,col)
      tem = np.zeros((row,col))
      for i in range(row):
        for j in range(col):
          if(map[i,j]==100):
             tem[i,j]=255
          else:
             tem[i,j]=map[i,j]
      print(map.shape)
      cv2.imshow("map",tem)
      cv2.waitKey(0)
#      plt.imshow(map)
#      plt.show()
    except Exception:
      print(e)
      rospy.loginfo('convert rgb image error')

  def getImage():
    return self.rgb_image

def main(_):
  rospy.init_node('map',anonymous=True)
  v=Map()
  rospy.spin()

if __name__=='__main__':
  main('_')