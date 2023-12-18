#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
np.float = np.float64
from ros_numpy import numpify



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_converted", Image)

    self.bridge = numpify()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

  def callback(self):
    image = self.bridge(self)
    print(image)
    cv2.imshow("Image window", image)
    cv2.waitKey(3)

    self.image_pub.publish(image)

def main(arg):
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)