#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import argparse

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    # stop or not
    self.flag = 0
    # the x-coordinate centroid
    self.cx_marker = 0

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([15,  10,  10])
    upper_yellow = numpy.array([35, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)

    # lower red marker
    search_top1 = 3*h/5
    search_bot1 = 3*h/5 + 180
    lower_red = numpy.array([0, 10, 10])
    upper_red = numpy.array([10, 255, 255])
    mask_r = cv2.inRange(hsv, lower_red, upper_red)
    #mask_r = cv2.erode(mask_r, None, iterations=2)
    #mask_r = cv2.dilate(mask_r, None, iterations=2)
    mask_r_2 = mask_r.copy()
    mask_r_2[0:search_top1, 0:w] = 0
    mask_r_2[search_bot1:h, 0:w] = 0
    mask_r[0:search_top, 0:w] = 0
    mask_r[search_bot:h, 0:w] = 0
    # bigger crop for shape detection
    M_R = cv2.moments(mask_r)

    #print("the value of M[00]", M_R['m00'])
    if self.flag == 0:
      if M_R['m00'] > 800000:
        print("the moments of M_R[m00]", M_R['m00'])
        cnts = cv2.findContours(mask_r_2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        c = max(cnts, key=cv2.contourArea)
        M_R_2 = cv2.moments(c)
        cx_m = int(M_R_2['m10']/M_R_2['m00'])
        self.cx_marker = cx_m
        shape, number = self.detect(c)
        print("the number of :", number)
        if number == 3:
          print("find the shape is triangle")
          # decide pointing direction
          (x, y), (MA, ma), angle = cv2.fitEllipse(c)
          print("the value of angle ", angle)
          if 90 < angle:
            self.twist.linear.x = 1
            self.twist.angular.z = 1
          else:
            # point to left, turn left
            self.twist.linear.x = 1
            self.twist.angular.z = -1
        elif number >= 4:
          print("find the shape is star")
          self.twist.linear.x = 0.0
          self.twist.angular.z = 0.0
          self.cmd_vel_pub.publish(self.twist)
          self.flag = 1

      elif M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        # END CONTROL

    elif self.flag == 1:
      cnts = cv2.findContours(mask_r_2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      c = max(cnts, key=cv2.contourArea)
      M_R_2 = cv2.moments(c)
      cx_m = int(M_R_2['m10']/M_R_2['m00'])
      self.cx_marker = cx_m
      err2 = self.cx_marker-w/2
      print("the value of err2", err2)
      if abs(err2) > 5:
        self.twist.linear.x = 0.0
        self.twist.angular.z = -float(err2) / 100
      else:
        self.flag = 2

    elif self.flag == 2:
      # move to the goal points
      self.twist.linear.x = 0.2
      self.twist.angular.z = 0.0
      r = rospy.Rate(50)
      duration = abs(1.2) / self.twist.linear.x
      ticks = int(duration * 50)
      for i in range(ticks):
        self.cmd_vel_pub.publish(self.twist)
        r.sleep()
      self.flag = 3
      self.twist.linear.x = 0.0
    self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)


  def detect(self, c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    # if the shape is a triangle, it will have 3 vertices
    if len(approx) == 3:
        shape = "triangle"

    # if the shape is a pentagon, it will have 5 vertices
    elif len(approx) == 5:
        shape = "pentagon"
    # otherwise, we assume the shape is a circle
    else:
        shape = "star"
    # return the name of the shape
    return shape, len(approx)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
