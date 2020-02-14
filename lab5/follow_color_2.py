#!/usr/bin/env python
# BEGIN ALL
# check different color
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

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
    self.error = 0
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # the original color masks
    lower_yellow = numpy.array([15,  10,  10])
    upper_yellow = numpy.array([35, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # green marker
    lower_green = numpy.array([40, 20, 6])
    upper_green = numpy.array([70, 255, 255])
    mask_g = cv2.inRange(hsv, lower_green, upper_green)
    
    # blue marker
    lower_blue = numpy.array([80, 10, 10])
    upper_blue = numpy.array([125, 255, 255])
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # lower red marker
    lower_red = numpy.array([0, 10, 10])
    upper_red = numpy.array([10, 255, 255])
    mask_r_lower = cv2.inRange(hsv, lower_red, upper_red)

    # upper red marker
    lower_red_2 = numpy.array([160, 10, 10])
    upper_red_2 = numpy.array([179, 255, 255])
    mask_r_upper = cv2.inRange(hsv, lower_red, upper_red)
    mask_r = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    mask_g[0:search_top, 0:w] = 0
    mask_g[search_bot:h, 0:w] = 0

    mask_b[0:search_top, 0:w] = 0
    mask_b[search_bot:h, 0:w] = 0

    mask_r_lower[0:search_top, 0:w] = 0
    mask_r_lower[search_bot:h, 0:w] = 0

    mask_r_upper[0:search_top, 0:w] = 0
    mask_r_upper[search_bot:h, 0:w] = 0

    # the moment of yellow
    M = cv2.moments(mask)

    # the moment of green
    M_G = cv2.moments(mask_g)
    M_B = cv2.moments(mask_b)
    M_R_L = cv2.moments(mask_r_lower)
    M_R_U = cv2.moments(mask_r_upper)
    M_R = cv2.moments(mask_r)

    # new modified
    search_top1 = 3*h/5
    search_bot1 = 3*h/5 + 180
    mask_r_2 = mask_r_lower.copy()
    mask_r_2[0:search_top1, 0:w] = 0
    mask_r_2[search_bot1:h, 0:w] = 0


    if self.flag == 0:
      if M_G['m00'] > 800000:
        self.twist.linear.x = 2.0
        self.twist.angular.z = 2.0

      elif M_B['m00'] > 800000:
        self.twist.linear.x = 2.0
        self.twist.angular.z = -2.0

      elif (M_R_L['m00'] > 800000) or (M_R_U['m00']) > 800000:
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.flag = 1
        print("find the goal")

      elif M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
    # END CONTROL
    # method 4: move a specified distance
    if self.flag == 1:
      cnts = cv2.findContours(mask_r_2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      c = max(cnts, key=cv2.contourArea)
      M_R_2 = cv2.moments(c)
      # correct the goal direction
      cx_m = int(M_R_2['m10']/M_R_2['m00'] * 0.8)
      #cx_m = int(M_R_2['m10']/M_R_2['m00'])
      err_r = cx_m - w/2

      if abs(err_r) > 2:
        self.twist.linear.x = 0.0
        self.twist.angular.z = -float(err_r) / 100
      else:
        self.flag = 2

    if self.flag == 2:
      self.twist.linear.x = 0.2
      self.twist.angular.z = 0.0
      r = rospy.Rate(50)
      duration = abs(1.2) / self.twist.linear.x
      ticks = int(duration * 50)
      for i in range(ticks):
        self.cmd_vel_pub.publish(self.twist)
        r.sleep()
      self.flag = 3
      print("reach the goal!!!")
      self.twist.linear.x = 0.0
    self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
