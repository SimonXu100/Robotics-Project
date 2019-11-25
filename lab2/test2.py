#!/usr/bin/env python
'''
bug2.py
author: Shusen Xu
Description: bug2 algorithm for path planning
Methods:
1 use the nav_msgs/Odometry messages to update your position of the robot
2 a laser scanner (Kinect-like device) that can be used to sense when obstacles are near
'''

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
import math

class Bug2:
    def __init__(self):
        rospy.init_node('Bug2', anonymous=False)
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # How fast will we update the robot's movement?
        self.rate = 20

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
        # Set the forward linear speed to 0.15 meters per second
        self.linear_speed = 0.5

        # Set the rotation speed in radians per second
        self.angular_speed = 0.25

        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(1.0)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'

        # Initialize the position variable as a Point type
        # record the current position
        self.position = Point()
        self.rotation = Point()

        # laser information
        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.scan_ranges = [0, 0, 0, 0]

        # hit point
        self.hit_point_previous = Point()
        self.hit_point_current = Point()

        # the goal point
        self.goal_point = Point()
        self.goal_point.x = 10

        # the starting point
        self.starting_point = Point()

        # the small delta movement
        self.delta_movement = 0.2
        # the delta angle of turn right
        self.delta_right = 0.25
        # the delta angle of turn left
        self.delta_left = 0.25

        # the delta distance of hit point from the obstacle
        self.delta_hit = 1
        rospy.sleep(2)
        print("self_scange ", self.scan_ranges)


    def bug2(self):

        while not rospy.is_shutdown() and not self.is_reach_goal():
            # states:
            # 0: following m line
            # 1: following obstacle/ wall
            # 2: goal reached
            # 3: goal cannot be reached
            
            state = 0
            if state == 0:
                state = self.follow_m_line()

        '''
            elif state == 1:
                state = self.follow_wall()
            elif state == 2:
                print ("Goal reached!")
                return
            elif state == 3:
                print ("Goal can NOT be reached!")
                return
            '''
                

    def follow_m_line(self):
    # follow_m_line state
    # This function first rotates the robot so it faces the goal
    # Then, it moves the robot forward, until it reaches the goal or until it faces obstacle
    # Last, it returns the next state, 1 for obstacle, 2 for goal reached
    
        print ("At position x:", round(self.position.x, 2), " y:", round(self.position.y,2))
        print ("At orientation:", round(self.rotation, 4))
        print ("Start following the m-line \n")
        
        # First, rotate the robot so it faces the goal
        (self.position, self.rotation) = self.get_odom()
        move_cmd = Twist()
        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        # allow 5 degrees of error
        while abs(self.rotation) > 0.03:
            #print ("Turned left to aim for the goal.")
            self.cmd_vel.publish(move_cmd)
            (self.position, self.rotation) = self.get_odom()
            rospy.sleep(1)
        print ("At position x:", round(self.position.x, 2), " y:", round(self.position.y,2))
        print ("At orientation:", round(self.rotation, 4))
        print("Robot oriented towards the goal. \n")
        
        # Then, move the robot forward, until it reaches the goal or until it faces obstacle
        move_cmd = Twist()
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        # Use a loop to move forward until goal hit or obstacle hit
        while True:
            (self.position, self.rotation) = self.get_odom()
            if abs(self.position.x - 10) < 0.1:
                # Goal reached
                print ("At position x:", round(self.position.x, 2), " y:", round(self.position.y,2))
                print ("At orientation:", round(self.rotation, 4))
                print ("Goal reached, State 2! \n")
                return 2

            else:
                # Keep going forward
                self.cmd_vel.publish(move_cmd)
                rospy.sleep(1)
        return 2
            
    #################################################################
        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        (self.position, self.rotation) = self.get_odom()
        print(self.position)
        ticks = int(abs(self.rotation) / self.angular_speed * self.rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # stop before move forward
        print("stop a while")
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        while not rospy.is_shutdown():
	    (self.position, self.rotation) = self.get_odom()
	    print("position:",self.position)
	    print("rotation:",self.rotation)
	    print("scan_ranges: " , self.scan_ranges[0])
            if self.scan_ranges[0] < self.delta_hit:
                move_cmd = Twist()
                self.cmd_vel.publish(move_cmd)
                rospy.sleep(1)
                print("hit the obstacles")
                (self.position, self.rotation) = self.get_odom()
                self.hit_point_previous = self.hit_point_current
                self.hit_point_current = self.position
                return 1

            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_vel.publish(twist)
            self.r.sleep()

    def is_reach_goal(self):
        (self.position, self.rotation) = self.get_odom()
        if abs(self.position.x - 10) < 0.0075:
            print("reach the goal")
            return True

    def is_reach_m_line(self):
        (self.position, self.rotation) = self.get_odom()
        if abs(self.position.y) < 0.0005 and abs(self.position.z) < 0.0005:
            self.hit_point_previous = self.hit_point_current
            self.hit_point_current = self.position
            print("reach m line again")
            return True
        return False


    def follow_wall(self):
        # turn left util you cannot detect the right
        rate = 20
        r = rospy.Rate(rate)
        print("scan2 :", self.scan_ranges[3])
        while not rospy.is_shutdown():
            print("scan_range[2]: ", self.scan_ranges[3])
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel.publish(twist)
            if math.isnan(self.scan_ranges[3]):
                break
            r.sleep()

        # stop before next movement
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        # follow the wall util reach the goal or recounter the m_line
        
        while not rospy.is_shutdown():

            if self.is_reach_goal():
                # reach the goal, exit
                print("reach the goal")
                return -1
            if self.is_reach_m_line():
                # this point are changeable accroding to the final result
                # recounter the same hit points
                if abs(self.hit_point_previous.x - self.hit_point_current.x) < 0.005:
                    # reach the same point, exit and return unreachable
                    print("recounter the same hit point, the goal is unreachable")
                    return -2
                if abs(self.hit_point_current.x - self.goal_point.x) < abs(
                        self.hit_point_previous.x - self.goal_point.x):
                    # should follow the m line
                    print('recounter more closer points ')
                    return 0

            # move forward for a small distance
            twist = Twist()
            twist.linear.x = self.linear_speed
            # self.delta_movement
            ticks = int(self.delta_movement / self.linear_speed * rate)
            for t in range(ticks):
                self.cmd_vel.publish(twist)
                r.sleep()

            # stop before next movement
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            print("scan_range2: ", self.scan_ranges[2])
            # if object not detected on right, then turn slightly right

            if math.isnan(self.scan_ranges[2]):
                twist = Twist()
                twist.angular.z = -self.angular_speed
                # self.delta_movement
                ticks = int(self.delta_right / self.angular_speed * rate)
                for t in range(ticks):
                    self.cmd_vel.publish(twist)
                    r.sleep()

            # stop before next movement
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            print("scan_0: ", self.scan_ranges[0])
            print("scan_2: ", self.scan_ranges[3])
            # if object close on right, turn left and move forward
            if not math.isnan(self.scan_ranges[3]) and self.scan_ranges[3] < 0.5:
                twist = Twist()
                twist.angular.z = self.angular_speed
                # self.delta_movement
                ticks = int(self.delta_left / self.angular_speed * rate)
                for t in range(ticks):
                    self.cmd_vel.publish(twist)
                    r.sleep()

            # stop before next movement
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)


    def scan_callback(self, msg):
        closet_range = min(msg.ranges)

        ahead_range = msg.ranges[len(msg.ranges) / 2]
        right_range = msg.ranges[len(msg.ranges) - 1]
        left_range = msg.ranges[0]
        self.scan_ranges = [closet_range, ahead_range, right_range, left_range]

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
'''
if __name__ == '__main__':
    try:
        bug2 = Bug2()
        bug2.bug2()
    except:
        rospy.loginfo("Bug2 node terminated.")
'''
bug2 = Bug2()
bug2.bug2()
