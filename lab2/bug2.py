#!/usr/bin/env python
''' 
bug2.py 
author: Shusen Xu
Description: bug2 algorithm for path planning
Methods: 
1 use the nav_msgs/Odometry messages to update your position of the robot
2 a laser scanner (Kinect-like device) that can be used to sense when obstacles are near
'''
'''
status:
1 hit the obstacle
-1 reach the goal
-2 recounter the same hit point
0  recounter the closer hit point
'''


import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class Bug2:
	def __init__(self):



    def bug2(self):
    	while not rospy.is_shutdown() and not is_reach_goal():
    	    follow_m_line()
    	    status = follow_wall()
    	    if status == -1:
    	    	print "reach the goal"
    			return
    	    if status == -2:
    			return

    # follow the m_line, find the hit point and stop
    def fllow_m_line(self):
    	 # Stop the robot before the rotation
         move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # Set the movement command to a rotation
            move_cmd.angular.z = self.angular_speed
            (self.position, self.rotation) = self.get_odom()

            ticks = int(abs(self.rotation.z) * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            # stop before move forward
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            while not rospy.is_shutdown():
            	if (self.scan_sub[0] < self.delta_hit):
            		twist = Twist()
            		self.cmd_vel.publish(twist)
            		rospy.sleep(1)
            		print "hit the obstacles"
            		(self.position, self.rotation) = self.get_odom()
            		self.hit_point_previous = self.hit_point_current
            		self.hit_point_current = self.position
            		return 1

	            twist.linear.x = 1
	            self.cmd_vel.publish(twist)
	            r.sleep()



    def is_reach_goal(self):
    	(self.position, self.rotation) = self.get_odom()
    	if abs(self.position.x - 10) < 0.0075:
    		print "reach the goal"
    		return True;


    def is_reach_m_Line(self):
    	(self.position, self.rotation) = self.get_odom()
    	if abs(self.position.y)< 0.0075 and abs(self.position.z) < 0.0075:
    		self.hit_point_previous = self.hit_point_current
    		self.hit_point_current = self.position
    		print "reach m line again"
    		return True
    	return False


    def follow_wall(self):
    	# turn left util you cannot detect the right
    	rate = 20
    	r = rospy.Rate(rate)

        while not rospy.is_shutdown() and self.scan_ranges[2] is not None:
			twist = Twist()
        	twist.angular.z = self.angular_speed
        	self.cmd_vel.publish(move_cmd)
        	r.sleep()

        # stop before next movement
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        #follow the wall util reach the goal or recounter the m_line
        while not rospy.is_shutdown():
        	if self.is_reach_goal():
        		# reach the goal, exit
        		print "reach the goal"
        		return -1
        	if self.is_reach_m_Line():
        		# this point are changeable accroding to the final result
        		# recounter the same hit points
        		if abs(self.hit_point_previous.x - self.hit_point_current.x) < 0.05:
        			# reach the same point, exit and return unreachable
        			print "recounter the same hit point, the goal is unreachable"
        			return -2
        		if abs(self.hit_point_current.x - self.goal_point.x) < abs(self.hit_point_previous.x - self.goal_point.x):
        			# should follow the m line
        			print ('recounter more closer points ')
        			return 0

            # move forward for a small distance
            twist = Twist()
            twist.line.x = self.linear_speed
            # self.delta_movement
            ticks = int(self.delta_movement / self.linear_speed * rate)
            while not rospy.is_shutdown():
                for t in range(ticks):
                	self.cmd_vel.publish(twist)
                    r.sleep()

            # stop before next movement
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)


            # if object not detected on right, then turn slightly right
            if self.scan_ranges[2] is None:
            	twist = Twist()
                twist.angular.z = -self.angular_speed
                # self.delta_movement
                ticks = int(self.delta_right / self.angular_speed * rate)
                while not rospy.is_shutdown():
                for t in range(ticks):
                	self.cmd_vel.publish(twist)
                    r.sleep()

                # stop before next movement
                move_cmd = Twist()
                self.cmd_vel.publish(move_cmd)
                rospy.sleep(1)

            # if object close on right, turn left and move forward
            if self.scan_ranges[0] == self.scan_ranges[2]:
            	twist = Twist()
                twist.angular.z = self.angular_speed
                # self.delta_movement
                ticks = int(self.delta_left / self.angular_speed * rate)
                while not rospy.is_shutdown():
                for t in range(ticks):
                	self.cmd_vel.publish(twist)
                    r.sleep()

                # stop before next movement
                move_cmd = Twist()
                self.cmd_vel.publish(move_cmd)
                rospy.sleep(1)


    def scan_callback(self, msg):
        closet_range = min(msg.ranges)
        ahead_range = msg.ranges[len(msg.ranges)/2]
        right_range = msg.ranges[len(msg.ranges)-1]
        self.scan_ranges = [closet_range, ahead_range, right_range]


	def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))


    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        bug2 = Bug2()
        bug2.bug2()
    except:
        rospy.loginfo("Bug2 node terminated.")