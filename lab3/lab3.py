#!/usr/bin/env python
'''
lab3.py
author: Shusen Xu, Zhilin Guo
Description: lab3 program that finds the best path to the goal, and guides the robot to the goal
'''
import numpy as np
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import math
import heapq
from visualization_msgs.msg import Marker, MarkerArray

# libraries required to run the robot
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

def load_obstacles(object_path):
    '''
    Function to load a list of obstacles.
    The obstacle txt file show points in clockwise order

    Return:
        3d list [[[1, 2], [3, 4], [5, 6]],
                        [[7, 8], [9, 10], [10, 11]]]
    '''
    obstacles = []
    obstacle = []
    with open(object_path) as f:
        numObstacles = int(f.readline())
        coordinates = int(f.readline())
        for i in range(coordinates):
            line = f.readline()
            obstacle.append(list(map(int, line.strip().split(' '))))
        for line in f:
            coordinates = list(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                obstacles.append(obstacle)
                obstacle = []
            else:
                obstacle.append(coordinates)
    obstacles.append(obstacle)
    assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
    return obstacles

def load_goal(goal_path):
    with open(goal_path) as f:
        line = f.readline()
        goal = list(map(int, line.strip().split(' ')))
    return goal
""" 
------------------------------------------
(0, 0) is at the center of the map;
(0, 0) is at the top left of the image

Thus, we need some transformation
------------------------------------------
"""

def map2img(ob):
    """ transform an obstacle in map frame to img frame """
    ob_tranformed = []
    t = np.array([[1, 0, 0, 300],
                    [0, -1, 0, 300],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])
    for p in ob:
        p = np.array(p) # in case it is not already numpy array
        p = np.hstack((p, [0, 1]))
        p = t.dot(p).astype(int)
        ob_tranformed.append(p[:2])
    return np.array(ob_tranformed)

def img2map(ob):
    """ transform an obstacle in img frame to map frame """
    ob_tranformed = []
    t = np.array([[1, 0, 0, -300],
                    [0, -1, 0, 300],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])
    for p in ob:
        p = np.array(p) # in case it is not already numpy array
        p = np.hstack((p, [0, 1]))
        p = t.dot(p).astype(int)
        ob_tranformed.append(p[:2])
    return np.array(ob_tranformed)

""" OUR OWN METHODS START HERE """

def find_fence(ob, effective_radius):
    """
    helper function that returns the coordinates of the fence vertex
    takes in an array of vertices on obstacle, and the effective radius of robot
    returns a numpy array of fence vertices in counterclock order
    """
    # initialize variables
    print "Obstacle coordinates: \n", ob, "\n"
    coord1 = ob[0][0]
    coord2 = ob[0][1]
    first_point_array = np.array([[coord1, coord2]])
    all_points = np.array(first_point_array)
    print "all_points initial value: \n", all_points, "\n"

    # iterate over each vertex of the box and
    for i in range(len(ob)):
        #print(ob[i])
        coord1 = ob[i][0]
        coord2 = ob[i][1]
        # compute the 4 points surrounding each vertex of an obstacle
        # top left
        top_left = [[coord1 - effective_radius, coord2 + effective_radius]]
        top_left_array = np.array(top_left)
        all_points = np.concatenate((all_points, top_left_array))
        # top right
        top_right = [[coord1 + effective_radius, coord2 + effective_radius]]
        top_right_array = np.array(top_right)
        all_points = np.concatenate((all_points, top_right_array))
        # bottom left
        bot_left = [[coord1 - effective_radius, coord2 - effective_radius]]
        bot_left_array = np.array(bot_left)
        all_points = np.concatenate((all_points, bot_left_array))
        # bottom right
        bot_right = [[coord1 + effective_radius, coord2 - effective_radius]]
        bot_right_array = np.array(bot_right)
        all_points = np.concatenate((all_points, bot_right_array))
    print "All collision detection points: \n", all_points, "\n"
    # find the ConvexHull of the fence
    hull = ConvexHull(all_points)
    print "ConvexHull indices:", hull.vertices, "\n"
    # initialize the result
    result = [all_points[hull.vertices[0]]]
    hull_indices = hull.vertices
    for i in range(1, len(hull_indices)):
        result = np.concatenate((result, [all_points[hull.vertices[i]]]))
    print "Fence vertices: \n", result, "\n"
    return result

def intersect_point_helper(segment_1, segment_2):
    """
    helper function that calculates the intersect point of two line segments
    :param input_tuple1: the first segment tuple (array, array)
    :param input_tuple2: the second segment tuple (array, array)
    :return: the intersect coordinate array
    """
    # disassemble the tuple
    a1 = segment_1[0]
    a2 = segment_1[1]
    b1 = segment_2[0]
    b2 = segment_2[1]
    # calculate the line function
    s = np.vstack([a1,a2,b1,b2])
    h = np.hstack((s, np.ones((4, 1))))
    line1 = np.cross(h[0], h[1])
    line2 = np.cross(h[2], h[3])
    # finding the intersect point
    x, y, z = np.cross(line1, line2)
    if z == 0:
        # two segments are parallel
        return None
    else:
        # intersect valid
        x_coord = x/z
        y_coord = y/z
        return (x_coord, y_coord)

def segments_intersection_helper(segment_1, segment_2):
    """
    helper function that checks whether two line segments intersect
    :param input_tuple1: the first segment tuple (array, array)
    :param input_tuple2: the second segment tuple (array, array)
    :return: True if two segments intersect, False otherwise
    """
    # find intersect using helper function
    intersect = intersect_point_helper(segment_1, segment_2)
    # return False and terminate if two lines are parallel
    if intersect == None:
        return False
    # disassemble the tuple
    a1 = segment_1[0]
    a2 = segment_1[1]
    b1 = segment_2[0]
    b2 = segment_2[1]
    (x_coord, y_coord) = intersect
    #print ("intersect: ", x_coord, y_coord)
    # check if intersect on four endpoints of segments
    # if intersect on endpoint, then the two segments are not intersecting
    # each other because they are both valid
    if intersect == (a1[0], a1[1]):
        return False
    elif intersect == (a2[0], a2[1]):
        return False
    elif intersect == (b1[0], b1[1]):
        return False
    elif intersect == (b2[0], b2[1]):
        return False
    # check if intersect NOT on segment 1
    if not (min(a1[0], a2[0]) <= x_coord <= max(a1[0], a2[0]) and
            min(a1[1], a2[1]) <= y_coord <= max(a1[1], a2[1])):
        return False
    # check if intersect on segment 2
    if not (min(b1[0], b2[0]) <= x_coord <= max(b1[0], b2[0]) and
            min(b1[1], b2[1]) <= y_coord <= max(b1[1], b2[1])):
        #print ("False 2")
        return False
    return True

def check_if_cross_fence (segment, fence_edge_list):
    """
    Helper function that checks if the given line segment crosses any edge in fence_edge_list
    :param segment: tuple of np.array
    :param fence_edge_list: list of tuples of np.array
    :return: True if segment crosses any fence_edge, False otherwise
    """
    for edge in fence_edge_list:
        if segments_intersection_helper(segment, edge) == True:
            return True
    return False

def points_distance (point1, point2):
    """
    Helper function that calculates the distance between two point tuples
    :param point1: (x, y) tuple
    :param point2: (x, y) tuple
    :return: distance between two points
    """
    (x1, y1) = point1
    (x2, y2) = point2
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist

def A_star_helper (start_tuple, goal_tuple, next_vertex_dict):
    """
    Helper function that uses A* to compute the best path from start to goal
    :param start_tuple: (x, y) of starting point
    :param goal_tuple: (x, y) of goal point
    :param next_vertex_dict: dictionary of [(x, y): [(x, y), ... ,(x, y)]] representing what a point connects to
    :return:
    """
    # initialize the frontier heap queue, frontier set and expanded set
    frontier_heap = []
    heapq.heapify(frontier_heap)
    frontier = set()
    expanded = set()

    # initiate the path dictionary, key is point tuple, value is parent point
    path_dict = {}

    # add the start node to frontier
    f = 0 + points_distance (start_tuple, goal_tuple)
    heapq.heappush(frontier_heap, [f, start_tuple, 0])
    frontier.add(start_tuple)

    # loop until goal found
    while True:
        if (378.0, 72.0) in frontier:
            print "(378.0, 72.0) in frontier"
        #print "current frontier:", frontier_heap
        # get current node
        current = heapq.heappop(frontier_heap)
        (f, current_point, dist) = (current[0], current[1], current[2])
        #print "currently popped heap:", current
        #print "frontier: ", frontier, "\n"
        frontier.remove(current_point)
        # add current point to expanded set
        expanded.add(current_point)
        # check is current is goal
        if current_point == goal_tuple:
            #print "goal found \n"
            break
        # generate children
        children = next_vertex_dict[current_point]
        for child in children:
            # if child on expanded set, continue
            if child in expanded:
                continue
            # create the f, g, and h values
            g = dist + points_distance (current_point, child)
            h = points_distance (child, goal_tuple)
            f = g + h
            # child already in frontier, compare it to stored value
            if child in frontier:
                child_cache = heapq.heappop(frontier_heap)
                frontier.remove(child)
                # current child is no better than point in frontier
                # put the point in frontier back
                if f >= child_cache[0]:
                    heapq.heappush(frontier_heap, child_cache)
                    frontier.add(child)
                    continue
            # add child to frontier
            #print "adding to dictionary", child, current_point, "\n"
            path_dict[child] = current_point
            heapq.heappush(frontier_heap, [f, child, g])
            frontier.add(child)
    print "backtracking dictionary:", path_dict, "\n"

    # construct the result list
    next = goal_tuple
    result = [next]
    # backtrack through the steps
    while True:
        prev = path_dict[next]
        if prev == start_tuple:
            break
        result = [prev] + result
        next = prev
    return result

def convert_to_rviz (tuple_list):
    """
    Helper function that converts an image point to Rviz's coordinate
    :param point: [(x, y), ..., (x, y)] in cm
    :return: [(x, y), ..., (x, y)] in m, after offset
    """
    result = []
    for (x, y) in tuple_list:
        x = float(x)
        y = float(y)
        x = round(x/100, 4)
        y = round(y/100, 4)
        result += [(x, y)]
    return result

# show_the_line
# a list of tuple: contains the positions
def show_line(edges):
    rospy.init_node('path_pub')
    pub_line = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=4)
    rospy.loginfo('Publishing possible path')
    marker_array = MarkerArray()
    print("the length of edges", len(edges))
    max = len(edges)

    count = 0
    while not rospy.is_shutdown() and count < 1000:

        for i in range(len(edges)):
            first_point = edges[i][0]
            second_point = edges[i][1]
            temp_marker = Marker()
            temp_marker.header.frame_id = "map"
            temp_marker.type = temp_marker.LINE_STRIP
            temp_marker.action = temp_marker.ADD

            # marker scale
            temp_marker.scale.x = 0.03
            temp_marker.scale.y = 0.03
            temp_marker.scale.z = 0.03

            # marker color
            temp_marker.color.a = 1.0
            temp_marker.color.r = 1.0
            temp_marker.color.g = 0.0
            temp_marker.color.b = 0.0

            # marker orientaiton
            temp_marker.pose.orientation.x = 0.0
            temp_marker.pose.orientation.y = 0.0
            temp_marker.pose.orientation.z = 0.0
            temp_marker.pose.orientation.w = 1.0

            # marker position
            temp_marker.pose.position.x = 0.0
            temp_marker.pose.position.y = 0.0
            temp_marker.pose.position.z = 0.0

            # marker line points
            temp_marker.points = []
            # first point
            first_line_point = Point()
            first_line_point.x = first_point[0] / 100.0
            first_line_point.y = first_point[1] / 100.0
            first_line_point.z = 0.0
            temp_marker.points.append(first_line_point)
            # second point
            second_line_point = Point()
            second_line_point.x = second_point[0] / 100.0
            second_line_point.y = second_point[1] / 100.0
            second_line_point.z = 0.0
            temp_marker.points.append(second_line_point)
            marker_array.markers.append(temp_marker)
            count = count + 1
            if count > max:
                marker_array.markers.pop(0)

            # Renumber the marker IDs
            id = 0
            for m in marker_array.markers:
                m.id = id
                id += 1

            pub_line.publish(marker_array)
        rospy.sleep(0.1)

#show the min_dist line
def show_min_dist_line(points_list):
    rospy.init_node('path_pub')
    pub_line_min_dist = rospy.Publisher('vgraph_markers', Marker, queue_size=1)
    rospy.loginfo('Publishing min dist line')
    count = 0
    while not rospy.is_shutdown() and count < 100:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        for i in range(len(points_list)):
            line_point = Point()
            line_point.x = points_list[i][0] / 100.0
            line_point.y = points_list[i][1] / 100.0
            line_point.z = 0.01
            marker.points.append(line_point)

        # Publish the Marker
        pub_line_min_dist.publish(marker)
        count = count + 1
        rospy.sleep(0.1)




def main(robot_length = 100, safe_distance = 10, start = [0, 0]):
    """ main function for lab3"""
    # initialize variables
    obstacles = load_obstacles("../data/world_obstacles.txt")
    goal = load_goal("../data/goal.txt")
    goal_array = np.array([goal])
    start_array = np.array([start]).astype(float)
    # goal_img = map2img(goal_array)
    # start_img = map2img(start_array)

    goal_img = goal_array
    start_img = start_array
    # dictionary that records vertices that any vertex connects to
    # key is (coord1, coord2)
    # value is list of (coord1, coord2) of next vertices
    next_vertex_dict = {}

    print "start on image frame:", start_img, "\n"
    print "goal on image frame:", goal_img, "\n"

    # effective radius is distance from obstacle to center of robot, including safe_distance
    effective_radius = robot_length/2 + safe_distance

    """ (1) Find fence around each obstacle """
    # records all existing vertices and fence edges according to each obstacle
    # all_fence_vertices is a list of list of np arrays, corresponding to each obstacle
    all_fence_vertices = []
    all_fence_edges = []
    # iterate over all obstacles
    """!!!uncomment the following range argument to iterate over all obstacles!!!"""
    for i in [0, 1, 2, 3, 4]: #range(len(obstacles)):
        each_fence_vertices = np.empty([0, 2]).astype(float)
        print "for obstacle:", i, "\n"
        ob = obstacles[i]
        # find all vertices on the current fence
        fence_vertices = find_fence(ob, effective_radius)
        # add these vertices to the each_fence_vertices array
        each_fence_vertices = np.concatenate((each_fence_vertices, fence_vertices))
        # iterate over all vertices on the current fence
        for j in range(len(fence_vertices)):
            current_coord1 = fence_vertices[j][0]
            current_coord2 = fence_vertices[j][1]
            if j == len(fence_vertices) - 1:
                # add the current edge tuple to the current_fence_edge list
                current_fence_edge = (fence_vertices[j], fence_vertices[0])
                # add the vertices that the current vertex connects to to the dictionary
                (last_coord1, last_coord2) = (fence_vertices[j-1][0], fence_vertices[j-1][1])
                (next_coord1, next_coord2) = (fence_vertices[0][0], fence_vertices[0][1])
                next_vertex_dict[(current_coord1, current_coord2)] = [(last_coord1, last_coord2), (next_coord1, next_coord2)]
            elif j == 0:
                current_fence_edge = (fence_vertices[j], fence_vertices[j+1])
                (last_coord1, last_coord2) = (fence_vertices[len(fence_vertices)-1][0], fence_vertices[len(fence_vertices)-1][1])
                (next_coord1, next_coord2) = (fence_vertices[j+1][0], fence_vertices[j+1][1])
                next_vertex_dict[(current_coord1, current_coord2)] = [(last_coord1, last_coord2), (next_coord1, next_coord2)]
            else:
                current_fence_edge = (fence_vertices[j], fence_vertices[j+1])
                (last_coord1, last_coord2) = (fence_vertices[j-1][0], fence_vertices[j-1][1])
                (next_coord1, next_coord2) = (fence_vertices[j+1][0], fence_vertices[j+1][1])
                next_vertex_dict[(current_coord1, current_coord2)] = [(last_coord1, last_coord2), (next_coord1, next_coord2)]
            all_fence_edges += [current_fence_edge]
        all_fence_vertices += [each_fence_vertices]

    print "All fence vertices: \n", all_fence_vertices, "\n"
    print "All fence edges: \n", all_fence_edges, "\n"

    """ (2) Visualize the fences """
    # for i in range(len(all_fence_vertices)):
    #     visualize_line(all_fence_vertices[i])

    show_line(all_fence_edges)
    rospy.sleep(2)


    """ (3) Fully connect start + fence vertices + goal """
    # initialize a list to record all independent edges
    independent_edges = []

    """ (3a) first connect start and goal to each vertex """
    # try to connect from start to foal
    current_cord1 = start_img[0][0]
    current_cord2 = start_img[0][1]
    # add start to dictionary
    next_vertex_dict[(current_cord1, current_cord2)] = []
    vertex2 = goal_img[0]
    current_segment = (start_img[0], vertex2)
    # test if current segment is valid
    if not check_if_cross_fence(current_segment, all_fence_edges):
        # current segment is valid, try to add to dictionary and connected edges
        if (vertex2[0], vertex2[1]) not in next_vertex_dict[(current_cord1, current_cord2)]:
            # current coordinate tuple not in dictionary list, add it
            next_vertex_dict[(current_cord1, current_cord2)] += [(vertex2[0], vertex2[1])]
            independent_edges += [current_segment]

    # connect from start to vertices
    current_cord1 = start_img[0][0]
    current_cord2 = start_img[0][1]
    # iterate over all obstacles
    for i in range(len(all_fence_vertices)):
        # iterate over all vertices in each obstacle
        for j in range(len(all_fence_vertices[i])):
            vertex2 = all_fence_vertices[i][j]
            current_segment = (start_img[0], vertex2)
            # test if current segment is valid
            if not check_if_cross_fence(current_segment, all_fence_edges):
                # current segment is valid, try to add to dictionary and connected edges
                if (vertex2[0], vertex2[1]) not in next_vertex_dict[(current_cord1, current_cord2)]:
                    # current coordinate tuple not in dictionary list, add it
                    next_vertex_dict[(current_cord1, current_cord2)] += [(vertex2[0], vertex2[1])]
                    independent_edges += [current_segment]

    # connect from vertices to goal
    current_cord1 = goal_img[0][0]
    current_cord2 = goal_img[0][1]
    # iterate over all obstacles
    for i in range(len(all_fence_vertices)):
        # iterate over all vertices in each obstacle
        for j in range(len(all_fence_vertices[i])):
            vertex1 = all_fence_vertices[i][j]
            current_segment = (vertex1, goal_img[0])
            # test if current segment is valid
            if not check_if_cross_fence(current_segment, all_fence_edges):
                # current segment is valid, try to add to dictionary and connected edges
                if (current_cord1, current_cord2) not in next_vertex_dict[(vertex1[0], vertex1[1])]:
                    # current coordinate tuple not in dictionary list, add it
                    # add from the vertex to the goal
                    next_vertex_dict[(vertex1[0], vertex1[1])] += [(current_cord1, current_cord2)]
                    independent_edges += [current_segment]

    print "After adding start and goal:"
    print "Number of independent edges from start/goal to obstacle vertices:", len(independent_edges)
    print "These edges are:", independent_edges, "\n"

    """ (3b) then connect each obstacle's vertices to other obstacle's vertices"""
    # iterate over all obstacles
    for i in range(len(all_fence_vertices)):
        # iterate over all vertices of the obstacle
        for vertex1 in all_fence_vertices[i]:
            # iterate over all other obstacles
            for j in range(len(all_fence_vertices)):
                if i == j:
                    continue
                # iterate over all vertices of the obstacle
                for vertex2 in all_fence_vertices[j]:
                    # construct a segment
                    current_segment = (vertex1, vertex2)
                    # test if current segment is valid
                    if not check_if_cross_fence(current_segment, all_fence_edges):
                        # current segment is valid, try to add to dictionary and connected edges
                        current_cord1 = vertex1[0]
                        current_cord2 = vertex1[1]
                        if (vertex2[0], vertex2[1]) not in next_vertex_dict[(current_cord1, current_cord2)]:
                            # current coordinate tuple not in dictionary list, add it
                            next_vertex_dict[(current_cord1, current_cord2)] += [(vertex2[0], vertex2[1])]
                            independent_edges += [current_segment]
    print "After adding all remaining vertices:"
    print "Number of independent edges in whole image:", len(independent_edges)
    print "These edges are:", independent_edges, "\n"

    """ (4) Visualize the connections made """
    # for i in range(len(independent_edges)):
    #     visualize_line(independent_edges[i])

    # create by shusen
    final_possible_path = independent_edges + all_fence_edges
    show_line(final_possible_path)


    """ (5) Use A* search to find the best path """
    start_tuple = (start_img[0][0], start_img[0][1])
    goal_tuple = (goal_img[0][0], goal_img[0][1])
    path_list = A_star_helper(start_tuple, goal_tuple, next_vertex_dict)
    print "The shortest path is:", path_list, "\n"

    """ (6) Visualize the best path """
    #added by shusen

    original_vertice = (0, 0)
    final_min_path = path_list
    final_min_path.insert(0, original_vertice)
    print("the final_min_path  ", final_min_path)
    show_min_dist_line(final_min_path)


    """ (7) Control the robot to follow the path """
    rviz_tuple_list = convert_to_rviz(path_list)
    print (rviz_tuple_list)
    print "Creating robot class. Passing points to robot. \n"
    robot = Robot()
    robot.follow_points(rviz_tuple_list)

    print "Robot reached the goal.\nTerminating...\n"


class Robot:
    def __init__(self):
        rospy.init_node('path_pub', anonymous=False)
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # How fast will we update the robot's movement?
        self.rate = 20
        r = rospy.Rate(self.rate)

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
        # Set the forward linear speed to 0.15 meters per second
        self.linear_speed = 0.2

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

        # the goal point
        self.goal_point = Point()
        self.goal_point.x = 0
        self.goal_point.y = 0

        # the starting point
        self.starting_point = Point()

        # the small delta movement
        self.delta_movement = 0.2
        # the delta angle of turn right
        self.delta_right = 0.25
        # the delta angle of turn left
        self.delta_left = 0.25

        print "Robot initialized. \n"
        rospy.sleep(2)


    def follow_points(self, point_list):
        """
        Method that call go_to_point iteratively to command the robot to follow a list of points
        :param tuple_list:
        :return: nothing
        """
        for (x, y) in point_list:
            self.go_to_point((x, y))

    def go_to_point(self, point):
        """
        Method that commands the robot to go to the input tuple coordinate
        :param point: (x, y)
        :return: nothing
        """
        # change the goal
        (x, y) = point
        self.goal_point.x = x
        self.goal_point.y = y
        print "Robot going to point: ", point, "\n"
        print (self.goal_point)
        # get current position and orientation
        (self.position, self.rotation) = self.get_odom()

        # First, rotate the robot so it faces the goal
        # Initiate some movement commands of different directions
        move_cmd_left = Twist()
        move_cmd_right = Twist()
        move_cmd_left.angular.z = self.angular_speed / 4
        move_cmd_right.angular.z = - self.angular_speed / 4

        radians_to_goal = math.atan2(self.goal_point.y - self.position.y, self.goal_point.x - self.position.x)

        # Turn the robot towards the goal
        print "Orienting the robot towards the goal"
        # When goal is to the right of robot
        desired_turn = radians_to_goal - self.rotation

        print "Desired turn: ", round(desired_turn, 6), " \n"
        if desired_turn < 0:
            # turn right
            while True:
                radians_to_goal = math.atan2(self.goal_point.y - self.position.y, self.goal_point.x - self.position.x)
                desired_turn = radians_to_goal - self.rotation
                if desired_turn >= -0.03:
                    break
                # print "reorienting, turn remaining: ", round(desired_turn, 6)
                # speed up the turn if radian is big
                if desired_turn < - 0.5:
                    move_cmd_right.angular.z = - self.angular_speed
                else:
                    move_cmd_right.angular.z = - self.angular_speed / 4
                self.cmd_vel.publish(move_cmd_right)
                rospy.sleep(1)
                (self.position, self.rotation) = self.get_odom()
        elif desired_turn > 0:
            # turn left
            while True:
                radians_to_goal = math.atan2(self.goal_point.y - self.position.y, self.goal_point.x - self.position.x)
                desired_turn = radians_to_goal - self.rotation
                if desired_turn <= 0.03:
                    break
                # print "reorienting, turn remaining: ", round(desired_turn, 6)
                # speed up the turn if radian is big
                if desired_turn > 0.5:
                    move_cmd_left.angular.z = self.angular_speed
                else:
                    move_cmd_left.angular.z = self.angular_speed / 4
                self.cmd_vel.publish(move_cmd_left)
                rospy.sleep(1)
                (self.position, self.rotation) = self.get_odom()

        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        # print "At position x:", round(self.position.x, 2), " y:", round(self.position.y, 2)
        # print "At orientation:", round(self.rotation, 4)
        # print "Robot oriented towards the goal."
        print "Started going towards the goal. \n"

        # Then, move the robot forward until it reaches the goal
        (self.position, self.rotation) = self.get_odom()
        move_cmd = Twist()

        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        goal_distance = self.points_distance(self.position, self.goal_point)
        print "forward_distance: ", goal_distance, " \n"

        # How long should it take us to get there?
        linear_duration = abs(goal_distance) / self.linear_speed

        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            rospy.Rate(self.rate).sleep()

        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)

        print "Waypoint reached. \n"
        return


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def points_distance(self, point1, point2):
        """
        Helper function that calculates the distance between two point tuples
        :param point1: Point()
        :param point2: Point()
        :return: distance between two points
        """
        (x1, y1) = (point1.x, point1.y)
        (x2, y2) = (point2.x, point2.y)
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return dist

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


""" variables for the program """
robot_length = 36 #cm
safe_distance = 0 #cm
start = [0, 0]


""" start the program """
main(robot_length, safe_distance, start)

