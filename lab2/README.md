# lab2
COMS 4733 Comp Aspects of Robotics Lab 2, Fall 2019
Shusen Xu
Zhilin Guo

1) Usage:
    To run the bug2.py script, first input into a terminal the command to launch Gazebo in bug world 0:

$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/zg2358/Desktop/lab2_worlds/bug2_0.world

    Then, in a new terminal, change into bug2.py's directory (/home/zg2358/catkin_ws/src/bug2/src on my computer), and enter
    
$ python bug2.py

    The python file will then control the robot and navigate throuth the world to find goal at (10, 0).
    
    Repeat the process for all other worlds by substituting their names in the command lines, namely:
    
    bug2_1.world, bug2_2.world, bug2_3.world, bug2_5.world, bug2_extra.world

    After testing the program with the worlds, the Python script will terminate on its own.
    
    Hit Ctrl + C to terminate the Gazebo simulator.

2) Method:

    The program employes the idea of a Finite State Machine with 4 states: 0) m-line-following state, 1) obstacle following state, 2) goal-reached state, and 3) no-solution state. 
    
    The program starts in the 0) m-line-following state, where it tries to align the robot with the direction of the goal by turning it in the correct direction incrementally. Then, it drives the robot forward by continuously sending the corresponding move command, until the robot
.detects any object in the direct front, front-left, or front-right direction. Then, the program transitions into state 1). Or if the robot detects that it has reached the goal, it transitions into state 2).

    In 1) obstacle following state, the robot first turns left until it's laser sensors sense nothing on the right or front. Then, the robot repeats three steps: going forward; turning right if there is no obstacle; and turning left plus moving if it is too close to obstacle. If it meets the m-line at a closer distance than last time before encountering a hitpoint that it already met in the past, it transitions into state 0). If it meets a previous hitpoint, it transitions into 3) to signify that a solution cannot be found.
    
    State 2) and 3) are exit states, where in 2) the goal is found by the robot, and in 3) a goal/solution cannot be found.
    
    The specific ROS methods we use include cmd_vel.publish(move_cmd), where we publish a Twist() message to control and relational and rotational movement of the robot; and rospy.Subscriber('/scan', LaserScan, self.scan_callback), where to subscribe to a continuous stream of robot's laser scanner's input.


3) Video:

    https://www.youtube.com/playlist?list=PLIjTZBf6bBGHFgdudMhfB64CVowR644vb

4) Others:

    N/A.
