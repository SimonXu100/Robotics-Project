Follow Bot
Zhilin Guo & Shusen Xu

Lab 5 for [COMSW4733 Computational Aspects of Robotics](http://www.cs.columbia.edu/~allen/F19/index.html) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Usage
First, create the catkin workspace like before.  
Then, for each map:  
  
1. Followbot 
Launch turtlebot and map for part one
```
roslaunch followbot launch.launch
```
In a new terminal window, change into directory of the lab,
and run python file
```
python follow_color_2.py
```
When finished, use ctrl+c to terminate both programs.

2. Map with color markers  
Launch turtlebot and map for part two
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```
In a new terminal window, change into directory of the lab,
and run python file
```
python follow_color_2.py
```
When finished, use ctrl+c to terminate both programs.

3. Map with shape markers
Launch turtlebot and map for part three
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```
In a new terminal window, change into directory of the lab,
and run python file
```
python follow_shape.py
```
When finished, use ctrl+c to terminate both programs.

4. Extra
N/A


## Method
1. Followbot & 2. Map with color markers
Part 1 and Part 2 shares the same python script.  

If no colored shapes
are discovered (other than the yellow path), the robot follows the path
by calculating whether left or right side of the camera image has larger
area of yellow path, and steers the robot to that side proportional
to the area difference.  

If colored shapes other than yellow is captured, the program steers 
the robot according to the color, i.e. turn left at green marker, right
at blue marker, and stop at red marker.

3. Map with shape markers
Part 3 uses a FSM approach. The script uses contour detection to find
out the number of edges a colored shape has. If the colored shape
matches the shape, the script uses method of moments to find out the
orientation of the triangle and steers the robot, or stops it if it
is a star.

4. Extra
N/A

## Video
https://youtu.be/-eq5uQ9HgKA

## Others
N/A
