# Lab 4 - Rapidly-exploring Random Tree (RRT)
Lab 4 for [COMSW4733 Computational Aspects of Robotics](https://www.cs.columbia.edu/~allen/F19/) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Author: Zhilin Guo , Shusen Xu

## Usage
Using Linux, with Python 2.7 and Pybullet installed, change into the directory where the code is located.\
First, to run RRT path planning, type in terminal:
```
python2 demo.py
```
Use ctrl + c in terminal to terminate the program.  
Next, to run Bi-directional RRT path planning, type in terminal:
```
python demo.py --birrt
```
Use ctrl + c in terminal to terminate the program.  
Lastly, to run Bi-directional RRT path planning with smoothing, type in terminal:
```
python demo.py --birrt --smoothing
```
Use ctrl + c in terminal to terminate the program.  
We do not have any additional scripts.

## Methods
1) rrt(): This is the method that handles ordinary RRT path finding.
It follows the pseudo code from lecture slides, and it records all
possible edges on the path in backwards order inside a dictionary for
path reconstruction.

2) birrt(): This is the method that handles Bi-directional RTT path finding.
It differs from rrt() in that it keeps two search trees, one from the start
and one from the goal, and it alternates between using these two trees to
build two backtracking dictionaries. In the end, it combines the content
from two dictionaries to build the path.

3) birrt_smoothing(): This is the method that handles Bi-directional RTT
path finding with smoothing. It calls birrt() to get an un-smoothed path Then
for a given number of iterations, it finds two random nodes in the path, and 
tests whether a collision-free path exists between the two nodes. If one such
path does exist, it fills this new path uniformly with nodes, and replace the path
between the two nodes with this new path segment.

4) visualize_helper(E, mode, num_tree=1): Helper method that visualize an edge, 
using mode to determine its color.

5) config_distance(config1, config2): Helper method that calculates the Euclidean
distance between to configuration vectors

6) random_config(): Helper method that generates a random configuration vector

7) nearest_node(node_list, q_rand): Helper method that finds the nearest configuration
in a list of configurations, given the target configuration

8) config_to_step(config, step_size): Helper function that normalizes a configuration
vector to step size length

9) vector_add_sub(config1, config2, operation): Helper function that returns config1 +
config2, or config1 - config2, given the operation

10) edge_collision(config1, config2, step_size): Helper function that checks if all
points segmented by step size on the edge is collision free

11) smooth_line(config1, config2, step_size): Helper function that finds a list of
valid configurations, given the two end points and step size

## Videos
RRT: https://www.youtube.com/watch?v=Q6U98TZcZO0  
BiRRT: https://www.youtube.com/watch?v=cAD8T6ddMPc  
BiRRT w/ Smoothing: https://www.youtube.com/watch?v=dsMA_HJKGPc


## Others
N/A.
