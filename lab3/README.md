Shusen Xu

1) Usage:
    First, launch the vgraph world in Rviz:
    
    $ roslaunch vgraph launch.launch

    Then, add MarkerArray using Rviz's GUI from Displays panel.
    
    Use ctrl + s to save this configuration for future testings.
    
    Next, in a new terminal, change into vgraph's src directory:
    
    $ cd /home/<UserName>/catkin_ws/src/vgraph/src 
    
    Finally, launch the lab3.py script:
    
    $ python lab3.py

2) Method:

    The program completes the path finding problem in several steps:
    
    (1) For each obstacle's all vertices, calculate the 4 points (top left, top right, bottom left, bottom right) surrounding each vertex. Put each obstacle's all such points into a seperate list, and use ConvexHull to calculate the subset of these points that can make a fence surrounding each obstacle. Keep a dictionary of connections possible between vertices. Then, visualize the fence for each obstacle.
    
    (2) Using our own collision detection, connect all independent vertices: that is, connect each pair of vertices that is not on the same obstacle. Also, connect from start to all possible fence vertices, from all possible fence vertices to goal, and from start to goal, if there is no collision. Keep a dictionary of connections possible between vertices. Visualize all such valid connections.

    (3) Use A* search to find the optimal path from start to goal, picking connection from the aforementioned dictionary of available connections. The heuristic value is the distance from the currently considered point (child in A*) to the goal. Keep another dictionary of each selected point's parent point for backtracking, and reconstruct the optimal path with this dictionary. Visualize the optimal path with a different color.
    
    (4) Pass the optimal path to the robot class, modified from Lab2's bug2.py, so the robot follow the optimal path and reaches the goal.
    
    Specifically, marker is implemented both with Marker and MarkerArray. All possible edges is done using MarkerArray, where the method takes in a list of tuple, each tuple containing two points consisting of a edge, and visualize them. Optimal path is done using Marker, and it takes a list of the optimal path tuples.

3) Video:

    https://youtu.be/OadfK13h_Nw

4) Others:

    N/A.
