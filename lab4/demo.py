"""
W4711 Comp Aspects of Robotics
Zhilin Guo, Shusen Xu
"""

from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import random as rand
import math

UR5_JOINT_INDICES = [0, 1, 2]
# our global variables
pie = round(math.pi, 10)
JOINT_RANGE = np.array([[-2 * pie, 2 * pie], [-2 * pie, 2 * pie], [-pie, pie]])
RRT_STEP = 0.12
BIRRT_STEP = 0.12
SMOOTHING_STEP = 0.06
BIAS = 0.08
N = 300
N_SMOOTHING = 100


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
    vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
    return marker_id


def remove_marker(marker_id):
    p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args


def rrt():
    """
    Function that finds a viable path using RRT algorithm
    :return: a list of tuples, representing the path of joint configurations
    """
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################

    print "starting RTT algorithm"

    # initialize variables
    all_nodes = []
    backtrack_dict = {}
    step_dist = RRT_STEP
    found = False

    print "Variables initialized"

    # start node is root of tree
    all_nodes += [start_conf]

    print "start making N attempts"

    # for n number of attempts to expand the tree
    n = 0
    while n < N:
        # 5% bias towards goal, q_rand == q_goal
        if rand.random() <= BIAS:
            # set q_rand to the goal
            q_rand = goal_conf
        else:
            # generate new random configuration q_rand
            q_rand = random_config()

        # find nearest node q
        best_node = nearest_node(all_nodes, q_rand)
        # show the best node
        set_joint_positions(ur5, UR5_JOINT_INDICES, best_node)
        # visulize
        # the position of end of arm in best node
        link_state_best_node = p.getLinkState(ur5, 3)
        pos_best_node = link_state_best_node[0]
        time.sleep(0.018)

        # print best_node, q_rand

        # scale the node down to the step_size
        # Move along path (q, q_rand) distance step_size
        delta = vector_add_sub(q_rand, best_node, '-')
        normed_delta = config_to_step(delta, step_dist)
        q_new = vector_add_sub(best_node, normed_delta, '+')

        # sleep to show q_new
        time.sleep(0.018)

        # check if q_new is valid
        # if path not valid, skip this q_rand
        if collision_fn(q_new):
            continue

        # collision free, add q_new as new tree node
        all_nodes += [q_new]
        # add q_new to the back tracking dictionary and forward trcking dictionary
        backtrack_dict[q_new] = best_node
        # visualize the edge of(best node to  q_new)
        link_state_new = p.getLinkState(ur5, 3)
        pos_q_new = link_state_new[0]
        new_edge = [pos_best_node, pos_q_new]
        visualize_helper(new_edge, mode="search")

        # goal check
        # As we add node q_new, see if it is within step_size of goal
        if config_distance(q_new, goal_conf) <= step_dist:
            # goal is within reach, add goal to list and dictionaries, and terminate the loop
            all_nodes += goal_conf
            backtrack_dict[goal_conf] = q_new
            found = True
            # visualize that goal is reached
            set_joint_positions(ur5, UR5_JOINT_INDICES, goal_conf)
            # if find the goal, then print the final edge(q_new,goal_conf)
            link_state_final = p.getLinkState(ur5, 3)
            pos_q_final = link_state_final[0]
            new_edge = [pos_q_new, pos_q_final]
            visualize_helper(new_edge, mode="search")
            time.sleep(0.2)
            break

        # repeat, increment counter
        n += 1

    print "Path findinf complete!"

    # print backtrack_dict

    # if goal is not found, pass
    if not found:
        print "Goal not found within time limit in RRT"
        return

    print "Goal found, reconstructing path"
    # backtrack construct the path
    path = [goal_conf]
    while True:
        # grab the first node in the path list
        first_node = path[0]
        # if the first node is the start, break the loop and return constructed path
        if first_node == start_conf:
            break
        # print first_node
        # add another node to the path
        prev_node = backtrack_dict[first_node]
        path = [prev_node] + path

    return path


def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################

    print "starting Bi-directional RTT algorithm"

    print "Variables initializing"

    # initialize variables
    tree1 = [start_conf]
    tree2 = [goal_conf]
    backtrack_dict1 = {}
    backtrack_dict2 = {}
    step_dist = BIRRT_STEP
    found = False

    print "start making N attempts"

    n = 0
    while n < N and not found:
        # n % 2 == 0, we grow tree 1 to goal
        if n % 2 == 0:
            # bias towards goal, q_rand == q_goal
            if rand.random() <= BIAS:
                # set q_rand to the goal
                q_rand = goal_conf
            else:
                # generate new random configuration q_rand
                q_rand = random_config()

            # find nearest node q
            best_node1 = nearest_node(tree1, q_rand)

            # show the best node
            set_joint_positions(ur5, UR5_JOINT_INDICES, best_node1)
            # modified:
            # visulize
            # the position of end of arm in best node
            link_state_best_node1 = p.getLinkState(ur5, 3)
            pos_best_node1 = link_state_best_node1[0]
            time.sleep(0.018)

            # scale the node down to the step_size
            # Move along path (q, q_rand) distance step_size
            delta = vector_add_sub(q_rand, best_node1, '-')
            normed_delta = config_to_step(delta, step_dist)
            q_new_1 = vector_add_sub(best_node1, normed_delta, '+')

            # show q_new
            set_joint_positions(ur5, UR5_JOINT_INDICES, q_new_1)
            time.sleep(0.018)

            # check if q_new_1 is valid
            # if path not valid, skip this q_rand
            if collision_fn(q_new_1) or q_new_1 in tree1:
                continue

            # collision free, add q_new as new tree node
            tree1 += [q_new_1]

            # add q_new_1's index to the dictionary for backtrack path construction
            backtrack_dict1[tree1.index(q_new_1)] = tree1.index(best_node1)
            # visualize the edge of(best node to  q_new)
            link_state_new1 = p.getLinkState(ur5, 3)
            pos_q_new1 = link_state_new1[0]
            new_edge = [pos_best_node1, pos_q_new1]
            visualize_helper(new_edge, mode="search", num_tree=1)

            # Expand tree2 towards q_new_1

            # find nearest node q
            best_node2 = nearest_node(tree2, q_new_1)

            # show the best node
            set_joint_positions(ur5, UR5_JOINT_INDICES, best_node2)
            # visulize
            # the position of end of arm in best node
            link_state_best_node2 = p.getLinkState(ur5, 3)
            pos_best_node2 = link_state_best_node2[0]
            time.sleep(0.018)

            # scale the node down to the step_size
            # Move along path (best_node2, q_new_1) distance step_size
            delta = vector_add_sub(best_node2, q_new_1, '-')
            normed_delta = config_to_step(delta, step_dist)
            q_new_2 = vector_add_sub(best_node2, normed_delta, '-')

            # show q_new
            set_joint_positions(ur5, UR5_JOINT_INDICES, q_new_2)
            time.sleep(0.018)

            # check if q_new_2 is valid
            # if path not valid, skip this q_new_2
            if collision_fn(q_new_2) or q_new_2 in tree2:
                continue

            # collision free, add q_new_2 as new tree node
            tree2 += [q_new_2]

            # add q_new_2's index to the dictionary for backtrack path construction
            backtrack_dict2[tree2.index(q_new_2)] = tree2.index(best_node2)
            # visualize the edge of(best node to  q_new)
            link_state_new2 = p.getLinkState(ur5, 3)
            pos_q_new2 = link_state_new2[0]
            new_edge2 = [pos_best_node2, pos_q_new2]
            visualize_helper(new_edge2, mode="search", num_tree=2)

            # check if q_new_1 and q_new_2 meet
            if config_distance(q_new_1, q_new_2) <= step_dist:
                found = True
                # modified
                # if find the goal, then print the final edge(q_new,goal_conf)
                new_edge = [pos_q_new2, pos_q_new1]
                visualize_helper(new_edge, mode="search", num_tree=2)
                print "Goal found"
                break

        # n % 2 == 1, we grow tree 2 to start
        else:
            # bias towards goal, q_rand == q_goal
            if rand.random() <= BIAS:
                # set q_rand to the goal
                q_rand = start_conf
            else:
                # generate new random configuration q_rand
                q_rand = random_config()

            # find nearest node q
            best_node2 = nearest_node(tree2, q_rand)
            # show the best node
            set_joint_positions(ur5, UR5_JOINT_INDICES, best_node2)
            # visulize
            # the position of end of arm in best node
            link_state_best_node2 = p.getLinkState(ur5, 3)
            pos_best_node2 = link_state_best_node2[0]
            time.sleep(0.018)

            # scale the node down to the step_size
            # Move along path (q, q_rand) distance step_size
            delta = vector_add_sub(best_node2, q_rand, '-')
            normed_delta = config_to_step(delta, step_dist)
            q_new_2 = vector_add_sub(best_node2, normed_delta, '-')

            # show q_new
            set_joint_positions(ur5, UR5_JOINT_INDICES, q_new_2)
            time.sleep(0.018)

            # check if q_new_2 is valid
            # if path not valid, skip this q_rand
            if collision_fn(q_new_2) or q_new_2 in tree2:
                continue

            # collision free, add q_new as new tree node
            tree2 += [q_new_2]

            # add q_new_1's index to the dictionary for backtrack path construction
            backtrack_dict2[tree2.index(q_new_2)] = tree2.index(best_node2)
            # visualize the edge of(best node to  q_new)
            link_state_new2 = p.getLinkState(ur5, 3)
            pos_q_new2 = link_state_new2[0]
            new_edge2 = [pos_best_node2, pos_q_new2]
            visualize_helper(new_edge2, mode="search", num_tree=2)

            # Expand tree1 towards q_new_2

            # find nearest node q
            best_node1 = nearest_node(tree1, q_new_2)

            # show the best node
            set_joint_positions(ur5, UR5_JOINT_INDICES, best_node1)
            # modified:
            # visulize
            # the position of end of arm in best node
            link_state_best_node1 = p.getLinkState(ur5, 3)
            pos_best_node1 = link_state_best_node1[0]
            time.sleep(0.018)

            # scale the node down to the step_size
            # Move along path (best_node2, q_new_1) distance step_size
            delta = vector_add_sub(q_new_2, best_node1, '-')
            normed_delta = config_to_step(delta, step_dist)
            q_new_1 = vector_add_sub(best_node1, normed_delta, '+')

            # show q_new
            set_joint_positions(ur5, UR5_JOINT_INDICES, q_new_1)
            time.sleep(0.018)

            # check if q_new_1 is valid
            # if path not valid, skip this q_new_1
            if collision_fn(q_new_1) or q_new_1 in tree1:
                continue

            # collision free, add q_new_1 as new tree node
            tree1 += [q_new_1]

            # add q_new_2's index to the dictionary for backtrack path construction
            backtrack_dict1[tree1.index(q_new_1)] = tree1.index(best_node1)
            # visualize the edge of(best node to  q_new)
            link_state_new1 = p.getLinkState(ur5, 3)
            pos_q_new1 = link_state_new1[0]
            new_edge = [pos_best_node1, pos_q_new1]
            visualize_helper(new_edge, mode="search", num_tree=1)

            # check if q_new_1 and q_new_2 meet
            if config_distance(q_new_1, q_new_2) <= step_dist:
                found = True
                # modified
                # if find the goal, then print the final edge(q_new,goal_conf)
                new_edge = [pos_q_new1, pos_q_new2]
                visualize_helper(new_edge, mode="search", num_tree=1)
                print "Goal found"
                break

        # increment iteration counter
        n += 1

    # if goal is not found, return nothing
    if not found:
        print "Goal not found within time limit in RRT"
        return

    print "Starting to reconstruct the path"

    # find the last q_new_1, as well as last q_new_2, and backtrack construct the result
    last_q_new_1 = tree1[-1]
    result1 = [last_q_new_1]

    last_q_new_2 = tree2[-1]
    result2 = [last_q_new_2]

    print "Reconstructing first half of the path"

    # first construct path from start to q_new_1
    while True:
        # get first item in result1
        first = result1[0]
        # find its index in tree 1
        first_index = tree1.index(first)
        # stop loop if it is the start, i.e. first item
        if first_index == 0:
            break
        # get the index of previous item in dictionary
        prev_index = backtrack_dict1[first_index]
        # grab the previous item
        prev = tree1[prev_index]
        # add it to the path
        result1 = [prev] + result1

    print "Reconstructing second half of the path"

    # then construct path from q_new_2 to goal
    while True:
        # get last item in result2
        last = result2[-1]
        # find its index in tree 2
        last_index = tree2.index(last)
        # stop loop if it is the goal, i.e. first item
        if last_index == 0:
            break
        # get the index of next item in dictionary
        next_index = backtrack_dict2[last_index]
        # grab the next item
        next_itm = tree2[next_index]
        # add it to the path
        result2 = result2 + [next_itm]

    return result1 + result2


def birrt_smoothing():
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    # get path from regular Bi-directional RRT algorithm
    path = birrt()
    # if path not found by birrt()
    if path is None:
        return
    step_size = SMOOTHING_STEP
    # Repeat N times
    n = 0
    while n < N_SMOOTHING:
        # get length of full path
        node_count = len(path)
        # get two random integers as indices
        rand1 = rand.randint(0, node_count - 2)
        rand2 = rand.randint(0, node_count - 2)
        # make sure two indices are not the same or adjacent
        if abs(rand1 - rand2) <= 1:
            continue
        node1_index = min(rand1, rand2)
        node2_index = max(rand1, rand2)
        # print node_count, node1_index, node2_index
        # get two nodes
        node1 = path[node1_index]
        node2 = path[node2_index]
        # check if they are collision free
        if not edge_collision(node1, node2, step_size):
            # there is no collision, we can skip the middle of the list
            middle_configs = smooth_line(node1, node2, step_size)
            path = path[:node1_index] + middle_configs + path[node2_index+1:]
        # increment counter n
        n += 1
    return path


def visualize_helper(E, mode, num_tree=1):
    '''
    :param E: the new added edge
    :param mode: process mode: search or path
    :param num_tree: the number of tree, tree extended from origin called 1 and tree extended from goal called 2
    :return: None
    '''

    if mode == "search":
        if num_tree == 1:
            colorRGB = (0, 1, 0)
            line_width = 0.2
        elif num_tree == 2:
            colorRGB = (0, 0, 1)
            line_width = 0.2
    if mode =="path":
        colorRGB = (1, 0, 0)
        line_width = 5

    p.addUserDebugLine(E[0], E[1], lineColorRGB=colorRGB, lineWidth=line_width)


def config_distance(config1, config2):
    """
    Helper function that calculates the SSD between two configuration vectors
    :param config1:
    :param config2:
    :return: distance in float
    """
    array1 = np.asarray(config1)
    array2 = np.asarray(config2)
    # Euclidean distance
    return math.sqrt(np.sum((array1 - array2) ** 2))


def random_config():
    """
    Helper function that generates a random (maybe invalid) configuration
    :return: a valid configuration (joint1, joint2, joint3)
    """
    # get random joint positions
    joint0 = rand.uniform(JOINT_RANGE[0, 0], JOINT_RANGE[0, 1])
    joint1 = rand.uniform(JOINT_RANGE[1, 0], JOINT_RANGE[1, 1])
    joint2 = rand.uniform(JOINT_RANGE[2, 0], JOINT_RANGE[2, 1])
    new_config = ((joint0, joint1, joint2))
    return new_config


def nearest_node(node_list, q_rand):
    """
    Helper function that finds the nearest node in node_list from q_rand
    :param node_list:
    :param q_rand:
    :return: the closest node
    """
    min_dist = float('inf')
    node_index = float('inf')
    # iterate over all nodes in node_list
    for i in range(len(node_list)):
        # find distance between current node and q_rand
        dist = config_distance(node_list[i], q_rand)
        # record node if it is better
        if dist < min_dist:
            min_dist = dist
            node_index = i
    return node_list[node_index]


def config_to_step(config, step_size):
    """
    Helper function that normalizes a configuration vector to step size length
    :param config: (joint1, joint2, joint3)
    :param step_size: float
    :return: new_config tuple
    """
    config_array = np.asarray(config)
    # find the scale
    c = math.sqrt(np.sum(config_array ** 2) / step_size ** 2)
    result_array = config_array / c
    return (result_array[0], result_array[1], result_array[2])


def vector_add_sub(config1, config2, operation):
    """
    Helper function that returns config1 + config2, or config1 - config2
    :param config1:
    :param config2:
    :param operation:
    :return: tuple
    """
    if operation == '+':
        joint0 = config1[0] + config2[0]
        joint1 = config1[1] + config2[1]
        joint2 = config1[2] + config2[2]
    else:
        joint0 = config1[0] - config2[0]
        joint1 = config1[1] - config2[1]
        joint2 = config1[2] - config2[2]
    return (joint0, joint1, joint2)


def edge_collision(config1, config2, step_size):
    """
    Helper function that checks if all points segmented by step size on the edge is collision free
    :param config1:
    :param config2:
    :param step_size:
    :return: True if there is collision, False if there is no collision
    """
    # base ase
    # if distance is smaller than step_size, then the edge is valid
    if config_distance(config1, config2) <= step_size:
        return collision_fn(config1) or collision_fn(config2)
    # get center point of two configs
    (joint1_0, joint1_1, joint1_2) = config1
    (joint2_0, joint2_1, joint2_2) = config2
    mid_config = ((joint1_0 + joint2_0) / 2, (joint1_1 + joint2_1) / 2, (joint1_2 + joint2_2) / 2)
    # if the mid config has collision, return True
    if collision_fn(mid_config):
        return True
    # now we test left and right edges separately
    return edge_collision(config1, mid_config, step_size) or edge_collision(mid_config, config2, step_size)


def smooth_line(config1, config2, step_size):
    """
    Helper function that finds a list of valid configurations, given the two end points and step size
    :param config1:
    :param config2:
    :param step_size:
    :return: list of configurations
    """
    result = []
    dist = config_distance(config1, config2)
    if dist <= step_size:
        return [config1, config2]

    # find distance between any two middle configurations
    delta = vector_add_sub(config2, config1, '-')
    normed_delta = config_to_step(delta, step_size)

    # create middle configurations

    for i in range(int(dist/step_size)):
        current = config1
        for j in range(i + 1):
            current = vector_add_sub(current, normed_delta, '+')
        result += [current]

    return [config1] + result + [config2]


if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200,
                                 cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1 / 4, 0, 1 / 2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2 / 4, 0, 2 / 3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn

    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                    attachments=[], self_collisions=True,
                                    disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt()

    # reset arm's configuration
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)
    time.sleep(0.5)

    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################

        # execute the path
        '''
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
            time.sleep(2)
        '''
        # quickly draw in the first time
        previous = (0, 0, 0)
        for q in path_conf:
            set_joint_positions(ur5, UR5_JOINT_INDICES, q)
            current_link_state = p.getLinkState(ur5, 3)
            current_pos = current_link_state[0]
            if q == path_conf[0]:
                previous = current_pos
                continue
            path_edge = [previous, current_pos]
            previous = current_pos
            visualize_helper(path_edge, mode="path")
            # first time: quickly draw the path
            time.sleep(0.018)
        time.sleep(2)
        while True:
            previous = (0, 0, 0)
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                current_link_state = p.getLinkState(ur5, 3)
                current_pos = current_link_state[0]
                if q == path_conf[0]:
                    previous = current_pos
                    continue
                path_edge = [previous, current_pos]
                previous = current_pos
                visualize_helper(path_edge, mode="path")
                # first time: quickly draw the path
                time.sleep(0.5)
            time.sleep(2)