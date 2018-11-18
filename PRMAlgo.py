from Node import Node
from random import random
from math import sqrt, pi, cos, sin, acos
import numpy as np
import quaternion
from scipy.spatial import distance
from nearest_neighbors import *

# import rospy

edge_list = []
vertex_list = []


def generate_quaternion():
    s = random.uniform(0, 1)
    sig_1 = sqrt(1 - s)
    sig_2 = sqrt(s)
    angle_1 = 2 * pi * random.random()
    angle_2 = 2 * pi * random.random()
    w = cos(angle_2) * sig_2
    x = sin(angle_1) * sig_1
    y = cos(angle_1) * sig_1
    z = sin(angle_2) * sig_2

    return quaternion.quaternion(w, x, y, z)


# Compute the quaternion inner product lambda
# Return a range between [-1,1]
def quaternion_distance(q1, q2):
    first = (q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z)
    second = (q1.w * -q2.w) + (q1.x * -q2.x) + (q1.y * -q2.y) + (q1.z * -q2.z)

    if math.isclose(1.0, first, rel_tol=1e-9):
        first = 1.0

    if math.isclose(1.0, second, rel_tol=1e-9):
        second = 1.0

    if math.isclose(-1.0, first, rel_tol=1e-9):
        print("hit!")
        first = -1.0

    if math.isclose(-1.0, second, rel_tol=1e-9):
        second = -1.0

    return min(acos(first), acos(second))


def translation_distance(t1, t2):
    return distance.euclidean(t1, t2)


def distance_func(start, goal):
    return translation_distance(start.translation, goal.translation) + quaternion_distance(start.rotation, goal.rotation)


def collision_free_configuration(translation_matrix, rotation_matrix):
    if len(translation_matrix) != 3 or len(rotation_matrix) != 9:
        print("Incorrect list size for pqp request")
        return True
    rospy.wait_for_service('pqp_server')
    try:
        pqp_server = rospy.ServiceProxy('pqp_server', pqpRequest)
        result = pqp_server(translation_matrix, rotation_matrix)
        return result
    except rospy.ServiceException as e:
        print("Service Call Failed: %s", e)



# local planner
def local_planner(cnode, nnode):
    interpolated_quaternion = slerp([cnode.w, cnode.x, cnode.y, cnode.z], [nnode.w, nnode.x, nnode.y, nnode.z], [0.1])

    for i in range(5):
        print()

    return "connect two vertex"


# use multiple calls of collision_free_config
def collision_free_path(path):

    return True


def slerp(v0, v1, t_array):
    # >>> slerp([1,0,0,0],[0,0,0,1],np.arange(0,1,0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0 * v1)

    if (dot < 0.0):
        v1 = -v1
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if (dot > DOT_THRESHOLD):
        result = v0[np.newaxis, :] + t_array[:, np.newaxis] * (v1 - v0)[np.newaxis, :]
        result = result / np.linalg.norm(result)
        return result

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t_array
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:, np.newaxis] * v0[np.newaxis, :]) + (s1[:, np.newaxis] * v1[np.newaxis, :])


if __name__ == "__main__":
    # User inputs how many starting samples and how many edges per neighbor
    iteration = int(input("How many iterations?"))
    k_nearest = int(input("How many k closest neighbors?"))

    # Instantiate a roadmap class using NearestNeighbor
    roadmap = NearestNeighbors(distance_func)

    # start_node and end_node
    # need to replace with file inputs
    start_node = Node([0, 0, 0], generate_quaternion())
    goal_node = Node([9, 9, 9], generate_quaternion())

    # add this to vertex_list
    vertex_list.append(start_node)
    vertex_list.append(goal_node)

    # sampling portion of user input iteration
    for n in range(iteration):
        good_sample = False

        while not good_sample:
            # random sampling
            # generate a random transformation matrix inside the world space
            # the z-axis value is temporary, may need to fix, but x and y should be fine
            t_matrix = [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(0, 5)]

            # generate a random quaternion
            q = generate_quaternion()

            # flatten the quaternion into a 1 x 9 matrix for collision checking
            r_matrix = [item for sublist in quaternion.as_rotation_matrix(q) for item in sublist]

            # collision check against the world to see if current configuration is correct
            # takes a len = 3 t_matrix and a len = 9 r_matrix, uncomment the code below and delete if 1 == 1
            # if collision_free_configuration(t_matrix, r_matrix) == 0:
            if 1 == 1:
                good_sample = True

                # create a new node by passing the translation and quaternion
                current_node = Node(t_matrix, q)

                # add the node to the roadmap and vertex_list
                vertex_list.append(current_node)
                roadmap.add_node(current_node)

    # Connecting the edges of the samples using kNN
    for current_node in vertex_list:
        # initialize two different array for keeping track of k closest neighbors
        # dist is a float array that keeps track of the distance from current_node to a neighbor_node
        # closest_node is an array that points to the neighbor_node obj
        dist = [0] * k_nearest
        closest_nodes = [0] * k_nearest

        # use the roadmap's kNN function to find all the neighbor node
        # node that dist and closest_node are modified now
        roadmap.find_k_close(current_node, closest_nodes, dist, k_nearest)

        # check to see if the edge between the two nodes are valid
        for neighbor_node in closest_nodes:
            path = local_planner(current_node, neighbor_node)

            if collision_free_path(path):
                current_node.add_node(neighbor_node)



