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


def neighborhood(q, vertex, edge):

    return [1, 2, 3]


# local planner
def local_planner(v1, v2):
    return "connect two vertex"


# use multiple calls of collision_free_config
def collision_free_path(path):

    return True


if __name__ == "__main__":
    iteration = int(input("How many iterations?"))
    k_nearest = int(input("How many k closest neighbors?"))

    roadmap = NearestNeighbors(distance_func)

    start_node = Node([0, 0, 0], generate_quaternion())
    goal_node = Node([9, 9, 9], generate_quaternion())

    vertex_list.append(start_node)
    vertex_list.append(goal_node)

    for n in range(iteration):
        good_sample = False

        while not good_sample:
            # generate a random transformation matrix inside the world space
            t_matrix = [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(0, 5)]

            # generate a random quaternion
            q = generate_quaternion()
            r_matrix = [item for sublist in quaternion.as_rotation_matrix(q) for item in sublist]

            # takes a len = 3 t_matrix and a len = 9 r_matrix
            # if collision_free_configuration(t_matrix, r_matrix) == 0:
            if 1 == 1:
                good_sample = True
                translation_matrix = t_matrix

                current_node = Node(translation_matrix, q)

                vertex_list.append(current_node)
                roadmap.add_node(current_node)

    for current_node in vertex_list:
        # initialize two different array for keeping track of k closest neighbors
        dist = [0] * k_nearest
        closest_nodes = [0] * k_nearest

        roadmap.find_k_close(current_node, closest_nodes, dist, k_nearest)

        for neighbor_node in closest_nodes:
            path = local_planner(current_node, neighbor_node)

            if collision_free_path(path):
                edge_list.append(path)



