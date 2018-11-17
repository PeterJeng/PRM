from Node import Node
from random import random
from math import sqrt, pi, cos, sin, acos
import numpy as np
import quaternion
from scipy.spatial import distance

# import rospy

edge_list = []
vertex_list = {}


def generate_quaternion():
    s = random.uniform(0, 1)
    sig_1 = sqrt(1 - s)
    sig_2 = sqrt(s)
    angle_1 = 2 * pi * random()
    angle_2 = 2 * pi * random()
    w = cos(angle_2) * sig_2
    x = sin(angle_1) * sig_1
    y = cos(angle_1) * sig_1
    z = sin(angle_2) * sig_2

    return quaternion.quaternion(w, x, y, z)


# Compute the quaternion inner product lambda
# Return a range between [-1,1]
def quaternion_distance(q1, q2):
    first = acos((q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z))
    second = acos((q1.w * -q2.w) + (q1.x * -q2.x) + (q1.y * -q2.y) + (q1.z * -q2.z))

    return min(first, second)


def translation_distance(t1, t2):
    return distance.euclidean(t1, t2)


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


def slerp(v0, v1, t_array):
    # >>> slerp([1,0,0,0],[0,0,0,1],np.arange(0,1,0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0 * v1)

    if dot < 0.0:
        v1 = -v1
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
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
    iteration = int(input("How many iterations?"))

    for n in range(iteration):
        print("Iteration: " + str(n + 1))

        good_sample = False
        translation_matrix = []
        rotation_matrix = []

        while not good_sample:
            # generate a random transformation matrix inside the world space
            t_matrix = [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(0, 5)]

            # generate a random quaternion
            r_matrix = quaternion.as_rotation_matrix(generate_quaternion())

            if collision_free_configuration(t_matrix, r_matrix) == 0:
                good_sample = True
                translation_matrix = t_matrix
                rotation_matrix = r_matrix

        current_node = Node(translation_matrix, rotation_matrix)

        # add node to vertex_list, might need to build this differently
        if current_node not in vertex_list:
            vertex_list.append(current_node)

        # create a neighborhood of nodes given a node
        neighborhood_list = neighborhood(current_node, vertex_list, edge_list)

        for neighbor_node in neighborhood_list:
            path = local_planner(current_node, neighbor_node)

            if collision_free_path(path):
                edge_list.append(path)



