from Node import Node
from random import random
from math import sqrt, pi, cos, sin
import numpy as np
import quaternion

# import rospy

edge_list = []
vertex_list = []


def generate_quaternion():

    s = random()
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
    q3 = q1 * q2


def sample_point():
    print()


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


def neighborhood(q, Vertex, Edge):
    return [1, 2, 3]


# local planner
def local_planner(v1, v2):
    return "connect two vertex"


# use multiple calls of collision_free_config
def collision_free_path(path):

    return True


if __name__ == "__main__":
    iteration = int(input("How many iterations?"))

    for n in range(iteration):
        print("Iteration: " + str(n + 1))

        collision_check = True
        translation_matrix = []
        rotation_matrix = []

        # while collision_check:
            # generate a translation matrix
            # generate a quaternion, use function to convert to rotation matrix

        current_node = Node(translation_matrix, rotation_matrix)

        # add node to vertex_list, might need to build this differently
        vertex_list.append(current_node)

        # create a neighborhood of nodes given a node
        neighborhood_list = neighborhood(current_node, vertex_list, edge_list)

        for neighbor_node in neighborhood_list:
            path = local_planner(current_node, neighbor_node)

            if collision_free_path(path):
                edge_list.append(path)



