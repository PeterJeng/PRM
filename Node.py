import nearest_neighbors as nn


# class that defines a node in se(3)
class RelativePosition:
    def __init__(self, translation, rotation):
        # need 7 dimensional node because of quaternion definition

        self.translation = translation
        self.rotation = rotation


# a vertex with neighbors
class Node(RelativePosition):
    def __init__(self, translation, rotation):
        RelativePosition.__init__(self, translation, rotation)
        self.neighbors = nn.pad_or_truncate([], nn.INIT_CAP_NEIGHBORS, -1)
        self.nr_neighbors = 0
        self.added_index = 0
        self.index = 0
        self.cap_neighbors = nn.INIT_CAP_NEIGHBORS

    def getX(self):
        return self.translation[0]

    def getY(self):
        return self.translation[1]

    def getZ(self):
        return self.translation[2]

    def set_index(self, index):
        self.index = index

    def get_index(self):
        return self.index

    def get_neighbors(self):
        return self.nr_neighbors, self.neighbors

    def add_neighbor(self, nd):
        if self.nr_neighbors >= self.cap_neighbors - 1:
            self.cap_neighbors = 2 * self.cap_neighbors
            self.neighbors = nn.pad_or_truncate(self.neighbors, self.cap_neighbors, -1)
        self.neighbors[self.nr_neighbors] = nd
        self.nr_neighbors += 1

    def delete_neighbor(self, nd):
        index = 0
        for x in range(0, self.nr_neighbors):
            if self.neighbors[index] == nd:
                break
            index += 1
        if index >= self.nr_neighbors:
            return

        self.neighbors[index] = None

        self.neighbors[index], self.neighbors[self.nr_neighbors - 1] \
            = self.neighbors[self.nr_neighbors - 1], self.neighbors[index]
        self.nr_neighbors -= 1

    def replace_neighbor(self, prev, new_index):
        index = 0
        for x in range(0, self.nr_neighbors):
            if self.neighbors[index] == prev:
                break
            index += 1
        if index >= self.nr_neighbors:
            return
        self.neighbors[index] = new_index


