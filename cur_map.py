#!/usr/bin/env
__author__ = 'rdml'


class CurMap(object):
    def __init__(self, nodes, frontiers, obstacles):
        self.nodes = nodes
        self.frontiers = frontiers
        self.obstacles = obstacles

    def passable(self, id):
        return bool(id not in self.obstacles)

    def in_bounds(self, id):
        return bool(id in self.nodes)

    def path_neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

    def neighbors(self, id, init_map):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]
        results = filter(init_map.in_bounds, results)
        return results

    def cost(self, loc1, loc2):
        return 1

