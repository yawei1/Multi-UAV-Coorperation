#! /usr/env/bin
__author__ = 'rdml'

class BaseRobot(object):
    def __init__(self, id, cur_loc, goal_loc, cur_batt, cur_map, mode, path, comm_range):#, mode_time):
        self.id = id
        self.cur_loc = cur_loc
        self.goal_loc = goal_loc
        self.cur_batt = cur_batt
        self.cur_map = cur_map
        self.mode = mode
        self.path = path
        self.comm_range = comm_range
        # self.mode_time = mode_time

    def move(self):
        self.cur_loc = self.path.pop(0)
        self.cur_batt -= 1
