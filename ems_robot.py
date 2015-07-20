__author__ = 'rdml'

class EmsRobot(object):
    def __init__(self, id, role, cur_loc, goal_loc, cur_batt, cur_map, comm_range, path=[]):
        self.id = id
        self.role = role
        self.cur_loc = cur_loc
        self.goal_loc = goal_loc
        self.cur_batt = cur_batt
        self.cur_map = cur_map
        self.path = path
        self.comm_range = comm_range