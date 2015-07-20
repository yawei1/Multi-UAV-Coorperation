__author__ = 'rdml'
from map import *
from ems_robot import *
from map_vis import *
from cur_map import *
from scipy import spatial
from priority_queue import *

def explore(init_map, robots, budget, run, axe, step):
    base_map = CurMap([(0, 0), (0, 1), (1, 0), (1, 1)], [(0, 1), (1, 0), (1, 1)], [(1, 1)])
    if run:
        mat_out = vis(axe, robots, init_map)
        print "---------------Step {0}------------------".format(step)

        # make decisions
        make_plans(robots)
        # move
        move(robots)

    return run, mat_out