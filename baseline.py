#!/usr/bin/env
__author__ = 'rdml'

import scipy
import numpy as np

from map import *
from base_robot import *
from map_vis import *
from cur_map import *
from scipy import spatial
from priority_queue import *




def cal_euclidean_distance(loc1, loc2):
    loc1 = np.array(loc1)
    loc2 = np.array(loc2)
    dis = np.linalg.norm(loc1 - loc2)
    return dis


def cal_manhattan_distance(loc1, loc2):
    a = [loc1, loc2]
    dis = scipy.spatial.distance.pdist(a, 'cityblock')
    return dis[0]


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def path_plan(goal, robot, init_map):
    frontier = PriorityQueue()
    frontier.put(robot.cur_loc, 0)
    came_from = {}
    cost_so_far = {}
    came_from[robot.cur_loc] = None
    cost_so_far[robot.cur_loc] = 0


    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break
        for next in robot.cur_map.path_neighbors(current):
            new_cost = cost_so_far[current] + robot.cur_map.cost(current, next)

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    path = [goal]
    while path[-1] != robot.cur_loc:
        path.append(came_from[path[-1]])
    del path[-1]
    path.reverse()
    return path


def find_closest_to_base(base, robot):
    options = {}
    for node in robot.cur_map.nodes:
        if robot.cur_map.passable(node):
            dis_base = cal_euclidean_distance(base, node)
            if dis_base < robot.comm_range:
                dis = cal_manhattan_distance(robot.cur_loc, node)
                options[node] = dis
    closest = min(options, key = options.get)
    return closest


def find_closest_frontier(robot):
    # if can't make it don't go
    min_dis = float("inf")
    dis = {}
    for frontier in robot.cur_map.frontiers:
        dis[frontier] = cal_manhattan_distance(robot.cur_loc, frontier)
        # if dis[frontier] < min_dis:
        #     closest = frontier
        #     min_dis = dis[frontier]
    if dis:
        closest = min(dis, key=dis.get)
        min_dis = dis[closest]
        if robot.cur_batt < (min_dis * 2):
            closest = (0, 0)
        return closest
    else:
        return False


def map_update(robot, init_map):
    for nbor in robot.cur_map.neighbors(robot.cur_loc, init_map):
        if nbor not in robot.cur_map.obstacles and not init_map.passable(nbor):
            robot.cur_map.obstacles.append(nbor)
        if nbor not in robot.cur_map.frontiers and robot.cur_map.passable(nbor):
            if nbor not in robot.cur_map.nodes:
                robot.cur_map.frontiers.append(nbor)
        if nbor not in robot.cur_map.nodes:
            robot.cur_map.nodes.append(nbor)



def map_merge(cur_map1, cur_map2):
    for frontier in cur_map1.frontiers:
        if frontier not in cur_map2.frontiers:
            if frontier in cur_map2.nodes:
                cur_map1.frontiers.remove(frontier)
            else:
                cur_map2.frontiers.append(frontier)

    for frontier in cur_map2.frontiers:
        if frontier not in cur_map1.frontiers:
            if frontier in cur_map1.nodes:
                cur_map2.frontiers.remove(frontier)
            else:
                cur_map1.frontiers.append(frontier)

    for node in cur_map1.nodes:
        if node not in cur_map2.nodes:
            cur_map2.nodes.append(node)

    for node in cur_map2.nodes:
        if node not in cur_map1.nodes:
            cur_map1.nodes.append(node)

    for obs in cur_map1.obstacles:
        if obs not in cur_map2.obstacles:
            cur_map2.obstacles.append(obs)

    for obs in cur_map2.obstacles:
        if obs not in cur_map1.obstacles:
            cur_map1.obstacles.append(obs)

    return [cur_map1, cur_map2]


def explore(init_map, robots, budget, run, axe, step):
    #TODO(yawei):make it general, read map, initial state
    base_map = CurMap([(0, 0), (0, 1), (1, 0), (1, 1)], [(0, 1), (1, 0), (1, 1)], [(1, 1)])
    if run:
        mat_out = vis(axe, robots, init_map)
        print "---------------Step {0}------------------".format(step)
        #   make plans
        for robot in robots:
            if robot.cur_batt <= 0:
                print "Oops, out of battery."
                print "Robot{0} is here:{1}".format(robots.index(robot), robot.cur_loc)

            elif robot.cur_batt <= cal_manhattan_distance(init_map.base, robot.cur_loc) or robot.goal_loc == init_map.base:
                print "Robot{0} goes home".format(robots.index(robot))
                robot.mode = 'go_home'

            elif (robot.cur_batt % (budget/4) == 0) and (robot.cur_batt != budget):
                print "Robot{0} checks base".format(robots.index(robot))
                robot.mode = 'check_base'

            elif robot.mode != 'check_base':
                print "Robot{0} explores".format(robots.index(robot))
                robot.mode = 'explore'

        #   move
        for robot in robots:
            if robot.mode == 'go_home':
                map_update(robot, init_map)
                if robot.cur_loc in robot.cur_map.frontiers:
                    robot.cur_map.frontiers.remove(robot.cur_loc)
                if robot.cur_loc == init_map.base:
                    print "Robot{0} is home.".format(robots.index(robot))
                    robot.mode = 'done'
                else:
                    if not robot.goal_loc == init_map.base:
                        robot.path = path_plan(init_map.base, robot, init_map)
                        robot.goal_loc = init_map.base
                    if cal_euclidean_distance(robot.cur_loc, init_map.base) < robot.comm_range:
                        [robot.cur_map, base_map] = map_merge(robot.cur_map, base_map)
                    robot.move()
            elif robot.mode == 'check_base':
                map_update(robot, init_map)
                if robot.cur_loc in robot.cur_map.frontiers:
                    robot.cur_map.frontiers.remove(robot.cur_loc)
                if cal_euclidean_distance(robot.goal_loc, init_map.base) > robot.comm_range:
                    if cal_euclidean_distance(robot.cur_loc, init_map.base) > robot.comm_range:
                        robot.goal_loc = find_closest_to_base(init_map.base, robot)
                    else:
                        robot.goal_loc = robot.cur_loc
                    robot.path = path_plan(robot.goal_loc, robot, init_map)
                if robot.cur_loc == robot.goal_loc:
                    print "Robot{0} is ready to communicate with base.".format(robots.index(robot))
                    [robot.cur_map, base_map] = map_merge(robot.cur_map, base_map)
                    robot.goal_loc = None
                    robot.mode = 'explore'
                else:
                    robot.move()
                    # TODO(yawei): make communication costs one step
                # if any robot in range, merge map
                for other_robot in robots:
                    if robot.id == other_robot.id:
                        continue
                    else:
                        dis = cal_euclidean_distance(robot.cur_loc, other_robot.cur_loc)
                        if dis < robot.comm_range:
                            [robot.cur_map, other_robot.cur_map] = map_merge(robot.cur_map, other_robot.cur_map)

            elif robot.mode == 'explore':
                if robot.cur_loc in robot.cur_map.frontiers:
                    robot.cur_map.frontiers.remove(robot.cur_loc)
                if robot.cur_loc == robot.goal_loc:
                    print "Robot{0} arrives frontier{1}.".format(robots.index(robot), robot.goal_loc)
                    map_update(robot, init_map)
                    robot.goal_loc = None
                # plan path
                if not robot.goal_loc:
                     # choose frontier
                    loc = find_closest_frontier(robot)
                    if loc == init_map.base:
                        print "Frontier too far. Go home."
                        robot.goal_loc = init_map.base
                        robot.path = path_plan(init_map.base, robot, init_map)
                        break
                    if not loc:
                        print "No more frontiers. Go home."
                        robot.goal_loc = init_map.base
                        robot.path = path_plan(init_map.base, robot, init_map)
                        break
                    # delete frontier, put it into nodes
                    robot.goal_loc = loc
                    robot.cur_map.frontiers.remove(robot.goal_loc)
                    robot.path = path_plan(robot.goal_loc, robot, init_map)

                # in any robot in communicate range, merge map, and check goal
                for other_robot in robots:
                    if other_robot.id == robot.id:
                        continue
                    else:
                        dis = cal_euclidean_distance(robot.cur_loc, other_robot.cur_loc)
                        if dis < robot.comm_range:
                            [robot.cur_map, other_robot.cur_map] = map_merge(robot.cur_map, other_robot.cur_map)
                            if other_robot.goal_loc == robot.goal_loc:
                                if other_robot.cur_batt >= robot.cur_batt:
                                    loc = find_closest_frontier(robot)
                                    if loc:
                                        robot.goal_loc = loc
                                        robot.path = path_plan(robot.goal_loc, robot, init_map)
                                        robot.cur_map.frontiers.remove(robot.goal_loc)

                                else:
                                    loc = find_closest_frontier(other_robot)
                                    if loc:
                                        other_robot.goal_loc = loc
                                        other_robot.path = path_plan(other_robot.goal_loc, other_robot, init_map)
                                        other_robot.cur_map.frontiers.remove(other_robot.goal_loc)

                # if basestation in range merge map
                dis = cal_euclidean_distance(robot.cur_loc, init_map.base)
                if dis < robot.comm_range:
                    map_merge(robot.cur_map, base_map)

                robot.move()
            print "robot{0} is here{1}, and going to {2}".format(robot.id, robot.cur_loc, robot.goal_loc)

        for robot in robots:
            for frontier in robot.cur_map.frontiers:
                inside = []
                for nbor in robot.cur_map.neighbors(frontier, init_map):
                    if nbor in robot.cur_map.nodes:
                        inside.append(1)
                    else:
                        inside.append(0)
                        break
                if all(inside):
                    robot.cur_map.frontiers.remove(frontier)

        # final condition check
        run = False
        for robot in robots:
            if robot.cur_loc != init_map.base or robot.goal_loc != init_map.base:
                run = True
                break
            print 'done'

    return run, mat_out



def mapping(init_map, robots, budget, run):
    # for nodes in current map, stay 2 steps
    pass



def search(init_map, robots, budget, run):
    # if run:
    #     #make plans\
    #     for robot in robots:
    #         if target.cur_loc == robot.cur_loc:
    #             robot.mode = 'done'
    #             print 'done'
    #         if target.cur_loc in robot.cur_map.neighbors():
    #             robot.cur_map.nodes
    pass