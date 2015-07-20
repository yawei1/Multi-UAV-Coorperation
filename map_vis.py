__author__ = 'rdml'

#!/usr/bin/env
__author__ = 'rdml'

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import random





def init_fig():
    fign = plt.figure()
    axn = fign.add_subplot(111)
    axn.set_aspect('equal', 'datalim')
    axn.tick_params(labelbottom='on',labeltop='off')
    axn.set_xlabel('x')
    axn.set_ylabel('y')
    axn.autoscale(tight=True)
    return fign, axn


def draw_grid(axes, robots, grid):
    grid_mat = np.zeros((grid.width, grid.height))
    for x in range(grid.width):
        for y in range(grid.height):
            grid_mat[x, y] = 0.8

    for robot in robots:
        for x, y in robot.cur_map.nodes:
            grid_mat[x, y] = 0.5
        for x, y in robot.cur_map.obstacles:
            grid_mat[x, y] = -1

    for robot in robots:
        for x, y in grid.neighbours(robot.cur_loc):
            grid_mat[x, y] = 2
            # print "x:{0}, y:{1}".format(x, y)

    for robot in robots:
        grid_mat[robot.cur_loc[0], robot.cur_loc[1]] = 0.1


    grid_mat = np.ma.masked_where(grid_mat == -1, grid_mat)
    # print "grid_mat{0}\n".format(grid_mat)
    cmap = plt.cm.RdGy
    cmap.set_over(color=(0, 0.6, 1))
    cmap.set_bad(color='black')
    axes.set_xlim([0, grid.width]); axes.set_ylim([0, grid.height])
    mat_out = [axes.matshow(grid_mat.transpose(), interpolation='none', cmap=cmap, vmin=0, vmax=1)]
    axes.tick_params(labelbottom='on', labeltop='off')
    return mat_out


def vis(axe, robots, init_map):
    mat_out = draw_grid(axe, robots, init_map)
    return mat_out


# def draw_corridor(axes, grid, cost_to_come, corridor, interface=[], path=[]):
#     cost_mat = -1*np.ones((grid.width, grid.height))
#     for node in corridor:
#         cost_mat[node[0],node[1]] = cost_to_come[node]
#     for x,y in grid.obstacles:
#         cost_mat[x,y] = -2
#     max_cost = cost_mat.max()
#     for node in interface:
#         cost_mat[node[0],node[1]] = max_cost+1
#     cost_mat = np.ma.masked_where(cost_mat == -2, cost_mat)
#     cmap = plt.cm.jet
#     cmap.set_bad(color='black')
#     cmap.set_over(color='#C2A366')
#     cmap.set_under(color='0.8')
#
#     axes.set_xlim([0, grid.width]); axes.set_ylim([0, grid.height])
#     mat_out = [axes.matshow(cost_mat.transpose(),
#         norm = matplotlib.colors.Normalize(vmin=0, vmax=max_cost, clip=False))]
#     if len(path) > 0:
#         x, y = zip(*path)
#         mat_out.append(axes.plot(x, y, 'w-', linewidth=2.0 )[0])
#     axes.tick_params(labelbottom='on',labeltop='off')
#
#     return mat_out
#
#
# def draw_fbfmcost(axes, grid, path_cost, path=[], min_cost = 1e7, max_cost = 0):
#     grid_mat = np.zeros((grid.width, grid.height))
#     for x in range(grid.width):
#         for y in range(grid.height):
#             if (x,y) in path_cost:
#                 grid_mat[x,y] = path_cost[(x,y)]
#     for x,y in grid.obstacles:
#         grid_mat[x,y] = -1
#     grid_mat = np.ma.masked_where(grid_mat == -1, grid_mat)
#     cmap = plt.cm.jet
#     cmap.set_bad(color='black')
#     axes.set_xlim([0, grid.width]); axes.set_ylim([0, grid.height])
#     max_cost = max(max_cost, grid_mat.max())
#     min_cost = min(min_cost, min(path_cost.values()))
#     mat_out = [axes.matshow(grid_mat.transpose(), interpolation='none', cmap=cmap, vmax=max_cost, vmin=min_cost)]
#     if len(path) > 0:
#         x, y = zip(*path)
#         mat_out.append(axes.plot(x, y, 'w-', linewidth=2.0 )[0])
#     axes.tick_params(labelbottom='on',labeltop='off')
#     #axes.figure.colorbar(mat_out[0])
#     return mat_out, [grid_mat.min(), grid_mat.max()]