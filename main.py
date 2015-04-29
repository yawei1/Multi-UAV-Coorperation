#!/usr/bin/env
import sys
from  map import *
__author__ = 'rdml'

def plot(map):
    

def main():
    #set up the initial weights
    explore = 0
    mapping = 0
    search = 0

    mode = 'baseline'
    obstacles = [(0, 6), (1, 1), (1, 2), (1, 6), (2, 1), (2, 4), (3, 4), (3, 7), (3, 8), (4, 0), (4, 2), (5, 2), (5, 3),
                 (6, 5), (6, 6), (7, 1), (7, 6), (8, 1), (8, 2), (8, 6), (9, 8)]
    map = Map(10, 10, obstacles)
    plot(map)



if __name__ == "__main__":
    #todo: Make it accept parameters from command line
    #main(sys.argv[1:])
    main()








