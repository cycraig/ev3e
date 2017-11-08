# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 14:31:34 2017

@author: Liron
"""

import numpy as np
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.children = []
        
        
class Tree:
    def __init__(self, node):
        self.root = node
        self.all_points = []
        self.all_points.append(self.root)
        
    def add_edge(self, p, node):
        node.parent = p
        p.children.append(node)
        self.all_points.append(node)
    
    
    def get_closest_vertex(self, point):
        min_dist = 99999
        min_vertex = -1
        for i in range(len(self.all_points)):
            d = distance(point, self.all_points[i].point)
            if d < min_dist:
                min_dist = d
                min_vertex = i
        
        return self.all_points[min_vertex]
   
def intersects_circle(l1, l2, c, r):
	a = l1[0] - l2[0]
	b = l1[1] - l2[1]
	x = np.sqrt(a*a + b*b)
	return (np.abs((c[0] - l1[0]) * (l2[1] - l1[1]) - (c[1] -  l1[1]) * (l2[0] - l1[0])) / x <= r)

    
def has_intersection_circle(p1, p2, obs):
    for o in obs:
        obs_point = (o[0], o[1])
        i = intersects_circle(p1, p2, obs_point, o[2])
        if i:
            return True        
    return False
    
    
def distance(pnt1, pnt2):
    return np.linalg.norm(pnt1 - pnt2)
    

def sample_point(width, height):
    x = (width) * np.random.random_sample()
    y = (height) * np.random.random_sample()
    return np.array([x, y])


def to_array(c):
    s = c.split(',')
    return np.array([int(s[0]), int(s[1])])


def step(near, rand, step_size):
    mag = np.linalg.norm(near - rand)
    if mag < step_size:
        return Node( np.array([ rand[0], rand[1] ]) )
    else:
        theta = math.atan2(rand[1] - near[1], rand[0] - near[0])
        x = near[0] + step_size * math.cos(theta)
        y = near[1] + step_size * math.sin(theta)
        node_new = Node(np.array([x, y]))
        return node_new
        

def det(a, b):
    return a[0] * b[1] - a[1] * b[0]

def ccw(A,B,C):
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

        
def has_intersection(p1, p2, obs):
    for o in obs:
        i = intersect(p1, p2, o[0], o[1])
        if i:
            return True
        
    return False

def rrt(start, goal, obs, w, h):
    n = Node(start)
    T = Tree(n)
    step_size = 30

    for k in range(1000):
        
#        q_goal = goal
#        node_near = T.get_closest_vertex(q_goal)
#        node_new = Node(q_goal)
#        intersects = has_intersection(node_new.point, node_near.point, obs)
#        if not intersects:
#            T.add_edge(node_near, node_new)
#            draw_line(grid, node_near.point, node_new.point, red)
#            print 'FOUND GOAL!!'
#            return True, node_new

        q_goal = goal
        node_new = Node(q_goal)
        for v in T.all_points:
            intersects = has_intersection_circle(node_new.point, v.point, obs)
            if not intersects:
                T.add_edge(v, node_new)
                print('FOUND GOAL!!')
                return True, node_new
        
        
        else:
            q_rand = sample_point(w, h)
            node_near = T.get_closest_vertex(q_rand)
            node_new = step(node_near.point, q_rand, step_size)
    
            intersects = has_intersection_circle(node_new.point, node_near.point, obs)
            if not intersects:
                T.add_edge(node_near, node_new)
#        else:
#            draw_line(grid, node_near.point, node_new.point, green)

    return False


def get_goal_path(goal):
    path = []
    x = goal
    while x is not None:
        path.append(x.point)
        x = x.parent
    return path



"""
10,10;80,30
20,10;20,50
20,50;90,50
30,30;40,40
-1
"""
def RRT(start, goal, obs, w = 170, h = 170):
	start = np.array([start[0], start[1]])
	goal = np.array([goal[0], goal[1]])
    
    found, goal_node = rrt(start, goal, obs, w, h)
    
    if found:
        goal_path = get_goal_path(goal_node)
#        for i in range(len(goal_path)):
#            print(goal_path[i])            
        return goal_path
    
    else:
        return None