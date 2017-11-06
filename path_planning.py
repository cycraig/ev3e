# This path planning using RRT 
# Assumes all given obsticle coordinates are circles
# Beware the get_final_path fn may crash if no path is able to be found ... (return none)
# Tree (G) G.found is a variable which stores whether a path to the goal has been found or not (T/F)
# Radius of obsticles must account for the bot

# Usage:

# import path_planning as pp

# Constants:
# ==========
# deltaq = 7. # This is the length to project from nearest node
# radius = 5. # This is how far from goal to stop
# num_nodes = K = 5000 #Ensures that the algorithm will stop at that many nodes
# World Limits
# min=0. # should stay at 0
# max=200. # upper bound of world ... i.e. random points

# This determines how often not to generate random point but rather project to goal
# Will make the line straighter but makes it more likely a path may not be found
# 
# percent_goal = .60 

# Required Data:
# ==============
# objectives # numpy array (2,2)
# obsticles # numpy array (number of obsticles, ((centre), radius)) .... (n, (2,1))

# Functions of importance:
# ========================
# Will configure and generate the tree (G) and nodes
# G = pp.build_RRT(qinit, obsticles, num_nodes, deltaq, radius, percent_goal, min, max)

# Will take a built tree and if there is a path then it will find and return it
# NB: if(G.found):
# final_path = pp.get_final_path(G, deltaq, radius, num_nodes)

# Will print a path to the console
# NB: if(G.found):
# pp.print_path(G, final_path)

# Will plot a RRT tree, all nodes, obstacles and the found path ... if there is one
# pp.plot(G, radius, final_path)

import numpy as np
import glob
import matplotlib.pyplot as plt
import random
import sys

# The equations and maths functions
def distance(p, q):
	return np.sqrt(np.power(q[0]-p[0],2) + np.power(q[1]-p[1],2))

def new_conf(qnear, qrand, deltaq):
	# can optimise ... 
	dist = distance(qnear, qrand)
	xnew = qnear[0] + (deltaq/dist)*(qrand[0]-qnear[0])
	ynew = qnear[1] + (deltaq/dist)*(qrand[1]-qnear[1])
	qnew = [xnew,ynew]

	return qnew

def nearest_vertex(q, G):
	# q = [x,y]
	min = 100000. # cannot be greater than size of world
	min_vertex = [0.,0.]
	# print G.vertices
	for i in xrange(len(G.vertices)):
		dist = distance(G.vertices[i], q)
		if (min > dist):
			min = dist
			min_vertex = G.vertices[i]

	return min_vertex

def gradient(p,q):
	m = 0.0
	if (q[0]-p[0] == 0. or q[1]-p[1] == 0.): #make undefined 0
		return 0.0
	else:
		m = (q[1]-p[1])/(q[0]-p[0])
		
	return m

def parent_edge_idx(node, G):
	edge_idx = 0

	for i in xrange(len(G.edges)):
		if (G.edges[i][1][0] == node[0] and G.edges[i][1][1] == node[1]):
			edge_idx = i
	return edge_idx

def check_if_at_goal(q, goal, radius):
	distance_from_centre = np.sqrt(np.power((q[0]-goal[0]), 2) + np.power(q[1]-goal[1], 2))

	if (distance_from_centre <= radius):
		return True
	else:
		return False

def intersects_circle(l1, l2, c, r):
	a = l1[0] - l2[0]
	b = l1[1] - l2[1]
	x = np.sqrt(a*a + b*b)
	return (np.abs((c[0] - l1[0]) * (l2[1] - l1[1]) - (c[1] -  l1[1]) * (l2[0] - l1[0])) / x <= r)

def get_final_path(G, deltaq, radius, max_count):
	path = []

	# print G.edges
	root_idx = G.edges[-1][2]
	edge = G.edges[-1]
	path.append(edge)

	while root_idx > 0:
		root_idx = edge[2]
		# print root_idx
		edge = G.edges[root_idx]
		path.append(edge)

	# print path[::-1]
	G.path = path[::-1]
	return path[::-1]

# Core RRT
class Tree():
	def __init__(self, start_pt, end_pt, K, radius, min=0, max=200):
		self.vertices = []
		self.obsticles = []
		self.edges = []
		self.K = K

		self.r = radius

		self.min = min
		self.max = max

		self.vertices.append(start_pt)
		self.goal = [end_pt]
		self.found = False
		self.path = []

	def add_vertex(self, x, y):
		qnew = [x,y]
		self.vertices.append(qnew)

	def add_edge(self, qnear, qnew, root_idx):
		# print self.edges
		self.edges.append([qnear,qnew,root_idx])

	def add_obsticles(self, edges):
		for j in xrange(len(edges)):
			self.obsticles.append(edges[j])

	def detect_intersection(self, p, q):
		for i in xrange(len(self.obsticles)):
			r = self.obsticles[i][1]

			if intersects_circle(p,q,self.obsticles[i][0],r):
				return True
		return False

def build_RRT(qinit, obsticles, K, deltaq, radius, percent_goal, min, max):
	G = Tree(qinit[0], qinit[1], K, radius, min, max)
	G.add_obsticles(obsticles)
	
	root_idx = 0
	root_node = G.vertices[0]
	for k in xrange(K):	
		qrand = G.goal[0]
		# percent_goal % of the time go to goal
		if random.random() < percent_goal: 
			qrand = G.goal[0]
		else:
			qrand = (G.max-G.min)*np.random.random((2,)) + G.min
		
		qnear = nearest_vertex(qrand, G)
		qnew = new_conf(qnear, qrand, deltaq)

		if not G.detect_intersection(qnear,qnew):
			root_idx = parent_edge_idx(qnear, G)
			# root_node = G.edges[root_idx][1]

			G.add_vertex(qnew[0], qnew[1])
			G.add_edge(qnear,qnew,root_idx)
			# root = qnew

			atGoal = check_if_at_goal(qnew, G.goal[0], radius)
			if atGoal:
				G.found = True
				break;

	if(G.found):
		print ("found goal\n")
	else:
		print ("failed to find goal\n")

	return G

# Plotting and printing
def connectpoints(x,y,point_colour='ro',line_colour='k-'):
	plt.plot(x,y,point_colour)

	x1, x2 = x[0], x[1]
	y1, y2 = y[0], y[1]
	plt.plot([x1,x2],[y1,y2],line_colour)

def plot_points(G):
	fig, ax = plt.subplots()
	s = 500
    
	ax.set_aspect('equal')
	ax.scatter(G.vertices[0][0], G.vertices[0][1], color='g', s=s/3, marker='s', alpha=.4) #start
	ax.scatter(G.goal[0][0], G.goal[0][1], color='r', s=2*s, marker='^', alpha=.4) #end

	return fig, ax

def plot_path(path,point_colour='ro',line_colour='k-'):
	for i in xrange(len(path)):
		x = [path[i][0][0], path[i][1][0]]
		y = [path[i][0][1], path[i][1][1]]
		connectpoints(x,y,point_colour,line_colour)

def plot_cirlces(centres, ax):
	for c in centres:
		circle = plt.Circle(c[0].tolist(), c[1], color='orange', fill=False)
		ax.add_artist(circle)
		ax.scatter(c[0][0], c[0][1], color='orange', s=50, alpha=.4)

def plot_RRT(G, r, ax):
	edges = G.edges
	obsticles = G.obsticles
	
	plot_cirlces(obsticles, ax)
	plot_path(edges,'bo','b-')

def plot(G, r, final_path):
	fig, ax = plot_points(G)
	plot_RRT(G, r, ax)
	plot_path(G.path,'go','g-')
	plt.show()

def print_path(G, path):
	print (path[0][0])
	for i in xrange(len(path)):
		print (path[i][1])
	print (G.goal[0]) #since well be very close to the goal (within radius r)

# hack so I dont have to use virtualenvs
if sys.version_info >= (3,0):
	xrange = range