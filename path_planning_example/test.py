# This tests path_planning module

# Lab: 3 Motion Planning lab
# Jason Chalom 711985

# In order to input data:
# Open from file: example.txt

# If No Given Parameters Will use example.txt
# usage: python path_planning.py filename
# Need python 2.7, numpy and matplotlib

# Main Algorithm
# Uses RRT and will find within a radius of the point, reduce it to get close to the centre of the point
# Then every 40% of the time will choose the goal instead of a random point to generate path
# If too many obstacles reduce percent_goal to find path

# Input format
# starting point;target point
# list of obstacle coordinates
# -1
# Example:
# 10,10;80,30
# 20,10;20,50
# 20,50;90,50
# 30,30;40,40
# -1
# 2.2 Output format
# The output should be a list of waypoints the robot will move through
# 10,10
# 10,51
# 91,51
# 91,30
# 80,30

import numpy as np
import glob
import matplotlib.pyplot as plt
import random
import os, sys
lib_path = os.path.abspath(os.path.join('..'))
sys.path.append(lib_path)

# the module
import path_planning as pp

def open_input(filename):
	file = open(filename,"r")
	lines = file.readlines()
	obstacles = []
	
	obj = lines[0]
	obj = obj.rstrip()
	sp = obj.split(";")
	start = sp[0]
	end = sp[1]

	start_pt = [float(x) for x in start.split(",")] #
	end_pt = [float(x) for x in end.split(",")]

	objectives = [start_pt,end_pt]

	for l in lines[1::]:
		l = l.rstrip() 
		ll = l.split(";")
		sp = [float(x) for x in ll[0].split(",")]
		sp = np.array(sp)
		
		obstacles.append(np.array([sp, float(ll[1])]))
		# print(sp)
	obstacles = np.array(obstacles)
	return objectives,obstacles

def print_usage():
	print("Jason Chalom 711985\n")
	#print("If No Given Parameters Will use example.txt")
	print("usage: python path_planning.py filename")
	print("example: python path_planning.py example.txt")
	print("Need python 2.7, numpy and matplotlib\n")

# hack so I dont have to use virtualenvs
if sys.version_info >= (3,0):
	xrange = range

# Main Algorithm
# Uses RRT and will find within a radius of the point, reduce it to get close to the centre of the point
# Then every 40% of the time will choose the goal instead of a random point to generate path
# If too many obstacles reduce percent_goal tro find path
if __name__ == '__main__':
	print_usage()
	#path = "example.txt"
	if len(sys.argv) == 2:
		path = sys.argv[1]

		deltaq = 7.
		radius = 5.
		num_nodes = K = 5000 # can reduce but other termination condition will help
		
		# World Limits
		min=0.
		max=200.
		percent_goal = .60

		objectives,obsticles = open_input(path)
		objectives = np.array(objectives)
		obsticles = np.array(obsticles)
		
		# print (objectives.shape)
		# print (obsticles.shape)

		G = pp.build_RRT(objectives, obsticles, num_nodes, deltaq, radius, percent_goal, min, max)
		final_path = []

		if(G.found):
			final_path = pp.get_final_path(G, deltaq, radius, num_nodes)
			pp.print_path(G, final_path)

		pp.plot(G, radius, final_path)
	

