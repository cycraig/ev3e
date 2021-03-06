# rrt_obstacles.py
#
# Path planning using RRT to navigate around rectangular
# obstacles to reach a given goal.
#
# Inspired by:
# Rahman, Md Mahbubur, Leonardo Bobadilla, and Brian Rapp. "Sampling-based planning 
# algorithms for multi-objective missions." Automation Science and Engineering (CASE), 
# 2016 IEEE International Conference on. IEEE, 2016.
# https://www.linkedin.com/pulse/motion-planning-algorithm-rrt-star-python-code-md-mahbubur-rahman/
#
# Author: Craig Bester, 783135
# Date: 2017-09-29
#


import argparse
import sys
import random
import math
import matplotlib as mpl
#mpl.use('agg')
import matplotlib.pyplot as plt

WIDTH=200
HEIGHT=150
MAXNodeS = 5000
DEBUG=False

class Point(object):
    def __init__(self, x, y):
        self.x = x;
        self.y = y;

    def __str__(self):
        return "%d,%d"%(self.x,self.y);

class Node(Point):
    def __init__(self, x, y):
        super(Node,self).__init__(x,y);
        self.parent = None;
        
    def drawable(self):
        '''
        Returns a list describing the position of this point.
        '''
        return (self.x,self.y)

class Obstacle(object):
    def __init__(self,x,y,r):
        self.centre = Point(x,y);
        self.radius = r;
        
    def drawable(self):
        return (self.centre.x,self.centre.y),self.radius
        
    def __str__(self):
        return "(%s),%s"%(self.centre,self.radius);
        
    def pointIntersects(self, p):
        '''
        Returns whether the obstacle overlaps the given point.
        '''
        return distance(self.centre,p)<=self.radius;
                
    def lineIntersects(self, a, b):
        '''
        Returns whether the line connecting points p1 and p2
        intersects the obstacle.
        '''
        return intersects_circle(a,b,self.centre,self.radius);
        
def intersects_circle(p, q, centre, radius):
    a = p.x - q.x
    b = p.y - q.y
    d = math.sqrt(a*a + b*b)
    if(d > 0):
        return (abs((centre.x - p.x) * (q.y - p.y) - (centre.y -  p.y) * (q.x - p.x)) / d <= radius)
    else:
        return distance(centre,p) <= radius;
        
def distance(a,b):
    # should use squared distance, cheaper
    d = (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)
    d = math.sqrt(d);
    return d;
        
def project_towards(a,b,delta=3.0):#delta=3.0):
    if distance(a,b) < delta:
        return Node(b.x,b.y)
    else:
        theta = math.atan2(b.y-a.y,b.x-a.x)
        return Node(int(a.x + delta*math.cos(theta)), int(a.y + delta*math.sin(theta)))
    
def RRT(start, goal, obstacles):
    '''
    Runs the RRT* path planning algorithm given a start Node, goal Node, 
    and a list of obstacles.
    
    Visualises the tree construction process and the resulting path.
    '''
    if not isinstance(start,Node):
        start = Node(start[0],start[1]);
    if not isinstance(goal,Node):
        goal = Node(goal[0],goal[1]);
    
    #plt.axis([0, WIDTH, 0, HEIGHT])
    #axis = plt.gca();
    #axis.invert_yaxis()
    #axis.xaxis.set_ticks_position('top')
    #axis.set_aspect('equal')
    
    #plt.plot([50,50],[70,70],'r');
    #plt.plot(start.x,start.y,'go');
    #plt.plot(goal.x,goal.y,'bo');
    
    def drawObstacles(obstacles, axis):
        for obs in obstacles: 
            xy, r = obs.drawable();
            #print xy, width, height
            circ = mpl.patches.Circle(xy,r,fill=False, color='r');
            axis.add_patch(circ);
    
    #drawObstacles(obstacles, axis);
    #plt.ion();
    #plt.show();
    #plt.pause(0.05);
    
    # -----------------
    # Build RRT
    # -----------------
    tree = [start]
    i = 0;
    while i < MAXNodeS:
    
        i += 1;
    
        if random.random() < 0.4:
            qrand = Node(goal.x,goal.y);
        else:
            # Random Node
            qrand = Node(int(random.random()*WIDTH), int(random.random()*HEIGHT))
        
        # Check for obstacle collisions - faster to do before checking nearest Node
        collision = False;
        for obs in obstacles: 
            if obs.pointIntersects(qrand):
                i = i-1;
                collision = True;
                break;
        if collision:
            continue;
        
        # Nearest Node
        qnear = start;
        mindist = distance(start,qrand);
        for a in tree:
            d = distance(a,qrand);
            if d < mindist:
              qnear = a;
              mindist = d;
        # Check if already in tree, don't want duplicates
        if mindist == 0:
            i = i-1;
            continue;
    
        # Project towards new Node
        qnew = project_towards(qnear,qrand);
        # Projection gives same point, too close, ignore
        if qnew.x == qnear.x and qnew.y == qnear.y:
            i = i-1;
            continue;
        
        # Check for obstacle collisions
        collision = False;
        for obs in obstacles: 
            #print qnew,"vs",obs,
            if obs.pointIntersects(qnew) or obs.lineIntersects(qnear,qnew):
                #print "TRUE!"
                i = i-1;
                collision = True;
                break;
            #print "false"
        if collision:
            continue;
        
                
        # Add Node to tree
        tree.append(qnew);
        qnew.parent = qnear;
        
        # Break if we found exact goal
        if int(qnew.x) == int(goal.x) and int(qnew.y) == int(goal.y):
            if DEBUG:
                print("Found goal! Stopping early...")
            break;     
        # Break if we got near to goal...
        if(distance(goal,qnew) < 5):
            # Check for obstacle collisions
            collision = False;
            for obs in obstacles: 
                if obs.lineIntersects(qnew,goal):
                    collision = True;
                    break;
            if not collision:
                if DEBUG:
                    print("Found goal! Stopping early...")
                break;  
        
        # Update screen
        #plt.plot([qnear.x,qnew.x],[qnear.y,qnew.y],color='k');
        
        #if(i%20 == 0):
        #    plt.pause(0.001);
            
    
    # -----------------
    # Find path to goal
    # -----------------
    path = [];
    
    # Goal found / maximum Nodes reached
    # Try find nearest Node to goal to append goal to tree
    qnear = None;
    mindist = float("inf");
    for p in tree:
        d = distance(p,goal)
        if d < mindist:
            # Check for obstacle collisions
            collision = False;
            for obs in obstacles: 
                if obs.lineIntersects(p,goal):
                    collision = True;
                    break;
            if collision:
                continue;
            qnear = p;
            mindist = d;
    # no unobstructed path found
    if qnear is None:
        return None;
    
    # Draw path to goal
    path = []
    goal.parent = qnear;
    p = goal;
    if qnear.x != goal.x or qnear.y != goal.y:
        path.append(goal) # don't append goal twice if it was found in the tree
    while p != start:
        #plt.plot([p.x,p.parent.x],[p.y,p.parent.y],color='c', linewidth=2)
        p=p.parent
        path.append(p)
    # Redraw start, goal so they don't get covered with lines
    #plt.plot(start.x,start.y,'go');
    #plt.plot(goal.x,goal.y,'bo');
    #plt.pause(0.05)
    
    # Reverse path to end at goal
    #path.reverse()
    
    # path smoothing
    i = 0
    while i < len(path)-2:
        # Check for obstacle collisions
        collision = False;
        for obs in obstacles: 
            #print qnew,"vs",obs,
            #print("Checking between",path[i],path[i+2]);
            if obs.lineIntersects(path[i],path[i+2]):
                collision = True;
                break;
            #print "false"
        if not collision and distance(path[i],path[i+2])<=60:
            #print("Deleting node",path[i+1]);
            del path[i+1]
        else: 
            i = i+1
    
    path.reverse()
    
    fpath = []
    for p in path:
        fpath.append([p.x,p.y]);
    return fpath;
    
"""
--------------
EXAMPLE INPUT:
--------------

10,10;80,30
20,10;20,50
20,50;90,50
30,30;40,40
-1

10,10;80,30
10,20;30,50
12,5;18,30
50,30;60,40
70,40;70,70
-1

"""

def convert_to_obstacles(circles):
    obstacles = [];
    for (x,y,r) in circles:
        obstacles.append(Obstacle(x,y,r));
    return obstacles;

if __name__ == '__main__':
    #random.seed(7)
    random.seed(1)
    
    # Parse arguments
    example = '''example:
 python rrt.py
  
 10,10;80,30
 20,10,5
 20,50,10
 30,30,2
 -1
 '''
    parser = argparse.ArgumentParser(description='Path planning using RRT to navigate around rectangular obstacles to reach some goal.',
                                     add_help=True,
                                     epilog=example,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    args = parser.parse_args();
    
    # Read input
    #input = map(float, sys.stdin.readline().strip().replace(';',' ').replace(',',' ').split());
    input = sys.stdin.readline().strip().replace(';',' ').replace(',',' ').split()
    for i in range(len(input)):
        input[i] = float(input[i]);
    start = Node(input[0],input[1]);
    goal = Node(input[2],input[3]);
    obstacles = [];
    finished = False;
    while not finished:
        #input = sys.stdin.readline()
        if input is None:
            sys.exit("Reached EOF while reading input!");
        
        #input = map(float, input.strip().replace(';',' ').replace(',',' ').split());
        input = sys.stdin.readline().strip().replace(';',' ').replace(',',' ').split()
        for i in range(len(input)):
            input[i] = float(input[i]);
        if len(input) <= 1 and input[0] == -1:
            finished = True;
        else:
            obstacles.append(Obstacle(input[0],input[1],input[2]));
              
    # DEBUG output
    if DEBUG:
        print("Start: (%s)"%start)
        print(" Goal: (%s)"%goal)
        print("Obstacles:")
        for o in obstacles:
            print(o)
        
    # Path planning
    path = RRT(start, goal, obstacles);
        
    # Print path
    if path is None or len(path) == 0:
        print("NO PATH FOUND!");
    else:
        for n in path:
            print(n);
    plt.show(block=True);
