import picker
from picker import cans, obs
import camera
import path_planning as pp
import time
import numpy as np
import rpyc
#from plt_rrt import convert_to_obstacles, RRT
#from rrt import RRT
from plt_rrtstar import convert_to_obstacles, RRTStar as RRT
import math

USB = '169.254.238.226';
BLUETOOTH = '192.168.50.2'; # *.*.*.1 for laptop?
HOSTNAME = 'ev3dev';

ROBOT_RADIUS = 17.5 # in cm
PIXELS_TO_CM_RATIO = 30./107.#30./105.7#30./111.#30./109.#30./106.#30./90.;
CM_TO_PIXELS_RATIO = 107./30.#105.7/30.#30./111.#30./109.#106./30.#90./30.;

# path planning parameters
deltaq = 7.
radius = 5.
num_nodes = 5000
# World Limits
min=0.
max=159.
percent_goal = .60

# connect to bot
connected = False;
while not connected:
    try:
        conn = rpyc.connect(BLUETOOTH, port=12345)
        bot = conn.root;
        connected = True;
    except:
        print("Could not connect to ev3 bot...");
        time.sleep(1.);
        pass;
        

def project_towards(a,b,length):
    a = np.array(a);
    b = np.array(b);
    mag = np.linalg.norm(a - b)
    if mag < length:
        return [ b[0], b[1] ]
    else:
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
        x = a[0] + length * math.cos(theta)
        y = a[1] + length * math.sin(theta)
        p = [x, y]
        return p
        
def pixels_to_cm(x,y):
    return x*PIXELS_TO_CM_RATIO,y*PIXELS_TO_CM_RATIO
    
def cm_to_pixels(xy):
    x = xy[0]
    y = xy[1]
    return (x*CM_TO_PIXELS_RATIO,y*CM_TO_PIXELS_RATIO)

def process_objects(cans, obs):
    '''
    Increases the radius of the cans and obstacles
    and converts their coordinates from pixels to 
    centimetres.
    '''
    for i in range(len(cans)):
        x,y,r = cans[i];
        x *= PIXELS_TO_CM_RATIO;
        y *= PIXELS_TO_CM_RATIO;
        r = r*PIXELS_TO_CM_RATIO+ROBOT_RADIUS*0.85#*0.75;
        cans[i] = (x,y,r);
        
    for i in range(len(obs)):
        x,y,r = obs[i];
        x *= PIXELS_TO_CM_RATIO;
        y *= PIXELS_TO_CM_RATIO;
        r = r*PIXELS_TO_CM_RATIO+ROBOT_RADIUS*0.85#0.75;
        obs[i] = (x,y,r);
        
def dist(a,b):
    return np.linalg.norm(np.subtract(a,b));
    
def to_bot_coordinates(pos):
    x = pos[0]
    y = pos[1]
    pos = [-y,x];
    return pos;
        
def move_along_path(path,end_action=None):
    '''
    Makes the bot travel along the given path from point-to-point.
    
    end_action: ["open","close","None",None]
        Determines what action to take at the end of the path.
        "open": open the pincers
        "close": close the pincers
        None,"None": no action
    '''
    
    n = len(path);
    print("Whole path =",path);
    for i in range(n-1):
    
        # convert path to pixels for drawing
        camera_path = []
        for j in range(i,n):
            camera_path.append(cm_to_pixels(path[j]));
        cam.set_path(camera_path);
    
        botPosition = camera.get_bot_position();
        botPos = to_bot_coordinates(pixels_to_cm(botPosition[0],botPosition[1]));
        botOrientation = camera.get_bot_orientation();
        print("Path",i,"=",path[i])
        point = to_bot_coordinates(path[i]);
        if(dist(botPos,point) < 3):
            continue;
        # set current position
        bot.set(botPos[0],botPos[1],botOrientation);
        # move to next point along path
        bot.move_to(point[0],point[1]);
        
        # attempt to correct if needed
        botPosition = camera.get_bot_position();
        botPos = to_bot_coordinates(pixels_to_cm(botPosition[0],botPosition[1]));
        botOrientation = camera.get_bot_orientation();
        if(dist(botPos,point) > 30):
            print("Correcting position");
            bot.set(botPos[0],botPos[1],botOrientation);
            bot.move_to(point[0],point[1]);
            
    
    botPosition = camera.get_bot_position();
    botPos = to_bot_coordinates(pixels_to_cm(botPosition[0],botPosition[1]));
    bot.set(botPos[0],botPos[1],botOrientation);
    botOrientation = camera.get_bot_orientation();
    # FINAL GOAL (projected backwards to account for distance to pincers)
    goal = path[-1];
    cgoal = (goal[0],goal[1])
    goal = to_bot_coordinates(goal);
    print("Goal before",goal);
    goal = project_towards(goal,botPos,7)
    print("Goal after",goal);
    
    # convert path to pixels for drawing
    camera_path = []
    #camera_path.append(cm_to_pixels((botPosition[0],botPosition[1])));
    camera_path.append(cm_to_pixels(cgoal));
    cam.set_path(camera_path);
    
    bot.move_to(goal[0],goal[1]);
    
    
    # open/close pincers
    if end_action == "open":
        bot.open_pincers();
    elif end_action == "close":
        bot.close_pincers();
        
    

if __name__ == '__main__':
    global botPosition, botOrientation, goalPosition
    # pick things
    picker.main()

    print(cans, obs)
    process_objects(cans,obs);
    print(cans, obs)
    
    botPosition = None
    botOrientation = None
    goalPosition = None
    
    done = False
    # collect all cans
    try:
        # start camera thread
        print("Starting camera...");
        cam = camera.Camera(camIndex=0,show=True,debug=False);
        cam.start();
        
        print("Waiting for camera...");
        # wait until camera starts up
        while(botPosition is None or botOrientation is None or goalPosition is None):
            botPosition = camera.get_bot_position();
            botOrientation = camera.get_bot_orientation();
            goalPosition = camera.get_goal_position();
            time.sleep(0.1);
        print("Initial position:",botPosition)
        print("Initial orientation:",botOrientation)
        print("Goal position:",goalPosition)
    
       
        num_cans = len(cans);
        i = 0;
        while i < num_cans:
            print("Going for can",i);
            i += 1;
            can = cans.pop(0); # remove can from obstacle lists
            
            # convert camera stuff to cm
            botPosition = camera.get_bot_position();
            botPos = pixels_to_cm(botPosition[0],botPosition[1]);
            goalPos = pixels_to_cm(goalPosition[0],goalPosition[1]);
            #print botPos
            #print goalPos
            #print can
            
            print("OBSTACLES:",cans+obs)
            print("Planning path to can...");
            obstacles = convert_to_obstacles(cans+obs);
            #obstacles = cans+obs;
            path = RRT(botPos,(can[0],can[1]),obstacles);
            # convert path to pixels for drawing
            camera_path = []
            for xy in path:
                camera_path.append(cm_to_pixels(xy));
            cam.set_path(camera_path);
            if path is None:
                print("CANNOT REACH CAN!")
                # add can back to list, maybe we can reach it after removing other cans
                cans.append(can);
                i = i-1;
                continue;
                
            
            
            print("Following path to can...");
            move_along_path(path,"close");
                
            print("Planning path to goal...");
            # now take can to goal
            # update bot position
            botPosition = camera.get_bot_position();
            botPos = pixels_to_cm(botPosition[0],botPosition[1]);
            #goalPosition = camera.get_goal_position();
            goalPos = pixels_to_cm(goalPosition[0],goalPosition[1]);
            path = RRT(botPos,goalPos,obstacles);
            # convert path to pixels for drawing
            camera_path = []
            for xy in path:
                camera_path.append(cm_to_pixels(xy));
            cam.set_path(camera_path);
            if path is None:
                print("CANNOT REACH GOAL!")
                bot.open_pincers();
                break
            else:
                print("Following path to goal...");
                move_along_path(path,"open");
                
                '''# reverse towards previous point/s
                reverses = 0;
                cumulative_distance = 0.;
                n = len(path)
                k = n-2;
                for j in range(3):
                    if k < 0:
                        break;
                    botPosition = camera.get_bot_position();
                    botPos = to_bot_coordinates(pixels_to_cm(botPosition[0],botPosition[1]));
                    botOrientation = camera.get_bot_orientation();
                
                    point = path[k];
                    d = dist(botPos,point);
                    p = project_towards(botPos,path[k],10);
                    bot.reverse_to(p[0],p[1]);
                    cumulative_distance += np.minimum(d,10);
                    if cumulative_distance >= 10:
                        break;
                    else:
                        k = k-1;'''
                cam.set_path(None);
                bot.reverse(np.minimum(dist(botPos,path[-1]),13));
                
                if len(cans) == 0:
                    done = True
            
            '''# convert to obstacles for path planning
            obstacles = cans+obs;
            for i in range(len(obstacles)):
                obstacles[i] = np.array([(obstacles[i][0],obstacles[i][1]),obstacles[i][2]]);
            
            print("Planning path to can...");
            # pass to path planning
            # go to can first
            G = pp.build_RRT([botPos,(can[0],can[1])], obstacles, num_nodes, deltaq, radius, percent_goal, min, max)
            final_path = []
            #pp.plot(G, radius, final_path)
            if(G.found):
                final_path = pp.get_final_path(G, deltaq, radius, num_nodes)
                pp.print_path(G, final_path)
            else:
                print("CANNOT REACH CAN!")
                # add can back to list, maybe we can reach it after removing other cans
                cans.append(can);
                i = i-1;
                continue;

            print("Following path...");
            move_along_path(final_path);
            
            print("Planning path to goal...");
            # now take can to goal
            # update bot position
            botPos = pixels_to_cm(botPosition[0],botPosition[1]);
            botPosition = camera.get_bot_position();
            # pass to path planning
            G = pp.build_RRT([botPos,goalPos], obstacles, num_nodes, deltaq, radius, percent_goal, min, max)
            final_path = []

            if(G.found):
                final_path = pp.get_final_path(G, deltaq, radius, num_nodes)
                pp.print_path(G, final_path)
            else:
                print("CANNOT REACH GOAL!");
                
            print("Following path...");
            move_along_path(final_path);

            #pp.plot(G, radius, final_path)'''
        
        if done is True:
            print("DONE!")
            bot.speak("Yay! Yay! Yay!");
            bot.rotate_by(-90);
            bot.rotate_by(90);
            bot.rotate_by(360);
            time.sleep(5);
        else:
            print("STUCK!");
            bot.speak("STUCK");
        
    finally:
        # clean-up thread, otherwise hangs on Ctrl-C
        cam.stop();
    #time.sleep(10000);