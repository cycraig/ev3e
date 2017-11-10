# bot.py
#
# Library of ev3 functions for motion.

import math
import time
from ev3dev import ev3

# State of the robot
CURRENT_POSITION = [0,0]; # x,y
CURRENT_ANGLE = 0; # degrees

# Ports
right_wheel = ev3.LargeMotor('outA');
left_wheel = ev3.LargeMotor('outD');
pincer = ev3.MediumMotor('outB');
gyro = ev3.GyroSensor('in3'); # can remove port specification, still works
gyro.mode='GYRO-ANG'
#sonar = ev3.UltrasonicSensor('in4');
#assert sonar.connected;
#sonar.mode = 'US-DIST-CM'

# Default values, tuned to the ev3e robot
MOVE_SPEED = 360; # wheel rotations / second
TURNING_SPEED = 300; # wheel rotations / second
AXLE_DIAMETER = 12.2; #cm (approximate, includes 2*half of wheel widths)
WHEEL_RADIUS = 2.8; #cm (approximate)
WHEEL_CIRCUMFERENCE = 2.*math.pi*WHEEL_RADIUS;

def set(x,y,angle):
    '''
    Sets the robot's believed position (x,y) and angle theta
    in degrees (clockwise rotation) in [-180,180].
    '''
    global CURRENT_POSITION, CURRENT_ANGLE
    CURRENT_POSITION = [x,y];
    CURRENT_ANGLE = angle;
    
def get_gyro():
    '''
    Returns the current gyroscope reading.
    Note that the reading is cumulative and unbounded.
    So a reading of 3600 means 10 full rotations clockwise have occured.
    '''
    return gyro.value();
    
def get_sonar():
    '''
    Returns the current distance reading in cm
    from the sonar sensor.
    '''
    return sonar.value()/10

def speak(s):
    '''
    Speaks the given string.
    '''
    ev3.Sound.speak(s).wait()

def rotate_by(degrees, speed=TURNING_SPEED):
    '''
    Performs a fixed-point (approximately) rotation by a set
    number of degrees.
    
    Accurate to within about 8 degrees after the axle diameter
    was calibrated (according to the gyroscope sensor).  Works 
    better for smaller angles < 180.
    
    NOTE: will undershoot rotation when dragging a bottle/can.
    
    degrees - number of relative degrees to turn
    speed - turning speed in wheel rotations / second
    '''
    print("Rotating",degrees,"degrees...");
    
    # recalibrated after 1.2
    calibration = 2; # for stop_action=brake
    #calibration = 1.2; # for stop_action=coast
    arclength = (degrees * math.pi * (AXLE_DIAMETER+calibration)) / 360.0
    wheel_angle = (arclength / WHEEL_CIRCUMFERENCE) * 360.0
    left_wheel.run_to_rel_pos(position_sp=wheel_angle, speed_sp=TURNING_SPEED, stop_action="brake")
    right_wheel.run_to_rel_pos(position_sp=-wheel_angle, speed_sp=TURNING_SPEED, stop_action="brake")
    #time.sleep(abs(wheel_angle/pwr/10))
    return abs(wheel_angle/TURNING_SPEED); # return time to sleep by
    
def reverse(distance, speed=MOVE_SPEED):
    '''
    Attempts to reverse by the specified distance.
    '''
    return forward(-distance,speed);
    
def forward(distance, speed=MOVE_SPEED):
    '''
    Attempts to move forward by the specified distance.
    Negative distances make the robot reverse.
    
    Accurate to within approximately 1cm (no longer true).
    
    distance - distance to move in centimetres (cm)
    speed - move speed in wheel rotations / second
    
    returns time taken to perform action in milliseconds (ms)
    '''
    #calibration = -0.5;
    calibration = -0.;
    if(distance < 0.): distance = distance - calibration;
    else: distance = distance + calibration;
    wheel_angle = (distance / WHEEL_CIRCUMFERENCE) * 360.0
    left_wheel.run_to_rel_pos(position_sp=wheel_angle, speed_sp=speed, stop_action="brake")
    right_wheel.run_to_rel_pos(position_sp=wheel_angle, speed_sp=speed, stop_action="brake")
    return abs(wheel_angle /speed);
    
def close_pincers(wait=True):
    '''
    Closes the pincers.
    
    WARNING: this does not check whether or not the pincers
             are in already a closed position.
    '''
    pincer.run_to_rel_pos(position_sp=-2.5*360, speed_sp=360);
    if(wait):
        pincer.wait_while('running');
    
def open_pincers(wait=True):
    '''
    Opens the pincers.
    
    WARNING: this does not check whether or not the pincers
             are in already an open position.
    '''
    pincer.run_to_rel_pos(position_sp=2.5*360, speed_sp=360);
    if(wait):
        pincer.wait_while('running');
        
    
def rotate_to(angle):
    '''
    Rotates to the target angle in degrees based on the current
    belived orientation.
    
    NOTE: does NOT update the CURRENT_ORIENTATION
    '''
    # Convert to radians
    currangle = CURRENT_ANGLE*math.pi/180.;
    angle = angle*math.pi/180.;
    
    # Calculate relative angle
    relative_angle = (angle-currangle)
    while relative_angle < 0.0:
            relative_angle += math.pi * 2
    relative_angle = ((relative_angle + math.pi) % (math.pi * 2)) - math.pi
   
    # Convert back to degrees
    relative_angle *= 180./math.pi;
    
    # Perform rotation
    time = rotate_by(relative_angle);
    return time;
    
def move_to(x,y):
    '''
    Moves to the specified (x,y) position and update the believed
    position and orientation.
    '''
    start_angle = gyro.value();
    global CURRENT_ANGLE
    global CURRENT_POSITION
    print("MOVING TO (%d,%d)"%(x,y));
    target = [x,y]
    curr = CURRENT_POSITION;
    
    # Angle calculations done in radians
    currangle = CURRENT_ANGLE*math.pi/180.;
    # Calculate angle between vectors
    angle = math.atan2(y - curr[1], x - curr[0])
    # map angle to a domain of [-pi, pi]
    while angle < 0.0:
            angle += math.pi * 2
    angle = ((angle + math.pi) % (math.pi * 2)) - math.pi
   
    # Account for current angle
    relative_angle = (angle-currangle)
    while relative_angle < 0.0:
            relative_angle += math.pi * 2
    relative_angle = ((relative_angle + math.pi) % (math.pi * 2)) - math.pi
    
    # Convert to degrees
    angle *= 180./math.pi;
    relative_angle *= 180./math.pi;
    
    print("CURRENT_ANGLE =",CURRENT_ANGLE);
    print("ANGLE =",angle);
    print("RELATIVE ANGLE =",relative_angle);
        
    # Euclidean distance to target position
    distance = math.sqrt((curr[0]-target[0])*(curr[0]-target[0])+(curr[1]-target[1])*(curr[1]-target[1]));
        
    #time.sleep(rotate_by(relative_angle)+0.15);
    time.sleep(rotate_by(relative_angle)+0.1);
    
    # Try correct angle if necessary
    end_angle = gyro.value();
    diff_angle = end_angle - start_angle;
    print("Rotated",diff_angle,"degrees");
    while(abs(relative_angle-diff_angle) >= 9):
        print("Correcting angle by",relative_angle-diff_angle,"degrees");
        time.sleep(rotate_by(0.8*(relative_angle-diff_angle))+0.15); # don't want to overshoot
        end_angle = gyro.value();
        diff_angle = end_angle - start_angle;
        print("Rotated",diff_angle,"degrees total");
    
    print("DISTANCE =",distance)
    
    move_time = forward(distance);
    time.sleep(move_time)#+0.1);
    '''
    if auto_close:
        while(time.time()-start < move_time+0.2):
            dist = sonar.value()/10; # mm -> cm
            if(dist < 7):
                print("Closing pincers automatically!");
                close_pincers(False);
            time.sleep(0.15);
    else:
        time.sleep(move_time+0.1);
    '''
    # Update belief of pose
    end_angle = gyro.value();
    diff_angle = end_angle - start_angle;
    #CURRENT_ANGLE = CURRENT_ANGLE+diff_angle;
    CURRENT_ANGLE = angle
    # TODO: could calculate what it should really be based on our 
    #       end angle when moving forwards?
    CURRENT_POSITION = [x,y];
    print("Final angle:",CURRENT_ANGLE);
    print("Final position: (%.0f,%.0f)"%(x,y));
   