# Run using python3 on the bot:
#   python3 move.py

"""
1) Start the ev3 bot (with ev3-dev installed)
2) Connect the bot to the server device (pair with Bluetooth or plug in USB and connect)
3) Login to the bot through ssh with username 'robot', password 'maker'.
4) Copy the client.py script onto the bot using scp.
5) Run server.py on the server device.
6) Run client.py on the bot through ssh.

Run using python3:
    python3 client.py
    
The IP address and port of the server default to 192.168.50.1 and 10000 respectively.
They can also be specified as optional positional arguments. The IP address and port 
are those of the server, not the bot.

e.g.
    python3 client.py 192.168.50.1 8888
     
NOTE: the bot must be paired and connected for the server to create the socket.
    
Partially based on:
https://github.com/lichiukenneth/EV3Dev-Python-Socket-Connection
"""

import socket
import sys
import math
import time
import errno
import select
import argparse
from ev3dev import ev3

USB = '169.254.238.226';
BLUETOOTH = '192.168.50.1';

# Parse arguments
parser = argparse.ArgumentParser(description='EV3 controller client', add_help=True)
parser.add_argument("ip",nargs='?', default=BLUETOOTH, help="IP address of server, defaults to %s for Bluetooth"%BLUETOOTH);
parser.add_argument("port", nargs='?', type=int, default=10000, help="Port of socket on server, defaults to 10000");
args = parser.parse_args();
HOST = args.ip
PORT = args.port

# Create a TCP/IP socket
server_address = (HOST,PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

### PORTS
right_wheel = ev3.LargeMotor('outA');
left_wheel = ev3.LargeMotor('outD');
pincer = ev3.MediumMotor('outB');
gyro = ev3.GyroSensor('in3'); # can remove port specification, still works
gyro.mode='GYRO-ANG'
sonar = ev3.UltrasonicSensor('in4');
#assert sonar.connected;
sonar.mode = 'US-DIST-CM'

MOVE_SPEED = 360; # wheel rotations / second
TURNING_SPEED = 300; # wheel rotations / second
AXLE_DIAMETER = 12.2; #cm (approximate, includes 2*half of wheel widths)
WHEEL_RADIUS = 2.8; #cm (approximate)
WHEEL_CIRCUMFERENCE = 2.*math.pi*WHEEL_RADIUS;
CURRENT_POSITION = [0,0]; # x,y
CURRENT_ANGLE = 0; # degrees
def rotate(degrees, speed=TURNING_SPEED):
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
    
def move(distance, speed=MOVE_SPEED):
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
    time = rotate(relative_angle);
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
        
    time.sleep(rotate(relative_angle)+0.15);
    
    # Try correct angle if necessary
    end_angle = gyro.value();
    diff_angle = end_angle - start_angle;
    print("Rotated",diff_angle,"degrees");
    while(abs(relative_angle-diff_angle) >= 9):
        print("Correcting angle by",relative_angle-diff_angle,"degrees");
        time.sleep(rotate(0.8*(relative_angle-diff_angle))+0.15); # don't want to overshoot
        end_angle = gyro.value();
        diff_angle = end_angle - start_angle;
        print("Rotated",diff_angle,"degrees total");
    
    print("DISTANCE =",distance)
    
    move_time = move(distance);
    time.sleep(move_time+0.1);
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

# -----------------------------------------------------------------------------
def parse_command(data, sock):
    '''
    Parses a command from the server.
    
    Should send back a message for any command so the server knows we received.
    Also helps prevent commands bunching together.
    '''
    global CURRENT_POSITION, CURRENT_ANGLE
    if(data == "reset"):
        '''
        Resets the believed position to (0,0)
        and orientation to 0 degrees.
        '''
        CURRENT_POSITION = [0,0];
        CURRENT_ANGLE = 0;
        sock.sendall('Reset position and orientation'.encode());
        
    elif(data.startswith("set:")):
        '''
        Sets the belief state to position (x,y) with angle 
        theta (clockwise rotation).
        
        e.g. set:10,-10,180
        '''
        pos = data.split(':')[1].split(',')
        x = float(pos[0]);
        y = float(pos[1]);
        angle = float(pos[2]);
        CURRENT_POSITION = [x,y];
        CURRENT_ANGLE = angle;
        sock.sendall(('Set belief state to (%.2f,%.2f) with angle %d'% (x,y,angle)).encode());
        
    elif(data.startswith("move:")):
        '''
        Move to specified (x,y) position.
        
        e.g. move:5,2.7
        '''
        pos = data.split(':')[1].split(',')
        x = float(pos[0]);
        y = float(pos[1]);
        move_to(x,y);
        sock.sendall(('Moved to position (%.2f,%.2f)'% (x,y)).encode());
        
    elif(data == "close"):
        '''
        Closes the pincers.
        
        WARNING: this does not check whether or not the pincers
                 are in already a closed position.
        '''
        close_pincers();
        sock.sendall('Closed pincers'.encode());
        
    elif(data == "open"):
        '''
        Opens the pincers.
        
        WARNING: this does not check whether or not the pincers
                 are in already an open position.
        '''
        open_pincers();
        sock.sendall('Opened pincers'.encode());
        
    elif(data == "gyro"):
        '''
        Sends the current orientation reading
        from the gyroscope.
        '''
        sock.sendall(str(gyro.value()).encode());
        
    elif(data == "dist"):
        '''
        Sends the current distance reading in cm
        from the sonar sensor.
        '''
        sock.sendall(str(sonar.value()/10).encode());
        
    elif(data.startswith("rotate:")):
        '''
        Rotates the specified degrees (integer only).
        
        e.g. rotate:90
         
        NOTE: for calibration, does NOT update the 
              current orientation.
        '''
        degrees = int(data.split(':')[1]);
        startdegrees = gyro.value();
        start = time.time();
        time.sleep(rotate(degrees)+0.1);
        enddegrees = gyro.value();
        s = "Rotated "+str(enddegrees-startdegrees)+" degrees"
        print(s);
        sock.sendall(s.encode());
        
    elif(data.startswith("rotateto:")):
        '''
        Rotates to the specified angle (based on the 
        belief orientation).
        
        e.g. rotateto:0
         
        NOTE: for calibration, does NOT update the 
              current orientation.
        '''
        angle = float(data.split(':')[1]);
        startdegrees = gyro.value();
        start = time.time();
        time.sleep(rotate_to(angle)+0.1);
        enddegrees = gyro.value();
        s = "Rotated "+str(enddegrees-startdegrees)+" degrees"
        print(s);
        sock.sendall(s.encode());
        
    elif(data.startswith("forward:")):
        '''
        Drives forward by the specified distance (cm).
        
        e.g. forward:10.2
        
        NOTE: for calibration, does NOT update the 
              current position.
        '''
        dist = float(data.split(':')[1]);
        time.sleep(move(dist))
        sock.sendall(("Moved forward by %.2f"%dist).encode());
        
    elif data == 'latency':
        '''
        Latency check from the server.
        '''
        sock.sendall('client-latency'.encode());
        
    else:
        '''
        UNKNOWN COMMAND
        
        Speaks the unknown command.
        '''
        print("UNKNOWN COMMAND: '%s'"%data);
        ev3.Sound.speak(data).wait()
        sock.sendall(("UNKNOWN COMMAND: '%s'"%data).encode());
        

# Connect the socket to the port where the server is listening
print('Connecting to server on %s port %s' % server_address);
sock.connect(server_address)

# TODO: connection hangs when finished, implement reconnecting...
try:
    while True:
        try:
            ready = select.select((sock,), (), (), 0.1)
            if (ready[0]):
                data = sock.recv(128)
                if(data is not None and len(data) > 1):
                    data = data.decode();
                    print('Received: '+data);
                    # TODO: create a command buffer with delimited commands
                    parse_command(data, sock);
                    #break; 
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                print('No data available');
                sleep(0.5);
                continue
            else:
                # a "real" error occurred
                raise
                #print e
                #sys.exit(1)
finally:
    print("Closing socket...");
    sock.close()
