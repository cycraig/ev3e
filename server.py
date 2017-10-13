"""
1) Start the ev3 bot (with ev3-dev installed)
2) Connect the bot to the server device (pair with Bluetooth or plug in USB and connect)
3) Login to the bot through ssh with username 'robot', password 'maker'.
4) Copy the client.py script onto the bot using scp.
5) Run server.py on the server device.
6) Run client.py on the bot through ssh.

Run using python2:
    python server.py
    
The IP address and port default to 192.168.50.1 and 10000 respectively.
They can also be specified as optional positional arguments. The IP address 
and port are those of the server, not the bot.

e.g.
    python server.py 192.168.50.1 8888
    
NOTE: the bot must be paired and connected for the server to create the socket.
    
Partially based on:
https://github.com/lichiukenneth/EV3Dev-Python-Socket-Connection
"""

import socket
import sys
import select
import argparse
import time
from clog import *
logInfo("-------------------- SERVER --------------------");

USB = '169.254.238.226';
BLUETOOTH = '192.168.50.1';

# Parse arguments
parser = argparse.ArgumentParser(description='EV3 controller server', add_help=True)
parser.add_argument("ip",nargs='?', default=BLUETOOTH, help="IP address to bind on, defaults to '192.168.50.1' for Bluetooth");
parser.add_argument("port", nargs='?', type=int, default=10000, help="Port to bind on, defaults to 10000");
args = parser.parse_args();

HOST = args.ip;
PORT = args.port;

# Create a TCP/IP socket
server_address = (HOST, PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
logInfo('Starting server on %s port %s' % server_address)
sock.bind(server_address)
# Allow only one client connection
sock.listen(1)

# Accept new connection if the previous was closed
while True:
    # select with timeout because sock.accept() blocks
    # and doesn't listen to Ctrl-C on Windows
    try:
        ready = select.select((sock,), (), (), 0.1)
        if (ready[0]):
            # Wait for a connection
            logInfo("Waiting for a connection...")
            connection, client_address = sock.accept()
            def send(msg):
                logServer(msg)
                connection.sendall(msg)
            def recv():
                data = connection.recv(1024)
                logClient(data);
                return data;
            def latency():
                start = time.time()
                send('latency')
                recv()
                end = time.time()
                logInfo("Latency: %.2f ms (round trip)"%((end-start)*1000))
            try:
                logInfo('Connection from '+str(client_address));
                
                latency()
                
                # Recv between them to avoid all the commands piling up
                send("reset");
                recv()
                send("set:0,0,270");
                recv();
                send("rotateto:0");
                recv()
                '''send("move:60,0");
                recv()
                send("move:0,-60");
                recv()
                send("move:120,-30");
                recv()
                send("close");
                recv()
                send("move:0,0");
                recv()
                send("open");
                recv()'''
                #send("rotate:90");
                #recv()
                # LOOK AT LINK FOR STEERING CALCULATIONS ETC.
                # https://github.com/mimoralea/kalman-karma/blob/master/particle-filter/ev3_wall_trace_localize.py 
                
            except KeyboardInterrupt:
                logError("Ctrl-C");
            finally:
                # Clean up the connection
                connection.close()
                logInfo('Connection closed.\n')
    except KeyboardInterrupt:
        logError("Ctrl-C");
        sys.exit(1)