# rpyc_server.py
#
# Start the rpyc_bot.py script on the ev3 bot
# first, then run this file.
#
# Run using python3 on the server/laptop:
#   python3 rpyc_server.py

import sys
import math
import time
import rpyc

USB = '169.254.238.226';
BLUETOOTH = '192.168.50.2'; # *.*.*.1 for laptop?
HOSTNAME = 'ev3dev';
'''
#conn = rpyc.classic.connect('localhost') # host name or IP address of the EV3
conn = rpyc.classic.connect(BLUETOOTH) # host name or IP address of the EV3

print(conn.modules.os.getcwd())

bot = conn.modules['client']
bot.move_to(0,3);
bot.move_to(0,0);
'''
if __name__ == '__main__':
    conn = rpyc.connect(BLUETOOTH, port=12345)
    bot = conn.root;
    '''bot.set(0,0,0);
    bot.move_to(30,40);
    bot.close_pincers();
    bot.forward(30,100);
    bot.reverse(30,100);
    bot.move_to(0,0);
    bot.open_pincers();
    bot.rotate_by(90);
    bot.rotate_to(0);
    bot.rotate_by(-90);'''
    
    #bot.set(0,0,0);
    #bot.reverse_to(-30,0);
    #bot.reverse_to(0,-30);
    #bot.reverse_to(30,-30);
    #bot.move_to(-30,0);
    #bot.move_to(-30,90);
    #bot.move_to(30,120);
    #bot.close_pincers();
    #bot.move_to(120,60);
    bot.open_pincers();
    #bot.reverse(30);
    #bot.set(
    #bot.move_to(0,0);
