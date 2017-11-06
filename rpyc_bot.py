# rpyc_bot.py
#
# Exposes several ev3 bot movement functions to rpyc calls.
#
# Ensure the bot.py file is in the same working directory
# as this script on the ev3 bot.
#
# Run using python3 on the ev3 bot itself:
#    python3 rpyc_bot.py

import time
import rpyc
from rpyc.utils.server import ThreadedServer

import bot

class BotService(rpyc.Service):
    '''
    RPyC service exposing ev3 bot functions
    '''
    def exposed_move_to(self, x, y):
        bot.move_to(x,y);
        
    def exposed_set(self, x, y, angle):
        bot.set(x,y,angle);
        
    def exposed_forward(self, distance, speed=None):
        if speed is not None:
            time.sleep(bot.forward(distance,speed))
        else:
            time.sleep(bot.forward(distance));
        
    def exposed_reverse(self, distance, speed=None):
        if speed is not None:
            time.sleep(bot.reverse(distance,speed))
        else:
            time.sleep(bot.reverse(distance));
        
    def exposed_open_pincers(self):
        bot.open_pincers();
        
    def exposed_close_pincers(self):
        bot.close_pincers();
        
    def exposed_rotate_to(self, angle):
        startdegrees = bot.get_gyro();
        time.sleep(bot.rotate_to(angle)+0.1);
        enddegrees = bot.get_gyro();
        s = "Rotated "+str(enddegrees-startdegrees)+" degrees"
        print(s);
        return s;
        
    def exposed_rotate_by(self, angle):
        startdegrees = bot.get_gyro();
        time.sleep(bot.rotate_by(angle)+0.1);
        enddegrees = bot.get_gyro();
        s = "Rotated "+str(enddegrees-startdegrees)+" degrees"
        print(s);
        return s;
    

if __name__ == '__main__':
    server = ThreadedServer(BotService, port=12345)
    server.start()