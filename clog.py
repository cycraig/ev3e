# clog.py
#
# Provides convenience functions to log with colour terminal output (if available)
# and file logging.

import logging
import platform
import os,sys

logger = logging.getLogger('server')
logger.setLevel(logging.DEBUG)
fhandler = logging.FileHandler('log.txt')
formatter = logging.Formatter('%(asctime)s %(levelname)s: %(message)s')
fhandler.setFormatter(formatter)
logger.addHandler(fhandler) 

# Terminal colours
class tcol:
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    WHITE = '\033[0m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
# Determine whether the current terminal supports colours
colourEnabled = None;
try:
    import colorama
    colorama.init()
    colourEnabled = True;
except:
    # https://gist.github.com/ssbarnea/1316877
    if hasattr(sys.stdout, "isatty") and sys.stdout.isatty() or \
        ('TERM' in os.environ and os.environ['TERM']=='ANSI'):
        if platform.system() == 'Windows' and not ('TERM' in os.environ and os.environ['TERM']=='ANSI'):
            colourEnabled = False;
        else:
            colourEnabled = True;
    pass;

def logInfo(message):
    # Log information messages about what the server's doing
    msg = '  [Info] ' + message;
    if colourEnabled: print tcol.WHITE + msg + tcol.ENDC
    else: print msg;
    logger.info(message);

def logClient(message):
    # Log commands the client sends to the server
    msg = '[Client] ' + message;
    if colourEnabled: print tcol.GREEN + msg + tcol.ENDC
    else: print msg;
    logger.info(msg);

def logServer(message):
    # Log commands the server sends to the client
    msg = '[Server] ' + message;
    if colourEnabled: print tcol.CYAN + msg + tcol.ENDC
    else: print msg;
    logger.info(msg);
    
def logError(message):
    # Log server error
    msg = ' [ERROR] ' + message;
    if colourEnabled: print tcol.RED + msg + tcol.ENDC
    else: print msg;
    logger.error(message)