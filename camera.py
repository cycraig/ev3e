import cv2
import cv2.aruco as aruco
import numpy as np
import math
import threading
import time
from collections import deque

# Capture external webcam
cameraMatrix = np.genfromtxt('cameraMatrix.txt');
distCoeffs = np.genfromtxt('distCoeffs.txt');

# Indices of the AR markers in the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
aruco_params =  aruco.DetectorParameters_create()
botID = 26;
goalID = 42;

# Global variables --- constantly updated by the camera thread
# If the bot/goal AR marker is not found, these will hold the last known values
botPosition = None;
botOrientation = None;
goalPosition = None;

class Camera():
    def __init__(self, camIndex=0, medianFrames=1, show=True, debug=False, record=True):
        # TODO: medianFrames -> number of frames over which to take the median
        # can do that outside of camera class?
        self.camIndex = camIndex;
        self.cam = None;
        self.debug = debug;
        self.show = show;
        self.running = False;
        self.path = None;
        self.video_writer = None;
        self.record = True
        
        # TODO: to keep track of last few frames to take median over
        self.botPosDeque = deque();
        self.botOriDeque = deque();
        self.goalPosDeque = deque();
        
    def start(self):
        self.running = True;
        self.thread = threading.Thread(target=self._run, args=())
        self.thread.daemon = True # Daemonize thread
        self.thread.start()       # Start the execution
        
    def stop(self):
        self.running = False;
        self.thread.join();
    
    def set_path(self, path):
        self.path = path
        
    def set_resolution(self,x,y):
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, int(x))
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, int(y))
        print(str(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)),str(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        
    def _run(self):
        self.cam = cv2.VideoCapture(self.camIndex);
        #self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        self.set_resolution(800, 600)
        if self.record:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter('output.avi',fourcc, 20.0, (800, 600))
        try:
            while(self.running):
                self._update();
        finally:
            # When everything done, release the capture
            self.cam.release()
            if self.record:
                self.video_writer.release()
            if(self.show is True):
                cv2.destroyAllWindows()
        
    def _update(self):
        global botPosition, botOrientation, goalPosition;
        
        foundBot = False;
        foundGoal = False;
        
        ret, frame = self.cam.read()
        if ret == True:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
            if ids is not None:
                frame = aruco.drawDetectedMarkers(frame, corners) #,ids)
                # adjust marker length from 0.168 (16.8 cm) to 0.0504=(0.168*(30/100)) to account for cm change? -- doesn't work, do conversion manually
                rvecs, tvecs, _ =  aruco.estimatePoseSingleMarkers(corners, 0.168, cameraMatrix, distCoeffs);
                #print rvecs, tvecs
                for i in range(len(ids)):
                    #aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                    
                    # Calculate bot position and orientation
                    if(ids[i][0] == botID):
                        foundBot = True;
                        botPosition = np.mean(corners[i],axis=1)[0]#*30./100.; # 30cm per 100px
                        estTrans = cv2.estimateRigidTransform(np.array([[-0.84,0.84],[0.84,0.84],[0.84,-0.84],[-0.84,-0.84]],dtype=np.float32),corners[i][0], fullAffine=True);
                        botOrientation = math.atan2(estTrans[1][0],estTrans[0][0])*180./np.pi # degrees
                        
                        # TODO:
                        #if(len(self.botPosDeque) >= self.medianFrames):
                        #    self.botPosDeque.popleft();
                        #self.botPosDeque.append(botPosition);
                    
                    # Calculate goal position
                    if(ids[i][0] == goalID):
                        foundGoal = True;
                        goalPosition = np.mean(corners[i],axis=1)[0]#*30./100.; # 30cm per 100px
                        
                # TODO: Reset the deques if we didn't find the bot/goal
                #if not foundBot:
                    #self.botPosDeque
                        
                if self.debug is True:
                    print("ARUCO RETURN =============================");
                    print("Goal position:", goalPosition);
                    print("Bot position:", botPosition);
                    print("Bot orientation:", botOrientation);
                    
            # Display the resulting frame
            if(self.show is True):
            
                # Draw path
                if self.path is not None:
                    #self.path[0] = (botPosition[0],botPosition[1])
                    p = (int(botPosition[0]),int(botPosition[1]))
                    pp = (int(self.path[0][0]),int(self.path[0][1]))
                    cv2.line(frame,p,pp,(0,255,0),2)#(180,105,255),2)
                    try:
                        if(len(self.path) > 1):
                            for i in range(len(self.path)-1):
                                p = (int(self.path[i][0]),int(self.path[i][1]))
                                pp = (int(self.path[i+1][0]),int(self.path[i+1][1]))
                                cv2.line(frame,p,pp,(0,255,0),2)#(180,105,255),2)
                    except:
                        pass
                if self.record:
                    self.video_writer.write(frame)
                cv2.imshow('frame',frame);
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False;
                    
def get_bot_position():
    global botPosition;
    # return bot position in pixels
    return botPosition;
    
def get_bot_orientation():
    global botOrientation;
    # return bot orientation in degrees
    return botOrientation;

def get_goal_position():
    global goalPosition;
    # return bot position in pixels
    return goalPosition;

if __name__ == "__main__":
    try:
        cam = Camera();
        cam.start();
        while(True):
            print("Goal position:", goalPosition);
            print("Bot position:", botPosition);
            print("Bot orientation:", botOrientation);
            time.sleep(0.1);
    finally:
        # clean-up thread, otherwise hangs on Ctrl-C
        cam.stop();