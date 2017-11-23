#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import sys

image = None
pixel = (20,60,80)
cans = []
obs = []

# mouse callback function
def pick_things(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDOWN:
#		pixel = image[y,x]
		print(x, y)
		cans.append((x, y, 13))
		cv2.circle(image,(x,y),13,(0,255,0),1)
	
	elif event == cv2.EVENT_RBUTTONDOWN:
#		pixel = image[y,x]
		print(x, y)
		obs.append((x, y, 9))
		cv2.circle(image,(x,y),10,(0,0,255),1)

	elif event == cv2.EVENT_FLAG_SHIFTKEY:
		cv2.destroyWindow("bgr")

	cv2.imshow("bgr", image)
	


def main():
	global image, pixel, cans, obs # so we can use it in mouse callback

	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
	ret, frame = cap.read()
	ret, frame = cap.read()
	image = frame

	if frame is None:
	    print ("the image read is None............")
	    return
	bgr = cv2.namedWindow("bgr")
	cv2.imshow("bgr",image)

	cv2.setMouseCallback("bgr", pick_things)	

	cv2.waitKey(0)
	print("DONE")

	#Do other stuff after getting obs and cans
	cv2.destroyAllWindows()


if __name__=='__main__':
	main()