#!/usr/bin/env python

# coding=utf-8
# license removed for brevity

#import roslib
import rospy
import sys
from air_hockey.msg import hockey_pose
from air_hockey.srv import todoOK
import cv2
import numpy as np
import time

global frecuency
global width
global height
#global cy_m
#global cx_m
global a
global b
global c
global d
global borders
global msg

frecuency = 30
width = 1.04
height = 0.71
#cx_m = 0.0
#cy_m = 0.0

a = 0
b = 15
c = 530
d = 380
borders = (a,b,c,d)

msg = hockey_pose()

def color_chase(frame):
    
    img=frame.copy()
    #pass to HSV
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Creating filter
    low=np.array([55,50,0])
    up=np.array([85,255,255])
    #filtering
    mask=cv2.inRange(img_hsv,low,up)
    out=cv2.bitwise_and(frame,frame,mask=mask)
    #soft processing
    out=cv2.erode(out,None,iterations=5)
    out=cv2.dilate(out,None,iterations=3)
    return out


def draw(frame, out):
    try:
        im=out.copy()
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        #threshold of image
        ret,thresh = cv2.threshold(imgray,10,255,0)
        #getting contours 
        _,contours,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours , -1,(0,255,0),2)
        #calculating the center and drawing it
        cnt = contours[0]
        Moments = cv2.moments(cnt)
        cx = int(Moments["m10"]/Moments["m00"])
        cy = int(Moments["m01"]/Moments["m00"])
        cv2.circle(frame,(cx,cy),7,(0,255,0),-1)
        #calculating i meters and print it
        cy_m = width - round((cx*width)/(borders[0]+borders[2]),3) # COORDENADA Y
        cx_m = height - round((cy*height)/(borders[1]+borders[3]),3) # COORDENADA X

        cx_m = cx_m*100
        cy_m = cy_m*100
        
        coord = "("+str(cx_m)+", "+str(cy_m)+")"
        rospy.loginfo(coord)

        cv2.putText(frame,coord,(20,20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		
        return cx_m, cy_m
    except:
        pass


# Client of state service
def handle_client_ask_vision(req):
    return False
    


# Main program
def vision():
    pub_pos_disco = rospy.Publisher('pos_disco', hockey_pose, queue_size=1)
    rospy.init_node('vision', anonymous=True)

    s = rospy.Service('ask_vision', todoOK, handle_client_ask_vision)
    
    #rospy.wait_for_service('client_ask_vision')
    rate = rospy.Rate(frecuency)

	# SELECCION DE CAMARA O VIDEO
    cap = cv2.VideoCapture(2) #PARA USAR LA WEBCAM(1) PARA USAR CAM PC(0)

	#Init variables
    sq2=np.ones((9,9))
    flag = True
    coor_x = 0
    coor_y = 0
    if borders == (0,0,0,0):
    	flag = False

    while not rospy.is_shutdown(): 
    	if flag==True:
    		try:
				#Read frames
    			if cap.isOpened():
    				
    				retval,frame = cap.read()
    				
					#crop ROI
    				frame = frame[int(borders[1]):int(borders[1]+borders[3]),int(borders[0]):int(borders[0]+borders[2])]
					#get Puck
    				blue = color_chase(frame)
    				coor_x, coor_y = draw(frame,blue)

    			else:
    				rospy.logwarn("No se esta leyendo el video")

    		except:
    			pass
		 
		#cv2.destroyAllWindows()

    	now = rospy.get_rostime()
    	msg.tiempo = now.secs + 1e-9*now.nsecs
    	msg.x = coor_x
    	msg.y = coor_y
        
        #rospy.loginfo("("+str(coor_x)+ ", "+str(coor_y)+")")

		#posicion_str = "Disco en [%f %f]", x_disco, y_disco
		#rospy.loginfo("Disco en [%f %f]", x_disco, y_disco)
    	pub_pos_disco.publish(msg)
    	rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    try:
        vision()
    except rospy.ROSInterruptException:
        pass
