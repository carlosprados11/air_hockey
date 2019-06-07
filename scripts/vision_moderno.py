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

global a
global b
global c
global d
global borders
global msg
global punch

frecuency = 30
width = 1.04
height = 0.71

a = 0
b = 15
c = 515
d = 380
borders = (a,b,c,d)

msg = hockey_pose()
punch = hockey_pose()

def color_chase(frame,low,up):
    
    img=frame.copy()
    #pass to HSV
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #filtering
    mask=cv2.inRange(img_hsv,low,up)
    out=cv2.bitwise_and(frame,frame,mask=mask)
    #soft processing
    out=cv2.erode(out,None,iterations=7)
    out=cv2.dilate(out,None,iterations=4)
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

        #cv2.putText(frame,coord,(20,20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
		
        return cx_m, cy_m
    except:
        pass


# Client of state service
def handle_client_ask_vision(req):
    return False
    


# Main program
def vision():
    pub_pos_disco = rospy.Publisher('pos_disco', hockey_pose, queue_size=1)
    pub_pos_punch = rospy.Publisher('pos_punch', hockey_pose, queue_size=1)
    
    rospy.init_node('vision', anonymous=True)

    s = rospy.Service('ask_vision', todoOK, handle_client_ask_vision)
    
    #rospy.wait_for_service('client_ask_vision')
    rate = rospy.Rate(frecuency)

	# SELECCION DE CAMARA O VIDEO
    cap = cv2.VideoCapture(2) #PARA USAR LA WEBCAM(1) PARA USAR CAM PC(0)

	#Init variables
    flag = True

    if borders == (0,0,0,0):
    	flag = False

    while not rospy.is_shutdown(): 
    	if flag==True:
    		try:
				#Read frames
    			if cap.isOpened():
    				
    				_,frame = cap.read()
    				
					#crop ROI
    				frame = frame[int(borders[1]):int(borders[1]+borders[3]),int(borders[0]):int(borders[0]+borders[2])]
					
					#get puck VERDE
    				lowverde=np.array([55,50,0]) 
    				upverde=np.array([85,255,255])

					# get puck AZUL
    				lowazul=np.array([95,50,0])
    				upazul=np.array([110,255,255])

					# color chase VERDE
    				verde = color_chase(frame,lowverde,upverde)
    				azul = color_chase(frame,lowazul,upazul)

    				coordverde=draw(frame,verde)     
    				coordazul=draw(frame,azul)
    				rospy.logwarn(coordazul[0])
    				rospy.logwarn(coordazul[1])

    			else:
    				rospy.logwarn("No se esta leyendo el video")

    		except:
    			pass
		 
		#cv2.destroyAllWindows()
		
    	now = rospy.get_rostime()
    	msg.tiempo = now.secs + 1e-9*now.nsecs
        punch.tiempo = now.secs + 1e-9*now.nsecs

    	msg.x = coordverde[0]
    	msg.y = coordverde[1]

        punch.x = coordazul[0]
    	punch.y = coordazul[1]
        
        coord = "Pose Disco ("+str(coordverde[0])+", "+str(coordverde[1])+")"
        rospy.loginfo(coord)

        coord = "Pose Punch ("+str(coordazul[0])+", "+str(coordazul[1])+")"
        rospy.loginfo(coord)
        
    	pub_pos_disco.publish(msg)
        pub_pos_punch.publish(punch)
    	rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    try:
        vision()
    except rospy.ROSInterruptException:
        pass
