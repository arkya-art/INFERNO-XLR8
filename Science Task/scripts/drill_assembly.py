#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from pcl_msgs.msg import Vertices
drill = Vertices()
drill.vertices = [0, 0, 0, 0, 0, 0, 0]
drill_x=0 #up-down movement
drill_y=0 #drill on off
drill_z=0#container rotation
drill_xpwm=255
drill_ypwm=255
drill_zpwm=255
drill_zangle=90

def find_drill_movements(data):
    global drill_x
    global drill_y
    global drill_z
    global drill_xpwm
    global drill_ypwm
    global drill_zpwm
    global drill_zangle
    
    #for drill up-down movement
    x = data.axes[0]
    if(x==0):
        drill_x =0
    if(x==1):
        drill_x=1
    if(x==-1):
        drill_x=2

    #for drill on off
    drill_clock = data.buttons[0]
    drill_anticlock = data.buttons[2]
    if((drill_clock == 0 and drill_anticlock == 0) or (drill_clock==1 and drill_anticlock ==1)) :
        drill_y = 0
    if(drill_clock == 1 and drill_anticlock == 0) :
        drill_y = 1
    if(drill_clock == 0 and drill_anticlock == 1) :
        drill_y = 2

    #for container rotation
    left= data.buttons[4]
    right=data.buttons[5]
    if((right == 0 and left == 0) or (right==1 and left ==1)) :
        drill_z = 0
    if(right == 1 and left == 0) :
        drill_z = 1
    if(right == 0 and left == 1) :
        drill_z = 2

    drill.vertices[0]=drill_x
    drill.vertices[1]=drill_xpwm
    drill.vertices[2]=drill_y
    drill.vertices[3]=drill_ypwm
    drill.vertices[4]=drill_z
    drill.vertices[5]=drill_zpwm
    drill.vertices[6]=drill_zangle



        
def move_drill(data):
    rate = rospy.Rate(10)
    find_drill_movements(data)
    pub = rospy.Publisher('move_drill', Vertices ,queue_size=1)  #sends the Vertices message to "robotic_drill" topic
    pub.publish(drill)
    rate.sleep()

def joy_to_drill(): #function to recieve values from joystick through joy_node of joy package
    rospy.init_node('joy_to_drill', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("joy_for_arm", Joy, move_drill, queue_size=1)
    rate.sleep()
    rospy.spin()
    
if __name__ == '__main__':
    joy_to_drill()
