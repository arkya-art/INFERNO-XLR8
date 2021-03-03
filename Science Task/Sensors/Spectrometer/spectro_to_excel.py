#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from pcl_msgs.msg import Vertices
import os

import xlsxwriter 
workbook = xlsxwriter.Workbook(os.path.join(os.path.dirname(os.path.abspath(__file__)),'Example2.xlsx')) 
print('workbook opened')
worksheet = workbook.add_worksheet("sheet1")
print('worksheet added')  
kc=0
row=0
for k in range(17):
    worksheet.write(row, kc, 'field')
    kc=kc+1


def callback(data):
    
    
    global row
    row=row+1
    column = 0
    
    
    #content = ["ankit", "rahul", "priya", "harshita", "sumit", "neeraj", "shivam"] 
    #content = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,]  
    # iterating through content list 
    for i in data.data : 
        print('running loop')

        
  
        # write operation perform 
        worksheet.write(row, column, str(i)) 
  
        # incrementing the value of row by one 
        # with each iteratons. 
        column=column+1
    
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('spectro', Float32MultiArray  , callback)

    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()
    workbook.close()
    print('done')


if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        
        pass
