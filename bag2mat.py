#!/usr/bin/env python
__author__="athanasia sapountzi, lydakis andreas, stavrinos georgios"

import roslib, warnings, rospy, math, pickle
import numpy as np
import scipy.io as sio
import os
import sys
from os.path import exists
from sensor_msgs.msg import LaserScan

limit=0
counter=1
save_file = ''
def laser_listener():

    global limit
    global save_file
    rospy.init_node('laser_listener', anonymous=True)
    sub_topic = ''
    while True :
        try:
            save_file=raw_input('Enter data file name: ')
            break
        except SyntaxError:
            print 'Try again'
    d = os.path.dirname(save_file)
    if d!='':
        if not os.path.exists(d):
            os.makedirs(d)
    print 'Data will be saved as {0}'.format(save_file)
    while sub_topic == '':
		sub_topic = raw_input('Please input laser scan topic: ')
    print 'Will subscribe from:', sub_topic 
    temp=input('Please insert approximate bag duration (s): ')
    limit=(temp-1)*40
    print limit,' scans will be converted'
    print 'Waiting laser scans to be published on {0}'.format(sub_topic)
    rospy.Subscriber(sub_topic,LaserScan,converter)
    rospy.spin()

def converter(laserscan):

    global counter,limit,ranges,intensities,angle_increment,scan_time,time_increment,angle_min,angle_max, save_file

    if counter==1:
        print 'Converting bag file...'
        ranges=np.array(laserscan.ranges)
        angle_min=laserscan.angle_min
        angle_max=laserscan.angle_max
        angle_increment=laserscan.angle_increment
        scan_time=laserscan.scan_time
        intensities=laserscan.intensities
        time_increment=laserscan.time_increment
        
    else:

        ranges=np.vstack((ranges,np.array(laserscan.ranges)))
        intensities=np.hstack((intensities,laserscan.intensities))

    if counter==limit:
        print 'Conversion is complete'
        print 'Saving data'
        b={}
        b['ranges']=ranges
        b['angle_increment']=angle_increment
        b['scan_time']=scan_time
        b['angle_min']=angle_min
        b['angle_max']=angle_max
        b['intensities']=intensities
        sio.savemat(save_file,b)
        sio.savemat('bagfile_test',b)
        b['time_increment']=time_increment
        print 'Data is stored'
        rospy.signal_shutdown('Exit')

    counter=counter+1

if __name__ == '__main__':
    laser_listener()
