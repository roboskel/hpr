#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib, warnings, rospy, math, pickle
import numpy as np
import scipy.io as sio

from sensor_msgs.msg import LaserScan

limit=0
counter=1
save_file ='mat_files/'
def laser_listener():

    global limit
    global save_file
    rospy.init_node('laser_listener', anonymous=True)
    sub_topic = ''
    #save_file = 'mat_files/'
    filename=input('Enter data file name: ')
    #if save_file=='mat_files/':
    save_file=save_file+filename
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
        print 'saving data'
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
        #exit()

    counter=counter+1

if __name__ == '__main__':
    laser_listener()
