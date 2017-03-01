#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib, warnings, rospy, math, pickle
import numpy as np
import scipy.io as sio
import os
import sys
import subprocess
from os.path import exists
from sensor_msgs.msg import LaserScan


def RepresentsInt(s):
    try: 
        int(s)
        return True
    except ValueError:
        return False
        
def RepresentsFloat(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


limit=0
counter=1
save_file = ''
def laser_listener():
    print len(sys.argv)
    bag_set=0
    bag_file = ''
    duration = 0
    global limit
    global save_file
    rospy.init_node('laser_listener', anonymous=True)
    sub_topic = ''
    if (len(sys.argv ) != 5) :
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
        duration=input('Please insert approximate bag duration (s): ')
        limit=(duration-1)*40
        print limit,' scans will be converted'
        print 'Waiting laser scans to be published on {0}'.format(sub_topic)
    else:
        print sys.argv[1]
        print sys.argv[2]
        print sys.argv[3]
        print sys.argv[4]
        save_file = str(sys.argv[2])
        '''
        if not os.path.isfile(save_file):
            while True :
                try:
                    save_file=raw_input('Destination file name: ')
                    if os.path.isfile(save_file):
                        bag_set=1
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        '''
        bag_file = str(sys.argv[1])
        if not os.path.isfile(bag_file):
            while True :
                try:
                    bag_file=raw_input('Enter bag file name: ')
                    if os.path.isfile(bag_file):
                        
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        bag_set=1
        sub_topic = str(sys.argv[3])
        while sub_topic == '':
            sub_topic = raw_input('Please input laser scan topic: ')
        #print 'Will subscribe from:', sub_topic 
        
        #print sys.argv[4]
        duration = sys.argv[4]
        if not (RepresentsInt(sys.argv[4]) or RepresentsFloat(sys.argv[4])):
            while True:
                duration = raw_input('Enter Duration In Seconds: ')
                if RepresentsInt(duration):
                    break
                else:
                    print 'Try again'
    limit=(int(duration)-1)*40
    print limit,' scans will be converted'
    print 'Waiting laser scans to be published on {0}'.format(sub_topic)                
    print "Destination File : {0}".format(save_file)
    print "Bag File : {0}".format(bag_file)
    print 'Will subscribe from:'.format(sub_topic)
    print 'Duration : {0}'.format(duration)
            
    rospy.Subscriber(sub_topic,LaserScan,converter)
    while not rospy.is_shutdown(): 
        if (bag_set==1):
            proc = subprocess.Popen("rosbag play "+bag_file+" -q --duration "+str(duration)+" &", shell=True)
            rospy.spin()
        else:
            rospy.spin()
    exit()
    

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
        b['time_increment']=time_increment
        sio.savemat(save_file,b)
        
        print 'Data is stored'
        rospy.signal_shutdown('Exit')

    counter=counter+1

if __name__ == '__main__':
    laser_listener()
    exit()
