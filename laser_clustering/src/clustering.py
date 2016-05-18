#!/usr/bin/env python

import roslib, rospy
import numpy as np
import mytools as mt #DBSCAN function and perquisites are stored here
from laser_wall_extraction.msg import BufferMsg
from laser_clustering.msg import ClustersMsg

buffer_topic = ''
num_c = 3
clusters_publisher = None
frame_id = ''

def init():
    global num_c, buffer_topic, clusters_publisher, frame_id

    num_c = rospy.get_param('~num_c',3)
    buffer_topic = rospy.get_param('~buffer_topic','laser_wall_extraction/buffer')
    frame_id = rospy.get_param('~frame_id','laser_link')
    clusters_topic = rospy.get_param('~clusters_topic','~clusters')

    rospy.init_node('laser_clustering')
   
    rospy.Subscriber(buffer_topic, BufferMsg, clustering_procedure)

    clusters_publisher = rospy.Publisher(clusters_topic, ClustersMsg, queue_size=10)
    while not rospy.is_shutdown():  
        rospy.spin()


def clustering_procedure(buffer):
    global num_c, clusters_publisher, frame_id

    clear_data = np.zeros((len(buffer.x), 3))
    for i in range(0,len(buffer.x)):
        clear_data[i] = ([buffer.x[i], buffer.y[i], buffer.z[i]])

    Eps, cluster_labels= mt.dbscan(clear_data, num_c)

    clustersmsg = ClustersMsg()
    clustersmsg.header.stamp = rospy.Time.now()
    clustersmsg.header.frame_id = frame_id
    clustersmsg.clusters = cluster_labels
    clustersmsg.x = buffer.x
    clustersmsg.y = buffer.y
    clustersmsg.z = buffer.z
    clusters_publisher.publish(clustersmsg)
    #print cluster_labels



    
if __name__ == '__main__':
    init()   
