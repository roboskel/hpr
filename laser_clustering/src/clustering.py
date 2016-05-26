#!/usr/bin/env python

import roslib, rospy
import numpy as np
import mytools as mt #DBSCAN function and perquisites are stored here
from laser_wall_extraction.msg import BufferMsg
from laser_clustering.msg import ClustersMsg
from laser_clustering.msg import ClusterLabelsMsg

buffer_topic = ''
num_c = 3
clusters_publisher = None
frame_id = ''
publish_cluster_labels = False
cluster_labels_publisher = None

def init():
    global num_c, buffer_topic, clusters_publisher, frame_id
    global publish_cluster_labels, cluster_labels_publisher

    rospy.init_node('laser_clustering')

    num_c = rospy.get_param('~num_c', 3)
    buffer_topic = rospy.get_param('~buffer_topic', 'laser_wall_extraction/buffer')
    frame_id = rospy.get_param('~frame_id', 'laser_link')
    clusters_topic = rospy.get_param('~clusters_topic', '~clusters')
    publish_cluster_labels = rospy.get_param('~publish_cluster_labels', False)
    cluster_labels_topic = rospy.get_param('~cluster_labels_topic', '~cluster_labels')

    rospy.Subscriber(buffer_topic, BufferMsg, clustering_procedure)

    clusters_publisher = rospy.Publisher(clusters_topic, ClustersMsg, queue_size=10)
    if publish_cluster_labels:
        cluster_labels_publisher = rospy.Publisher(cluster_labels_topic, ClusterLabelsMsg, queue_size=10)
    while not rospy.is_shutdown():  
        rospy.spin()


def clustering_procedure(buffer):
    global num_c, clusters_publisher, frame_id, publish_cluster_labels, cluster_labels_publisher

    if len(buffer.x) == 0: #the area is empty!
        clustersmsg = ClustersMsg()
        clustersmsg.header.stamp = rospy.Time.now()
        clustersmsg.header.frame_id = frame_id
        clustersmsg.x = []
        clustersmsg.y = []
        clustersmsg.z = []
        #empty array_sizes means that anyone listening to this message won't loop through the data
        clustersmsg.array_sizes = []
        clusters_publisher.publish(clustersmsg)
    else:
        clear_data = np.zeros((len(buffer.x), 3))
        for i in range(0,len(buffer.x)):
            clear_data[i] = ([buffer.x[i], buffer.y[i], buffer.z[i]])

        Eps, cluster_labels= mt.dbscan(clear_data, num_c)

        max_label=int(np.amax(cluster_labels))

        arr_sz = []
        x_ = []
        y_ = []
        z_ = []

        for k in range(1,max_label+1) :
            filter = np.where(cluster_labels==k)
            if len(filter[0])>40 :
                xk = np.array(buffer.x)[filter]
                yk = np.array(buffer.y)[filter]
                zk = np.array(buffer.z)[filter]
                for i in range(0, len(xk)):
                    x_.append(xk[i])
                    y_.append(yk[i])
                    z_.append(zk[i])
                arr_sz.append(len(xk))

        clustersmsg = ClustersMsg()
        clustersmsg.header.stamp = rospy.Time.now()
        clustersmsg.header.frame_id = frame_id
        #clustersmsg.clusters = cluster_labels
        clustersmsg.x = x_
        clustersmsg.y = y_
        clustersmsg.z = z_
        clustersmsg.array_sizes = arr_sz
        clusters_publisher.publish(clustersmsg)

        if publish_cluster_labels:
            clusterlabelsmsg = ClusterLabelsMsg()
            clusterlabelsmsg.header = clustersmsg.header
            clusterlabelsmsg.cluster_labels = cluster_labels
            cluster_labels_publisher.publish(clusterlabelsmsg)

if __name__ == '__main__':
    init()   
