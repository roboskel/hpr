#!/usr/bin/env python
import roslib, rospy
import numpy as np
import scipy.spatial.distance as dist
from laser_clustering.msg import ClustersMsg

trace_array = []
trace_count = False
trace_results = []
cls_results = []
traced_clusters = []
max_cls = 0
first_trace = True
track_parts = 4
clusters_publisher = None
frame_id = ''

#TODO delete data after 2*timewindow time of not receiving data
#del trace_results[:]
#del cls_results[:]
#del traced_clusters[:]
#first_trace = True
#trace_count = False
#max_cls =0


def init():
    global clusters_publisher, frame_id

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    output_clusters_topic = rospy.get_param('~output_clusters_topic','~clusters')
    frame_id = rospy.get_param('~frame_id','laser_link')

    rospy.init_node('laser_overlap_trace')
   
    rospy.Subscriber(input_clusters_topic, ClustersMsg, overlap_trace)

    clusters_publisher = rospy.Publisher(output_clusters_topic, ClustersMsg, queue_size=10)
    while not rospy.is_shutdown():
        rospy.spin()


#choice parameter declares the first (True) or the last (False) part of cluster
def get_centroid(cluster, choice):

    if choice == False:
        z_filter = np.where(cluster[2]==cluster[2][len(cluster[2])-1])
    else:
        z_filter = np.where(cluster[2]==cluster[2][0])

    mean_x = np.mean(cluster[0][z_filter])
    mean_y = np.mean(cluster[1][z_filter])

    return np.array((mean_x, mean_y))


def create_trace():

    global trace_results, cls_results
    global traced_clusters, max_cls

    temp = []
    counter = 1

    #for each cluster
    for j in range(0,max_cls):
        for i in range(0, len(trace_results)):
            if len(trace_results[i]) > counter:
                counter = counter +1
                temp.append(cls_results[i][trace_results[i].index(-1)])
            if j in trace_results[i]:
                temp.append(cls_results[i][trace_results[i].index(j)])

        traced_clusters.append(temp)
        temp=[]


def continue_trace():

    global trace_results, cls_results
    global traced_clusters

    last_element = len(trace_results) - 1

    for i in range(0, len(trace_results[last_element])):
        index = trace_results[last_element][i]

        if index != -1:
            if len(traced_clusters[index]) == 4:
                del traced_clusters[index][0]

            traced_clusters[index].append(cls_results[last_element][i])
        else:
            traced_clusters.append([cls_results[last_element][i]])



#Tracks the human walks by fitting the new set of points(frame) to the closest cluster.
#First it finds the centroids of the last part(frame) of the previous clusters.
#Then it connects a new cluster to a previous one, by getting the minimum euclidean distance of the new cluster's centroid and the previous clusters' centroids.
#Finally, the function calculates an array of trace results, where:
#	a) each index denotes the number of a new coming cluster
#	b) the value at each index denotes the number of a previous cluster that most likely fits
def overlap_trace(clusters_msg):
    global trace_array  #the centroid point of each cluster at every scan
    global trace_count, track_parts

    global trace_results  #the position of the clusters at each scan
    global cls_results   #the set of points of each cluster at every scan
    global traced_clusters, first_trace, max_cls
    global frame_id, clusters_publisher

    cls = []

    error = 100
    min_dist = -1.0
    index = 0
    list_dist = []
    temp_list = []
    flag = True

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    cluster_labels = np.array(clusters_msg.clusters)

    max_label=int(np.amax(cluster_labels))

    for k in range(1,max_label+1) :
        filter = np.where(cluster_labels==k)
        if len(filter[0])>40 :
            [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]
            cls.append([xk,yk,zk])


    if len(trace_array) == 0:
        for i in range(0, len(cls)):
            trace_array.append(get_centroid(cls[i], False))
            trace_results.append([i])
            cls_results.append([cls[i]])
    else:
        #condition where the clusters have been reduced
        if trace_count:
            traced_clusters2 = [x for x in traced_clusters if x != []]
            traced_clusters = list(traced_clusters2)
            trace_count = False

        if len(cls) > len(trace_array):
            first = trace_array
            second = cls
            flag = True
        else:
            first = cls
            second = trace_array
            flag = False

        for i in range(0, len(first)):
            if flag == False:
                coord = get_centroid(first[i], True)

            for j in range(0, len(second)):
            #eucl_dist for every combination
                if flag:
                    coord = get_centroid(second[j], True)
                    d = dist.euclidean(coord,first[i])
                    temp_list.append(d)
                else:
                    d = dist.euclidean(coord,second[j])
                    temp_list.append(d)

            list_dist.append(temp_list)

            temp_list = []

        min_val = -1.0
        row = 0
        num = 0
        temp_num = 0
        col = -1
        results = []

        temp_list = list(list_dist)
        length = len(list_dist)

        for i in range(0,len(cls)):
            results.append(-1)

        while num<length:
            for i in range(0, len(temp_list)):
                if min_val == -1.0:
                    min_val = min(temp_list[i])
                    temp_row = i
                else:
                    if min_val > min(temp_list[i]):
                        min_val = min(temp_list[i])
                        temp_row = i

            for i in range(0, len(list_dist)):
                if min_val in list_dist[i]:
                    row = i
                    break

            if flag:
                results[list_dist[row].index(min_val)] = row

            else:
                results[row] = list_dist[row].index(min_val)

            ind = temp_list[temp_row].index(min_val)
            del temp_list[temp_row]

            for i in range(0, len(temp_list)):
                temp_list[i][ind] = error

            num = num + 1
            col = -1
            row =0
            min_val = -1.0

        #print 'results = ' , results

        # a cluster disappears
        if len(results) < len(trace_array) and len(traced_clusters) != 0:
            rm_list = []
            #remove the unnecessary clusters
            for j in range(0, len(trace_array)):
                if j not in results:
                    rm_list.append(j)
            for i in range(0, len(rm_list)):
                del traced_clusters[rm_list[i]][:]
                trace_count = True

        #remove previous and add the new ones
        del trace_array[:]
        for i in range(0, len(cls)):
            trace_array.append(get_centroid(cls[i], False))

        trace_results.append(results)
        cls_results.append(cls)

        #the maximum number of clusters
        if max_cls < len(results):
            max_cls = len(results)


        #Sort our data based on their cluster.
        #print '-----------------'
        #print 'trace results = ' , trace_results
        #for k in range(0, len(trace_results)):
            #trace_results[k] = np.sort(trace_results[k])
        #print 'trace results = ' , trace_results
        #print '-----------------'

        #PLACEHOLDER FOR REAL MESSAGE INFO
        cls_msg = ClustersMsg()
        cls_msg.header.stamp = rospy.Time.now()
        cls_msg.header.frame_id = frame_id
        cls_msg.clusters = clusters_msg.clusters
        cls_msg.x = clusters_msg.x
        cls_msg.y = clusters_msg.y
        cls_msg.z = clusters_msg.z
        clusters_publisher.publish(cls_msg)

        if len(trace_results) == track_parts:
            if first_trace:
                create_trace()
                first_trace = False
            else:
                continue_trace()
            del trace_results[0]
            del cls_results[0]
            max_cls = len(results)

if __name__ == '__main__':
    init() 