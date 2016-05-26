#!/usr/bin/env python
import roslib, rospy
import numpy as np
import scipy.spatial.distance as dist
from laser_clustering.msg import ClustersMsg

trace_array = []
trace_count = False
trace_results = []
cls_results = []
#traced_clusters attribute: is used to store the points of the traced clusters in a slide window manner
# -> format: [ CL1=[[pTW1], [pTW2], ...] ,CL2=[[pTW1],[pTW2],...] , ... ]
#    where 
#	- CLn: indicates a cluster in the slide window
#	- pTWn: indicates the points of the specific cluster in the timewindow n (TWn)
#Therefore CL1, which represents cluster 1, is a list of points for <max_num_slide_window> timewindow
traced_clusters = []
max_cls = 0
max_num_slide_window = 4
first_trace = True
clusters_publisher = None
frame_id = ''
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)


def init():
    global clusters_publisher, frame_id, dt, speed_, z_scale

    rospy.init_node('laser_overlap_trace')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    output_clusters_topic = rospy.get_param('~output_clusters_topic','~clusters')
    frame_id = rospy.get_param('~frame_id','laser_link')
    dt = rospy.get_param('~dt', 25)
    speed_ = rospy.get_param('~human_speed', 5)

    z_scale = float(speed_*dt) / float(3600)
   
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
    global trace_count, max_num_slide_window

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
    new_cluster = True

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    array_sizes = np.array(clusters_msg.array_sizes)

    prev_index = 0

    if(len(array_sizes) > 0):

        for i in range(0, len(array_sizes)):
            xk = []
            yk = []
            zk = []
            for j in range(prev_index, prev_index+array_sizes[i]-1):
                xk.append(xi[j])
                yk.append(yi[j])
                zk.append(zi[j])
            cls.append([np.array(xk), np.array(yk), np.array(zk)])
            prev_index = array_sizes[i]-1

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
                new_cluster = True
            else:
                first = cls
                second = trace_array
                new_cluster = False

            for i in range(0, len(first)):
                if new_cluster == False:
                    coord = get_centroid(first[i], True)

                for j in range(0, len(second)):
                #eucl_dist for every combination
                    if new_cluster:
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

                if new_cluster:
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

            if len(trace_results) == max_num_slide_window:
                if first_trace:
                    create_trace()
                    first_trace = False
                else:
                    continue_trace()

                final_clusters = getClusterSet()
                x_ = []
                y_ = []
                z_ = []
                arr_sz = []
                for i in range(0, len(final_clusters)):
                    sz = len(final_clusters[i][0])
                    arr_sz.append(sz)
                    for j in range(0, sz):
                        x_.append(final_clusters[i][0][j])
                        y_.append(final_clusters[i][1][j])
                        z_.append(final_clusters[i][2][j])

                cls_msg = ClustersMsg()
                cls_msg.header.stamp = rospy.Time.now()
                cls_msg.header.frame_id = frame_id
                cls_msg.x = x_
                cls_msg.y = y_
                cls_msg.z = z_
                cls_msg.array_sizes = arr_sz
                clusters_publisher.publish(cls_msg)

                del trace_results[0]
                del cls_results[0]
                max_cls = len(results)
    else:
        del trace_results[:]
        del cls_results[:]
        del traced_clusters[:]
        first_trace = True
        trace_count = False
        max_cls =0


#combines the points of each tracked cluster.
#returns a list of the trackedCluster points
def getClusterSet():
    global traced_clusters
    global z_scale

    final_clusters = []

    #for every tracked cluster
    for i in range(0, len(traced_clusters)):
        if len(traced_clusters[i]) == 0:
             continue

        #initialize x,y,z of trackedCluster_i
        xar = np.array(traced_clusters[i][0][0])
        yar = np.array(traced_clusters[i][0][1])
        zar = np.array(traced_clusters[i][0][2])

        #z dimension changes: 
        #Each timewindow resets the time, thus the time (z dimention) begins from 0.0.
        #In order to combine the points of the available timewindows, the time should increment periodically in each set of timewindow points.
        for j in range(1, len(traced_clusters[i])):
             xar = np.append(xar,traced_clusters[i][j][0])
             yar = np.append(yar,traced_clusters[i][j][1])
             B = traced_clusters[i][j][2].copy()

             increment = zar[len(zar) - 1] + z_scale
             B[::1]  += increment
             zar =  np.append(zar,B)	

        final_clusters.append([xar,yar,zar])

    return final_clusters



if __name__ == '__main__':
    init() 
