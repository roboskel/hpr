#!/usr/bin/env python
import roslib, rospy
import numpy as np
import scipy.spatial.distance as dist
from laser_clustering.msg import ClustersMsg
import hungarian as hg
import hpr_track as tr

trace_array = []
prev_centroids = []
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

#num_clusters: it is an array that defines the number of clusters that consist a traced cluster.
num_clusters = []

max_cls = 0
max_num_slide_window = 4
first_trace = True
clusters_publisher = None
frame_id = ''
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)

trackHandler = None
twCounter = 0


def init():
    global clusters_publisher, frame_id, dt, speed_, z_scale
    global trackHandler, twCounter

    rospy.init_node('laser_overlap_trace')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    output_clusters_topic = rospy.get_param('~output_clusters_topic','~clusters')
    frame_id = rospy.get_param('~frame_id','laser_link')
    dt = rospy.get_param('~dt', 25)
    speed_ = rospy.get_param('~human_speed', 5)

    z_scale = float(speed_*dt) / float(3600)

    trackHandler = tr.Tracker()
   
    rospy.Subscriber(input_clusters_topic, ClustersMsg, tracking_procedure)

    clusters_publisher = rospy.Publisher(output_clusters_topic, ClustersMsg, queue_size=10)
    while not rospy.is_shutdown():
        rospy.spin()


#Get the mean point (x,y) of a set of laser-scan points.
# -> choice parameter declares the first (True) or the last (False) part of cluster
def get_centroid(cluster, choice):

    if choice == False:
        z_filter = np.where(cluster[2]==cluster[2][len(cluster[2])-1])
    else:
        z_filter = np.where(cluster[2]==cluster[2][0])

    a = np.array([cluster[0][z_filter], cluster[1][z_filter]])

    return np.mean(a, axis=1)


def create_trace():

    global trace_results, cls_results
    global traced_clusters, max_cls
    global num_clusters

    temp = []
    counter = 1
    num_clusters = np.zeros(max_cls, dtype=int)

    #for each cluster
    for j in range(0,max_cls):
        for i in range(0, len(trace_results)):

            if len(trace_results[i]) > counter:
                counter = counter +1
       
                try:
                    temp.append(cls_results[i][trace_results[i].index(-1)])
                except ValueError:
                   if j in trace_results[i]:
                        print 'ValueError exception: with trace_results = {} and j = {}'.format(trace_results, j)
                        temp.append(cls_results[i][trace_results[i].index(j)])
                   else:
                        continue

                num_clusters[j] = num_clusters[j] + 1
            if j in trace_results[i]:
                temp.append(cls_results[i][trace_results[i].index(j)])
                num_clusters[j] = num_clusters[j] + 1

        traced_clusters.append(temp)
        temp=[]

def continue_trace():

    global trace_results, cls_results
    global traced_clusters
    global num_clusters

    last_element = len(trace_results) - 1

    for i in range(0, len(trace_results[last_element])):
        index = trace_results[last_element][i]

        if index != -1:
            if len(traced_clusters[index]) == max_num_slide_window:
                del traced_clusters[index][0]
                num_clusters[index] = num_clusters[index] - 1

            traced_clusters[index].append(cls_results[last_element][i])
            num_clusters[index] = num_clusters[index] + 1
        else:
            traced_clusters.append([cls_results[last_element][i]])
            num_clusters = np.append(num_clusters, 1)

def update_centroids(clusters):
    global prev_centroids

    for i,cl in enumerate(clusters): #for each cluster get the centroid of the last scan
        prev_centroids.append(get_centroid(cl, False))

def fill_centroids(clusters):
    current_centroids = []

    for i,cl in enumerate(clusters): #for each cluster get the centroid of the first scan
        current_centroids.append(get_centroid(cl, True))

    return current_centroids

def compute_cost(currentCentroids, previousCentroids):
    #the rows == new clusters and the cols == previous clusters
    costmap = [[euclidean(i,j) for j in previousCentroids] for i in currentCentroids]

    return costmap

def euclidean(point1, point2):
    return np.linalg.norm(point1-point2)

#Update the associations in the situations where clusters appear or diappear from timewindow to timewindow
def get_associations(associations, newSize, oldSize):
    #more clusters in this timewindow
    if newSize > oldSize:
        for row in range(0, newSize):
            if row not in (item[0] for item in associations):
                associations.append((row, -1))

    #less clusters in this timewindow
    else:
        for col in range(0, oldSize):
            if col not in (item[1] for item in associations):
                associations.append((-1, col))

    return associations

#based on the costmap apply hungarian algorithm to determine the minimum cost for this assingement problem
def hungarian_approach(costmap):
    hungarian = hg.Hungarian(costmap)
    hungarian.calculate()
    associations = hungarian.get_results()

    return associations

#based on the costmap apply a greedy algorithm (get the minimum cost each time) for this assingement problem
def greedy_approach(costmap):
    costmap = np.array(costmap)
    shape = costmap.shape #num of rows and num of cols in the 2D array
    associations = []

    if shape[0] > shape[1]:
        mindimension = shape[1]
    else:
        mindimension = shape[0]

    for k in range(0, mindimension):
        index = divmod(costmap.argmin(),costmap.shape[1])   #finds the index of the min value of the costmap
        associations.append(index)

        #set infinite value to each row and column of the index because we want a 1-1 association
        costmap[index[0], :] = float("Inf") 
        costmap[:, index[1]] = float("Inf")  

    return associations

"""
def association_procedure(clusters_msg):
    global frame_id, clusters_publisher
    global prev_centroids

    current_centroids = []
    associations = []
    cls = []

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    scan_time = clusters_msg.scan_time

    array_sizes = np.array(clusters_msg.array_sizes)

    prev_index = 0

    if len(array_sizes) > 0:

        #pre-processing step to form the clusters with their respective points
        for i,size in enumerate(array_sizes):
            indices = range(prev_index, prev_index+size-1)  #the indices of the desired points (points of each cluster)
            cls.append([np.array(xi[indices]), np.array(yi[indices]), np.array(zi[indices])])   #add a new cluster

            prev_index = size - 1

        #list is not empty
        if prev_centroids:
            current_centroids = fill_centroids(cls)
            costmap = compute_cost(current_centroids)

            #associations = greedy_approach(costmap)  
            associations = hungarian_approach(costmap)

            #handle the issues where more or less clusters exist in the current frame
            if not (len(current_centroids) == len(prev_centroids)):
                associations = get_associations(associations, len(current_centroids), len(prev_centroids))
            
            print 'size = {} , {}'.format(len(current_centroids), len(prev_centroids))
            del prev_centroids[:]


            print 'resulted associations = ',associations

        compose_tracks(cls, associations)
        update_centroids(cls)

        clusters_publisher.publish(clusters_msg)
        '''
        cls_msg = ClustersMsg()
        cls_msg.header.stamp = rospy.Time.now()
        cls_msg.header.frame_id = frame_id
        cls_msg.x = x_
        cls_msg.y = y_
        cls_msg.z = z_
        cls_msg.frames = clusters_msg.frames
        cls_msg.array_sizes = arr_sz
        cls_msg.scan_time = scan_time
        cls_msg.num_clusters = index_clusters
        clusters_publisher.publish(cls_msg)
        '''

        #del trace_results[0]
        #del cls_results[0]
        #max_cls = len(results)
    else:
        del trace_results[:]
        del cls_results[:]
        del traced_clusters[:]
        del trace_array[:]
        num_clusters = []
        first_trace = True
        trace_count = False
        max_cls =0
        clusters_publisher.publish(clusters_msg)

"""
def compose_tracks(cls, associations):
    global traced_clusters

    if traced_clusters:
        for item in associations:
            # a new cluster
            if item[1] == -1:
                traced_clusters.append([cls[item[0]]])
            else:
                #there is a cluster less
                if item[0] == -1:
                    traced_clusters[item[1]].append([])
                else:
                    traced_clusters[item[1]].append([cls[item[0]]])     #put the new cluster to the appropriate list, as the association specified.
                    #TODO: check when to remove some clusters

    else:
        for cluster in cls:
            traced_clusters.append([cluster])


def tracking_procedure(clusters_msg):
    global frame_id, clusters_publisher, z_scale
    global prev_centroids
    global trackHandler, twCounter

    current_centroids = []
    associations = []
    cls = []

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    scan_time = clusters_msg.scan_time

    array_sizes = np.array(clusters_msg.array_sizes)

    prev_index = 0

    if len(array_sizes) > 0:
        #pre-processing step to form the clusters with their respective points
        for i,size in enumerate(array_sizes):
            indices = range(prev_index, prev_index+size-1)  #the indices of the desired points (points of each cluster)
            cls.append([np.array(xi[indices]), np.array(yi[indices]), np.array(zi[indices])])   #add a new cluster

            prev_index = size - 1

        trackHandler.update()
       
        if trackHandler.has_previous():   
            association_procedure(cls)
        else:
            for newCluster in cls:
                trackHandler.add_track(newCluster)

        x_, y_, z_, clusterSizes, clusterSeparation = trackHandler.combine_tracks(z_scale)

        cls_msg = ClustersMsg()
        cls_msg.header.stamp = rospy.Time.now()
        cls_msg.header.frame_id = frame_id
        cls_msg.x = x_
        cls_msg.y = y_
        cls_msg.z = z_
        cls_msg.frames = clusters_msg.frames
        cls_msg.array_sizes = clusterSeparation
        cls_msg.scan_time = scan_time
        cls_msg.num_clusters = clusterSizes
        clusters_publisher.publish(cls_msg)
        
    else:
        clusters_publisher.publish(clusters_msg)


def association_procedure(newClusters):
    global trackHandler

    #previousTracks = trackHandler.previous()
    previousIds, previousClusters = trackHandler.previous()
    previousCentroids = [get_centroid(cl, True) for cl in previousClusters]

    currentCentroids = [get_centroid(cl, False) for cl in newClusters]

    #currentCentroids = fill_centroids(newClusters)
    costmap = compute_cost(currentCentroids, previousCentroids)

    #associations = greedy_approach(costmap)  
    associations = hungarian_approach(costmap)

    #handle the issues where more or less clusters exist in the current frame
    if not (len(currentCentroids) == len(previousCentroids)):
        associations = get_associations(associations, len(currentCentroids), len(previousCentroids))

    #print 'associations = ',associations
    for item in associations:
        #new cluster that does not associates with a track of the previous frame
        if item[1] == -1:
            #check whether it fits with a track of some <X> previous frames
            #If not -> new cluster
            trackHandler.add_track(newClusters[item[0]])
        else:
            if not (item[0] == -1):
                trackId = previousIds[item[1]]
                trackHandler.update_track(trackId, newClusters[item[0]])



def convert_tracks():
    global trackHandler
    global z_scale

    index_clusters = []
    final_clusters = []

    tracks = trackHandler.track_array

    #for every tracked cluster
    for i in range(0, len(tracks)):
        if len(tracks[i]) == 0:
             index_clusters.append(0)
             continue

        #initialize x,y,z of trackedCluster_i
        xar = np.array(tracks[i][0][0])
        yar = np.array(tracks[i][0][1])
        zar = np.array(tracks[i][0][2])

        index_clusters.append(len(xar))

        #z dimension changes: 
        #Each timewindow resets the time, thus the time (z dimention) begins from 0.0.
        #In order to combine the points of the available timewindows, the time should increment periodically in each set of timewindow points.
        for j in range(1, len(tracks[i])):
             xar = np.append(xar,tracks[i][j][0])
             yar = np.append(yar,tracks[i][j][1])
             B = tracks[i][j][2].copy()
           
             index_clusters.append(len(tracks[i][j][0]))

             increment = zar[len(zar) - 1] + z_scale
             B[::1]  += increment
             zar =  np.append(zar,B)    

        final_clusters.append([xar,yar,zar])

    return final_clusters, index_clusters

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
    global num_clusters

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

    scan_time = clusters_msg.scan_time

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
                num_cl = np.where(num_clusters == 0)
                num_clusters = np.delete(num_clusters, num_cl, 0)

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
            if (len(results) < len(trace_array) and len(traced_clusters) != 0):
                rm_list = []
                #remove the unnecessary clusters
                for j in range(0, len(trace_array)):
                    if j not in results:
                        rm_list.append(j)

                for i in rm_list:
                    del traced_clusters[i][:]
                    num_clusters[i] = 0
                    
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

                final_clusters, index_clusters = getClusterSet()
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
                cls_msg.frames = clusters_msg.frames
                cls_msg.array_sizes = arr_sz
                cls_msg.scan_time = scan_time
                cls_msg.num_clusters = index_clusters
                clusters_publisher.publish(cls_msg)

                del trace_results[0]
                del cls_results[0]
                max_cls = len(results)
    else:
        del trace_results[:]
        del cls_results[:]
        del traced_clusters[:]
        del trace_array[:]
        num_clusters = []
        first_trace = True
        trace_count = False
        max_cls =0
        clusters_publisher.publish(clusters_msg)


#combines the points of each tracked cluster.
#returns a list of the trackedCluster points
def getClusterSet():
    global traced_clusters
    global z_scale

    index_clusters = []
    final_clusters = []

    #for every tracked cluster
    for i in range(0, len(traced_clusters)):
        if len(traced_clusters[i]) == 0:
             index_clusters.append(0)
             continue

        #initialize x,y,z of trackedCluster_i
        xar = np.array(traced_clusters[i][0][0])
        yar = np.array(traced_clusters[i][0][1])
        zar = np.array(traced_clusters[i][0][2])

        index_clusters.append(len(xar))

        #z dimension changes: 
        #Each timewindow resets the time, thus the time (z dimention) begins from 0.0.
        #In order to combine the points of the available timewindows, the time should increment periodically in each set of timewindow points.
        for j in range(1, len(traced_clusters[i])):
             xar = np.append(xar,traced_clusters[i][j][0])
             yar = np.append(yar,traced_clusters[i][j][1])
             B = traced_clusters[i][j][2].copy()
           
             index_clusters.append(len(traced_clusters[i][j][0]))

             increment = zar[len(zar) - 1] + z_scale
             B[::1]  += increment
             zar =  np.append(zar,B)	

        final_clusters.append([xar,yar,zar])

    return final_clusters, index_clusters



if __name__ == '__main__':
    init() 
