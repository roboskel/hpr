#!/usr/bin/env python
import roslib, rospy
import numpy as np
from laser_clustering.msg import ClustersMsg
import hungarian as hg
import hpr_track as tr

import scipy.spatial.distance as dist

from mayavi import mlab

trace_publisher = None
tracks_publisher = None

frame_id = ''
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)

trackHandler = None
errorDist = 1.0

count = 0


def init():
    global trace_publisher, tracks_publisher, frame_id, dt, speed_, z_scale
    global trackHandler

    rospy.init_node('laser_overlap_trace')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    output_tracks_topic = rospy.get_param('~output_tracks_topic','~tracks')
    output_clusters_topic = rospy.get_param('~output_clusters_topic','~clusters')

    frame_id = rospy.get_param('~frame_id','laser_link')
    dt = rospy.get_param('~dt', 25)
    speed_ = rospy.get_param('~human_speed', 5)

    z_scale = float(speed_*dt) / float(3600)

    trackHandler = tr.Tracker()
   
    rospy.Subscriber(input_clusters_topic, ClustersMsg, tracking_procedure)

    tracks_publisher = rospy.Publisher(output_tracks_topic, ClustersMsg, queue_size=10)
    trace_publisher = rospy.Publisher(output_clusters_topic, ClustersMsg, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()


#Tracks the incoming clusters by fitting the new set of points (timewindow) to the closest cluster of the previous timewindow (association procedure).
#   a) It publishes the tracks until now (maximum <num_slide_window> length)
#   b) It publishes the trace (cluster) of each track in the current timewindow for further analysis (eg walk analysis)
def tracking_procedure(clusters_msg):
    global frame_id, trace_publisher, tracks_publisher, z_scale
    global trackHandler

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
            previousCentroids, currentCentroids = association_procedure(cls)
        else:
            for newCluster in cls:
                trackHandler.add_track(newCluster)

        x_, y_, z_, clusterSizes, clusterSeparation, idTracks = trackHandler.combine_tracks(z_scale)
        twX, twY, twZ, traceSeparation, idArray = trackHandler.get_last_trace()

        ##############################################
        #temp_visualize(x_, y_, z_, clusterSizes, clusterSeparation, idTracks, previousCentroids, currentCentroids)
        ##############################################

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
        cls_msg.id_array = idTracks
        tracks_publisher.publish(cls_msg)

        trace_msg = ClustersMsg()
        trace_msg.header.stamp = rospy.Time.now()
        trace_msg.header.frame_id = frame_id
        trace_msg.x = twX
        trace_msg.y = twY
        trace_msg.z = twZ
        trace_msg.frames = clusters_msg.frames
        trace_msg.array_sizes = traceSeparation
        trace_msg.scan_time = scan_time
        trace_msg.id_array = idArray
        trace_publisher.publish(trace_msg)
        
    else:
        tracks_publisher.publish(clusters_msg)
        trace_publisher.publish(clusters_msg)

#just for local evaluation
def temp_visualize(x, y, z, clusterSizes, clusterSeparation, idArray, preC, curC):
    global count

    prev_index = 0
    colors = [(1, 0, 0), (0, 1, 1), (0, 0, 1), (0, 1, 0), (0.5, 0.5, 0.5), (1, 0.0784314, 0.576471), (0.133333, 0.545098, 0.133333), (1, 0.843137, 0), (0.580392, 0, 0.827451), (0.823529, 0.411765, 0.117647)]
    mm = ['2dcircle', '2dcross', '2ddiamond', '2dsquare', '2dthick_cross', '2dtriangle', '2dvertex', 'cube']
    colors2 = [(0.133333, 0.545098, 0.133333), (1, 0.843137, 0), (0.580392, 0, 0.827451)]

    if(len(clusterSeparation) > 0):
        myfig = mlab.figure(1, fgcolor=(0, 0, 0), bgcolor=(1, 1, 1), size=(900,900))
        mlab.clf(myfig)

        
        for i in range(0, len(clusterSeparation)):
            xk = []
            yk = []
            zk = []

            for j in range(prev_index, prev_index+clusterSeparation[i]-1):
                xk.append(x[j])
                yk.append(y[j])
                zk.append(z[j])
            prev_index = clusterSeparation[i] - 1

            ind = idArray[i]    #%len(colors)
            pts = mlab.points3d(xk, yk, zk, mode=mm[ind%len(mm)], color=colors[ind%len(colors)], scale_factor = 0.1, figure=myfig)

        #the previous and current association points        
        for i in range(0, len(preC)):
            mlab.points3d(preC[i][0], preC[i][1],preC[i][2], mode='cube', color=colors2[i%len(colors2)], scale_factor = 0.3, figure=myfig)

        for i in range(0, len(curC)):
            mlab.points3d(curC[i][0], curC[i][1], curC[i][2], mode='2dtriangle',  color=colors2[i%len(colors2)], scale_factor = 0.3, figure=myfig)
        
        mlab.show()



#First it finds the centroids of the last part(last frame in a timewindow) of the previous clusters.
#Then it connects a new cluster to a previous one (1-1), by taking into account the euclidean distance of each combination (new cluster - previous cluster).
#Two possible association approaches: 
#   a) greedy
#   b) hungarian/munkres
#If more clusters appeared in this frame :: check whether it fits to a track existed in the past (maximum <num_slide_window> timewindows ago)
#If less clusters appeared in this frame :: ignore it
def association_procedure(newClusters):
    global trackHandler
    global errorDist

    previousIds, previousClusters = trackHandler.previous()

    previousCentroids = [get_centroid(cl, True) for cl in previousClusters]

    currentCentroids = [get_centroid(cl, False) for cl in newClusters]

    costmap = compute_cost(currentCentroids, previousCentroids)

    #print 'COSTMAP = {}\n var = {}'.format(costmap, np.var(costmap))

    #if there is a small variance, it means that the association points are really close, thus use the hungarian approach to minimize the overall cost.
    #Otherwise, greedy is enough
    #variance threshold = 0.1 
    if np.var(costmap) < 0.1:
        associations = hungarian_approach(costmap)
    else:
        associations = greedy_approach(costmap)  
    #associations = hungarian_approach(costmap)

    #handle the issues where more or less clusters exist in the current frame
    if not (len(currentCentroids) == len(previousCentroids)):
        associations = get_associations(associations, len(currentCentroids), len(previousCentroids))

    #print 'associations = ',associations
    for item in associations:
        #new cluster that does not associates with a track of the previous timewindow
        if item[1] == -1:
            #check whether it fits with a track of some <X> previous timewindows
            candidateCls = trackHandler.previous_fit(newClusters[item[0]])
            if not (candidateCls is None):
                candidateCentroids = [get_centroid(cl.get_last_part(), True) for cl in candidateCls]
                distArray = [compute_distance(p1, currentCentroids[item[0]], distance='euclidean') for p1 in candidateCentroids]
                minDist = np.argmin(distArray)

                #not far away -> combine it with this candidate
                if distArray[minDist] < errorDist:
                    trackHandler.update_track(candidateCls[minDist].track_id, newClusters[item[0]])
                else:
                    trackHandler.add_track(newClusters[item[0]])
            else:
                #If not -> new cluster
                trackHandler.add_track(newClusters[item[0]])
        else:
            if not (item[0] == -1):
                trackId = previousIds[item[1]]
                trackHandler.update_track(trackId, newClusters[item[0]])

    return previousCentroids, currentCentroids

#take into account the z-dimension of the last trace from each previous cluster and compare them with each new cluster, 
#by adapting the z-dimension of the new cluster accordingly to the compareds trace each time
def association_procedure3D(newClusters):
    global trackHandler
    global errorDist

    cCent = []

    previousIds, previousClusters = trackHandler.previous()

    costmap, previousCentroids, currentCentroids = compute_cost3D(previousClusters, newClusters)

    associations = greedy_approach(costmap)  
    #associations = hungarian_approach(costmap)

    #handle the issues where more or less clusters exist in the current frame
    if not (len(costmap[0]) == len(costmap)):
        associations = get_associations(associations, len(costmap), len(costmap[0]))

    #print 'associations = ',associations
    for item in associations:
        #new cluster that does not associates with a track of the previous timewindow
        if item[1] == -1:
            #check whether it fits with a track of some <X> previous timewindows -> TODO
            
            trackHandler.add_track(newClusters[item[0]])
        else:
            if not (item[0] == -1):
                trackId = previousIds[item[1]]
                trackHandler.update_track(trackId, newClusters[item[0]])
                cCent.append(currentCentroids[item[0]][item[1]])

    return previousCentroids, cCent


#Get the median point (x,y,z) of a set of range points.
#   <choice> parameter declares the first (True) or the last (False) part of the cluster.
def get_centroid(cluster, choice):
    global z_scale

    if choice == False:
        z_filter = np.where(cluster[2]==cluster[2][len(cluster[2])-1])
        z_ = cluster[2][z_filter]
    else:
        z_filter = np.where(cluster[2]==cluster[2][0])
        z_ = cluster[2][z_filter] + z_scale

    x_ = []
    y_ = []

    for i in z_filter[0]:
        x_.append(cluster[0][i])
        y_.append(cluster[1][i])

    a = [x_,y_,z_]
    mm = np.median(a, axis=1)
    med = [mm[0], mm[1], mm[2]]

    return np.array(med)

#Cost map computation for each combination
def compute_cost(currentCentroids, previousCentroids):
    #the rows == new clusters and the cols == previous clusters
    costmap = [[compute_distance(i,j, distance='euclidean') for j in previousCentroids] for i in currentCentroids]

    return costmap


def compute_cost3D(currentClusters, previousClusters):
    global z_scale
    previousCentroids = []
    currentCentroids = []
    costmap = []

    for pCl in previousClusters:
        z_filter = np.where(pCl[2]==pCl[2][len(pCl[2])-1])
        p_a = np.array([pCl[0][z_filter], pCl[1][z_filter], pCl[2][z_filter]])

        previousCentroids.append(np.median(p_a, axis=1))

        temp_c = []
        temp_cost = []
        for cCl in currentClusters:
            z_filterCur = np.where(cCl[2]==cCl[2][0])
            z_ = cCl[2][z_filterCur] + pCl[2][z_filter][0]

            c_a = np.array([cCl[0][z_filterCur], cCl[1][z_filterCur], z_])

            temp_c.append(np.median(c_a, axis=1))

            cost = compute_distance(np.median(c_a, axis=1), np.median(p_a, axis=1), distance='euclidean')
            temp_cost.append(cost)

        costmap.append(temp_cost)
        currentCentroids.append(temp_c)

    return costmap, previousCentroids, currentCentroids

def compute_distance(point1, point2, distance='euclidean'):
    if distance == 'euclidean':
        return euclidean(point1, point2)
    elif distance == 'mahalanobis':
        return mahalanobis(point1, point2)
    else:
        return float("Inf")

#Euclidean Distance of 2 points
def euclidean(point1, point2):
    return np.linalg.norm(point1-point2)

def mahalanobis(point1, point2):
    P = np.vstack((p1,p2))
    cov = np.cov(P)
    
    return dist.mahalanobis(point1, point2, np.linalg.inv(cov))

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

#based on the costmap, apply the hungarian algorithm to determine the minimum cost for this assignement/association problem
def hungarian_approach(costmap):
    hungarian = hg.Hungarian(costmap)
    hungarian.calculate()
    associations = hungarian.get_results()

    return associations

#based on the costmap, apply a greedy algorithm (get the minimum cost-combination each time) for this assignement/association problem
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



if __name__ == '__main__':
    init() 
