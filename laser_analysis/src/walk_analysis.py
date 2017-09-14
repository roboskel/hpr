#!/usr/bin/env python
import roslib, rospy
import numpy as np
from gridfit import gridfit
import skimage #skimage library has been slightly customized (_hog.py)
import pickle
from skimage.feature import hog
from laser_clustering.msg import ClustersMsg
from laser_analysis.msg import Analysis4MetersMsg
from laser_analysis.msg import HumanPredictionMsg
import rospkg, math
import walk_track as wt 
import csv
from sklearn.decomposition import PCA

Classifier = None
results4meters_publisher = None
results4scans_publisher = None
cluster_parts = 4
timewindow = 40
distance = 4
publish_viz = False
viz_publisher = None
stat_file = '/home/hprStats.csv'
writeToFile = False

scan_time = 0.0
timestamp = 0.0
dt_ratio = 1.0
pca_obj = PCA()
min_distance = 1

#list_of<WalkTrack>: 
#It contains information about the walk statistics (distance in meters, time for that distance) for each traced_cluster.
#Necessary for walk_estimation() calculation
walkAnalyser = []

frames_array = []   #contains frames' ids (used at back2scans function)


def init():
    global results4meters_publisher, results4scans_publisher, frame_id, Classifier, publish_viz, viz_publisher
    global timewindow, distance, cluster_parts
    global stat_file, writeToFile
    global pca_obj

    rospy.init_node('laser_analysis')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    results4meters_topic = rospy.get_param('~results4meters_topic','~results4meters')
    results4scans_topic = rospy.get_param('~results4scans_topic','~results4scans')
    frame_id = rospy.get_param('~frame_id','laser_link')
    classifier_file = rospy.get_param('~classifier_file','LDA_classifier.p')
    publish_viz = rospy.get_param('~publish_viz', False)
    viz_topic = rospy.get_param('~viz_topic', "~viz_req")
    cluster_parts = rospy.get_param('~cluster_parts', 4)
    timewindow = rospy.get_param('~timewindow', 40)
    distance = rospy.get_param('~distance', 4)
    writeToFile = rospy.get_param('~write_to_file', False)
    stat_file = rospy.get_param('~file', '/home/hprStats.csv')
    pca_file = rospy.get_param('~pca_file','/home/myPCA.p')

    rospack = rospkg.RosPack()

    classifier_path = rospack.get_path('laser_analysis')+'/classification_files/'+classifier_file
    Classifier = pickle.load(open(classifier_path, "rb"))

    pca_path = rospack.get_path('laser_analysis')+'/classification_files/'+pca_file
    pca_obj = pickle.load(open ( pca_path, "rb"))
    
    rospy.Subscriber(input_clusters_topic, ClustersMsg, trace_analysis)

    results4meters_publisher = rospy.Publisher(results4meters_topic, Analysis4MetersMsg, queue_size=10)

    results4scans_publisher = rospy.Publisher(results4scans_topic, HumanPredictionMsg, queue_size=10)


    if publish_viz:
        print 'todo'
        #viz_publisher = rospy.Publisher(viz_topic, TODO, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()


#Procedure of handling the new (coming) traces
    #Algorithm - Approach ::
    #------------------------------------------------|
    #If the trace (specified by the trace_id) exists:|
    #   If it is a human walk, then:                 |
    #       analyse the walk (speed, distance, time) |
    #       update the stable condition              |
    #   Else (If it is not a human walk):            |
    #       Check if it is stable and initialize     |
    #Else (this walktrack doesnt exist):             |
    #   If it is human walk, then:                   |
    #       insert it as new                         |
    #       analyse the walk (speed, distance, time) |
    #       update the stable condition              |
    #   Else:                                        |
    #       skip                                     |
    #------------------------------------------------|
def trace_analysis(traces_msg):
    global frame_id, publish_viz
    global seconds,prev_sec
    global scan_time, timestamp
    global walkAnalyser, hum_id
    global frames_array

    x_ = np.array(traces_msg.x)
    y_ = np.array(traces_msg.y)
    z_ = np.array(traces_msg.z)
    
    if len(x_) != 0:
        scan_time = traces_msg.scan_time
        timestamp = traces_msg.header.stamp
    else:
        del walkAnalyser[:] #should we remove everything or keep them for some scans ???

    array_sizes = np.array(traces_msg.array_sizes)
    idArray = np.array(traces_msg.id_array)
    frames_array = traces_msg.frames

    prev_index = 0

    #there are some traces into it
    for i, trace_id in enumerate(idArray):
        last_cluster_index = (array_sizes[i] + prev_index - 1)
        xk = x_[prev_index:last_cluster_index]
        yk = y_[prev_index:last_cluster_index]
        zk = z_[prev_index:last_cluster_index]

        walkTrace = next((w for w in walkAnalyser if w.hum_id == trace_id), None)
        if walkTrace is None:
            if human_predict(xk,yk,zk) == 1:
                newWalkTrace = wt.WalkTrack(trace_id)
                walkAnalyser.append(newWalkTrace)
                walk_estimation(xk, yk, newWalkTrace)
                back2scans(xk, yk, zk, trace_id, True)

                print 'WalkTrack {} is walking '.format(trace_id)
            else:
                back2scans(xk, yk, zk, trace_id, False)
                print 'WalkTrack {} is standing still '.format(trace_id)
            
        else:
            if human_predict(xk,yk,zk) == 1:
                walk_estimation(xk, yk, walkTrace)
                walkTrace.set_stable(False)

                back2scans(xk, yk, zk, trace_id, True)
                print 'WalkTrack {} is walking '.format(trace_id)
            else:
                if walkTrace.is_stable():
                    walkTrace.initialise()
                else:
                    walk_estimation(xk, yk, walkTrace)

                walkTrace.set_stable(True)
                back2scans(xk, yk, zk, trace_id, False)
                print 'WalkTrack {} is standing still '.format(trace_id)

        prev_index = prev_index + array_sizes[i]
    

#Publish the recognition decision, and the mean point for each scan 
def back2scans(x, y, z, obj_id, is_human):
    global results4scans_publisher, frames_array

    z_angle = z[0]

    while z_angle <= z[len(z)-1]:
        z_filter = np.where(z==z_angle) #get the positions that have the same z_angle

        xk = []
        yk = []
        zk = []
        for i, ind in enumerate(z_filter[0]):
            xk.append(x[ind])
            yk.append(y[ind])
            zk.append(z[ind])

        hpr_msg = HumanPredictionMsg()
        hpr_msg.header.stamp = rospy.Time.now()
        hpr_msg.header.frame_id = frame_id
        hpr_msg.x = np.mean(xk)      #mean x
        hpr_msg.y = np.mean(yk)      #mean y -> mean point (mean_x,mean_y)
        hpr_msg.z = zk[0]
        hpr_msg.frame_seq = frames_array[0]
        hpr_msg.is_human = is_human
        hpr_msg.object_id = obj_id

        results4scans_publisher.publish(hpr_msg)

        #get the next z value and terminate if there is no other
        ind = z_filter[0][len(z_filter[0]) - 1] + 1
        if ind <= (len(z) -1):
            z_angle = z[ind]
        else:
            break

#Recognise whether a trace (cluster) is following a pattern of a human walk or not.
#Steps:
#    - align each traced_cluster regarding the variances of each dimension
#    - gridfit each aligned cluster -> becomes an image
#    - hogs on each image for feature extraction
#    - the prediction is achieved with the use of LDA or SVM or NB+PCA trained classifier
#Arguments:
#    - (x,y,z): the 3D data points of a traced_cluster
def human_predict(x, y, z):

    global Classifier, pca_obj

    hogs=[]

    trans_matrix =[[x,y,z]]

    #we get U by applying svd to the covariance matrix. U represents the rotation matrix of each cluster based on the variance of each dimension.
    U,s,V=np.linalg.svd(np.cov([x,y,z]), full_matrices=False)

    #translate each cluster to the beginning of the axis and then do the rotation
    [xnew,ynew,znew]=translate_cluster(x,y,z)

    #(traslation matrix) x (rotation matrix) = alignemt of cluster
    alignment_result=[[sum(a*b for a,b in zip(X_row,Y_col)) for X_row in zip(*[xnew,ynew,znew])] for Y_col in U]
    alignment_result=multiply_array(xnew,ynew,znew, V)

    #grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2], 16, 16) #extract surface - x,y,z alignment_result[1]
    grid=gridfit(alignment_result[1], alignment_result[2], alignment_result[0], 16, 16) #extract surface - y,z,x alignment_result[1]

    grid=grid-np.amin(grid)

    features=hog(grid)
    f=hog(grid, orientations=6, pixels_per_cell=(8, 8), cells_per_block=(1, 1), visualise=False)
    hogs.append(f)  #extract hog features

    temp = []

    if np.array(hogs).shape==(1,36):
        temp = np.array(hogs)[0]

    else:
        for k in range(0,len(hogs)):
            temp.append(np.array(hogs[k]))


    temp_pca = pca_obj.transform(temp)
    results = Classifier.predict(temp_pca)
    #print 'predicted result = ',results

    return results[0]


#It calculates the time (in seconds) where a human needs to cover <distance> meters.
#Description of algorithm:
#    It splits the data in <cluster_parts> parts.
#    It takes the median (x,y)-points of each part and it calculates their distance and the respective time.
#Arguments:
#    x: data in x-dimension
#    y: data in y-dimension
#    human: the <WalkTrack>, which represents a human
#Recommendations: 
#    - x,y should be points of a cluster (and not a traced_cluster)
#    - usual number of points ~= 400-600
def walk_estimation(x, y, human):

    global timewindow, scan_time, distance, timestamp, cluster_parts
    global walkAnalyser, writeToFile, frame_id, results4meters_publisher
    global dt_ratio
    global min_distance

    split = len(x)/cluster_parts
    split_count = 0

    
    #the incrementation of time
    #    -> it is related to the time between scans
    time_increment = (float(scan_time)/dt_ratio)*((timewindow-2)/cluster_parts)    

    if split == 0:
        return

    while split_count < len(x):
        xmed = np.median(x[split_count:split_count+split])
        ymed = np.median(y[split_count:split_count+split])

        if ((math.isnan(xmed)) or (math.isnan(ymed))):
            split_count += split
            continue

        if not human.empty():
            human.add_distance(xmed, ymed)
            
            if human.compute_error(xmed, ymed) == True:
                print '-----\nHuman {} stops walking. Already distance {} in {} seconds\n-----'.format(human.get_id(), human.get_distance(), human.get_time())
                human.initialise()
                human.set_timestamp(timestamp)
            

        else:
            human.set_timestamp(timestamp)
    
        human.set_prevMedian(xmed, ymed)
        human.set_time(time_increment)

        human.addX(xmed)
        human.addY(ymed)

        split_count += split

        if human.get_distance() >= distance or human.get_distance() > min_distance:
            if human.get_distance() >= (distance-min_distance):
                print '\n*****\nHuman {} walked {} meters in {} seconds\n*****\n'.format(human.get_id(), human.get_distance(), human.get_time())

            analysis4meters_msg = Analysis4MetersMsg()
            analysis4meters_msg.header.stamp = rospy.Time.now()
            analysis4meters_msg.header.frame_id = frame_id
            analysis4meters_msg.human_id = human.get_id()
            analysis4meters_msg.time_needed = human.get_time() * distance / human.get_distance()
            analysis4meters_msg.distance = human.get_distance()
            results4meters_publisher.publish(analysis4meters_msg)
            if writeToFile:
                write_results(pos)
            if human.get_distance() >= distance:
                human.initialise()


def write_results(pos):

    global walkTrack,timestamp
    global stat_file

    human = walkTrack[pos]

    ar = [[human.get_id(), human.get_timestamp(), timestamp, human.get_distance(), human.get_time()]]
    with open(stat_file,'a') as csvfile:
        wr = csv.writer(csvfile, dialect='excel')
        for row in ar:
            wr.writerow(row)
        

def translate_cluster(x, y, z) :

    xmean=np.mean(x)
    ymean=np.mean(y)
    zmean=np.mean(z)

    new_x=[]
    new_y=[]
    new_z=[]

    for i in range(0, len(x)):
        new_x.append(x[i]-xmean)
        new_y.append(y[i]-ymean)
        new_z.append(z[i]-zmean)
    return [new_x,new_y,new_z]


def multiply_array(x,y,z, V) :

    new_x=[]
    new_y=[]
    new_z=[]

    for i in range(0, len(x)):
        new_x.append(x[i]*V[0][0] + y[i]*V[0][1] + z[i]*V[0][2])
        new_y.append(x[i]*V[1][0] + y[i]*V[1][1] + z[i]*V[1][2])
        new_z.append(x[i]*V[2][0] + y[i]*V[2][1] + z[i]*V[2][2])

    return [new_x,new_y,new_z]



if __name__ == '__main__':
    init() 
