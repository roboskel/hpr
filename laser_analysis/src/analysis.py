#!/usr/bin/env python
import roslib, rospy
import numpy as np
from gridfit import gridfit
import skimage #skimage library has been slightly customized (_hog.py)
import pickle
from skimage.feature import hog
from laser_clustering.msg import ClustersMsg
from laser_analysis.msg import Analysis4MetersMsg
import rospkg, math
import walk_track as wt 
import csv
from sklearn.decomposition import PCA

z = 0
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)
Classifier = None
results4meters_publisher = None
scan_parts = 5
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

#list_of<WalkTrack>: 
#It contains information about the walk statistics (distance in meters, time for that distance) for each traced_cluster.
#Necessary for walk_analysis() calculation
walkTrack = []
hum_id = 0


def init():
    global results4meters_publisher, frame_id, Classifier, publish_viz, viz_publisher
    global dt, speed_, z_scale
    global timewindow, distance
    global stat_file, writeToFile
    global pca_obj

    rospy.init_node('laser_analysis')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    results4meters_topic = rospy.get_param('~results4meters_topic','~results4meters')
    frame_id = rospy.get_param('~frame_id','laser_link')
    classifier_file = rospy.get_param('~classifier_file','LDA_classifier.p')
    publish_viz = rospy.get_param('~publish_viz', False)
    viz_topic = rospy.get_param('~viz_topic', "~viz_req")
    dt = rospy.get_param('~dt', 25)
    speed_ = rospy.get_param('~human_speed', 5)
    timewindow = rospy.get_param('~timewindow', 40)
    distance = rospy.get_param('~distance', 4)
    writeToFile = rospy.get_param('~write_to_file', False)
    stat_file = rospy.get_param('~file', '/home/hprStats.csv')
    pca_file = rospy.get_param('~pca_file','/home/myPCA.p')


    print input_clusters_topic

    z_scale = float(speed_*dt) / float(3600)


    rospack = rospkg.RosPack()

    classifier_path = rospack.get_path('laser_analysis')+'/classification_files/'+classifier_file
    Classifier = pickle.load(open(classifier_path, "rb"))

    pca_path = rospack.get_path('laser_analysis')+'/classification_files/'+pca_file
    pca_obj = pickle.load(open ( pca_path, "rb"))
    


    rospy.Subscriber(input_clusters_topic, ClustersMsg, analysis)
    #rospy.Subscriber(input_clusters_topic, ClustersMsg, cluster_analysis)

    results4meters_publisher = rospy.Publisher(results4meters_topic, Analysis4MetersMsg, queue_size=10)


    if publish_viz:
        print 'todo'
        #viz_publisher = rospy.Publisher(viz_topic, TODO, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()


#It makes the walk analysis for each traced_cluster that receives during the period of time.
#Receives the data from the traced_clusters.
#If a traced cluster represents a human walk, the method calls the walk_analysis for each cluster that is just received.
#For two not human walks in a row, it initialises the respective walkTrack.
#-----------------------------------------
#calls: human_predict(), walk_analysis()
def analysis(clusters_msg):

    global z, z_scale, dt
    global frame_id, publish_viz
    global seconds,prev_sec
    global scan_time, timestamp
    global walkTrack, hum_id

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)
    if len(xi) != 0:
        scan_time = clusters_msg.scan_time

        #ratio between the time that the laser scans to the time between them. zscale depends on dt value.
        dt_ratio = round(float(scan_time * 1000) / dt, 2) 
        print 'dt_ration = {}, scan_time = {}'.format(dt_ratio, scan_time)
        timestamp = clusters_msg.header.stamp
    else:
        del walkTrack[:]

    array_sizes = np.array(clusters_msg.array_sizes)
    num_clusters = np.array(clusters_msg.num_clusters)
    
    cnt=0
    prev_index = 0
    prev_index_walk = 0

    #add new walk tracks
    if len(array_sizes) > len(walkTrack):
        for i in range(len(walkTrack), len(array_sizes)):
            walkTrack.append(wt.WalkTrack(hum_id))
            hum_id += 1

    #remove walk tracks
    #you can identify them by the zeros in the num_clusters array
    elif len(array_sizes) < len(walkTrack):
        sumV = 0
        k = 0
        for j in range(0, len(num_clusters)):
            if num_clusters[j] == 0:
                if sumV == 0:
                    if j == 0:
                        #the first part of traced cluster is removed
                        del walkTrack[0]
                    else:
                        #case where the removed clusters were in a row
                        del walkTrack[k]

                else:
                    #the last cluster disappeared
                    if j == len(num_clusters)-1:
                        del walkTrack[len(walkTrack)-1]
                    else:
                        pos = np.where(array_sizes == sumV)[0]
                        del walkTrack[pos+ (k+1)]
                        sumV = 0

            if sumV == array_sizes[k]:
                k = k + 1
                sumV = 0
		    
            sumV += num_clusters[j]

        num_clusters = num_clusters[num_clusters != 0]


    tot_sum = 0
    cl_index = 0


    for i in range(0, len(array_sizes)):
        xk = []
        yk = []
        zk = []

        #(xk, yk, zk): 3D data points for each traced_cluster
        for j in range(prev_index, prev_index+array_sizes[i]-1):
            xk.append(xi[j])
            yk.append(yi[j])
            zk.append(zi[j])
        prev_index = array_sizes[i] - 1
       
        
        if walkTrack[i].is_new():
            newCluster = True
        else:
            newCluster = False

        #(xCl,yCl): data points for each cluster in a traced_cluster
        xCl = []
        yCl = []

        for j in range(prev_index_walk, prev_index_walk+array_sizes[i]-1):

            xCl.append(xi[j])
            yCl.append(yi[j])

            #it is a cluster (part of traced one)
            if len(xCl)-1 == num_clusters[cl_index]-1:
                if newCluster:
                    #call walk_analysis only if it is a human walk. 
                    #Otherwise, check whether it was predicted that it was motionless in the previous cluster too. If yes then initialise its walk track.
                    if human_predict(xk,yk,zk) == 1:
                        walk_analysis(xCl, yCl, i)
                        walkTrack[i].set_stable(False)
                    else:
                        if walkTrack[i].is_stable():
                            walkTrack[i].initialise()
                        else:
                            walk_analysis(xCl, yCl, i)

                        walkTrack[i].set_stable(True)
                cl_index += 1

                if cl_index == len(num_clusters):
                    cl_index = 0
                    break
                tot_sum = tot_sum +len(xCl)
                xCl = []
                yCl = []


            
        prev_index_walk = array_sizes[i] - 1

	# it takes only the last part-cluster bc the previous clusters where computed before (slice-window mode).
        if human_predict(xk,yk,zk) == 1:
            walk_analysis(xCl, yCl, i)
        else:
            if walkTrack[i].is_stable():
                walkTrack[i].initialise()
            else:
                walk_analysis(xCl, yCl, i)

            walkTrack[i].set_stable(True)

        tot_sum -= 1
   

        if publish_viz:
            #TODO publish required arrays
            print 'test'


def cluster_analysis(clusters_msg):

    global z, z_scale
    global frame_id, publish_viz
    global seconds,prev_sec
    global scan_time, timestamp
    global walkTrack, hum_id

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)
    
    if len(xi) != 0:
        scan_time = clusters_msg.scan_time
        timestamp = clusters_msg.header.stamp
    else:
        del walkTrack[:]

    array_sizes = np.array(clusters_msg.array_sizes)
    num_clusters = np.array(clusters_msg.num_clusters)
    
    cnt=0
    prev_index = 0
    prev_index_walk = 0

    #add new walk tracks
    if len(array_sizes) > len(walkTrack):
        for i in range(len(walkTrack), len(array_sizes)):
            walkTrack.append(wt.WalkTrack(hum_id))
            hum_id += 1

    #remove walk tracks
    #you can identify them by the zeros in the num_clusters array
    elif len(array_sizes) < len(walkTrack):
        sumV = 0
        k = 0
        for j in range(0, len(num_clusters)):
            if num_clusters[j] == 0:
                if sumV == 0:
                    if j == 0:
                        #the first part of traced cluster is removed
                        del walkTrack[0]
                    else:
                        #case where the removed clusters were in a row
                        del walkTrack[k]

                else:
                    #the last cluster disappeared
                    if j == len(num_clusters)-1:
                        del walkTrack[len(walkTrack)-1]
                    else:
                        pos = np.where(array_sizes == sumV)[0]
                        del walkTrack[pos+ (k+1)]
                        sumV = 0

            if sumV == array_sizes[k]:
                k = k + 1
                sumV = 0
		    
            sumV += num_clusters[j]

        num_clusters = num_clusters[num_clusters != 0]


    tot_sum = 0
    cl_index = 0

    for i in range(0, len(array_sizes)):
        xk = []
        yk = []
        zk = []

        #(xk, yk, zk): 3D data points for each traced_cluster
        for j in range(prev_index, prev_index+array_sizes[i]-1):
            xk.append(xi[j])
            yk.append(yi[j])
            zk.append(zi[j])
        prev_index = array_sizes[i] - 1
       
        
        if walkTrack[i].is_new():
            newCluster = True
        else:
            newCluster = False

        #(xCl,yCl,zCl): data points for each cluster in a traced_cluster
        xCl = []
        yCl = []
        zCl = []

        for j in range(prev_index_walk, prev_index_walk+array_sizes[i]-1):

            xCl.append(xi[j])
            yCl.append(yi[j])
            zCl.append(zi[j])
           
            #it is a cluster (part of traced one)
            if len(xCl)-1 == num_clusters[cl_index]-1:
                if newCluster:
                    #call walk_analysis only if it is a human walk. 
                    #Otherwise, check whether it was predicted that it was motionless in the previous cluster too. If yes then initialise its walk track.
                    if human_predict(xCl,yCl,zCl) == 1:
                        walk_analysis(xCl, yCl, i)
                        walkTrack[i].set_stable(False)
                    else:
                        if walkTrack[i].is_stable():
                            walkTrack[i].initialise()
                        else:
                            walk_analysis(xCl, yCl, i)

                        walkTrack[i].set_stable(True)
                cl_index += 1
                tot_sum = tot_sum +len(xCl)
                xCl = []
                yCl = []
                zCl = []

            
        prev_index_walk = array_sizes[i] - 1

	# it takes only the last part-cluster bc the previous clusters where computed before (slice-window mode).
        
        if human_predict(xCl,yCl,zCl) == 1:
            walk_analysis(xCl, yCl, i)
        else:
            if walkTrack[i].is_stable():
                walkTrack[i].initialise()
            else:
                walk_analysis(xCl, yCl, i)

            walkTrack[i].set_stable(True)
        
        tot_sum -= 1
   

        if publish_viz:
            #TODO publish required arrays
            print 'test'


#Recognition whether a traced_cluster is following a pattern of a human walk or not.
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

    grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2], 16, 16) #extract surface - y,z,x alignment_result[1]

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


    #temp_pca = pca_obj.transform(temp)
    results = Classifier.predict(temp)
    print 'predicted result = ',results

    return results[0]


#It calculates the time (in seconds) where a human needs to cover <distance> meters.
#Description of algorithm:
#    It splits the data in <cluster_parts> parts.
#    It takes the median (x,y)-points of each part and it calculates their distance and the respective time.
#Arguments:
#    x: data in x-dimension
#    y: data in y-dimension
#    pos: the position in walkTack array, which denotes the human-id
#Recommendations: 
#    - x,y should be points of a cluster (and not a traced_cluster)
#    - usual number of points ~= 400-600
def walk_analysis(x, y, pos):

    global timewindow, scan_time, distance, timestamp, cluster_parts
    global walkTrack, writeToFile, frame_id, results4meters_publisher
    global dt_ratio

    split = len(x)/cluster_parts
    split_count = 0

    human = walkTrack[pos]
    
    #the incrementation of time
    #    -> it is related to the time between scans
    time_increment = (float(scan_time)/dt_ratio)*((timewindow-2)/cluster_parts)    

    if split == 0:
        return

    while split_count <= len(x):
        xmed = np.median(x[split_count:split_count+split])
        ymed = np.median(y[split_count:split_count+split])

        if ((math.isnan(xmed)) or (math.isnan(ymed))):
            split_count += split
            continue

        if not human.empty():
            human.add_distance(xmed, ymed)
            #print 'added dist = ',human.get_distance()
            
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

        if human.get_distance() >= distance:
            print '\n*****\nHuman {} walked {} meters in {} seconds\n*****\n'.format(human.get_id(), human.get_distance(), human.get_time())
	    analysis4meters_msg = Analysis4MetersMsg()
            analysis4meters_msg.header.stamp = rospy.Time.now()
            analysis4meters_msg.header.frame_id = frame_id
	    analysis4meters_msg.human_id = human.get_id()
	    analysis4meters_msg.time_needed = human.get_time()
	    analysis4meters_msg.distance = human.get_distance()
	    results4meters_publisher.publish(analysis4meters_msg)
            if writeToFile:
                write_results(pos)
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
        


#calculates the speed of the first 25 frames (for each cluster), i.e the m/sec that the human walk for each scan.
#it gets the median point (x,y) for every frame - set of points
#the total distance is the sum of the euclidean distance of a point to its previous one
def speed(x, y, z) :

    global z_scale, scan_parts, tot_speed

    z_angle = z[0]
    dist = 0.0
    scan_speed1 = 0.0
    mean_array = []
    count = 0
    xk = []
    yk = []


    while z_angle <= z[len(z)-1]:
        z_filter = np.where(z==z_angle) #get the positions that have the same z_angle

        if count < scan_parts:

            for i in range(0, len(x[z_filter])):
                xk.append(x[z_filter][i])
                yk.append(y[z_filter][i])
        else:

            if len(xk) !=0 and len(yk) !=0:

                mean_array.append([np.median(xk), np.median(yk)])
                del xk[:]
                del yk[:]

                for i in range(0, len(x[z_filter])):
                    xk.append(x[z_filter][i])
                    yk.append(y[z_filter][i])
                count = 0

        count = count+1
        z_angle = z_angle + z_scale


    if len(xk) !=0 and len(yk) !=0:
        mean_array.append([np.median(xk), np.median(yk)])

    d = 0.0
    for i in range(0,len(mean_array)-1):
        d = d + math.sqrt(pow(mean_array[i+1][0] - mean_array[i][0], 2) + pow(mean_array[i+1][1] - mean_array[i][1], 2))

    #compute the speed at each scan -> m/sec
    scan_speed = d/(z_angle - z[0])

    tot_speed.append(scan_speed)


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
