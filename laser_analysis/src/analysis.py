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


z = 0
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)
LDA_classifier = None
results4meters_publisher = None
scan_parts = 5
timewindow = 40
distance = 4
publish_viz = False
viz_publisher = None

scan_time = 0.0
timestamp = 0.0

#list_of<WalkTrack>: 
#It contains information about the walk statistics (distance in meters, time for that distance) for each traced_cluster.
#Necessary for walk_speed()
walkTrack = []


def init():
    global results4meters_publisher, frame_id, LDA_classifier, publish_viz, viz_publisher
    global dt, speed_, z_scale
    global timewindow, distance

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

    print input_clusters_topic
    print 'tim = {} dist = {}'.format(timewindow, distance)

    z_scale = float(speed_*dt) / float(3600)

    rospack = rospkg.RosPack()

    classifier_path = rospack.get_path('laser_analysis')+'/classification_files/'+classifier_file
    LDA_classifier = pickle.load(open(classifier_path, "rb"))

    rospy.Subscriber(input_clusters_topic, ClustersMsg, analysis)

    results4meters_publisher = rospy.Publisher(results4meters_topic, Analysis4MetersMsg, queue_size=10)


    if publish_viz:
        print 'todo'
        #viz_publisher = rospy.Publisher(viz_topic, TODO, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()

#get the data from the laser scans and cluster them with DBSCAN
#align each cluster regarding the variances of each dimension
#gridfit each aligned cluster
#hogs on each image for feature extraction
#-----------------------------------------
#calls: speed(), overlap_trace()
def analysis(clusters_msg):

    global z, z_scale
    global all_clusters, all_hogs, all_orthogonal
    global frame_id, publish_viz
    global seconds,prev_sec
    global scan_time, timestamp
    global walkTrack

    all_clusters = []
    all_hogs = []
    all_orthogonal = []
    hogs=[]
    align_cl=[] #contains the aligned data clouds of each cluster

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
            walkTrack.append(wt.WalkTrack())

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

        
        for j in range(prev_index, prev_index+array_sizes[i]-1):
            xk.append(xi[j])
            yk.append(yi[j])
            zk.append(zi[j])
        prev_index = array_sizes[i] - 1
        


	'''
        if walkTrack[i].is_new():
            newCluster = True
        else:
            newCluster = False

        xCl = []
        yCl = []

        for j in range(prev_index, prev_index+array_sizes[i]-1):

            xCl.append(xi[j])
            yCl.append(yi[j])
           
            #it is a cluster (part of traced one)
            if len(xCl)-1 == num_clusters[cl_index]-1:
                if newCluster:
                    walk_speed(xCl, yCl, i)

                cl_index += 1
                tot_sum = tot_sum +len(xCl)
                xCl = []
                yCl = []


            xk.append(xi[j])
            yk.append(yi[j])
            zk.append(zi[j])
        prev_index = array_sizes[i] - 1

	# it takes only the last part-cluster
        walk_speed(xCl, yCl, i)
        tot_sum -= 1
        '''
        

        #speed(xk,yk,zk)
        trans_matrix =[[xk,yk,zk]]

        all_clusters.append([xk,yk,zk])

        #we get U by applying svd to the covariance matrix. U represents the rotation matrix of each cluster based on the variance of each dimension.
        U,s,V=np.linalg.svd(np.cov([xk,yk,zk]), full_matrices=False)

        #translate each cluster to the beginning of the axis and then do the rotation
        [xnew,ynew,znew]=translate_cluster(xk,yk,zk)

        #(traslation matrix) x (rotation matrix) = alignemt of cluster
        alignment_result=[[sum(a*b for a,b in zip(X_row,Y_col)) for X_row in zip(*[xnew,ynew,znew])] for Y_col in U]
        alignment_result=multiply_array(xnew,ynew,znew, V)

        #steps2(xk,yk,zk)

        align_cl.append(alignment_result)
        all_orthogonal.append(alignment_result)
        grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2], 16, 16) #extract surface - y,z,x alignment_result[1]

        grid=grid-np.amin(grid)

        features=hog(grid)
        f=hog(grid, orientations=6, pixels_per_cell=(8, 8), cells_per_block=(1, 1), visualise=False)
        all_hogs.append(f)
        hogs.append(f)  #extract hog features

        #update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids)
        temp = []

        if np.array(hogs).shape==(1,36):
            temp = np.array(hogs)[0]

        else:
            for k in range(0,len(hogs)):
                temp.append(np.array(hogs[k]))


        results = LDA_classifier.predict(temp)
        print results

        
        ###################################
        if results[i] == 1:
            if walkTrack[i].is_new():
                newCluster = True
            else:
                newCluster = False

            xCl = []
            yCl = []

            for j in range(prev_index_walk, prev_index_walk+array_sizes[i]-1):

                xCl.append(xi[j])
                yCl.append(yi[j])
           
                #it is a cluster (part of traced one)
                if len(xCl)-1 == num_clusters[cl_index]-1:
                    if newCluster:
                        walk_speed(xCl, yCl, i)

                    cl_index += 1
                    tot_sum = tot_sum +len(xCl)
                    xCl = []
                    yCl = []


            
            prev_index_walk = array_sizes[i] - 1

	    # it takes only the last part-cluster
            walk_speed(xCl, yCl, i)
            tot_sum -= 1
        #######################################
        

        [xc,yc,zc] = [align_cl[cnt][0], align_cl[cnt][1], align_cl[cnt][2]]


        if len(xc)==0:
            print 'out of data'
            continue

        cnt=cnt+1

        hogs_temp = np.array(np.array(temp))

        analysis4meters_msg = Analysis4MetersMsg()
        analysis4meters_msg.header.stamp = rospy.Time.now()
        analysis4meters_msg.header.frame_id = frame_id
        #TODO add speed here

        if publish_viz:
            #TODO publish required arrays
            print 'test'


#It calculates the time (in seconds) where a human needs to cover <distance> meters.
#Description of algorithm:
#    It splits the data in <parts> parts.
#    It takes the median (x,y)-points of each part and it calculates their distance and the respective time.
#Arguments:
#    x: data in x-dimension
#    y: data in y-dimension
#    pos: the position in walkTack array, which denotes the human-id
#Recommendations: 
#    - x,y should be points of a cluster (and not a traced_cluster)
#    - usual number of points ~= 400-600
def walk_speed(x, y, pos):

    global timewindow, scan_time, distance, timestamp
    global walkTrack

    parts = 4
    split = len(x)/parts
    split_count = 0

    human = walkTrack[pos]
    
    #the incrementation of time
    #    -> it is related to the time between scans
    time_increment = scan_time*((timewindow-2)/parts) 

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
        else:
            human.set_timestamp(timestamp)
	
        human.set_prevMedian(xmed, ymed)
        human.set_time(time_increment)

        human.addX(xmed)
        human.addY(ymed)

        split_count += split

        if human.get_distance() >= distance:
            print '\n*****\nHuman {}: He/She walked {} meters in {} seconds\n*****\n'.format(pos, human.get_distance(), human.get_time())
            write_results(pos)
            human.initialise()


def write_results(pos):

    global walkTrack,timestamp

    human = walkTrack[pos]

    ar = [[pos, human.get_timestamp(), timestamp, human.get_distance(), human.get_time()]]
    with open('/home/rosturtle/Desktop/testFile.csv','a') as csvfile:
        wr = csv.writer(csvfile, dialect='excel')
        for row in ar:
            #print 'row = ',row
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
