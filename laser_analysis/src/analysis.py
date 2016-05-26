#!/usr/bin/env python
import roslib, rospy
import numpy as np
from gridfit import gridfit
import skimage #skimage library has been slightly customized (_hog.py)
import pickle
from skimage.feature import hog
from laser_clustering.msg import ClustersMsg
from laser_analysis.msg import Analysis4MetersMsg
import rospkg

z = 0
dt = 25;#period in ms (dt between scans)
speed_ = 5;#human walking speed in km/h
z_scale = float(speed_*dt) / float(3600)
LDA_classifier = None
results4meters_publisher = None
scan_parts = 5
publish_viz = False
viz_publisher = None

def init():
    global results4meters_publisher, frame_id, LDA_classifier, publish_viz, viz_publisher
    global dt, speed_, z_scale

    rospy.init_node('laser_analysis')

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    results4meters_topic = rospy.get_param('~results4meters_topic','~results4meters')
    frame_id = rospy.get_param('~frame_id','laser_link')
    classifier_file = rospy.get_param('~classifier_file','LDA_classifier.p')
    publish_viz = rospy.get_param('~publish_viz', False)
    viz_topic = rospy.get_param('~viz_topic', "~viz_req")
    dt = rospy.get_param('~dt', 25)
    speed_ = rospy.get_param('~human_speed', 5)

    print input_clusters_topic

    z_scale = float(speed_*dt) / float(3600)

    rospack = rospkg.RosPack()

    classifier_path = rospack.get_path('laser_analysis')+'/classification_files/'+classifier_file
    LDA_classifier = pickle.load(open(classifier_path, "rb"))

    rospy.Subscriber(input_clusters_topic, ClustersMsg, analysis)

    results4meters_publisher = rospy.Publisher(results4meters_topic, Analysis4MetersMsg, queue_size=10)

    if publish_viz:
        viz_publisher = rospy.Publisher(viz_topic, TODO, queue_size=10)

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

    all_clusters = []
    all_hogs = []
    all_orthogonal = []
    hogs=[]
    align_cl=[] #contains the aligned data clouds of each cluster

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    array_sizes = np.array(clusters_msg.array_sizes)
    
    cnt=0
    prev_index = 0

    for i in range(0, len(array_sizes)):
        xk = []
        yk = []
        zk = []
        for j in range(prev_index, prev_index+array_sizes[i]-1):
            xk.append(xi[j])
            yk.append(yi[j])
            zk.append(zi[j])
        prev_index = array_sizes[i] - 1

        print [xk,yk,zk]

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