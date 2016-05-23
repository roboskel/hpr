#!/usr/bin/env python
import roslib, rospy
import numpy as np
from gridfit import gridfit
import skimage #skimage library has been slightly customized (_hog.py)
import pickle
from skimage.feature import hog
from laser_clustering.msg import ClustersMsg
import rospkg

z = 0
dt = 25;#period in ms (dt between scans)
speed = 5;#human walking speed in km/h
z_scale = float(speed*dt) / float(3600)
LDA_classifier = None

def init():
    global clusters_publisher, frame_id, LDA_classifier

    input_clusters_topic = rospy.get_param('~input_clusters_topic','laser_clustering/clusters')
    output_clusters_topic = rospy.get_param('~output_clusters_topic','~clusters')
    frame_id = rospy.get_param('~frame_id','laser_link')

    rospy.init_node('laser_analysis')

    rospack = rospkg.RosPack()

    classifier_path = rospack.get_path('laser_analysis')+'/classification_files/LDA_classifier.p'
    LDA_classifier = pickle.load(open(classifier_path, "rb"))

    rospy.Subscriber(input_clusters_topic, ClustersMsg, analysis)

    clusters_publisher = rospy.Publisher(output_clusters_topic, ClustersMsg, queue_size=10)
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
    global all_clusters, all_hogs, all_gridfit, all_orthogonal
    global tot_results, all_annotations

    all_clusters = []
    all_hogs = []
    all_gridfit = []
    all_orthogonal = []
    hogs=[]
    align_cl=[] #contains the aligned data clouds of each cluster
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at least one valid cluster
    grids=[]
    cls = []

    xi = np.array(clusters_msg.x)
    yi = np.array(clusters_msg.y)
    zi = np.array(clusters_msg.z)

    cluster_labels = np.array(clusters_msg.clusters)

    max_label=int(np.amax(cluster_labels))

    #for every created cluster - its data points
    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)

        if len(filter[0])>40 :

            valid_flag=1

            #points of every cluster at each timewindow-frame
            [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]

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

            cls.append([xk,yk,zk])

            align_cl.append(alignment_result)
            all_orthogonal.append(alignment_result)

            vcl.append(k)
            grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2], 16, 16) #extract surface - y,z,x alignment_result[1]
            all_gridfit.append(grid)

            grid=grid-np.amin(grid)
            grids.append(grid)

            features=hog(grid)
            f=hog(grid, orientations=6, pixels_per_cell=(8, 8), cells_per_block=(1, 1), visualise=False)
            all_hogs.append(f)
            hogs.append(f)  #extract hog features

        update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids)


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


def update_plots(flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids):

    global wall_cart, LDA_classifier, hogs_temp, align_plot, top_view_figure
    global first_time_annotations, all_annotations
    global tot_results, metrics, first_time_results

    temp = []
    store_results = []
    centerx = []
    centery = []
    centerz = []


    if flag==1:
        if np.array(hogs).shape==(1,36):
            temp = np.array(hogs)[0]

        else:
            for i in range(0,len(hogs)):
                temp.append(np.array(hogs[i]))


        results = LDA_classifier.predict(temp)
        print results
        cnt=0
        col_list=[]


        for k in vcl:

            filter=np.where(cluster_labels==k)

            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            [xc,yc,zc] = [align_cl[cnt][0], align_cl[cnt][1], align_cl[cnt][2]]


            if len(xc)==0:
                print 'out of data'
                continue

            cnt=cnt+1

    hogs_temp = np.array(np.array(temp))


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