#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib, warnings, rospy, math, pickle, scipy.stats
from gridfit import gridfit
import numpy as np
import scipy.spatial.distance as dist
import scipy.io as sio
import scipy.special
import matplotlib.pyplot as plt
import mytools as mt #DBSCAN function and perquisites are stored here
import sys
import os.path
import time
from os import listdir
from os.path import isfile, join, splitext
#from myhog import hog
from skimage.feature import hog
from sensor_msgs.msg import LaserScan
from scipy.stats.mstats import zscore
from scipy import interpolate
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull

ccnames =['gray', 'black', 'violet', 'blue', 'cyan', 'rosy', 'orange', 'red', 'green', 'brown', 'yellow', 'gold']
cc  =  ['#808080',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','r','g','#8B4513','y','#FFD700']
wall_flag=0
fr_index=1
z=0
dt = 25;#period in ms (dt between scans)
speed = 5;#human walking speed in km/h
z_scale= float(speed*dt) / float(3600)
w_index=1
limit=3
scan_active = True
classification_array = []
scan_received = 0
plt.ion()
class_path = ''
pca_path = ''
pca_obj = PCA()
annotation_file = ''
first_time = True
first_time_ranges = True
f_time=True
sub_topic = 'scan'
#metrics=0
total_cluster_time = 0
hogs_temp=[]
scan_parts = 5
step_parts = 6

annotated_humans = 0
annotated_obstacles = 0

#temp2 = np.zeros((1, 36))

def RepresentsInt(s):
    try: 
        int(s)
        return True
    except ValueError:
        return False
        
def RepresentsFloat(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

'''
def Calculate_Metrics(annotated_data):
    global classification_array
    pos = 1
    true_pos = 0.0
    true_neg = 0.0
    false_pos = 0.0
    false_neg = 0.0
    neg = 0
    #print len(annotated_data)
    classification_array = np.array(classification_array)
    #print len(classification_array)
    for i in range(len(classification_array)):
        if annotated_data[i]==1:
            if classification_array[i]==1:
                true_pos += 1.0
            else:
                false_neg += 1.0
        else:
            if classification_array[i]==1:
                false_pos += 1.0
            else:
                true_neg += 1.0
    precision = true_pos/(true_pos + false_pos)
    recall = true_pos/(true_pos + false_neg)
    accuracy = (true_pos + true_neg)/(true_pos + false_pos + true_neg + false_neg)
    print "Precision : {0}".format(precision)
    print "Recall : {0}".format(recall)
    print "Accuracy : {0}".format(accuracy)
    input ("Press any key to exit")
    return
''' 


def laser_listener():
    
    global class_path, pca_path, sub_topic, timewindow, range_limit
    global annotations
    global gaussian, pca_obj
    global timewindow, range_limit, annotated_data, classification_array
    global all_clusters, all_hogs, all_gridfit, all_orthogonal,all_annotations
    global tot_results, metrics
    
    all_clusters=[]
    all_hogs=[]
    all_gridfit=[]
    all_orthogonal=[]
    all_annotations=[]
    tot_results=[]

    if not len(sys.argv) == 7:
        print "###################################"
        print "For non interactive input run as follows : "
        print "python hpr.py <classifier_object_path> <pca_objec_path> <laserscan_topic> <timewindow_in_frames> <maximum_scan_range> <0_or_1_for_metrics>"
        print "###################################"
        exit()
    else:
        class_path = str(sys.argv[1])
        pca_path = str(sys.argv[2])
        if not os.path.isfile(class_path):
            while True :
                try:
                    class_path=raw_input('Enter classifier object file path: ')
                    if os.path.isfile(class_path):
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        print "Classifier File : {0}".format(class_path)
        
        if not os.path.isfile(pca_path):
            while True :
                try:
                    pca_path=raw_input('Enter pca object file path: ')
                    if os.path.isfile(pca_path):
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        print "File : {0}".format(pca_path)

        rospy.init_node('laser_listener', anonymous=True)
        #ADDITIONS command line inputs klp
        scan_topic = str(sys.argv[3])
        timewindow = int(sys.argv[4])
        while not RepresentsInt(timewindow):
            timewindow=input('Set timewindow in frames: ')
            if RepresentsInt(timewindow):
                break
            else:
                print 'Try again'
        range_limit = float(sys.argv[5])
        while not (RepresentsInt(range_limit) or RepresentsFloat(range_limit)):
            range_limit=input('Set maximum scan range in m: ')
            if RepresentsInt(range_limit):
                break
            else:
                print 'Try again'

        metrics = int(sys.argv[6])
        while not (RepresentsInt(metrics) and metrics != '1' and metrics != '0'):
            metrics=input('Set if you want performance metrics or not: ')
            if RepresentsInt(metrics):
                break
            else:
                print 'Try again'

    if metrics == 1:
	global store_file

	print 'Please give a filename for storage.'
	filename = raw_input()
	if filename != '':
	    store_file = filename
	else:
	    print 'You have to give a valid filename.'
	    exit()

    print "metrics: ",metrics
    print "Classifier object path : ", class_path 
    print "PCA object path : ", pca_path
    print "Scan Topic : ", sub_topic
    print "Timewindow (frames) : ",timewindow
    print "Maximum scan range (meters)", range_limit
    print "Waiting for laser scans ..."
    #ADDITIONS command line inputs klp
   
 
    rospy.Subscriber(sub_topic,LaserScan,online_test)
    scan_received=rospy.Time.now().to_sec()
    gaussian, pca_obj = loadfiles()
    while not rospy.is_shutdown():  
        rospy.spin()
    #we come here when Ctrl+C is pressed, so we can save!
    if metrics == 1:
        b={}
        b['timewindow']=timewindow
        b['range_limit']=range_limit
        b['angle_increment']=angle_increment
        #b['scan_time']=scan_time
        b['angle_min']=angle_min
        b['angle_max']=angle_max
        b['intensities']=intensities
        b['wall']=wall
        #print b['wall']
        #b['annotations']=annotations
        #b['ranges']=ranges_
        try:
            os.remove('classification_results.mat')
        except OSError:
            pass
        sio.savemat('classification_results',b);

    print 'duration in milliseconds = {0}'.format(total_cluster_time)

    if metrics == 1:
    	get_accuracy(all_annotations,tot_results)
    	save_data(all_clusters, all_orthogonal, all_gridfit, all_hogs, all_annotations)
    print "D O N E !"
    #Calculate_Metrics(annotated_data)
    #sys.exit()


def online_test(laser_data):
    global wall_flag, wall, fr_index, intens, w_index, phi, sampling, limit #prosthesa to limit edw giati den to epairne global
    global phi, mybuffer, z, zscale, gaussian,timewindow , wall_cart,ax, ax3 ,fig1,fig4, kat
    global pca_obj, pca_plot
    global ranges_, intensities, angle_increment, scan_time, angle_min, angle_max, first_time_ranges, total_cluster_time
    global mybuffer2, num_c
    global all_clusters, all_hogs, all_gridfit, all_orthogonal,all_annotations
    global tot_results, metrics


    millis_start = int(round(time.time() * 1000))
    if wall_flag == 0:
        #print "-------------- 1"
        if w_index == 1:
            #print "-------------- 2"
            sampling = np.arange(0,len(np.array(laser_data.ranges)),2)#apply sampling e.g every 2 steps

            #wall data now contains the scan ranges
            wall = np.array(laser_data.ranges)
            
            mybuffer = wall
            #get indexes of scans >= range_limit
            filter=np.where(wall >= range_limit)
            #set those scans to maximum range
            wall[filter] = range_limit
            w_index=w_index+1
            
        if w_index<limit: #loop until you have enough scans to set walls
            #print "-------------- 3"
            wall = np.array(laser_data.ranges)
            filter = np.where(wall >= range_limit)
            wall[filter] = range_limit
            mybuffer = np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 360)
            w_index = w_index+1
        if w_index==limit:
            #print "-------------- 4"
            mybuffer = np.vstack((mybuffer,wall ))
            phi = np.arange(laser_data.angle_min,laser_data.angle_max,laser_data.angle_increment)[sampling]
            wall = (np.min(mybuffer, axis=0)[sampling])-0.1 #select min of measurements
            wall_cart = np.array(pol2cart(wall,phi,0) ) #convert to Cartesian
            wall_flag = 1
            kat,ax,ax3=initialize_plots(wall_cart)

            angle_increment=laser_data.angle_increment
            scan_time=laser_data.scan_time
            angle_min=laser_data.angle_min
            angle_max=laser_data.angle_max
            intensities=laser_data.intensities
	    angle_prev=angle_min
            print 'walls set...'
        
    else:
        #walls are set, process scans
        #print "-------------- 5"
        ranges = np.array(laser_data.ranges)[sampling]
        filter = np.where(ranges < wall) # filter out walls
        ranges = ranges[filter]
        theta = phi[filter]


        if metrics == 1:
            if first_time_ranges:
                ranges_= np.array(laser_data.ranges)[sampling]
                first_time_ranges = False
            else:
                ranges_ = np.vstack((ranges_, np.array(laser_data.ranges)[sampling]))

        if (len(ranges)>3): #each scan should consist of at least 3 points to be valid
            #print "-------------- 6"
            C = np.array(pol2cart(ranges,theta,z) ) #convert to Cartesian


            if (fr_index ==1 ):
                mybuffer = C #mybuffer is the cartesian coord of the first scan
		mybuffer2 = [C]
		num_c = np.array(len(C))
	
            else :
                mybuffer = np.concatenate((mybuffer,C), axis=0 )  #  add the next incoming scans to mybuffer until you have <timewindow>scans
		mybuffer2.append((mybuffer2,[C]))
		num_c=np.vstack((num_c,len(C)))

            if (fr_index == timewindow ):
                mybuffer=mybuffer[np.where( mybuffer[:,0] > 0.2),:][0] #mishits safety margin
                mybuffer=mybuffer[np.where( mybuffer[:,0] < 5),:][0]#ignore distant points


                if len(mybuffer>3): #at least 3 points are needed to form a cluster
		    clustering_procedure(mybuffer, num_c)
 
                fr_index=0
                z=- z_scale
            z = z + z_scale
            fr_index=fr_index+1

    millis_end = int(round(time.time() * 1000))
    total_cluster_time = total_cluster_time + millis_end - millis_start



def pol2cart(r,theta,zed):

    #convert cylindical coordinates to cartesian ones
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C


def loadfiles():
    
    global class_path
    global all_annotations
    global pca_path
    

    classifier = pickle.load( open( class_path, "rb" ) )
    pca_obj = pickle.load(open ( pca_path, "rb"))
    #all_annotations = pickle.load(open("cluster_labels/video5annotations.p","rb"))
    mat=sio.loadmat("cluster_labels/video5_labels.mat")
    #all_annotations=mat['annotations'][0]
    #print 'AN = ',len(all_annotations)
    
    return classifier, pca_obj


def initialize_plots(wall_cart):
    global fig1,fig4

    #2D projection of the cluster
    
    temp=plt.figure()
    plot2d = temp.add_subplot(111)
    plot2d.set_xlabel('Vertical distance')
    plot2d.set_ylabel('Robot is here')
    plot2d.plot(wall_cart[:,0],wall_cart[:,1])
    


    #3D clusters
    fig1=plt.figure()
    plot3d= fig1.gca(projection='3d')
    plot3d.set_xlabel('X - Distance')
    plot3d.set_ylabel('Y - Robot')
    plot3d.set_zlabel('Z - time')

    #translate and rotate the 3D cluster
    fig4=plt.figure()
    plot_align= fig4.gca(projection='3d')
    plot_align.set_xlabel('X - Distance')
    plot_align.set_ylabel('Y - Robot')
    plot_align.set_zlabel('Z - time')


    plt.show()
    return plot2d,plot3d,plot_align

def save_data(point_clouds, alignment, gridfit, hogs, annotations) :
    global store_file

    b={}
    b['point_clouds']=point_clouds
    b['alignment']=alignment
    b['gridfit']=gridfit
    b['hogs']=hogs
    b['annotations']=annotations

    sio.savemat(store_file+'.mat',b);
    

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





def clustering_procedure(clear_data, num_c):

    global cc, ccnames, fig1, z, z_scale, fig4
    global all_clusters,all_hogs,all_gridfit,all_orthogonal
    global tot_results, all_annotations, metrics
    
    #warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    colors=[]
    align_cl=[]	#contains the aligned data clouds of each cluster
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at least one valid cluster
    grids=[]

    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN

    max_label=int(np.amax(cluster_labels))


    #[xi,yi,zi]: the array of data points of the specific frame
    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]

    fig1.clear()
    fig4.clear()

    #for every created cluster - its data points
    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
	    print ' cluster ',k

            valid_flag=1

	    #points of every cluster at each timewindow-frame
	    [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]

	    speed(xk,yk,zk)
	    trans_matrix =[[xk,yk,zk]]
	    all_clusters.append([xk,yk,zk])


	    #we get U by appying svd to the covariance matrix. U represents the rotation matrix of each cluster based on the variance of each dimention.
	    U,s,V=np.linalg.svd(np.cov([xk,yk,zk]), full_matrices=False)
	    #print 'U = {} s = {} V={}'.format(U,s,V)

	    #translate each cluster to the begining of the axis and then do the rotation
	    [xnew,ynew,znew]=translate_cluster(xk,yk,zk)

	    #(traslation matrix) x (rotation matrix) = alignemt of cluster
	    #alignment_result=[[sum(a*b for a,b in zip(X_row,Y_col)) for X_row in zip(*[xnew,ynew,znew])] for Y_col in U]
	    alignment_result=multiply_array(xnew,ynew,znew, V)
	
	    #print 'x = {} \n align_x = {}'.format(xk, alignment_result[0])
	    steps2(xk,yk,zk)
	    '''
	    points = []
	    points.append([xk[0],yk[0]])
	
	    for i in range(1,len(xk)) :
		points.append([xk[i],yk[i]])

	    #print 'PP {}'.format(points)
	    hull = ConvexHull(points)
	    
	    hull_indices = np.unique(hull.simplices.flat)
	    print 'HULL {}'.format(hull_indices)
	    hull_pts = []
	    for j in range(0, len(hull_indices)):
		index = hull_indices[j]
	    	hull_pts.append(points[index])
	    #print 'hull_pts {}'.format(hull_pts)
	    

	    #plt.plot(points[:, 0], points[:, 1], 'ko', markersize=10)
	    plt.plot(hull_pts[:, 0], hull_pts[:, 1], 'ro', alpha=.25, markersize=20)
	    plt.show()
	    '''
	    
	    align_cl.append(alignment_result)
	    all_orthogonal.append(alignment_result)

	    #steps2(alignment_result[0], alignment_result[1], alignment_result[2])

	    vcl.append(k)
            colors.append(ccnames[k%12])
            grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2], 16, 16) #extract surface - y,z,x alignment_result[1]
	    all_gridfit.append(grid)

            grid=grid-np.amin(grid)
	    grids.append(grid)

	    features=hog(grid)
	    f=hog(grid, orientations=6, pixels_per_cell=(8, 8), cells_per_block=(1, 1), visualise=False)
	    all_hogs.append(f)
            hogs.append(f)  #extract hog features

	    
    
    fig1.show()
    #fig4.show()


    update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids)



def get_accuracy(ann, results):

    acc=0.0
    T=0.0
    F=0.0

    if len(ann) != len(results):
	print 'ERROR'

    for i in range(0, len(ann)):
	if ann[i]==results[i]:
	    T=T+1
	else:
	    F=F+1

    acc=100*T/(T+F)
    print 'acc = {}'.format(acc)


#gets the local minimums and maximums of the st.deviation matrix
#tries to compute the number of the steps with the use of this information
def compute_steps(dev) :

    min_array = []
    max_array = []
    m = round(sum(dev)/len(dev),2)

    #print 'compute steps ... '

    if dev[0] < dev[1] :
	if diff_error2(dev[0], m):
	    min_array.append(0)

    elif dev[0] > dev[1] :
	if diff_error2(dev[0], m):
	    max_array.append(0)

    for i in range(1,len(dev)-1) :
	if dev[i-1] <= dev[i] and dev[i] > dev[i+1] :
	    if diff_error2(dev[i], m):
	    	max_array.append(i)

	elif dev[i-1] >= dev[i] and dev[i] < dev[i+1]:
	    if diff_error2(dev[i], m):
	    	min_array.append(i)

	else:
	    continue



    if dev[len(dev)-2] < dev[len(dev)-1] :
	if diff_error2(dev[len(dev)-1], m):
	    max_array.append(len(dev)-1)

    elif dev[len(dev)-2] > dev[len(dev)-1] :
	if diff_error2(dev[len(dev)-1], m):
	    min_array.append(len(dev)-1)
    elif dev[len(dev)-2] == dev[len(dev)-1] :
	if diff_error2(dev[len(dev)-1], m):
	    max_array.append(len(dev)-1)
    print 'max = {}    min = {}'.format(max_array, min_array)



    #remove min and max that are in sequence
    #every two min or two max -> + 1 step
    #if a min/max is between two local max/min respectively  -> + 0.5 step
    steps = 0.0

    if len(min_array) == 0 and len(max_array) == 0 or len(min_array) == 0 and len(max_array) == 1 or len(min_array) == 1 and len(max_array) == 0 or len(min_array) == 1 and len(max_array) == 1:
	steps = 0
    elif len(max_array) >= 2 and (len(min_array) == 1 or len(min_array) == 0): 
	new_max = []
    	for i in range(0, len(max_array)-1) :
	    if max_array[i] +1 != max_array[i+1]:
	   	new_max.append(max_array[i])

    	if max_array[len(max_array)-2] +1 != max_array[len(max_array)-1]:
	    new_max.append(max_array[len(max_array)-1])
	
	steps = len(new_max) - 1.0
    elif len(min_array) >= 2 and (len(max_array) == 1 or len(max_array) == 0): 
	new_min = []
    	for i in range(0, len(min_array)-1) :
	    if min_array[i] +1 != min_array[i+1]:
	   	new_min.append(min_array[i])

        if min_array[len(min_array)-2] +1 != min_array[len(min_array)-1]:
	    new_min.append(min_array[len(min_array)-1])
	
	steps = len(new_min) - 1.0
    else:
    	new_min = []
        for i in range(0, len(min_array)-1) :
	    if min_array[i] +1 != min_array[i+1]:
	   	new_min.append(min_array[i])

        if min_array[len(min_array)-2] +1 != min_array[len(min_array)-1]:
	    new_min.append(min_array[len(min_array)-1])

	new_max = []
    	for i in range(0, len(max_array)-1) :
	    if max_array[i] +1 != max_array[i+1]:
	   	new_max.append(max_array[i])

    	if max_array[len(max_array)-2] +1 != max_array[len(max_array)-1]:
	    new_max.append(max_array[len(max_array)-1])

	if len(new_min) == 0 and len(new_max) == 0 or len(new_min) == 0 and len(new_max) == 1 or len(new_min) == 1 and len(new_max) == 0 or len(new_min) == 1 and len(new_max) == 1:
	    steps = 0
    	elif len(new_max) >= 2 and (len(new_min) == 1 or len(new_min) == 0): 
	    steps = len(new_max) - 1.0
	elif len(new_min) >= 2 and (len(new_max) == 1 or len(new_max) == 0): 
	    steps = len(new_min) - 1.0
	else:
	    if new_min[0] > new_max[0]:
		first = new_max
		second = new_min
	    else:
		first = new_min
		second = new_max
	    fv1 = first[0]
	    fv2 = first[1]
	    del first[0]
	    steps = steps + 1.0

	    while len(first) != 0 and len(second) != 0:
		if fv1 < second[0] and fv2 > second[0]:
		    steps = steps + 0.5
		else:
		    steps = steps + 1.0

		if len(second) == 1:
		    break
		fv1 = second[0]
		fv2 = second[1]
		del second[0]
	
		temp = first
		first = second
		second = temp
    
	    if len(first) >=2:
		steps = steps + len(first) -1

    print 'Compute steps :  ',steps




def diff_error(prev, value, next):

    error = 0.01

    if abs(value - prev)<=error and abs(next - value)<= error:
	return False
    
    return True

def diff_error2(value, m):

    error = 0.005

    if abs(value - m)<=error :
	return False
    
    return True


# separate data in equal parts of points.
# compute the standard deviation by median and not the avg of points
def steps2(x, y, z):

    global scan_parts, step_parts

    num = 0
    split = len(x)/step_parts
    dev2 = []
    flag = False

    while num <= len(x) :
	if flag == True:
		break

	xn = x[num:num+split]
	yn = y[num:num+split]
	zn = z[num:num+split]
	
	if len(xn) != 0 and len(yn)!=0 and len(zn)!=0 :
		xmean = np.median(np.array(xn))
		ymean = np.median(np.array(yn))
		zmean = np.median(np.array(zn))

	
		sumx = 0.0

		for m in range(0,len(xn)):
	    		sumx = sumx + pow((xn[m] - xmean), 2) + pow((yn[m] - ymean), 2) + pow((zn[m] - zmean), 2)

		dev2.append(round((math.sqrt(sumx/len(xn))),2))
	num = num + split
	
	if len(x)-num < split :
	    	split = len(x)-num
		flag = True
	

    print 'deviation = {}'.format(dev2)
    #steps_from_deviation(dev2)
    compute_steps(dev2)


# separate data in equal time parts.
# compute the standard deviation by median and not the avg of points
def steps(x, y, z):

    global z_scale, scan_parts,ax3,fig4

    z_angle = z[0]
    count = 0 
    xk = []
    yk = []
    zk = []
    deviation = []
    dev2 = []

    xmed = []
    ymed = []
    zmed = []

    c=0
    
    while z_angle <= z[len(z)-1]:
    	z_filter = np.where(z==z_angle)

	if count < scan_parts:
	    for i in range(0, len(x[z_filter])):
	        xk.append(x[z_filter][i])
	        yk.append(y[z_filter][i])
		zk.append(z[z_filter][i])

	else:
	    
    	    if len(xk) !=0 and len(yk) !=0:
		ax3.scatter(xk,yk, zk, 'z', 30, cc[c%12]) 
		fig4.add_axes(ax3)
		fig4.show()
		arr = np.array([xk,yk,zk])
		#print 'standard dev = {}'.format(np.std(arr))
		
		deviation.append(round(np.std(arr),2))

		xmean = np.median(np.array(xk))
		ymean = np.median(np.array(yk))
		zmean = np.median(np.array(zk))

		xavg = np.average(np.array(xk))
		yavg = np.average(np.array(yk))
		zavg = np.average(np.array(zk))
	
		sumx = 0.0
		sumy = 0.0
		sumz = 0.0
		for m in range(0,len(xk)):
		    sumx = sumx + pow((xk[m] - xmean), 2) + pow((yk[m] - ymean), 2)

		dev2.append(round((math.sqrt(sumx/len(xk))),2))

		del xk[:]
		del yk[:]
		del zk[:]

		for i in range(0, len(x[z_filter])):
		    xk.append(x[z_filter][i])
	    	    yk.append(y[z_filter][i])
		    zk.append(z[z_filter][i])
		count = 0
		c=c+1

	    
	count = count+1
	z_angle = z_angle + z_scale
	
	
    if len(xk) !=0 and len(yk) != 0:
	arr = np.array([xk,yk,zk])
	deviation.append(round(np.std(arr),2))

	sumx = 0.0
	xmean = np.median(np.array(xk))
	ymean = np.median(np.array(yk))
	for m in range(0,len(xk)):
	    sumx = sumx + pow((xk[m] - xmean), 2) + pow((yk[m] - ymean), 2)
	dev2.append(round((math.sqrt(sumx/len(xk))),2))

    step = 0
    temp_i = 1
    for i in range(0, len(deviation)-1):
	diff = abs(deviation[i+1] - deviation[temp_i])

	if diff>=0.1 :
	    step = step + 1
	    temp_i = i

    dev = np.array(deviation)
   # min_dev = np.where(dev == dev.min())

    print 'deviation : {} \n '.format(deviation)
    print 'dev2 {}'.format(dev2)

    compute_steps(dev2)




def speed(x, y, z) :

    global z_scale, scan_parts

    z_angle = z[0]
    dist = 0.0
    scan_speed1 = 0.0
    mean_array = []
    count = 0 
    xk = []
    yk = []
    

    
    while z_angle <= z[len(z)-1]:
    	z_filter = np.where(z==z_angle)
	
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

		#dist = dist + math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))
	    
	count = count+1
	z_angle = z_angle + z_scale
	

    if len(xk) !=0 and len(yk) !=0:
	mean_array.append([np.median(xk), np.median(yk)])

    d = 0.0
    for i in range(0,len(mean_array)-1):
	d = d + math.sqrt(pow(mean_array[i+1][0] - mean_array[i][0], 2) + pow(mean_array[i+1][1] - mean_array[i][1], 2))

    #compute the speed at each scan -> m/sec
    scan_speed = d/(z_angle - z[0])

    print ' SPEED = ',scan_speed


def update_plots(flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids):
    
    global fig1, ax, ax3, wall_cart, gaussian, classification_array, pca_obj, hogs_temp, align_plot, fig4,kat
    global annotations, first_time, all_annotations,annotated_humans,annotated_obstacles
    global tot_results, metrics, f_time

    temp = []
    store_results = []
    centerx = []
    centery = []
    centerz = []


    #zscore the entire hogs table, not single cluster hogs
    if flag==1:
        kat.clear()
        kat.plot(wall_cart[:,0],wall_cart[:,1])

        if np.array(hogs).shape==(1,36):
            #temp = zscore(np.array(hogs)[0])
            temp = np.array(hogs)[0]

        else:
            for i in range(0,len(hogs)):
                #temp.append(zscore(np.array(hogs[i])))
                temp.append(np.array(hogs[i]))
        

	#temp_pca = pca_obj.transform(temp)
        #results = gaussian.predict(temp_pca)
	results = gaussian.predict(temp)
        print results

        cnt=0
	col_list=[]

        for k in vcl:
	    #fig1.clear()
            filter=np.where(cluster_labels==k)
            
            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

	    [xc,yc,zc] = [align_cl[cnt][0], align_cl[cnt][1], align_cl[cnt][2]]


	    if len(xc)==0:
		print 'out of data'
		continue

	    #print 'xc = {} align_cl[cnt][0] ={}'.format(xc,align_cl[cnt][0])

            if results[cnt]==1:
                #classification_array.append(1)
                kat.scatter(x,y,s=20, c='r')
                ax.scatter(x,y, zed, 'z', 30, cc[k%12]) #human
		ax.scatter(xc,yc, zc, 'z', 30, cc[k+1%12]) #human
                fig1.add_axes(ax)

		#ax3.scatter(xc,yc, zc, 'z', 30, cc[k%12]) #human
                #fig4.add_axes(ax3)
            else:
                #classification_array.append(0)
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, cc[k%12]) #object
		ax.scatter(xc,yc, zc, 'z', 30, cc[k+1%12]) #obj
                fig1.add_axes(ax)

		#ax3.scatter(xc,yc, zc, 'z', 30, cc[k%12]) #obj
                #fig4.add_axes(ax3)

	    #plt.imshow(grids[cnt], cmap = 'gray', interpolation = 'bicubic')
	    #plt.show()
	    cnt=cnt+1
	    #fig1.show()
            plt.pause(0.0001)

	    if metrics == 1 :
	    	ha = raw_input()
            	if (int(ha)==1 or int(ha)==0):
                	ha = int(ha)
                	

			#all_annotations.append(ha)
	    
			if first_time:
                		all_annotations = np.array(ha)
				tot_results = np.array(results)
                		first_time = False
            		else:
                		all_annotations=np.hstack((all_annotations,np.array(ha)))
				


	pickle.dump(store_results, open('stored_predictions.p','a'))
	file_name=open('stored_predictions.txt','a')
	file_name.write(str(store_results))
	file_name.write("\n")
	file_name.close()

        if metrics == 1:
            if f_time:
                tot_results = np.array(results)
                f_time = False
            else:
		tot_results=np.hstack((tot_results,np.array(results)))	

	'''
        if metrics == 1:
            if first_time:
                annotations = np.array(results)
                first_time = False
            else:
                annotations=np.hstack((annotations,np.array(results)))
	'''
	

	hogs_temp = np.array(np.array(temp))
        


if __name__ == '__main__':
    laser_listener()






      
