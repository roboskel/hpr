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
import my_skimage #skimage library has been slightly customized (_hog.py)
from os import listdir
from os.path import isfile, join, splitext
from my_skimage.feature import hog
from sensor_msgs.msg import LaserScan
from scipy.stats.mstats import zscore
from scipy import interpolate
import curses 

#stdscr = curses.initscr()
#curses.cbreak()
#stdscr.keypad(1)
#stdscr.timeout(0);
stdscr = None


ccnames =['red', 'black', 'violet', 'blue', 'cyan', 'rosy', 'orange', 'gray','green', 'brown', 'yellow', 'gold']
cc  =  ['r',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','#808080', 'g','#8B4513','y','#FFD700']
wall_flag = 0
fr_index = 1
z = 0
dt = 25;#period in ms (dt between scans)
speed = 5;#human walking speed in km/h
z_scale = float(speed*dt) / float(3600)
w_index = 0 #current laser scan used (wall index)
limit = 3 #how many laser scans to use in order to create the walls

plt.ion()

classifier_path = ''
save_folder = ''
pause_function = False
first_time_annotations = True
first_time_ranges = True
first_time_results = True

total_cluster_time = 0
hogs_temp = []
scan_parts = 5
step_parts = 10
trace_array = []
trace_count = False

num_of_diagrams = 0

trace_results = []
cls_results = []
traced_clusters = []
max_cls = 0
first_trace = True
track_parts = 4
step_counter = 0
pause = False
basic_counter = 0
tot_speed = []
tot_steps = []
plot_figure = None

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

def heardEnter():
    i,o,e = select.select([sys.stdin],[],[],0.0001)
    for s in i:
        if s == sys.stdin:
            input = sys.stdin.readline()
            return True
    return False


def laser_listener():
    
    global classifier_path, save_folder, timewindow, range_limit
    global LDA_classifier
    global timewindow, annotated_data
    global all_clusters, all_hogs, all_gridfit, all_orthogonal,all_annotations
    global tot_results, metrics, pause_function, stdscr, num_of_diagrams
    
    all_clusters=[]
    all_hogs=[]
    all_gridfit=[]
    all_orthogonal=[]
    all_annotations=[]
    tot_results=[]
    
    classifier_path = rospy.get_param('/human_pattern_recognition/classifier_path', os.path.dirname(os.path.realpath(sys.argv[0]))+'/cluster_labels/LDA_classifier.p')
    save_folder = rospy.get_param('/human_pattern_recognition/save_path',os.path.dirname(os.path.realpath(sys.argv[0]))+'/cluster_labels/')
    scan_topic = rospy.get_param('/human_pattern_recognition/scan_topic','scan')
    timewindow = rospy.get_param('/human_pattern_recognition/timewindow', 40)
    range_limit = rospy.get_param('/human_pattern_recognition/max_range', 10)
    metrics = rospy.get_param('/human_pattern_recognition/save_metrics', False)
    pause_function = rospy.get_param('/human_pattern_recognition/pause_function', False)
    
    num_of_diagrams = rospy.get_param('/human_pattern_recognition/num_of_diagrams', 0)

    rospy.init_node('laser_listener', anonymous=True)
    if pause_function:
	stdscr = curses.initscr()
	curses.cbreak()
	stdscr.keypad(1)
	stdscr.timeout(0);


    if metrics:
	global store_file

	print 'Please give a filename for storage.'
	filename = raw_input()
	if filename != '':
	    store_file = filename
	else:
	    print 'You have to give a valid filename.'
	    exit()

    print "Waiting for laser scans ..."
   
 
    rospy.Subscriber(scan_topic,LaserScan,online_test)
    LDA_classifier = loadClassifier()
    while not rospy.is_shutdown():  
        rospy.spin()
    #we come here when Ctrl+C is pressed, so we can save!
    if metrics:
        b={}
        b['timewindow']=timewindow
        b['range_limit']=range_limit
        b['angle_increment']=angle_increment
        b['angle_min']=angle_min
        b['angle_max']=angle_max
        b['intensities']=intensities
        b['wall']=wall
        try:
            os.remove('classification_results.mat')
        except OSError:
            pass
        sio.savemat('classification_results',b);


    if metrics:
    	get_accuracy(all_annotations,tot_results)
    	save_data(all_clusters, all_orthogonal, all_gridfit, all_hogs, all_annotations)
    if pause_function:
	curses.nocbreak()
	stdscr.keypad(0)
    	curses.echo()
    	curses.endwin()


def online_test(laser_data):
    global wall_flag, wall, fr_index, w_index, sampling, limit
    global phi, mybuffer, z, zscale, LDA_classifier, timewindow , wall_cart, ax, ax3 , top_view_figure
    global ranges_, intensities, angle_increment, scan_time, angle_min, angle_max, first_time_ranges, total_cluster_time
    global mybuffer2, num_c
    global all_clusters, all_hogs, all_gridfit, all_orthogonal,all_annotations
    global tot_results, metrics
    global trace_array

    global trace_count
    global trace_results, cls_results
    global traced_clusters, first_trace, max_cls

    global pause, pause_function, stdscr, num_of_diagrams

    if pause_function:
    	key = stdscr.getch()
    	stdscr.refresh()
    	if key == curses.KEY_ENTER or key == 10:
		pause = not pause
        	print '\033[93m '+ str(pause) + ' \033[0m'

    laser_ranges = list(laser_data.ranges)
    for i in range(0, len(laser_ranges)):
	if laser_ranges[i]>range_limit:
		j = i - 1;
		while  laser_ranges[j] > range_limit:
			j = j - 1;
			if j == 0:
				break;
		laser_ranges[i] = laser_ranges[j];
    millis_start = int(round(time.time() * 1000))
    if wall_flag == 0:
        if w_index == 0:
            sampling = np.arange(0,len(np.array(laser_ranges)),2)#apply sampling e.g every 2 steps

            #wall data now contains the scan ranges
            wall = np.array(laser_ranges)
            
            mybuffer = wall
            #get indexes of scans >= range_limit
            filter=np.where(wall >= range_limit)
            #set those scans to maximum range
            wall[filter] = range_limit
            w_index=w_index+1
            
        if w_index<limit: #loop until you have enough scans to set walls
            wall = np.array(laser_ranges)
            filter = np.where(wall >= range_limit)
            wall[filter] = range_limit
            mybuffer = np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 360)
            w_index = w_index+1
        if w_index==limit:
            mybuffer = np.vstack((mybuffer,wall ))
            phi = np.arange(laser_data.angle_min,laser_data.angle_max,laser_data.angle_increment)[sampling]
            wall = (np.min(mybuffer, axis=0)[sampling])-0.1 #select min of measurements
            wall_cart = np.array(pol2cart(wall,phi,0) ) #convert to Cartesian
            wall_flag = 1
	    
	    if num_of_diagrams > 0:
	            initialize_plots(wall_cart)

            angle_increment=laser_data.angle_increment
            scan_time=laser_data.scan_time
            angle_min=laser_data.angle_min
            angle_max=laser_data.angle_max
            intensities=laser_data.intensities
	    angle_prev=angle_min
            print 'walls set...'
        
    else:
        #walls are set, process scans
	ranges = np.array(laser_ranges)[sampling]
        filter = np.where(ranges < wall) # filter out walls
        ranges = ranges[filter]
        theta = phi[filter]


        if metrics:
            if first_time_ranges:
                ranges_= np.array(laser_ranges)[sampling]
                first_time_ranges = False
            else:
                ranges_ = np.vstack((ranges_, np.array(laser_ranges)[sampling]))

        if (len(ranges)>3): #each scan should consist of at least 3 points to be valid
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
	else:
	    
	    	del trace_results[:]
	    	del cls_results[:]
	    	del traced_clusters[:]
	   	trace_count = 0
	    	first_trace = True
		trace_count = False
	    	max_cls =0
	

    millis_end = int(round(time.time() * 1000))
    total_cluster_time = total_cluster_time + millis_end - millis_start



def pol2cart(r,theta,zed):

    #convert polar coordinates to cartesian ones
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C


def loadClassifier():
    
    global classifier_path
    

    classifier = pickle.load( open( classifier_path, "rb" ) )
    
    return classifier


def initialize_plots(wall_cart):
    global plot_figure, top_view_figure, ax, ax3, num_of_diagrams

    if num_of_diagrams == 2:
    	plot_figure = plt.figure() 
    	top_view_figure = plot_figure.add_subplot(1,2,1)
	top_view_figure.set_title("Top view")
    	top_view_figure.set_xlabel('Vertical distance')
    	top_view_figure.set_ylabel('Robot is here')
    	top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])
    


    	ax = plot_figure.add_subplot(1,2,2,projection='3d')
    	ax.set_title("3D view")
    	ax.set_xlabel('X - Distance')
    	ax.set_ylabel('Y - Robot')
    	ax.set_zlabel('Z - Time')

    elif num_of_diagrams == 3:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(2,2,1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')
        top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])



        ax = plot_figure.add_subplot(2,2,2,projection='3d')
        ax.set_title("3D view")
        ax.set_xlabel('X - Distance')
        ax.set_ylabel('Y - Robot')
        ax.set_zlabel('Z - Time')

        ax3 = plot_figure.add_subplot(2,2,3,projection='3d')
        ax3.set_title("Aligned 3D clusters")
        ax3.set_xlabel('X - Distance')
        ax3.set_ylabel('Y - Robot')
        ax3.set_zlabel('Z - Time')

    elif num_of_diagrams == 1:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(1,1,1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')
        top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])

 

    plt.show()
    #return plot2d,plot3d,plot_align

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

    global cc, ccnames, z, z_scale, _3d_figure
    global all_clusters,all_hogs,all_gridfit,all_orthogonal
    global tot_results, all_annotations, metrics
    global pause
    
    hogs=[]
    colors=[]
    align_cl=[]	#contains the aligned data clouds of each cluster
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at least one valid cluster
    grids=[]
    cls = []

    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN

    max_label=int(np.amax(cluster_labels))

    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]

    #for every created cluster - its data points
    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :

            valid_flag=1

	    #points of every cluster at each timewindow-frame
	    [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]

	    speed(xk,yk,zk)
	    trans_matrix =[[xk,yk,zk]]
	    all_clusters.append([xk,yk,zk])


	    #we get U by applying svd to the covariance matrix. U represents the rotation matrix of each cluster based on the variance of each dimention.
	    U,s,V=np.linalg.svd(np.cov([xk,yk,zk]), full_matrices=False)

	    #translate each cluster to the begining of the axis and then do the rotation
	    [xnew,ynew,znew]=translate_cluster(xk,yk,zk)

	    #(traslation matrix) x (rotation matrix) = alignemt of cluster
	    alignment_result=[[sum(a*b for a,b in zip(X_row,Y_col)) for X_row in zip(*[xnew,ynew,znew])] for Y_col in U]
	    alignment_result=multiply_array(xnew,ynew,znew, V)
	
	    #steps2(xk,yk,zk)

	    
	    cls.append([xk,yk,zk])
	    
	    align_cl.append(alignment_result)
	    all_orthogonal.append(alignment_result)

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

	
    if valid_flag != 0:    
    	trace(cls)


	#3d_figure.show()
    if pause_function:
    	print '\033[93m '+ str(pause) + ' \033[0m'
    if not pause:
	#_3d_figure.clear()
	if num_of_diagrams > 1:
		ax.clear()
		ax.set_title("3D view")
    		ax.set_xlabel('X - Distance')
    		ax.set_ylabel('Y - Robot')
    		ax.set_zlabel('Z - Time')

    	update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl, align_cl, grids)


#choice parameter declares the first (True) or the last (False) part of cluster
def get_centroid(cluster, choice):

    if choice == True:
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

    global trace_results, cls_results, trace_count
    global traced_clusters, max_cls

    last_element = len(trace_results) - 1

    for i in range(0, len(trace_results[last_element])):
	index = trace_results[last_element][i]

	if index != -1:
	    if len(traced_clusters[index]) == 4:
	    	del traced_clusters[index][0]

	    traced_clusters[index].append(cls_results[last_element][i])
	else:
	    traced_clusters.append([cls_results[last_element][i]])


def trace(cls):
    global trace_array
    global trace_count, track_parts

    global trace_results, cls_results, trace_3d_figure
    global traced_clusters, first_trace, max_cls

    global step_counter, num_of_diagrams

    error = 100
    min_dist = -1.0
    index = 0
    list_dist = []
    temp_list = []
    flag = True

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

	if len(trace_results) == track_parts:
	    if not pause:
            	#trace_3d_figure.clear()
		if num_of_diagrams > 2:
			ax3.clear()
			ax3.set_title("Aligned 3D clusters")
			ax3.set_xlabel('X - Distance')
			ax3.set_ylabel('Y - Robot')
			ax3.set_zlabel('Z - Time')


	    	if first_trace:
			create_trace()
			first_trace = False
			step_processing()
		
	    	else:
			continue_trace()
			step_counter  = step_counter + 1

	    	if step_counter == track_parts:
			step_processing()
			step_counter = 0
		
            #display in plot
            #trace_3d_figure.show()
            plot_trace()

            del trace_results[0]
            del cls_results[0]
            max_cls = len(results)


def plot_trace():

    global traced_clusters,trace_3d_figure,ax3, basic_counter, save_folder

    for i in range(0, len(traced_clusters)):
	if len(traced_clusters[i]) == 0:
	    continue

	xar = np.array(traced_clusters[i][0][0])
	yar = np.array(traced_clusters[i][0][1])
	zar = np.array(traced_clusters[i][0][2])


	if len(traced_clusters[i]) > 1:
	    for j in range(1, len(traced_clusters[i])):
		xar = np.append(xar,traced_clusters[i][j][0])
		yar = np.append(yar,traced_clusters[i][j][1])
		B = traced_clusters[i][j][2].copy()

		B[::1]  += zar[len(zar) - 1]
		zar =  np.append(zar,B)

	if num_of_diagrams > 2:
		ax3.scatter(xar, yar, zar, 'z', 30, cc[i%12]) #human
        #trace_3d_figure.add_axes(ax3)
	
	#UNCOMMENT THE LINE BELOW TO SAVE A SCREENSHOT!
	#trace_3d_figure.savefig(save_folder+'tracedCl_'+str(basic_counter), format='png')
	plt.pause(0.0001)


def step_processing():

    global traced_clusters

    for i in range(0, len(traced_clusters)):
	if len(traced_clusters[i]) == 0:
	    continue

	xar = np.array(traced_clusters[i][0][0])
	yar = np.array(traced_clusters[i][0][1])
	zar = np.array(traced_clusters[i][0][2])


	if len(traced_clusters[i]) > 1:
	    for j in range(1, len(traced_clusters[i])):
		xar = np.append(xar,traced_clusters[i][j][0])
		yar = np.append(yar,traced_clusters[i][j][1])
		B = traced_clusters[i][j][2].copy()

		B[::1]  += zar[len(zar) - 1]
		zar =  np.append(zar,B)

    	steps2(xar, yar, zar)


def get_accuracy(ann, results):

    acc=0.0
    T=0.0
    F=0.0

    if len(ann) != len(results):
	print 'ERROR: different size in annotations and results.'
	return


    for i in range(0, len(ann)):
	if ann[i]==results[i]:
	    T=T+1
	else:
	    F=F+1

    acc=100*T/(T+F)


#gets the local minimums and maximums of the st.deviation matrix
#tries to compute the number of the steps with the use of this information
def compute_steps(dev) :

    global tot_steps

    min_array = []
    max_array = []
    m = round(sum(dev)/len(dev),2)

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
    #print 'Compute steps :  ',steps
    tot_steps.append(steps)



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


def steps3(x, y, z):

    global scan_parts, step_parts, trace_3d_figure, ax3

    num = 0
    split = len(x)/step_parts
    dev2 = []
    flag = False

    arr = np.array(x)
    newx = arr.argsort()[:len(x)]

    newy = []
    newz = []
    c=0

    for i in range(0, len(newx)):
	newy.append(y[newx[i]])
	newz.append(z[newx[i]])

    
    while num <= len(x) :
	if flag == True:
		break

	xn = newx[num:num+split]
	yn = newy[num:num+split]
	zn = newz[num:num+split]
	
	if len(xn) != 0 and len(yn)!=0 and len(zn)!=0 :
		xmean = np.median(np.array(xn))
		ymean = np.median(np.array(yn))
		zmean = np.median(np.array(zn))

	
		sumx = 0.0

		for m in range(0,len(xn)):
	    		sumx = sumx +  pow((yn[m] - ymean), 2) + pow((zn[m] - zmean), 2) #pow((xn[m] - xmean), 2) +

		dev2.append(round((math.sqrt(sumx/len(xn))),2))
	num = num + split

	c=c+1

	
	if len(x)-num < split :
	    	split = len(x)-num
		flag = True
	

    compute_steps(dev2)
    


# separate data in equal parts of points.
# compute the standard deviation by median and not the avg of points
def steps2(x, y, z):

    global scan_parts, step_parts, trace_3d_figure, ax3

    num = 0
    split = len(x)/step_parts
    dev2 = []
    flag = False
    c=0

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
	

    compute_steps(dev2)


# separate data in equal time parts.
# compute the standard deviation by median and not the avg of points
def steps(x, y, z):

    global z_scale, scan_parts,ax3,trace_3d_figure

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
	    
		arr = np.array([xk,yk,zk])
		
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

    compute_steps(dev2)




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
    
    global _3d_figure, ax, ax3, wall_cart, LDA_classifier, hogs_temp, align_plot, trace_3d_figure, top_view_figure
    global first_time_annotations, all_annotations
    global tot_results, metrics, first_time_results, num_of_diagrams

    temp = []
    store_results = []
    centerx = []
    centery = []
    centerz = []


    if flag==1:
	if num_of_diagrams > 0:
        	top_view_figure.clear()
		top_view_figure.set_title("Top view")
    		top_view_figure.set_xlabel('Vertical distance')
    		top_view_figure.set_ylabel('Robot is here')

        	top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])

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


            if results[cnt]==1:
		if num_of_diagrams > 0:
                	top_view_figure.scatter(x,y,s=20, c='r')
		if num_of_diagrams > 1:
			ax.scatter(x,y, zed, 'z', 30, cc[k+1%12]) #human
                #_3d_figure.add_axes(ax)

            else:
		if num_of_diagrams > 0:
                	top_view_figure.scatter(x,y,s=20, c='b')
		if num_of_diagrams > 1:
			ax.scatter(x,y, zed, 'z', 30, cc[k+1%12]) #obj
                #_3d_figure.add_axes(ax)

	    cnt=cnt+1

            plt.pause(0.0001)

	    if metrics:
	    	ha = raw_input()
            	if (int(ha)==1 or int(ha)==0):
                	ha = int(ha)
                	
	    
			if first_time_annotations:
                		all_annotations = np.array(ha)
				tot_results = np.array(results)
                		first_time_annotations = False
            		else:
                		all_annotations=np.hstack((all_annotations,np.array(ha)))
				
        #UNCOMMENT TO STORE INFO
	#store_info(results)



	pickle.dump(store_results, open('stored_predictions.p','a'))
	file_name=open('stored_predictions.txt','a')
	file_name.write(str(store_results))
	file_name.write("\n")
	file_name.close()

        if metrics:
            if first_time_results:
                tot_results = np.array(results)
                first_time_results = False
            else:
		tot_results=np.hstack((tot_results,np.array(results)))	

	hogs_temp = np.array(np.array(temp))
  
      
def store_info(results):

    global basic_counter, _3d_figure, save_folder, tot_steps, tot_speed
    
      
    _3d_figure.savefig(save_folder+'cluster_'+str(basic_counter), format='png')

    file_name=open('general_info.txt','a')
    file_name.write("id: "+str(basic_counter))
    file_name.write(" results: "+str(results))
    file_name.write(" steps: "+str(tot_steps))
    file_name.write(" speeds: "+str(tot_speed))
    file_name.write("\n")
    file_name.close()


    b={}
    b['id']=basic_counter
    b['results']=results
    b['steps']=tot_steps
    b['speeds']=tot_speed

    sio.savemat('general_info.mat',b);
    
    basic_counter = basic_counter +1
    del tot_steps[:]
    del tot_speed[:]


if __name__ == '__main__':
    laser_listener()   
