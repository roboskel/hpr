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
from myhog import hog
from sensor_msgs.msg import LaserScan
from scipy.stats.mstats import zscore
from scipy import interpolate
from sklearn.decomposition import PCA

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
sub_topic = 'scan'
metrics = 0
total_cluster_time = 0
hogs_temp=[]
flag_hogs=False
scan_pieces=5

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
        print b['wall']
        b['annotations']=annotations
        b['ranges']=ranges_
        try:
            os.remove('classification_results.mat')
        except OSError:
            pass
        sio.savemat('classification_results',b);

    print 'duration in milliseconds = {0}'.format(total_cluster_time)
    print "D O N E !"
    #Calculate_Metrics(annotated_data)
    #sys.exit()

def online_test(laser_data):
    global wall_flag, wall, fr_index, intens, w_index, phi, sampling, limit #prosthesa to limit edw giati den to epairne global
    global phi, mybuffer, z, zscale, gaussian,timewindow , wall_cart,ax,fig1,fig3, kat, center
    global pca_obj, pca_plot
    global ranges_, intensities, angle_increment, scan_time, angle_min, angle_max, first_time_ranges, total_cluster_time
    global mybuffer2, num_c


    millis_start = int(round(time.time() * 1000))
    if wall_flag == 0:
        #print "-------------- 1"
        if w_index == 1:
            #print "-------------- 2"
            sampling = np.arange(0,len(np.array(laser_data.ranges)),2)#apply sampling e.g every 2 steps
            
            #ADDITIONS allaksa na mhn epistrefei ta asxeta poy den xrhsimopoioyntai
            #ADDITIONS TO PARAKATW METAFERTHIKE PANW, GINETAI PLEON MIA FORA
            #gaussian, pca_obj = loadfiles()
            
            #wall data now contains the scan ranges
            wall = np.array(laser_data.ranges)
            #ADDITIONS
            
            mybuffer = wall
            #get indexes of scans >= range_limit
            filter=np.where(wall >= range_limit)
            #set thos scans to maximum range
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
            center,kat,ax=initialize_plots(wall_cart)

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
	    #print 'C = {}'.format(C)


            if (fr_index ==1 ):
                mybuffer = C #mybuffer is the cartesian coord of the first scan
		mybuffer2 = [C]
		num_c = np.array(len(C))
	
            else :
                mybuffer = np.concatenate((mybuffer,C), axis=0 )  #  add the next incoming scans to mybuffer until you have <timewindow>scans
		mybuffer2.append((mybuffer2,[C]))
		num_c=np.vstack((num_c,len(C)))

            if (fr_index == timewindow ):
		#print 'mybuffer[:,0] = {}'.format(mybuffer[:,0])
                mybuffer=mybuffer[np.where( mybuffer[:,0] > 0.2),:][0] #mishits safety margin
                mybuffer=mybuffer[np.where( mybuffer[:,0] < 5),:][0]#ignore distant points


                if len(mybuffer>3): #at least 3 points are needed to form a cluster
                    #clustering(mybuffer, num_c)
		    cluster_into_pieces(mybuffer, num_c)
 
                fr_index=0
                z=- z_scale
            z = z + z_scale
            fr_index=fr_index+1
    #classification_array = np.array(classification_array)
    millis_end = int(round(time.time() * 1000))
    total_cluster_time = total_cluster_time + millis_end - millis_start
    

def pol2cart(r,theta,zed):

    #metatropi kylindrikon syntentagmenon se kartesianes
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C

def loadfiles():
    
    #ADDITIONS
    global class_path
    global pca_path
    #ADDITIONS
    
    #ADDITIONS ta apokatw htan ayta ta axrista poy fortonontousan
    #Load intensity-range-angle data and classifier
    #mat=sio.loadmat('ideal_data.mat')
    #ranges=mat.get('ranges')
    #intensities=mat.get('intensities')
    #angle_min=mat.get('angle_min')
    #angle_max=mat.get('angle_max')
    #angle_increment=mat.get('angle_increment')
    #theta=np.arange(angle_min,angle_max,angle_increment)
    
    classifier = pickle.load( open( class_path, "rb" ) )
    pca_obj = pickle.load(open ( pca_path, "rb"))
    
    #print classifier_ilithiou.class_prior
    #return ranges,intensities,theta,classifier
    return classifier, pca_obj

def initialize_plots(wall_cart):
    global fig1,fig3

    fig3=plt.figure()
    plot2d1= fig3.gca(projection='3d')
    plot2d1.set_xlabel('X - Distance')
    plot2d1.set_ylabel('Y - Robot')
    plot2d1.set_zlabel('Z - time')

    temp=plt.figure()
    plot2d = temp.add_subplot(111)
    plot2d.set_xlabel('Vertical distance')
    plot2d.set_ylabel('Robot is here')
    plot2d.plot(wall_cart[:,0],wall_cart[:,1])

    fig1=plt.figure()
    plot3d= fig1.gca(projection='3d')
    plot3d.set_xlabel('X - Distance')
    plot3d.set_ylabel('Y - Robot')
    plot3d.set_zlabel('Z - time')

    plt.show()
    return plot2d1,plot2d,plot3d


def extract_features(array_pieces):

    first_time=True

    #print 'size = {} , array: {}'.format(len(array_pieces), array_pieces)
    #print 'size {} array = {}'.format(len(array_pieces), array_pieces)

    #for i in range(0,len(array_pieces)):
    [xi,yi]=[array_pieces[0][0], array_pieces[0][1]]
	#print 'xi = {}'.format([xi,yi])


    #if first_time:
    xmin=min(xi)
    xmax=max(xi)
    ymin=min(yi)
    ymax=max(yi)
	    
    xmean=np.mean(xi)
    num_xi=len(xi)
    num_yi=len(yi)
    ymean=np.mean(yi)

    sum_deviation = 0.0
    for i in range(0,len(xi)) :
	sum_deviation=sum_deviation+ math.pow(xi[i]-xmean,2)+ math.pow(yi[i]-ymean,2)

    standard_deviation=math.sqrt(sum_deviation/len(xi))
    '''
    else:
	if xmin>=min(xi):
	    xmin=min(xi)
	if xmax<=max(xi):
	    xmax=max(xi)
	if ymin>=min(yi):
	    ymin=min(yi)
	if ymax<=max(yi):
	    ymax=max(yi)
    

	xmean=xmean+np.mean(xi)
	num_xi=num_xi+len(xi)
	ymean=ymean+np.mean(yi)
	num_yi=num_yi+len(yi)
    '''

    

    #print 'xmin {} xmax {} ymin {} ymax {} xmean {} ymean {} numxi {} num yi {} standard_deviation = {}'.format(xmin,xmax,ymin,ymax, xmean/num_xi, ymean/num_yi, num_xi, num_yi, standard_deviation)


def cluster_into_pieces(clear_data, num_c):


    global cc, ccnames, fig1, z, z_scale, center,fig3, kat, wall_cart
    global scan_pieces
    
    #warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    centerx=[]
    centery=[]
    centerz=[]
    centerk=[]
    centerx_list=[]
    centery_list=[]
    centerz_list=[]
    centertot_list=[]
    array_pieces=[]
    point_slots=[]
    colors=[]
    flag_x=False
    flag_y=False
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at leat one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    #print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))


    #print 'cl = {}'.format(clear_data)
    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    
    xii = []
    yii=[]
    zii=[]
    clear_data1 =sorted(clear_data, key=lambda xs: xs[0])
    for i in range(0,len(clear_data1)) :
	xii.append(clear_data1[i][0])
	yii.append(clear_data1[i][1])
	zii.append(clear_data1[i][2])


    #tck =interpolate.BarycentricInterpolator(xi, yi)
    #t = interpolate.PiecewisePolynomial(xi,yi)
    #tck=interpolate.splrep(xi,yi)
    #print '!!!! tck = {}'.format(tck)
    #print '!!!! t = {}'.format(t)

    s = interpolate.splrep(xii, yii)
    print 's {}'.format(s)
    
    # calculate polynomial
    zp = np.polyfit(xi, yi, 3)
    f = np.poly1d(zp)

    # calculate new x's and y's
    x_new = np.linspace(xi[0], xi[-1], 50)
    y_new = f(x_new)
    #print 'xnew {} ynew {}'.format(x_new,y_new)
    plt.plot(xi,yi,'o',x_new,f(x_new))
    plt.show()
    

    #print '[xi,yi,zi] = {} \n'.format([xi,yi,zi])
    fig1.clear()
    fig3.clear()
    #print 'into clustering: [xi,yi,zi] = {}'.format([xi,yi,zi])
    
    #print '[xk,yk] {} '.format([xk,yk])
    
    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters
    prev=0
    '''
    for p in range(0,len(num_c)):
	pp=num_c[p]
	print '!!! pp = {} , [xp,yp,zp]={}'.format(pp,num_c)
    '''
    pp=0
    p_prev=0
    for p in range(0,len(num_c),scan_pieces):
	for t in range(p_prev,p):
	    pp=pp+num_c[t]

	p_prev=p
	[xp,yp,zp]=[clear_data[prev:prev+pp-1:1,0], clear_data[prev:prev+pp-1:1,1], clear_data[prev:prev+pp-1:1,2]]
	#print 'pp = {} p_prev = {}'.format(pp, p_prev)
	#print 'pp = {} , [xp,yp,zp]={}'.format(pp,[xp,yp,zp])
	#print 'clear_data[prev:p:1,0] {}'.format(clear_data[prev:p:1,0])
	
	cl_labels = cluster_labels[prev:prev+pp-1:1]
	#print 'cl_labels {} max {}'.format(cl_labels, int(np.amax(cl_labels)))

        max_cl = int(np.amax(cl_labels))
	for k in range(1,max_cl+1) :
	    filter=np.where(cl_labels==k)
	    #print 'filter {} '.format(filter)
	    [xk,yk,zk]=[xp[filter],yp[filter],zp[filter]]
	    #print '[xk,yk,zk]={}'.format([xk,yk,zk])
	    point_slots.append([xk,yk])
	    #print 'point_slots {} '.format(point_slots)
	    
	    if len(xk)!=0 and len(yk)!=0 and len(zk)!=0:
		
	    	if len(xk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(xk))
	    	if len(yk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(yk))
	    	if len(zk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(zk))

	    	#print 'centerk = {}'.format(centerk)
	    	centerz_list.append(centerk)
	    	centerk=[]

	    	#get features of a cluster piece, where its points are extracted at each 'scan_pieces' scans
	    	extract_features(point_slots)
	    	point_slots=[]
	pp=0
	prev=pp+prev
	centertot_list.append(centerz_list)
	centerz_list = []



def clustering2(clear_data, num_c):

    global cc, ccnames, fig1, z, z_scale, center,fig3
    
    #warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    centerx=[]
    centery=[]
    centerz=[]
    centerk=[]
    centerx_list=[]
    centery_list=[]
    centerz_list=[]
    centertot_list=[]
    array_pieces=[]
    point_slots=[]
    colors=[]
    flag_x=False
    flag_y=False
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at leat one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    #print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))



    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    #print '[xi,yi,zi] = {} \n'.format([xi,yi,zi])
    fig1.clear()
    fig3.clear()
    #print 'into clustering: [xi,yi,zi] = {}'.format([xi,yi,zi])
    
    #print '[xk,yk] {} '.format([xk,yk])
    
    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters
    prev=0
    '''
    for p in range(0,len(num_c)):
	pp=num_c[p]
	print '!!! pp = {} , [xp,yp,zp]={}'.format(pp,num_c)
    '''

    for p in range(0,len(num_c)):
	pp=num_c[p]
	[xp,yp,zp]=[clear_data[prev:prev+pp-1:1,0], clear_data[prev:prev+pp-1:1,1], clear_data[prev:prev+pp-1:1,2]]
	print 'pp = ',pp
	#print 'pp = {} , [xp,yp,zp]={}'.format(pp,[xp,yp,zp])
	#print 'clear_data[prev:p:1,0] {}'.format(clear_data[prev:p:1,0])
	
	cl_labels = cluster_labels[prev:prev+pp-1:1]
	#print 'cl_labels {} max {}'.format(cl_labels, int(np.amax(cl_labels)))

        max_cl = int(np.amax(cl_labels))
	for k in range(1,max_cl+1) :
	    filter=np.where(cl_labels==k)
	    #print 'filter {} '.format(filter)
	    [xk,yk,zk]=[xp[filter],yp[filter],zp[filter]]
	    print '[xk,yk,zk]={}'.format([xk,yk,zk])
	    point_slots.append([xk,yk])
	    #print 'point_slots {} '.format(point_slots)
	    
	    if len(xk)==0 & len(yk==0) & len(zk)==0:
		continue
	    
	    else:
		if len(xk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(xk))
	    	if len(yk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(yk))
	    	if len(zk)==0:
		    centerk.append(0)
	    	else:
	    	    centerk.append(np.mean(zk))
	    #print 'centerk = {}'.format(centerk)
	    centerz_list.append(centerk)
	    centerk=[]
	
	extract_features(point_slots)
	
	prev=pp+prev

	centertot_list.append(centerz_list)
	centerz_list = []

    
    #print 'centertot_list = {}'.format(centertot_list)
    
    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
	    #print 'xi[filter] {} \n yi[filter] {}\n zi[filter] {}'.format(xi[filter],yi[filter], zi[filter])
	   
            #ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster '

	    #points of every cluster at each timewindow
	    [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]
	    #print '[xk,yk,zk] {}'.format([xk,yk,zk])
	    '''
	    for j in np.arange(0,z,z_scale):
		flag_x=False
		flag_y=False
		
	    	zfilter=np.where(zk==j)

		try:
		    #get (x,y) points at each timeslot
    	    	    [xj,yj]=[xk[zfilter],yk[zfilter]]

	    	    #print '[xj,yj] {}'.format([xj,yj])
		    #get the centroid
		    if len(xj)!=0:
		    	centerx.append(np.mean(xj))
			#print 'put the xj = {}'.format(np.mean(xj))
			flag_x=True
		    if len(yj)!=0:
		    	centery.append(np.mean(yj))
			#print 'put the yj = {}'.format(np.mean(yj))
			flag_y=True
		    if flag_x==True & flag_y==True:
			centerz.append(j)
			#print 'put the j = {}'.format(j)
			array_pieces.append([xj,yj])
		except IndexError:
		    break

	    extract_features(array_pieces)
	    #centerx_list contains the lists of centroids of each cluster at each timeslot
	    centerx_list.append(centerx)
	    centery_list.append(centery)
	    centerz_list.append(centerz)
	    centerx = []
	    centery=[]
	    centerz=[]
	    '''
	    
            vcl.append(k)
            colors.append(ccnames[k%12])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features

    fig1.show()
    fig3.show()
    #print 'centerx = {} , centery={}'.format(centerx_list,centery_list)

    #update_plots2(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl,centertot_list)
  

def clustering(clear_data, num_c):

    global cc, ccnames, fig1, z, z_scale, center,fig3
    
    #warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    centerx=[]
    centery=[]
    centerz=[]
    centerx_list=[]
    centery_list=[]
    centerz_list=[]
    colors=[]
    flag_x=False
    flag_y=False
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at leat one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    #print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))

    #print 'clear data = {}'.format(clear_data)
    ss=0
    for s in range(0,len(num_c)):
	ss=ss+num_c[s]
    print 'num point {} num_c {}'.format(len(clear_data),ss)
    print 'cluster_labels={} max_label={}'.format(cluster_labels,max_label)

    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    #print '[xi,yi,zi] = {} \n'.format([xi,yi,zi])
    fig1.clear()
    fig3.clear()
    #print 'into clustering: [xi,yi,zi] = {}'.format([xi,yi,zi])
    
    #print '[xk,yk] {} '.format([xk,yk])
    
    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters
    prev=0
    for p in range(0,len(num_c)):
	pp=num_c[p]
	[xp,yp,zp]=[clear_data[prev:prev+pp:1,0], clear_data[prev:prev+pp:1,1], clear_data[prev:prev+pp:1,2]]
	#print 'pp = {} , [xp,yp,zp]={}'.format(pp,[xp,yp,zp])
	#print 'clear_data[prev:p:1,0] {}'.format(clear_data[prev:p:1,0])
	cl_labels = cluster_labels[prev:prev+pp:1]
	print 'cl_labels {} max {}'.format(cl_labels, int(np.amax(cl_labels)))
	for k in range(1,int(np.amax(cl_labels))) :
	    filter=np.where(cl_labels==k)
	    [xk,yk,zk]=[xp[filter],yp[filter],zp[filter]]
	    print '[xk,yk,zk]={}'.format([xk,yk,zk])
	
	prev=pp



    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
	    #print 'xi[filter] {} \n yi[filter] {}\n zi[filter] {}'.format(xi[filter],yi[filter], zi[filter])
	   
            #ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster '

	    #points of every cluster at each timewindow
	    [xk,yk,zk]=[xi[filter],yi[filter],zi[filter]]
	    #print '[xk,yk,zk] {}'.format([xk,yk,zk])
	    for j in np.arange(0,z,z_scale):
		flag_x=False
		flag_y=False
		
	    	zfilter=np.where(zk==j)

		try:
		    #get (x,y) points at each timeslot
    	    	    [xj,yj]=[xk[zfilter],yk[zfilter]]

	    	    #print '[xj,yj] {}'.format([xj,yj])
		    #get the centroid
		    if len(xj)!=0:
		    	centerx.append(np.mean(xj))
			#print 'put the xj = {}'.format(np.mean(xj))
			flag_x=True
		    if len(yj)!=0:
		    	centery.append(np.mean(yj))
			#print 'put the yj = {}'.format(np.mean(yj))
			flag_y=True
		    if flag_x==True & flag_y==True:
			centerz.append(j)
			#print 'put the j = {}'.format(j)

		except IndexError:
		    break

	    print 'centerx = {} , centery={} , centerz={}'.format(centerx,centery,centerz)
	    #centerx_list contains the lists of centroids of each cluster at each timeslot
	    centerx_list.append(centerx)
	    centery_list.append(centery)
	    centerz_list.append(centerz)
	    centerx = []
	    centery=[]
	    centerz=[]
	    
            vcl.append(k)
            colors.append(ccnames[k%12])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features

    fig1.show()
    fig3.show()
    #print 'centerx = {} , centery={}'.format(centerx_list,centery_list)

    update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl,centerx_list,centery_list,centerz_list)
  




def scatter_all(xi,yi,zi,cluster_labels):
    
    global cc
    fig2 = plt.figure()
    ax2 = fig2.gca(projection='3d')
    ax2.set_xlabel('X - Distance')
    ax2.set_ylabel('Y - Robot')
    ax2.set_zlabel('Z - time')

    max_label=int(np.amax(cluster_labels))

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
            ax2.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k%12])

    fig2.pause(0.00001)

def euclidean_distance(v1, v2, flag):

    list_dist = []
    min_dist = -1.0
    index=0

    
    if isinstance(v1, list) & flag==False:
        for i in range(0,len(v1)):
	    for j in range(0,len(v2)):
	        distance = dist.euclidean(np.array(v1)[i],np.array(v2)[j])

	        if min_dist == -1.0:
		    min_dist=distance
		    index=j
	        elif distance <= min_dist:
		    min_dist=distance
		    index=j

	    list_dist.append(index)
	    index=0
	    min_dist=-1.0
    if not isinstance(v1, list):
	for j in range(0,len(v2)):
	    distance = dist.euclidean(v1,np.array(v2)[j])

	    if min_dist == -1.0:
		min_dist=distance
		index=j
	    elif distance <= min_dist:
		min_dist=distance
		index=j

	list_dist.append(index)
    if flag==True:
	for j in range(0,len(v1)):
	    distance = dist.euclidean(np.array(v1)[j],v2)

	    if min_dist == -1.0:
		min_dist=distance
		index=j
	    elif distance <= min_dist:
		min_dist=distance
		index=j

	list_dist.append(index)

    return list_dist


def update_plots(flag,hogs,xi,yi,zi,cluster_labels,vcl,centerx_list,centery_list,centerz_list):
    
    global kat, fig1, ax, wall_cart, gaussian, classification_array, pca_obj, hogs_temp, pca_plot, center, fig3
    global annotations, first_time, flag_hogs

    temp = []
    store_results = []
    #temp2 = np.empty(36)           #Currently removed this way of calculating the zscore with temp2 because an update of python made it unusable
    
    #ZSCORE UPDATE
    #zscore the entire hogs table, not single cluster hogs
    if flag==1:
        kat.clear()
        kat.plot(wall_cart[:,0],wall_cart[:,1])

	#center.clear()
    	#center.plot(wall_cart[:,0],wall_cart[:,1])
        print 'centerx_list = {} , centery_list={} , centerz_list={}'.format(centerx_list,centery_list,centerz_list)

        if np.array(hogs).shape==(1,36):
            #BEFORE
            temp = zscore(np.array(hogs)[0])
            #AFTER
            #temp2 = np.array(hogs)[0]
        else:
            #BEFORE
            for i in range(0,len(hogs)):
                temp.append(zscore(np.array(hogs[i])))
            #AFTER
            #temp2 = np.array(hogs)
            #print temp2.shape
        
        #AFTER, zscore the array of size <# of clusters> x <#number of features>

        #temp2_zscore = zscore(temp2)
        #temp2_zscore = temp2_zscore[np.logical_not(np.isnan(temp2_zscore))]    #remove NaNs from the matrix
        #temp2_zscore = pca_obj.transform(temp2_zscore)
        
        

        #temp2_zscore = zscore(temp)
        #temp2_zscore = pca_obj.transform(temp2_zscore)
        
        #results = gaussian.predict(temp2_zscore)
	#print 'TEMP {}'.format(temp)
	list_dist = []
	if len(hogs_temp) != 0:
	    if not isinstance(temp, list):
		if flag_hogs==True:
		    list_dist.append(0)
		else:
	            list_dist=euclidean_distance(temp,hogs_temp, flag_hogs)
	    else:
	        list_dist=euclidean_distance(temp,hogs_temp, flag_hogs)

	if len(list_dist)==0:
	    list_dist.append(0)	

	temp_pca = pca_obj.transform(temp)
        results = gaussian.predict(temp_pca)
        print results
	print list_dist
	#print 'temp = {}'.format(temp)
	#print 'pca = {} temp_pca[0,:] {}'.format(temp_pca, temp_pca[0,:])

        cnt=0
	list_len=0
	col=0
	col_list=[]

	

        for k in vcl:

	    if list_len>=len(list_dist):
		col=list_len
	    else:
		col=list_dist[list_len]

            filter=np.where(cluster_labels==k)
            
            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            if results[cnt]==1:
                #classification_array.append(1)
                kat.scatter(x,y,s=20, c='r')
                ax.scatter(x,y, zed, 'z', 30, cc[col%12]) #human
                fig1.add_axes(ax)
            else:
                #classification_array.append(0)
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, cc[col%12]) #object
                fig1.add_axes(ax)

	    center.scatter(centerx_list[cnt],centery_list[cnt],centerz_list[cnt], 'z', 30, c=cc[k%12])
	    fig3.add_axes(center)

	    #fig1.add_axes(pca_plot)
	    #add to a struct the classification prediction and the point cloud of the respective cluster
	    
	    store_results.append([])
	    store_results[cnt].append(results[cnt])
	    store_results[cnt].append(np.array([x,y,zed]))

            cnt=cnt+1
	    list_len=list_len+1
        plt.pause(0.0001)

	pickle.dump(store_results, open('stored_predictions.p','a'))
	file_name=open('stored_predictions.txt','a')
	file_name.write(str(store_results))
	file_name.write("\n")
	file_name.close()

        if metrics == 1:
            if first_time:
                annotations = np.array(results)
                first_time = False
            else:
                annotations=np.hstack((annotations,np.array(results)))

	if isinstance(temp, list):
	    flag_hogs=False
	else:
	    flag_hogs=True

	hogs_temp = np.array(np.array(temp))
        
	#b={}
        #b['annotations']=classification_array
        #sio.savemat('classification_results',b);


def update_plots2(flag,hogs,xi,yi,zi,cluster_labels,vcl,centertot_list):
    
    global kat, fig1, ax, wall_cart, gaussian, classification_array, pca_obj, hogs_temp, pca_plot, center, fig3
    global annotations, first_time, flag_hogs

    temp = []
    store_results = []
    centerx = []
    centery = []
    centerz = []
    #temp2 = np.empty(36)           #Currently removed this way of calculating the zscore with temp2 because an update of python made it unusable
    
    #ZSCORE UPDATE
    #zscore the entire hogs table, not single cluster hogs
    if flag==1:
        kat.clear()
        kat.plot(wall_cart[:,0],wall_cart[:,1])

	#center.clear()
    	#center.plot(wall_cart[:,0],wall_cart[:,1])
        
        if np.array(hogs).shape==(1,36):
            #BEFORE
            temp = zscore(np.array(hogs)[0])
            #AFTER
            #temp2 = np.array(hogs)[0]
        else:
            #BEFORE
            for i in range(0,len(hogs)):
                temp.append(zscore(np.array(hogs[i])))
            #AFTER
            #temp2 = np.array(hogs)
            #print temp2.shape
        
        #AFTER, zscore the array of size <# of clusters> x <#number of features>

        #temp2_zscore = zscore(temp2)
        #temp2_zscore = temp2_zscore[np.logical_not(np.isnan(temp2_zscore))]    #remove NaNs from the matrix
        #temp2_zscore = pca_obj.transform(temp2_zscore)
        
        

        #temp2_zscore = zscore(temp)
        #temp2_zscore = pca_obj.transform(temp2_zscore)
        
        #results = gaussian.predict(temp2_zscore)
	#print 'TEMP {}'.format(temp)
	list_dist = []
	if len(hogs_temp) != 0:
	    if not isinstance(temp, list):
		if flag_hogs==True:
		    list_dist.append(0)
		else:
	            list_dist=euclidean_distance(temp,hogs_temp, flag_hogs)
	    else:
	        list_dist=euclidean_distance(temp,hogs_temp, flag_hogs)

	if len(list_dist)==0:
	    list_dist.append(0)	

	temp_pca = pca_obj.transform(temp)
        results = gaussian.predict(temp_pca)
        print results
	print list_dist
	#print 'temp = {}'.format(temp)
	#print 'pca = {} temp_pca[0,:] {}'.format(temp_pca, temp_pca[0,:])

        cnt=0
	list_len=0
	col=0
	col_list=[]

	

        for k in vcl:

	    if list_len>=len(list_dist):
		col=list_len
	    else:
		col=list_dist[list_len]

            filter=np.where(cluster_labels==k)
            
            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            if results[cnt]==1:
                #classification_array.append(1)
                kat.scatter(x,y,s=20, c='r')
                ax.scatter(x,y, zed, 'z', 30, cc[k%12]) #human
                fig1.add_axes(ax)
            else:
                #classification_array.append(0)
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, cc[k%12]) #object
                fig1.add_axes(ax)
	    
	    for t in range(0,len(centertot_list)) :
		try:
		    centerx.append(centertot_list[t][cnt][0])
		    centery.append(centertot_list[t][cnt][1])
		    centerz.append(centertot_list[t][cnt][2])
		except IndexError:
		    break;
		
	    print 'centerx {} , centery{} , centerz {}'.format(centerx, centery, centerz)
	    center.scatter(centerx,centery,centerz, 'z', 30, c=cc[k%12])
	    fig3.add_axes(center)

	    #fig1.add_axes(pca_plot)
	    #add to a struct the classification prediction and the point cloud of the respective cluster
	    '''
	    store_results.append([])
	    store_results[cnt].append(results[cnt])
	    store_results[cnt].append(np.array([x,y,zed]))
	    '''
            cnt=cnt+1
	    list_len=list_len+1
            centerx = []
            centery = []
            centerz = []

        plt.pause(0.20)

	pickle.dump(store_results, open('stored_predictions.p','a'))
	file_name=open('stored_predictions.txt','a')
	file_name.write(str(store_results))
	file_name.write("\n")
	file_name.close()

        if metrics == 1:
            if first_time:
                annotations = np.array(results)
                first_time = False
            else:
                annotations=np.hstack((annotations,np.array(results)))

	if isinstance(temp, list):
	    flag_hogs=False
	else:
	    flag_hogs=True

	hogs_temp = np.array(np.array(temp))
        
	#b={}
        #b['annotations']=classification_array
        #sio.savemat('classification_results',b);


if __name__ == '__main__':
    laser_listener()
