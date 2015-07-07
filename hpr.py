#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib, warnings, rospy, math, pickle, scipy.stats
from gridfit import gridfit
import numpy as np
import scipy.io as sio
import scipy.special
import matplotlib.pyplot as plt
import mytools as mt #DBSCAN function and perquisites are stored here
import sys
import os.path
from os import listdir
from os.path import isfile, join, splitext
from myhog import hog
from sensor_msgs.msg import LaserScan
from scipy.stats.mstats import zscore

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
        while not (RepresentsInt(metrics)):
            metrics=input('Set maximum scan range in m: ')
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
    print "D O N E !"
    #Calculate_Metrics(annotated_data)
    #sys.exit()

def online_test(laser_data):
    global wall_flag, wall, fr_index, intens, w_index, phi, sampling, limit #prosthesa to limit edw giati den to epairne global
    global phi, mybuffer, z, zscale, gaussian,timewindow , wall_cart,ax,fig1, kat
    global pca_obj
    global ranges_, intensities, angle_increment, scan_time, angle_min, angle_max, first_time_ranges
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
            kat,ax=initialize_plots(wall_cart)
            #kat=initialize_plots(wall_cart)
            angle_increment=laser_data.angle_increment
            scan_time=laser_data.scan_time
            angle_min=laser_data.angle_min
            angle_max=laser_data.angle_max
            intensities=laser_data.intensities
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
            else :
                mybuffer = np.concatenate((mybuffer,C), axis=0 )  #  add the next incoming scans to mybuffer until you have <timewindow>scans

            if (fr_index == timewindow ):

                mybuffer=mybuffer[np.where( mybuffer[:,0] > 0.2),:][0] #mishits safety margin
                mybuffer=mybuffer[np.where( mybuffer[:,0] < 5),:][0]#ignore distant points

                if len(mybuffer>3): #at least 3 points are needed to form a cluster
                    clustering(mybuffer)
                fr_index=0
                z=- z_scale
            z = z + z_scale
            fr_index=fr_index+1
    #classification_array = np.array(classification_array)
    

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
    global fig1

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
    return plot2d,plot3d

def clustering(clear_data):

    global cc, ccnames, fig1
    #warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    colors=[]
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at leat one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    #print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))

    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    fig1.clear()
    
    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
            #ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster '
            vcl.append(k)
            colors.append(ccnames[k%12])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features

    fig1.show()
    
    update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl)
    
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

def update_plots(flag,hogs,xi,yi,zi,cluster_labels,vcl):
    
    global kat, fig1, ax, wall_cart, gaussian, classification_array, pca_obj
    global annotations, first_time
    temp = []
    #temp2 = np.empty(36)           #Currently removed this way of calculating the zscore with temp2 because after an update on python it stopped working
    
    #ZSCORE UPDATE
    #zscore the entire hogs table, not single cluster hogs
    if flag==1:
        kat.clear()
        kat.plot(wall_cart[:,0],wall_cart[:,1])
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
        
        #results = gaussian.predict(temp2_zscore)
        results = gaussian.predict(temp)
        print results
        cnt=0
        for k in vcl:

            filter=np.where(cluster_labels==k)
            
            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            if results[cnt]==1:
                #classification_array.append(1)
                kat.scatter(x,y,s=20, c='r')
                ax.scatter(x,y, zed, 'z', 30, c='r') #human
                fig1.add_axes(ax)
            else:
                #classification_array.append(0)
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, c='b') #object
                fig1.add_axes(ax)
            cnt=cnt+1
        plt.pause(0.0001)
        if metrics == 1:
            if first_time:
                annotations = np.array(results)
                first_time = False
            else:
                annotations=np.hstack((annotations,np.array(results)))
        #b={}
        #b['annotations']=classification_array
        #sio.savemat('classification_results',b);



if __name__ == '__main__':
    laser_listener()
