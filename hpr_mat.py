#!/usr/bin/env python
__author__="athanasia sapountzi"

import  warnings, math, pickle, scipy.stats
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
from scipy.stats.mstats import zscore
from sklearn.decomposition import PCA
from sklearn.naive_bayes import GaussianNB

ccnames =['gray', 'black', 'violet', 'blue', 'cyan', 'rosy', 'orange', 'red', 'green', 'brown', 'yellow', 'gold']
cc  =  ['#808080',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','r','g','#8B4513','y','#FFD700']
wall_flag = 0
fr_index = 0
z = 0
dt = 25;#period in ms (dt between scans)
speed = 5;#human walking speed in km/h
z_scale= float(speed*dt) / float(3600)
w_index = 1
limit = 40
slot_count = 0

metrics = 0
first_time = True
first_time_ranges = True
data_path = ''
class_path = ''
pca_path = ''
classification_array = []

gau_classifier = GaussianNB()
pca_obj = PCA()
plt.ion()

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

def offline_test():

    global wall_flag, wall , fr_index ,  intens, w_index, phi, sampling
    global phi, z, zscale, gau_classifier, timewindow , wall_cart, ax, fig1, kat
    global data_path, class_path, pca_path, pca_obj
    global slot_count, limit
    global ranges_, intensities, angle_increment, scan_time, angle_min, angle_max, first_time_ranges
    
    if not (len(sys.argv)==7):
        print "###################################"
        print "You gave {0} arguments".format(len(sys.argv))
        print "Run using"
        print ">python hpr_mat.py <.mat_file_path> <classifier_path> <pca_object_path> <timewindow> <frames for walls> <0_or_1_for_metrics>"
        print "###################################"
        exit()
    #INPUT MANAGEMENT    
    else:
        data_path = sys.argv[1]
        if not os.path.isfile(data_path):
            while True :
                try:
                    data_path=raw_input('Enter .mat file path: ')
                    if os.path.isfile(data_path):
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        
        class_path = sys.argv[2]
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
        
        pca_path = sys.argv[3]
        if not os.path.isfile(pca_path):
            while True :
                try:
                    pca_path=raw_input('Enter pca object object file path: ')
                    if os.path.isfile(pca_path):
                        break
                    else:
                        print 'File does not exist! Try again!'
                except SyntaxError:
                    print 'Try again'
        
        timewindow = sys.argv[4]
        while not RepresentsInt(timewindow):
            timewindow=input('Set timewindow in frames: ')
            if RepresentsInt(timewindow):
                break
            else:
                print 'Try again'
                
        wall_end = float(sys.argv[5])
        while not (RepresentsInt(wall_end) or RepresentsFloat(wall_end)):
            wall_end=input('Set frames for wall: ')
            if RepresentsInt(wall_end):
                break
            else:
                print 'Try again'
                
        metrics = int(sys.argv[6])
        while not (RepresentsInt(metrics) and metrics != '1' and metrics != '0'):
            metrics=input('Set maximum scan range in m: ')
            if RepresentsInt(metrics):
                break
            else:
                print 'Try again'
    #INPUT MANAGEMENT
    
    print "Data File : {0}".format(data_path)
    print "PCA File : {0}".format(pca_path)
    print "Classifier File : {0}".format(class_path)
    print "Timewindow : {0}".format(timewindow)
    print "Wall Frames : {0}".format(wall_end)
    
    #LOAD FILE TO BE CLASSIFIED
    test_mat = sio.loadmat(data_path)
    all_data = test_mat.get('ranges')
    angle_min = test_mat.get('angle_min')
    angle_max = test_mat.get('angle_max')
    angle_increment = test_mat.get('angle_increment')
    intensities = test_mat.get('intensities')
    mybuffer = all_data[0]
    range_limit = np.amax(all_data)
    
    sampling = np.arange(0,len(mybuffer),2)#apply sampling e.g every 2 steps
    max_index = len(all_data)
    print "Max Index : {0}".format(max_index)
    key_press = raw_input("Press key to continue ")
    em_index = 0
    
    limit = int((max_index-wall_end-(3*int(timewindow)))/int(timewindow))
    
    #LOAD CLASSIFIER and PCA
    gau_classifier = pickle.load(open( class_path, "rb" ))
    pca_obj = pickle.load(open (pca_path, "rb"))

    phi = np.arange(angle_min,angle_max,angle_increment)[sampling]
    wall = all_data[0]
    
    for wall_index in range(1,int(wall_end)):

        print 'Wall_index : {0}'.format(wall_index)
        wall = all_data[fr_index]
        filter = np.where(wall>=range_limit)
        wall[filter] = range_limit

        if (wall_index<wall_end):
            mybuffer = np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 720)

    mybuffer = mybuffer[:,sampling]

    wall = np.min(mybuffer, axis=0)-0.1 #select min of measurements
    print "Wall Index : {0}".format(wall_index)
    print "Wall : {0}".format(wall)
    wall_cart=np.array(pol2cart(wall,phi,0) )[:,0:2] #convert to Cartesian
    kat,ax=initialize_plots(wall_cart)

    print 'Walls set...'

    for outer_index in range(wall_index,max_index):
        
        print 'Outer_index : {0}'.format(outer_index)
        ranges = all_data[outer_index]#[ind]
        ranges = ranges[sampling]
        filter = np.where(ranges <= wall) # remove walls
        ranges = ranges[filter]
        theta = phi[filter]
        if metrics == 1:
            if first_time_ranges:
                ranges_= np.array(all_data[outer_index])[sampling]
                first_time_ranges = False
            else:
                ranges_ = np.vstack((ranges_, np.array(all_data[outer_index])[sampling]))

        print "Ranges : {0}".format(len(ranges))
        
        if (len(ranges)<3 ):
            print "Empty Scan"
            em_index = em_index+1
            
        if (len(ranges)>=3): #each scan should consist of at least 3 points to be valid
            print "Valid Scan"
            print "Frame Index"
            
            z = z + z_scale
            fr_index=fr_index+1
            C=np.array(pol2cart(ranges,theta,z) ) #convert to Cartesian
            
            if ((int(fr_index) % int(timewindow)) == 1 or fr_index==1):
                mybuffer = C
                
            if (fr_index>1) :
                mybuffer=np.concatenate((mybuffer,C), axis=0 )
            
            if ((int(fr_index) % int(timewindow)) == 0):

                mybuffer = mybuffer[np.where( mybuffer[:,0] > 0.2),:][0] #mishits safety margin
                mybuffer = mybuffer[np.where( mybuffer[:,0] < 5),:][0]#ignore distant points

                if len(mybuffer>3): #at least 3 points are needed to form a cluster
                    print "Clustering"
                    clustering(mybuffer)

    if metrics == 1:
        b={}
        b['timewindow']=int(timewindow)
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
    
def pol2cart(r,theta,zed):

    #metatropi kylindrikon syntentagmenon se kartesianes
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C


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

    global cc, ccnames, fig1, slot_count
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    colors=[]
    vcl=[] #Valid Cluster Labels
    valid_flag=0 #this flag is only set if we have at least one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    max_label=int(np.amax(cluster_labels))
    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    fig1.clear()

    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
            #ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k%12]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster'
            vcl.append(k)
            colors.append(ccnames[k%12])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features
    
    print"Found {0} clusters".format(len(hogs))
    
    if len(hogs)!=0:
        print'len(hogs)!=0'
        slot_count=slot_count+1
        #print 'File : {0}'.format(filename)
        print 'slot count : {0} || limit :{1}'.format(slot_count, limit)
        ha=np.array(slot_count*np.ones(len(clear_data))) #data point -> slot_number
        update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl)
        fig1.show()
        if slot_count>=limit-1 :
            #build_classifier(np.array(all_hogs),np.array(annotations))
            #save_data()
            exit()
    print "Slot Count : {0}".format(slot_count)
    print "Limit : {0}".format(limit)                    
    
    
    return valid_flag,hogs,xi,yi,zi,cluster_labels,vcl

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

    global kat, fig1, ax, wall_cart, gau_classifier, pca_obj
    global annotations, first_time
    
    temp=[]
    #temp2=np.empty(36)             #Currently removed this way of calculating the zscore with temp2 because an update of python made it unusable
    
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
        
        temp_pca = pca_obj.transform(temp)
        results = gau_classifier.predict(temp_pca)

        #temp2_zscore = zscore(temp)
        #temp2_zscore = pca_obj.transform(temp2_zscore)
        
        #results = gau_classifier.predict(temp2_zscore)
        print results
        cnt=0
        for k in vcl:

            filter=np.where(cluster_labels==k)

            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            if results[cnt]==1:
                kat.scatter(x,y,s=20, c='r')
                ax.scatter(x,y, zed, 'z', 30, c='r') #human
                fig1.add_axes(ax)
                
            else:
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, c='b') #object
                fig1.add_axes(ax)
            cnt=cnt+1
        plt.pause(0.0001)
        key_press = raw_input("Press a key to continue")
        if metrics == 1:
            if first_time:
                annotations = np.array(results)
                first_time = False
            else:
                annotations=np.hstack((annotations,np.array(results)))


if __name__ == '__main__':
    offline_test()